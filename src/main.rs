use std::io::{Read, Write};
use std::thread;
use std::time;

use clap::Parser;
use serialport::{SerialPort, TTYPort};

use ssp::{Error, FirmwareData, FirmwareHeader, FirmwareRam, MessageOps, ResponseOps, Result};
use ssp::{FIRMWARE_ACK, FIRMWARE_DATA_SECTION_LEN, FIRMWARE_HEADER_LEN};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial path for connecting to the device
    #[arg(short='s', long, default_value_t = String::from("/dev/ttyUSB0"))]
    serial: String,

    /// Location of the firmware file to flash to the device
    #[arg(short = 'f', long)]
    file: String,
}

fn connect(serial_path: &str, baud_rate: u32) -> Result<TTYPort> {
    serialport::new(serial_path, baud_rate)
        // disable flow control serial lines
        .flow_control(serialport::FlowControl::None)
        // seven-bit data size
        .data_bits(serialport::DataBits::Eight)
        // even control bit parity
        .parity(serialport::Parity::Even)
        // one bit stop
        .stop_bits(serialport::StopBits::One)
        // serial device times out after one second, so do we
        .timeout(time::Duration::from_secs(1))
        // get back a TTY port for POSIX systems, Windows is not supported
        .open_native()
        .map_err(|e| Error::Io(format!("Error opening serial port connection: {e}")))
}

fn send_sync(port: &mut TTYPort) -> Result<()> {
    let mut sync = ssp::SyncCommand::new();
    port.write_all(sync.as_bytes())?;

    let mut sync_res = ssp::SyncResponse::new();
    port.read_exact(sync_res.buf_mut())?;
    sync_res.verify_checksum()?;

    let status = sync_res.response_status();
    if status != ssp::ResponseStatus::Ok {
        let expected = ssp::ResponseStatus::Ok;
        Err(Error::Firmware(format!(
            "error sending initial Sync command, have: {status}, expected: {expected}"
        )))
    } else {
        Ok(())
    }
}

fn send_program_firmware_command(port: &mut TTYPort) -> Result<u16> {
    let mut program_cmd = ssp::ProgramFirmwareCommand::new();
    port.write_all(program_cmd.as_bytes())?;

    let mut program_res = ssp::ProgramFirmwareResponse::new();
    port.read_exact(program_res.buf_mut())?;
    program_res.verify_checksum()?;

    let block_len = program_res.block_len();
    log::info!("Received expected block length: {block_len}");

    Ok(block_len)
}

// Send the firmware header to the unit.
fn send_header(port: &mut TTYPort, header: &FirmwareHeader) -> Result<()> {
    log::debug!("Sending firmware header...");

    let mut header_cmd = ssp::FirmwareHeaderCommand::create(header)?;
    port.write_all(header_cmd.as_bytes())?;

    let mut header_res = ssp::FirmwareHeaderResponse::new();
    port.read_exact(header_res.buf_mut())?;

    let status = header_res.response_status();
    if status != ssp::ResponseStatus::Ok {
        let expected = ssp::ResponseStatus::Ok;
        Err(Error::Firmware(format!(
            "error sending firmware header, have: {status}, expected: {expected}"
        )))
    } else {
        log::debug!("Successfully sent firmware header");
        Ok(())
    }
}

// Send the firmware RAM code to the unit.
fn send_ram(port: &mut TTYPort, mut ram: FirmwareRam) -> Result<()> {
    log::debug!("Sending firmware RAM...");

    let mut checksum = 0u8;

    while let Some(section) = ram.next_section() {
        section.iter().for_each(|&b| checksum ^= b);
        port.write_all(section)?;
    }

    let mut res_checksum = [0u8];
    port.read_exact(res_checksum.as_mut())?;
    port.flush()?;

    validate_checksum(checksum, res_checksum[0])?;

    log::debug!("Successfully sent firmware RAM");

    Ok(())
}

// Check the response from a raw transmission is ACKed by the unit.
fn check_ack(res: u8) -> Result<()> {
    if res == FIRMWARE_ACK {
        Ok(())
    } else {
        Err(Error::Firmware(format!(
            "error sending firmware file code, expected ACK, have: {res}, expected: {FIRMWARE_ACK}"
        )))
    }
}

// Validate the response checksum matches the one calculated locally.
fn validate_checksum(checksum: u8, res_checksum: u8) -> Result<()> {
    if checksum == res_checksum {
        Ok(())
    } else {
        Err(Error::Firmware(format!("error sending firmware RAM block, invalid checksum, have: {res_checksum}, expected: {checksum}")))
    }
}

// Send the firmware dataset to the unit.
fn send_dataset(port: &mut TTYPort, mut data: FirmwareData, header: &FirmwareHeader) -> Result<()> {
    log::debug!("Sending firmware dataset...");

    // Send the firmware file code to the unit
    let code = header.file_code();
    port.write_all(&[code])?;

    // Check for an ACK response
    let mut res_buf = [0u8];
    port.read_exact(res_buf.as_mut())?;
    port.flush()?;
    check_ack(res_buf[0])?;

    // Send the header block as raw bytes
    let header_block: [u8; FIRMWARE_HEADER_LEN] = header.try_into()?;
    port.write_all(header_block.as_ref())?;
    port.read_exact(res_buf.as_mut())?;
    check_ack(res_buf[0])?;

    let data_len = data.len();
    let block_len = data.block_len();
    let blocks = data_len / block_len;
    let block_sections = block_len / FIRMWARE_DATA_SECTION_LEN;
    let sections = (data_len / FIRMWARE_DATA_SECTION_LEN)
        + ((data_len % FIRMWARE_DATA_SECTION_LEN != 0) as usize);
    let block_rem = data_len % block_len;

    // Send the dataset in "blocks" that are further subdivided into section lengths (128 bytes)
    for block in 0..blocks {
        let mut checksum = 0u8;

        for block_section in 0..block_sections {
            let section_num = (block * block_sections) + block_section;

            // Write the next 128 byte section
            let section = data.next_section().ok_or(Error::Firmware(format!(
                "invalid firmware data sections, have: {section_num}, expected: {sections}"
            )))?;
            // Calculate the XOR checksum for this section
            section.iter().for_each(|&b| checksum ^= b);
            port.write_all(section)?;
        }

        // Write the calculate block checksum to the unit
        port.write_all(&[checksum])?;

        // Read the response checksum from the unit
        let mut res_checksum = [0u8];
        port.read_exact(res_checksum.as_mut())?;
        port.flush()?;

        // Validate that the checksums match
        validate_checksum(checksum, res_checksum[0])?;
    }

    // If the dataset is not divided equally into blocks,
    // send the remaining data in section lengths (128 bytes)
    if block_rem != 0 {
        let mut checksum = 0u8;

        while let Some(section) = data.next_section() {
            section.iter().for_each(|&b| checksum ^= b);
            port.write_all(section)?;
        }

        port.write_all(&[checksum])?;

        let mut res_checksum = [0u8];
        port.read_exact(res_checksum.as_mut())?;
        port.flush()?;

        validate_checksum(checksum, res_checksum[0])?;
    }

    log::debug!("Successfully sent firmware dataset");

    Ok(())
}

// Send the ITL firmware file to the unit over SSP.
fn send_firmware(serial_path: &str, file_path: &str) -> Result<()> {
    log::info!("Sending firmware file...");
    log::debug!("Firmware file path: {file_path}");

    let (header, ram, mut data) = ssp::parse_firmware_file(file_path)?;

    log::debug!("Firmware header: {header}");

    let mut port = connect(serial_path, 9600)?;

    send_sync(&mut port)?;

    let block_len = send_program_firmware_command(&mut port)?;
    data.set_block_len(block_len);

    send_header(&mut port, &header)?;

    // increase baud rate for sending RAM and dataset blocks
    port.set_baud_rate(38400)?;

    send_ram(&mut port, ram)?;

    // sleep for 2.5 seconds to allow unit to run the RAM code we just sent
    // (recommended by SSP Implementation Guide)
    thread::sleep(time::Duration::from_millis(2500));

    send_dataset(&mut port, data, &header)?;

    log::info!("Successfully sent firmware file");

    Ok(())
}

// Check that the unit reset successfully, and has come back online
fn check_online(serial_path: &str) -> Result<()> {
    log::info!("Checking for the unit to come back online...");

    let mut port = connect(serial_path, 9600)?;
    let mut poll_cmd = ssp::PollCommand::new();

    let mut online = false;
    let now = time::SystemTime::now();

    let timeout_secs = 10;
    let timeout = time::Duration::from_secs(timeout_secs);

    while !online
        && now
            .elapsed()
            .map_err(|err| Error::Firmware(format!("error getting elapse system time: {err}")))?
            < timeout
    {
        port.write_all(poll_cmd.as_bytes())?;
        let mut res = ssp::PollResponse::new();
        let read = port.read(res.buf_mut()).unwrap_or(0);
        online = read > 0
            && res.verify_checksum().is_ok()
            && res.response_status() == ssp::ResponseStatus::Ok;
    }

    if online {
        log::info!("Unit is online");
        Ok(())
    } else {
        Err(Error::Firmware(format!(
            "unit is not online after polling for {timeout_secs} seconds"
        )))
    }
}

fn main() -> Result<()> {
    env_logger::init();

    let args = Args::parse();

    let serial_path = args.serial;
    let file_path = args.file;

    send_firmware(serial_path.as_str(), file_path.as_str())?;
    check_online(serial_path.as_str())
}
