use std::io::{Read, Write};
use std::thread;
use std::time;

use clap::Parser;
use serialport::TTYPort;

use ssp::{Error, FirmwareData, FirmwareHeader, FirmwareRam, MessageOps, ResponseOps, Result};

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
        // no control bit parity
        .parity(serialport::Parity::None)
        // two bit stop
        .stop_bits(serialport::StopBits::Two)
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

fn send_reset(port: &mut TTYPort) -> Result<()> {
    let mut reset = ssp::ResetCommand::new();
    port.write_all(reset.as_bytes())?;

    Ok(())
}

fn send_program_firmware_command(port: &mut TTYPort) -> Result<u16> {
    log::debug!("Sending ProgramFirmware command...");

    let mut program_cmd = ssp::ProgramFirmwareCommand::create(ssp::ProgramFirmwareCode::Currency);
    port.write_all(program_cmd.as_bytes())?;

    let mut program_res = ssp::ProgramFirmwareResponse::new();
    port.read_exact(program_res.buf_mut())?;
    program_res.verify_checksum()?;

    let block_len = program_res.block_len();
    log::info!("Received expected block length: {block_len}");

    log::debug!("ProgramFirmware command successful");

    Ok(block_len)
}

// Send the firmware header to the unit.
fn send_header(port: &mut TTYPort, header: &FirmwareHeader) -> Result<()> {
    log::debug!("Sending firmware header...");

    let header_buf: [u8; ssp::FIRMWARE_HEADER_LEN] = header.try_into()?;
    send_data_packet(
        port,
        ssp::SequenceId::new(),
        0xffff_ffff,
        0,
        header_buf.as_ref(),
    )?;

    log::debug!("Successfully sent firmware header");

    Ok(())
}

// Send the firmware RAM code to the unit.
fn send_ram(port: &mut TTYPort, mut ram: FirmwareRam) -> Result<()> {
    log::debug!("Sending firmware RAM...");

    let mut block = 0xffff_fffe;
    let mut line = 0;
    let mut seq_id = ssp::SequenceId::new();

    while let Some(section) = ram.next_section() {
        seq_id.toggle_flag();

        send_data_packet(port, seq_id, block, line, section)?;

        // advance to the next "line"
        line = line.saturating_add(1);
        // if line is at the max, advance to next block
        if line == 255 {
            block = block.saturating_sub(1);
            line = 0;
        }
    }

    log::debug!("Successfully sent firmware RAM");

    Ok(())
}

// Send the firmware dataset to the unit.
fn send_dataset(port: &mut TTYPort, mut data: FirmwareData) -> Result<()> {
    log::debug!("Sending firmware dataset...");

    let mut block = 0xffff_fffe;
    let mut line = 0;
    let mut seq_id = ssp::SequenceId::new();

    while let Some(section) = data.next_section() {
        seq_id.toggle_flag();

        send_data_packet(port, seq_id, block, line, section)?;

        // advance to the next "line"
        line = line.saturating_add(1);
        // if line is at the max, advance to next block
        if line == 255 {
            block = block.saturating_sub(1);
            line = 0;
        }
    }

    log::debug!("Successfully sent firmware dataset");

    Ok(())
}

// Send a data packet to the device.
fn send_data_packet(
    port: &mut TTYPort,
    seq_id: ssp::SequenceId,
    block: u32,
    line: u8,
    data: &[u8],
) -> Result<()> {
    let mut data_cmd = ssp::DownloadDataPacketCommand::create(block, line, data)?;
    data_cmd.set_sequence_id(seq_id);

    port.write_all(data_cmd.as_bytes())?;

    let mut data_res = ssp::DownloadDataPacketResponse::new();
    let read = port.read(data_res.buf_mut())?;

    let exp_len = data_res.len();
    if read != exp_len {
        return Err(Error::InvalidLength((read, exp_len)));
    }

    data_res.verify_checksum()?;

    let status = data_res.response_status();
    let exp_status = ssp::ResponseStatus::Ok;
    if status == exp_status {
        Ok(())
    } else {
        Err(Error::Firmware(format!(
            "invalid firmware download status: {status}, expected: {exp_status}"
        )))
    }
}

// Send the ITL firmware file to the unit over SSP.
fn send_firmware(serial_path: &str, file_path: &str) -> Result<()> {
    log::info!("Sending firmware file...");
    log::debug!("Firmware file path: {file_path}");

    let (header, ram, data) = ssp::parse_firmware_file(file_path)?;

    log::debug!("Firmware header: {header}");

    let mut port = connect(serial_path, 9600)?;

    send_sync(&mut port)?;

    send_program_firmware_command(&mut port)?;

    send_header(&mut port, &header)?;

    send_ram(&mut port, ram)?;

    // sleep for 2.5 seconds to allow unit to run the RAM code we just sent
    // (recommended by SSP Implementation Guide)
    thread::sleep(time::Duration::from_millis(2500));

    send_sync(&mut port)?;

    send_header(&mut port, &header)?;

    send_dataset(&mut port, data)?;

    log::info!("Successfully sent firmware file");

    send_reset(&mut port)?;

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
