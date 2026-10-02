//! CAN transport over the SLCAN (LAWICEL) ASCII protocol.

use crate::error::{CanError, Result};
use core::fmt::Write as _;
use core::time::Duration;
use serialport::SerialPort;
use std::io::{Read as _, Write as _};

/// I/O boundary for sending and receiving CAN frames.
///
/// [`SlcanPort`] is the hardware implementation. Tests can inject a mock.
pub trait CanIo {
    /// Send a standard CAN frame. `data` is at most 8 bytes.
    fn send_frame(&mut self, can_id: u16, data: &[u8]) -> Result<()>;

    /// Receive one standard CAN frame and return `(can_id, data)`.
    fn recv_frame(&mut self) -> Result<(u16, [u8; 8])>;

    /// Flush the receive buffer.
    fn drain(&mut self) -> Result<()>;
}

/// CAN bit rate supported by the SLCAN `S` command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanBitrate {
    /// 1 Mbps (SLCAN `S8`), the RMD default.
    Bps1M,
    /// 500 kbps（SLCAN `S6`）。
    Bps500k,
    /// 250 kbps（SLCAN `S5`）。
    Bps250k,
    /// 125 kbps（SLCAN `S4`）。
    Bps125k,
}

impl CanBitrate {
    /// Return the SLCAN bit-rate command without a trailing CR.
    pub fn as_slcan_command(self) -> &'static str {
        match self {
            CanBitrate::Bps1M => "S8",
            CanBitrate::Bps500k => "S6",
            CanBitrate::Bps250k => "S5",
            CanBitrate::Bps125k => "S4",
        }
    }
}

/// Convert `(can_id, data)` to SLCAN standard-frame ASCII, including trailing `\r`.
///
/// Format: `t<III><L><DD..>\r` (`III` is the 3-digit uppercase hex ID, `L` is DLC, and `DD` is 2-digit hex data).
///
/// # Contract (truncation and masking)
/// RMD-X uses standard 11-bit CAN IDs (`0x140 + id`) and 8-byte data, so these rules
/// normally do not apply. They define behavior for out-of-range inputs:
/// - `can_id` is masked to the standard frame's 11 bits (`can_id & 0x7FF`).
/// - Data longer than 8 bytes is truncated to its first 8 bytes.
pub fn encode_slcan_frame(can_id: u16, data: &[u8]) -> String {
    let dlc = data.len().min(8);
    let mut s = String::with_capacity(5 + dlc * 2 + 1);
    let _ = write!(s, "t{:03X}{}", can_id & 0x7FF, dlc);
    for b in &data[..dlc] {
        let _ = write!(s, "{:02X}", b);
    }
    s.push('\r');
    s
}

/// Convert one line of SLCAN standard-frame ASCII (without `\r`) to `(can_id, data)`.
///
/// If DLC is less than 8, pad the remaining bytes with zeroes.
pub fn parse_slcan_frame(line: &str) -> Result<(u16, [u8; 8])> {
    if !line.starts_with('t') {
        return Err(CanError::SlcanParse(format!(
            "not a standard frame: {line}"
        )));
    }
    if line.len() < 5 {
        return Err(CanError::SlcanParse(format!("frame is too short: {line}")));
    }
    let id = u16::from_str_radix(&line[1..4], 16)
        .map_err(|_| CanError::SlcanParse(format!("failed to parse ID: {line}")))?;
    let dlc = line[4..5]
        .parse::<u8>()
        .map_err(|_| CanError::SlcanParse(format!("failed to parse DLC: {line}")))?
        as usize;
    if dlc > 8 {
        return Err(CanError::SlcanParse(format!("DLC exceeds 8: {line}")));
    }
    let data_hex = &line[5..];
    if data_hex.len() < dlc * 2 {
        return Err(CanError::SlcanParse(format!(
            "data length is insufficient: {line}"
        )));
    }
    let mut data = [0u8; 8];
    for (i, slot) in data.iter_mut().take(dlc).enumerate() {
        let pair = &data_hex[i * 2..i * 2 + 2];
        *slot = u8::from_str_radix(pair, 16)
            .map_err(|_| CanError::SlcanParse(format!("failed to parse data: {line}")))?;
    }
    Ok((id, data))
}

/// Maximum lines to skip while receiving, such as acknowledgements.
const MAX_RECV_LINES: usize = 16;

/// [`CanIo`] implementation for a SLCAN-compatible USB-CAN adapter.
#[derive(Debug)]
pub struct SlcanPort {
    port: Box<dyn SerialPort>,
}

impl SlcanPort {
    /// Open the port and initialize the SLCAN channel.
    ///
    /// Initialization sequence: `C\r` (close), set bit rate (for example, `S8\r`), then `O\r` (open).
    ///
    /// # Arguments
    /// * `port_path` - Serial port (for example, Windows `COM3` or Linux `/dev/ttyACM0`).
    /// * `serial_baud` - USB serial baud rate (often ignored by USB-CDC adapters).
    /// * `bitrate` - CAN bit rate.
    /// * `timeout` - Read timeout.
    pub fn open(
        port_path: &str,
        serial_baud: u32,
        bitrate: CanBitrate,
        timeout: Duration,
    ) -> Result<Self> {
        let port = serialport::new(port_path, serial_baud)
            .timeout(timeout)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;

        let mut this = Self { port };
        // close → bitrate → open
        this.write_line("C")?;
        this.write_line(bitrate.as_slcan_command())?;
        this.write_line("O")?;
        Ok(this)
    }

    /// Wrap an already-open port.
    ///
    /// Test-only seam: the ASCII line layer (ACK skipping, BEL handling, the closing
    /// `C\r`) is otherwise reachable only through a physical USB-CAN adapter.
    /// See `can::fake::FakeSerial`.
    #[cfg(test)]
    pub(crate) fn from_port(port: Box<dyn SerialPort>) -> Self {
        Self { port }
    }

    /// Write a control command with CR and flush the port.
    fn write_line(&mut self, cmd: &str) -> Result<()> {
        self.port.write_all(cmd.as_bytes())?;
        self.port.write_all(b"\r")?;
        self.port.flush()?;
        Ok(())
    }

    /// Read one line through `\r`. Map timeouts to [`CanError::Timeout`].
    // `core::io` is unstable on stable Rust, so `ErrorKind` has to come from `std` here.
    #[allow(clippy::std_instead_of_core)]
    fn read_line(&mut self) -> Result<String> {
        let mut buf = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            match self.port.read_exact(&mut byte) {
                Ok(()) => match byte[0] {
                    b'\r' => break,
                    0x07 => return Err(CanError::SlcanParse("BEL error response".to_string())),
                    b => buf.push(b),
                },
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    return Err(CanError::Timeout);
                }
                Err(e) => return Err(CanError::Io(e)),
            }
        }
        String::from_utf8(buf).map_err(|_| CanError::SlcanParse("non-ASCII line".to_string()))
    }
}

impl CanIo for SlcanPort {
    fn send_frame(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
        let frame = encode_slcan_frame(can_id, data);
        self.port.write_all(frame.as_bytes())?;
        self.port.flush()?;
        Ok(())
    }

    fn recv_frame(&mut self) -> Result<(u16, [u8; 8])> {
        for _ in 0..MAX_RECV_LINES {
            let line = self.read_line()?;
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if line.starts_with('t') {
                return parse_slcan_frame(line);
            }
            // Skip 'z'/'Z' send acknowledgements and other lines.
        }
        Err(CanError::Timeout)
    }

    fn drain(&mut self) -> Result<()> {
        self.port.clear(serialport::ClearBuffer::Input)?;
        loop {
            let n = self.port.bytes_to_read().unwrap_or(0);
            if n == 0 {
                break;
            }
            let mut discard = vec![0u8; n as usize];
            let _ = self.port.read(&mut discard);
        }
        Ok(())
    }
}

impl Drop for SlcanPort {
    fn drop(&mut self) {
        // Best-effort channel close.
        let _ = self.port.write_all(b"C\r");
        let _ = self.port.flush();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn encode_torque_frame_golden() {
        // CAN ID 0x141, data = [A1,00,00,00,64,00,00,00]
        let s = encode_slcan_frame(0x141, &[0xA1, 0, 0, 0, 0x64, 0, 0, 0]);
        assert_eq!(s, "t1418A100000064000000\r");
    }

    #[test]
    fn parse_roundtrip() {
        let data = [0xA1, 0, 0, 0, 0x64, 0, 0, 0];
        let s = encode_slcan_frame(0x141, &data);
        let line = s.trim_end_matches('\r');
        let (id, parsed) = parse_slcan_frame(line).unwrap();
        assert_eq!(id, 0x141);
        assert_eq!(parsed, data);
    }

    #[test]
    fn parse_short_dlc_zero_pads() {
        // DLC=2, data hex = "A164" → [A1,64,0,0,0,0,0,0]
        let (id, data) = parse_slcan_frame("t1412A164").unwrap();
        assert_eq!(id, 0x141);
        assert_eq!(data, [0xA1, 0x64, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn parse_rejects_non_t_frame() {
        assert!(matches!(
            parse_slcan_frame("z"),
            Err(CanError::SlcanParse(_))
        ));
        assert!(matches!(
            parse_slcan_frame(""),
            Err(CanError::SlcanParse(_))
        ));
    }

    #[test]
    fn parse_rejects_dlc_out_of_range() {
        assert!(matches!(
            parse_slcan_frame("t1419"),
            Err(CanError::SlcanParse(_))
        ));
    }

    #[test]
    fn parse_rejects_non_digit_dlc() {
        // A non-digit 'A' in the DLC position must fail parsing.
        assert!(matches!(
            parse_slcan_frame("t141A0000000000000000"),
            Err(CanError::SlcanParse(_))
        ));
    }

    #[test]
    fn bitrate_command_all_variants() {
        assert_eq!(CanBitrate::Bps1M.as_slcan_command(), "S8");
        assert_eq!(CanBitrate::Bps500k.as_slcan_command(), "S6");
        assert_eq!(CanBitrate::Bps250k.as_slcan_command(), "S5");
        assert_eq!(CanBitrate::Bps125k.as_slcan_command(), "S4");
    }

    // -----------------------------------------------------------------------
    // `SlcanPort` — the ASCII line layer above the serial adapter. Driven through
    // `can::fake::FakeSerial`, since the only other way in is a real USB-CAN dongle.
    // -----------------------------------------------------------------------

    use crate::can::fake::FakeSerial;
    use alloc::sync::Arc;
    use std::sync::Mutex;

    fn slcan(incoming: &[u8]) -> (SlcanPort, Arc<Mutex<Vec<u8>>>) {
        let (fake, written) = FakeSerial::new(incoming);
        (SlcanPort::from_port(Box::new(fake)), written)
    }

    fn sent(log: &Arc<Mutex<Vec<u8>>>) -> String {
        String::from_utf8_lossy(&log.lock().unwrap_or_else(|e| e.into_inner())).into_owned()
    }

    #[test]
    fn send_frame_puts_one_slcan_line_on_the_wire() {
        let (mut port, log) = slcan(b"");
        port.send_frame(0x141, &[0xA1, 0, 0, 0, 0x64, 0, 0, 0])
            .unwrap();
        assert_eq!(sent(&log), "t1418A100000064000000\r");
    }

    #[test]
    fn recv_frame_reads_one_line_and_parses_it() {
        let (mut port, _log) = slcan(b"t1418A100000064000000\r");
        let (id, data) = port.recv_frame().unwrap();
        assert_eq!(id, 0x141);
        assert_eq!(data, [0xA1, 0, 0, 0, 0x64, 0, 0, 0]);
    }

    #[test]
    fn send_acknowledgements_and_blank_lines_are_skipped_not_parsed() {
        // Many adapters echo `z`/`Z` after a successful transmit; treating one as a frame
        // would hand the caller garbage instead of the motor's answer.
        let (mut port, _log) = slcan(b"z\r\rZ\rt1412A164\r");
        let (id, data) = port.recv_frame().unwrap();
        assert_eq!(id, 0x141);
        assert_eq!(&data[..2], &[0xA1, 0x64]);
    }

    #[test]
    fn a_flood_of_noise_gives_up_rather_than_looping_forever() {
        // 17 acknowledgement lines, one past the scan limit.
        let noise: Vec<u8> = core::iter::repeat_n(b"z\r".to_vec(), 17)
            .flatten()
            .collect();
        let (mut port, _log) = slcan(&noise);
        assert!(matches!(port.recv_frame(), Err(CanError::Timeout)));
    }

    #[test]
    fn a_bel_answer_is_reported_as_a_parse_error_not_as_a_frame() {
        // BEL (0x07) is how SLCAN says "I refused that command".
        let (mut port, _log) = slcan(b"\x07");
        assert!(matches!(port.recv_frame(), Err(CanError::SlcanParse(_))));
    }

    #[test]
    fn a_silent_adapter_is_reported_as_a_timeout() {
        let (mut port, _log) = slcan(b"");
        assert!(matches!(port.recv_frame(), Err(CanError::Timeout)));
    }

    #[test]
    fn a_non_ascii_line_is_refused_instead_of_being_lossily_decoded() {
        let (mut port, _log) = slcan(b"t\xFF\xFE\r");
        assert!(matches!(port.recv_frame(), Err(CanError::SlcanParse(_))));
    }

    #[test]
    fn drain_empties_whatever_the_adapter_had_queued() {
        let (mut port, _log) = slcan(b"t1418A100000064000000\rz\r");
        port.drain().unwrap();
        assert!(matches!(port.recv_frame(), Err(CanError::Timeout)));
    }

    #[test]
    fn dropping_the_port_closes_the_slcan_channel() {
        // Leaving the channel open keeps the adapter arbitrating on the bus after we are
        // gone, so the close is best-effort but must be attempted.
        let (port, log) = slcan(b"");
        drop(port);
        assert_eq!(sent(&log), "C\r");
    }
}
