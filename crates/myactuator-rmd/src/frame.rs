//! CAN frame representation and little-endian payload helpers.

use crate::command::CommandType;
use crate::error::{Error, Result};

/// Base CAN ID for a single motor. The actual CAN ID is `BASE_ID + motor_id`.
pub const BASE_CAN_ID: u16 = 0x140;

/// Valid motor/CAN ID range (1..=32). Out-of-range values return `ValueOutOfRange`.
pub const MOTOR_ID_RANGE: core::ops::RangeInclusive<u8> = 1..=32;

/// A frame containing a fixed 8-byte CAN data field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// CAN identifier (`0x140 + motor_id`).
    pub can_id: u16,
    /// 8-byte data field. `data[0]` is the command ID.
    pub data: [u8; 8],
}

impl CanFrame {
    /// Create an empty frame for the specified command (zeroed payload).
    ///
    /// `motor_id` must be 1..=32. Out-of-range values return `ValueOutOfRange`.
    pub fn new(motor_id: u8, command: CommandType) -> Result<Self> {
        if !MOTOR_ID_RANGE.contains(&motor_id) {
            return Err(Error::ValueOutOfRange);
        }
        let mut data = [0u8; 8];
        data[0] = command as u8;
        Ok(Self {
            can_id: BASE_CAN_ID + motor_id as u16,
            data,
        })
    }

    /// Write a `u16` in little-endian order to `data[index..index+2]`.
    pub fn write_u16(&mut self, index: usize, value: u16) {
        self.data[index..index + 2].copy_from_slice(&value.to_le_bytes());
    }

    /// Write an `i16` in little-endian order to `data[index..index+2]`.
    pub fn write_i16(&mut self, index: usize, value: i16) {
        self.data[index..index + 2].copy_from_slice(&value.to_le_bytes());
    }

    /// Write an `i32` in little-endian order to `data[index..index+4]`.
    pub fn write_i32(&mut self, index: usize, value: i32) {
        self.data[index..index + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// Write a `u32` in little-endian order to `data[index..index+4]`.
    pub fn write_u32(&mut self, index: usize, value: u32) {
        self.data[index..index + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// Read a little-endian `i16` from `data[index..index+2]`.
    pub fn read_i16(data: &[u8; 8], index: usize) -> i16 {
        i16::from_le_bytes([data[index], data[index + 1]])
    }

    /// Read a little-endian `u16` from `data[index..index+2]`.
    pub fn read_u16(data: &[u8; 8], index: usize) -> u16 {
        u16::from_le_bytes([data[index], data[index + 1]])
    }

    /// Read a little-endian `i32` from `data[index..index+4]`.
    pub fn read_i32(data: &[u8; 8], index: usize) -> i32 {
        i32::from_le_bytes([
            data[index],
            data[index + 1],
            data[index + 2],
            data[index + 3],
        ])
    }

    /// Read a little-endian `u32` from `data[index..index+4]`.
    pub fn read_u32(data: &[u8; 8], index: usize) -> u32 {
        u32::from_le_bytes([
            data[index],
            data[index + 1],
            data[index + 2],
            data[index + 3],
        ])
    }
}

/// Check whether the response frame's first byte matches the expected command.
pub fn expect_command(data: &[u8; 8], expected: CommandType) -> Result<()> {
    let actual = data[0];
    if actual == expected as u8 {
        Ok(())
    } else {
        Err(Error::UnexpectedCommand {
            expected: expected as u8,
            actual,
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn new_sets_can_id_and_command() {
        let frame = CanFrame::new(1, CommandType::TorqueClosedLoopControl).unwrap();
        assert_eq!(frame.can_id, 0x141);
        assert_eq!(frame.data, [0xA1, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn motor_id_out_of_range_errors() {
        assert_eq!(
            CanFrame::new(0, CommandType::StopMotor),
            Err(Error::ValueOutOfRange)
        );
        assert_eq!(
            CanFrame::new(33, CommandType::StopMotor),
            Err(Error::ValueOutOfRange)
        );
    }

    #[test]
    fn le_roundtrip() {
        let mut frame = CanFrame::new(1, CommandType::SpeedClosedLoopControl).unwrap();
        frame.write_i32(4, 1000);
        assert_eq!(&frame.data[4..8], &[0xE8, 0x03, 0x00, 0x00]);
        assert_eq!(CanFrame::read_i32(&frame.data, 4), 1000);
    }

    #[test]
    fn expect_command_detects_mismatch() {
        let data = [0xA1, 0, 0, 0, 0, 0, 0, 0];
        assert!(expect_command(&data, CommandType::TorqueClosedLoopControl).is_ok());
        assert_eq!(
            expect_command(&data, CommandType::StopMotor),
            Err(Error::UnexpectedCommand {
                expected: 0x81,
                actual: 0xA1
            })
        );
    }
}
