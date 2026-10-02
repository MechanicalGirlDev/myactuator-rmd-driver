//! Decode actuator response frames into domain types.

use crate::command::CommandType;
use crate::error::Result;
use crate::frame::{CanFrame, expect_command};
use crate::types::{ControlMode, Feedback, Gains, MotorStatus1, MotorStatus3, PiGains};

/// Decode a control command or MotorStatus2 response into `Feedback`.
///
/// Pass the corresponding command as `expected` (for example, `TorqueClosedLoopControl`).
pub fn decode_feedback(data: &[u8; 8], expected: CommandType) -> Result<Feedback> {
    expect_command(data, expected)?;
    Ok(Feedback::from_payload(data))
}

/// Decode a motor status 1 response (`0x9A`).
pub fn decode_motor_status1(data: &[u8; 8]) -> Result<MotorStatus1> {
    expect_command(data, CommandType::ReadMotorStatus1AndErrorFlag)?;
    Ok(MotorStatus1::from_payload(data))
}

/// Decode a motor status 3 response (`0x9D`).
pub fn decode_motor_status3(data: &[u8; 8]) -> Result<MotorStatus3> {
    expect_command(data, CommandType::ReadMotorStatus3)?;
    Ok(MotorStatus3::from_payload(data))
}

/// Decode a multi-turn angle response (`0x92`) in degrees.
pub fn decode_multi_turn_angle(data: &[u8; 8]) -> Result<f32> {
    expect_command(data, CommandType::ReadMultiTurnAngle)?;
    Ok(CanFrame::read_i32(data, 4) as f32 * 0.01)
}

/// Decode an acceleration response (`0x42`) in dps.
pub fn decode_acceleration(data: &[u8; 8]) -> Result<i32> {
    expect_command(data, CommandType::ReadAcceleration)?;
    Ok(CanFrame::read_i32(data, 4))
}

/// Decode a software version date response (`0xB2`) in `YYYYMMDD` format.
pub fn decode_version_date(data: &[u8; 8]) -> Result<u32> {
    expect_command(data, CommandType::ReadSystemSoftwareVersionDate)?;
    Ok(CanFrame::read_u32(data, 4))
}

/// Decode an operating mode response (`0x70`).
pub fn decode_operating_mode(data: &[u8; 8]) -> Result<ControlMode> {
    expect_command(data, CommandType::ReadSystemOperatingMode)?;
    ControlMode::from_byte(data[7])
}

/// Decode a PID gains response (`0x30`).
pub fn decode_gains(data: &[u8; 8]) -> Result<Gains> {
    expect_command(data, CommandType::ReadPidParameters)?;
    Ok(Gains {
        current: PiGains {
            kp: data[2],
            ki: data[3],
        },
        speed: PiGains {
            kp: data[4],
            ki: data[5],
        },
        position: PiGains {
            kp: data[6],
            ki: data[7],
        },
    })
}

/// Decode a motor power response (`0x71`) in watts.
pub fn decode_motor_power(data: &[u8; 8]) -> Result<f32> {
    expect_command(data, CommandType::ReadMotorPower)?;
    Ok(CanFrame::read_u16(data, 6) as f32 * 0.1)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::error::Error;

    #[test]
    fn feedback_command_mismatch_errors() {
        let data = [0xA2, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(
            decode_feedback(&data, CommandType::TorqueClosedLoopControl),
            Err(Error::UnexpectedCommand {
                expected: 0xA1,
                actual: 0xA2
            })
        );
    }

    #[test]
    fn multi_turn_angle_scaled() {
        // raw 9000 → 90.0 deg
        let data = [0x92, 0, 0, 0, 0x28, 0x23, 0x00, 0x00];
        assert!((decode_multi_turn_angle(&data).unwrap() - 90.0).abs() < 1e-3);
    }

    #[test]
    fn operating_mode_decoded() {
        let data = [0x70, 0, 0, 0, 0, 0, 0, 0x03];
        assert_eq!(decode_operating_mode(&data).unwrap(), ControlMode::Position);
    }

    #[test]
    fn gains_decoded() {
        let data = [0x30, 0, 10, 20, 30, 40, 50, 60];
        let g = decode_gains(&data).unwrap();
        assert_eq!(g.current, PiGains { kp: 10, ki: 20 });
        assert_eq!(g.position, PiGains { kp: 50, ki: 60 });
    }

    #[test]
    fn version_date_decoded() {
        // 20231201 = 0x0134B421 (LE: 0x21, 0xB4, 0x34, 0x01)
        let data = [0xB2, 0, 0, 0, 0x21, 0xB4, 0x34, 0x01];
        assert_eq!(decode_version_date(&data).unwrap(), 20_231_201);
    }

    #[test]
    fn motor_power_scaled() {
        // raw 250 → 25.0 W
        let data = [0x71, 0, 0, 0, 0, 0, 0xFA, 0x00];
        assert!((decode_motor_power(&data).unwrap() - 25.0).abs() < 1e-3);
    }
}
