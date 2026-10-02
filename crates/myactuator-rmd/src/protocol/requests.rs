//! Build request frames sent from the driver to the actuator.

use crate::command::CommandType;
use crate::error::Result;
use crate::frame::{CanFrame, MOTOR_ID_RANGE};
use crate::types::{AccelerationType, Gains};

/// Set acceleration in RAM/ROM. `acceleration` is in dps (100..=60000).
pub fn encode_set_acceleration(
    motor_id: u8,
    acceleration: u32,
    mode: AccelerationType,
) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, CommandType::WriteAccelerationToRamAndRom)?;
    frame.data[1] = mode as u8;
    frame.write_u32(4, acceleration);
    Ok(frame)
}

/// Write PID gains to RAM (`0x31`).
pub fn encode_write_gains_ram(motor_id: u8, gains: Gains) -> Result<CanFrame> {
    encode_write_gains(motor_id, gains, CommandType::WritePidParametersToRam)
}

/// Write PID gains to ROM (`0x32`).
pub fn encode_write_gains_rom(motor_id: u8, gains: Gains) -> Result<CanFrame> {
    encode_write_gains(motor_id, gains, CommandType::WritePidParametersToRom)
}

fn encode_write_gains(motor_id: u8, gains: Gains, command: CommandType) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, command)?;
    frame.data[2] = gains.current.kp;
    frame.data[3] = gains.current.ki;
    frame.data[4] = gains.speed.kp;
    frame.data[5] = gains.speed.ki;
    frame.data[6] = gains.position.kp;
    frame.data[7] = gains.position.ki;
    Ok(frame)
}

/// Set the CAN ID (`0x79`). `can_id` must be 1..=32.
///
/// Verified against the C++ implementation: the write flag is `data[2]=0x00` and `can_id` is in `data[6]`.
pub fn encode_set_can_id(motor_id: u8, can_id: u8) -> Result<CanFrame> {
    if !MOTOR_ID_RANGE.contains(&can_id) {
        return Err(crate::Error::ValueOutOfRange);
    }
    let mut frame = CanFrame::new(motor_id, CommandType::CanIdSetting)?;
    // data[2] = 0x00 selects write (read uses 0x01); the ID is in data[6].
    frame.data[6] = can_id;
    Ok(frame)
}

/// Shut down the motor (`0x80`).
pub fn encode_shutdown_motor(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ShutdownMotor)
}

/// Stop the motor (`0x81`).
pub fn encode_stop_motor(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::StopMotor)
}

/// Reset the motor (`0x76`).
pub fn encode_reset(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ResetSystem)
}

/// Release the brake (`0x77`).
pub fn encode_release_brake(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReleaseBrake)
}

/// Lock the brake (`0x78`).
pub fn encode_lock_brake(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::LockBrake)
}

/// Closed-loop torque (current) control. `current` is in amperes.
pub fn encode_torque_setpoint(motor_id: u8, current: f32) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, CommandType::TorqueClosedLoopControl)?;
    let raw = (current / 0.01) as i16;
    frame.write_i16(4, raw);
    Ok(frame)
}

/// Closed-loop velocity control. `speed` is in dps.
pub fn encode_velocity_setpoint(motor_id: u8, speed: f32) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, CommandType::SpeedClosedLoopControl)?;
    let raw = (speed * 100.0) as i32;
    frame.write_i32(4, raw);
    Ok(frame)
}

/// Closed-loop absolute position control. `position` is in degrees and `max_speed` is in dps.
pub fn encode_position_absolute_setpoint(
    motor_id: u8,
    position: f32,
    max_speed: u16,
) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, CommandType::AbsolutePositionClosedLoopControl)?;
    frame.write_u16(2, max_speed);
    let raw = (position * 100.0) as i32;
    frame.write_i32(4, raw);
    Ok(frame)
}

/// Request the multi-turn angle (`0x92`).
pub fn encode_read_multi_turn_angle(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadMultiTurnAngle)
}

/// Request acceleration (`0x42`).
pub fn encode_read_acceleration(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadAcceleration)
}

/// Request motor status 1 (`0x9A`).
pub fn encode_read_motor_status1(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadMotorStatus1AndErrorFlag)
}

/// Request motor status 2 (`0x9C`).
pub fn encode_read_motor_status2(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadMotorStatus2)
}

/// Request motor status 3 (`0x9D`).
pub fn encode_read_motor_status3(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadMotorStatus3)
}

/// Request the operating mode (`0x70`).
pub fn encode_read_operating_mode(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadSystemOperatingMode)
}

/// Request PID gains (`0x30`).
pub fn encode_read_gains(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadPidParameters)
}

/// Request the software version date (`0xB2`).
pub fn encode_read_version_date(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadSystemSoftwareVersionDate)
}

use crate::types::CanBaudRate;

/// Request motor power (`0x71`).
pub fn encode_read_motor_power(motor_id: u8) -> Result<CanFrame> {
    CanFrame::new(motor_id, CommandType::ReadMotorPower)
}

/// Set the communication baud rate (`0xB4`).
pub fn encode_set_baud_rate(motor_id: u8, baud: CanBaudRate) -> Result<CanFrame> {
    let mut frame = CanFrame::new(motor_id, CommandType::CommunicationBaudRateSetting)?;
    frame.data[7] = baud as u8;
    Ok(frame)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn torque_golden() {
        let f = encode_torque_setpoint(1, 1.0).unwrap();
        assert_eq!(f.data, [0xA1, 0, 0, 0, 0x64, 0x00, 0, 0]);
    }

    #[test]
    fn velocity_golden() {
        let f = encode_velocity_setpoint(1, 10.0).unwrap();
        assert_eq!(f.data, [0xA2, 0, 0, 0, 0xE8, 0x03, 0x00, 0x00]);
    }

    #[test]
    fn position_golden() {
        let f = encode_position_absolute_setpoint(1, 90.0, 500).unwrap();
        assert_eq!(f.data, [0xA4, 0, 0xF4, 0x01, 0x28, 0x23, 0x00, 0x00]);
    }

    #[test]
    fn acceleration_golden() {
        use crate::types::AccelerationType;
        let f =
            encode_set_acceleration(1, 10000, AccelerationType::SpeedPlanningAcceleration).unwrap();
        assert_eq!(f.data, [0x43, 0x02, 0, 0, 0x10, 0x27, 0x00, 0x00]);
    }

    #[test]
    fn gains_golden() {
        use crate::types::{Gains, PiGains};
        let gains = Gains {
            current: PiGains { kp: 10, ki: 20 },
            speed: PiGains { kp: 30, ki: 40 },
            position: PiGains { kp: 50, ki: 60 },
        };
        let f = encode_write_gains_ram(1, gains).unwrap();
        assert_eq!(f.data, [0x31, 0, 10, 20, 30, 40, 50, 60]);
    }

    #[test]
    fn set_can_id_golden() {
        // C++ SetCanIdRequest: data[2]=write flag (0), data[6]=can_id.
        let f = encode_set_can_id(1, 2).unwrap();
        assert_eq!(f.data, [0x79, 0, 0, 0, 0, 0, 0x02, 0x00]);
    }

    #[test]
    fn set_can_id_out_of_range_errors() {
        use crate::Error;
        assert_eq!(encode_set_can_id(1, 0), Err(Error::ValueOutOfRange));
        assert_eq!(encode_set_can_id(1, 33), Err(Error::ValueOutOfRange));
    }

    #[test]
    fn system_commands_have_empty_payload() {
        assert_eq!(
            encode_stop_motor(1).unwrap().data,
            [0x81, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_shutdown_motor(1).unwrap().data,
            [0x80, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(encode_reset(1).unwrap().data, [0x76, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            encode_release_brake(1).unwrap().data,
            [0x77, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_lock_brake(1).unwrap().data,
            [0x78, 0, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn read_requests_are_command_only() {
        assert_eq!(
            encode_read_multi_turn_angle(1).unwrap().data,
            [0x92, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_read_motor_status1(1).unwrap().data,
            [0x9A, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_read_gains(1).unwrap().data,
            [0x30, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(
            encode_read_version_date(1).unwrap().data,
            [0xB2, 0, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn remaining_requests_golden() {
        use crate::types::CanBaudRate;
        assert_eq!(
            encode_set_baud_rate(1, CanBaudRate::Bps1M).unwrap().data,
            [0xB4, 0, 0, 0, 0, 0, 0, 0x01]
        );
    }
}
