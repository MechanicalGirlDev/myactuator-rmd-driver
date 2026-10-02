//! Command ID definitions matching the C++ `protocol/command_type.hpp` reference.

/// MyActuator RMD command type. The value is the first byte of CAN data (`data[0]`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandType {
    /// Read PID parameters.
    ReadPidParameters = 0x30,
    /// Write PID parameters to RAM.
    WritePidParametersToRam = 0x31,
    /// Write PID parameters to ROM.
    WritePidParametersToRom = 0x32,
    /// Read acceleration.
    ReadAcceleration = 0x42,
    /// Write acceleration to RAM/ROM.
    WriteAccelerationToRamAndRom = 0x43,
    /// Read the system operating mode.
    ReadSystemOperatingMode = 0x70,
    /// Read motor power.
    ReadMotorPower = 0x71,
    /// Reset the system.
    ResetSystem = 0x76,
    /// Release the brake.
    ReleaseBrake = 0x77,
    /// Lock the brake.
    LockBrake = 0x78,
    /// Set the CAN ID.
    CanIdSetting = 0x79,
    /// Shut down the motor.
    ShutdownMotor = 0x80,
    /// Stop the motor.
    StopMotor = 0x81,
    /// Read the multi-turn angle.
    ReadMultiTurnAngle = 0x92,
    /// Read motor status 1 and the error flags.
    ReadMotorStatus1AndErrorFlag = 0x9A,
    /// Read motor status 2.
    ReadMotorStatus2 = 0x9C,
    /// Read motor status 3.
    ReadMotorStatus3 = 0x9D,
    /// Closed-loop torque (current) control.
    TorqueClosedLoopControl = 0xA1,
    /// Closed-loop velocity control.
    SpeedClosedLoopControl = 0xA2,
    /// Closed-loop absolute position control.
    AbsolutePositionClosedLoopControl = 0xA4,
    /// Read the software version date.
    ReadSystemSoftwareVersionDate = 0xB2,
    /// Set the communication baud rate.
    CommunicationBaudRateSetting = 0xB4,
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn command_ids_match_cpp() {
        assert_eq!(CommandType::ReadPidParameters as u8, 0x30);
        assert_eq!(CommandType::TorqueClosedLoopControl as u8, 0xA1);
        assert_eq!(CommandType::AbsolutePositionClosedLoopControl as u8, 0xA4);
        assert_eq!(CommandType::ShutdownMotor as u8, 0x80);
    }
}
