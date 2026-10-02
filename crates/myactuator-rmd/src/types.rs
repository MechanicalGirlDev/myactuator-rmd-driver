//! Domain types that encapsulate byte-to-physical-value scaling.

use crate::frame::CanFrame;

/// PI gains (KP/KI), each one byte (0..=255), for one axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PiGains {
    /// Proportional gain.
    pub kp: u8,
    /// Integral gain.
    pub ki: u8,
}

/// PI gains for the current, velocity, and position loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Gains {
    /// Current loop gains.
    pub current: PiGains,
    /// Velocity loop gains.
    pub speed: PiGains,
    /// Position loop gains.
    pub position: PiGains,
}

/// Operating mode returned by `READ_SYSTEM_OPERATING_MODE` (`0x70`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlMode {
    /// Current (torque) control.
    Current = 0x01,
    /// Velocity control.
    Speed = 0x02,
    /// Position control.
    Position = 0x03,
}

impl ControlMode {
    /// Decode an operating mode from a raw byte. Unknown values return `ValueOutOfRange`.
    pub fn from_byte(value: u8) -> crate::Result<Self> {
        match value {
            0x01 => Ok(ControlMode::Current),
            0x02 => Ok(ControlMode::Speed),
            0x03 => Ok(ControlMode::Position),
            _ => Err(crate::Error::ValueOutOfRange),
        }
    }
}

/// Target for an acceleration write (`SetAcceleration` mode in `data[1]`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelerationType {
    /// Position-planning acceleration.
    PositionPlanningAcceleration = 0x00,
    /// Position-planning deceleration.
    PositionPlanningDeceleration = 0x01,
    /// Velocity-planning acceleration.
    SpeedPlanningAcceleration = 0x02,
    /// Velocity-planning deceleration.
    SpeedPlanningDeceleration = 0x03,
}

/// CAN baud rate set by `COMMUNICATION_BAUD_RATE_SETTING` (`0xB4`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CanBaudRate {
    /// 500 kbps.
    Bps500k = 0x00,
    /// 1 Mbps.
    Bps1M = 0x01,
}

/// Shared response for control commands (`0xA1`/`0xA2`/`0xA4`) and MotorStatus2.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feedback {
    /// Temperature [°C].
    pub temperature: i8,
    /// Torque current [A] (raw value × 0.01).
    pub torque_current: f32,
    /// Shaft speed [dps].
    pub shaft_speed: i16,
    /// Shaft angle (raw `i16`, degrees).
    pub angle: i16,
}

impl Feedback {
    /// Decode from a response payload.
    pub fn from_payload(data: &[u8; 8]) -> Self {
        Self {
            temperature: data[1] as i8,
            torque_current: CanFrame::read_i16(data, 2) as f32 * 0.01,
            shaft_speed: CanFrame::read_i16(data, 4),
            angle: CanFrame::read_i16(data, 6),
        }
    }
}

/// Motor status 1 (temperature, voltage, brake, and error flags).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorStatus1 {
    /// Temperature [°C].
    pub temperature: i8,
    /// Whether the brake is released.
    pub brake_released: bool,
    /// Supply voltage [V] (raw value × 0.1).
    pub voltage: f32,
    /// Error state bit flags.
    pub error_state: u16,
}

impl MotorStatus1 {
    /// Decode from a response payload.
    pub fn from_payload(data: &[u8; 8]) -> Self {
        Self {
            temperature: data[1] as i8,
            brake_released: data[3] != 0,
            voltage: CanFrame::read_u16(data, 4) as f32 * 0.1,
            error_state: CanFrame::read_u16(data, 6),
        }
    }
}

/// Motor status 3 (temperature and three-phase current).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorStatus3 {
    /// Temperature [°C].
    pub temperature: i8,
    /// Phase A current [A] (raw value × 0.01).
    pub phase_a_current: f32,
    /// Phase B current [A] (raw value × 0.01).
    pub phase_b_current: f32,
    /// Phase C current [A] (raw value × 0.01).
    pub phase_c_current: f32,
}

impl MotorStatus3 {
    /// Decode from a response payload.
    pub fn from_payload(data: &[u8; 8]) -> Self {
        Self {
            temperature: data[1] as i8,
            phase_a_current: CanFrame::read_i16(data, 2) as f32 * 0.01,
            phase_b_current: CanFrame::read_i16(data, 4) as f32 * 0.01,
            phase_c_current: CanFrame::read_i16(data, 6) as f32 * 0.01,
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn control_mode_roundtrip() {
        assert_eq!(ControlMode::from_byte(0x02).unwrap(), ControlMode::Speed);
        assert!(ControlMode::from_byte(0xFF).is_err());
    }

    #[test]
    fn acceleration_type_value() {
        assert_eq!(AccelerationType::SpeedPlanningAcceleration as u8, 0x02);
    }

    #[test]
    fn feedback_decodes_le_payload() {
        // temp=30, current=100(×0.01=1.0A), speed=1000dps, angle=4500deg-raw
        let data = [0xA1, 30, 0x64, 0x00, 0xE8, 0x03, 0x94, 0x11];
        let fb = Feedback::from_payload(&data);
        assert_eq!(fb.temperature, 30);
        assert!((fb.torque_current - 1.0).abs() < 1e-6);
        assert_eq!(fb.shaft_speed, 1000);
        assert_eq!(fb.angle, 0x1194);
    }

    #[test]
    fn motor_status1_decodes() {
        // temp=25, brake released=1, voltage=240(×0.1=24.0V), error=0x0000
        let data = [0x9A, 25, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00];
        let s = MotorStatus1::from_payload(&data);
        assert_eq!(s.temperature, 25);
        assert!(s.brake_released);
        assert!((s.voltage - 24.0).abs() < 1e-6);
        assert_eq!(s.error_state, 0x0000);
    }
}
