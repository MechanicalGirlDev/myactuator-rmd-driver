//! MyActuator RMD-X CAN protocol with wire-compatible encoding and decoding.
//! Supports `no_std`.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod command;
pub mod error;
pub mod frame;
pub mod protocol;
pub mod types;

pub use command::CommandType;
pub use error::{Error, Result};
pub use frame::{BASE_CAN_ID, CanFrame};
pub use types::{
    AccelerationType, CanBaudRate, ControlMode, Feedback, Gains, MotorStatus1, MotorStatus3,
    PiGains,
};

pub use protocol::requests::{
    encode_lock_brake, encode_position_absolute_setpoint, encode_read_acceleration,
    encode_read_gains, encode_read_motor_power, encode_read_motor_status1,
    encode_read_motor_status2, encode_read_motor_status3, encode_read_multi_turn_angle,
    encode_read_operating_mode, encode_read_version_date, encode_release_brake, encode_reset,
    encode_set_acceleration, encode_set_baud_rate, encode_set_can_id, encode_shutdown_motor,
    encode_stop_motor, encode_torque_setpoint, encode_velocity_setpoint, encode_write_gains_ram,
    encode_write_gains_rom,
};
pub use protocol::responses::{
    decode_acceleration, decode_feedback, decode_gains, decode_motor_power, decode_motor_status1,
    decode_motor_status3, decode_multi_turn_angle, decode_operating_mode, decode_version_date,
};
