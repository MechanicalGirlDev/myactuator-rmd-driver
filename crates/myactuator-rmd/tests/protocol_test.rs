//! Wire-format tests for the MyActuator RMD-X protocol (command IDs match the C++
//! reference implementation's `protocol/command_type.hpp`).
//!
//! Every payload is little-endian and 8 bytes; `data[0]` is the command ID, which the
//! actuator echoes back on every reply. That echo is the only framing there is, so the
//! encode / decode pair per command is what these tests pin.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

use myactuator_rmd::frame::expect_command;
use myactuator_rmd::{
    AccelerationType, BASE_CAN_ID, CanBaudRate, CanFrame, CommandType, ControlMode, Error, Gains,
    PiGains, decode_acceleration, decode_feedback, decode_gains, decode_motor_power,
    decode_motor_status1, decode_motor_status3, decode_multi_turn_angle, decode_operating_mode,
    decode_version_date, encode_lock_brake, encode_position_absolute_setpoint,
    encode_read_acceleration, encode_read_gains, encode_read_motor_power,
    encode_read_motor_status1, encode_read_motor_status2, encode_read_motor_status3,
    encode_read_multi_turn_angle, encode_read_operating_mode, encode_read_version_date,
    encode_release_brake, encode_reset, encode_set_acceleration, encode_set_baud_rate,
    encode_set_can_id, encode_shutdown_motor, encode_stop_motor, encode_torque_setpoint,
    encode_velocity_setpoint, encode_write_gains_ram, encode_write_gains_rom,
};

const ID: u8 = 1;
const CAN_ID: u16 = 0x141;

// ---------------------------------------------------------------------------
// Requests that carry no payload beyond the command ID
// ---------------------------------------------------------------------------

#[test]
fn payload_less_requests_are_just_their_command_id() {
    let cases: [(CanFrame, u8); 12] = [
        (encode_shutdown_motor(ID).unwrap(), 0x80),
        (encode_stop_motor(ID).unwrap(), 0x81),
        (encode_reset(ID).unwrap(), 0x76),
        (encode_release_brake(ID).unwrap(), 0x77),
        (encode_lock_brake(ID).unwrap(), 0x78),
        (encode_read_multi_turn_angle(ID).unwrap(), 0x92),
        (encode_read_acceleration(ID).unwrap(), 0x42),
        (encode_read_motor_status1(ID).unwrap(), 0x9A),
        (encode_read_motor_status2(ID).unwrap(), 0x9C),
        (encode_read_motor_status3(ID).unwrap(), 0x9D),
        (encode_read_operating_mode(ID).unwrap(), 0x70),
        (encode_read_gains(ID).unwrap(), 0x30),
    ];
    for (frame, cmd) in cases {
        assert_eq!(frame.can_id, CAN_ID);
        assert_eq!(frame.data, [cmd, 0, 0, 0, 0, 0, 0, 0], "cmd {cmd:#04X}");
    }
    // The two read commands not in the table above, for completeness.
    assert_eq!(encode_read_version_date(ID).unwrap().data[0], 0xB2);
    assert_eq!(encode_read_motor_power(ID).unwrap().data[0], 0x71);
}

#[test]
fn can_id_is_the_base_plus_the_motor_id_across_the_whole_range() {
    assert_eq!(BASE_CAN_ID, 0x140);
    for id in 1..=32u8 {
        assert_eq!(
            encode_stop_motor(id).unwrap().can_id,
            BASE_CAN_ID + u16::from(id)
        );
    }
}

#[test]
fn motor_ids_outside_one_to_thirty_two_are_refused_by_every_encoder() {
    for id in [0u8, 33, 255] {
        assert_eq!(encode_stop_motor(id), Err(Error::ValueOutOfRange));
        assert_eq!(encode_torque_setpoint(id, 0.0), Err(Error::ValueOutOfRange));
        assert_eq!(encode_read_gains(id), Err(Error::ValueOutOfRange));
        assert_eq!(
            encode_set_acceleration(id, 1000, AccelerationType::PositionPlanningAcceleration),
            Err(Error::ValueOutOfRange)
        );
    }
}

// ---------------------------------------------------------------------------
// Setpoints
// ---------------------------------------------------------------------------

#[test]
fn torque_setpoint_scales_amps_by_one_hundredth_and_signs_correctly() {
    // 1.0 A → raw 100 (0x0064, LE at data[4..6])
    assert_eq!(
        encode_torque_setpoint(ID, 1.0).unwrap().data,
        [0xA1, 0, 0, 0, 0x64, 0x00, 0, 0]
    );
    // -1.0 A → raw -100 = 0xFF9C
    assert_eq!(
        encode_torque_setpoint(ID, -1.0).unwrap().data,
        [0xA1, 0, 0, 0, 0x9C, 0xFF, 0, 0]
    );
    assert_eq!(
        encode_torque_setpoint(ID, 0.0).unwrap().data,
        [0xA1, 0, 0, 0, 0, 0, 0, 0]
    );
}

#[test]
fn velocity_setpoint_scales_dps_by_one_hundred_into_an_i32() {
    // 10 dps → 1000 (0x000003E8)
    assert_eq!(
        encode_velocity_setpoint(ID, 10.0).unwrap().data,
        [0xA2, 0, 0, 0, 0xE8, 0x03, 0x00, 0x00]
    );
    // -10 dps → -1000 = 0xFFFFFC18
    assert_eq!(
        encode_velocity_setpoint(ID, -10.0).unwrap().data,
        [0xA2, 0, 0, 0, 0x18, 0xFC, 0xFF, 0xFF]
    );
}

#[test]
fn position_setpoint_puts_max_speed_at_two_and_the_angle_at_four() {
    // 90 deg → 9000 (0x00002328); max_speed 500 → 0x01F4 at data[2..4]
    assert_eq!(
        encode_position_absolute_setpoint(ID, 90.0, 500)
            .unwrap()
            .data,
        [0xA4, 0, 0xF4, 0x01, 0x28, 0x23, 0x00, 0x00]
    );
    // Negative angles use the same i32 field.
    assert_eq!(
        &encode_position_absolute_setpoint(ID, -90.0, 0)
            .unwrap()
            .data[4..],
        &[0xD8, 0xDC, 0xFF, 0xFF]
    );
}

// ---------------------------------------------------------------------------
// Configuration writes
// ---------------------------------------------------------------------------

#[test]
fn gain_writes_differ_only_in_their_command_id() {
    let gains = Gains {
        current: PiGains { kp: 10, ki: 20 },
        speed: PiGains { kp: 30, ki: 40 },
        position: PiGains { kp: 50, ki: 60 },
    };
    let ram = encode_write_gains_ram(ID, gains).unwrap();
    let rom = encode_write_gains_rom(ID, gains).unwrap();
    assert_eq!(ram.data, [0x31, 0, 10, 20, 30, 40, 50, 60]);
    assert_eq!(rom.data, [0x32, 0, 10, 20, 30, 40, 50, 60]);
    // A written image reads back through the decoder unchanged.
    assert_eq!(
        decode_gains(&[0x30, 0, 10, 20, 30, 40, 50, 60]).unwrap(),
        gains
    );
}

#[test]
fn set_acceleration_puts_the_target_at_one_and_the_value_at_four() {
    for (mode, byte) in [
        (AccelerationType::PositionPlanningAcceleration, 0x00),
        (AccelerationType::PositionPlanningDeceleration, 0x01),
        (AccelerationType::SpeedPlanningAcceleration, 0x02),
        (AccelerationType::SpeedPlanningDeceleration, 0x03),
    ] {
        let f = encode_set_acceleration(ID, 10_000, mode).unwrap();
        assert_eq!(f.data[0], 0x43);
        assert_eq!(f.data[1], byte);
        // 10000 = 0x00002710
        assert_eq!(&f.data[4..], &[0x10, 0x27, 0x00, 0x00]);
    }
}

#[test]
fn set_can_id_writes_the_new_id_at_byte_six_and_range_checks_it() {
    let f = encode_set_can_id(ID, 32).unwrap();
    assert_eq!(f.data, [0x79, 0, 0, 0, 0, 0, 32, 0]);
    // The *new* id has the same 1..=32 constraint as the addressed motor.
    assert_eq!(encode_set_can_id(ID, 0), Err(Error::ValueOutOfRange));
    assert_eq!(encode_set_can_id(ID, 33), Err(Error::ValueOutOfRange));
}

#[test]
fn set_baud_rate_writes_the_code_at_byte_seven() {
    assert_eq!(
        encode_set_baud_rate(ID, CanBaudRate::Bps500k).unwrap().data,
        [0xB4, 0, 0, 0, 0, 0, 0, 0x00]
    );
    assert_eq!(
        encode_set_baud_rate(ID, CanBaudRate::Bps1M).unwrap().data,
        [0xB4, 0, 0, 0, 0, 0, 0, 0x01]
    );
}

// ---------------------------------------------------------------------------
// Responses
// ---------------------------------------------------------------------------

#[test]
fn every_decoder_rejects_a_reply_carrying_another_command_id() {
    // A reply for the wrong command is the failure mode a shared bus actually produces
    // (a late frame from the previous request), so each decoder must catch it.
    let wrong = [0x00u8, 0, 0, 0, 0, 0, 0, 0];
    assert!(decode_multi_turn_angle(&wrong).is_err());
    assert!(decode_acceleration(&wrong).is_err());
    assert!(decode_motor_status1(&wrong).is_err());
    assert!(decode_motor_status3(&wrong).is_err());
    assert!(decode_gains(&wrong).is_err());
    assert!(decode_motor_power(&wrong).is_err());
    assert!(decode_version_date(&wrong).is_err());
    assert!(decode_operating_mode(&wrong).is_err());
    assert_eq!(
        decode_feedback(&wrong, CommandType::TorqueClosedLoopControl),
        Err(Error::UnexpectedCommand {
            expected: 0xA1,
            actual: 0x00
        })
    );
}

#[test]
fn feedback_is_shared_by_the_three_setpoints_and_motor_status2() {
    // temp=30, current raw 100 (=1.00 A), speed 1000 dps, angle raw 0x1194
    for cmd in [
        CommandType::TorqueClosedLoopControl,
        CommandType::SpeedClosedLoopControl,
        CommandType::AbsolutePositionClosedLoopControl,
        CommandType::ReadMotorStatus2,
    ] {
        let data = [cmd as u8, 30, 0x64, 0x00, 0xE8, 0x03, 0x94, 0x11];
        let fb = decode_feedback(&data, cmd).unwrap();
        assert_eq!(fb.temperature, 30);
        assert!((fb.torque_current - 1.0).abs() < 1e-6);
        assert_eq!(fb.shaft_speed, 1000);
        assert_eq!(fb.angle, 0x1194);
    }
}

#[test]
fn feedback_carries_signed_temperature_current_and_speed() {
    // temp = -1 (0xFF), current raw -100, speed -1000, angle -1
    let data = [0xA1, 0xFF, 0x9C, 0xFF, 0x18, 0xFC, 0xFF, 0xFF];
    let fb = decode_feedback(&data, CommandType::TorqueClosedLoopControl).unwrap();
    assert_eq!(fb.temperature, -1);
    assert!((fb.torque_current + 1.0).abs() < 1e-6);
    assert_eq!(fb.shaft_speed, -1000);
    assert_eq!(fb.angle, -1);
}

#[test]
fn motor_status1_decodes_voltage_brake_and_error_flags() {
    // temp 25, brake released, 24.0 V (raw 240), error 0x1234
    let s = decode_motor_status1(&[0x9A, 25, 0x00, 0x01, 0xF0, 0x00, 0x34, 0x12]).unwrap();
    assert_eq!(s.temperature, 25);
    assert!(s.brake_released);
    assert!((s.voltage - 24.0).abs() < 1e-6);
    assert_eq!(s.error_state, 0x1234);
    // data[3] == 0 means the brake is engaged.
    let locked = decode_motor_status1(&[0x9A, 25, 0x00, 0x00, 0xF0, 0x00, 0, 0]).unwrap();
    assert!(!locked.brake_released);
}

#[test]
fn motor_status3_decodes_three_signed_phase_currents() {
    // A = +1.00 A (100), B = -1.00 A (-100), C = +0.01 A (1)
    let s = decode_motor_status3(&[0x9D, 40, 0x64, 0x00, 0x9C, 0xFF, 0x01, 0x00]).unwrap();
    assert_eq!(s.temperature, 40);
    assert!((s.phase_a_current - 1.0).abs() < 1e-6);
    assert!((s.phase_b_current + 1.0).abs() < 1e-6);
    assert!((s.phase_c_current - 0.01).abs() < 1e-6);
}

#[test]
fn multi_turn_angle_and_acceleration_share_the_i32_at_byte_four() {
    // 9000 raw → 90.00 deg
    assert!((decode_multi_turn_angle(&[0x92, 0, 0, 0, 0x28, 0x23, 0, 0]).unwrap() - 90.0) < 1e-3);
    // Negative multi-turn angle (-90.00 deg = -9000 = 0xFFFFDCD8)
    assert!(
        (decode_multi_turn_angle(&[0x92, 0, 0, 0, 0xD8, 0xDC, 0xFF, 0xFF]).unwrap() + 90.0).abs()
            < 1e-3
    );
    // Acceleration is the same field, unscaled.
    assert_eq!(
        decode_acceleration(&[0x42, 0, 0, 0, 0x10, 0x27, 0x00, 0x00]).unwrap(),
        10_000
    );
    assert_eq!(
        decode_acceleration(&[0x42, 0, 0, 0, 0xF0, 0xD8, 0xFF, 0xFF]).unwrap(),
        -10_000
    );
}

#[test]
fn operating_mode_maps_the_three_documented_codes_and_rejects_the_rest() {
    for (byte, want) in [
        (0x01, ControlMode::Current),
        (0x02, ControlMode::Speed),
        (0x03, ControlMode::Position),
    ] {
        assert_eq!(
            decode_operating_mode(&[0x70, 0, 0, 0, 0, 0, 0, byte]).unwrap(),
            want
        );
    }
    for byte in [0x00, 0x04, 0xFF] {
        assert_eq!(
            decode_operating_mode(&[0x70, 0, 0, 0, 0, 0, 0, byte]),
            Err(Error::ValueOutOfRange)
        );
    }
}

#[test]
fn motor_power_and_version_date_decode_their_unsigned_fields() {
    // 25.0 W = raw 250 at data[6..8]
    assert!((decode_motor_power(&[0x71, 0, 0, 0, 0, 0, 0xFA, 0x00]).unwrap() - 25.0).abs() < 1e-3);
    // 20231201 = 0x0134B421
    assert_eq!(
        decode_version_date(&[0xB2, 0, 0, 0, 0x21, 0xB4, 0x34, 0x01]).unwrap(),
        20_231_201
    );
}

// ---------------------------------------------------------------------------
// Frame helpers / errors
// ---------------------------------------------------------------------------

#[test]
fn little_endian_helpers_round_trip_at_every_offset_they_are_used_at() {
    let mut f = CanFrame::new(ID, CommandType::SpeedClosedLoopControl).unwrap();
    f.write_u16(2, 0xBEEF);
    assert_eq!(CanFrame::read_u16(&f.data, 2), 0xBEEF);
    f.write_i16(2, -2);
    assert_eq!(CanFrame::read_i16(&f.data, 2), -2);
    f.write_i32(4, i32::MIN);
    assert_eq!(CanFrame::read_i32(&f.data, 4), i32::MIN);
    f.write_u32(4, u32::MAX);
    assert_eq!(CanFrame::read_u32(&f.data, 4), u32::MAX);
}

#[test]
fn expect_command_is_the_single_gate_every_decoder_goes_through() {
    let data = [0x92u8, 0, 0, 0, 0, 0, 0, 0];
    assert!(expect_command(&data, CommandType::ReadMultiTurnAngle).is_ok());
    assert_eq!(
        expect_command(&data, CommandType::ReadMotorStatus2),
        Err(Error::UnexpectedCommand {
            expected: 0x9C,
            actual: 0x92
        })
    );
}

#[test]
fn errors_render_the_offending_bytes_in_hex() {
    let e = Error::UnexpectedCommand {
        expected: 0xA1,
        actual: 0x9C,
    };
    let s = format!("{e}");
    assert!(s.contains("0xa1") && s.contains("0x9c"), "{s}");
    assert_eq!(format!("{}", Error::ValueOutOfRange), "value out of range");
}
