//! Integration tests for the public API and wire compatibility.
#![allow(clippy::unwrap_used)]

use myactuator_rmd::{
    CommandType, Feedback, decode_feedback, encode_position_absolute_setpoint,
    encode_torque_setpoint,
};

#[test]
fn torque_encode_then_feedback_decode() {
    // Encode current=1.0 A into the golden frame.
    let cmd = encode_torque_setpoint(1, 1.0).unwrap();
    assert_eq!(cmd.can_id, 0x141);
    assert_eq!(cmd.data, [0xA1, 0, 0, 0, 0x64, 0x00, 0, 0]);

    // Decode the matching command response into Feedback.
    let resp = [0xA1, 25, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00];
    let fb: Feedback = decode_feedback(&resp, CommandType::TorqueClosedLoopControl).unwrap();
    assert_eq!(fb.temperature, 25);
    assert!((fb.torque_current - 1.0).abs() < 1e-6);
}

#[test]
fn position_golden_frame() {
    let f = encode_position_absolute_setpoint(1, 90.0, 500).unwrap();
    assert_eq!(f.data, [0xA4, 0, 0xF4, 0x01, 0x28, 0x23, 0x00, 0x00]);
}
