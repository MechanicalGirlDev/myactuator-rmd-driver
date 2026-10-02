//! Integration tests for the public API that do not require hardware.
#![allow(clippy::unwrap_used)]

use myactuator_rmd_can::{encode_slcan_frame, parse_slcan_frame};

#[test]
fn slcan_torque_frame_golden() {
    // CAN ID 0x141, torque control frame.
    let s = encode_slcan_frame(0x141, &[0xA1, 0, 0, 0, 0x64, 0, 0, 0]);
    assert_eq!(s, "t1418A100000064000000\r");
}

#[test]
fn slcan_parse_roundtrip() {
    let data = [0xA4, 0, 0xF4, 0x01, 0x28, 0x23, 0, 0];
    let s = encode_slcan_frame(0x141, &data);
    let (id, parsed) = parse_slcan_frame(s.trim_end_matches('\r')).unwrap();
    assert_eq!(id, 0x141);
    assert_eq!(parsed, data);
}
