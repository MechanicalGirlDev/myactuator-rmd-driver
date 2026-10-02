//! Command/response transaction handling.
//!
//! Combines `myactuator-rmd` encoding/decoding with [`CanIo`] and provides high-level methods.

use crate::can::CanIo;
use crate::error::{CanError, Result};
use myactuator_rmd::{
    AccelerationType, CanBaudRate, CanFrame, CommandType, ControlMode, Feedback, Gains,
    MotorStatus1, MotorStatus3, decode_acceleration, decode_feedback, decode_gains,
    decode_motor_power, decode_motor_status1, decode_motor_status3, decode_multi_turn_angle,
    decode_operating_mode, decode_version_date, encode_lock_brake,
    encode_position_absolute_setpoint, encode_read_acceleration, encode_read_gains,
    encode_read_motor_power, encode_read_motor_status1, encode_read_motor_status2,
    encode_read_motor_status3, encode_read_multi_turn_angle, encode_read_operating_mode,
    encode_read_version_date, encode_release_brake, encode_reset, encode_set_acceleration,
    encode_set_baud_rate, encode_set_can_id, encode_shutdown_motor, encode_stop_motor,
    encode_torque_setpoint, encode_velocity_setpoint, encode_write_gains_ram,
    encode_write_gains_rom,
};

/// A transaction with an RMD actuator.
#[derive(Debug)]
pub struct RmdTransaction<T: CanIo> {
    io: T,
    verbose: bool,
}

impl<T: CanIo> RmdTransaction<T> {
    /// Create a new transaction.
    pub fn new(io: T) -> Self {
        Self { io, verbose: false }
    }

    /// Enable or disable verbose output.
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Send a frame and receive an 8-byte response with the same CAN ID.
    fn execute(&mut self, frame: CanFrame) -> Result<[u8; 8]> {
        if self.verbose {
            eprintln!("TX: id=0x{:X} data={:02X?}", frame.can_id, frame.data);
        }
        self.io.send_frame(frame.can_id, &frame.data)?;
        let (resp_id, data) = self.io.recv_frame()?;
        if resp_id != frame.can_id {
            return Err(CanError::UnexpectedCanId {
                expected: frame.can_id,
                actual: resp_id,
            });
        }
        if self.verbose {
            eprintln!("RX: id=0x{resp_id:X} data={data:02X?}");
        }
        Ok(data)
    }

    // ===== Control =====

    /// Closed-loop torque (current) control. `amp` is in amperes.
    pub fn set_torque(&mut self, id: u8, amp: f32) -> Result<Feedback> {
        let resp = self.execute(encode_torque_setpoint(id, amp)?)?;
        Ok(decode_feedback(
            &resp,
            CommandType::TorqueClosedLoopControl,
        )?)
    }

    /// Closed-loop velocity control. `dps` is in degrees per second.
    pub fn set_velocity(&mut self, id: u8, dps: f32) -> Result<Feedback> {
        let resp = self.execute(encode_velocity_setpoint(id, dps)?)?;
        Ok(decode_feedback(&resp, CommandType::SpeedClosedLoopControl)?)
    }

    /// Closed-loop absolute position control. `deg` is degrees; `max_speed` is degrees per second.
    pub fn set_position(&mut self, id: u8, deg: f32, max_speed: u16) -> Result<Feedback> {
        let resp = self.execute(encode_position_absolute_setpoint(id, deg, max_speed)?)?;
        Ok(decode_feedback(
            &resp,
            CommandType::AbsolutePositionClosedLoopControl,
        )?)
    }

    // ===== Read =====

    /// Read the multi-turn angle in degrees.
    pub fn read_multi_turn_angle(&mut self, id: u8) -> Result<f32> {
        let resp = self.execute(encode_read_multi_turn_angle(id)?)?;
        Ok(decode_multi_turn_angle(&resp)?)
    }

    /// Read motor status 1 (temperature, voltage, brake, and errors).
    pub fn read_motor_status1(&mut self, id: u8) -> Result<MotorStatus1> {
        let resp = self.execute(encode_read_motor_status1(id)?)?;
        Ok(decode_motor_status1(&resp)?)
    }

    /// Read motor status 2 (feedback: temperature, current, velocity, and angle).
    pub fn read_motor_status2(&mut self, id: u8) -> Result<Feedback> {
        let resp = self.execute(encode_read_motor_status2(id)?)?;
        Ok(decode_feedback(&resp, CommandType::ReadMotorStatus2)?)
    }

    /// Read motor status 3 (temperature and three-phase current).
    pub fn read_motor_status3(&mut self, id: u8) -> Result<MotorStatus3> {
        let resp = self.execute(encode_read_motor_status3(id)?)?;
        Ok(decode_motor_status3(&resp)?)
    }

    /// Read PID gains.
    pub fn read_gains(&mut self, id: u8) -> Result<Gains> {
        let resp = self.execute(encode_read_gains(id)?)?;
        Ok(decode_gains(&resp)?)
    }

    /// Read motor power in watts.
    pub fn read_power(&mut self, id: u8) -> Result<f32> {
        let resp = self.execute(encode_read_motor_power(id)?)?;
        Ok(decode_motor_power(&resp)?)
    }

    /// Read the software version date (`YYYYMMDD`).
    pub fn read_version(&mut self, id: u8) -> Result<u32> {
        let resp = self.execute(encode_read_version_date(id)?)?;
        Ok(decode_version_date(&resp)?)
    }

    /// Read the operating mode.
    pub fn read_operating_mode(&mut self, id: u8) -> Result<ControlMode> {
        let resp = self.execute(encode_read_operating_mode(id)?)?;
        Ok(decode_operating_mode(&resp)?)
    }

    /// Read acceleration in degrees per second.
    pub fn read_acceleration(&mut self, id: u8) -> Result<i32> {
        let resp = self.execute(encode_read_acceleration(id)?)?;
        Ok(decode_acceleration(&resp)?)
    }

    // ===== System =====
    //
    // Stop, shutdown, release_brake, and lock_brake return an echoed command frame,
    // which `execute` receives and discards. Reset does not reply, so it only sends.

    /// Stop the motor (`0x81`) and discard the echoed response.
    pub fn stop(&mut self, id: u8) -> Result<()> {
        let _ = self.execute(encode_stop_motor(id)?)?;
        Ok(())
    }

    /// Shut down the motor (`0x80`) and discard the echoed response.
    pub fn shutdown(&mut self, id: u8) -> Result<()> {
        let _ = self.execute(encode_shutdown_motor(id)?)?;
        Ok(())
    }

    /// Reset the motor (`0x76`) without waiting for a response.
    pub fn reset(&mut self, id: u8) -> Result<()> {
        let frame = encode_reset(id)?;
        self.io.send_frame(frame.can_id, &frame.data)
    }

    /// Release the brake (`0x77`) and discard the echoed response.
    pub fn release_brake(&mut self, id: u8) -> Result<()> {
        let _ = self.execute(encode_release_brake(id)?)?;
        Ok(())
    }

    /// Lock the brake (`0x78`) and discard the echoed response.
    pub fn lock_brake(&mut self, id: u8) -> Result<()> {
        let _ = self.execute(encode_lock_brake(id)?)?;
        Ok(())
    }

    // ===== Configuration =====

    /// Write PID gains to RAM (`0x31`) and discard the response.
    pub fn write_gains_ram(&mut self, id: u8, gains: Gains) -> Result<()> {
        let _ = self.execute(encode_write_gains_ram(id, gains)?)?;
        Ok(())
    }

    /// Write PID gains to ROM (`0x32`) and discard the response.
    pub fn write_gains_rom(&mut self, id: u8, gains: Gains) -> Result<()> {
        let _ = self.execute(encode_write_gains_rom(id, gains)?)?;
        Ok(())
    }

    /// Set acceleration (`0x43`). `accel` is in degrees per second; `mode` selects the target.
    pub fn set_acceleration(&mut self, id: u8, accel: u32, mode: AccelerationType) -> Result<()> {
        let _ = self.execute(encode_set_acceleration(id, accel, mode)?)?;
        Ok(())
    }

    /// Set the CAN ID (`0x79`) without waiting for a response because the communication ID changes.
    pub fn set_can_id(&mut self, id: u8, new_can_id: u8) -> Result<()> {
        let frame = encode_set_can_id(id, new_can_id)?;
        self.io.send_frame(frame.can_id, &frame.data)
    }

    /// Set the communication baud rate (`0xB4`) without waiting for a response because the rate changes.
    pub fn set_baud_rate(&mut self, id: u8, baud: CanBaudRate) -> Result<()> {
        let frame = encode_set_baud_rate(id, baud)?;
        self.io.send_frame(frame.can_id, &frame.data)
    }

    // ===== Utilities =====

    /// Check whether a motor responds (a timeout means it is absent).
    pub fn ping(&mut self, id: u8) -> Result<bool> {
        match self.read_multi_turn_angle(id) {
            Ok(_) => Ok(true),
            Err(CanError::Timeout) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Return the IDs of responding motors in `start..=end`.
    ///
    /// Drain the receive buffer before each probe so delayed responses or stale bus frames
    /// from the previous motor cannot affect the next ID.
    pub fn scan(&mut self, start: u8, end: u8) -> Result<Vec<u8>> {
        let mut found = Vec::new();
        for id in start..=end {
            if !(1..=32).contains(&id) {
                continue;
            }
            self.io.drain()?;
            if self.ping(id)? {
                found.push(id);
            }
        }
        Ok(found)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use alloc::collections::VecDeque;

    /// Mock that records sent frames and returns preconfigured responses.
    struct MockCanIo {
        sent: Vec<(u16, [u8; 8])>,
        responses: VecDeque<Result<(u16, [u8; 8])>>,
    }

    impl MockCanIo {
        fn new(responses: Vec<Result<(u16, [u8; 8])>>) -> Self {
            Self {
                sent: Vec::new(),
                responses: responses.into_iter().collect(),
            }
        }
    }

    impl CanIo for MockCanIo {
        fn send_frame(&mut self, can_id: u16, data: &[u8]) -> Result<()> {
            let mut d = [0u8; 8];
            d[..data.len()].copy_from_slice(data);
            self.sent.push((can_id, d));
            Ok(())
        }
        fn recv_frame(&mut self) -> Result<(u16, [u8; 8])> {
            match self.responses.pop_front() {
                Some(r) => r,
                None => Err(CanError::Timeout),
            }
        }
        fn drain(&mut self) -> Result<()> {
            Ok(())
        }
    }

    #[test]
    fn set_torque_sends_golden_and_decodes_feedback() {
        // Response: temperature 25, raw current 100 (= 1.0 A).
        let mock = MockCanIo::new(vec![Ok((0x141, [0xA1, 25, 0x64, 0x00, 0, 0, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        let fb = tx.set_torque(1, 1.0).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0xA1, 0, 0, 0, 0x64, 0, 0, 0]));
        assert_eq!(fb.temperature, 25);
        assert!((fb.torque_current - 1.0).abs() < 1e-6);
    }

    #[test]
    fn set_position_sends_golden_frame() {
        let mock = MockCanIo::new(vec![Ok((0x141, [0xA4, 0, 0, 0, 0, 0, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        let _ = tx.set_position(1, 90.0, 500).unwrap();
        assert_eq!(
            tx.io.sent[0],
            (0x141, [0xA4, 0, 0xF4, 0x01, 0x28, 0x23, 0, 0])
        );
    }

    #[test]
    fn unexpected_can_id_errors() {
        let mock = MockCanIo::new(vec![Ok((0x142, [0x92, 0, 0, 0, 0, 0, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        let err = tx.read_multi_turn_angle(1).unwrap_err();
        assert!(matches!(
            err,
            CanError::UnexpectedCanId {
                expected: 0x141,
                actual: 0x142
            }
        ));
    }

    #[test]
    fn ping_timeout_is_absent() {
        let mock = MockCanIo::new(vec![Err(CanError::Timeout)]);
        let mut tx = RmdTransaction::new(mock);
        assert!(!tx.ping(1).unwrap());
    }

    #[test]
    fn scan_collects_responders() {
        // ID 1 responds, ID 2 times out, and ID 3 responds.
        let mock = MockCanIo::new(vec![
            Ok((0x141, [0x92, 0, 0, 0, 0x28, 0x23, 0, 0])),
            Err(CanError::Timeout),
            Ok((0x143, [0x92, 0, 0, 0, 0x28, 0x23, 0, 0])),
        ]);
        let mut tx = RmdTransaction::new(mock);
        let found = tx.scan(1, 3).unwrap();
        assert_eq!(found, vec![1, 3]);
    }

    #[test]
    fn write_gains_ram_sends_golden() {
        use myactuator_rmd::{Gains, PiGains};
        let mock = MockCanIo::new(vec![Ok((0x141, [0x31, 0, 10, 20, 30, 40, 50, 60]))]);
        let mut tx = RmdTransaction::new(mock);
        let gains = Gains {
            current: PiGains { kp: 10, ki: 20 },
            speed: PiGains { kp: 30, ki: 40 },
            position: PiGains { kp: 50, ki: 60 },
        };
        tx.write_gains_ram(1, gains).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0x31, 0, 10, 20, 30, 40, 50, 60]));
    }

    #[test]
    fn set_can_id_is_send_only() {
        // Succeeds with an empty response queue because this command only sends.
        let mock = MockCanIo::new(vec![]);
        let mut tx = RmdTransaction::new(mock);
        tx.set_can_id(1, 2).unwrap();
        assert_eq!(tx.io.sent[0].0, 0x141);
        // encode_set_can_id: data[6]=can_id (C++ protocol specification).
        assert_eq!(tx.io.sent[0].1[6], 2);
    }

    /// One responder that echoes the command byte back, which is what the RMD does for
    /// every command that has a reply.
    fn echo(cmd: u8) -> Result<(u16, [u8; 8])> {
        Ok((0x141, [cmd, 0, 0, 0, 0, 0, 0, 0]))
    }

    #[test]
    fn set_velocity_sends_golden_and_decodes_feedback() {
        // 10 dps → raw 1000; reply carries temp 20 / speed 1000.
        let mock = MockCanIo::new(vec![Ok((0x141, [0xA2, 20, 0, 0, 0xE8, 0x03, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        let fb = tx.set_velocity(1, 10.0).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0xA2, 0, 0, 0, 0xE8, 0x03, 0, 0]));
        assert_eq!(fb.temperature, 20);
        assert_eq!(fb.shaft_speed, 1000);
    }

    #[test]
    fn every_read_command_sends_its_id_and_decodes_its_reply() {
        // multi-turn angle: raw 9000 → 90.00 deg
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x92, 0, 0, 0, 0x28, 0x23, 0, 0],
        ))]));
        assert!((tx.read_multi_turn_angle(1).unwrap() - 90.0).abs() < 1e-3);
        assert_eq!(tx.io.sent[0].1[0], 0x92);

        // status1: 24.0 V, brake released
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x9A, 25, 0, 1, 0xF0, 0, 0, 0],
        ))]));
        let s1 = tx.read_motor_status1(1).unwrap();
        assert_eq!(tx.io.sent[0].1[0], 0x9A);
        assert!(s1.brake_released);
        assert!((s1.voltage - 24.0).abs() < 1e-6);

        // status2 shares the Feedback layout
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x9C, 30, 0x64, 0, 0, 0, 0, 0],
        ))]));
        let s2 = tx.read_motor_status2(1).unwrap();
        assert_eq!(tx.io.sent[0].1[0], 0x9C);
        assert_eq!(s2.temperature, 30);

        // status3: three phase currents
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x9D, 40, 0x64, 0, 0x9C, 0xFF, 0, 0],
        ))]));
        let s3 = tx.read_motor_status3(1).unwrap();
        assert_eq!(tx.io.sent[0].1[0], 0x9D);
        assert!((s3.phase_a_current - 1.0).abs() < 1e-6);
        assert!((s3.phase_b_current + 1.0).abs() < 1e-6);

        // gains
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x30, 0, 10, 20, 30, 40, 50, 60],
        ))]));
        let g = tx.read_gains(1).unwrap();
        assert_eq!(tx.io.sent[0].1[0], 0x30);
        assert_eq!(g.position.ki, 60);

        // power: raw 250 → 25.0 W
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x71, 0, 0, 0, 0, 0, 0xFA, 0],
        ))]));
        assert!((tx.read_power(1).unwrap() - 25.0).abs() < 1e-3);
        assert_eq!(tx.io.sent[0].1[0], 0x71);

        // version date
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0xB2, 0, 0, 0, 0x21, 0xB4, 0x34, 0x01],
        ))]));
        assert_eq!(tx.read_version(1).unwrap(), 20_231_201);
        assert_eq!(tx.io.sent[0].1[0], 0xB2);

        // operating mode
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x70, 0, 0, 0, 0, 0, 0, 3],
        ))]));
        assert_eq!(tx.read_operating_mode(1).unwrap(), ControlMode::Position);
        assert_eq!(tx.io.sent[0].1[0], 0x70);

        // acceleration
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![Ok((
            0x141,
            [0x42, 0, 0, 0, 0x10, 0x27, 0, 0],
        ))]));
        assert_eq!(tx.read_acceleration(1).unwrap(), 10_000);
        assert_eq!(tx.io.sent[0].1[0], 0x42);
    }

    #[test]
    fn system_commands_consume_the_echo_reply() {
        // stop / shutdown / release_brake / lock_brake all wait for the echo.
        for (cmd, call) in [(0x81u8, 0usize), (0x80, 1), (0x77, 2), (0x78, 3)] {
            let mut tx = RmdTransaction::new(MockCanIo::new(vec![echo(cmd)]));
            match call {
                0 => tx.stop(1).unwrap(),
                1 => tx.shutdown(1).unwrap(),
                2 => tx.release_brake(1).unwrap(),
                _ => tx.lock_brake(1).unwrap(),
            }
            assert_eq!(tx.io.sent[0], (0x141, [cmd, 0, 0, 0, 0, 0, 0, 0]));
        }
    }

    #[test]
    fn reset_and_set_baud_rate_do_not_wait_for_a_reply() {
        // Both change the link out from under the answer, so an empty response queue
        // (which `recv_frame` turns into a Timeout) must not fail them.
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![]));
        tx.reset(1).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0x76, 0, 0, 0, 0, 0, 0, 0]));

        let mut tx = RmdTransaction::new(MockCanIo::new(vec![]));
        tx.set_baud_rate(1, CanBaudRate::Bps1M).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0xB4, 0, 0, 0, 0, 0, 0, 0x01]));
    }

    #[test]
    fn write_gains_rom_and_set_acceleration_send_golden_frames() {
        use myactuator_rmd::{Gains, PiGains};
        let gains = Gains {
            current: PiGains { kp: 1, ki: 2 },
            speed: PiGains { kp: 3, ki: 4 },
            position: PiGains { kp: 5, ki: 6 },
        };
        let mut tx = RmdTransaction::new(MockCanIo::new(vec![echo(0x32)]));
        tx.write_gains_rom(1, gains).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0x32, 0, 1, 2, 3, 4, 5, 6]));

        let mut tx = RmdTransaction::new(MockCanIo::new(vec![echo(0x43)]));
        tx.set_acceleration(1, 10_000, AccelerationType::SpeedPlanningAcceleration)
            .unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0x43, 0x02, 0, 0, 0x10, 0x27, 0, 0]));
    }

    #[test]
    fn ping_propagates_errors_that_are_not_a_timeout() {
        // A wrong-id reply means the bus is confused, not that the motor is absent —
        // reporting it as "not present" would hide a real wiring fault during a scan.
        let mock = MockCanIo::new(vec![Ok((0x142, [0x92, 0, 0, 0, 0, 0, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        assert!(tx.ping(1).is_err());
    }

    #[test]
    fn scan_skips_ids_outside_the_protocol_range_without_probing_them() {
        // 0 and 33 are not addressable, so they must not consume a response slot.
        let mock = MockCanIo::new(vec![Ok((0x141, [0x92, 0, 0, 0, 0x28, 0x23, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        let found = tx.scan(0, 1).unwrap();
        assert_eq!(found, vec![1]);
        assert_eq!(tx.io.sent.len(), 1, "id 0 must not be probed");
    }

    #[test]
    fn verbose_logging_does_not_change_the_wire() {
        let mock = MockCanIo::new(vec![Ok((0x141, [0xA1, 0, 0, 0, 0, 0, 0, 0]))]);
        let mut tx = RmdTransaction::new(mock);
        tx.set_verbose(true);
        let _ = tx.set_torque(1, 0.0).unwrap();
        assert_eq!(tx.io.sent[0], (0x141, [0xA1, 0, 0, 0, 0, 0, 0, 0]));
    }
}
