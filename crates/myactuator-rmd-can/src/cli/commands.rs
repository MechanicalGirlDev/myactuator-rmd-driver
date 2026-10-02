//! Command handlers that print the result of one `RmdTransaction` call.

use crate::can::{CanIo, RmdTransaction};
use crate::cli::args::{ReadCommands, SystemCommands, WriteCommands};
use crate::error::Result;
use myactuator_rmd::{Gains, PiGains};

/// Run a motor scan.
///
/// Frame-level traces are printed by `RmdTransaction::set_verbose`, so this handler
/// prints only the results and does not take a verbose argument.
pub fn execute_scan<T: CanIo>(tx: &mut RmdTransaction<T>, start: u8, end: u8) -> Result<()> {
    println!("Scanning IDs {start} through {end}...");
    let found = tx.scan(start, end)?;
    if found.is_empty() {
        println!("No motors found");
    } else {
        println!("Motors found:");
        for id in &found {
            println!("  - ID: {id}");
        }
        println!("Total: {}", found.len());
    }
    Ok(())
}

/// Execute absolute position control.
pub fn execute_position<T: CanIo>(
    tx: &mut RmdTransaction<T>,
    id: u8,
    deg: f32,
    max_speed: u16,
) -> Result<()> {
    let fb = tx.set_position(id, deg, max_speed)?;
    println!("{fb:?}");
    Ok(())
}

/// Execute velocity control.
pub fn execute_velocity<T: CanIo>(tx: &mut RmdTransaction<T>, id: u8, dps: f32) -> Result<()> {
    let fb = tx.set_velocity(id, dps)?;
    println!("{fb:?}");
    Ok(())
}

/// Execute torque (current) control.
pub fn execute_torque<T: CanIo>(tx: &mut RmdTransaction<T>, id: u8, amp: f32) -> Result<()> {
    let fb = tx.set_torque(id, amp)?;
    println!("{fb:?}");
    Ok(())
}

/// Execute a read command.
pub fn execute_read<T: CanIo>(tx: &mut RmdTransaction<T>, cmd: &ReadCommands) -> Result<()> {
    match cmd {
        ReadCommands::Angle { id } => {
            let v = tx.read_multi_turn_angle(*id)?;
            println!("Angle: {v:.2} deg");
        }
        ReadCommands::Status1 { id } => {
            let s = tx.read_motor_status1(*id)?;
            println!("Status1: {s:?}");
        }
        ReadCommands::Status2 { id } => {
            let s = tx.read_motor_status2(*id)?;
            println!("Status2: {s:?}");
        }
        ReadCommands::Status3 { id } => {
            let s = tx.read_motor_status3(*id)?;
            println!("Status3: {s:?}");
        }
        ReadCommands::Gains { id } => {
            let g = tx.read_gains(*id)?;
            println!("Gains: {g:?}");
        }
        ReadCommands::Power { id } => {
            let v = tx.read_power(*id)?;
            println!("Power: {v:.1} W");
        }
        ReadCommands::Version { id } => {
            let v = tx.read_version(*id)?;
            println!("Version date: {v:08}");
        }
        ReadCommands::Mode { id } => {
            let m = tx.read_operating_mode(*id)?;
            println!("Operating mode: {m:?}");
        }
        ReadCommands::Accel { id } => {
            let v = tx.read_acceleration(*id)?;
            println!("Acceleration: {v} dps");
        }
    }
    Ok(())
}

/// Execute a system command.
pub fn execute_system<T: CanIo>(tx: &mut RmdTransaction<T>, cmd: &SystemCommands) -> Result<()> {
    match cmd {
        SystemCommands::Stop { id } => {
            tx.stop(*id)?;
            println!("Stopped motor ID {id}");
        }
        SystemCommands::Shutdown { id } => {
            tx.shutdown(*id)?;
            println!("Shut down motor ID {id}");
        }
        SystemCommands::Reset { id } => {
            tx.reset(*id)?;
            println!("Reset motor ID {id}");
        }
        SystemCommands::ReleaseBrake { id } => {
            tx.release_brake(*id)?;
            println!("Released brake for motor ID {id}");
        }
        SystemCommands::LockBrake { id } => {
            tx.lock_brake(*id)?;
            println!("Locked brake for motor ID {id}");
        }
    }
    Ok(())
}

/// Execute a configuration write.
pub fn execute_write<T: CanIo>(tx: &mut RmdTransaction<T>, cmd: &WriteCommands) -> Result<()> {
    match cmd {
        WriteCommands::Gains {
            id,
            current_kp,
            current_ki,
            speed_kp,
            speed_ki,
            position_kp,
            position_ki,
            rom,
        } => {
            let gains = Gains {
                current: PiGains {
                    kp: *current_kp,
                    ki: *current_ki,
                },
                speed: PiGains {
                    kp: *speed_kp,
                    ki: *speed_ki,
                },
                position: PiGains {
                    kp: *position_kp,
                    ki: *position_ki,
                },
            };
            if *rom {
                tx.write_gains_rom(*id, gains)?;
                println!("Wrote PID gains to ROM for motor ID {id}");
            } else {
                tx.write_gains_ram(*id, gains)?;
                println!("Wrote PID gains to RAM for motor ID {id}");
            }
        }
        WriteCommands::Accel { id, value, mode } => {
            tx.set_acceleration(*id, *value, (*mode).into())?;
            println!("Wrote acceleration for motor ID {id}: {value} dps, {mode:?}");
        }
        WriteCommands::CanId { id, new_id } => {
            tx.set_can_id(*id, *new_id)?;
            println!("Sent CAN ID change only: {id} -> {new_id}");
        }
        WriteCommands::Baud { id, baud } => {
            tx.set_baud_rate(*id, (*baud).into())?;
            println!("Sent baud rate change only for motor ID {id}: {baud:?}");
        }
    }
    Ok(())
}
