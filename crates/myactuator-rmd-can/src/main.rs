//! MyActuator RMD-X control CLI (SLCAN).

use clap::Parser;
use core::time::Duration;
use myactuator_rmd_can::can::{RmdTransaction, SlcanPort};
use myactuator_rmd_can::cli::Cli;
use myactuator_rmd_can::cli::args::Commands;
use myactuator_rmd_can::cli::commands::{
    execute_position, execute_read, execute_scan, execute_system, execute_torque, execute_velocity,
    execute_write,
};
use myactuator_rmd_can::error::Result;

fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();

    let timeout = Duration::from_millis(cli.timeout);
    let port = SlcanPort::open(&cli.port, cli.serial_baud, cli.bitrate.into(), timeout)?;

    let mut tx = RmdTransaction::new(port);
    tx.set_verbose(cli.verbose);

    match &cli.command {
        Commands::Scan { start, end } => execute_scan(&mut tx, *start, *end),
        Commands::Read(cmd) => execute_read(&mut tx, cmd),
        Commands::Position { id, deg, max_speed } => {
            execute_position(&mut tx, *id, *deg, *max_speed)
        }
        Commands::Velocity { id, dps } => execute_velocity(&mut tx, *id, *dps),
        Commands::Torque { id, amp } => execute_torque(&mut tx, *id, *amp),
        Commands::Write(cmd) => execute_write(&mut tx, cmd),
        Commands::System(cmd) => execute_system(&mut tx, cmd),
    }
}
