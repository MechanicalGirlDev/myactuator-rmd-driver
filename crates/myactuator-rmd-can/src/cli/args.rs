//! CLI argument definitions.

use crate::can::CanBitrate;
use clap::{Parser, Subcommand, ValueEnum};

/// MyActuator RMD-X control CLI (SLCAN).
#[derive(Parser, Debug)]
#[command(name = "myactuator-rmd")]
#[command(author, version, about = "MyActuator RMD-X control CLI (SLCAN)", long_about = None)]
pub struct Cli {
    /// Serial port (for example, Windows `COM3` or Linux `/dev/ttyACM0`).
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    pub port: String,

    /// USB serial baud rate.
    #[arg(long, default_value_t = 115200)]
    pub serial_baud: u32,

    /// CAN bit rate.
    #[arg(long, value_enum, default_value_t = Bitrate::M1)]
    pub bitrate: Bitrate,

    /// Timeout in milliseconds.
    #[arg(short, long, default_value_t = 100)]
    pub timeout: u64,

    /// Enable verbose output.
    #[arg(short, long)]
    pub verbose: bool,

    /// Subcommand.
    #[command(subcommand)]
    pub command: Commands,
}

/// CAN bit rate for the CLI.
#[derive(ValueEnum, Clone, Copy, Debug)]
pub enum Bitrate {
    /// 1 Mbps
    #[value(name = "1m")]
    M1,
    /// 500 kbps
    #[value(name = "500k")]
    K500,
    /// 250 kbps
    #[value(name = "250k")]
    K250,
    /// 125 kbps
    #[value(name = "125k")]
    K125,
}

impl From<Bitrate> for CanBitrate {
    fn from(b: Bitrate) -> Self {
        match b {
            Bitrate::M1 => CanBitrate::Bps1M,
            Bitrate::K500 => CanBitrate::Bps500k,
            Bitrate::K250 => CanBitrate::Bps250k,
            Bitrate::K125 => CanBitrate::Bps125k,
        }
    }
}

/// CLI subcommands.
#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Scan for motors.
    Scan {
        /// First motor ID.
        #[arg(long, default_value_t = 1)]
        start: u8,
        /// Last motor ID.
        #[arg(long, default_value_t = 32)]
        end: u8,
    },

    /// Read motor data.
    #[command(subcommand)]
    Read(ReadCommands),

    /// Control absolute position.
    Position {
        /// Motor ID (1-32).
        id: u8,
        /// Target angle [deg].
        deg: f32,
        /// Maximum speed [dps].
        #[arg(long, default_value_t = 500)]
        max_speed: u16,
    },

    /// Control velocity.
    Velocity {
        /// Motor ID (1-32).
        id: u8,
        /// Velocity [dps].
        dps: f32,
    },

    /// Control torque (current).
    Torque {
        /// Motor ID (1-32).
        id: u8,
        /// Current [A].
        amp: f32,
    },

    /// Write settings.
    #[command(subcommand)]
    Write(WriteCommands),

    /// Send a system command.
    #[command(subcommand)]
    System(SystemCommands),
}

/// Read subcommands.
#[derive(Subcommand, Debug)]
pub enum ReadCommands {
    /// Multi-turn angle [deg].
    Angle {
        /// Motor ID.
        id: u8,
    },
    /// Motor status 1 (temperature, voltage, brake, and errors).
    Status1 {
        /// Motor ID.
        id: u8,
    },
    /// Motor status 2 (temperature, current, velocity, and angle).
    Status2 {
        /// Motor ID.
        id: u8,
    },
    /// Motor status 3 (temperature and three-phase current).
    Status3 {
        /// Motor ID.
        id: u8,
    },
    /// PID gains.
    Gains {
        /// Motor ID.
        id: u8,
    },
    /// Motor power [W].
    Power {
        /// Motor ID.
        id: u8,
    },
    /// Software version date.
    Version {
        /// Motor ID.
        id: u8,
    },
    /// Operating mode.
    Mode {
        /// Motor ID.
        id: u8,
    },
    /// Acceleration [dps].
    Accel {
        /// Motor ID.
        id: u8,
    },
}

/// Settings write subcommands.
#[derive(Subcommand, Debug)]
pub enum WriteCommands {
    /// Write PID gains (RAM by default, or ROM with `--rom`).
    Gains {
        /// Motor ID.
        id: u8,
        /// Current loop KP.
        #[arg(long)]
        current_kp: u8,
        /// Current loop KI.
        #[arg(long)]
        current_ki: u8,
        /// Velocity loop KP.
        #[arg(long)]
        speed_kp: u8,
        /// Velocity loop KI.
        #[arg(long)]
        speed_ki: u8,
        /// Position loop KP.
        #[arg(long)]
        position_kp: u8,
        /// Position loop KI.
        #[arg(long)]
        position_ki: u8,
        /// Write to ROM.
        #[arg(long)]
        rom: bool,
    },
    /// Write acceleration.
    Accel {
        /// Motor ID.
        id: u8,
        /// Value [dps].
        value: u32,
        /// Acceleration target.
        #[arg(long, value_enum, default_value_t = AccelMode::PositionAccel)]
        mode: AccelMode,
    },
    /// Set CAN ID (send only).
    CanId {
        /// Current motor ID.
        id: u8,
        /// New CAN ID (1-32).
        new_id: u8,
    },
    /// Set communication baud rate (send only).
    Baud {
        /// Motor ID.
        id: u8,
        /// Baud rate.
        #[arg(value_enum)]
        baud: BaudRate,
    },
}

/// System subcommands.
#[derive(Subcommand, Debug)]
pub enum SystemCommands {
    /// Stop the motor.
    Stop {
        /// Motor ID.
        id: u8,
    },
    /// Shut down the motor.
    Shutdown {
        /// Motor ID.
        id: u8,
    },
    /// Reset the motor.
    Reset {
        /// Motor ID.
        id: u8,
    },
    /// Release the brake.
    ReleaseBrake {
        /// Motor ID.
        id: u8,
    },
    /// Lock the brake.
    LockBrake {
        /// Motor ID.
        id: u8,
    },
}

/// Acceleration setting target for the CLI.
#[derive(ValueEnum, Clone, Copy, Debug)]
pub enum AccelMode {
    /// Position-planning acceleration.
    #[value(name = "position-accel")]
    PositionAccel,
    /// Position-planning deceleration.
    #[value(name = "position-decel")]
    PositionDecel,
    /// Velocity-planning acceleration.
    #[value(name = "speed-accel")]
    SpeedAccel,
    /// Velocity-planning deceleration.
    #[value(name = "speed-decel")]
    SpeedDecel,
}

impl From<AccelMode> for myactuator_rmd::AccelerationType {
    fn from(m: AccelMode) -> Self {
        use myactuator_rmd::AccelerationType as A;
        match m {
            AccelMode::PositionAccel => A::PositionPlanningAcceleration,
            AccelMode::PositionDecel => A::PositionPlanningDeceleration,
            AccelMode::SpeedAccel => A::SpeedPlanningAcceleration,
            AccelMode::SpeedDecel => A::SpeedPlanningDeceleration,
        }
    }
}

/// CAN baud rate available for configuration writes.
#[derive(ValueEnum, Clone, Copy, Debug)]
pub enum BaudRate {
    /// 500 kbps.
    #[value(name = "500k")]
    K500,
    /// 1 Mbps.
    #[value(name = "1m")]
    M1,
}

impl From<BaudRate> for myactuator_rmd::CanBaudRate {
    fn from(b: BaudRate) -> Self {
        match b {
            BaudRate::K500 => myactuator_rmd::CanBaudRate::Bps500k,
            BaudRate::M1 => myactuator_rmd::CanBaudRate::Bps1M,
        }
    }
}
