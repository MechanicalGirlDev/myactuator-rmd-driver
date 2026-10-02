//! SLCAN transport layer and CLI for the MyActuator RMD-X series.
//!
//! Wraps the `no_std` `myactuator-rmd` protocol crate for synchronous communication with
//! hardware through a SLCAN-compatible USB-CAN adapter.

extern crate alloc;

pub mod can;
pub mod cli;
pub mod error;

pub use can::{
    CanBitrate, CanIo, RmdTransaction, SlcanPort, encode_slcan_frame, parse_slcan_frame,
};
pub use error::{CanError, Result};
