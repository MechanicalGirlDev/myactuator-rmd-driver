//! Error types corresponding to the `kondo-ics-rs485` `CliError`.

use thiserror::Error;

/// Error type for `myactuator-rmd-can`.
#[derive(Error, Debug)]
pub enum CanError {
    /// Serial port error.
    #[error("serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// RMD protocol error.
    #[error("RMD protocol error: {0}")]
    Protocol(myactuator_rmd::Error),

    /// No response before the timeout.
    #[error("timeout: no response")]
    Timeout,

    /// SLCAN frame parsing error.
    #[error("SLCAN frame parsing error: {0}")]
    SlcanParse(String),

    /// Response from an unexpected CAN ID.
    #[error("unexpected CAN ID: expected 0x{expected:X}, received 0x{actual:X}")]
    UnexpectedCanId {
        /// Expected CAN ID.
        expected: u16,
        /// Actual CAN ID received.
        actual: u16,
    },
}

impl From<myactuator_rmd::Error> for CanError {
    fn from(err: myactuator_rmd::Error) -> Self {
        Self::Protocol(err)
    }
}

/// The `Result` type alias for this crate.
pub type Result<T> = core::result::Result<T, CanError>;
