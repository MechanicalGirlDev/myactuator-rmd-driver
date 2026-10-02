//! Protocol error types corresponding to the C++ `ProtocolException`.

/// An error that can occur while processing the protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The response frame's command ID does not match the expected value.
    UnexpectedCommand {
        /// Expected command ID.
        expected: u8,
        /// Actual command ID received.
        actual: u8,
    },
    /// A physical value is outside its representable range.
    ValueOutOfRange,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::UnexpectedCommand { expected, actual } => {
                write!(
                    f,
                    "unexpected command: expected {expected:#04x}, got {actual:#04x}"
                )
            }
            Error::ValueOutOfRange => write!(f, "value out of range"),
        }
    }
}

impl core::error::Error for Error {}

/// The `Result` type alias for this crate.
pub type Result<T> = core::result::Result<T, Error>;
