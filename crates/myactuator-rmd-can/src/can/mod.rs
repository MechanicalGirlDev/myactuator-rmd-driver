//! CAN transport (SLCAN) and transaction handling.

#[cfg(test)]
pub(crate) mod fake;
pub mod slcan;
pub mod transaction;

pub use slcan::{CanBitrate, CanIo, SlcanPort, encode_slcan_frame, parse_slcan_frame};
pub use transaction::RmdTransaction;
