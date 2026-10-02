//! An in-memory [`SerialPort`] carrying SLCAN ASCII lines, for tests.
//!
//! [`SlcanPort`](super::SlcanPort) is the half of the CAN stack that no `CanIo` mock can
//! reach: the ASCII line framing, the adapter's `z`/`Z` send acknowledgements that have to
//! be skipped, the `BEL` error answer, and the `C\r` the `Drop` impl sends to close the
//! channel. Opening a real adapter is the only other way in, so the tests drive it through
//! this instead.
//!
//! Model: bytes queued by [`FakeSerial::new`] are handed out by `read`; once they run out a
//! `read` reports [`io::ErrorKind::TimedOut`], which is what an idle adapter does. Writes
//! are recorded so a test can assert the control commands that went out.

use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::cell::RefCell;
use core::time::Duration;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::sync::Mutex;

/// See the module docs.
#[derive(Debug)]
pub(crate) struct FakeSerial {
    rx: RefCell<VecDeque<u8>>,
    written: Arc<Mutex<Vec<u8>>>,
}

impl FakeSerial {
    /// `incoming` is the byte stream the adapter will produce, verbatim.
    pub(crate) fn new(incoming: &[u8]) -> (Self, Arc<Mutex<Vec<u8>>>) {
        let written = Arc::new(Mutex::new(Vec::new()));
        let port = Self {
            rx: RefCell::new(incoming.iter().copied().collect()),
            written: Arc::clone(&written),
        };
        (port, written)
    }
}

impl Read for FakeSerial {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut rx = self.rx.borrow_mut();
        if rx.is_empty() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "adapter is idle"));
        }
        let n = buf.len().min(rx.len());
        for slot in buf.iter_mut().take(n) {
            *slot = rx.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

impl Write for FakeSerial {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.written
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Settings plumbing the SLCAN layer never reads back; only `clear` / `bytes_to_read`
/// carry behaviour, because `drain` uses both.
impl SerialPort for FakeSerial {
    fn name(&self) -> Option<String> {
        Some("fake".to_string())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(115_200)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(u32::try_from(self.rx.borrow().len()).unwrap_or(u32::MAX))
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        if matches!(buffer_to_clear, ClearBuffer::Input | ClearBuffer::All) {
            self.rx.borrow_mut().clear();
        }
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "the fake adapter is not clonable",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}
