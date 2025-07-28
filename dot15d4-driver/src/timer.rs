//! Time structures.
//!
//! - [`Instant`] is used to represent a point in time.
//! - [`Duration`] is used to represent a duration of time.

use core::future::Future;

use fugit::NanosDurationU64;

pub mod export {
    pub use fugit::{Duration, ExtU64, Instant};
}

use export::*;

/// O-QPSK 250kB/s = 31.25kb/s = 62.5ksymbol/s (1 byte = 8 bit = 2 O-QPSK symbols)
pub const O_QPSK_FREQUENCY: u32 = 62_500;
pub type SymbolsOQpsk250Instant = Instant<u64, 1, 62_500>;
pub type SymbolsOQpsk250Duration = Duration<u64, 1, 62_500>;

pub type SyntonizedInstant = Instant<u64, 1, 1_000_000_000>;
pub type SyntonizedDuration = NanosDurationU64;

pub trait RadioTimerApi: Sized {
    fn now() -> SyntonizedInstant;

    /// Waits until the given instant, then wakes the current task.
    ///
    /// Note: This method induces considerable latency and jitter as there may
    ///       be an arbitrary delay between waking the task and the task
    ///       executing. For more precisely timed alarms, use one of the
    ///       hardware-backed methods
    fn wait_until(instant: SyntonizedInstant) -> impl Future<Output = ()>;
}

pub fn now<Timer: RadioTimerApi>() -> SyntonizedInstant {
    Timer::now()
}

pub async fn wait_until<Timer: RadioTimerApi>(instant: SyntonizedInstant) {
    Timer::wait_until(instant).await
}
