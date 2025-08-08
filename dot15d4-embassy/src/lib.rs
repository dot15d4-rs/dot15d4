#![no_std]

#[cfg(feature = "nrf")]
use embassy_nrf as _;

pub mod driver;
pub mod stack;

pub mod export {
    pub use crate::stack::export::*;
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    #[cfg(feature = "defmt")]
    compile_error!(
        "Tracing cannot be enabled at the same time as defmt. Logs will be visible in the SystemView application if the 'log' feature is enabled."
    );

    use dot15d4::util::trace::TraceOffset;

    const OFFSET: TraceOffset = TraceOffset::Dot15d4Embassy;

    // Markers
    pub const RX_TOKEN_CONSUMED: u32 = OFFSET.wrap(0);
    pub const TX_TOKEN_CONSUMED: u32 = OFFSET.wrap(1);

    /// Instrument the library for tracing.
    pub(crate) fn instrument() {
        rtos_trace::trace::name_marker(RX_TOKEN_CONSUMED, "RX Token consumed\0");
        rtos_trace::trace::name_marker(TX_TOKEN_CONSUMED, "TX Token consumed\0");
    }
}
