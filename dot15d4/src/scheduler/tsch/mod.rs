//! TSCH (Time Slotted Channel Hopping) scheduler module.

pub mod beacon;
mod log;
pub mod logic;
pub mod pib;
pub mod task;
#[cfg(any(test, feature = "_fuzz-harness"))]
pub mod tests;

pub use self::pib::{ScheduleError, TschAsn, TschLink, TschLinkType, TschPib, TschSlotframe};
pub use self::task::{
    BeaconConfig, TschCsmaBackoff, TschOperation, TschState, TschTask, TxOrigin,
    INFINITE_DEADLINE, TIMESLOT_GUARD_TIME_US,
};
