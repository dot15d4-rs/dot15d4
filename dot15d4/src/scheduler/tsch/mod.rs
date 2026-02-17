//! TSCH (Time Slotted Channel Hopping) scheduler module.

pub mod beacon;
pub mod logic;
pub mod pib;
pub mod task;
#[cfg(test)]
mod tests;

pub use self::pib::{ScheduleError, TschAsn, TschLink, TschLinkType, TschPib, TschSlotframe};
pub use self::task::{
    BeaconConfig, TschOperation, TschState, TschTask, INFINITE_DEADLINE, TIMESLOT_GUARD_TIME_US,
};
