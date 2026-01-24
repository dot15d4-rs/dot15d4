//! CSMA-CA scheduler module.
//!
//! Implements the CSMA-CA medium access protocol for IEEE 802.15.4.

pub mod logic;
pub mod task;

pub use self::task::CsmaTask;
