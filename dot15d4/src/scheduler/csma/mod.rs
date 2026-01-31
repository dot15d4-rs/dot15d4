//! CSMA-CA scheduler module.
//!
//! This module implements the CSMA-CA medium access protocol as defined in
//! IEEE 802.15.4.
//!
//! # Module Structure
//!
//! - [`task`]: State machine and task structures
//! - [`logic`]: State transition logic implementation

pub mod logic;
pub mod task;

pub use self::task::CsmaTask;
