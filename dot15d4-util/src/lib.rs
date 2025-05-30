//! This crate contains generic utilities other dot15d4 crates depend upon but
//! not directly related to the IEEE 802.15.4 standard.
//!
//! The main purpose of this crate is to make dot15d4 as self-contained as
//! possible.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod allocator;
pub mod frame;
pub mod sync;
pub mod tokens;
#[cfg(feature = "rtos-trace")]
pub mod trace;

pub mod log;

#[cfg(any(feature = "defmt", feature = "log"))]
pub use log::*;

/// A generic error.
#[derive(Debug, Clone, Copy)]
pub struct Error;

/// A type alias for `Result<T, dot15d4-util::Error>`.
pub type Result<T> = core::result::Result<T, Error>;
