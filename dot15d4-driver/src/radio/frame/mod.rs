//! This module provides support for radio driver configuration and
//! implementations.
//!
//! Most notably it exports the trait required to communicate radio driver
//! configuration, a radio frame representation and a simple default
//! implementation of a buffer-backed radio frame.

use core::fmt::Debug;

mod addressing;
mod frame_control;
mod radio_frame;
mod repr;
mod utils;

pub use addressing::*;
pub use frame_control::*;
pub use radio_frame::*;
pub use repr::*;
pub use utils::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFrameUnsized;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFrameSized;
