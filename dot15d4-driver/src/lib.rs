//! This crate provides everything related to low-level radio hardware access:
//! - interfaces exposed to the driver service,
//! - utilities shared between driver implementations,
//! - actual driver implementations - currently a driver for nrf52840 is
//!   provided as a showcase.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod config;
pub mod const_config;
pub mod constants;
pub mod frame;
pub mod radio;
pub mod socs;
pub mod tasks;
pub mod timer;
