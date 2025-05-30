//! This module contains minimal structural representations for all MPDU fields.
//!
//! These representations contain just enough information to calculate field
//! offsets and lengths. They are optimized for minimal runtime memory and CPU
//! resource usage.
//!
//! On incoming frames structural representations are used to calculate offsets
//! and ranges into the incoming radio frame buffer while parsing the frame.
//! Field content may then be read directly from the incoming zero-copy buffer.
//!
//! On outgoing frames structural representations are used to calculate the
//! required buffer length with minimal runtime footprint. Once a zero-copy
//! buffer has been allocated, the same information can then be used to write
//! field content directly into the buffer.

mod ies;
mod mpdu;
mod security;
mod seq_nr;

pub use ies::*;
pub use mpdu::*;
pub use security::*;
pub use seq_nr::*;
