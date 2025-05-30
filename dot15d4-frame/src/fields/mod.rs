//! This module contains types providing direct zero-copy field access. All
//! structured fields are little-endian and `#[repr(C,packed)]`. Multi-byte
//! fields handle endianness on-the-fly providing appropriate accessors. This
//! means that no intermediate representation is needed to access fields in
//! their encoded form.
//!
//! Packed little-endian structures imply non-aligned read/write access possibly
//! with runtime endianness conversions. As we receive and send packed,
//! little-endian frames, this is unavoidable and therefore not to be considered
//! unnecessary runtime overhead if clients read/write the same field only once.
//! Clients that need to read/write the same field more than once SHALL cache
//! unpacked, host-encoded field data locally as required.
//!
//! All fields are instantiated from a type that implements `AsRef<[u8]>` or
//! `AsMut<[u8]>`. Therefore fields MAY be instantiated from a packed buffer
//! slice that is known to contain the field. Alternatively "owned" versions of
//! all fields may instantiated from an array of bytes.
//!
//! Field representations are intended to be used both, on incoming (parsed) and
//! outgoing (built) frames. This allows us to re-use a large portion of code in
//! both directions - including critical validations and conversions - saving
//! code size on small embedded devices.

mod field_ranges;
mod ies;
mod mpdu;

pub use ies::*;
pub use mpdu::*;
