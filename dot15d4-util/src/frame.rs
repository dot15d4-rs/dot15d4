//! This module exposes a few generic traits to model zero-copy buffer backed
//! network frames across arbitrary network layers.
//!
//! The basic idea behind these traits is, that converting between
//! layer-specific frame representations should be as cheap as handing over an
//! allocated zero-copy buffer between them.

use crate::allocator::IntoBuffer;

/// Generic representation of a buffer-backed structured frame providing access
/// to its protocol data unit (i.e. the frame including its current layer
/// protocol's header and footer).
pub trait FramePdu: IntoBuffer {
    type Pdu: ?Sized;

    /// Exposes a read-only, possibly structured representation of the frame's
    /// PDU.
    fn pdu_ref(&self) -> &Self::Pdu;

    /// Exposes a mutable, possibly structured representation of the frame's
    /// PDU.
    fn pdu_mut(&mut self) -> &mut Self::Pdu;
}

/// Generic representation of a buffer-backed structured frame providing access
/// to its service data unit (i.e. its payload) and protocol data unit (i.e. its
/// header and footer).
pub trait Frame: FramePdu {
    /// Retrieves the frame's SDU for reading.
    fn sdu_ref(&self) -> &[u8];

    /// Retrieves the frame's SDU for writing.
    fn sdu_mut(&mut self) -> &mut [u8];
}
