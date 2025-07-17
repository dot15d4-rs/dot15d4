use core::{num::NonZero, ops::Range};

use dot15d4_driver::{
    export::Unsigned,
    frame::{RadioFrame, RadioFrameSized},
    DriverConfig,
};
use dot15d4_util::{
    allocator::{BufferToken, IntoBuffer},
    frame::FramePdu,
};

/// An unparsed MPDU.
#[derive(Debug, PartialEq, Eq)]
#[must_use = "Must recover the contained buffer after use."]
pub struct MpduFrame {
    pub(crate) buffer: BufferToken,
    /// Contains the buffer offset at which the MPDU starts.
    pub(crate) offset: u8,
    /// Contains the length of the MPDU excluding the FCS.
    pub(crate) length_wo_fcs: NonZero<u16>,
}

impl MpduFrame {
    /// Creates a new unparsed MPDU frame.
    ///
    /// # Safety
    ///
    /// 1. Incoming frames must always be backed by a `buffer` that is at least
    ///    [`RadioFrameRepr::max_buffer_length()`] bytes long. This is because we don't know
    ///    the length of the MPDU at this point, so we assume the maximum length.
    /// 2. Outgoing frames must always be backed by a `buffer` that can at least hold the smallest
    ///    MPDU, which is the immediate acknowledgment (ACK) frame.
    ///
    /// These conditions are not checked at runtime, so it is the caller's
    /// responsibility to ensure that the buffer is large enough.
    ///
    /// [`RadioFrameRepr::max_buffer_length()`]:
    /// dot15d4_driver::frame::RadioFrameRepr::max_buffer_length
    pub unsafe fn new(buffer: BufferToken, offset: u8, length_wo_fcs: NonZero<u16>) -> Self {
        Self {
            buffer,
            offset,
            length_wo_fcs,
        }
    }

    /// Returns the MPDU length of the frame including the FCS if the FCS is not
    /// offloaded to the driver or hardware, otherwise including the FCS length.
    ///
    /// This number depends on the driver configuration.
    pub fn pdu_length<Config: DriverConfig>(&self) -> u16 {
        self.length_wo_fcs.get() + size_of::<<Config as DriverConfig>::Fcs>() as u16
    }

    /// Calculates the MPDU length of the frame without any FCS.
    ///
    /// This number is independent of the driver configuration.
    pub fn pdu_length_wo_fcs(&self) -> u16 {
        self.length_wo_fcs.get()
    }

    fn pdu_range_wo_fcs(&self) -> Range<usize> {
        self.offset as usize..(self.offset as usize + self.pdu_length_wo_fcs() as usize)
    }

    fn pdu_ref_wo_fcs(&self) -> &[u8] {
        &self.buffer[self.pdu_range_wo_fcs()]
    }

    pub fn pdu_mut_wo_fcs(&mut self) -> &mut [u8] {
        let pdu_range = self.pdu_range_wo_fcs();
        &mut self.buffer[pdu_range]
    }

    /// Produces an unparsed MPDU from a radio frame.
    pub fn from_radio_frame(radio_frame: RadioFrame<RadioFrameSized>) -> Self {
        let offset = radio_frame.headroom_length();
        let length_wo_fcs = radio_frame.sdu_wo_fcs_length();
        MpduFrame {
            buffer: radio_frame.into_buffer(),
            offset,
            length_wo_fcs,
        }
    }

    /// Converts an MPDU into a radio frame.
    ///
    /// Calculates the driver-specific FCS if required.
    pub fn into_radio_frame<Config: DriverConfig>(self) -> RadioFrame<RadioFrameSized> {
        debug_assert_eq!(self.offset, <Config::Headroom as Unsigned>::U8);

        // TODO: Calculate the FCS if required.
        // Safety: The length must be set for a sized MPDU.
        RadioFrame::new::<Config>(self.buffer).with_size(self.length_wo_fcs)
    }
}

impl AsRef<Self> for MpduFrame {
    fn as_ref(&self) -> &Self {
        self
    }
}

impl AsMut<Self> for MpduFrame {
    fn as_mut(&mut self) -> &mut Self {
        self
    }
}

impl IntoBuffer for MpduFrame {
    fn into_buffer(self) -> BufferToken {
        self.buffer
    }
}

impl FramePdu for MpduFrame {
    type Pdu = Self;

    fn pdu_ref(&self) -> &Self {
        self
    }

    fn pdu_mut(&mut self) -> &mut Self {
        self
    }
}
