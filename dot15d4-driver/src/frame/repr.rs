use typenum::Unsigned;

use core::{fmt::Debug, marker::PhantomData, num::NonZero, ops::Range};

use crate::radio::DriverConfig;

use super::{RadioFrameSized, RadioFrameUnsized};

/// Provides a simple default radio frame representation implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFrameRepr<Config: DriverConfig, State> {
    config: PhantomData<Config>,
    /// Contains [`None`] in state [`RadioFrameUnsized`] and the SDU length in
    /// state [`RadioFrameSized`].
    ///
    /// The SDU length is the driver configuration dependent length of the PSDU
    /// (=MPDU). It contains the FCS length unless FCS calculation is offloaded
    /// to the driver or hardware (see [`crate::radio::FcsNone`])
    ///
    /// Safety: When set, the SDU length must be strictly greater then the
    ///         length of the FCS.
    sdu_length: Option<NonZero<u16>>,
    state: PhantomData<State>,
}

impl<Config: DriverConfig, State> RadioFrameRepr<Config, State> {
    pub const fn headroom_length(&self) -> u8 {
        <Config::Headroom as Unsigned>::U8
    }

    pub const fn tailroom_length(&self) -> u8 {
        <Config::Tailroom as Unsigned>::U8
    }

    pub const fn driver_overhead(&self) -> u8 {
        self.headroom_length() + self.tailroom_length()
    }

    pub const fn max_sdu_length(&self) -> u16 {
        <Config::MaxSduLength as Unsigned>::U16
    }

    pub const fn max_sdu_length_wo_fcs(&self) -> u16 {
        self.max_sdu_length() - self.fcs_length() as u16
    }

    pub const fn fcs_length(&self) -> u8 {
        size_of::<Config::Fcs>() as u8
    }

    pub const fn max_buffer_length(&self) -> u16 {
        self.max_sdu_length() + self.driver_overhead() as u16
    }
}

impl<Config: DriverConfig> RadioFrameRepr<Config, RadioFrameUnsized> {
    pub const fn new() -> Self {
        Self {
            config: PhantomData,
            sdu_length: None,
            state: PhantomData,
        }
    }

    pub const fn with_sdu(
        &self,
        sdu_length_wo_fcs: NonZero<u16>,
    ) -> RadioFrameRepr<Config, RadioFrameSized> {
        RadioFrameRepr::<Config, RadioFrameSized>::new(sdu_length_wo_fcs)
    }
}

impl<Config: DriverConfig> Default for RadioFrameRepr<Config, RadioFrameUnsized> {
    fn default() -> Self {
        Self::new()
    }
}

impl<Config: DriverConfig> RadioFrameRepr<Config, RadioFrameSized> {
    pub const fn new(sdu_length_wo_fcs: NonZero<u16>) -> Self {
        let sdu_length =
            sdu_length_wo_fcs.saturating_add(size_of::<<Config as DriverConfig>::Fcs>() as u16);
        let this = Self {
            config: PhantomData,
            sdu_length: Some(sdu_length),
            state: PhantomData,
        };
        debug_assert!(sdu_length.get() <= this.max_sdu_length());
        this
    }

    pub const fn offset_sdu(&self) -> u8 {
        self.headroom_length()
    }

    pub const fn offset_tailroom(&self) -> NonZero<u16> {
        // Safety: The SDU length must be set for a sized radio frame.
        self.sdu_length
            .unwrap()
            .saturating_add(self.offset_sdu() as u16)
    }

    pub const fn pdu_length(&self) -> u16 {
        self.offset_tailroom().get() + self.tailroom_length() as u16
    }

    pub const fn headroom_range(&self) -> Range<usize> {
        0..self.offset_sdu() as usize
    }

    pub const fn tailroom_range(&self) -> Range<usize> {
        self.offset_tailroom().get() as usize..self.pdu_length() as usize
    }

    pub const fn offset_fcs(&self) -> NonZero<u16> {
        // Safety: We added the FCS length to the non-zero SDU length on
        //         instantiation.
        unsafe { NonZero::new_unchecked(self.offset_tailroom().get() - self.fcs_length() as u16) }
    }

    pub const fn sdu_range_wo_fcs(&self) -> Range<usize> {
        self.offset_sdu() as usize..self.offset_fcs().get() as usize
    }

    pub const fn fcs_range(&self) -> Option<Range<usize>> {
        let offset_fcs = self.offset_fcs().get();
        let offset_tailroom = self.offset_tailroom().get();
        if offset_fcs == offset_tailroom {
            None
        } else {
            Some(self.offset_fcs().get() as usize..self.offset_tailroom().get() as usize)
        }
    }

    pub const fn pdu_range(&self) -> Range<usize> {
        0..self.pdu_length() as usize
    }

    /// Returns the PSDU (=MPDU) length of the frame including the FCS if the
    /// FCS is not offloaded to the driver or hardware, otherwise including the
    /// FCS length.
    ///
    /// This number depends on the driver configuration.
    pub const fn sdu_length(&self) -> NonZero<u16> {
        // Safety: The SDU length must be set for a sized radio frame.
        self.sdu_length.unwrap()
    }

    /// Calculates the PSDU (=MPDU) length of the frame without any FCS.
    ///
    /// This number is independent of the driver configuration.
    pub const fn sdu_length_wo_fcs(&self) -> NonZero<u16> {
        // Safety: We added the FCS length on creation so SDU length is always
        //         greater than the FCS length.
        unsafe { NonZero::new_unchecked(self.sdu_length().get() - self.fcs_length() as u16) }
    }
}
