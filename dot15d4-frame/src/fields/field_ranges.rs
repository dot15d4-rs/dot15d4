use core::{marker::PhantomData, num::NonZero, ops::Range};

use dot15d4_driver::{frame::AddressingRepr, DriverConfig};
use dot15d4_util::{Error, Result};

#[cfg(feature = "ies")]
use crate::repr::IeListRepr;
#[cfg(feature = "security")]
use crate::repr::SecurityRepr;
use crate::{
    repr::SeqNrRepr, MpduParsedUpToAddressing, MpduParsedUpToSecurity, MpduWithAddressing,
    MpduWithAllFields, MpduWithFrameControl, MpduWithSecurity,
};

/// This structure contains various field offsets successively collected while
/// parsing an MPDU.
///
/// All offsets are actually pure functions of an unparsed MPDU in state
/// [`MpduSized`]`, i.e. an MPDU whose overall length is known. This is the type
/// of MPDU we get from a receiving driver. Storing offsets is therefore
/// redundant and can only be justified as a performance optimization when
/// fields need to be accessed repeatedly.
///
/// As acquiring offsets is not free and we often only want to read a few
/// initial fields, e.g. just the frame control, sequence number or addressing
/// fields, we acquire offsets in stages on an as-needed basis.
///
/// Each additional stage unlocks access to further fields which will be
/// accessible via typestates of an MPDU reader generic over the MPDU parsing
/// stage. We introduce traits implemented by different stages so that we can
/// incrementally add offsets to the API.
///
/// All offsets use the smallest representation possible to keep the memory
/// footprint of this structure low as it may have to be transported across
/// channels.
///
/// We use options of non-zero values for all offsets. This allows us to express
/// the information whether a field has already been parsed idiomatically
/// without runtime overhead. Offsets that are [`None`] are not yet known.
///
/// If a field does not exist in the frame then the offset will be the same as
/// the previous offset. Our accessor methods will recognize this and return
/// [`None`] as the field range.
///
/// All offsets are relative to the start of the buffer.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub(crate) struct MpduFieldRanges<State> {
    state: PhantomData<State>,

    // The frame control field is the first field of the MPDU. As the driver's
    // headroom is known statically and corresponds to the MPDU offset, the
    // field can always be accessed. It may be zero if the driver does not
    // require headroom.
    offset_frame_control: u8,
    // The addressing offset is always known as we pass in sequence number
    // configuration when instantiating the parser info.
    offset_addressing: NonZero<u8>,
    #[cfg(feature = "security")]
    offset_aux_sec_hdr: Option<NonZero<u8>>,
    // We cannot save the MIC offset directly as the intervening fields are not
    // known when the MIC length information becomes available.
    #[cfg(feature = "security")]
    length_mic: Option<NonZero<u8>>,
    #[cfg(feature = "ies")]
    offset_ies: Option<NonZero<u8>>,
    // Header fields and IEs together may occupy more than u8::MAX bytes,
    // therefore we need to represent the remaining offsets as u16.
    offset_frame_payload: Option<NonZero<u16>>,
    offset_fcs: Option<NonZero<u16>>,
    offset_remainder: Option<NonZero<u16>>,
} // 12 bytes, 4 bytes less without security, 2 bytes less without IEs

impl MpduFieldRanges<MpduWithFrameControl> {
    pub(crate) const fn new(offset_mpdu: u8, seq_nr: SeqNrRepr) -> Self {
        const FRAME_CONTROL_LEN: u8 = 2;
        const SEQ_NR_LEN: u8 = 1;
        let offset_frame_control = offset_mpdu;
        let offset_seq_nr = offset_frame_control + FRAME_CONTROL_LEN;
        let offset_addressing = offset_seq_nr
            + match seq_nr {
                SeqNrRepr::Yes => SEQ_NR_LEN,
                SeqNrRepr::No => 0,
            };
        Self {
            state: PhantomData,
            offset_frame_control,
            offset_addressing: as_nz_u8(offset_addressing).unwrap(),
            #[cfg(feature = "security")]
            offset_aux_sec_hdr: None,
            #[cfg(feature = "security")]
            length_mic: None,
            #[cfg(feature = "ies")]
            offset_ies: None,
            offset_frame_payload: None,
            offset_fcs: None,
            offset_remainder: None,
        }
    }

    pub(crate) const fn with_addressing(
        &self,
        addressing: AddressingRepr,
    ) -> Result<MpduFieldRanges<MpduWithAddressing>> {
        let addressing_fields_length = match addressing.addressing_fields_length() {
            Ok(len) => len,
            Err(e) => return Err(e),
        };
        Ok(self.next_state(self.offset_addressing.get() + addressing_fields_length as u8))
    }

    pub(crate) const fn without_addressing(&self) -> MpduFieldRanges<MpduWithAddressing> {
        self.next_state(self.offset_addressing.get())
    }

    const fn next_state(&self, next_offset: u8) -> MpduFieldRanges<MpduWithAddressing> {
        #[cfg(all(feature = "ies", feature = "security"))]
        let offset_ies = None;
        #[cfg(all(feature = "ies", not(feature = "security")))]
        let offset_ies = as_nz_u8(next_offset);
        #[cfg(any(feature = "security", feature = "ies"))]
        let offset_frame_payload = None;
        #[cfg(not(any(feature = "security", feature = "ies")))]
        let offset_frame_payload = as_nz_u16(next_offset as u16);
        MpduFieldRanges {
            state: PhantomData,
            offset_frame_control: self.offset_frame_control,
            offset_addressing: self.offset_addressing,
            #[cfg(feature = "security")]
            offset_aux_sec_hdr: as_nz_u8(next_offset),
            #[cfg(feature = "security")]
            length_mic: None,
            #[cfg(feature = "ies")]
            offset_ies,
            offset_frame_payload,
            offset_fcs: None,
            offset_remainder: None,
        }
    }
}

impl MpduFieldRanges<MpduWithAddressing> {
    #[cfg(feature = "security")]
    pub(crate) const fn with_security(
        &self,
        security: SecurityRepr,
    ) -> MpduFieldRanges<MpduWithSecurity> {
        let aux_sec_header_length = security.aux_sec_header_length() as u8;
        let length_mic = as_nz_u8(security.mic_length() as u8);
        self.next_state(
            self.offset_aux_sec_hdr.unwrap().get() + aux_sec_header_length,
            length_mic,
        )
    }

    pub(crate) const fn without_security(&self) -> MpduFieldRanges<MpduWithSecurity> {
        #[cfg(feature = "security")]
        return self.next_state(self.offset_aux_sec_hdr.unwrap().get(), None);
        #[cfg(not(feature = "security"))]
        return self.next_state(self.offset_addressing.get(), None);
    }

    const fn next_state(
        &self,
        next_offset: u8,
        _length_mic: Option<NonZero<u8>>,
    ) -> MpduFieldRanges<MpduWithSecurity> {
        #[cfg(feature = "ies")]
        let offset_frame_payload = None;
        #[cfg(not(feature = "ies"))]
        let offset_frame_payload = as_nz_u16(next_offset as u16);
        MpduFieldRanges {
            state: PhantomData,
            offset_frame_control: self.offset_frame_control,
            offset_addressing: self.offset_addressing,
            #[cfg(feature = "security")]
            offset_aux_sec_hdr: as_nz_u8(next_offset),
            #[cfg(feature = "security")]
            length_mic: _length_mic,
            #[cfg(feature = "ies")]
            offset_ies: as_nz_u8(next_offset),
            offset_frame_payload,
            offset_fcs: None,
            offset_remainder: None,
        }
    }
}

impl MpduFieldRanges<MpduWithSecurity> {
    /// Call this method to configure information elements when the frame
    /// payload length is known. This is usually the case when building frames
    /// from scratch.
    ///
    /// Validates the given IE list.
    #[cfg(feature = "ies")]
    pub(crate) const fn with_ies_and_payload_length<Config: DriverConfig>(
        &self,
        ies: IeListRepr,
        frame_payload_length: u16,
    ) -> Result<MpduFieldRanges<MpduWithAllFields>> {
        let ies_length = match ies.ies_length(frame_payload_length > 0) {
            Ok(len) => len,
            Err(e) => return Err(e),
        };
        Ok(self.next_state::<Config>(ies_length, frame_payload_length))
    }

    /// Call this method to configure information elements when the frame
    /// payload length must be derived from the overall length of the frame.
    /// This is usually the case when parsing incoming radio frames.
    ///
    /// This only works when the given IE list contains valid termination
    /// headers. Otherwise the calculation is non-deterministic as payload
    /// termination IEs are optional. The given IE list is additionally
    /// validated for consistency.
    ///
    /// The given MPDU length is the length of the MPDU without any driver- or
    /// PHY-level headers/footers and _without the FCS_, i.e. the number of
    /// bytes consumed by the MAC header and MAC payload without the MAC footer.
    ///
    /// See [`dot15d4_driver::frame::RadioFrame::sdu_wo_fcs_length()`].
    #[cfg(feature = "ies")]
    pub(crate) const fn with_ies_and_mpdu_length<Config: DriverConfig>(
        &self,
        ies: IeListRepr,
        mpdu_length_wo_fcs: u16,
    ) -> Result<MpduFieldRanges<MpduWithAllFields>> {
        let mpdu_less_ies_and_payload_length = self.last_offset();
        if mpdu_less_ies_and_payload_length > mpdu_length_wo_fcs {
            return Err(Error);
        }
        let mpdu_ies_and_payload_length = mpdu_length_wo_fcs - mpdu_less_ies_and_payload_length;
        let (ies_length, frame_payload_length) =
            match ies.ies_and_frame_payload_length(mpdu_ies_and_payload_length) {
                Ok(frame_payload_length) => frame_payload_length,
                Err(e) => return Err(e),
            };
        Ok(self.next_state::<Config>(ies_length, frame_payload_length))
    }

    /// Call this method to finalize the frame without IEs when the frame
    /// payload length is known. This is usually the case when building frames
    /// from scratch.
    pub(crate) const fn without_ies_with_payload_length<Config: DriverConfig>(
        &self,
        frame_payload_length: u16,
    ) -> MpduFieldRanges<MpduWithAllFields> {
        self.next_state::<Config>(0, frame_payload_length)
    }

    /// Call this method to finalize the frame without IEs when the frame
    /// payload length must be derived from the overall length of the frame.
    /// This is usually the case when parsing incoming radio frames.
    ///
    /// The given MPDU length is the length of the MPDU without any driver- or
    /// PHY-level headers/footers and _without the FCS_, i.e. the number of
    /// bytes consumed by the MAC header and MAC payload without the MAC footer.
    ///
    /// See [`dot15d4_driver::frame::RadioFrame::sdu_wo_fcs_length()`].
    pub(crate) const fn without_ies_with_mpdu_length<Config: DriverConfig>(
        &self,
        mpdu_length_wo_fcs: u16,
    ) -> Result<MpduFieldRanges<MpduWithAllFields>> {
        let mpdu_less_payload_length = self.last_offset();
        if mpdu_less_payload_length > mpdu_length_wo_fcs {
            return Err(Error);
        }
        let frame_payload_length = mpdu_length_wo_fcs - mpdu_less_payload_length;
        Ok(self.next_state::<Config>(0, frame_payload_length))
    }

    const fn last_offset(&self) -> u16 {
        #[cfg(feature = "ies")]
        let last_offset = self.offset_ies.unwrap().get();
        #[cfg(all(feature = "security", not(feature = "ies")))]
        let last_offset = self.offset_aux_sec_hdr.unwrap().get();
        #[cfg(not(any(feature = "security", feature = "ies")))]
        let last_offset = self.offset_addressing.get();
        last_offset as u16
    }

    const fn next_state<Config: DriverConfig>(
        &self,
        ies_length: u16,
        frame_payload_length: u16,
    ) -> MpduFieldRanges<MpduWithAllFields> {
        let offset_frame_payload = self.last_offset() + ies_length;

        #[cfg(feature = "security")]
        let length_mic = match self.length_mic {
            Some(length_mic) => length_mic.get() as u16,
            None => 0,
        };
        #[cfg(not(feature = "security"))]
        let length_mic = 0;
        let offset_fcs = offset_frame_payload + frame_payload_length + length_mic;

        let offset_remainder = offset_fcs + size_of::<<Config as DriverConfig>::Fcs>() as u16;
        MpduFieldRanges {
            state: PhantomData,
            offset_frame_control: self.offset_frame_control,
            offset_addressing: self.offset_addressing,
            #[cfg(feature = "security")]
            offset_aux_sec_hdr: self.offset_aux_sec_hdr,
            #[cfg(feature = "security")]
            length_mic: self.length_mic,
            #[cfg(feature = "ies")]
            offset_ies: self.offset_ies,
            offset_frame_payload: as_nz_u16(offset_frame_payload),
            offset_fcs: as_nz_u16(offset_fcs),
            offset_remainder: as_nz_u16(offset_remainder),
        }
    }
}

/// Initial fields are available from all parser states.
impl<State> MpduFieldRanges<State> {
    const FRAME_CONTROL_LEN: u16 = 2;

    /// The buffer range containing the frame control field.
    pub(crate) const fn range_frame_control(&self) -> Range<usize> {
        let offset_frame_control = self.offset_frame_control as usize;
        offset_frame_control..self.offset_seq_nr().get() as usize
    }

    /// As the sequence number is only one byte long, we return it as index
    /// into the buffer rather than as range.
    pub(crate) const fn offset_seq_nr(&self) -> NonZero<u16> {
        // Safety: This value is non-zero as FRAME_CONTROL_LEN is non-zero.
        unsafe {
            NonZero::new_unchecked(self.offset_frame_control as u16 + Self::FRAME_CONTROL_LEN)
        }
    }
}

/// Addressing fields are available on all states implementing
/// [`MpduParsedUpToAddressing`].
impl<State: MpduParsedUpToAddressing> MpduFieldRanges<State> {
    /// The buffer range containing all addressing fields.
    pub(crate) const fn range_addressing(&self) -> Option<Range<usize>> {
        let offset_addressing = self.offset_addressing.get() as usize;

        #[cfg(feature = "security")]
        let next_offset = self.offset_aux_sec_hdr.unwrap().get() as usize;
        #[cfg(all(not(feature = "security"), feature = "ies"))]
        let next_offset = self.offset_ies.unwrap().get() as usize;
        #[cfg(not(any(feature = "security", feature = "ies")))]
        let next_offset = self.offset_frame_payload.unwrap().get() as usize;

        if next_offset == offset_addressing {
            None
        } else {
            Some(offset_addressing..next_offset)
        }
    }
}

/// Security fields are available on all states implementing
/// [`MpduParsedUpToSecurity`].
impl<State: MpduParsedUpToSecurity> MpduFieldRanges<State> {
    /// The buffer range containing the auxiliary security header.
    pub(crate) const fn range_aux_sec_header(&self) -> Option<Range<usize>> {
        #[cfg(feature = "security")]
        return {
            let offset_aux_sec_hdr = self.offset_aux_sec_hdr.unwrap().get() as usize;

            #[cfg(feature = "ies")]
            let next_offset = self.offset_ies.unwrap().get() as usize;
            #[cfg(not(feature = "ies"))]
            let next_offset = self.offset_frame_payload.unwrap().get() as usize;

            if next_offset == offset_aux_sec_hdr {
                None
            } else {
                Some(offset_aux_sec_hdr..next_offset)
            }
        };
        #[cfg(not(feature = "security"))]
        return None;
    }
}

/// The [`MpduSized`] parsing state represents a fully parsed frame with access
/// to all sub-fields.
impl MpduFieldRanges<MpduWithAllFields> {
    /// The buffer range containing information elements.
    pub(crate) const fn range_ies(&self) -> Option<Range<usize>> {
        #[cfg(feature = "ies")]
        return {
            let offset_ies = self.offset_ies.unwrap().get() as usize;
            let next_offset = self.offset_frame_payload.unwrap().get() as usize;
            if next_offset == offset_ies {
                None
            } else {
                Some(offset_ies..next_offset)
            }
        };
        #[cfg(not(feature = "ies"))]
        return None;
    }

    /// The index of the first byte of frame payload.
    pub(crate) const fn offset_frame_payload(&self) -> u16 {
        self.offset_frame_payload.unwrap().get()
    }

    /// The index of the first byte after the end of the frame payload.
    ///
    /// If this is the same index as [`Self::offset_frame_payload()`] then the
    /// frame does not have a payload.
    pub(crate) const fn offset_frame_payload_end(&self) -> u16 {
        #[cfg(feature = "security")]
        return self.offset_fcs.unwrap().get() - self.length_mic.unwrap().get() as u16;
        #[cfg(not(feature = "security"))]
        return self.offset_fcs.unwrap().get();
    }

    /// The buffer range containing the frame payload.
    pub(crate) const fn range_frame_payload(&self) -> Option<Range<usize>> {
        let start_frame_payload = self.offset_frame_payload() as usize;
        let end_frame_payload = self.offset_frame_payload_end() as usize;
        if start_frame_payload == end_frame_payload {
            None
        } else {
            Some(start_frame_payload..end_frame_payload)
        }
    }

    /// The buffer range containing the MIC.
    pub(crate) const fn range_mic(&self) -> Option<Range<usize>> {
        #[cfg(feature = "security")]
        return {
            let next_offset = self.offset_fcs.unwrap().get() as usize;
            let offset_mic = next_offset - self.length_mic.unwrap().get() as usize;
            if offset_mic == next_offset {
                None
            } else {
                Some(offset_mic..next_offset)
            }
        };
        #[cfg(not(feature = "security"))]
        return None;
    }

    /// The buffer range containing the FCS.
    pub(crate) const fn range_fcs(&self) -> Option<Range<usize>> {
        let offset_fcs = self.offset_fcs.unwrap().get() as usize;
        let next_offset = self.offset_remainder.unwrap().get() as usize;
        if offset_fcs == next_offset {
            None
        } else {
            Some(offset_fcs..next_offset)
        }
    }

    /// The index of the first byte of tailroom. We don't return a range as
    /// interpreting tailroom is only done on radio frames.
    pub(crate) const fn offset_tailroom(&self) -> u16 {
        self.offset_remainder.unwrap().get()
    }
}

/// A helper function that saves some typing when constructing non-zero values
/// from known-to-be positive values.
///
/// Safety: requires the given value to be non-zero.
const fn as_nz_u8(from: u8) -> Option<NonZero<u8>> {
    Some(unsafe { NonZero::new_unchecked(from) })
}

/// Same as [`as_nz_u8`], just for u16 values.
///
/// Safety: requires the given value to be non-zero.
const fn as_nz_u16(from: u16) -> Option<NonZero<u16>> {
    Some(unsafe { NonZero::new_unchecked(from) })
}
