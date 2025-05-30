use const_for::const_for;

use dot15d4_util::{Error, Result};

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum IeRepr<'ie> {
    TimeCorrectionHeaderIe,
    ReducedChannelHoppingNestedIe,
    FullChannelHoppingNestedIe(u8, bool), // num channels, is SUN PHY
    TschSynchronizationNestedIe,
    TschSlotframeAndLinkNestedIe(&'ie [u8]), // for each slotframe descriptor: number of links
    ReducedTschTimeslotNestedIe,
    FullTschTimeslotNestedIe,
} // 12 bytes
  // TODO: Consider removing IEs based on the supported protocol to reduce size to
  //       1 byte for protocols that don't require parameterized IE config.

impl IeRepr<'_> {
    /// Returns `(header_ie_len, nested_ie_len)`. The length of a header IE
    /// includes its IE header fields. The nested IE length includes the header
    /// fields of the nested IE but does not include the MLME header length.
    ///
    /// Safety: Must not be called on termination IEs.
    pub const fn length(&self) -> (u16, u16) {
        if cfg!(feature = "ies") {
            const IE_HDR_SIZE: u16 = 2;

            let (header_ie_content_len, nested_ie_content_len) = match self {
                IeRepr::TimeCorrectionHeaderIe => (2, 0),
                IeRepr::ReducedChannelHoppingNestedIe => (0, 1),
                IeRepr::FullChannelHoppingNestedIe(num_channels, is_sun_phy) => {
                    let extended_bm_len = if *is_sun_phy {
                        num_channels.div_ceil(u8::BITS as u8) as u16
                    } else {
                        0
                    };
                    (0, 12 + (*num_channels as u16) + extended_bm_len)
                }
                IeRepr::TschSynchronizationNestedIe => (0, 6),
                IeRepr::TschSlotframeAndLinkNestedIe(slotframes) => {
                    const LINK_INFO_LEN: u16 = 5;
                    const SLOTFRAME_DESCRIPTOR_HDR_LEN: u16 = 4;
                    const TSCH_SLOTFRAME_AND_LINK_HDR_LEN: u16 = 1;
                    let mut content_len = TSCH_SLOTFRAME_AND_LINK_HDR_LEN
                        + slotframes.len() as u16 * SLOTFRAME_DESCRIPTOR_HDR_LEN;
                    const_for!(sf_idx in 0..slotframes.len() => {
                        let link_info_fields = slotframes[sf_idx];
                        content_len += (link_info_fields as u16) * LINK_INFO_LEN;

                    });
                    (0, content_len)
                }
                IeRepr::ReducedTschTimeslotNestedIe => (0, 1),
                IeRepr::FullTschTimeslotNestedIe => (0, 25),
            };

            if header_ie_content_len > 0 {
                (IE_HDR_SIZE + header_ie_content_len, 0)
            } else if nested_ie_content_len > 0 {
                (0, IE_HDR_SIZE + nested_ie_content_len)
            } else {
                unreachable!()
            }
        } else {
            (0, 0)
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum IeReprWithTermination<'ie> {
    NonTerminationIe(IeRepr<'ie>),

    // Termination IEs will be generated synthetically when building a frame and
    // will be parsed from incoming frames.
    HeaderTerminationIe1,
    HeaderTerminationIe2,
    PayloadTerminationIe,
} // 12 bytes

/// A list of IE representations.
///
/// The list is generic over the implementation of the IE representation so that
/// it can accept both, a list including or excluding termination IEs.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct IeReprList<'ies, IeRepr>(&'ies [IeRepr]);

impl<'ies, IeRepr> IeReprList<'ies, IeRepr> {
    const IE_HDR_SIZE: u16 = 2;

    pub const fn new(ies: &'ies [IeRepr]) -> Self {
        Self(ies)
    }
}

/// A list of IE representations without termination IEs.
///
/// This is usually required when building MPDUs from scratch as termination IEs
/// will then be added automatically.
impl IeReprList<'_, IeRepr<'_>> {
    /// Calculates the length of the IEs adding termination headers if required,
    /// also depending on whether a frame payload will be added or not.
    ///
    /// This is usually required when building an outgoing frame.
    pub const fn ies_length(&self, has_frame_payload: bool) -> u16 {
        let mut len = 0;

        // State required to validate IE termination.
        let mut has_header_ie = false;
        let mut has_nested_ie = false;

        const_for!(ie_idx in 0..self.0.len() => {
            let ie = self.0[ie_idx];
            let (header_ie_len, nested_ie_len) = ie.length();

            if header_ie_len > 0 {
                has_header_ie = true;
                len += header_ie_len;
            } else if nested_ie_len > 0 {
                has_nested_ie = true;
                len += nested_ie_len;
            } else {
                unreachable!()
            }
        });

        if has_nested_ie {
            // MLME IE header
            len += Self::IE_HDR_SIZE;
        }

        // See IEEE 802.15.4-2024, section 7.4.1
        len += match (has_header_ie, has_nested_ie, has_frame_payload) {
            // Header Termination | Payload Termination
            // ========================================
            // None               | None
            (false, false, false) | (true, false, false) | (false, false, true) => 0,
            // HT1                | None (Optional)
            (false, true, false) | (true, true, false) |
            // HT2                | None
            (true, false, true) => Self::IE_HDR_SIZE,
            // HT1                | PT
            (false, true, true) | (true, true, true) => 2*Self::IE_HDR_SIZE,
        };

        len
    }
}

/// A list of IE representations including termination IEs.
///
/// This is usually required when parsing incoming MPDUs as termination IEs will
/// then have to be parsed an validated.
impl IeReprList<'_, IeReprWithTermination<'_>> {
    /// Calculates the length of the IEs and determines whether a payload is
    /// expected after the IE list. Returns a tuple `(ies_length,
    /// has_frame_payload)`.
    ///
    /// This is usually required when parsing an incoming frame.
    ///
    /// Safety: Must not be called with an empty IE list as then the length is
    ///         deterministically zero and a value for `has_frame_payload`
    ///         cannot be determined.
    pub const fn ies_length_and_payload_presence(&self) -> Result<(u16, bool)> {
        debug_assert!(!self.0.is_empty());

        let mut len = 0;

        // State required to validate IE termination.
        let mut has_header_ie = false;
        let mut has_nested_ie = false;
        let mut has_header_termination_ie_1 = false;
        let mut has_header_termination_ie_2 = false;
        let mut has_payload_termination_ie = false;

        const_for!(ie_idx in 0..self.0.len() => {
            let ie = self.0[ie_idx];
            match ie {
                IeReprWithTermination::NonTerminationIe(ie) => {
                    let (header_ie_len, nested_ie_len) = ie.length();

                    if header_ie_len > 0 {
                        has_header_ie = true;
                        len += header_ie_len;
                    } else if nested_ie_len > 0 {
                        has_nested_ie = true;
                        len += nested_ie_len;
                    } else {
                        unreachable!()
                    }
                }
                IeReprWithTermination::HeaderTerminationIe1 => {
                    len += Self::IE_HDR_SIZE;
                    has_header_termination_ie_1 = true
                }
                IeReprWithTermination::HeaderTerminationIe2 => {
                    len += Self::IE_HDR_SIZE;
                    has_header_termination_ie_2 = true
                }
                IeReprWithTermination::PayloadTerminationIe => {
                    len += Self::IE_HDR_SIZE;
                    has_payload_termination_ie = true
                }
            }
        });

        if has_nested_ie {
            // MLME IE
            len += Self::IE_HDR_SIZE;
        }

        let has_header_termination_ie = has_header_termination_ie_1 || has_header_termination_ie_2;

        // See IEEE 802.15.4-2024, section 7.4.1
        let has_frame_payload = match (
            has_header_ie,
            has_nested_ie,
            has_header_termination_ie,
            has_payload_termination_ie,
        ) {
            // Header IE | Payload IE | Header Termination | Payload Termination
            // =================================================================
            // Yes       | No         | None               | None
            (true, false, false, false) => false,
            // Any       | Yes        | HT1                | Optional
            (_, true, true, _) if has_header_termination_ie_1 => false,
            // Yes       | No         | HT2                | None
            (true, false, true, false) if has_header_termination_ie_2 => true,
            // Any       | No         | HT2                | None
            (_, true, true, true) if has_header_termination_ie_1 => true,

            // Error conditions:
            // - When both, header and payload IEs are present, then an HT1 is
            //   required.
            (true, true, false, _) |
            // - When payload IEs are present, then only HT1 can also be
            //   present.
            (_, true, _, _) |
            // - When no payload IEs are present, then only HT2 can also be
            //   present.
            (_, false, true, _) |
            // - A payload termination IE may only be present when there are
            //   payload IEs.
            (_, false, false, true) => return Err(Error),

            // Non-recoverable error: It is ok for the IE list to be empty but
            // in this case we cannot decide whether we have a payload or not.
            (false, false, false, false) => panic!("IE list empty"),
        };

        Ok((len, has_frame_payload))
    }
}

/// Generic structural representation of a list or IEs.
///
/// Provides functionality required both, on incoming and outgoing MPDUs.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum IeListRepr<'ies> {
    Empty,
    WithTerminationIes(IeReprList<'ies, IeReprWithTermination<'ies>>),
    WithoutTerminationIes(IeReprList<'ies, IeRepr<'ies>>),
}

impl IeListRepr<'_> {
    pub const fn is_empty(&self) -> bool {
        matches!(self, IeListRepr::Empty)
    }

    pub const fn ies_length(&self, has_frame_payload: bool) -> Result<u16> {
        let len = match self {
            IeListRepr::Empty => 0,
            IeListRepr::WithTerminationIes(ie_list) => {
                if let Ok((ies_length, should_have_frame_payload)) =
                    ie_list.ies_length_and_payload_presence()
                {
                    if has_frame_payload != should_have_frame_payload {
                        // Inconsistency between IEs and input parameter.
                        return Err(Error);
                    }
                    ies_length
                } else {
                    // Invalid IEs list.
                    return Err(Error);
                }
            }
            IeListRepr::WithoutTerminationIes(ie_list) => ie_list.ies_length(has_frame_payload),
        };
        Ok(len)
    }

    /// Calculates `(ies_length, frame_payload_length)` from the sum of the IE
    /// and payload length based on the IE list.
    ///
    /// This is usually required when parsing incoming frames. In this case the
    /// remaining length of the MPDU is known from which the length of the IE
    /// and frame payload fields need to be derived.
    pub const fn ies_and_frame_payload_length(
        &self,
        mpdu_ies_and_payload_length: u16,
    ) -> Result<(u16, u16)> {
        let ies_and_frame_payload_len = match self {
            IeListRepr::Empty => (0, mpdu_ies_and_payload_length),
            IeListRepr::WithTerminationIes(ie_list) => {
                let (ies_length, has_frame_payload) =
                    match ie_list.ies_length_and_payload_presence() {
                        Ok(ies_length_and_payload_presence) => ies_length_and_payload_presence,
                        Err(e) => return Err(e),
                    };

                if ies_length > mpdu_ies_and_payload_length {
                    return Err(Error);
                }

                let frame_payload_len = mpdu_ies_and_payload_length - ies_length;

                if (frame_payload_len > 0) != has_frame_payload {
                    return Err(Error);
                }

                (ies_length, frame_payload_len)
            }
            IeListRepr::WithoutTerminationIes(_) => {
                // The frame payload length is non-deterministic in this
                // case as the payload termination IE is optional.
                return Err(Error);
            }
        };
        Ok(ies_and_frame_payload_len)
    }
}
