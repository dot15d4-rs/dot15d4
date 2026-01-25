pub mod header;
pub mod nested;
pub mod payload;

use const_for::const_for;
use dot15d4_util::{Error, Result};

use self::header::{
    find_header_ie_content, find_header_ie_content_range, header_ie_id, HeaderIeHeader,
    HeaderIeIter, TimeCorrectionIe,
};
use self::nested::{
    find_nested_ie_content, find_nested_ie_content_range, nested_ie_id, ChannelHoppingIe,
    LinkDescriptor, NestedIeIter, NestedIeLongHeader, NestedIeShortHeader, SlotframeDescriptor,
    TschSlotframeLinkIe, TschSyncIe, TschTimeslotIe,
};
use self::payload::{find_mlme_content_range, payload_ie_id, PayloadIeHeader};

// ============================================================================
// IE Fields Top-Level Accessor
// ============================================================================

/// Top-level accessor for IE fields in an MPDU.
///
/// All Header IE, Payload IE, and Nested IE accessors are methods on this type.
#[derive(Clone, Copy, Debug)]
pub struct IesFields<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> IesFields<Bytes> {
    #[inline]
    pub fn new(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes.as_ref()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.bytes.as_ref().len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bytes.as_ref().is_empty()
    }

    // -------------------------------------------------------------------------
    // Header IE Iteration
    // -------------------------------------------------------------------------

    /// Iterate over Header IEs.
    #[inline]
    pub fn header_ies(&self) -> HeaderIeIter<'_> {
        HeaderIeIter::new(self.bytes.as_ref())
    }

    // -------------------------------------------------------------------------
    // MLME / Nested IE Access
    // -------------------------------------------------------------------------

    /// Find the MLME Payload IE content (contains nested IEs).
    #[inline]
    pub fn mlme_content(&self) -> Option<&[u8]> {
        let buf = self.bytes.as_ref();
        let (start, end) = find_mlme_content_range(buf)?;
        Some(&buf[start..end])
    }

    /// Iterate over Nested IEs within MLME.
    #[inline]
    pub fn nested_ies(&self) -> Option<NestedIeIter<'_>> {
        self.mlme_content().map(NestedIeIter::new)
    }

    // -------------------------------------------------------------------------
    // Header IE Accessors (Read-only)
    // -------------------------------------------------------------------------

    /// Find Time Correction Header IE.
    #[inline]
    pub fn time_correction(&self) -> Option<TimeCorrectionIe<&[u8]>> {
        let buf = self.bytes.as_ref();
        let content = find_header_ie_content(buf, header_ie_id::TIME_CORRECTION)?;
        TimeCorrectionIe::new(content)
    }

    // -------------------------------------------------------------------------
    // Nested IE Accessors (Read-only)
    // -------------------------------------------------------------------------

    /// Find TSCH Synchronization Nested IE.
    #[inline]
    pub fn tsch_sync(&self) -> Option<TschSyncIe<&[u8]>> {
        let buf = self.bytes.as_ref();
        let content = find_nested_ie_content(buf, nested_ie_id::TSCH_SYNC, false)?;
        TschSyncIe::new(content)
    }

    /// Find TSCH Timeslot Nested IE.
    #[inline]
    pub fn tsch_timeslot(&self) -> Option<TschTimeslotIe<&[u8]>> {
        let buf = self.bytes.as_ref();
        let content = find_nested_ie_content(buf, nested_ie_id::TSCH_TIMESLOT, false)?;
        TschTimeslotIe::new(content)
    }

    /// Find TSCH Slotframe and Link Nested IE.
    #[inline]
    pub fn tsch_slotframe_link(&self) -> Option<TschSlotframeLinkIe<&[u8]>> {
        let buf = self.bytes.as_ref();
        let content = find_nested_ie_content(buf, nested_ie_id::TSCH_SLOTFRAME_LINK, false)?;
        TschSlotframeLinkIe::new(content)
    }

    /// Find Channel Hopping Nested IE.
    #[inline]
    pub fn channel_hopping(&self) -> Option<ChannelHoppingIe<&[u8]>> {
        let buf = self.bytes.as_ref();
        let content = find_nested_ie_content(buf, nested_ie_id::CHANNEL_HOPPING, true)?;
        ChannelHoppingIe::new(content)
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> IesFields<Bytes> {
    #[inline]
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        self.bytes.as_mut()
    }

    /// Get mutable Time Correction Header IE.
    #[inline]
    pub fn time_correction_mut(&mut self) -> Option<TimeCorrectionIe<&mut [u8]>> {
        let (start, end) = {
            let buf = self.bytes.as_ref();
            find_header_ie_content_range(buf, header_ie_id::TIME_CORRECTION)?
        };
        TimeCorrectionIe::new(&mut self.bytes.as_mut()[start..end])
    }

    /// Get mutable TSCH Sync IE.
    #[inline]
    pub fn tsch_sync_mut(&mut self) -> Option<TschSyncIe<&mut [u8]>> {
        let (start, end) = {
            let buf = self.bytes.as_ref();
            find_nested_ie_content_range(buf, nested_ie_id::TSCH_SYNC, false)?
        };
        TschSyncIe::new(&mut self.bytes.as_mut()[start..end])
    }

    /// Get mutable TSCH Timeslot IE.
    #[inline]
    pub fn tsch_timeslot_mut(&mut self) -> Option<TschTimeslotIe<&mut [u8]>> {
        let (start, end) = {
            let buf = self.bytes.as_ref();
            find_nested_ie_content_range(buf, nested_ie_id::TSCH_TIMESLOT, false)?
        };
        TschTimeslotIe::new(&mut self.bytes.as_mut()[start..end])
    }

    /// Get mutable TSCH Slotframe and Link IE.
    #[inline]
    pub fn tsch_slotframe_link_mut(&mut self) -> Option<TschSlotframeLinkIe<&mut [u8]>> {
        let (start, end) = {
            let buf = self.bytes.as_ref();
            find_nested_ie_content_range(buf, nested_ie_id::TSCH_SLOTFRAME_LINK, false)?
        };
        TschSlotframeLinkIe::new(&mut self.bytes.as_mut()[start..end])
    }

    /// Get mutable Channel Hopping IE.
    #[inline]
    pub fn channel_hopping_mut(&mut self) -> Option<ChannelHoppingIe<&mut [u8]>> {
        let (start, end) = {
            let buf = self.bytes.as_ref();
            find_nested_ie_content_range(buf, nested_ie_id::CHANNEL_HOPPING, true)?
        };
        ChannelHoppingIe::new(&mut self.bytes.as_mut()[start..end])
    }
}

/// Initialize IE headers in a buffer based on IeRepr list.
///
/// This writes all IE headers (Header IEs, MLME Payload IE, Nested IEs, terminations)
/// into the buffer. Content areas are left for the caller to populate.
///
/// Returns the total bytes written.
pub fn init_ie_headers(buf: &mut [u8], ies: &[IeRepr<'_>], has_frame_payload: bool) -> usize {
    let mut offset = 0;
    let mut has_header_ie = false;
    let mut has_nested_ie = false;
    let mut nested_ie_total_len: u16 = 0;

    // First pass: calculate nested IE total length and write header IEs
    for ie in ies {
        let (header_len, nested_len) = ie.length();

        if header_len > 0 {
            has_header_ie = true;
            // Write header IE
            match ie {
                IeRepr::TimeCorrectionHeaderIe => {
                    if offset + 4 <= buf.len() {
                        let mut header =
                            HeaderIeHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(header_ie_id::TIME_CORRECTION, 2);
                        offset += 4;
                    }
                }
                _ => {}
            }
        } else if nested_len > 0 {
            has_nested_ie = true;
            nested_ie_total_len += nested_len;
        }
    }

    // Determine termination needs
    let needs_ht1 = has_nested_ie;
    let needs_ht2 = has_header_ie && !has_nested_ie && has_frame_payload;
    let needs_pt = has_nested_ie && has_frame_payload;

    if needs_ht1 {
        // Write HT1
        if offset + 2 <= buf.len() {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf[offset..offset + 2]);
            header.init(header_ie_id::HT1, 0);
            offset += 2;
        }

        // Write MLME Payload IE header
        if offset + 2 <= buf.len() {
            let mut header = PayloadIeHeader::new_unchecked(&mut buf[offset..offset + 2]);
            header.init(payload_ie_id::MLME, nested_ie_total_len);
            offset += 2;
        }

        // Write nested IE headers
        for ie in ies {
            match ie {
                IeRepr::TschSynchronizationNestedIe => {
                    if offset + 2 <= buf.len() {
                        let mut header =
                            NestedIeShortHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::TSCH_SYNC, 6);
                        offset += 8; // header + content
                    }
                }
                IeRepr::ReducedTschTimeslotNestedIe => {
                    if offset + 2 <= buf.len() {
                        let mut header =
                            NestedIeShortHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::TSCH_TIMESLOT, 1);
                        offset += 3;
                    }
                }
                IeRepr::FullTschTimeslotNestedIe => {
                    if offset + 2 <= buf.len() {
                        let mut header =
                            NestedIeShortHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::TSCH_TIMESLOT, 25);
                        offset += 27;
                    }
                }
                IeRepr::TschSlotframeAndLinkNestedIe(slotframes) => {
                    let (_, nested_len) = ie.length();
                    let content_len = (nested_len - 2) as usize; // minus header
                    if offset + 2 + content_len <= buf.len() {
                        // Write nested IE header
                        let mut header =
                            NestedIeShortHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::TSCH_SLOTFRAME_LINK, content_len as u8);

                        // Initialize content: num_slotframes + each slotframe descriptor
                        let content_start = offset + 2;
                        buf[content_start] = slotframes.len() as u8; // num_slotframes

                        let mut sf_offset = content_start + 1;
                        for &num_links in slotframes.iter() {
                            if sf_offset + SlotframeDescriptor::<&[u8]>::HEADER_LENGTH <= buf.len()
                            {
                                buf[sf_offset + 3] = num_links; // num_links (pre-set)

                                // Skip past slotframe header + links
                                sf_offset += SlotframeDescriptor::<&[u8]>::HEADER_LENGTH
                                    + (num_links as usize) * LinkDescriptor::<&[u8]>::LENGTH;
                            }
                        }

                        offset += 2 + content_len;
                    }
                }
                IeRepr::ReducedChannelHoppingNestedIe => {
                    if offset + 2 <= buf.len() {
                        let mut header =
                            NestedIeLongHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::CHANNEL_HOPPING, 1);
                        offset += 3;
                    }
                }
                IeRepr::FullChannelHoppingNestedIe(_num_channels, _is_sun_phy) => {
                    let (_, nested_len) = ie.length();
                    let content_len = nested_len - 2;
                    if offset + 2 <= buf.len() {
                        let mut header =
                            NestedIeLongHeader::new_unchecked(&mut buf[offset..offset + 2]);
                        header.init(nested_ie_id::CHANNEL_HOPPING, content_len);
                        offset += 2 + content_len as usize;
                    }
                }
                _ => {} // Header IEs already handled
            }
        }

        // Write PT if needed
        if needs_pt && offset + 2 <= buf.len() {
            let mut header = PayloadIeHeader::new_unchecked(&mut buf[offset..offset + 2]);
            header.init(payload_ie_id::TERMINATION, 0);
            offset += 2;
        }
    } else if needs_ht2 && offset + 2 <= buf.len() {
        let mut header = HeaderIeHeader::new_unchecked(&mut buf[offset..offset + 2]);
        header.init(header_ie_id::HT2, 0);
        offset += 2;
    }

    offset
}

/// Compute IE length from buffer by parsing header and payload IEs.
/// Returns `(ies_length, frame_payload_length)`.
pub fn compute_ies_length_from_buffer(
    buf: &[u8],
    total_ies_and_payload_len: u16,
) -> Result<(u16, u16)> {
    let mut offset = 0usize;
    let mut termination_type: Option<u8> = None;

    // Parse header IEs
    let mut iter = HeaderIeIter::new(buf);
    while let Some(ie) = iter.next() {
        offset = iter.offset();

        if ie.is_termination() {
            termination_type = Some(ie.element_id());
            break;
        }
    }

    // Determine if there are payload IEs and frame payload
    let ies_length = match termination_type {
        None => {
            // No termination IE - only header IEs, no payload IEs, no frame payload
            // Per IEEE 802.15.4-2024 section 7.4.1: Header IEs only with no termination
            // means no frame payload follows
            offset as u16
        }
        Some(header_ie_id::HT1) => {
            // HT1 - payload IEs follow
            // Parse payload IEs to find their total length
            while offset + PayloadIeHeader::<&[u8]>::LENGTH <= buf.len() {
                let Some(header) = PayloadIeHeader::new(&buf[offset..]) else {
                    break;
                };
                let content_len = header.length() as usize;
                offset += PayloadIeHeader::<&[u8]>::LENGTH + content_len;

                if header.is_termination() {
                    break;
                }
            }
            offset as u16
        }
        Some(header_ie_id::HT2) => {
            // HT2 - no payload IEs but frame payload follows
            offset as u16
        }
        Some(_) => return Err(Error),
    };

    if ies_length > total_ies_and_payload_len {
        return Err(Error);
    }

    let frame_payload_length = total_ies_and_payload_len - ies_length;

    Ok((ies_length, frame_payload_length))
}

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

    /// Returns the inner slice of IE representations.
    pub const fn as_slice(&self) -> &'ies [IeRepr] {
        self.0
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
    pub const fn try_ies_and_frame_payload_length(
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
#[cfg(test)]
mod tests {
    use crate::radio::frame::ies::nested::{link_options, LinkDescriptor};

    use super::*;

    #[test]
    fn test_ie_fields_iteration() {
        // Build a frame with Time Correction + HT1 + MLME(TSCH Sync)
        let frame = [
            // Header IE: Time Correction (id=0x1e, len=2)
            0x02, 0x0F, 0x64, 0x00, // Header IE: HT1 (id=0x7e, len=0)
            0x00, 0x3F, // Payload IE: MLME (group=1, len=8)
            0x08, 0x88, // Nested IE: TSCH Sync (short, id=0x1a, len=6)
            0x06, 0x1A, 0x05, 0x04, 0x03, 0x02, 0x01, 0x07,
        ];

        let ie_fields = IesFields::new(&frame[..]);

        // Test header IE iteration
        let mut count = 0;
        for ie in ie_fields.header_ies() {
            count += 1;
            if ie.is_termination() {
                break;
            }
        }
        assert_eq!(count, 2); // Time Correction + HT1

        // Test direct accessors
        let tc = ie_fields.time_correction().unwrap();
        assert_eq!(tc.time_sync(), 100);

        let sync = ie_fields.tsch_sync().unwrap();
        assert_eq!(sync.asn(), 0x0102030405);
        assert_eq!(sync.join_metric(), 7);
    }

    #[test]
    fn test_init_ie_headers() {
        let mut buf = [0u8; 64];

        let ies = [
            IeRepr::TimeCorrectionHeaderIe,
            IeRepr::TschSynchronizationNestedIe,
        ];

        let written = init_ie_headers(&mut buf, &ies, false);

        // Should have: TimeCorrectionHeader(2) + content(2) + HT1(2) + MLME header(2) + TschSyncHeader(2) + content(6)
        assert_eq!(written, 4 + 2 + 2 + 2 + 6);

        // Verify Time Correction header
        let tc_header = HeaderIeHeader::new(&buf[0..2]).unwrap();
        assert_eq!(tc_header.element_id(), header_ie_id::TIME_CORRECTION);
        assert_eq!(tc_header.length(), 2);

        // Verify HT1
        let ht1_header = HeaderIeHeader::new(&buf[4..6]).unwrap();
        assert_eq!(ht1_header.element_id(), header_ie_id::HT1);

        // Verify MLME
        let mlme_header = PayloadIeHeader::new(&buf[6..8]).unwrap();
        assert_eq!(mlme_header.group_id(), payload_ie_id::MLME);
        assert_eq!(mlme_header.length(), 8); // nested header(2) + content(6)

        // Verify TSCH Sync nested header
        let sync_header = NestedIeShortHeader::new(&buf[8..10]).unwrap();
        assert_eq!(sync_header.sub_id(), nested_ie_id::TSCH_SYNC);
        assert_eq!(sync_header.length(), 6);
    }
}

