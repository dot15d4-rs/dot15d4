// ============================================================================
// IE Type Constants
// ============================================================================

use super::payload::find_mlme_content_range;

pub mod nested_ie_id {
    //! MLME Nested IE Sub-IDs (Short format)
    pub const TSCH_SYNC: u8 = 0x1a;
    pub const TSCH_SLOTFRAME_LINK: u8 = 0x1b;
    pub const TSCH_TIMESLOT: u8 = 0x1c;
    pub const HOPPING_TIMING: u8 = 0x1d;

    // Long format
    pub const CHANNEL_HOPPING: u8 = 0x09;
}

pub mod link_options {
    //! Link option bit masks
    pub const TX: u8 = 0x01;
    pub const RX: u8 = 0x02;
    pub const SHARED: u8 = 0x04;
    pub const TIMEKEEPING: u8 = 0x08;
    pub const PRIORITY: u8 = 0x10;
}

/// Nested IE descriptor - Short format (2 bytes).
///
/// Format (little-endian):
/// - Bits 0-7: Length (0-255)
/// - Bits 8-14: Sub-ID
/// - Bit 15: Type (0 = Short)
#[derive(Clone, Copy)]
pub struct NestedIeShortHeader<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> NestedIeShortHeader<Bytes> {
    pub const LENGTH: usize = 2;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::LENGTH {
            return None;
        }
        let header = Self { bytes };
        if header.ie_type() != 0 {
            return None;
        }
        Some(header)
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn length(&self) -> u8 {
        self.bytes.as_ref()[0]
    }

    #[inline]
    pub fn sub_id(&self) -> u8 {
        self.bytes.as_ref()[1] & 0x7F
    }

    #[inline]
    pub fn ie_type(&self) -> u8 {
        (self.bytes.as_ref()[1] >> 7) & 0x01
    }

    #[inline]
    pub fn total_length(&self) -> usize {
        Self::LENGTH + self.length() as usize
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> NestedIeShortHeader<Bytes> {
    #[inline]
    pub fn set_length(&mut self, length: u8) {
        self.bytes.as_mut()[0] = length;
    }

    #[inline]
    pub fn set_sub_id(&mut self, id: u8) {
        let b = self.bytes.as_mut();
        b[1] = (b[1] & 0x80) | (id & 0x7F);
    }

    #[inline]
    pub fn set_ie_type(&mut self, ie_type: u8) {
        let b = self.bytes.as_mut();
        b[1] = (b[1] & 0x7F) | ((ie_type & 0x01) << 7);
    }

    #[inline]
    pub fn init(&mut self, sub_id: u8, length: u8) {
        self.set_ie_type(0);
        self.set_sub_id(sub_id);
        self.set_length(length);
    }
}

/// Nested IE descriptor - Long format (2 bytes).
///
/// Format (little-endian):
/// - Bits 0-10: Length (0-2047)
/// - Bits 11-14: Sub-ID
/// - Bit 15: Type (1 = Long)
#[derive(Clone, Copy)]
pub struct NestedIeLongHeader<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> NestedIeLongHeader<Bytes> {
    pub const LENGTH: usize = 2;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::LENGTH {
            return None;
        }
        let header = Self { bytes };
        if header.ie_type() != 1 {
            return None;
        }
        Some(header)
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    fn raw(&self) -> u16 {
        let b = self.bytes.as_ref();
        u16::from_le_bytes([b[0], b[1]])
    }

    #[inline]
    pub fn length(&self) -> u16 {
        self.raw() & 0x07FF
    }

    #[inline]
    pub fn sub_id(&self) -> u8 {
        ((self.raw() >> 11) & 0x0F) as u8
    }

    #[inline]
    pub fn ie_type(&self) -> u8 {
        ((self.raw() >> 15) & 0x01) as u8
    }

    #[inline]
    pub fn total_length(&self) -> usize {
        Self::LENGTH + self.length() as usize
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> NestedIeLongHeader<Bytes> {
    #[inline]
    pub fn set_length(&mut self, length: u16) {
        debug_assert!(length <= 2047);
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !0x07FF) | (length & 0x07FF);
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }

    #[inline]
    pub fn set_sub_id(&mut self, id: u8) {
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !(0x0F << 11)) | ((id as u16 & 0x0F) << 11);
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }

    #[inline]
    pub fn set_ie_type(&mut self, ie_type: u8) {
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !(1 << 15)) | ((ie_type as u16 & 1) << 15);
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }

    #[inline]
    pub fn init(&mut self, sub_id: u8, length: u16) {
        self.set_ie_type(1);
        self.set_sub_id(sub_id);
        self.set_length(length);
    }
}

/// TSCH Synchronization Nested IE content (6 bytes).
///
/// Format:
/// - Bytes 0-4: ASN (5 bytes, little-endian)
/// - Byte 5: Join Metric
#[derive(Clone, Copy)]
pub struct TschSyncIe<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> TschSyncIe<Bytes> {
    pub const LENGTH: usize = 6;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::LENGTH {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    /// Absolute Slot Number (40-bit).
    #[inline]
    pub fn asn(&self) -> u64 {
        let b = self.bytes.as_ref();
        u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], 0, 0, 0])
    }

    /// Join metric (hop count).
    #[inline]
    pub fn join_metric(&self) -> u8 {
        self.bytes.as_ref()[5]
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> TschSyncIe<Bytes> {
    #[inline]
    pub fn set_asn(&mut self, asn: u64) {
        let bytes = asn.to_le_bytes();
        self.bytes.as_mut()[0..5].copy_from_slice(&bytes[0..5]);
    }

    #[inline]
    pub fn set_join_metric(&mut self, metric: u8) {
        self.bytes.as_mut()[5] = metric;
    }
}

/// TSCH Timeslot Nested IE content.
///
/// Reduced format: 1 byte (just ID)
/// Full format: 25 bytes (all timing parameters)
#[derive(Clone, Copy)]
pub struct TschTimeslotIe<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> TschTimeslotIe<Bytes> {
    pub const REDUCED_LENGTH: usize = 1;
    pub const FULL_LENGTH: usize = 25;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().is_empty() {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn is_reduced(&self) -> bool {
        self.bytes.as_ref().len() < Self::FULL_LENGTH
    }

    #[inline]
    pub fn timeslot_id(&self) -> u8 {
        self.bytes.as_ref()[0]
    }

    #[inline]
    fn u16_at(&self, offset: usize) -> Option<u16> {
        let b = self.bytes.as_ref();
        if b.len() >= offset + 2 {
            Some(u16::from_le_bytes([b[offset], b[offset + 1]]))
        } else {
            None
        }
    }

    // Full format field accessors
    #[inline]
    pub fn cca_offset(&self) -> Option<u16> {
        self.u16_at(1)
    }
    #[inline]
    pub fn cca(&self) -> Option<u16> {
        self.u16_at(3)
    }
    #[inline]
    pub fn tx_offset(&self) -> Option<u16> {
        self.u16_at(5)
    }
    #[inline]
    pub fn rx_offset(&self) -> Option<u16> {
        self.u16_at(7)
    }
    #[inline]
    pub fn rx_ack_delay(&self) -> Option<u16> {
        self.u16_at(9)
    }
    #[inline]
    pub fn tx_ack_delay(&self) -> Option<u16> {
        self.u16_at(11)
    }
    #[inline]
    pub fn rx_wait(&self) -> Option<u16> {
        self.u16_at(13)
    }
    #[inline]
    pub fn ack_wait(&self) -> Option<u16> {
        self.u16_at(15)
    }
    #[inline]
    pub fn rx_tx(&self) -> Option<u16> {
        self.u16_at(17)
    }
    #[inline]
    pub fn max_ack(&self) -> Option<u16> {
        self.u16_at(19)
    }

    #[inline]
    pub fn max_tx(&self) -> Option<u16> {
        let b = self.bytes.as_ref();
        // TODO: support 3-bytes value
        if b.len() >= 24 {
            self.u16_at(21)
        } else {
            None
        }
    }

    #[inline]
    pub fn timeslot_length(&self) -> Option<u16> {
        let b = self.bytes.as_ref();
        // TODO: support 3-bytes value
        if b.len() >= Self::FULL_LENGTH {
            Some(u16::from_le_bytes([b[23], b[24]]))
        } else {
            None
        }
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> TschTimeslotIe<Bytes> {
    #[inline]
    pub fn set_timeslot_id(&mut self, id: u8) {
        self.bytes.as_mut()[0] = id;
    }

    #[inline]
    fn set_u16_at(&mut self, offset: usize, value: u16) {
        let b = self.bytes.as_mut();
        if b.len() >= offset + 2 {
            b[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
    }

    #[inline]
    pub fn set_cca_offset(&mut self, v: u16) {
        self.set_u16_at(1, v);
    }
    #[inline]
    pub fn set_cca(&mut self, v: u16) {
        self.set_u16_at(3, v);
    }
    #[inline]
    pub fn set_tx_offset(&mut self, v: u16) {
        self.set_u16_at(5, v);
    }
    #[inline]
    pub fn set_rx_offset(&mut self, v: u16) {
        self.set_u16_at(7, v);
    }
    #[inline]
    pub fn set_rx_ack_delay(&mut self, v: u16) {
        self.set_u16_at(9, v);
    }
    #[inline]
    pub fn set_tx_ack_delay(&mut self, v: u16) {
        self.set_u16_at(11, v);
    }
    #[inline]
    pub fn set_rx_wait(&mut self, v: u16) {
        self.set_u16_at(13, v);
    }
    #[inline]
    pub fn set_ack_wait(&mut self, v: u16) {
        self.set_u16_at(15, v);
    }
    #[inline]
    pub fn set_rx_tx(&mut self, v: u16) {
        self.set_u16_at(17, v);
    }
    #[inline]
    pub fn set_max_ack(&mut self, v: u16) {
        self.set_u16_at(19, v);
    }

    #[inline]
    pub fn set_max_tx(&mut self, v: u16) {
        let b = self.bytes.as_mut();
        // TODO: support 3-bytes value
        if b.len() >= 24 {
            self.set_u16_at(21, v);
        }
    }

    #[inline]
    pub fn set_timeslot_length(&mut self, v: u16) {
        let b = self.bytes.as_mut();
        // TODO: support 3-bytes value
        if b.len() >= Self::FULL_LENGTH {
            b[23..25].copy_from_slice(&v.to_le_bytes());
        }
    }
}

/// Channel Hopping Nested IE content (Long format, variable length).
#[derive(Clone, Copy)]
pub struct ChannelHoppingIe<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> ChannelHoppingIe<Bytes> {
    pub const MIN_LENGTH: usize = 1;
    pub const REDUCED_LENGTH: usize = 1;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().is_empty() {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn is_reduced(&self) -> bool {
        self.bytes.as_ref().len() == Self::REDUCED_LENGTH
    }

    /// Hopping sequence ID (0 = default).
    #[inline]
    pub fn hopping_sequence_id(&self) -> u8 {
        self.bytes.as_ref()[0]
    }

    /// Channel page (full format only).
    #[inline]
    pub fn channel_page(&self) -> Option<u8> {
        let b = self.bytes.as_ref();
        if b.len() >= 2 {
            Some(b[1])
        } else {
            None
        }
    }

    /// Number of channels (full format only).
    #[inline]
    pub fn num_channels(&self) -> Option<u16> {
        let b = self.bytes.as_ref();
        if b.len() >= 4 {
            Some(u16::from_le_bytes([b[2], b[3]]))
        } else {
            None
        }
    }

    /// PHY configuration (full format only).
    #[inline]
    pub fn phy_configuration(&self) -> Option<u32> {
        let b = self.bytes.as_ref();
        if b.len() >= 8 {
            Some(u32::from_le_bytes([b[4], b[5], b[6], b[7]]))
        } else {
            None
        }
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> ChannelHoppingIe<Bytes> {
    #[inline]
    pub fn set_hopping_sequence_id(&mut self, id: u8) {
        self.bytes.as_mut()[0] = id;
    }

    #[inline]
    pub fn set_channel_page(&mut self, page: u8) {
        let b = self.bytes.as_mut();
        if b.len() >= 2 {
            b[1] = page;
        }
    }

    #[inline]
    pub fn set_num_channels(&mut self, num: u16) {
        let b = self.bytes.as_mut();
        if b.len() >= 4 {
            b[2..4].copy_from_slice(&num.to_le_bytes());
        }
    }

    #[inline]
    pub fn set_phy_configuration(&mut self, config: u32) {
        let b = self.bytes.as_mut();
        if b.len() >= 8 {
            b[4..8].copy_from_slice(&config.to_le_bytes());
        }
    }
}

/// Link descriptor within TSCH Slotframe and Link IE (5 bytes).
#[derive(Clone, Copy)]
pub struct LinkDescriptor<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> LinkDescriptor<Bytes> {
    pub const LENGTH: usize = 5;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::LENGTH {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn timeslot(&self) -> u16 {
        let b = self.bytes.as_ref();
        u16::from_le_bytes([b[0], b[1]])
    }

    #[inline]
    pub fn channel_offset(&self) -> u16 {
        let b = self.bytes.as_ref();
        u16::from_le_bytes([b[2], b[3]])
    }

    #[inline]
    pub fn options(&self) -> u8 {
        self.bytes.as_ref()[4]
    }

    #[inline]
    pub fn is_tx(&self) -> bool {
        self.options() & link_options::TX != 0
    }
    #[inline]
    pub fn is_rx(&self) -> bool {
        self.options() & link_options::RX != 0
    }
    #[inline]
    pub fn is_shared(&self) -> bool {
        self.options() & link_options::SHARED != 0
    }
    #[inline]
    pub fn is_timekeeping(&self) -> bool {
        self.options() & link_options::TIMEKEEPING != 0
    }
    #[inline]
    pub fn is_priority(&self) -> bool {
        self.options() & link_options::PRIORITY != 0
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> LinkDescriptor<Bytes> {
    #[inline]
    pub fn set_timeslot(&mut self, ts: u16) {
        self.bytes.as_mut()[0..2].copy_from_slice(&ts.to_le_bytes());
    }

    #[inline]
    pub fn set_channel_offset(&mut self, co: u16) {
        self.bytes.as_mut()[2..4].copy_from_slice(&co.to_le_bytes());
    }

    #[inline]
    pub fn set_options(&mut self, opts: u8) {
        self.bytes.as_mut()[4] = opts;
    }
}

/// Slotframe descriptor within TSCH Slotframe and Link IE.
#[derive(Clone, Copy)]
pub struct SlotframeDescriptor<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> SlotframeDescriptor<Bytes> {
    pub const HEADER_LENGTH: usize = 4;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::HEADER_LENGTH {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn handle(&self) -> u8 {
        self.bytes.as_ref()[0]
    }

    #[inline]
    pub fn size(&self) -> u16 {
        let b = self.bytes.as_ref();
        u16::from_le_bytes([b[1], b[2]])
    }

    #[inline]
    pub fn num_links(&self) -> u8 {
        self.bytes.as_ref()[3]
    }

    /// Total length including header and all links.
    #[inline]
    pub fn total_length(&self) -> usize {
        Self::HEADER_LENGTH + self.num_links() as usize * LinkDescriptor::<&[u8]>::LENGTH
    }

    /// Get a read-only iterator over links.
    #[inline]
    pub fn links(&self) -> LinkIter<'_>
    where
        Bytes: AsRef<[u8]>,
    {
        let b = self.bytes.as_ref();
        let links_start = Self::HEADER_LENGTH;
        let links_end = links_start + self.num_links() as usize * LinkDescriptor::<&[u8]>::LENGTH;
        LinkIter {
            buf: &b[links_start..links_end.min(b.len())],
            remaining: self.num_links(),
        }
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> SlotframeDescriptor<Bytes> {
    #[inline]
    pub fn set_handle(&mut self, handle: u8) {
        self.bytes.as_mut()[0] = handle;
    }

    #[inline]
    pub fn set_size(&mut self, size: u16) {
        self.bytes.as_mut()[1..3].copy_from_slice(&size.to_le_bytes());
    }

    #[inline]
    pub fn set_num_links(&mut self, num: u8) {
        self.bytes.as_mut()[3] = num;
    }

    /// Get a mutable iterator over links.
    #[inline]
    pub fn links_mut(&mut self) -> LinkIterMut<'_> {
        // Read num_links from immutable view first
        let num_links = self.bytes.as_ref()[3];
        let links_start = Self::HEADER_LENGTH;
        let links_end = links_start + num_links as usize * LinkDescriptor::<&[u8]>::LENGTH;
        let buf = self.bytes.as_mut();
        let buf_len = buf.len();
        LinkIterMut {
            buf: &mut buf[links_start..links_end.min(buf_len)],
            remaining: num_links,
        }
    }

    /// Get a specific link by index (mutable).
    #[inline]
    pub fn link_mut(&mut self, index: u8) -> Option<LinkDescriptor<&mut [u8]>> {
        if index >= self.num_links() {
            return None;
        }
        let b = self.bytes.as_mut();
        let start = Self::HEADER_LENGTH + index as usize * LinkDescriptor::<&[u8]>::LENGTH;
        let end = start + LinkDescriptor::<&[u8]>::LENGTH;
        if end > b.len() {
            return None;
        }
        Some(LinkDescriptor::new_unchecked(&mut b[start..end]))
    }
}

/// TSCH Slotframe and Link Nested IE content.
#[derive(Clone, Copy)]
pub struct TschSlotframeLinkIe<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> TschSlotframeLinkIe<Bytes> {
    pub const MIN_LENGTH: usize = 1;

    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().is_empty() {
            return None;
        }
        Some(Self { bytes })
    }

    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    #[inline]
    pub fn num_slotframes(&self) -> u8 {
        self.bytes.as_ref()[0]
    }

    /// Get an iterator over slotframe descriptors.
    #[inline]
    pub fn slotframes(&self) -> SlotframeIter<'_> {
        SlotframeIter {
            buf: &self.bytes.as_ref()[1..],
            slotframes_remaining: self.num_slotframes(),
        }
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> TschSlotframeLinkIe<Bytes> {
    #[inline]
    pub fn set_num_slotframes(&mut self, num: u8) {
        self.bytes.as_mut()[0] = num;
    }
    /// Get a mutable iterator over slotframe descriptors.
    #[inline]
    pub fn slotframes_mut(&mut self) -> SlotframeIterMut<'_> {
        let num = self.num_slotframes();
        SlotframeIterMut {
            buf: &mut self.bytes.as_mut()[1..],
            slotframes_remaining: num,
        }
    }
}

/// Iterator over slotframe descriptors.
pub struct SlotframeIter<'a> {
    buf: &'a [u8],
    slotframes_remaining: u8,
}

impl<'a> Iterator for SlotframeIter<'a> {
    type Item = SlotframeDescriptor<&'a [u8]>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.slotframes_remaining == 0
            || self.buf.len() < SlotframeDescriptor::<&[u8]>::HEADER_LENGTH
        {
            return None;
        }

        let sf = SlotframeDescriptor::new_unchecked(self.buf);
        let total = sf.total_length();

        if self.buf.len() < total {
            return None;
        }

        let result = SlotframeDescriptor::new_unchecked(&self.buf[..total]);
        self.buf = &self.buf[total..];
        self.slotframes_remaining -= 1;
        Some(result)
    }
}
/// Mutable iterator over slotframe descriptors.
pub struct SlotframeIterMut<'a> {
    buf: &'a mut [u8],
    slotframes_remaining: u8,
}

impl<'a> SlotframeIterMut<'a> {
    /// Get the next slotframe descriptor mutably.
    ///
    /// Note: This consumes the iterator portion for that slotframe.
    pub fn next(&mut self) -> Option<SlotframeDescriptor<&mut [u8]>> {
        if self.slotframes_remaining == 0
            || self.buf.len() < SlotframeDescriptor::<&[u8]>::HEADER_LENGTH
        {
            return None;
        }

        // Read header to get total length
        let num_links = self.buf[3];
        let total = SlotframeDescriptor::<&[u8]>::HEADER_LENGTH
            + num_links as usize * LinkDescriptor::<&[u8]>::LENGTH;

        if self.buf.len() < total {
            return None;
        }

        // Split buffer
        let (current, rest) = core::mem::take(&mut self.buf).split_at_mut(total);
        self.buf = rest;
        self.slotframes_remaining -= 1;

        Some(SlotframeDescriptor::new_unchecked(current))
    }
}

/// Iterator over link descriptors within a slotframe.
pub struct LinkIter<'a> {
    buf: &'a [u8],
    remaining: u8,
}

impl<'a> LinkIter<'a> {
    pub fn new(slotframe: &SlotframeDescriptor<&'a [u8]>) -> Self {
        let links_start = SlotframeDescriptor::<&[u8]>::HEADER_LENGTH;
        Self {
            buf: &slotframe.bytes[links_start..],
            remaining: slotframe.num_links(),
        }
    }
}

/// Mutable iterator over link descriptors.
pub struct LinkIterMut<'a> {
    buf: &'a mut [u8],
    remaining: u8,
}

impl<'a> LinkIterMut<'a> {
    /// Get the next link descriptor mutably.
    pub fn next(&mut self) -> Option<LinkDescriptor<&mut [u8]>> {
        if self.remaining == 0 || self.buf.len() < LinkDescriptor::<&[u8]>::LENGTH {
            return None;
        }

        let (current, rest) =
            core::mem::take(&mut self.buf).split_at_mut(LinkDescriptor::<&[u8]>::LENGTH);
        self.buf = rest;
        self.remaining -= 1;

        Some(LinkDescriptor::new_unchecked(current))
    }
}

impl<'a> Iterator for LinkIter<'a> {
    type Item = LinkDescriptor<&'a [u8]>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 || self.buf.len() < LinkDescriptor::<&[u8]>::LENGTH {
            return None;
        }

        let link = LinkDescriptor::new_unchecked(&self.buf[..LinkDescriptor::<&[u8]>::LENGTH]);
        self.buf = &self.buf[LinkDescriptor::<&[u8]>::LENGTH..];
        self.remaining -= 1;
        Some(link)
    }
}

/// Parsed Nested IE.
#[derive(Clone, Copy)]
pub enum NestedIe<'a> {
    Short { sub_id: u8, content: &'a [u8] },
    Long { sub_id: u8, content: &'a [u8] },
}

impl<'a> NestedIe<'a> {
    #[inline]
    pub fn sub_id(&self) -> u8 {
        match self {
            NestedIe::Short { sub_id, .. } | NestedIe::Long { sub_id, .. } => *sub_id,
        }
    }

    #[inline]
    pub fn content(&self) -> &'a [u8] {
        match self {
            NestedIe::Short { content, .. } | NestedIe::Long { content, .. } => content,
        }
    }

    /// Try to interpret as TSCH Sync IE.
    #[inline]
    pub fn as_tsch_sync(&self) -> Option<TschSyncIe<&'a [u8]>> {
        if self.sub_id() == nested_ie_id::TSCH_SYNC {
            TschSyncIe::new(self.content())
        } else {
            None
        }
    }

    /// Try to interpret as TSCH Timeslot IE.
    #[inline]
    pub fn as_tsch_timeslot(&self) -> Option<TschTimeslotIe<&'a [u8]>> {
        if self.sub_id() == nested_ie_id::TSCH_TIMESLOT {
            TschTimeslotIe::new(self.content())
        } else {
            None
        }
    }

    /// Try to interpret as TSCH Slotframe and Link IE.
    #[inline]
    pub fn as_tsch_slotframe_link(&self) -> Option<TschSlotframeLinkIe<&'a [u8]>> {
        if self.sub_id() == nested_ie_id::TSCH_SLOTFRAME_LINK {
            TschSlotframeLinkIe::new(self.content())
        } else {
            None
        }
    }

    /// Try to interpret as Channel Hopping IE.
    #[inline]
    pub fn as_channel_hopping(&self) -> Option<ChannelHoppingIe<&'a [u8]>> {
        if self.sub_id() == nested_ie_id::CHANNEL_HOPPING {
            ChannelHoppingIe::new(self.content())
        } else {
            None
        }
    }
}

/// Iterator over Nested IEs within MLME content.
pub struct NestedIeIter<'a> {
    buf: &'a [u8],
    offset: usize,
}

impl<'a> NestedIeIter<'a> {
    #[inline]
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, offset: 0 }
    }
}

impl<'a> Iterator for NestedIeIter<'a> {
    type Item = NestedIe<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset + 2 > self.buf.len() {
            return None;
        }

        let is_long = self.buf[self.offset + 1] & 0x80 != 0;

        if is_long {
            let header = NestedIeLongHeader::new(&self.buf[self.offset..])?;
            let content_len = header.length() as usize;
            let content_start = self.offset + NestedIeLongHeader::<&[u8]>::LENGTH;
            let content_end = content_start + content_len;

            if content_end > self.buf.len() {
                return None;
            }

            let content = &self.buf[content_start..content_end];
            self.offset = content_end;

            Some(NestedIe::Long {
                sub_id: header.sub_id(),
                content,
            })
        } else {
            let header = NestedIeShortHeader::new(&self.buf[self.offset..])?;
            let content_len = header.length() as usize;
            let content_start = self.offset + NestedIeShortHeader::<&[u8]>::LENGTH;
            let content_end = content_start + content_len;

            if content_end > self.buf.len() {
                return None;
            }

            let content = &self.buf[content_start..content_end];
            self.offset = content_end;

            Some(NestedIe::Short {
                sub_id: header.sub_id(),
                content,
            })
        }
    }
}

/// Find a Nested IE content by sub-ID (returns slice).
pub fn find_nested_ie_content(buf: &[u8], sub_id: u8, is_long_format: bool) -> Option<&[u8]> {
    let (start, end) = find_nested_ie_content_range(buf, sub_id, is_long_format)?;
    Some(&buf[start..end])
}

/// Find a Nested IE content range by sub-ID.
pub fn find_nested_ie_content_range(
    ies_buf: &[u8],
    sub_id: u8,
    is_long_format: bool,
) -> Option<(usize, usize)> {
    let (mlme_start, mlme_end) = find_mlme_content_range(ies_buf)?;
    let mut offset = mlme_start;

    while offset + 2 <= mlme_end {
        let is_long = ies_buf[offset + 1] & 0x80 != 0;

        if is_long {
            let header = NestedIeLongHeader::new(&ies_buf[offset..])?;
            let content_len = header.length() as usize;
            let content_start = offset + NestedIeLongHeader::<&[u8]>::LENGTH;
            let content_end = content_start + content_len;

            if content_end > mlme_end {
                return None;
            }

            if is_long_format && header.sub_id() == sub_id {
                return Some((content_start, content_end));
            }

            offset = content_end;
        } else {
            let header = NestedIeShortHeader::new(&ies_buf[offset..])?;
            let content_len = header.length() as usize;
            let content_start = offset + NestedIeShortHeader::<&[u8]>::LENGTH;
            let content_end = content_start + content_len;

            if content_end > mlme_end {
                return None;
            }

            if !is_long_format && header.sub_id() == sub_id {
                return Some((content_start, content_end));
            }

            offset = content_end;
        }
    }

    None
}

/// Find a Nested IE content mutably by sub-ID (short format).
pub fn find_nested_ie_content_mut(ies_buf: &mut [u8], sub_id: u8) -> Option<&mut [u8]> {
    // First find MLME range
    let (mlme_start, mlme_end) = find_mlme_content_range(ies_buf)?;

    // Then find nested IE within MLME
    let mut offset = 0;
    let mlme_len = mlme_end - mlme_start;

    let (content_start, content_end) = loop {
        if offset + 2 > mlme_len {
            return None;
        }

        let is_long = ies_buf[mlme_start + offset + 1] & 0x80 != 0;

        if is_long {
            let header = NestedIeLongHeader::new(&ies_buf[mlme_start + offset..])?;
            let content_len = header.length() as usize;
            offset += NestedIeLongHeader::<&[u8]>::LENGTH + content_len;
        } else {
            let header = NestedIeShortHeader::new(&ies_buf[mlme_start + offset..])?;
            let content_len = header.length() as usize;
            let c_start = offset + NestedIeShortHeader::<&[u8]>::LENGTH;
            let c_end = c_start + content_len;

            if c_end > mlme_len {
                return None;
            }

            if header.sub_id() == sub_id {
                break (mlme_start + c_start, mlme_start + c_end);
            }

            offset = c_end;
        }
    };

    Some(&mut ies_buf[content_start..content_end])
}
#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_link_descriptor() {
        let mut buf = [0u8; 5];
        {
            let mut link = LinkDescriptor::new_unchecked(&mut buf);
            link.set_timeslot(10);
            link.set_channel_offset(3);
            link.set_options(link_options::TX | link_options::SHARED);
        }

        let link = LinkDescriptor::new(&buf).unwrap();
        assert_eq!(link.timeslot(), 10);
        assert_eq!(link.channel_offset(), 3);
        assert!(link.is_tx());
        assert!(!link.is_rx());
        assert!(link.is_shared());
    }

    #[test]
    fn test_nested_ie_short_header_new_valid() {
        let mut buf = [0u8; 2];
        {
            let mut header = NestedIeShortHeader::new_unchecked(&mut buf);
            header.init(nested_ie_id::TSCH_SYNC, 6);
        }

        let header = NestedIeShortHeader::new(&buf).unwrap();
        assert_eq!(header.ie_type(), 0);
        assert_eq!(header.sub_id(), nested_ie_id::TSCH_SYNC);
        assert_eq!(header.length(), 6);
        assert_eq!(header.total_length(), 8);
    }

    #[test]
    fn test_nested_ie_short_header_too_short() {
        let buf = [0u8; 1];
        assert!(NestedIeShortHeader::new(&buf).is_none());
    }

    #[test]
    fn test_nested_ie_short_header_wrong_type() {
        // Create a long format header (type=1)
        let mut buf = [0u8; 2];
        buf[1] = 0x80; // Set type bit to 1
        assert!(NestedIeShortHeader::new(&buf).is_none());
    }

    #[test]
    fn test_nested_ie_short_header_length_max() {
        let mut buf = [0u8; 2];
        let mut header = NestedIeShortHeader::new_unchecked(&mut buf);

        // Maximum length is 255 (8 bits)
        header.set_length(255);
        assert_eq!(header.length(), 255);
    }

    // =========================================================================
    // NestedIeLongHeader Tests
    // =========================================================================

    #[test]
    fn test_nested_ie_long_header_new_valid() {
        let mut buf = [0u8; 2];
        {
            let mut header = NestedIeLongHeader::new_unchecked(&mut buf);
            header.init(nested_ie_id::CHANNEL_HOPPING, 20);
        }

        let header = NestedIeLongHeader::new(&buf).unwrap();
        assert_eq!(header.ie_type(), 1);
        assert_eq!(header.sub_id(), nested_ie_id::CHANNEL_HOPPING);
        assert_eq!(header.length(), 20);
        assert_eq!(header.total_length(), 22);
    }

    #[test]
    fn test_nested_ie_long_header_too_short() {
        let buf = [0u8; 1];
        assert!(NestedIeLongHeader::new(&buf).is_none());
    }

    #[test]
    fn test_nested_ie_long_header_wrong_type() {
        // Create a short format header (type=0)
        let buf = [0u8; 2];
        assert!(NestedIeLongHeader::new(&buf).is_none());
    }

    #[test]
    fn test_nested_ie_long_header_length_max() {
        let mut buf = [0u8; 2];
        let mut header = NestedIeLongHeader::new_unchecked(&mut buf);

        // Maximum length is 2047 (11 bits)
        header.init(0, 2047);
        assert_eq!(header.length(), 2047);
    }

    #[test]
    fn test_tsch_sync_ie_new_valid() {
        let buf = [0u8; 6];
        let ie = TschSyncIe::new(&buf).unwrap();
        assert_eq!(ie.asn(), 0);
        assert_eq!(ie.join_metric(), 0);
    }

    #[test]
    fn test_tsch_sync_ie_new_too_short() {
        let buf = [0u8; 5];
        assert!(TschSyncIe::new(&buf).is_none());
    }

    #[test]
    fn test_tsch_sync_ie() {
        let mut buf = [0u8; 6];
        let mut ie = TschSyncIe::new_unchecked(&mut buf);

        ie.set_asn(0x1234567890);
        ie.set_join_metric(42);

        assert_eq!(ie.asn(), 0x1234567890);
        assert_eq!(ie.join_metric(), 42);
    }

    #[test]
    fn test_tsch_timeslot_ie_reduced() {
        let mut buf = [0u8; 1];
        let mut ie = TschTimeslotIe::new_unchecked(&mut buf);

        ie.set_timeslot_id(5);
        assert!(ie.is_reduced());
        assert_eq!(ie.timeslot_id(), 5);

        // Optional fields should be None
        assert!(ie.cca_offset().is_none());
        assert!(ie.cca().is_none());
        assert!(ie.tx_offset().is_none());
    }

    #[test]
    fn test_tsch_timeslot_ie_full() {
        let mut buf = [0u8; 25];
        let mut ie = TschTimeslotIe::new_unchecked(&mut buf);

        ie.set_timeslot_id(1);
        ie.set_cca_offset(1800);
        ie.set_cca(128);
        ie.set_tx_offset(2120);
        ie.set_rx_offset(1020);
        ie.set_rx_ack_delay(800);
        ie.set_tx_ack_delay(1000);
        ie.set_rx_wait(2200);
        ie.set_ack_wait(400);
        ie.set_rx_tx(192);
        ie.set_max_ack(2400);
        ie.set_max_tx(4256);
        ie.set_timeslot_length(10000);

        assert!(!ie.is_reduced());
        assert_eq!(ie.timeslot_id(), 1);
        assert_eq!(ie.cca_offset(), Some(1800));
        assert_eq!(ie.cca(), Some(128));
        assert_eq!(ie.tx_offset(), Some(2120));
        assert_eq!(ie.rx_offset(), Some(1020));
        assert_eq!(ie.rx_ack_delay(), Some(800));
        assert_eq!(ie.tx_ack_delay(), Some(1000));
        assert_eq!(ie.rx_wait(), Some(2200));
        assert_eq!(ie.ack_wait(), Some(400));
        assert_eq!(ie.rx_tx(), Some(192));
        assert_eq!(ie.max_ack(), Some(2400));
        assert_eq!(ie.max_tx(), Some(4256));
        assert_eq!(ie.timeslot_length(), Some(10000));
    }

    #[test]
    fn test_tsch_timeslot_ie_new_empty() {
        let buf: [u8; 0] = [];
        assert!(TschTimeslotIe::new(&buf).is_none());
    }

    #[test]
    fn test_link_descriptor_new_valid() {
        let buf = [0u8; 5];
        let link = LinkDescriptor::new(&buf).unwrap();
        assert_eq!(link.timeslot(), 0);
        assert_eq!(link.channel_offset(), 0);
        assert_eq!(link.options(), 0);
    }

    #[test]
    fn test_link_descriptor_new_too_short() {
        let buf = [0u8; 4];
        assert!(LinkDescriptor::new(&buf).is_none());
    }

    #[test]
    fn test_link_descriptor_fields() {
        let mut buf = [0u8; 5];
        let mut link = LinkDescriptor::new_unchecked(&mut buf);

        link.set_timeslot(100);
        link.set_channel_offset(5);
        link.set_options(link_options::TX | link_options::SHARED);

        assert_eq!(link.timeslot(), 100);
        assert_eq!(link.channel_offset(), 5);
        assert!(link.is_tx());
        assert!(!link.is_rx());
        assert!(link.is_shared());
        assert!(!link.is_timekeeping());
        assert!(!link.is_priority());
    }

    #[test]
    fn test_link_descriptor_all_options() {
        let mut buf = [0u8; 5];
        let mut link = LinkDescriptor::new_unchecked(&mut buf);

        link.set_options(
            link_options::TX
                | link_options::RX
                | link_options::SHARED
                | link_options::TIMEKEEPING
                | link_options::PRIORITY,
        );

        assert!(link.is_tx());
        assert!(link.is_rx());
        assert!(link.is_shared());
        assert!(link.is_timekeeping());
        assert!(link.is_priority());
    }

    #[test]
    fn test_slotframe_descriptor_new_valid() {
        let buf = [0u8; 4];
        let sf = SlotframeDescriptor::new(&buf).unwrap();
        assert_eq!(sf.handle(), 0);
        assert_eq!(sf.size(), 0);
        assert_eq!(sf.num_links(), 0);
    }

    #[test]
    fn test_slotframe_descriptor_new_too_short() {
        let buf = [0u8; 3];
        assert!(SlotframeDescriptor::new(&buf).is_none());
    }

    #[test]
    fn test_slotframe_descriptor_fields() {
        let mut buf = [0u8; 4];
        let mut sf = SlotframeDescriptor::new_unchecked(&mut buf);

        sf.set_handle(1);
        sf.set_size(101);
        sf.set_num_links(3);

        assert_eq!(sf.handle(), 1);
        assert_eq!(sf.size(), 101);
        assert_eq!(sf.num_links(), 3);
        assert_eq!(sf.total_length(), 4 + 3 * 5); // header + 3 links
    }

    #[test]
    fn test_slotframe_descriptor_with_links() {
        // Slotframe header (4) + 2 links (10)
        let mut buf = [0u8; 14];

        {
            let mut sf = SlotframeDescriptor::new_unchecked(&mut buf[..]);
            sf.set_handle(0);
            sf.set_size(101);
            sf.set_num_links(2);

            // Link 1
            let mut links = sf.links_mut();
            let mut link1 = links.next().unwrap();
            link1.set_timeslot(0);
            link1.set_channel_offset(0);
            link1.set_options(link_options::TX | link_options::RX);

            // Link 2
            let mut link2 = links.next().unwrap();
            link2.set_timeslot(50);
            link2.set_channel_offset(1);
            link2.set_options(link_options::TX);
        }

        // Read back
        let sf = SlotframeDescriptor::new(&buf).unwrap();
        assert_eq!(sf.num_links(), 2);

        let mut links = sf.links();

        let link1 = links.next().unwrap();
        assert_eq!(link1.timeslot(), 0);
        assert_eq!(link1.channel_offset(), 0);
        assert!(link1.is_tx());
        assert!(link1.is_rx());

        let link2 = links.next().unwrap();
        assert_eq!(link2.timeslot(), 50);
        assert_eq!(link2.channel_offset(), 1);
        assert!(link2.is_tx());
        assert!(!link2.is_rx());

        assert!(links.next().is_none());
    }

    // #[test]
    // fn test_tsch_slotframe_link_ie_single_slotframe() {
    //     // 1 byte count + slotframe header (4) + 1 link (5)
    //     let mut buf = [0u8; 10];

    //     {
    //         let mut ie = TschSlotframeLinkIe::new_unchecked(&mut buf[..]);
    //         ie.set_num_slotframes(1);

    //         let mut slotframes = ie.slotframes_mut();
    //         let mut sf = slotframes.next().unwrap();
    //         sf.set_handle(0);
    //         sf.set_size(101);
    //         sf.set_num_links(1);

    //         let mut links = sf.links_mut();
    //         let mut link = links.next().unwrap();
    //         link.set_timeslot(10);
    //         link.set_channel_offset(2);
    //         link.set_options(link_options::TX | link_options::SHARED);
    //     }

    //     let ie = TschSlotframeLinkIe::new(&buf).unwrap();
    //     assert_eq!(ie.num_slotframes(), 1);

    //     let mut slotframes = ie.slotframes();
    //     let sf = slotframes.next().unwrap();
    //     assert_eq!(sf.handle(), 0);
    //     assert_eq!(sf.size(), 101);
    //     assert_eq!(sf.num_links(), 1);

    //     let mut links = sf.links();
    //     let link = links.next().unwrap();
    //     assert_eq!(link.timeslot(), 10);
    //     assert_eq!(link.channel_offset(), 2);
    //     assert!(link.is_tx());
    //     assert!(link.is_shared());
    // }
}

