// ============================================================================
// IE Type Constants
// ============================================================================

pub mod header_ie_id {
    //! Header IE Element IDs (IEEE 802.15.4-2020, Table 7-7)
    pub const TIME_CORRECTION: u8 = 0x1e;
    pub const HT1: u8 = 0x7e;
    pub const HT2: u8 = 0x7f;

    #[inline]
    pub const fn is_termination(id: u8) -> bool {
        id == HT1 || id == HT2
    }
}

// ============================================================================
// IE Header Wrapper Types
// ============================================================================

/// Header IE descriptor (2 bytes).
///
/// Format (little-endian):
/// - Bits 0-6: Length (0-127)
/// - Bits 7-14: Element ID
/// - Bit 15: Type (0 = Header IE)
#[derive(Clone, Copy)]
pub struct HeaderIeHeader<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> HeaderIeHeader<Bytes> {
    pub const LENGTH: usize = 2;

    /// Create with validation.
    #[inline]
    pub fn new(bytes: Bytes) -> Option<Self> {
        if bytes.as_ref().len() < Self::LENGTH {
            return None;
        }
        let header = Self { bytes };
        // Type bit must be 0 for Header IE
        if header.ie_type() != 0 {
            return None;
        }
        Some(header)
    }

    /// Create without validation.
    #[inline]
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    /// Content length (0-127).
    #[inline]
    pub fn length(&self) -> u8 {
        self.bytes.as_ref()[0] & 0x7F
    }

    /// Element ID.
    #[inline]
    pub fn element_id(&self) -> u8 {
        let b = self.bytes.as_ref();
        ((b[0] >> 7) & 0x01) | ((b[1] & 0x7F) << 1)
    }

    /// Type bit (0 for Header IE).
    #[inline]
    pub fn ie_type(&self) -> u8 {
        (self.bytes.as_ref()[1] >> 7) & 0x01
    }

    /// Total size including header.
    #[inline]
    pub fn total_length(&self) -> usize {
        Self::LENGTH + self.length() as usize
    }

    /// Check if this is a termination IE.
    #[inline]
    pub fn is_termination(&self) -> bool {
        header_ie_id::is_termination(self.element_id())
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> HeaderIeHeader<Bytes> {
    /// Set length field.
    #[inline]
    pub fn set_length(&mut self, length: u8) {
        debug_assert!(length <= 127);
        let b = self.bytes.as_mut();
        b[0] = (b[0] & 0x80) | (length & 0x7F);
    }

    /// Set element ID.
    #[inline]
    pub fn set_element_id(&mut self, id: u8) {
        let b = self.bytes.as_mut();
        b[0] = (b[0] & 0x7F) | ((id & 0x01) << 7);
        b[1] = (b[1] & 0x80) | ((id >> 1) & 0x7F);
    }

    /// Set type bit (0 for Header IE).
    #[inline]
    pub fn set_ie_type(&mut self, ie_type: u8) {
        let b = self.bytes.as_mut();
        b[1] = (b[1] & 0x7F) | ((ie_type & 0x01) << 7);
    }

    /// Initialize as Header IE with given ID and length.
    #[inline]
    pub fn init(&mut self, id: u8, length: u8) {
        self.set_ie_type(0);
        self.set_element_id(id);
        self.set_length(length);
    }
}

// ============================================================================
// TSCH IE Content Wrapper Types
// ============================================================================

/// Time Correction Header IE content (2 bytes).
///
/// Format:
/// - Bits 0-11: Time Sync Info (signed, ~30.5 µs units)
/// - Bit 15: NACK
#[derive(Clone, Copy, Debug)]
pub struct TimeCorrectionIe<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> TimeCorrectionIe<Bytes> {
    pub const LENGTH: usize = 2;

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
    fn raw(&self) -> u16 {
        let b = self.bytes.as_ref();
        u16::from_le_bytes([b[0], b[1]])
    }

    /// Time sync value (signed 12-bit, ~30.5 µs units).
    #[inline]
    pub fn time_sync(&self) -> i16 {
        let raw = self.raw() & 0x0FFF;
        if raw & 0x0800 != 0 {
            (raw | 0xF000) as i16
        } else {
            raw as i16
        }
    }

    /// NACK bit.
    #[inline]
    pub fn nack(&self) -> bool {
        self.raw() & 0x8000 != 0
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> TimeCorrectionIe<Bytes> {
    #[inline]
    pub fn set_time_sync(&mut self, value: i16) {
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !0x0FFF) | ((value as u16) & 0x0FFF);
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }

    #[inline]
    pub fn set_nack(&mut self, nack: bool) {
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !0x8000) | if nack { 0x8000 } else { 0 };
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }
}

// ============================================================================
// IE Iterators
// ============================================================================

/// Parsed Header IE with typed access to content.
#[derive(Clone, Copy)]
pub struct HeaderIe<'a> {
    header: HeaderIeHeader<&'a [u8]>,
    content: &'a [u8],
}

impl<'a> HeaderIe<'a> {
    #[inline]
    pub fn element_id(&self) -> u8 {
        self.header.element_id()
    }

    #[inline]
    pub fn content(&self) -> &'a [u8] {
        self.content
    }

    #[inline]
    pub fn is_termination(&self) -> bool {
        self.header.is_termination()
    }

    /// Try to interpret content as Time Correction IE.
    #[inline]
    pub fn as_time_correction(&self) -> Option<TimeCorrectionIe<&'a [u8]>> {
        if self.element_id() == header_ie_id::TIME_CORRECTION {
            TimeCorrectionIe::new(self.content)
        } else {
            None
        }
    }
}

/// Iterator over Header IEs.
pub struct HeaderIeIter<'a> {
    buf: &'a [u8],
    offset: usize,
}

impl<'a> HeaderIeIter<'a> {
    #[inline]
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, offset: 0 }
    }

    /// Remaining bytes after current position.
    #[inline]
    pub fn remaining(&self) -> &'a [u8] {
        &self.buf[self.offset..]
    }

    /// Current offset.
    #[inline]
    pub fn offset(&self) -> usize {
        self.offset
    }
}

impl<'a> Iterator for HeaderIeIter<'a> {
    type Item = HeaderIe<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let header = HeaderIeHeader::new(&self.buf[self.offset..])?;
        let content_len = header.length() as usize;
        let header_end = self.offset + HeaderIeHeader::<&[u8]>::LENGTH;
        let content_end = header_end + content_len;

        if content_end > self.buf.len() {
            return None;
        }

        let header_slice = &self.buf[self.offset..header_end];
        let content = &self.buf[header_end..content_end];
        self.offset = content_end;

        Some(HeaderIe {
            header: HeaderIeHeader::new_unchecked(header_slice),
            content,
        })
    }
}

// ============================================================================
// Helper functions for finding IE content (read-only)
// ============================================================================

/// Find a Header IE content by element ID (returns slice).
pub fn find_header_ie_content(buf: &[u8], element_id: u8) -> Option<&[u8]> {
    let (start, end) = find_header_ie_content_range(buf, element_id)?;
    Some(&buf[start..end])
}

/// Find a Header IE content range by element ID.
pub fn find_header_ie_content_range(buf: &[u8], element_id: u8) -> Option<(usize, usize)> {
    let mut offset = 0;

    while offset + HeaderIeHeader::<&[u8]>::LENGTH <= buf.len() {
        let header = HeaderIeHeader::new(&buf[offset..])?;
        let content_len = header.length() as usize;
        let content_start = offset + HeaderIeHeader::<&[u8]>::LENGTH;
        let content_end = content_start + content_len;

        if content_end > buf.len() {
            return None;
        }

        if header.element_id() == element_id {
            return Some((content_start, content_end));
        }

        if header.is_termination() {
            break;
        }

        offset = content_end;
    }

    None
}

/// Find a Header IE content mutably by element ID.
pub fn find_header_ie_content_mut(buf: &mut [u8], element_id: u8) -> Option<&mut [u8]> {
    let mut offset = 0;

    // First pass: find the offset
    let (content_start, content_end) = loop {
        if offset + HeaderIeHeader::<&[u8]>::LENGTH > buf.len() {
            return None;
        }

        let header = HeaderIeHeader::new(&buf[offset..])?;
        let content_len = header.length() as usize;
        let c_start = offset + HeaderIeHeader::<&[u8]>::LENGTH;
        let c_end = c_start + content_len;

        if c_end > buf.len() {
            return None;
        }

        if header.element_id() == element_id {
            break (c_start, c_end);
        }

        if header.is_termination() {
            return None;
        }

        offset = c_end;
    };

    Some(&mut buf[content_start..content_end])
}
