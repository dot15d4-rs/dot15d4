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
#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_header_ie_header() {
        // Time Correction: id=0x1e, len=2
        let mut buf = [0u8; 4];
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf[..2]);
            header.init(header_ie_id::TIME_CORRECTION, 2);
        }

        let header = HeaderIeHeader::new(&buf[..2]).unwrap();
        assert_eq!(header.element_id(), header_ie_id::TIME_CORRECTION);
        assert_eq!(header.length(), 2);
        assert_eq!(header.ie_type(), 0);
    }

    #[test]
    fn test_time_correction_ie() {
        let mut buf = [0u8; 2];
        {
            let mut ie = TimeCorrectionIe::new_unchecked(&mut buf);
            ie.set_time_sync(-100);
            ie.set_nack(true);
        }

        let ie = TimeCorrectionIe::new(&buf).unwrap();
        assert_eq!(ie.time_sync(), -100);
        assert!(ie.nack());
    }

    #[test]
    fn test_header_ie_header_new_valid() {
        // Build a valid Header IE header: type=0, id=0x1e (TIME_CORRECTION), len=2
        let mut buf = [0u8; 2];
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf);
            header.init(header_ie_id::TIME_CORRECTION, 2);
        }

        let header = HeaderIeHeader::new(&buf).unwrap();
        assert_eq!(header.ie_type(), 0);
        assert_eq!(header.element_id(), header_ie_id::TIME_CORRECTION);
        assert_eq!(header.length(), 2);
        assert_eq!(header.total_length(), 4); // 2 header + 2 content
    }

    #[test]
    fn test_header_ie_header_new_too_short() {
        let buf = [0u8; 1]; // Only 1 byte, need 2
        assert!(HeaderIeHeader::new(&buf).is_none());
    }

    #[test]
    fn test_header_ie_header_new_invalid_type() {
        // Create a Payload IE header (type=1) and try to parse as Header IE
        let mut buf = [0u8; 2];
        buf[1] = 0x80; // Set type bit to 1
        assert!(HeaderIeHeader::new(&buf).is_none());
    }

    #[test]
    fn test_header_ie_header_element_id_encoding() {
        // Test element ID encoding across the byte boundary
        // Element ID spans bits 7-14: bit 7 of byte 0 and bits 0-6 of byte 1
        let mut buf = [0u8; 2];
        let mut header = HeaderIeHeader::new_unchecked(&mut buf);

        // Test ID 0x00
        header.set_element_id(0x00);
        assert_eq!(header.element_id(), 0x00);

        // Test ID 0x01 (sets bit 7 of byte 0)
        header.set_element_id(0x01);
        assert_eq!(header.element_id(), 0x01);

        // Test ID 0x1e (TIME_CORRECTION)
        header.set_element_id(0x1e);
        assert_eq!(header.element_id(), 0x1e);

        // Test ID 0x7e (HT1)
        header.set_element_id(0x7e);
        assert_eq!(header.element_id(), 0x7e);

        // Test ID 0x7f (HT2)
        header.set_element_id(0x7f);
        assert_eq!(header.element_id(), 0x7f);
    }

    #[test]
    fn test_header_ie_header_length_max() {
        let mut buf = [0u8; 2];
        let mut header = HeaderIeHeader::new_unchecked(&mut buf);

        // Maximum length is 127 (7 bits)
        header.set_length(127);
        assert_eq!(header.length(), 127);
        assert_eq!(header.total_length(), 129);

        // Length 0
        header.set_length(0);
        assert_eq!(header.length(), 0);
        assert_eq!(header.total_length(), 2);
    }

    #[test]
    fn test_header_ie_header_termination_ids() {
        assert!(header_ie_id::is_termination(header_ie_id::HT1));
        assert!(header_ie_id::is_termination(header_ie_id::HT2));
        assert!(!header_ie_id::is_termination(header_ie_id::TIME_CORRECTION));
    }

    #[test]
    fn test_header_ie_header_is_termination() {
        let mut buf = [0u8; 2];
        let mut header = HeaderIeHeader::new_unchecked(&mut buf);

        header.init(header_ie_id::HT1, 0);
        assert!(header.is_termination());

        header.init(header_ie_id::HT2, 0);
        assert!(header.is_termination());

        header.init(header_ie_id::TIME_CORRECTION, 2);
        assert!(!header.is_termination());
    }
    #[test]

    fn test_header_ie_iter_multiple_ies() {
        // Build: Time Correction + HT1
        let mut buf = [0u8; 6];

        // Time Correction (id=0x1e, len=2)
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf[..2]);
            header.init(header_ie_id::TIME_CORRECTION, 2);
        }

        // HT1 (id=0x7e, len=0)
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf[4..6]);
            header.init(header_ie_id::HT1, 0);
        }

        let mut iter = HeaderIeIter::new(&buf);

        // First IE: Time Correction
        let ie1 = iter.next().unwrap();
        assert_eq!(ie1.element_id(), header_ie_id::TIME_CORRECTION);
        assert!(!ie1.is_termination());

        // Second IE: HT1
        let ie2 = iter.next().unwrap();
        assert_eq!(ie2.element_id(), header_ie_id::HT1);
        assert!(ie2.is_termination());

        // No more
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_header_ie_iter_remaining() {
        let mut buf = [0u8; 8];
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf[..2]);
            header.init(header_ie_id::TIME_CORRECTION, 2);
        }

        let mut iter = HeaderIeIter::new(&buf);
        assert_eq!(iter.remaining().len(), 8);
        assert_eq!(iter.offset(), 0);

        iter.next();
        assert_eq!(iter.remaining().len(), 4);
        assert_eq!(iter.offset(), 4);
    }

    #[test]
    fn test_header_ie_iter_empty_buffer() {
        let buf: [u8; 0] = [];
        let mut iter = HeaderIeIter::new(&buf);
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_time_correction_ie_new_valid() {
        let buf = [0u8; 2];
        let ie = TimeCorrectionIe::new(&buf).unwrap();
        assert_eq!(ie.time_sync(), 0);
        assert!(!ie.nack());
    }

    #[test]
    fn test_time_correction_ie_new_too_short() {
        let buf = [0u8; 1];
        assert!(TimeCorrectionIe::new(&buf).is_none());
    }

    #[test]
    fn test_time_correction_ie_positive_time_sync() {
        let mut buf = [0u8; 2];
        let mut ie = TimeCorrectionIe::new_unchecked(&mut buf);

        ie.set_time_sync(100);
        assert_eq!(ie.time_sync(), 100);
        assert!(!ie.nack());

        ie.set_time_sync(2047); // Max positive 12-bit value
        assert_eq!(ie.time_sync(), 2047);
    }

    #[test]
    fn test_time_correction_ie_negative_time_sync() {
        let mut buf = [0u8; 2];
        let mut ie = TimeCorrectionIe::new_unchecked(&mut buf);

        ie.set_time_sync(-100);
        assert_eq!(ie.time_sync(), -100);

        ie.set_time_sync(-1);
        assert_eq!(ie.time_sync(), -1);

        ie.set_time_sync(-2048); // Min negative 12-bit value
        assert_eq!(ie.time_sync(), -2048);
    }

    #[test]
    fn test_time_correction_ie_nack() {
        let mut buf = [0u8; 2];
        let mut ie = TimeCorrectionIe::new_unchecked(&mut buf);

        ie.set_nack(true);
        assert!(ie.nack());

        ie.set_nack(false);
        assert!(!ie.nack());
    }

    #[test]
    fn test_time_correction_ie_combined() {
        let mut buf = [0u8; 2];
        let mut ie = TimeCorrectionIe::new_unchecked(&mut buf);

        // Set both fields
        ie.set_time_sync(-500);
        ie.set_nack(true);

        // Verify both are preserved
        assert_eq!(ie.time_sync(), -500);
        assert!(ie.nack());

        // Change time_sync, verify nack preserved
        ie.set_time_sync(200);
        assert_eq!(ie.time_sync(), 200);
        assert!(ie.nack());

        // Change nack, verify time_sync preserved
        ie.set_nack(false);
        assert_eq!(ie.time_sync(), 200);
        assert!(!ie.nack());
    }
}

