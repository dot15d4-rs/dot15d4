// ============================================================================
// IE Type Constants
// ============================================================================

use super::{header_ie_id, HeaderIeHeader};

pub mod payload_ie_id {
    //! Payload IE Group IDs (IEEE 802.15.4-2020, Table 7-15)
    pub const ESDU: u8 = 0x00;
    pub const MLME: u8 = 0x01;
    pub const VENDOR_SPECIFIC: u8 = 0x02;
    pub const TERMINATION: u8 = 0x0f;
}

/// Payload IE descriptor (2 bytes).
///
/// Format (little-endian):
/// - Bits 0-10: Length (0-2047)
/// - Bits 11-14: Group ID
/// - Bit 15: Type (1 = Payload IE)
#[derive(Clone, Copy)]
pub struct PayloadIeHeader<Bytes> {
    bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> PayloadIeHeader<Bytes> {
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

    /// Content length (0-2047).
    #[inline]
    pub fn length(&self) -> u16 {
        self.raw() & 0x07FF
    }

    /// Group ID.
    #[inline]
    pub fn group_id(&self) -> u8 {
        ((self.raw() >> 11) & 0x0F) as u8
    }

    /// Type bit (1 for Payload IE).
    #[inline]
    pub fn ie_type(&self) -> u8 {
        ((self.raw() >> 15) & 0x01) as u8
    }

    #[inline]
    pub fn total_length(&self) -> usize {
        Self::LENGTH + self.length() as usize
    }

    #[inline]
    pub fn is_termination(&self) -> bool {
        self.group_id() == payload_ie_id::TERMINATION
    }

    #[inline]
    pub fn is_mlme(&self) -> bool {
        self.group_id() == payload_ie_id::MLME
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> PayloadIeHeader<Bytes> {
    #[inline]
    pub fn set_length(&mut self, length: u16) {
        debug_assert!(length <= 2047);
        let b = self.bytes.as_mut();
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !0x07FF) | (length & 0x07FF);
        b[..2].copy_from_slice(&raw.to_le_bytes());
    }

    #[inline]
    pub fn set_group_id(&mut self, id: u8) {
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
    pub fn init(&mut self, group_id: u8, length: u16) {
        self.set_ie_type(1);
        self.set_group_id(group_id);
        self.set_length(length);
    }
}

/// Find MLME content range within IE buffer.
pub fn find_mlme_content_range(buf: &[u8]) -> Option<(usize, usize)> {
    let mut offset = 0;

    // Skip header IEs to find HT1
    while offset + HeaderIeHeader::<&[u8]>::LENGTH <= buf.len() {
        let header = HeaderIeHeader::new(&buf[offset..])?;
        let content_len = header.length() as usize;
        offset += HeaderIeHeader::<&[u8]>::LENGTH + content_len;

        if header.element_id() == header_ie_id::HT1 {
            break;
        }
        if header.element_id() == header_ie_id::HT2 {
            return None;
        }
    }

    // Now find MLME Payload IE
    while offset + PayloadIeHeader::<&[u8]>::LENGTH <= buf.len() {
        let header = PayloadIeHeader::new(&buf[offset..])?;
        let content_len = header.length() as usize;
        let content_start = offset + PayloadIeHeader::<&[u8]>::LENGTH;
        let content_end = content_start + content_len;

        if content_end > buf.len() {
            return None;
        }

        if header.is_mlme() {
            return Some((content_start, content_end));
        }

        if header.is_termination() {
            break;
        }

        offset = content_end;
    }

    None
}
#[cfg(test)]
mod tests {
    use crate::radio::frame::header::{header_ie_id, HeaderIeHeader};
    use crate::radio::frame::payload::{payload_ie_id, PayloadIeHeader};

    #[test]
    fn test_payload_ie_header_new_valid() {
        let mut buf = [0u8; 2];
        {
            let mut header = PayloadIeHeader::new_unchecked(&mut buf);
            header.init(payload_ie_id::MLME, 10);
        }

        let header = PayloadIeHeader::new(&buf).unwrap();
        assert_eq!(header.ie_type(), 1);
        assert_eq!(header.group_id(), payload_ie_id::MLME);
        assert_eq!(header.length(), 10);
        assert_eq!(header.total_length(), 12);
    }

    #[test]
    fn test_payload_ie_header_new_too_short() {
        let buf = [0u8; 1];
        assert!(PayloadIeHeader::new(&buf).is_none());
    }

    #[test]
    fn test_payload_ie_header_new_invalid_type() {
        // Create Header IE (type=0) and try to parse as Payload IE
        let mut buf = [0u8; 2];
        {
            let mut header = HeaderIeHeader::new_unchecked(&mut buf);
            header.init(header_ie_id::TIME_CORRECTION, 2);
        }
        assert!(PayloadIeHeader::new(&buf).is_none());
    }

    #[test]
    fn test_payload_ie_header_length_max() {
        let mut buf = [0u8; 2];
        let mut header = PayloadIeHeader::new_unchecked(&mut buf);

        // Maximum length is 2047 (11 bits)
        header.set_length(2047);
        assert_eq!(header.length(), 2047);

        header.set_length(0);
        assert_eq!(header.length(), 0);
    }

    #[test]
    fn test_payload_ie_header_group_ids() {
        let mut buf = [0u8; 2];
        let mut header = PayloadIeHeader::new_unchecked(&mut buf);

        header.init(payload_ie_id::ESDU, 0);
        assert_eq!(header.group_id(), payload_ie_id::ESDU);
        assert!(!header.is_mlme());
        assert!(!header.is_termination());

        header.init(payload_ie_id::MLME, 0);
        assert_eq!(header.group_id(), payload_ie_id::MLME);
        assert!(header.is_mlme());
        assert!(!header.is_termination());

        header.init(payload_ie_id::VENDOR_SPECIFIC, 0);
        assert_eq!(header.group_id(), payload_ie_id::VENDOR_SPECIFIC);
        assert!(!header.is_mlme());
        assert!(!header.is_termination());

        header.init(payload_ie_id::TERMINATION, 0);
        assert_eq!(header.group_id(), payload_ie_id::TERMINATION);
        assert!(!header.is_mlme());
        assert!(header.is_termination());
    }
}
