//! IEEE 802.15.4 Frame Control field access.
use dot15d4_util::{Error, Result};

use super::AddressingMode;

/// IEEE 802.15.4 frame type.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum FrameType {
    /// Beacon frame.
    Beacon = 0b000,
    /// Data frame.
    Data = 0b001,
    /// Acknowledgement frame.
    Ack = 0b010,
    /// MAC command frame.
    MacCommand = 0b011,
    /// Multipurpose frame.
    Multipurpose = 0b101,
    /// Fragmentation frame.
    FragmentOrFrak = 0b110,
    /// Extended frame.
    Extended = 0b111,
    /// Unknown frame type.
    Unknown,
}

impl From<u8> for FrameType {
    fn from(value: u8) -> Self {
        match value {
            0b000 => Self::Beacon,
            0b001 => Self::Data,
            0b010 => Self::Ack,
            0b011 => Self::MacCommand,
            0b101 => Self::Multipurpose,
            0b110 => Self::FragmentOrFrak,
            0b111 => Self::Extended,
            _ => Self::Unknown,
        }
    }
}

/// IEEE 802.15.4 frame version.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum FrameVersion {
    /// IEEE 802.15.4-2003 frame version.
    Ieee802154_2003 = 0b00,
    /// IEEE 802.15.4-2006 frame version.
    Ieee802154_2006 = 0b01,
    /// IEEE 802.15.4-2015 and beyond.
    Ieee802154 = 0b10,
    /// Unknown frame version.
    Unknown,
}

impl From<u8> for FrameVersion {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Self::Ieee802154_2003,
            0b01 => Self::Ieee802154_2006,
            0b10 => Self::Ieee802154,
            _ => Self::Unknown,
        }
    }
}

/// A reader/writer for the IEEE 802.15.4 Frame Control field.
#[derive(Debug, PartialEq, Eq)]
pub struct FrameControl<Bytes> {
    bytes: Bytes,
}

impl<Bytes: Clone> Clone for FrameControl<Bytes> {
    fn clone(&self) -> Self {
        Self {
            bytes: self.bytes.clone(),
        }
    }
}

impl<Bytes: AsRef<[u8]>> FrameControl<Bytes> {
    /// Create a new [`FrameControl`] reader/writer from a given buffer.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is too short.
    pub fn new(bytes: Bytes) -> Result<Self> {
        let fc = Self::new_unchecked(bytes);

        if !fc.check_len() {
            return Err(Error);
        }

        Ok(fc)
    }

    /// Returns `false` if the buffer is too short to contain the Frame Control
    /// field.
    fn check_len(&self) -> bool {
        self.bytes.as_ref().len() >= 2
    }

    /// Create a new [`FrameControl`] reader/writer from a given buffer without
    /// length checking.
    pub const fn new_unchecked(bytes: Bytes) -> Self {
        Self { bytes }
    }

    /// Return the inner buffer.
    pub fn into_inner(self) -> Bytes {
        self.bytes
    }

    /// Return the [`FrameType`] field.
    pub fn frame_type(&self) -> FrameType {
        let b = &self.bytes.as_ref()[..2];
        FrameType::from((u16::from_le_bytes([b[0], b[1]]) & 0b111) as u8)
    }

    /// Returns `true` when the security enabled field is set.
    pub fn security_enabled(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 3) & 0b1) == 1
    }

    /// Returns `true` when the frame pending field is set.
    pub fn frame_pending(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 4) & 0b1) == 1
    }

    /// Returns `true` when the acknowledgement request field is set.
    pub fn ack_request(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 5) & 0b1) == 1
    }

    /// Returns `true` when the PAN ID compression field is set.
    pub fn pan_id_compression(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 6) & 0b1) == 1
    }

    /// Returns the sequence number suppression field.
    pub fn sequence_number_suppression(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 8) & 0b1) == 1
    }

    /// Returns `true` when the information element field is set.
    pub fn information_elements_present(&self) -> bool {
        let b = &self.bytes.as_ref()[..2];
        ((u16::from_le_bytes([b[0], b[1]]) >> 9) & 0b1) == 1
    }

    /// Return the Destination [`AddressingMode`].
    pub fn dst_addressing_mode(&self) -> AddressingMode {
        let b = &self.bytes.as_ref()[..2];
        let raw = (u16::from_le_bytes([b[0], b[1]]) >> 10) & 0b11;
        AddressingMode::from(raw as u8)
    }

    /// Return the Source [`AddressingMode`].
    pub fn src_addressing_mode(&self) -> AddressingMode {
        let b = &self.bytes.as_ref()[..2];
        let raw = (u16::from_le_bytes([b[0], b[1]]) >> 14) & 0b11;
        AddressingMode::from(raw as u8)
    }

    /// Return the [`FrameVersion`].
    pub fn frame_version(&self) -> FrameVersion {
        let b = &self.bytes.as_ref()[..2];
        let raw = (u16::from_le_bytes([b[0], b[1]]) >> 12) & 0b11;
        FrameVersion::from(raw as u8)
    }

    pub fn is_valid(&self) -> bool {
        if matches!(self.frame_type(), FrameType::Unknown) {
            return false;
        }

        if matches!(self.frame_version(), FrameVersion::Unknown) {
            return false;
        }

        true
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> FrameControl<Bytes> {
    /// Set the frame type field.
    pub fn set_frame_type(&mut self, frame_type: FrameType) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !0b111) | ((frame_type as u8) as u16 & 0b111);
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the security enabled field.
    pub fn set_security_enabled(&mut self, security_enabled: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (security_enabled as u16) << 3;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the frame pending field.
    pub fn set_frame_pending(&mut self, frame_pending: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (frame_pending as u16) << 4;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the acknowledgement request field.
    pub fn set_ack_request(&mut self, ack_request: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (ack_request as u16) << 5;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the PAN ID compression field.
    pub fn set_pan_id_compression(&mut self, pan_id_compression: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (pan_id_compression as u16) << 6;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the sequence number suppression field.
    pub fn set_sequence_number_suppression(&mut self, sequence_number_suppression: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (sequence_number_suppression as u16) << 8;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the information element present field.
    pub fn set_information_elements_present(&mut self, information_elements_present: bool) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw |= (information_elements_present as u16) << 9;
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the destination addressing mode field.
    pub fn set_dst_addressing_mode(&mut self, addressing_mode: AddressingMode) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !(0b11 << 10)) | (((addressing_mode as u8) as u16 & 0b11) << 10);
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the source addressing mode field.
    pub fn set_src_addressing_mode(&mut self, addressing_mode: AddressingMode) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !(0b11 << 14)) | (((addressing_mode as u8) as u16 & 0b11) << 14);
        b.copy_from_slice(&raw.to_le_bytes());
    }

    /// Set the frame version field.
    pub fn set_frame_version(&mut self, frame_version: FrameVersion) {
        let b = &mut self.bytes.as_mut()[..2];
        let mut raw = u16::from_le_bytes([b[0], b[1]]);
        raw = (raw & !(0b11 << 12)) | (((frame_version as u8) as u16 & 0b11) << 12);
        b.copy_from_slice(&raw.to_le_bytes());
    }
}

impl<Bytes: AsRef<[u8]>> core::fmt::Display for FrameControl<Bytes> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "Frame Control")?;
        writeln!(f, "  type: {:?}", self.frame_type())?;
        writeln!(
            f,
            "  security enabled: {}",
            self.security_enabled() as usize
        )?;
        writeln!(f, "  frame pending: {}", self.frame_pending() as usize)?;
        writeln!(f, "  ack request: {}", self.ack_request() as usize)?;
        writeln!(
            f,
            "  pan id compression: {}",
            self.pan_id_compression() as usize
        )?;
        writeln!(
            f,
            "  sequence number suppression: {}",
            self.sequence_number_suppression() as usize
        )?;
        writeln!(
            f,
            "  information elements present: {}",
            self.information_elements_present() as usize
        )?;
        writeln!(f, "  dst addressing mode: {:?}", self.dst_addressing_mode())?;
        writeln!(f, "  src addressing mode: {:?}", self.src_addressing_mode())?;
        writeln!(f, "  frame version: {:?}", self.frame_version())?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bad_length() {
        let fc = [0x0];
        assert!(FrameControl::new(&fc).is_err());
    }

    #[test]
    fn get_fields() {
        let fc = [0x0, 0x0];
        let fc = FrameControl::new(&fc).unwrap();
        assert_eq!(fc.frame_type(), FrameType::Beacon);
        assert!(!fc.security_enabled());
        assert!(!fc.frame_pending());
        assert!(!fc.ack_request());
        assert!(!fc.pan_id_compression());
        assert!(!fc.sequence_number_suppression());
        assert!(!fc.information_elements_present());
        assert_eq!(fc.dst_addressing_mode(), AddressingMode::Absent);
        assert_eq!(fc.src_addressing_mode(), AddressingMode::Absent);
        assert_eq!(fc.frame_version(), FrameVersion::Ieee802154_2003);

        let fc = [0b0010_1001, 0b1010_1010];
        let fc = FrameControl::new(&fc).unwrap();
        assert_eq!(fc.frame_type(), FrameType::Data);
        assert!(fc.security_enabled());
        assert!(!fc.frame_pending());
        assert!(fc.ack_request());
        assert!(!fc.pan_id_compression());
        assert!(!fc.sequence_number_suppression());
        assert!(fc.information_elements_present());
        assert_eq!(fc.dst_addressing_mode(), AddressingMode::Short);
        assert_eq!(fc.src_addressing_mode(), AddressingMode::Short);
        assert_eq!(fc.frame_version(), FrameVersion::Ieee802154);
    }

    #[test]
    fn set_fields() {
        let mut fc = [0x0, 0x0];
        let mut fc = FrameControl::new_unchecked(&mut fc);
        fc.set_frame_type(FrameType::Beacon);
        fc.set_security_enabled(false);
        fc.set_frame_pending(false);
        fc.set_ack_request(false);
        fc.set_pan_id_compression(false);
        fc.set_sequence_number_suppression(false);
        fc.set_information_elements_present(false);
        fc.set_dst_addressing_mode(AddressingMode::Absent);
        fc.set_src_addressing_mode(AddressingMode::Absent);
        fc.set_frame_version(FrameVersion::Ieee802154_2003);
        assert_eq!(*fc.into_inner(), [0x0, 0x0]);

        let mut fc = [0x0, 0x0];
        let mut fc = FrameControl::new_unchecked(&mut fc);
        fc.set_frame_type(FrameType::Data);
        fc.set_security_enabled(true);
        fc.set_frame_pending(false);
        fc.set_ack_request(true);
        fc.set_pan_id_compression(false);
        fc.set_sequence_number_suppression(false);
        fc.set_information_elements_present(true);
        fc.set_dst_addressing_mode(AddressingMode::Short);
        fc.set_src_addressing_mode(AddressingMode::Short);
        fc.set_frame_version(FrameVersion::Ieee802154);
        assert_eq!(*fc.into_inner(), [0b0010_1001, 0b1010_1010]);
    }

    #[test]
    fn frame_type() {
        assert_eq!(FrameType::from(0b000), FrameType::Beacon);
        assert_eq!(FrameType::from(0b001), FrameType::Data);
        assert_eq!(FrameType::from(0b010), FrameType::Ack);
        assert_eq!(FrameType::from(0b011), FrameType::MacCommand);
        assert_eq!(FrameType::from(0b101), FrameType::Multipurpose);
        assert_eq!(FrameType::from(0b110), FrameType::FragmentOrFrak);
        assert_eq!(FrameType::from(0b111), FrameType::Extended);
        assert_eq!(FrameType::from(0b100), FrameType::Unknown);
    }

    #[test]
    fn frame_version() {
        assert_eq!(FrameVersion::from(0b00), FrameVersion::Ieee802154_2003);
        assert_eq!(FrameVersion::from(0b01), FrameVersion::Ieee802154_2006);
        assert_eq!(FrameVersion::from(0b10), FrameVersion::Ieee802154);
        assert_eq!(FrameVersion::from(0b11), FrameVersion::Unknown);
    }

    #[test]
    #[cfg(feature = "std")]
    fn formatting() {
        let fc = [0b0010_1001, 0b1010_1010];
        let fc = FrameControl::new(&fc).unwrap();
        assert_eq!(
            format!("{fc}"),
            r"Frame Control
  type: Data
  security enabled: 1
  frame pending: 0
  ack request: 1
  pan id compression: 0
  sequence number suppression: 0
  information elements present: 1
  dst addressing mode: Short
  src addressing mode: Short
  frame version: Ieee802154
"
        );
    }
}
