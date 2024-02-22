//! Addressing fields readers and writers.

use super::FrameControl;
use super::FrameVersion;

/// An IEEE 802.15.4 address.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Address {
    Absent,
    Short([u8; 2]),
    Extended([u8; 8]),
}

impl Address {
    /// The broadcast address.
    pub const BROADCAST: Address = Address::Short([0xff; 2]);

    /// Query whether the address is an unicast address.
    pub fn is_unicast(&self) -> bool {
        !self.is_broadcast()
    }

    /// Query whether this address is the broadcast address.
    pub fn is_broadcast(&self) -> bool {
        *self == Self::BROADCAST
    }

    pub fn from_bytes(a: &[u8]) -> Self {
        if a.is_empty() {
            Address::Absent
        } else if a.len() == 2 {
            let mut b = [0u8; 2];
            b.copy_from_slice(a);
            Address::Short(b)
        } else if a.len() == 8 {
            let mut b = [0u8; 8];
            b.copy_from_slice(a);
            Address::Extended(b)
        } else {
            unreachable!()
        }
    }

    pub const fn as_bytes(&self) -> &[u8] {
        match self {
            Address::Absent => &[],
            Address::Short(value) => value,
            Address::Extended(value) => value,
        }
    }

    const fn short_from_bytes(a: [u8; 2]) -> Self {
        Self::Short(a)
    }

    const fn extended_from_bytes(a: [u8; 8]) -> Self {
        Self::Extended(a)
    }

    pub fn len(&self) -> usize {
        match self {
            Address::Absent => 0,
            Address::Short(_) => 2,
            Address::Extended(_) => 8,
        }
    }

    pub fn is_empty(&self) -> bool {
        matches!(self, Address::Absent)
    }
}

impl core::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Address::Absent => write!(f, "absent"),
            Address::Short(value) => write!(f, "{:02x}:{:02x}", value[0], value[1]),
            Address::Extended(value) => write!(
                f,
                "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]
            ),
        }
    }
}

/// IEEE 802.15.4 addressing mode.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum AddressingMode {
    Absent = 0b00,
    Short = 0b10,
    Extended = 0b11,
    Unknown,
}

impl AddressingMode {
    /// Return the size of the address in octets.
    pub fn size(&self) -> usize {
        match self {
            Self::Absent => 0,
            Self::Short => 2,
            Self::Extended => 8,
            Self::Unknown => 0,
        }
    }
}

impl From<u8> for AddressingMode {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Self::Absent,
            0b10 => Self::Short,
            0b11 => Self::Extended,
            _ => Self::Unknown,
        }
    }
}

/// A high-level representation of the IEEE 802.15.4 Addressing Fields.
#[derive(Debug)]
pub struct AddressingFieldsRepr {
    pub dst_pan_id: Option<u16>,
    pub src_pan_id: Option<u16>,
    pub dst_address: Option<Address>,
    pub src_address: Option<Address>,
}

impl AddressingFieldsRepr {
    pub fn parse<'f>(addressing: AddressingFields<&'f [u8]>, fc: FrameControl<&'f [u8]>) -> Self {
        Self {
            dst_pan_id: addressing.dst_pan_id(&fc),
            src_pan_id: addressing.src_pan_id(&fc),
            dst_address: addressing.dst_address(&fc),
            src_address: addressing.src_address(&fc),
        }
    }
}

/// A reader/writer for the IEEE 802.15.4 Addressing Fields.
pub struct AddressingFields<T: AsRef<[u8]>> {
    buffer: T,
}

impl<T: AsRef<[u8]>> AddressingFields<T> {
    pub fn new(buffer: T) -> Self {
        Self { buffer }
    }

    /// Return the length of the Addressing Fields in octets.
    pub fn len(&self, fc: &FrameControl<T>) -> usize {
        (match self.dst_pan_id(fc) {
            Some(_) => 2,
            None => 0,
        }) + match fc.dst_addressing_mode() {
            AddressingMode::Absent => 0,
            AddressingMode::Short => 2,
            AddressingMode::Extended => 8,
            _ => unreachable!(),
        } + match self.src_pan_id(fc) {
            Some(_) => 2,
            None => 0,
        } + match fc.src_addressing_mode() {
            AddressingMode::Absent => 0,
            AddressingMode::Short => 2,
            AddressingMode::Extended => 8,
            _ => unreachable!(),
        }
    }

    fn address_present_flags(
        &self,
        fc: &FrameControl<T>,
    ) -> Option<(bool, AddressingMode, bool, AddressingMode)> {
        let dst_addr_mode = fc.dst_addressing_mode();
        let src_addr_mode = fc.src_addressing_mode();
        let pan_id_compression = fc.pan_id_compression();

        use AddressingMode::*;
        match fc.frame_version() {
            FrameVersion::Ieee802154_2003 | FrameVersion::Ieee802154_2006 => {
                match (dst_addr_mode, src_addr_mode) {
                    (Absent, src) => Some((false, Absent, true, src)),
                    (dst, Absent) => Some((true, dst, false, Absent)),

                    (dst, src) if pan_id_compression => Some((true, dst, false, src)),
                    (dst, src) if !pan_id_compression => Some((true, dst, true, src)),
                    _ => None,
                }
            }
            FrameVersion::Ieee802154 => {
                Some(match (dst_addr_mode, src_addr_mode, pan_id_compression) {
                    (Absent, Absent, false) => (false, Absent, false, Absent),
                    (Absent, Absent, true) => (true, Absent, false, Absent),
                    (dst, Absent, false) if !matches!(dst, Absent) => (true, dst, false, Absent),
                    (dst, Absent, true) if !matches!(dst, Absent) => (false, dst, false, Absent),
                    (Absent, src, false) if !matches!(src, Absent) => (false, Absent, true, src),
                    (Absent, src, true) if !matches!(src, Absent) => (false, Absent, true, src),
                    (Extended, Extended, false) => (true, Extended, false, Extended),
                    (Extended, Extended, true) => (false, Extended, false, Extended),
                    (Short, Short, false) => (true, Short, true, Short),
                    (Short, Extended, false) => (true, Short, true, Extended),
                    (Extended, Short, false) => (true, Extended, true, Short),
                    (Short, Extended, true) => (true, Short, false, Extended),
                    (Extended, Short, true) => (true, Extended, false, Short),
                    (Short, Short, true) => (true, Short, false, Short),
                    _ => return None,
                })
            }
            _ => None,
        }
    }

    /// Return the IEEE 802.15.4 destination [`Address`] if not absent.
    pub fn dst_address(&self, fc: &FrameControl<T>) -> Option<Address> {
        if let Some((dst_pan_id, dst_addr, _, _)) = self.address_present_flags(fc) {
            let offset = if dst_pan_id { 2 } else { 0 };

            match dst_addr {
                AddressingMode::Absent => Some(Address::Absent),
                AddressingMode::Short => {
                    let mut raw = [0u8; 2];
                    raw.clone_from_slice(&self.buffer.as_ref()[offset..offset + 2]);
                    raw.reverse();
                    Some(Address::short_from_bytes(raw))
                }
                AddressingMode::Extended => {
                    let mut raw = [0u8; 8];
                    raw.clone_from_slice(&self.buffer.as_ref()[offset..offset + 8]);
                    raw.reverse();
                    Some(Address::extended_from_bytes(raw))
                }
                AddressingMode::Unknown => None,
            }
        } else {
            None
        }
    }

    /// Return the IEEE 802.15.4 source [`Address`] if not absent.
    pub fn src_address(&self, fc: &FrameControl<T>) -> Option<Address> {
        if let Some((dst_pan_id, dst_addr, src_pan_id, src_addr)) = self.address_present_flags(fc) {
            let mut offset = if dst_pan_id { 2 } else { 0 };
            offset += dst_addr.size();
            offset += if src_pan_id { 2 } else { 0 };

            match src_addr {
                AddressingMode::Absent => Some(Address::Absent),
                AddressingMode::Short => {
                    let mut raw = [0u8; 2];
                    raw.clone_from_slice(&self.buffer.as_ref()[offset..offset + 2]);
                    raw.reverse();
                    Some(Address::short_from_bytes(raw))
                }
                AddressingMode::Extended => {
                    let mut raw = [0u8; 8];
                    raw.clone_from_slice(&self.buffer.as_ref()[offset..offset + 8]);
                    raw.reverse();
                    Some(Address::extended_from_bytes(raw))
                }
                AddressingMode::Unknown => None,
            }
        } else {
            None
        }
    }

    /// Return the IEEE 802.15.4 destination PAN ID if not elided.
    pub fn dst_pan_id(&self, fc: &FrameControl<T>) -> Option<u16> {
        if let Some((true, _, _, _)) = self.address_present_flags(fc) {
            let b = &self.buffer.as_ref()[..2];
            Some(u16::from_le_bytes([b[0], b[1]]))
        } else {
            None
        }
    }

    /// Return the IEEE 802.15.4 source PAN ID if not elided.
    pub fn src_pan_id(&self, fc: &FrameControl<T>) -> Option<u16> {
        if let Some((dst_pan_id, dst_addr, true, _)) = self.address_present_flags(fc) {
            let mut offset = if dst_pan_id { 2 } else { 0 };
            offset += dst_addr.size();

            let b = &self.buffer.as_ref()[offset..][..2];
            Some(u16::from_le_bytes([b[0], b[1]]))
        } else {
            None
        }
    }

    pub fn fmt(&self, f: &mut core::fmt::Formatter<'_>, fc: &FrameControl<T>) -> core::fmt::Result {
        writeln!(f, "Addressing Fields")?;

        if let Some(id) = self.dst_pan_id(fc) {
            writeln!(f, "  dst pan id: {:0x}", id)?;
        }

        if let Some(addr) = self.dst_address(fc) {
            writeln!(f, "  dst address: {}", addr)?;
        }

        if let Some(id) = self.src_pan_id(fc) {
            writeln!(f, "  src pan id: {:0x}", id)?;
        }

        if let Some(addr) = self.src_address(fc) {
            writeln!(f, "  src address: {}", addr)?;
        }

        Ok(())
    }
}

impl<T: AsRef<[u8]> + AsMut<[u8]>> AddressingFields<T> {
    pub fn write_fields(&mut self, fields: &AddressingFieldsRepr) {
        let mut offset = 0;

        if let Some(id) = fields.dst_pan_id {
            let b = &mut self.buffer.as_mut()[offset..][..2];
            b.copy_from_slice(&id.to_le_bytes());
            offset += 2;
        }

        if let Some(addr) = fields.dst_address {
            let b = &mut self.buffer.as_mut()[offset..][..addr.len()];
            b.copy_from_slice(addr.as_bytes());
            offset += addr.len();
        }

        if let Some(id) = fields.src_pan_id {
            let b = &mut self.buffer.as_mut()[offset..][..2];
            b.copy_from_slice(&id.to_le_bytes());
            offset += 2;
        }

        if let Some(addr) = fields.src_address {
            let b = &mut self.buffer.as_mut()[offset..][..addr.len()];
            b.copy_from_slice(addr.as_bytes());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_broadcast() {
        assert!(Address::BROADCAST.is_broadcast());
        assert!(Address::Short([0xff, 0xff]).is_broadcast());
        assert!(!Address::Short([0xff, 0xfe]).is_broadcast());

        assert!(!Address::BROADCAST.is_unicast());
        assert!(!Address::Short([0xff, 0xff]).is_unicast());
        assert!(Address::Short([0xff, 0xfe]).is_unicast());
    }

    #[test]
    fn as_bytes() {
        assert_eq!(Address::BROADCAST.as_bytes(), &[0xff, 0xff]);
        assert_eq!(Address::Short([0xff, 0xff]).as_bytes(), &[0xff, 0xff]);
        assert_eq!(Address::Short([0xff, 0xfe]).as_bytes(), &[0xff, 0xfe]);
        assert_eq!(Address::Extended([0xff; 8]).as_bytes(), &[0xff; 8]);
        assert_eq!(Address::Extended([0x01; 8]).as_bytes(), &[0x01; 8]);
        assert_eq!(Address::Absent.as_bytes(), &[]);
    }

    #[test]
    fn from_bytes() {
        assert_eq!(
            Address::from_bytes(&[0xff, 0xff]),
            Address::Short([0xff, 0xff])
        );
        assert_eq!(
            Address::from_bytes(&[0xff, 0xfe]),
            Address::Short([0xff, 0xfe])
        );
        assert_eq!(
            Address::from_bytes(&[0xff; 8]),
            Address::Extended([0xff; 8])
        );
        assert_eq!(
            Address::from_bytes(&[0x01; 8]),
            Address::Extended([0x01; 8])
        );
        assert_eq!(Address::from_bytes(&[]), Address::Absent);
    }

    #[test]
    #[should_panic]
    fn from_bytes_panic() {
        Address::from_bytes(&[0xff, 0xff, 0xff]);
    }
}
