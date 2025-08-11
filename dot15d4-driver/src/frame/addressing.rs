//! IEEE 802.15.4 addressing related fields.
use core::{fmt::Debug, ops::Range};

use dot15d4_util::{Error, Result};

use super::{FrameControl, FrameVersion};

const BROADCAST_ADDR_DATA: [u8; 2] = [0xff, 0xff];
/// The broadcast PAN id.
pub const BROADCAST_PAN_ID: PanId<&'static [u8]> = PanId(&BROADCAST_ADDR_DATA);

/// IEEE 802.15.4 addressing mode.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum AddressingMode {
    /// The address is absent.
    Absent = 0b00,
    /// The address is a short address.
    Short = 0b10,
    /// The address is an extended address.
    Extended = 0b11,
    /// Unknown addressing mode.
    Unknown,
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

impl AddressingMode {
    /// Length of an address with the given addressing mode.
    pub const fn length(&self) -> u16 {
        match self {
            AddressingMode::Absent => 0,
            AddressingMode::Short => 2,
            AddressingMode::Extended => 8,
            AddressingMode::Unknown => 0,
        }
    }
}

/// Short address field.
///
/// The internal representation is little-endian.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct ShortAddress<Bytes>(Bytes);

impl ShortAddress<[u8; 2]> {
    pub const fn new_owned(le_bytes: [u8; 2]) -> Self {
        Self(le_bytes)
    }
}

impl<Bytes: AsRef<[u8]>> ShortAddress<Bytes> {
    pub fn new(le_bytes: Bytes) -> Self {
        debug_assert_eq!(le_bytes.as_ref().len(), 2);
        Self(le_bytes)
    }

    #[cfg(feature = "std")]
    const fn new_unchecked(le_bytes: Bytes) -> Self {
        Self(le_bytes)
    }

    pub fn into_be_bytes(&self) -> [u8; 2] {
        // Safety: Length was checked on instantiation.
        let mut be_bytes = <[u8; 2]>::try_from(self.as_ref()).unwrap();
        be_bytes.reverse();
        be_bytes
    }

    pub fn from_be_bytes(mut be_bytes: [u8; 2]) -> ShortAddress<[u8; 2]> {
        be_bytes.reverse();
        ShortAddress::new(be_bytes)
    }

    pub fn into_u16(&self) -> u16 {
        // Safety: Length was checked on instantiation.
        u16::from_le_bytes(self.0.as_ref().try_into().unwrap())
    }

    pub fn from_u16(short_addr: u16) -> ShortAddress<[u8; 2]> {
        ShortAddress::new(short_addr.to_le_bytes())
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> ShortAddress<Bytes> {
    pub fn set_le_bytes<SrcBytes: AsRef<[u8]>>(&mut self, le_bytes: SrcBytes) {
        debug_assert_eq!(le_bytes.as_ref().len(), 2, "invalid");
        self.as_mut().clone_from_slice(le_bytes.as_ref());
    }

    pub fn set_be_bytes<SrcBytes: AsRef<[u8]>>(&mut self, be_bytes: SrcBytes) {
        debug_assert_eq!(be_bytes.as_ref().len(), 2, "invalid");
        let mut be_bytes = <[u8; 2]>::try_from(be_bytes.as_ref()).expect("invalid");
        be_bytes.reverse();
        self.as_mut().clone_from_slice(be_bytes.as_ref());
    }

    pub fn set_u16(&mut self, short_addr: u16) {
        self.as_mut().clone_from_slice(&short_addr.to_le_bytes());
    }
}

impl<Bytes: AsRef<[u8]>> AsRef<[u8]> for ShortAddress<Bytes> {
    /// Little endian
    fn as_ref(&self) -> &[u8] {
        self.0.as_ref()
    }
}

impl<Bytes: AsMut<[u8]>> AsMut<[u8]> for ShortAddress<Bytes> {
    /// Little endian
    fn as_mut(&mut self) -> &mut [u8] {
        self.0.as_mut()
    }
}

/// Extended address field.
///
/// The internal representation is little-endian.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct ExtendedAddress<Bytes>(Bytes);

impl ExtendedAddress<[u8; 8]> {
    pub const fn new_owned(le_bytes: [u8; 8]) -> Self {
        Self(le_bytes)
    }
}

impl<Bytes: AsRef<[u8]>> ExtendedAddress<Bytes> {
    pub fn new(le_bytes: Bytes) -> Self {
        debug_assert_eq!(le_bytes.as_ref().len(), 8);
        Self(le_bytes)
    }

    #[cfg(feature = "std")]
    const fn new_unchecked(le_bytes: Bytes) -> Self {
        Self(le_bytes)
    }

    pub fn into_be_bytes(&self) -> [u8; 8] {
        // Safety: Length was checked on instantiation.
        let mut be_bytes = <[u8; 8]>::try_from(self.as_ref()).unwrap();
        be_bytes.reverse();
        be_bytes
    }

    pub fn from_be_bytes(mut be_bytes: [u8; 8]) -> ExtendedAddress<[u8; 8]> {
        be_bytes.reverse();
        ExtendedAddress::new(be_bytes)
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> ExtendedAddress<Bytes> {
    pub fn set_le_bytes<SrcBytes: AsRef<[u8]>>(&mut self, le_bytes: SrcBytes) {
        debug_assert_eq!(le_bytes.as_ref().len(), 8, "invalid");
        self.as_mut().clone_from_slice(le_bytes.as_ref());
    }

    pub fn set_be_bytes<SrcBytes: AsRef<[u8]>>(&mut self, be_bytes: SrcBytes) {
        debug_assert_eq!(be_bytes.as_ref().len(), 8, "invalid");
        let mut be_bytes = <[u8; 2]>::try_from(be_bytes.as_ref()).expect("invalid");
        be_bytes.reverse();
        self.as_mut().clone_from_slice(be_bytes.as_ref());
    }
}

impl<Bytes: AsRef<[u8]>> AsRef<[u8]> for ExtendedAddress<Bytes> {
    /// Little endian
    fn as_ref(&self) -> &[u8] {
        self.0.as_ref()
    }
}

impl<Bytes: AsMut<[u8]>> AsMut<[u8]> for ExtendedAddress<Bytes> {
    /// Little endian
    fn as_mut(&mut self) -> &mut [u8] {
        self.0.as_mut()
    }
}

/// PAN id field.
///
/// The internal representation is little-endian.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct PanId<Bytes>(Bytes);

impl PanId<[u8; 2]> {
    pub const fn new_owned(le_bytes: [u8; 2]) -> Self {
        Self(le_bytes)
    }
}

impl<Bytes: AsRef<[u8]>> PanId<Bytes> {
    pub fn new(le_bytes: Bytes) -> Self {
        debug_assert_eq!(le_bytes.as_ref().len(), 2);
        Self(le_bytes)
    }

    pub fn into_be_bytes(&self) -> [u8; 2] {
        // Safety: Length was checked on instantiation.
        let mut be_bytes = <[u8; 2]>::try_from(self.as_ref()).unwrap();
        be_bytes.reverse();
        be_bytes
    }

    pub fn from_be_bytes(mut be_bytes: [u8; 2]) -> PanId<[u8; 2]> {
        be_bytes.reverse();
        PanId::new(be_bytes)
    }

    pub fn into_u16(&self) -> u16 {
        // Safety: Length was checked on instantiation.
        u16::from_le_bytes(self.0.as_ref().try_into().unwrap())
    }

    pub fn from_u16(pan_id: u16) -> PanId<[u8; 2]> {
        PanId::new(pan_id.to_le_bytes())
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> PanId<Bytes> {
    pub fn set_le_bytes<SrcBytes: AsRef<[u8]>>(&mut self, le_bytes: SrcBytes) {
        debug_assert_eq!(le_bytes.as_ref().len(), 2, "invalid");
        self.as_mut().clone_from_slice(le_bytes.as_ref());
    }

    pub fn set_be_bytes<SrcBytes: AsRef<[u8]>>(&mut self, be_bytes: SrcBytes) {
        debug_assert_eq!(be_bytes.as_ref().len(), 2, "invalid");
        let mut be_bytes = <[u8; 2]>::try_from(be_bytes.as_ref()).expect("invalid");
        be_bytes.reverse();
        self.as_mut().clone_from_slice(be_bytes.as_ref());
    }

    pub fn set_u16(&mut self, pan_id: u16) {
        self.as_mut().clone_from_slice(&pan_id.to_le_bytes());
    }
}

impl<Bytes: AsRef<[u8]>> AsRef<[u8]> for PanId<Bytes> {
    /// Little endian
    fn as_ref(&self) -> &[u8] {
        self.0.as_ref()
    }
}

impl<Bytes: AsMut<[u8]>> AsMut<[u8]> for PanId<Bytes> {
    /// Little endian
    fn as_mut(&mut self) -> &mut [u8] {
        self.0.as_mut()
    }
}

/// An IEEE 802.15.4 address.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub enum Address<Bytes> {
    /// The address is absent.
    Absent,
    /// A short address.
    Short(ShortAddress<Bytes>),
    /// An extended address.
    Extended(ExtendedAddress<Bytes>),
}

impl<Bytes> Address<Bytes> {
    /// The broadcast address.
    pub const BROADCAST_ADDR: Address<&'static [u8]> =
        Address::Short(ShortAddress(&BROADCAST_ADDR_DATA));

    /// Return the length of the address in octets.
    #[allow(clippy::len_without_is_empty)]
    pub fn length(&self) -> usize {
        match self {
            Address::Absent => 0,
            Address::Short(_) => 2,
            Address::Extended(_) => 8,
        }
    }

    /// Query whether the address is absent.
    pub fn is_absent(&self) -> bool {
        matches!(self, Address::Absent)
    }

    /// Query whether the address is short.
    pub fn is_short(&self) -> bool {
        matches!(self, Address::Short(_))
    }

    /// Query whether the address is extended.
    pub fn is_extended(&self) -> bool {
        matches!(self, Address::Extended(_))
    }
}

#[cfg(feature = "std")]
impl Address<Vec<u8>> {
    /// Parse an address from a string.
    ///
    /// The string is assumed to encode bytes in little-endian order.
    pub fn parse(address: &str) -> Result<Self> {
        if address.is_empty() {
            return Ok(Address::Absent);
        }

        let parts: std::vec::Vec<&str> = address.split(':').collect();
        match parts.len() {
            2 => {
                let mut le_bytes = Vec::with_capacity(2);
                for part in parts.iter() {
                    le_bytes.push(u8::from_str_radix(part, 16).unwrap());
                }
                Ok(Address::Short(ShortAddress::new_unchecked(le_bytes)))
            }
            8 => {
                let mut le_bytes = Vec::with_capacity(8);
                for part in parts.iter() {
                    le_bytes.push(u8::from_str_radix(part, 16).unwrap());
                }
                Ok(Address::Extended(ExtendedAddress::new_unchecked(le_bytes)))
            }
            _ => Err(Error),
        }
    }
}

#[cfg(feature = "std")]
impl From<Address<&[u8]>> for Address<Vec<u8>> {
    fn from(value: Address<&[u8]>) -> Self {
        match value {
            Address::Absent => Address::Absent,
            Address::Short(short_address) => {
                Address::Short(ShortAddress::new(short_address.0.to_vec()))
            }
            Address::Extended(extended_address) => {
                Address::Extended(ExtendedAddress::new(extended_address.0.to_vec()))
            }
        }
    }
}

impl<Bytes: AsRef<[u8]>> Address<Bytes> {
    /// Query whether the address is an unicast address.
    pub fn is_unicast(&self) -> bool {
        !self.is_broadcast()
    }

    /// Query whether this address is the broadcast address.
    pub fn is_broadcast(&self) -> bool {
        match self {
            Address::Absent => false,
            Address::Short(short_address) => {
                *short_address.as_ref() == *BROADCAST_ADDR_DATA.as_ref()
            }
            Address::Extended(_) => false,
        }
    }

    /// Derives a short address from an extended address' first two bytes, a
    /// short address remains unchanged.
    ///
    /// TODO: This is not a valid approach. We need to store an explicit
    ///       extended-to-short address map - at least on the coordinator.
    ///
    /// Note: This is not an IEEE 802.15.4 standard feature.
    pub fn to_short(&self) -> Option<Address<&[u8]>> {
        match self {
            Address::Short(le_bytes) => Some(Address::Short(ShortAddress::new(le_bytes.as_ref()))),
            // Safety: The slice always has the correct size.
            Address::Extended(le_bytes) => {
                Some(Address::Short(ShortAddress::new(&le_bytes.as_ref()[..2])))
            }
            _ => None,
        }
    }

    /// Return the address as a slice of little-endian ordered bytes.
    pub fn as_le_bytes(&self) -> &[u8] {
        match self {
            Address::Absent => &[],
            Address::Short(le_bytes) => le_bytes.as_ref(),
            Address::Extended(le_bytes) => le_bytes.as_ref(),
        }
    }
}

impl<Bytes: AsMut<[u8]>> Address<Bytes> {
    pub fn as_le_bytes_mut(&mut self) -> &mut [u8] {
        match self {
            Address::Absent => &mut [],
            Address::Short(le_bytes) => le_bytes.as_mut(),
            Address::Extended(le_bytes) => le_bytes.as_mut(),
        }
    }

    pub fn set<Src: AsRef<[u8]>>(&mut self, src: &Address<Src>) -> Result<()> {
        let src = src.as_le_bytes();
        let dst = self.as_le_bytes_mut();
        if src.len() != dst.len() {
            return Err(Error);
        }

        dst.copy_from_slice(src);

        Ok(())
    }
}

impl<'bytes, Bytes: AsRef<[u8]> + TryFrom<&'bytes [u8]>> Address<Bytes>
where
    <Bytes as TryFrom<&'bytes [u8]>>::Error: Debug,
{
    /// Create an [`Address`] from a slice of bytes.
    ///
    /// Panics if the given slice is not 0, 2 or 8 bytes long.
    pub fn from_le_bytes(le_bytes: &'bytes [u8]) -> Self {
        if le_bytes.is_empty() {
            Address::Absent
        } else if le_bytes.len() == 2 {
            // Safety: Slice length has been checked explicitly.
            Address::Short(ShortAddress::new(Bytes::try_from(le_bytes).unwrap()))
        } else if le_bytes.len() == 8 {
            // Safety: Slice length has been checked explicitly.
            Address::Extended(ExtendedAddress::new(Bytes::try_from(le_bytes).unwrap()))
        } else {
            panic!("invalid")
        }
    }
}

impl<Bytes> From<Address<Bytes>> for AddressingMode {
    fn from(value: Address<Bytes>) -> Self {
        match value {
            Address::Absent => AddressingMode::Absent,
            Address::Short(_) => AddressingMode::Short,
            Address::Extended(_) => AddressingMode::Extended,
        }
    }
}

impl<Bytes: AsRef<[u8]>> core::fmt::Display for Address<Bytes> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Address::Absent => write!(f, "absent"),
            Address::Short(bytes) => {
                let bytes = bytes.as_ref();
                write!(f, "{:02x}:{:02x}", bytes[0], bytes[1])
            }
            Address::Extended(bytes) => {
                let bytes = bytes.as_ref();
                write!(
                    f,
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                    bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]
                )
            }
        }
    }
}

/// A reader/writer for the IEEE 802.15.4 Addressing Fields.
#[derive(Debug, PartialEq, Eq)]
pub struct AddressingFields<Bytes> {
    dst_addr_offset: u8,
    src_pan_id_offset: u8,
    src_addr_offset: u8,
    last_byte: u8,
    le_bytes: Bytes,
}

impl<Bytes: AsRef<[u8]>> core::fmt::Display for AddressingFields<Bytes> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "Addressing Fields")?;

        if let Some(dst_pan_id) = self.dst_pan_id() {
            writeln!(f, "  dst pan id: {:0x}", dst_pan_id.into_u16())?;
        }

        if let Some(dst_addr) = self.dst_address() {
            writeln!(f, "  dst address: {dst_addr}")?;
        }

        if let Some(src_pan_id) = self.src_pan_id() {
            writeln!(f, "  src pan id: {:0x}", src_pan_id.into_u16())?;
        }

        if let Some(src_addr) = self.src_address() {
            writeln!(f, "  src address: {src_addr}")?;
        }

        Ok(())
    }
}

impl<Bytes: AsRef<[u8]>> AddressingFields<Bytes> {
    /// Create a new [`AddressingFields`] reader/writer from a given
    /// little-endian bytes slice.
    ///
    /// # Errors
    ///
    /// This function will check the length of the buffer to ensure it is large
    /// enough to contain the addressing fields. If the buffer is too small, an
    /// error will be returned.
    pub fn new(le_bytes: Bytes, repr: AddressingRepr) -> Result<Self> {
        let expected_len = repr.addressing_fields_length()? as usize;
        if le_bytes.as_ref().len() != expected_len {
            return Err(Error);
        }

        // Safety: We checked the length of the given bytes buffer.
        unsafe { Self::new_unchecked(le_bytes, repr) }
    }

    /// Create a new [`AddressingFields`] reader/writer from a given
    /// little-endian bytes slice without checking the length.
    ///
    /// # Safety
    ///
    /// Requires the length of the bytes buffer to match the address
    /// representation exactly.
    pub unsafe fn new_unchecked(le_bytes: Bytes, repr: AddressingRepr) -> Result<Self> {
        let [dst_pan_id_len, dst_addr_len, src_pan_id_len, src_addr_len] =
            repr.addressing_fields_lengths()?;

        let dst_addr_offset = dst_pan_id_len;
        let src_pan_id_offset = dst_addr_offset + dst_addr_len;
        let src_addr_offset = src_pan_id_offset + src_pan_id_len;
        let last_byte = src_addr_offset + src_addr_len;

        Ok(Self {
            dst_addr_offset,
            src_pan_id_offset,
            src_addr_offset,
            last_byte,
            le_bytes,
        })
    }

    /// Return the length of the Addressing Fields in octets.
    #[allow(clippy::len_without_is_empty)]
    pub fn length(&self) -> usize {
        // Safety: We checked that the length matched exactly when instantiating
        //         the object.
        self.le_bytes.as_ref().len()
    }

    /// Return the IEEE 802.15.4 destination [`Address`] if not absent.
    pub fn dst_address(&self) -> Option<Address<&[u8]>> {
        self.addr_from_range(self.dst_addr_range())
    }

    /// Return the IEEE 802.15.4 source [`Address`] if not absent.
    pub fn src_address(&self) -> Option<Address<&[u8]>> {
        self.addr_from_range(self.src_addr_range())
    }

    /// Return the IEEE 802.15.4 destination PAN ID if not elided.
    pub fn dst_pan_id(&self) -> Option<PanId<&[u8]>> {
        self.pan_id_from_range(self.dst_pan_id_range())
    }

    /// Return the IEEE 802.15.4 source PAN ID if not elided.
    pub fn src_pan_id(&self) -> Option<PanId<&[u8]>> {
        self.pan_id_from_range(self.src_pan_id_range())
    }

    fn addr_from_range(&self, range: Range<usize>) -> Option<Address<&[u8]>> {
        let addr = &self.le_bytes.as_ref()[range];
        match addr.len() {
            0 => Some(Address::Absent),
            2 => Some(Address::Short(ShortAddress(addr))),
            8 => Some(Address::Extended(ExtendedAddress(addr))),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }

    fn pan_id_from_range(&self, range: Range<usize>) -> Option<PanId<&[u8]>> {
        let pan_id = &self.le_bytes.as_ref()[range];
        match pan_id.len() {
            0 => None,
            2 => Some(PanId::new(pan_id)),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }

    const fn dst_pan_id_range(&self) -> Range<usize> {
        0..self.dst_addr_offset as usize
    }

    const fn dst_addr_range(&self) -> Range<usize> {
        self.dst_addr_offset as usize..self.src_pan_id_offset as usize
    }

    const fn src_pan_id_range(&self) -> Range<usize> {
        self.src_pan_id_offset as usize..self.src_addr_offset as usize
    }

    const fn src_addr_range(&self) -> Range<usize> {
        self.src_addr_offset as usize..self.last_byte as usize
    }
}

impl<'bytes> AddressingFields<&'bytes [u8]> {
    /// Return the IEEE 802.15.4 destination [`Address`] if not absent.
    pub fn into_dst_address(self) -> Option<Address<&'bytes [u8]>> {
        let dst_addr_range = self.dst_addr_range();
        self.into_addr_from_range(dst_addr_range)
    }

    /// Return the IEEE 802.15.4 source [`Address`] if not absent.
    pub fn into_src_address(self) -> Option<Address<&'bytes [u8]>> {
        let src_addr_range = self.src_addr_range();
        self.into_addr_from_range(src_addr_range)
    }

    /// Return the IEEE 802.15.4 destination PAN ID if not elided.
    pub fn into_dst_pan_id(self) -> Option<PanId<&'bytes [u8]>> {
        let dst_pan_id_range = self.dst_pan_id_range();
        self.into_pan_id_from_range(dst_pan_id_range)
    }

    /// Return the IEEE 802.15.4 source PAN ID if not elided.
    pub fn into_src_pan_id(self) -> Option<PanId<&'bytes [u8]>> {
        let src_pan_id_range = self.src_pan_id_range();
        self.into_pan_id_from_range(src_pan_id_range)
    }

    fn into_addr_from_range(self, range: Range<usize>) -> Option<Address<&'bytes [u8]>> {
        let addr = &self.le_bytes[range];
        match addr.len() {
            0 => Some(Address::Absent),
            2 => Some(Address::Short(ShortAddress(addr))),
            8 => Some(Address::Extended(ExtendedAddress(addr))),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }

    fn into_pan_id_from_range(self, range: Range<usize>) -> Option<PanId<&'bytes [u8]>> {
        let pan_id = &self.le_bytes[range];
        match pan_id.len() {
            0 => None,
            2 => Some(PanId::new(pan_id)),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }
}

impl<Bytes: AsRef<[u8]> + AsMut<[u8]>> AddressingFields<Bytes> {
    /// Return the IEEE 802.15.4 destination [`Address`] if not absent.
    pub fn dst_address_mut(&mut self) -> Option<Address<&mut [u8]>> {
        self.addr_from_range_mut(self.dst_addr_range())
    }

    /// Return the IEEE 802.15.4 source [`Address`] if not absent.
    pub fn src_address_mut(&mut self) -> Option<Address<&mut [u8]>> {
        self.addr_from_range_mut(self.src_addr_range())
    }

    /// Return the IEEE 802.15.4 destination PAN ID if not elided.
    pub fn dst_pan_id_mut(&mut self) -> Option<PanId<&mut [u8]>> {
        self.pan_id_from_range_mut(self.dst_pan_id_range())
    }

    /// Return the IEEE 802.15.4 source PAN ID if not elided.
    pub fn src_pan_id_mut(&mut self) -> Option<PanId<&mut [u8]>> {
        self.pan_id_from_range_mut(self.src_pan_id_range())
    }

    fn addr_from_range_mut(&mut self, range: Range<usize>) -> Option<Address<&mut [u8]>> {
        let addr = &mut self.le_bytes.as_mut()[range];
        match addr.len() {
            0 => Some(Address::Absent),
            2 => Some(Address::Short(ShortAddress(addr))),
            4 => Some(Address::Extended(ExtendedAddress(addr))),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }

    fn pan_id_from_range_mut(&mut self, range: Range<usize>) -> Option<PanId<&mut [u8]>> {
        let pan_id = &mut self.le_bytes.as_mut()[range];
        match pan_id.len() {
            0 => None,
            2 => Some(PanId::new(pan_id)),
            // Safety: This is a guarantee of AddressingRepr.
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PanIdCompressionRepr {
    Yes,
    No,
    Legacy,
} // 1 byte

#[derive(Debug, Clone, PartialEq, Eq, Copy)]
pub struct AddressingRepr {
    pub(crate) dst: AddressingMode,
    pub(crate) src: AddressingMode,
    pub(crate) pan_ids_equal: bool,
    pub(crate) pan_id_compression: PanIdCompressionRepr,
} // 4 bytes

impl AddressingRepr {
    /// Instantiate a new addressing representation.
    ///
    /// Safety: The given addressing modes must be known (i.e. absent, short or
    ///         extended).
    pub const fn new(
        dst: AddressingMode,
        src: AddressingMode,
        pan_ids_equal: bool,
        pan_id_compression: PanIdCompressionRepr,
    ) -> Self {
        if matches!(dst, AddressingMode::Unknown) || matches!(src, AddressingMode::Unknown) {
            panic!("invalid")
        }

        Self {
            dst,
            src,
            pan_ids_equal,
            pan_id_compression,
        }
    }

    /// Instantiate a new addressing representation with pre IEEE 802.15.4-2015
    /// addressing mode.
    ///
    /// Safety: The given addressing modes must be known.
    pub const fn new_legacy_addressing(
        dst: AddressingMode,
        src: AddressingMode,
        pan_ids_equal: bool,
    ) -> Self {
        Self::new(dst, src, pan_ids_equal, PanIdCompressionRepr::Legacy)
    }

    /// Safety: The frame version and addressing modes must be known.
    pub fn from_frame_control<Bytes: AsRef<[u8]>>(
        frame_control: FrameControl<Bytes>,
    ) -> Result<Self> {
        let dst = frame_control.dst_addressing_mode();
        let src = frame_control.src_addressing_mode();
        let frame_version = frame_control.frame_version();
        let (pan_id_compression, pan_ids_equal) = match frame_version {
            FrameVersion::Ieee802154_2003 | FrameVersion::Ieee802154_2006 => {
                let pan_id_compression = PanIdCompressionRepr::Legacy;

                // See see IEEE 802.15.4-2024, section 7.2.2.6
                let pan_ids_equal = if !matches!(dst, AddressingMode::Absent)
                    && !matches!(src, AddressingMode::Absent)
                {
                    // - If both destination and source addressing information is
                    //   present, the MAC sublayer shall compare the destination and
                    //   source PAN identifiers. If the PAN IDs are identical, the
                    //   PAN ID Compression field shall be set to one, and the
                    //   Source PAN ID field shall be omitted from the transmitted
                    //   frame. If the PAN IDs are different, the PAN ID Compression
                    //   field shall be set to zero, and both Destination PAN ID
                    //   field and Source PAN ID fields shall be included in the
                    //   transmitted frame.
                    frame_control.pan_id_compression()
                } else {
                    // - If only either the destination or the source addressing
                    //   information is present, the PAN ID Compression field shall
                    //   be set to zero, and the PAN ID field of the single address
                    //   shall be included in the transmitted frame.
                    false
                };

                (pan_id_compression, pan_ids_equal)
            }
            FrameVersion::Ieee802154 => {
                let pan_id_compression = if frame_control.pan_id_compression() {
                    PanIdCompressionRepr::Yes
                } else {
                    PanIdCompressionRepr::No
                };

                // If both the destination and source addressing information is present
                // and either is a short address, the MAC sublayer shall compare the
                // destination and source PAN IDs and the PAN ID Compression field shall
                // be set to zero if and only if the PAN identifiers are identical,
                // see IEEE 802.15.4-2024, section 7.2.2.6.
                let pan_ids_equal = if matches!(dst, AddressingMode::Short)
                    || matches!(src, AddressingMode::Short)
                {
                    matches!(pan_id_compression, PanIdCompressionRepr::Yes)
                } else {
                    false
                };

                (pan_id_compression, pan_ids_equal)
            }
            FrameVersion::Unknown => return Err(Error),
        };

        let addressing = Self::new(dst, src, pan_ids_equal, pan_id_compression);
        Ok(addressing)
    }

    /// Addressing fields length
    pub const fn addressing_fields_length(&self) -> Result<u16> {
        if let Ok([dst_pan_id_len, dst_addr_len, src_pan_id_len, src_addr_len]) =
            self.addressing_fields_lengths()
        {
            // fast const-compat calculation
            Ok((dst_pan_id_len + dst_addr_len + src_pan_id_len + src_addr_len) as u16)
        } else {
            Err(Error)
        }
    }

    /// Pan ID compression
    pub const fn pan_id_compression(&self) -> bool {
        match self.pan_id_compression {
            PanIdCompressionRepr::Yes | PanIdCompressionRepr::No => true,
            PanIdCompressionRepr::Legacy => match (self.dst, self.src) {
                (AddressingMode::Short, AddressingMode::Short)
                | (AddressingMode::Short, AddressingMode::Extended)
                | (AddressingMode::Extended, AddressingMode::Short)
                | (AddressingMode::Extended, AddressingMode::Extended) => self.pan_ids_equal,

                _ => false,
            },
        }
    }

    /// Destination [`AddressingMode`]
    pub const fn dst_addr_mode(&self) -> AddressingMode {
        self.dst
    }

    /// Source [`AddressingMode`]
    pub const fn src_addr_mode(&self) -> AddressingMode {
        self.src
    }

    /// Returns (dst_pan_id_present, dst_address_mode, src_pan_id_present, src_address_mode)
    const fn address_present_flags(&self) -> Result<(bool, AddressingMode, bool, AddressingMode)> {
        use AddressingMode::*;
        match self.pan_id_compression {
            // IEEE 802.15.4-2006 or earlier.
            PanIdCompressionRepr::Legacy => {
                match (self.dst, self.src, self.pan_ids_equal) {
                    // If both destination and source address information is
                    // present, and the destination and source PAN IDs are
                    // identical, then the source PAN ID is omitted.

                    // In the following case, the destination and source PAN IDs
                    // are not identical, and thus both are present.
                    (dst @ (Short | Extended), src @ (Short | Extended), false) => {
                        Ok((true, dst, true, src))
                    }

                    // In the following case, the destination and source PAN IDs
                    // are identical, and thus only the destination PAN ID is
                    // present.
                    (dst @ (Short | Extended), src @ (Short | Extended), true) => {
                        Ok((true, dst, false, src))
                    }

                    // If either the destination or the source address is
                    // present, then the PAN ID of the corresponding address is
                    // present and the PAN ID compression field is set to 0.
                    (Absent, src @ (Short | Extended), false) => Ok((false, Absent, true, src)),
                    (dst @ (Short | Extended), Absent, false) => Ok((true, dst, false, Absent)),

                    // All other cases are invalid.
                    _ => Err(Error),
                }
            }

            // IEEE 802.15.4-2015 and beyond.
            PanIdCompressionRepr::Yes | PanIdCompressionRepr::No => {
                let pan_id_compression =
                    matches!(self.pan_id_compression, PanIdCompressionRepr::Yes);

                match (self.dst, self.src, pan_id_compression) {
                    (Absent, Absent, false) => Ok((false, Absent, false, Absent)),
                    (Absent, Absent, true) => Ok((true, Absent, false, Absent)),
                    (dst, Absent, false) if !matches!(dst, Absent) => {
                        Ok((true, dst, false, Absent))
                    }
                    (dst, Absent, true) if !matches!(dst, Absent) => {
                        Ok((false, dst, false, Absent))
                    }
                    (Absent, src, false) if !matches!(src, Absent) => {
                        Ok((false, Absent, true, src))
                    }
                    (Absent, src, true) if !matches!(src, Absent) => {
                        Ok((false, Absent, false, src))
                    }
                    (Extended, Extended, false) => Ok((true, Extended, false, Extended)),
                    (Extended, Extended, true) => Ok((false, Extended, false, Extended)),
                    (Short, Short, false) => Ok((true, Short, true, Short)),
                    (Short, Extended, false) => Ok((true, Short, true, Extended)),
                    (Extended, Short, false) => Ok((true, Extended, true, Short)),
                    (Short, Extended, true) => Ok((true, Short, false, Extended)),
                    (Extended, Short, true) => Ok((true, Extended, false, Short)),
                    (Short, Short, true) => Ok((true, Short, false, Short)),
                    _ => Err(Error),
                }
            }
        }
    }

    /// Returns [dst_pan_id_len, dst_address_len, src_pan_id_len, src_address_len]
    pub const fn addressing_fields_lengths(&self) -> Result<[u8; 4]> {
        if let Ok((dst_pan_id_present, dst_address_mode, src_pan_id_present, src_address_mode)) =
            self.address_present_flags()
        {
            Ok([
                if dst_pan_id_present { 2 } else { 0 },
                dst_address_mode.length() as u8,
                if src_pan_id_present { 2 } else { 0 },
                src_address_mode.length() as u8,
            ])
        } else {
            Err(Error)
        }
    }
}
#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;

    const BROADCAST_ADDR: Address<&[u8]> = Address::<&[u8]>::BROADCAST_ADDR;

    const SOME_EXTENDED_ADDRESS: ExtendedAddress<&[u8]> =
        ExtendedAddress::new_unchecked(&[0xff; 8]);
    const OTHER_EXTENDED_ADDRESS: ExtendedAddress<&[u8]> =
        ExtendedAddress::new_unchecked(&[0x01; 8]);

    const SOME_SHORT_ADDRESS: ShortAddress<&[u8]> = ShortAddress::new_unchecked(&[0xff, 0xff]);
    const OTHER_SHORT_ADDRESS: ShortAddress<&[u8]> = ShortAddress::new_unchecked(&[0xff, 0xfe]);

    #[test]
    fn address_type() {
        assert!(Address::<&[u8]>::Absent.is_absent());
        assert!(!Address::<&[u8]>::Absent.is_short());
        assert!(!Address::<&[u8]>::Absent.is_extended());

        assert!(!BROADCAST_ADDR.is_absent());
        assert!(BROADCAST_ADDR.is_short());
        assert!(!BROADCAST_ADDR.is_extended());

        assert!(!Address::Extended(SOME_EXTENDED_ADDRESS).is_absent());
        assert!(!Address::Extended(SOME_EXTENDED_ADDRESS).is_short());
        assert!(Address::Extended(SOME_EXTENDED_ADDRESS).is_extended());

        assert_eq!(Address::<&[u8]>::Absent.length(), 0);
        assert_eq!(BROADCAST_ADDR.length(), 2);
        assert_eq!(Address::Extended(SOME_EXTENDED_ADDRESS).length(), 8);
    }

    #[test]
    fn addressing_mode() {
        assert_eq!(AddressingMode::from(0b00), AddressingMode::Absent);
        assert_eq!(AddressingMode::from(0b10), AddressingMode::Short);
        assert_eq!(AddressingMode::from(0b11), AddressingMode::Extended);
        assert_eq!(AddressingMode::from(0b01), AddressingMode::Unknown);

        assert_eq!(AddressingMode::Unknown.length(), 0);
        assert_eq!(AddressingMode::Absent.length(), 0);
        assert_eq!(AddressingMode::Short.length(), 2);
        assert_eq!(AddressingMode::Extended.length(), 8);
    }

    #[test]
    fn is_broadcast() {
        assert!(BROADCAST_ADDR.is_broadcast());
        assert!(Address::Short(SOME_SHORT_ADDRESS).is_broadcast());
        assert!(!Address::Short(OTHER_SHORT_ADDRESS).is_broadcast());

        assert!(!BROADCAST_ADDR.is_unicast());
        assert!(!Address::Short(SOME_SHORT_ADDRESS).is_unicast());
        assert!(Address::Short(OTHER_SHORT_ADDRESS).is_unicast());
    }

    #[test]
    fn as_bytes() {
        assert_eq!(BROADCAST_ADDR.as_le_bytes(), &[0xff, 0xff]);
        assert_eq!(
            Address::Short(SOME_SHORT_ADDRESS).as_le_bytes(),
            &[0xff, 0xff]
        );
        assert_eq!(
            Address::Short(OTHER_SHORT_ADDRESS).as_le_bytes(),
            &[0xff, 0xfe]
        );
        assert_eq!(
            Address::Extended(SOME_EXTENDED_ADDRESS).as_le_bytes(),
            &[0xff; 8]
        );
        assert_eq!(
            Address::Extended(OTHER_EXTENDED_ADDRESS).as_le_bytes(),
            &[0x01; 8]
        );
        assert_eq!(Address::<&[u8]>::Absent.as_le_bytes(), &[] as &[u8]);
    }

    #[test]
    fn from_bytes() {
        assert_eq!(
            Address::from_le_bytes(&[0xff, 0xff]),
            Address::Short(SOME_SHORT_ADDRESS)
        );
        assert_eq!(
            Address::from_le_bytes(&[0xff, 0xfe]),
            Address::Short(OTHER_SHORT_ADDRESS)
        );
        assert_eq!(
            Address::from_le_bytes(&[0xff; 8]),
            Address::Extended(SOME_EXTENDED_ADDRESS)
        );
        assert_eq!(
            Address::from_le_bytes(&[0x01; 8]),
            Address::Extended(OTHER_EXTENDED_ADDRESS)
        );
        assert_eq!(Address::<&[u8]>::from_le_bytes(&[]), Address::Absent);
    }

    #[test]
    #[should_panic]
    fn from_bytes_panic() {
        Address::<&[u8]>::from_le_bytes(&[0xff, 0xff, 0xff]);
    }

    #[test]
    fn address_present_flags() {
        use AddressingMode::*;

        macro_rules! check {
            (($compression:expr, $dst:ident, $src:ident, $pan_ids_equal: literal) -> $expected:expr) => {
                assert_eq!(
                    AddressingRepr::new($dst, $src, $pan_ids_equal, $compression)
                        .address_present_flags()
                        .ok(),
                    $expected
                );
            };
        }

        check!((PanIdCompressionRepr::Legacy, Short, Short, false) -> Some((true, Short, true, Short)));
        check!((PanIdCompressionRepr::Legacy, Short, Short, true) -> Some((true, Short, false, Short)));
        check!((PanIdCompressionRepr::Legacy, Extended, Extended, false) -> Some((true, Extended, true, Extended)));
        check!((PanIdCompressionRepr::Legacy, Extended, Extended, true) -> Some((true, Extended, false, Extended)));
        check!((PanIdCompressionRepr::Legacy, Short, Extended, false) -> Some((true, Short, true, Extended)));
        check!((PanIdCompressionRepr::Legacy, Short, Extended, true) -> Some((true, Short, false, Extended)));
        check!((PanIdCompressionRepr::Legacy, Extended, Short, false) -> Some((true, Extended, true, Short)));
        check!((PanIdCompressionRepr::Legacy, Extended, Short, true) -> Some((true, Extended, false, Short)));
        check!((PanIdCompressionRepr::Legacy, Absent, Short, false) -> Some((false, Absent, true, Short)));
        check!((PanIdCompressionRepr::Legacy, Absent, Extended, false) -> Some((false, Absent, true, Extended)));
        check!((PanIdCompressionRepr::Legacy, Short, Absent, false) -> Some((true, Short, false, Absent)));
        check!((PanIdCompressionRepr::Legacy, Extended, Absent, false) -> Some((true, Extended, false, Absent)));
        check!((PanIdCompressionRepr::Legacy, Absent, Short, true) -> None);
        check!((PanIdCompressionRepr::Legacy, Absent, Extended, true) -> None);
        check!((PanIdCompressionRepr::Legacy, Short, Absent, true) -> None);
        check!((PanIdCompressionRepr::Legacy, Extended, Absent, true) -> None);
        check!((PanIdCompressionRepr::Legacy, Absent, Absent, false) -> None);
        check!((PanIdCompressionRepr::Legacy, Absent, Absent, true) -> None);

        check!((PanIdCompressionRepr::No, Short, Short, false) -> Some((true, Short, true, Short)));
        check!((PanIdCompressionRepr::Yes, Short, Short, true) -> Some((true, Short, false, Short)));
        check!((PanIdCompressionRepr::No, Extended, Extended, false) -> Some((true, Extended, false, Extended)));
        check!((PanIdCompressionRepr::Yes, Extended, Extended, true) -> Some((false, Extended, false, Extended)));
        check!((PanIdCompressionRepr::No, Short, Extended, false) -> Some((true, Short, true, Extended)));
        check!((PanIdCompressionRepr::Yes, Short, Extended, true) -> Some((true, Short, false, Extended)));
        check!((PanIdCompressionRepr::No, Extended, Short, false) -> Some((true, Extended, true, Short)));
        check!((PanIdCompressionRepr::Yes, Extended, Short, true) -> Some((true, Extended, false, Short)));
        check!((PanIdCompressionRepr::No, Absent, Short, false) -> Some((false, Absent, true, Short)));
        check!((PanIdCompressionRepr::No, Absent, Extended, false) -> Some((false, Absent, true, Extended)));
        check!((PanIdCompressionRepr::No, Short, Absent, false) -> Some((true, Short, false, Absent)));
        check!((PanIdCompressionRepr::No, Extended, Absent, false) -> Some((true, Extended, false, Absent)));
        check!((PanIdCompressionRepr::Yes, Absent, Short, true) -> Some((false, Absent, false, Short)));
        check!((PanIdCompressionRepr::Yes, Absent, Extended, true) -> Some((false, Absent, false, Extended)));
        check!((PanIdCompressionRepr::Yes, Short, Absent, true) -> Some((false, Short, false, Absent)));
        check!((PanIdCompressionRepr::Yes, Extended, Absent, true) -> Some((false, Extended, false, Absent)));
        check!((PanIdCompressionRepr::No, Absent, Absent, false) -> Some((false, Absent, false, Absent)));
        check!((PanIdCompressionRepr::Yes, Absent, Absent, true) -> Some((true, Absent, false, Absent)));
    }

    #[test]
    fn parse() {
        let mut addresses: Vec<(&'static str, Address<&[u8]>)> = vec![
            ("", Address::Absent),
            ("ff:ff", Address::Short(SOME_SHORT_ADDRESS)),
            ("ff:fe", Address::Short(OTHER_SHORT_ADDRESS)),
            (
                "ff:ff:ff:ff:ff:ff:ff:ff",
                Address::Extended(SOME_EXTENDED_ADDRESS),
            ),
            (
                "01:01:01:01:01:01:01:01",
                Address::Extended(OTHER_EXTENDED_ADDRESS),
            ),
            (
                "00:00:00:00:00:00:00:00",
                Address::Extended(ExtendedAddress::new(&[0x00; 8])),
            ),
            (
                "00:00:00:00:00:00:00:01",
                Address::Extended(ExtendedAddress::new(&[
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
                ])),
            ),
        ];

        for (s, expected) in addresses.drain(..) {
            assert_eq!(Address::parse(s).unwrap(), expected.into());
        }
    }
}
