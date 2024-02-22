use crate::time::Duration;
use bitflags::bitflags;

/// A reader/writer for the IEEE 802.15.4 Nested Information Elements.
///
/// ## Short format
/// ```notrust
/// +--------+--------+--------+--------------------------+
/// | Length | Sub-ID | Type=0 | Content (0-255 octets)...|
/// +--------+--------+--------+--------------------------+
/// ```
///
/// ## Long format
/// ```notrust
/// +--------+--------+--------+---------------------------+
/// | Length | Sub-ID | Type=1 | Content (0-2046 octets)...|
/// +--------+--------+--------+---------------------------+
/// ```
///
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct NestedInformationElement<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> NestedInformationElement<T> {
    /// Return the length of the Nested Information Element in bytes.
    pub fn length(&self) -> usize {
        let b = &self.data.as_ref()[0..2];
        if self.is_long() {
            (u16::from_le_bytes([b[0], b[1]]) & 0b1111111111) as usize
        } else {
            (u16::from_le_bytes([b[0], b[1]]) & 0b1111111) as usize
        }
    }

    /// Return the [`NestedSubId`].
    pub fn sub_id(&self) -> NestedSubId {
        let b = &self.data.as_ref()[0..2];
        let id = u16::from_le_bytes([b[0], b[1]]);
        if self.is_long() {
            NestedSubId::Long(NestedSubIdLong::from(((id >> 11) & 0b1111) as u8))
        } else {
            NestedSubId::Short(NestedSubIdShort::from(((id >> 8) & 0b111111) as u8))
        }
    }

    /// Returns `true` when the Nested Information Element is a short type.
    pub fn is_short(&self) -> bool {
        !self.is_long()
    }

    /// Returns `true` when the Nested Information Element is a long type.
    pub fn is_long(&self) -> bool {
        let b = &self.data.as_ref()[0..2];
        (u16::from_le_bytes([b[0], b[1]]) >> 15) & 0b1 == 0b1
    }

    /// Return the content of this Nested Information Element.
    pub fn content(&self) -> &[u8] {
        &self.data.as_ref()[2..][..self.length()]
    }
}

#[cfg(feature = "std")]
impl<T: AsRef<[u8]>> core::fmt::Display for NestedInformationElement<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.sub_id() {
            NestedSubId::Short(id) => match id {
                NestedSubIdShort::TschSynchronization => {
                    write!(f, "  {id} {}", TschSynchronization::new(self.content()))
                }
                NestedSubIdShort::TschTimeslot => {
                    write!(f, "  {id} {}", TschTimeslot::new(self.content()))
                }
                NestedSubIdShort::TschSlotframeAndLink => {
                    write!(f, "  {id} {}", TschSlotframeAndLink::new(self.content()))
                }
                _ => write!(f, "  {:?}({:0x?})", id, self.content()),
            },
            NestedSubId::Long(id) => match id {
                NestedSubIdLong::ChannelHopping => {
                    write!(f, "  {id} {}", ChannelHopping::new(self.content()))
                }
                id => write!(f, "  {:?}({:0x?})", id, self.content()),
            },
        }
    }
}

/// Nested Information Element ID.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum NestedSubId {
    /// Short Nested Information Element ID.
    Short(NestedSubIdShort),
    /// Long Nested Information Element ID.
    Long(NestedSubIdLong),
}

impl NestedSubId {
    /// Create a short [`NestedSubId`] from a `u8`.
    pub fn from_short(value: u8) -> Self {
        Self::Short(NestedSubIdShort::from(value))
    }

    /// Create a long [`NestedSubId`] from a `u8`.
    pub fn from_long(value: u8) -> Self {
        Self::Long(NestedSubIdLong::from(value))
    }
}

/// Short Nested Information Element ID.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum NestedSubIdShort {
    TschSynchronization = 0x1a,
    TschSlotframeAndLink = 0x1b,
    TschTimeslot = 0x1c,
    HoppingTiming = 0x1d,
    EnhancedBeaconFilter = 0x1e,
    MacMetrics = 0x1f,
    AllMacMetrics = 0x20,
    CoexistenceSpecification = 0x21,
    SunDeviceCapabilities = 0x22,
    SunFskGenericPhy = 0x23,
    ModeSwitchParameter = 0x24,
    PhyParameterChange = 0x25,
    OQpskPhyMode = 0x26,
    PcaAllocation = 0x27,
    LecimDsssOperatingMode = 0x28,
    LecimFskOperatingMode = 0x29,
    TvwsPhyOperatingMode = 0x2b,
    TvwsDeviceCapabilities = 0x2c,
    TvwsDeviceCategory = 0x2d,
    TvwsDeviceIdentification = 0x2e,
    TvwsDeviceLocation = 0x2f,
    TvwsChannelInformationQuery = 0x30,
    TvwsChannelInformationSource = 0x31,
    Ctm = 0x32,
    Timestamp = 0x33,
    TimestampDifference = 0x34,
    TmctpSpecification = 0x35,
    RccPhyOperatingMode = 0x36,
    LinkMargin = 0x37,
    RsGfskDeviceCapabilities = 0x38,
    MultiPhy = 0x39,
    VendorSpecific = 0x40,
    Srm = 0x46,
    Unkown,
}

impl From<u8> for NestedSubIdShort {
    fn from(value: u8) -> Self {
        match value {
            0x1a => Self::TschSynchronization,
            0x1b => Self::TschSlotframeAndLink,
            0x1c => Self::TschTimeslot,
            0x1d => Self::HoppingTiming,
            0x1e => Self::EnhancedBeaconFilter,
            0x1f => Self::MacMetrics,
            0x20 => Self::AllMacMetrics,
            0x21 => Self::CoexistenceSpecification,
            0x22 => Self::SunDeviceCapabilities,
            0x23 => Self::SunFskGenericPhy,
            0x24 => Self::ModeSwitchParameter,
            0x25 => Self::PhyParameterChange,
            0x26 => Self::OQpskPhyMode,
            0x27 => Self::PcaAllocation,
            0x28 => Self::LecimDsssOperatingMode,
            0x29 => Self::LecimFskOperatingMode,
            0x2b => Self::TvwsPhyOperatingMode,
            0x2c => Self::TvwsDeviceCapabilities,
            0x2d => Self::TvwsDeviceCategory,
            0x2e => Self::TvwsDeviceIdentification,
            0x2f => Self::TvwsDeviceLocation,
            0x30 => Self::TvwsChannelInformationQuery,
            0x31 => Self::TvwsChannelInformationSource,
            0x32 => Self::Ctm,
            0x33 => Self::Timestamp,
            0x34 => Self::TimestampDifference,
            0x35 => Self::TmctpSpecification,
            0x36 => Self::RccPhyOperatingMode,
            0x37 => Self::LinkMargin,
            0x38 => Self::RsGfskDeviceCapabilities,
            0x39 => Self::MultiPhy,
            0x40 => Self::VendorSpecific,
            0x46 => Self::Srm,
            _ => Self::Unkown,
        }
    }
}

impl core::fmt::Display for NestedSubIdShort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TschTimeslot => write!(f, "TSCH Timeslot"),
            Self::TschSlotframeAndLink => write!(f, "TSCH Slotframe and Link"),
            Self::TschSynchronization => write!(f, "TSCH Synchronization"),
            _ => write!(f, "{:?}", self),
        }
    }
}

/// Long Nested Information Element ID.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum NestedSubIdLong {
    VendorSpecificNested = 0x08,
    ChannelHopping = 0x09,
    Unkown,
}

impl From<u8> for NestedSubIdLong {
    fn from(value: u8) -> Self {
        match value {
            0x08 => Self::VendorSpecificNested,
            0x09 => Self::ChannelHopping,
            _ => Self::Unkown,
        }
    }
}

impl core::fmt::Display for NestedSubIdLong {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelHopping => write!(f, "Channel Hopping"),
            _ => write!(f, "{:?}", self),
        }
    }
}

/// A reader/writer for the TSCH synchronization IE.
/// ```notrust
/// +-----+-------------+
/// | ASN | Join metric |
/// +-----+-------------+
/// 0     5             6
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TschSynchronization<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> TschSynchronization<T> {
    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the absolute slot number field.
    pub fn absolute_slot_number(&self) -> u64 {
        let data = self.data.as_ref();
        let mut asn = data[0] as u64;
        asn += (data[1] as u64) << 8;
        asn += (data[2] as u64) << 16;
        asn += (data[3] as u64) << 24;
        asn += (data[4] as u64) << 32;
        asn
    }

    /// Return the join metric field.
    pub fn join_metric(&self) -> u8 {
        self.data.as_ref()[5]
    }
}

impl<T: AsRef<[u8]>> core::fmt::Display for TschSynchronization<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ASN: {}, join metric: {}",
            self.absolute_slot_number(),
            self.join_metric()
        )
    }
}

/// A reader/writer for the TSCH timeslot IE.
/// ```notrust
/// +----+--------------------------+
/// | ID | TSCH timeslot timings... |
/// +----+--------------------------+
/// 0    1
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TschTimeslot<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> TschTimeslot<T> {
    pub const DEFAULT_ID: u8 = 0;

    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the TSCH timeslot ID field.
    pub fn id(&self) -> u8 {
        self.data.as_ref()[0]
    }

    /// Return the TSCH timeslot timings.
    pub fn timeslot_timings(&self) -> TschTimeslotTimings {
        if self.id() == Self::DEFAULT_ID {
            TschTimeslotTimings::default()
        } else {
            TschTimeslotTimings {
                id: self.id(),
                cca_offset: Duration::from_us({
                    let b = &self.data.as_ref()[1..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                cca: Duration::from_us({
                    let b = &self.data.as_ref()[3..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                tx_offset: Duration::from_us({
                    let b = &self.data.as_ref()[5..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                rx_offset: Duration::from_us({
                    let b = &self.data.as_ref()[7..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                rx_ack_delay: Duration::from_us({
                    let b = &self.data.as_ref()[9..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                tx_ack_delay: Duration::from_us({
                    let b = &self.data.as_ref()[11..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                rx_wait: Duration::from_us({
                    let b = &self.data.as_ref()[13..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                ack_wait: Duration::from_us({
                    let b = &self.data.as_ref()[15..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                rx_tx: Duration::from_us({
                    let b = &self.data.as_ref()[17..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                max_ack: Duration::from_us({
                    let b = &self.data.as_ref()[19..][..2];
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                max_tx: Duration::from_us({
                    let len = if self.data.as_ref().len() == 25 { 2 } else { 3 };
                    let b = &self.data.as_ref()[21..][..len];
                    // TODO: handle the case where a 3 byte length is used.
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
                time_slot_length: Duration::from_us({
                    let offset = if self.data.as_ref().len() == 25 {
                        23
                    } else {
                        24
                    };
                    let len = if self.data.as_ref().len() == 25 { 2 } else { 3 };
                    let b = &self.data.as_ref()[offset..][..len];
                    // TODO: handle the case where a 3 byte length is used.
                    u16::from_le_bytes([b[0], b[1]]) as i64
                }),
            }
        }
    }
}

impl<T: AsRef<[u8]>> core::fmt::Display for TschTimeslot<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "slot ID: {}", self.id())
    }
}

/// A TSCH time slot timings (figure 6-30 in IEEE 802.15.4-2020).
///
/// If the time slot ID is 0, the default timings are used.
///
/// ```notrust
/// +----+------------+-----+-----------+-----------+--------------+--------------+---------+----------+-------+---------+--------+------------------+
/// | ID | CCA offset | CCA | TX offset | RX offset | RX ACK delay | TX ACK delay | RX wait | ACK wait | RX/TX | Max ACK | Max TX | Time slot length |
/// +----+------------+-----+-----------+-----------+--------------+--------------+---------+----------+-------+---------+--------+------------------+
/// ```
#[derive(Debug)]
pub struct TschTimeslotTimings {
    id: u8,
    /// Offset from the start of the time slot to the start of the CCA in microseconds.
    cca_offset: Duration,
    /// Duration of the CCA in microseconds.
    cca: Duration,
    /// Radio turnaround time in microseconds.
    rx_tx: Duration,

    /// Offset from the start of the time slot to the start of the TX in microseconds.
    tx_offset: Duration,
    /// Maximum transmission time for a frame in microseconds.
    max_tx: Duration,
    /// Wait time between the end of the TX and the start of the ACK RX in microseconds.
    rx_ack_delay: Duration,
    /// Maximum time to wait for receiving an ACK.
    ack_wait: Duration,

    /// Offset from the start of the time slot to the start of the RX in microseconds.
    rx_offset: Duration,
    /// Maximum time to wait for receiving a frame.
    rx_wait: Duration,
    /// Wait time between the end of the RX and the start of the ACK TX in microseconds.
    tx_ack_delay: Duration,
    /// Maximum transmission time for an ACK in microseconds.
    max_ack: Duration,

    /// Length of the time slot in microseconds.
    time_slot_length: Duration,
}

impl Default for TschTimeslotTimings {
    fn default() -> Self {
        Self::new(0, Self::DEFAULT_GUARD_TIME)
    }
}

impl TschTimeslotTimings {
    /// The default guard time (2200us) in microseconds.
    pub const DEFAULT_GUARD_TIME: Duration = Duration::from_us(2200);

    /// Create a new set of time slot timings.
    pub fn new(id: u8, guard_time: Duration) -> Self {
        Self {
            id,
            cca_offset: Duration::from_us(1800),
            cca: Duration::from_us(128),
            tx_offset: Duration::from_us(2120),
            rx_offset: Duration::from_us(2120) - (guard_time / 2),
            rx_ack_delay: Duration::from_us(800),
            tx_ack_delay: Duration::from_us(1000),
            rx_wait: guard_time,
            ack_wait: Duration::from_us(400),
            rx_tx: Duration::from_us(192),
            max_ack: Duration::from_us(2400),
            max_tx: Duration::from_us(4256),
            time_slot_length: Duration::from_us(10000),
        }
    }

    /// Return the CCA offset in microseconds.
    pub const fn cca_offset(&self) -> Duration {
        self.cca_offset
    }

    /// Set the CCA offset in microseconds.
    pub fn set_cca_offset(&mut self, cca_offset: Duration) {
        self.cca_offset = cca_offset;
    }

    /// Return the CCA duration in microseconds.
    pub const fn cca(&self) -> Duration {
        self.cca
    }

    /// Set the CCA duration in microseconds.
    pub fn set_cca(&mut self, cca: Duration) {
        self.cca = cca;
    }

    /// Return the TX offset in microseconds.
    pub const fn tx_offset(&self) -> Duration {
        self.tx_offset
    }

    /// Set the TX offset in microseconds.
    pub fn set_tx_offset(&mut self, tx_offset: Duration) {
        self.tx_offset = tx_offset;
    }

    /// Return the RX offset in microseconds.
    pub const fn rx_offset(&self) -> Duration {
        self.rx_offset
    }

    /// Set the RX offset in microseconds.
    pub fn set_rx_offset(&mut self, rx_offset: Duration) {
        self.rx_offset = rx_offset;
    }

    /// Return the RX ACK delay in microseconds.
    pub const fn rx_ack_delay(&self) -> Duration {
        self.rx_ack_delay
    }

    /// Set the RX ACK delay in microseconds.
    pub fn set_rx_ack_delay(&mut self, rx_ack_delay: Duration) {
        self.rx_ack_delay = rx_ack_delay;
    }

    /// Return the TX ACK delay in microseconds.
    pub const fn tx_ack_delay(&self) -> Duration {
        self.tx_ack_delay
    }

    /// Set the TX ACK delay in microseconds.
    pub fn set_tx_ack_delay(&mut self, tx_ack_delay: Duration) {
        self.tx_ack_delay = tx_ack_delay;
    }

    /// Return the RX wait in microseconds.
    pub const fn rx_wait(&self) -> Duration {
        self.rx_wait
    }

    /// Set the RX wait in microseconds.
    pub fn set_rx_wait(&mut self, rx_wait: Duration) {
        self.rx_wait = rx_wait;
    }

    /// Return the ACK wait in microseconds.
    pub const fn ack_wait(&self) -> Duration {
        self.ack_wait
    }

    /// Set the ACK wait in microseconds.
    pub fn set_ack_wait(&mut self, ack_wait: Duration) {
        self.ack_wait = ack_wait;
    }

    /// Return the RX/TX in microseconds.
    pub const fn rx_tx(&self) -> Duration {
        self.rx_tx
    }

    /// Set the RX/TX in microseconds.
    pub fn set_rx_tx(&mut self, rx_tx: Duration) {
        self.rx_tx = rx_tx;
    }

    /// Return the maximum ACK in microseconds.
    pub const fn max_ack(&self) -> Duration {
        self.max_ack
    }

    /// Set the maximum ACK in microseconds.
    pub fn set_max_ack(&mut self, max_ack: Duration) {
        self.max_ack = max_ack;
    }

    /// Return the maximum TX in microseconds.
    pub const fn max_tx(&self) -> Duration {
        self.max_tx
    }

    /// Set the maximum TX in microseconds.
    pub fn set_max_tx(&mut self, max_tx: Duration) {
        self.max_tx = max_tx;
    }

    /// Return the time slot length in microseconds.
    pub const fn time_slot_length(&self) -> Duration {
        self.time_slot_length
    }

    /// Set the time slot length in microseconds.
    pub fn set_time_slot_length(&mut self, time_slot_length: Duration) {
        self.time_slot_length = time_slot_length;
    }

    /// Emit the time slot timings into a buffer.
    pub fn emit(&self, buffer: &mut [u8]) {
        buffer[0] = self.id;
        buffer[1..][..2].copy_from_slice(&(self.cca_offset.as_us() as u16).to_le_bytes());
        buffer[3..][..2].copy_from_slice(&(self.cca.as_us() as u16).to_le_bytes());
        buffer[5..][..2].copy_from_slice(&(self.tx_offset.as_us() as u16).to_le_bytes());
        buffer[7..][..2].copy_from_slice(&(self.rx_offset.as_us() as u16).to_le_bytes());
        buffer[9..][..2].copy_from_slice(&(self.rx_ack_delay.as_us() as u16).to_le_bytes());
        buffer[11..][..2].copy_from_slice(&(self.tx_ack_delay.as_us() as u16).to_le_bytes());
        buffer[13..][..2].copy_from_slice(&(self.rx_wait.as_us() as u16).to_le_bytes());
        buffer[15..][..2].copy_from_slice(&(self.ack_wait.as_us() as u16).to_le_bytes());
        buffer[17..][..2].copy_from_slice(&(self.rx_tx.as_us() as u16).to_le_bytes());
        buffer[19..][..2].copy_from_slice(&(self.max_ack.as_us() as u16).to_le_bytes());

        // TODO: handle the case where the buffer is too small
        buffer[21..][..2].copy_from_slice(&(self.max_tx.as_us() as u16).to_le_bytes());
        // TODO: handle the case where the buffer is too small
        buffer[23..][..2].copy_from_slice(&(self.time_slot_length.as_us() as u16).to_le_bytes());
    }

    pub fn fmt(&self, indent: usize, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let indent = " ".repeat(indent);
        writeln!(f, "{indent}cca_offset: {}", self.cca_offset())?;
        writeln!(f, "{indent}cca: {}", self.cca())?;
        writeln!(f, "{indent}tx offset: {}", self.tx_offset())?;
        writeln!(f, "{indent}rx offset: {}", self.rx_offset())?;
        writeln!(f, "{indent}tx ack delay: {}", self.tx_ack_delay())?;
        writeln!(f, "{indent}rx ack delay: {}", self.rx_ack_delay())?;
        writeln!(f, "{indent}rx wait: {}", self.rx_wait())?;
        writeln!(f, "{indent}ack wait: {}", self.ack_wait())?;
        writeln!(f, "{indent}rx/tx: {}", self.rx_tx())?;
        writeln!(f, "{indent}max ack: {}", self.max_ack())?;
        writeln!(f, "{indent}max tx: {}", self.max_tx())?;
        writeln!(f, "{indent}time slot length: {}", self.time_slot_length())
    }
}

impl core::fmt::Display for TschTimeslotTimings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.fmt(0, f)
    }
}

/// A reader/writer for the TSCH slotframe and link IE.
/// ```notrust
/// +----------------------+--------------------------+
/// | Number of slotframes | Slotframe descriptors... |
/// +----------------------+--------------------------+
/// 0                      1
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TschSlotframeAndLink<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> TschSlotframeAndLink<T> {
    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the number of slotframes field.
    pub fn number_of_slot_frames(&self) -> u8 {
        self.data.as_ref()[0]
    }

    /// Returns an [`Iterator`] over the [`SlotframeDescriptor`]s.
    pub fn slotframe_descriptors(&self) -> SlotframeDescriptorIterator {
        SlotframeDescriptorIterator::new(
            self.number_of_slot_frames() as usize,
            &self.data.as_ref()[1..],
        )
    }
}

impl<T: AsRef<[u8]>> core::fmt::Display for TschSlotframeAndLink<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#slot frames: {}", self.number_of_slot_frames())
    }
}

/// A reader/writer for the Slotframe Descriptor.
/// ```notrust
/// +--------+------+-------+---------------------+
/// | Handle | Size | Links | Link descriptors... |
/// +--------+------+-------+---------------------+
/// 0        1      3       4
/// ```
pub struct SlotframeDescriptor<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> SlotframeDescriptor<T> {
    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the length of the Slotframe Descriptor in bytes.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        4 + (self.links() as usize) * 5
    }

    /// Return the handle field.
    pub fn handle(&self) -> u8 {
        self.data.as_ref()[0]
    }

    /// Return the size field.
    pub fn size(&self) -> u16 {
        let b = &self.data.as_ref()[1..][..2];
        u16::from_le_bytes([b[0], b[1]])
    }

    /// Return the links field.
    pub fn links(&self) -> u8 {
        self.data.as_ref()[3]
    }

    /// Return the link descriptors.
    pub fn link_descriptors(&self) -> LinkDescriptorIterator {
        LinkDescriptorIterator::new(
            &self.data.as_ref()[4..][..(self.links() as usize * LinkDescriptor::<&[u8]>::len())],
        )
    }
}

/// An [`Iterator`] over [`SlotframeDescriptor`].
pub struct SlotframeDescriptorIterator<'f> {
    data: &'f [u8],
    offset: usize,
    terminated: bool,
    slotframes: usize,
    slotframe_count: usize,
}

impl<'f> SlotframeDescriptorIterator<'f> {
    pub fn new(slotframes: usize, data: &'f [u8]) -> Self {
        let terminated = slotframes == 0;

        Self {
            data,
            offset: 0,
            terminated,
            slotframes,
            slotframe_count: 0,
        }
    }
}

impl<'f> Iterator for SlotframeDescriptorIterator<'f> {
    type Item = SlotframeDescriptor<&'f [u8]>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.terminated {
            None
        } else {
            let descriptor = SlotframeDescriptor::new(&self.data[self.offset..]);
            self.slotframe_count += 1;

            self.offset += descriptor.len();

            if self.offset >= self.data.as_ref().len() || self.slotframe_count >= self.slotframes {
                self.terminated = true;
            }

            Some(descriptor)
        }
    }
}

/// A reader/writer for the Link Descriptor.
/// ```notrust
/// +----------+----------------+--------------+
/// | Timeslot | Channel offset | Link options |
/// +----------+----------------+--------------+
/// 0          2                4
/// ```
pub struct LinkDescriptor<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> LinkDescriptor<T> {
    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the length of the Link Descriptor in bytes.
    pub const fn len() -> usize {
        5
    }

    /// Return the timeslot field.
    pub fn timeslot(&self) -> u16 {
        let b = &self.data.as_ref()[0..][..2];
        u16::from_le_bytes([b[0], b[1]])
    }

    /// Return the channel offset field.
    pub fn channel_offset(&self) -> u16 {
        let b = &self.data.as_ref()[2..][..2];
        u16::from_le_bytes([b[0], b[1]])
    }

    /// Return the link options field.
    pub fn link_options(&self) -> TschLinkOption {
        TschLinkOption::from_bits_truncate(self.data.as_ref()[4])
    }
}

/// An [`Iterator`] over [`LinkDescriptor`].
pub struct LinkDescriptorIterator<'f> {
    data: &'f [u8],
    offset: usize,
    terminated: bool,
}

impl<'f> LinkDescriptorIterator<'f> {
    pub fn new(data: &'f [u8]) -> Self {
        Self {
            data,
            offset: 0,
            terminated: false,
        }
    }
}

impl<'f> Iterator for LinkDescriptorIterator<'f> {
    type Item = LinkDescriptor<&'f [u8]>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.terminated {
            return None;
        }

        let descriptor = LinkDescriptor::new(&self.data[self.offset..]);

        self.offset += LinkDescriptor::<&[u8]>::len();
        self.terminated = self.offset >= self.data.as_ref().len();

        Some(descriptor)
    }
}

bitflags! {
    /// TSCH link options bitfield.
    /// ```notrust
    /// +----+----+--------+--------------+----------+----------+
    /// | Tx | Rx | Shared | Time keeping | Priority | Reserved |
    /// +----+----+--------+--------------+----------+----------+
    /// ```
    #[derive(Copy, Clone)]
    pub struct TschLinkOption: u8 {
        const Tx = 0b0000_0001;
        const Rx = 0b0000_0010;
        const Shared = 0b0000_0100;
        const TimeKeeping = 0b0000_1000;
        const Priority = 0b0001_0000;
    }
}

impl core::fmt::Debug for TschLinkOption {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        bitflags::parser::to_writer(self, f)
    }
}

/// A reader/writer for the Channel Hopping IE.
/// ```notrust
/// +-------------+-----+
/// | Sequence ID | ... |
/// +-------------+-----+
/// 0             1
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelHopping<T: AsRef<[u8]>> {
    data: T,
}

impl<T: AsRef<[u8]>> ChannelHopping<T> {
    pub fn new(data: T) -> Self {
        Self { data }
    }

    /// Return the hopping sequence ID field.
    pub fn hopping_sequence_id(&self) -> u8 {
        self.data.as_ref()[0]
    }
}

impl<T: AsRef<[u8]>> core::fmt::Display for ChannelHopping<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sequence ID: {}", self.hopping_sequence_id())
    }
}

/// An [`Iterator`] over [`NestedInformationElement`].
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct NestedInformationElementsIterator<'f> {
    data: &'f [u8],
    offset: usize,
    terminated: bool,
}

impl<'f> NestedInformationElementsIterator<'f> {
    pub fn new(data: &'f [u8]) -> Self {
        Self {
            data,
            offset: 0,
            terminated: false,
        }
    }
}

impl<'f> Iterator for NestedInformationElementsIterator<'f> {
    type Item = NestedInformationElement<&'f [u8]>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.terminated {
            None
        } else {
            let nested_len = NestedInformationElement {
                data: &self.data[self.offset..],
            }
            .length()
                + 2;

            let nested = NestedInformationElement {
                data: &self.data[self.offset..][..nested_len],
            };

            self.offset += nested_len;

            if self.offset >= self.data.len() {
                self.terminated = true;
            }

            Some(nested)
        }
    }
}

/// A high-level representation of a MLME Payload Information Element.
#[derive(Debug)]
pub enum NestedInformationElementRepr {
    TschSynchronization(TschSynchronizationRepr),
    TschTimeslot(TschTimeslotRepr),
    TschSlotframeAndLink(TschSlotframeAndLinkRepr),
}

impl NestedInformationElementRepr {
    pub fn parse(ie: NestedInformationElement<&[u8]>) -> Self {
        match ie.sub_id() {
            NestedSubId::Short(NestedSubIdShort::TschSynchronization) => {
                Self::TschSynchronization(TschSynchronizationRepr {
                    absolute_slot_number: TschSynchronization::new(ie.content())
                        .absolute_slot_number(),
                    join_metric: TschSynchronization::new(ie.content()).join_metric(),
                })
            }
            NestedSubId::Short(NestedSubIdShort::TschTimeslot) => {
                Self::TschTimeslot(TschTimeslotRepr {
                    id: TschTimeslot::new(ie.content()).id(),
                })
            }
            NestedSubId::Short(NestedSubIdShort::TschSlotframeAndLink) => {
                Self::TschSlotframeAndLink(TschSlotframeAndLinkRepr {
                    number_of_slot_frames: TschSlotframeAndLink::new(ie.content())
                        .number_of_slot_frames(),
                })
            }
            NestedSubId::Long(NestedSubIdLong::ChannelHopping) => {
                Self::TschSlotframeAndLink(TschSlotframeAndLinkRepr {
                    number_of_slot_frames: TschSlotframeAndLink::new(ie.content())
                        .number_of_slot_frames(),
                })
            }
            _ => todo!(),
        }
    }
}

/// A high-level representation of a TSCH Synchronization Nested Information Element.
#[derive(Debug)]
pub struct TschSynchronizationRepr {
    /// The absolute slot number (ASN).
    pub absolute_slot_number: u64,
    /// The join metric.
    pub join_metric: u8,
}

/// A high-level representation of a TSCH Timeslot Nested Information Element.
#[derive(Debug)]
pub struct TschTimeslotRepr {
    /// The timeslot ID.
    pub id: u8,
}

/// A high-level representation of a TSCH Slotframe and Link Nested Information Element.
#[derive(Debug)]
pub struct TschSlotframeAndLinkRepr {
    /// The number of slotframes.
    pub number_of_slot_frames: u8,
}
