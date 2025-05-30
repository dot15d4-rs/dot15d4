//! Currently this module only provides the minimal structures required to make
//! a basic dot15d4 MVP run.
//!
//! In the future it will contain field read/write accessors for all
//! TSCH-related IEs.

use bitflags::bitflags;

use dot15d4_driver::time::{Duration, Microseconds};

/// TSCH timeslot timings (figure 6-30 in IEEE 802.15.4-2020).
///
/// If the timeslot ID is 0, the default timings are used.
///
/// ```notrust
/// +----+------------+-----+-----------+-----------+--------------+--------------+---------+----------+-------+---------+--------+------------------+
/// | ID | CCA offset | CCA | TX offset | RX offset | RX ACK delay | TX ACK delay | RX wait | ACK wait | RX/TX | Max ACK | Max TX | Timeslot length |
/// +----+------------+-----+-----------+-----------+--------------+--------------+---------+----------+-------+---------+--------+------------------+
/// ```
#[derive(Debug)]
pub struct TschTimeslotTimings {
    id: u8,
    /// Offset from the start of the timeslot to the start of the CCA in
    /// microseconds.
    cca_offset: Duration<Microseconds>,
    /// Duration of the CCA in microseconds.
    cca: Duration<Microseconds>,
    /// Radio turnaround time in microseconds.
    rx_tx: Duration<Microseconds>,

    /// Offset from the start of the timeslot to the start of the TX in
    /// microseconds.
    tx_offset: Duration<Microseconds>,
    /// Maximum transmission time for a frame in microseconds.
    max_tx: Duration<Microseconds>,
    /// Wait time between the end of the TX and the start of the ACK RX in
    /// microseconds.
    rx_ack_delay: Duration<Microseconds>,
    /// Maximum time to wait for receiving an ACK.
    ack_wait: Duration<Microseconds>,

    /// Offset from the start of the timeslot to the start of the RX in
    /// microseconds.
    rx_offset: Duration<Microseconds>,
    /// Maximum time to wait for receiving a frame.
    rx_wait: Duration<Microseconds>,
    /// Wait time between the end of the RX and the start of the ACK TX in
    /// microseconds.
    tx_ack_delay: Duration<Microseconds>,
    /// Maximum transmission time for an ACK in microseconds.
    max_ack: Duration<Microseconds>,

    /// Length of the timeslot in microseconds.
    timeslot_length: Duration<Microseconds>,
}

impl Default for TschTimeslotTimings {
    fn default() -> Self {
        Self::new(0, Self::DEFAULT_GUARD_TIME)
    }
}

impl TschTimeslotTimings {
    /// The default guard time (2200us) in microseconds.
    pub const DEFAULT_GUARD_TIME: Duration<Microseconds> = Duration::new(2200);

    /// Create a new set of timeslot timings.
    pub fn new(id: u8, guard_time: Duration<Microseconds>) -> Self {
        Self {
            id,
            cca_offset: Duration::new(1800),
            cca: Duration::new(128),
            tx_offset: Duration::new(2120),
            rx_offset: Duration::new(2120) - (guard_time / 2),
            rx_ack_delay: Duration::new(800),
            tx_ack_delay: Duration::new(1000),
            rx_wait: guard_time,
            ack_wait: Duration::new(400),
            rx_tx: Duration::new(192),
            max_ack: Duration::new(2400),
            max_tx: Duration::new(4256),
            timeslot_length: Duration::new(10000),
        }
    }

    /// Return the Timeslot timing ID.
    pub const fn id(&self) -> u8 {
        self.id
    }

    /// Return the CCA offset in microseconds.
    pub const fn cca_offset(&self) -> Duration<Microseconds> {
        self.cca_offset
    }

    /// Set the CCA offset in microseconds.
    pub fn set_cca_offset(&mut self, cca_offset: Duration<Microseconds>) {
        self.cca_offset = cca_offset;
    }

    /// Return the CCA duration in microseconds.
    pub const fn cca(&self) -> Duration<Microseconds> {
        self.cca
    }

    /// Set the CCA duration in microseconds.
    pub fn set_cca(&mut self, cca: Duration<Microseconds>) {
        self.cca = cca;
    }

    /// Return the TX offset in microseconds.
    pub const fn tx_offset(&self) -> Duration<Microseconds> {
        self.tx_offset
    }

    /// Set the TX offset in microseconds.
    pub fn set_tx_offset(&mut self, tx_offset: Duration<Microseconds>) {
        self.tx_offset = tx_offset;
    }

    /// Return the RX offset in microseconds.
    pub const fn rx_offset(&self) -> Duration<Microseconds> {
        self.rx_offset
    }

    /// Set the RX offset in microseconds.
    pub fn set_rx_offset(&mut self, rx_offset: Duration<Microseconds>) {
        self.rx_offset = rx_offset;
    }

    /// Return the RX ACK delay in microseconds.
    pub const fn rx_ack_delay(&self) -> Duration<Microseconds> {
        self.rx_ack_delay
    }

    /// Set the RX ACK delay in microseconds.
    pub fn set_rx_ack_delay(&mut self, rx_ack_delay: Duration<Microseconds>) {
        self.rx_ack_delay = rx_ack_delay;
    }

    /// Return the TX ACK delay in microseconds.
    pub const fn tx_ack_delay(&self) -> Duration<Microseconds> {
        self.tx_ack_delay
    }

    /// Set the TX ACK delay in microseconds.
    pub fn set_tx_ack_delay(&mut self, tx_ack_delay: Duration<Microseconds>) {
        self.tx_ack_delay = tx_ack_delay;
    }

    /// Return the RX wait in microseconds.
    pub const fn rx_wait(&self) -> Duration<Microseconds> {
        self.rx_wait
    }

    /// Set the RX wait in microseconds.
    pub fn set_rx_wait(&mut self, rx_wait: Duration<Microseconds>) {
        self.rx_wait = rx_wait;
    }

    /// Return the ACK wait in microseconds.
    pub const fn ack_wait(&self) -> Duration<Microseconds> {
        self.ack_wait
    }

    /// Set the ACK wait in microseconds.
    pub fn set_ack_wait(&mut self, ack_wait: Duration<Microseconds>) {
        self.ack_wait = ack_wait;
    }

    /// Return the RX/TX in microseconds.
    pub const fn rx_tx(&self) -> Duration<Microseconds> {
        self.rx_tx
    }

    /// Set the RX/TX in microseconds.
    pub fn set_rx_tx(&mut self, rx_tx: Duration<Microseconds>) {
        self.rx_tx = rx_tx;
    }

    /// Return the maximum ACK in microseconds.
    pub const fn max_ack(&self) -> Duration<Microseconds> {
        self.max_ack
    }

    /// Set the maximum ACK in microseconds.
    pub fn set_max_ack(&mut self, max_ack: Duration<Microseconds>) {
        self.max_ack = max_ack;
    }

    /// Return the maximum TX in microseconds.
    pub const fn max_tx(&self) -> Duration<Microseconds> {
        self.max_tx
    }

    /// Set the maximum TX in microseconds.
    pub fn set_max_tx(&mut self, max_tx: Duration<Microseconds>) {
        self.max_tx = max_tx;
    }

    /// Return the timeslot length in microseconds.
    pub const fn timeslot_length(&self) -> Duration<Microseconds> {
        self.timeslot_length
    }

    /// Set the timeslot length in microseconds.
    pub fn set_timeslot_length(&mut self, timeslot_length: Duration<Microseconds>) {
        self.timeslot_length = timeslot_length;
    }
}

impl core::fmt::Display for TschTimeslotTimings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let indent = f.width().unwrap_or(0);
        writeln!(f, "cca_offset: {}", self.cca_offset(),)?;
        writeln!(f, "{:indent$}cca: {}", "", self.cca())?;
        writeln!(f, "{:indent$}tx offset: {}", "", self.tx_offset(),)?;
        writeln!(f, "{:indent$}rx offset: {}", "", self.rx_offset(),)?;
        writeln!(f, "{:indent$}tx ack delay: {}", "", self.tx_ack_delay())?;
        writeln!(f, "{:indent$}rx ack delay: {}", "", self.rx_ack_delay(),)?;
        writeln!(f, "{:indent$}rx wait: {}", "", self.rx_wait(),)?;
        writeln!(f, "{:indent$}ack wait: {}", "", self.ack_wait(),)?;
        writeln!(f, "{:indent$}rx/tx: {}", "", self.rx_tx())?;
        writeln!(f, "{:indent$}max ack: {}", "", self.max_ack(),)?;
        writeln!(f, "{:indent$}max tx: {}", "", self.max_tx(),)?;
        writeln!(
            f,
            "{:indent$}timeslot length: {}",
            "",
            self.timeslot_length(),
        )
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
        /// Transmit.
        const Tx = 0b0000_0001;
        /// Receive.
        const Rx = 0b0000_0010;
        /// Shared.
        const Shared = 0b0000_0100;
        /// Time keeping.
        const TimeKeeping = 0b0000_1000;
        /// Priority.
        const Priority = 0b0001_0000;
    }
}

impl core::fmt::Debug for TschLinkOption {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        bitflags::parser::to_writer(self, f)
    }
}

impl core::fmt::Display for TschLinkOption {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        bitflags::parser::to_writer(self, f)
    }
}
