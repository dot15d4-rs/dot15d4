//! TSCH PIB (PAN Information Base) attributes.
//!
//! This module contains TSCH-specific PIB attributes as defined in
//! IEEE 802.15.4-2024 Section 10.3.11.

#![allow(dead_code)]

use dot15d4_driver::radio::config::Channel;
use dot15d4_driver::timer::{NsDuration, NsInstant};
use heapless::Vec;

use crate::constants::{
    MAC_DISCONNECT_TIME, MAC_JOIN_METRIC, MAC_MAX_BE, MAC_TSCH_MAX_LINKS, MAC_TSCH_MAX_SLOTFRAMES,
    MAC_TSCH_MIN_BE,
};
use crate::mac::frame::fields::{TschLinkOption, TschTimeslotTimings};

/// Type alias for Absolute Slot Number.
pub type TschAsn = u64;

/// Schedule operation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    InvalidSlotframe,
    InvalidTimeslot,
    InvalidChannelOffset,
    CapacityExceeded,
    HandleDuplicate,
}

/// Type of link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TschLinkType {
    /// Advertisement/beacon link
    Advertising,
    /// Normal data link
    Normal,
}

/// A TSCH link is a pairwise assignment of a directed communication between
/// devices for a given slotframe, in a given timeslot on a given channel offset.
///
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.3
#[derive(Debug, Clone)]
pub struct TschLink<Neighbor> {
    /// Slotframe identifier of the slotframe to which the link is associated.
    pub slotframe_handle: u8,
    /// Associated timeslot in the slotframe.
    pub timeslot: u16,
    /// Associated channel offset for the given timeslot for the link.
    pub channel_offset: u16,
    /// Link communication option (TX, RX, Shared, etc.).
    pub link_options: TschLinkOption,
    /// Type of link (normal or advertising).
    pub link_type: TschLinkType,
    /// Neighbor assigned to the link for communication. None if not a
    /// dedicated link.
    pub neighbor: Option<Neighbor>,
    /// Whether this link shall be advertised in Enhanced beacon frames
    /// using the TSCH Slotframe and Link IE.
    pub link_advertise: bool,
}

impl<Neighbor> Default for TschLink<Neighbor> {
    fn default() -> Self {
        Self {
            slotframe_handle: 0,
            timeslot: 0,
            channel_offset: 0,
            link_options: TschLinkOption::Shared,
            link_type: TschLinkType::Normal,
            neighbor: None,
            link_advertise: true,
        }
    }
}

/// A TSCH slotframe - a collection of timeslots repeating in time.
///
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.2
#[derive(Debug)]
pub struct TschSlotframe {
    /// Slotframe Identifier.
    pub handle: u8,
    /// The number of timeslots in a given slotframe.
    pub size: u16,
}

impl TschSlotframe {
    /// Create a new slotframe.
    pub fn new(handle: u8, size: u16) -> Self {
        Self { handle, size }
    }

    /// Return the timeslot within the slotframe for a given ASN.
    pub fn timeslot(&self, asn: TschAsn) -> u16 {
        (asn % self.size as u64) as u16
    }

    /// Calculate the next ASN when this link will be active.
    pub fn next_asn_for_link<Neighbor>(
        &self,
        link: &TschLink<Neighbor>,
        current_asn: TschAsn,
    ) -> TschAsn {
        let current_timeslot = self.timeslot(current_asn);
        if current_timeslot < link.timeslot {
            // Link is later in this slotframe cycle
            current_asn + (link.timeslot - current_timeslot) as u64
        } else {
            // Link is at current timeslot or earlier - go to next slotframe cycle
            current_asn + (self.size - current_timeslot + link.timeslot) as u64
        }
    }

    /// Calculate the next ASN for a link, strictly after current_asn.
    /// TODO: remove, redundant
    pub fn next_asn_for_link_after<Neighbor>(
        &self,
        link: &TschLink<Neighbor>,
        current_asn: TschAsn,
    ) -> TschAsn {
        // Start search from the next ASN to ensure we get a strictly future occurrence
        self.next_asn_for_link(link, current_asn.saturating_add(1).saturating_sub(1))
    }

    /// Get the slotframe size.
    pub fn size(&self) -> u16 {
        self.size
    }

    /// Get the slotframe handle.
    pub fn handle(&self) -> u8 {
        self.handle
    }
}

/// TSCH-specific PIB attributes.
///
/// IEEE802.15.4-2024 Section 10.3.11
pub struct TschPib<Neighbor> {
    /// The minimum value of the backoff exponent (BE) in the TSCH-CA algorithm.
    pub tsch_min_be: u8,
    /// The maximum value of the BE in the TSCH-CA algorithm.
    pub tsch_max_be: u8,
    /// Time (in timeslots) to send Disassociate frames before disconnecting.
    pub disconnect_time: u16,
    /// Metric used when selecting and joining a TSCH network.
    pub join_metric: u16,
    /// Slotframe table (macSlotframeTable).
    pub slotframes: Vec<TschSlotframe, MAC_TSCH_MAX_SLOTFRAMES>,
    /// Link table (macLinkTable).
    pub links: Vec<TschLink<Neighbor>, MAC_TSCH_MAX_LINKS>,
    /// Timeslot timing template (macTimeslotTemplate).
    pub timeslot_timings: TschTimeslotTimings,
    /// Current Absolute Slot Number.
    pub asn: TschAsn,
    /// Timestamp of last known timeslot start.
    pub last_base_time: NsInstant,
    /// RMARKER and ASN of last received frame
    pub last_rx: Option<(NsInstant, TschAsn)>,
    /// Estimated drift per timeslot in nanoseconds
    pub drift_ns: i32,
}

impl<Neighbor> TschPib<Neighbor> {
    /// Create a new TSCH PIB with default values.
    pub fn new() -> Self {
        Default::default()
    }

    /// Get the timeslot length in microseconds.
    pub fn timeslot_length_us(&self) -> u64 {
        self.timeslot_timings.timeslot_length() as u64
    }

    /// Create a slotframe and add it to the schedule.
    pub fn create_slotframe(&mut self, handle: u8, size: u16) -> Result<u16, ScheduleError> {
        // Check for duplicate handle
        if self.slotframes.iter().any(|sf| sf.handle == handle) {
            return Err(ScheduleError::HandleDuplicate);
        }

        if self
            .slotframes
            .push(TschSlotframe::new(handle, size))
            .is_err()
        {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.slotframes.len() as u16 - 1)
        }
    }

    /// Get a slotframe by its handle.
    pub fn get_slotframe(&self, handle: u8) -> Option<&TschSlotframe> {
        self.slotframes.iter().find(|sf| sf.handle == handle)
    }

    /// Get a slotframe by index.
    pub fn get_slotframe_by_index(&self, index: usize) -> Option<&TschSlotframe> {
        self.slotframes.get(index)
    }

    /// Add a link to the schedule.
    pub fn add_link(&mut self, link: TschLink<Neighbor>) -> Result<u16, ScheduleError>
    where
        Neighbor: Clone,
    {
        // Validate slotframe exists
        let slotframe = self
            .get_slotframe(link.slotframe_handle)
            .ok_or(ScheduleError::InvalidSlotframe)?;

        // Validate timeslot
        if link.timeslot >= slotframe.size {
            return Err(ScheduleError::InvalidTimeslot);
        }

        // TODO: Validate channel offset from parent PIB

        // Add link
        if self.links.push(link).is_err() {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.links.len() as u16 - 1)
        }
    }

    /// Sort links by (slotframe_handle, timeslot) for efficient iteration
    /// during link selection. Lower slotframe handles are processed first,
    /// which naturally respects IEEE 802.15.4-2024 priority rules.
    pub fn sort_links(&mut self) {
        let len = self.links.len();
        for i in 1..len {
            let mut j = i;
            while j > 0 {
                let should_swap = {
                    let a = &self.links[j - 1];
                    let b = &self.links[j];
                    (a.slotframe_handle, a.timeslot) > (b.slotframe_handle, b.timeslot)
                };
                if should_swap {
                    self.links.swap(j - 1, j);
                    j -= 1;
                } else {
                    break;
                }
            }
        }
    }

    /// Select the next valid link across all slotframes that matches the given
    /// criteria, returning the link with the smallest next ASN strictly after
    /// `current_asn`.
    ///
    /// When multiple links resolve to the same ASN (from different slotframes),
    /// the IEEE 802.15.4-2024 priority rules apply:
    /// - Lower macSlotframeHandle takes precedence over higher handle.
    ///
    /// TX vs RX precedence is handled at the task level since this method
    /// operates on a single filter (TX-only or RX-only).
    pub fn select_next_link(
        &self,
        current_asn: TschAsn,
        filter: impl Fn(&TschLink<Neighbor>) -> bool,
    ) -> Option<(&TschLink<Neighbor>, TschAsn)> {
        let mut best: Option<(&TschLink<Neighbor>, u64)> = None;

        for link in self.links.iter() {
            if !filter(link) {
                continue;
            }

            let slotframe = match self.get_slotframe(link.slotframe_handle) {
                Some(sf) => sf,
                None => continue,
            };

            let next_asn = slotframe.next_asn_for_link(link, current_asn);

            // Skip if the computed ASN is not strictly in the future.
            if next_asn <= current_asn {
                continue;
            }

            let is_better = match best {
                None => true,
                Some((best_link, best_asn)) => {
                    if next_asn < best_asn {
                        true
                    } else if next_asn == best_asn {
                        // Tie-break per IEEE 802.15.4-2024:
                        // lower slotframe handle takes precedence.
                        let best_handle = best_link.slotframe_handle;
                        link.slotframe_handle < best_handle
                    } else {
                        false
                    }
                }
            };

            if is_better {
                best = Some((link, next_asn));
            }
        }

        best
    }

    /// Calculate the channel for a given ASN and link.
    pub fn channel_for_link(
        &self,
        asn: TschAsn,
        link: &TschLink<Neighbor>,
        hopping_sequence: &[Channel],
    ) -> Channel {
        let channel_index =
            ((asn + link.channel_offset as u64) % hopping_sequence.len() as u64) as usize;
        hopping_sequence[channel_index]
    }

    /// Update base time and ASN after executing an operation.
    pub fn update_timing(&mut self, asn: TschAsn) {
        let timeslot_length_us = self.timeslot_length_us();
        self.last_base_time = self.expected_slot_start(asn, timeslot_length_us);
        self.asn = asn;
    }

    /// Calculate expected slot start time for given ASN.
    pub fn expected_slot_start(&self, asn: TschAsn, timeslot_length_us: u64) -> NsInstant {
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        let adjusted_slot_duration =
            NsDuration::nanos(((timeslot_length_us * 1000) as i32 - self.drift_ns) as u64);
        #[cfg(feature = "no-tsch-drift-adjustment")]
        let adjusted_slot_duration = NsDuration::micros(timeslot_length_us);
        let slots_diff = asn.saturating_sub(self.asn) as u32;
        self.last_base_time + slots_diff * adjusted_slot_duration
    }

    /// Calculate current ASN from timestamp.
    pub fn asn_at(&self, instant: NsInstant) -> TschAsn {
        let last_asn = self.asn;
        let timeslot_length_us = self.timeslot_length_us();
        if instant <= self.last_base_time {
            return last_asn;
        }
        let elapsed = instant - self.last_base_time;
        last_asn + elapsed.to_micros() / timeslot_length_us
    }

    /// Synchronize an ASN to an observed RMARKER.
    pub fn sync_asn(&mut self, asn: TschAsn, instant: NsInstant) {
        self.asn = asn;
        let tx_offset_us = self.timeslot_timings.tx_offset() as u64;
        let timeslot_start = instant - NsDuration::micros(tx_offset_us);
        self.last_base_time = timeslot_start;
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.update_drift(asn, timeslot_start);
        self.last_rx = Some((timeslot_start, asn));
    }

    /// Update clock drift relative to timesource with an instant derived from
    /// communication with timesource neighbor (i.e. when receiving a frame
    /// or using Time Sync information in Time Correction IE).
    #[cfg(not(feature = "no-tsch-drift-adjustment"))]
    fn update_drift(&mut self, asn: TschAsn, timeslot_start: NsInstant) {
        if let Some((last_slot_start, last_rx_asn)) = self.last_rx {
            let delta_asn = asn - last_rx_asn;

            if delta_asn == 0 {
                return;
            }

            let elapsed_time = timeslot_start - last_slot_start;
            let expected_elapsed_time = NsDuration::micros(delta_asn * self.timeslot_length_us());

            // Drift per slot in nanoseconds
            let measured_drift_ns = if elapsed_time > expected_elapsed_time {
                -(((elapsed_time - expected_elapsed_time).to_nanos() / delta_asn) as i32)
            } else {
                ((expected_elapsed_time - elapsed_time).to_nanos() / delta_asn) as i32
            };

            self.drift_ns = measured_drift_ns;
        }
    }

    /// Acknowledgment-based synchronization.
    #[cfg(not(feature = "no-tsch-ack-sync"))]
    pub fn sync_ack(&mut self, timesync_us: i16) {
        let asn = self.asn;
        if timesync_us < 0 {
            self.last_base_time += NsDuration::micros(timesync_us.unsigned_abs() as u64);
        } else {
            self.last_base_time -= NsDuration::micros(timesync_us.unsigned_abs() as u64);
        }
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.update_drift(asn, self.last_base_time);
        self.last_rx = Some((self.last_base_time, asn));
    }

    /// Get iterator over slotframes.
    pub fn slotframes(&self) -> impl Iterator<Item = &TschSlotframe> {
        self.slotframes.iter()
    }

    /// Get iterator over links.
    pub fn links(&self) -> impl Iterator<Item = &TschLink<Neighbor>> {
        self.links.iter()
    }

    /// Get slotframe info for beacon IE generation.
    /// Returns iterator of (handle, size) tuples.
    pub fn slotframe_info(&self) -> impl Iterator<Item = (u8, u16)> + '_ {
        self.slotframes.iter().map(|sf| (sf.handle, sf.size))
    }

    /// Get number of slotframes.
    pub fn num_slotframes(&self) -> usize {
        self.slotframes.len()
    }

    /// Get number of links.
    pub fn num_links(&self) -> usize {
        self.links.len()
    }
}

impl<Neighbor> Default for TschPib<Neighbor> {
    fn default() -> Self {
        Self {
            tsch_min_be: MAC_TSCH_MIN_BE,
            tsch_max_be: MAC_MAX_BE,
            disconnect_time: MAC_DISCONNECT_TIME,
            join_metric: MAC_JOIN_METRIC,
            slotframes: Vec::new(),
            links: Vec::new(),
            timeslot_timings: TschTimeslotTimings::default(),
            asn: 0,
            last_base_time: NsInstant::from_ticks(0),
            last_rx: None,
            drift_ns: 0,
        }
    }
}
