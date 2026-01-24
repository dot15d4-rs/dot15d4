//! TSCH PIB (PAN Information Base) attributes.
//!
//! This module contains TSCH-specific PIB attributes as defined in
//! IEEE 802.15.4-2024 Section 10.3.11.

#![allow(dead_code)]

use dot15d4_driver::radio::config::Channel;
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
    pub slotframe_handle: u16,
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
    pub handle: u16,
    /// The number of timeslots in a given slotframe.
    pub size: u16,
}

impl TschSlotframe {
    /// Create a new slotframe.
    pub fn new(handle: u16, size: u16) -> Self {
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
    pub fn handle(&self) -> u16 {
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
    pub fn create_slotframe(&mut self, handle: u16, size: u16) -> Result<u16, ScheduleError> {
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
    pub fn get_slotframe(&self, handle: u16) -> Option<&TschSlotframe> {
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

    /// Find the next advertisement link after the given ASN.
    pub fn next_advertisement_link(&self, _current_asn: TschAsn) -> Option<&TschLink<Neighbor>> {
        // For now, return the first advertising link
        self.links
            .iter()
            .find(|l| matches!(l.link_type, TschLinkType::Advertising))
            .or_else(|| self.links.first())
    }

    /// Find links matching the given options.
    pub fn find_links_with_options(
        &self,
        options: TschLinkOption,
    ) -> impl Iterator<Item = &TschLink<Neighbor>> {
        self.links
            .iter()
            .filter(move |l| l.link_options.contains(options))
    }

    /// Calculate the next ASN for a given link.
    pub fn next_asn_for_link(
        &self,
        link: &TschLink<Neighbor>,
        current_asn: TschAsn,
    ) -> Option<TschAsn> {
        self.get_slotframe(link.slotframe_handle)
            .map(|sf| sf.next_asn_for_link(link, current_asn))
    }

    /// Calculate the next ASN for a link, strictly after current_asn.
    /// TODO: remove, redundant
    pub fn next_asn_for_link_strict(
        &self,
        link: &TschLink<Neighbor>,
        current_asn: TschAsn,
    ) -> Option<TschAsn> {
        self.get_slotframe(link.slotframe_handle)
            .map(|sf| sf.next_asn_for_link(link, current_asn.saturating_add(1)))
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
    pub fn slotframe_info(&self) -> impl Iterator<Item = (u16, u16)> + '_ {
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
        }
    }
}
