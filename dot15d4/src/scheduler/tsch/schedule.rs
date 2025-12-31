#![allow(dead_code)]
use dot15d4_driver::{radio::config::Channel, timer::NsInstant};
use heapless::Vec;

use crate::mac::{
    frame::fields::{TschLinkOption, TschTimeslotTimings},
    neighbors::MacNeighbor,
};

use super::asn::AbsoluteSlotNumber;

#[derive(Debug)]
pub enum ScheduleError {
    InvalidSlotframe,
    InvalidTimeslot,
    InvalidChannelOffset,
    CapacityExceeded,
    HandleDuplicate,
}

/// Type of link
pub enum TschLinkType {
    Advertising,
    Normal,
}

pub(crate) type TschAsn = u64;

/// A TSCH link is a pairwise assignment of a directed communication between
/// devices for a given slotframe, in a given timeslot on a given channel offset.
#[allow(dead_code)]
pub struct TschLink<Neighbor> {
    /// Slotframe identifier of the slotframe to which the link is associated.
    pub slotframe_handle: u16,
    /// Associated timeslot in the slotframe
    pub timeslot: u16,
    /// Associated Channel offset for the given timeslot for the link
    pub channel_offset: u16,
    /// Link communication option
    pub link_options: TschLinkOption,
    /// Type of link (normal or advertising)
    pub link_type: TschLinkType,
    /// Neighbor assigned to the link for communication. None if not a
    /// dedicated link
    pub neighbor: Option<Neighbor>,
    /// Wether this link shall be advertised in Enhanced beacon frames
    /// using the TSCH Slotframe and Link IE. If not, this link shall
    /// be added locally only.
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

/// Represents a channel hopping sequence
pub type TschHoppingSequence = [Channel; 2];

/// A TSCH slotframe collection of timeslots repeating in time, analogous to a
/// superframe in that it defines periods of communication opportunities.
#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct TschSlotframe {
    // /// Slotframe Identifier
    // handle: u16,
    /// The number of timeslots in a given slotframe, representing of often a
    /// timeslot repeats.
    size: u16,
}

#[allow(dead_code)]
impl TschSlotframe {
    /// Return the timeslot within the slotframe for a given ASN
    ///
    /// * `asn` - Absolute slot number
    fn timeslot(&self, asn: TschAsn) -> u16 {
        (asn % self.size as u64) as u16
    }

    fn next_asn<Neighbor>(&self, link: &TschLink<Neighbor>, current_asn: TschAsn) -> TschAsn {
        let timeslot = self.timeslot(current_asn);
        if timeslot < link.timeslot {
            current_asn + (link.timeslot - timeslot) as u64
        } else {
            current_asn + (self.size - (timeslot - link.timeslot)) as u64
        }
    }

    /// Get the slotframe size
    pub fn size(&self) -> u16 {
        self.size
    }
}

pub struct TschSchedule<const MAX_SLOTFRAMES: usize, const MAX_LINKS: usize, Neighbor> {
    slotframes: Vec<TschSlotframe, MAX_SLOTFRAMES>,
    links: Vec<TschLink<Neighbor>, MAX_LINKS>,
    /// Sequence of PHY channels that allows for a different channel to be
    /// used at a given ASN
    hopping_sequence: TschHoppingSequence,
    /// Metric used when selecting and joining a TSCH network
    join_metric: u16,
    /// Timings used for communication inside a timeslot
    pub(crate) timeslot_timings: TschTimeslotTimings,
}

impl<const MAX_SLOTFRAMES: usize, const MAX_LINKS: usize, Neighbor>
    TschSchedule<MAX_SLOTFRAMES, MAX_LINKS, Neighbor>
{
    pub fn new(hopping_sequence: TschHoppingSequence) -> Self {
        Self {
            hopping_sequence,
            ..Default::default()
        }
    }

    /// Add a given slotframe to the schedule.
    ///
    /// * `slotframe` - Slotframe to add
    pub(crate) fn create_slotframe(&mut self, size: u16) -> Result<u16, ScheduleError> {
        if self.slotframes.push(TschSlotframe { size }).is_err() {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.slotframes.len() as u16 - 1)
        }
    }
    /// Add the given link to the slotframe
    ///
    /// * `link` - Link to add
    pub fn add_link(&mut self, link: TschLink<Neighbor>) -> Result<u16, ScheduleError> {
        if let Some(slotframe) = self.slotframes.get(link.slotframe_handle as usize) {
            if link.timeslot >= slotframe.size {
                Err(ScheduleError::InvalidTimeslot)
            } else if link.channel_offset as usize >= self.hopping_sequence.len() {
                Err(ScheduleError::InvalidChannelOffset)
            } else if self.links.push(link).is_err() {
                Err(ScheduleError::CapacityExceeded)
            } else {
                Ok(self.links.len() as u16 - 1)
            }
        } else {
            Err(ScheduleError::InvalidSlotframe)
        }
    }

    pub fn next_advertisement_link(&self, _asn: TschAsn) -> Option<&TschLink<Neighbor>> {
        // TODO: for now we only support a single link
        self.links.first()
    }

    pub(crate) fn next_asn_for_link(
        &self,
        link: &TschLink<Neighbor>,
        current_asn: TschAsn,
    ) -> TschAsn {
        if let Some(slotframe) = self.slotframes.get(link.slotframe_handle as usize) {
            slotframe.next_asn(link, current_asn)
        } else {
            // TODO: handle invalid/outdated link
            panic!()
        }
    }

    /// Return the channel offset for a given link at a given ASN
    /// * `asn` - Absolute slot number
    /// * `link_channel_offset` - Channel offset of the link to consider
    pub(crate) fn channel(&self, asn: TschAsn, link: &TschLink<Neighbor>) -> Channel {
        let channel_offset =
            ((asn + link.channel_offset as u64) % self.hopping_sequence.len() as u64) as usize;
        // Safety: index in range by using modulo
        self.hopping_sequence[channel_offset]
    }

    /// Get an iterator over slotframes
    pub(crate) fn slotframes(&self) -> impl Iterator<Item = &TschSlotframe> {
        self.slotframes.iter()
    }

    /// Get an iterator over links
    pub fn links(&self) -> impl Iterator<Item = &TschLink<Neighbor>> {
        self.links.iter()
    }

    /// Get slotframe info for beacon IE generation
    /// Returns an iterator of (handle, size) tuples
    pub fn slotframe_info(&self) -> impl Iterator<Item = (u16, u16)> + '_ {
        self.slotframes
            .iter()
            .enumerate()
            .map(|(idx, sf)| (idx as u16, sf.size))
    }

    /// Get the hopping sequence
    pub fn hopping_sequence(&self) -> &TschHoppingSequence {
        &self.hopping_sequence
    }

    /// Get number of slotframes
    pub fn num_slotframes(&self) -> usize {
        self.slotframes.len()
    }

    /// Get number of links
    pub fn num_links(&self) -> usize {
        self.links.len()
    }
}

impl<const MAX_SLOTFRAMES: usize, const MAX_LINKS: usize, Neighbor> Default
    for TschSchedule<MAX_SLOTFRAMES, MAX_LINKS, Neighbor>
{
    fn default() -> Self {
        Self {
            slotframes: heapless::Vec::new(),
            links: heapless::Vec::new(),
            join_metric: 1,
            timeslot_timings: TschTimeslotTimings::default(),
            hopping_sequence: [Channel::_12, Channel::_14],
        }
    }
}
