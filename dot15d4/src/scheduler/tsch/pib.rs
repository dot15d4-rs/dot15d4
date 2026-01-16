#![allow(dead_code)]
use dot15d4_driver::{
    radio::{config::Channel, DriverConfig},
    timer::NsInstant,
};
use heapless::Vec;

use crate::{
    constants::{
        MAC_DISCONNECT_TIME, MAC_JOIN_METRIC, MAC_MAX_BE, MAC_TSCH_MAX_LINKS,
        MAC_TSCH_MAX_SLOTFRAMES, MAC_TSCH_MIN_BE,
    },
    mac::{
        frame::fields::{TschLinkOption, TschTimeslotTimings},
        neighbors::MacNeighbor,
    },
    scheduler::SchedulerService,
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
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.3
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

/// A TSCH slotframe collection of timeslots repeating in time, analogous to a
/// superframe in that it defines periods of communication opportunities.
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.2
#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct TschSlotframe {
    /// Slotframe Identifier
    handle: u16,
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

/// Representation of TSCH related attributes of MAC PIB, as described in
/// IEEE802.15.4-2024 Section 10.3.11
pub struct TschPib<Neighbor> {
    /// The minimum value of the backoff exponent (BE) in the TSCH-CA algorithm (macTschMinBe)
    tsch_min_be: u8,
    /// The maximum value of the BE in the CSMA-CA algorithm, in the TSCHCA algorithm (macTschMaxBe)
    tsch_max_be: u8,
    /// Time (in Timeslots) to send out Disassociate frames before disconnecting (macDisconnectTime)
    disconnect_time: u16,
    /// Metric used when selecting and joining a TSCH network (macJoinMetric)
    join_metric: u16,
    /// macSlotframeTable
    slotframes: Vec<TschSlotframe, MAC_TSCH_MAX_SLOTFRAMES>,
    /// macLinkTable
    links: Vec<TschLink<Neighbor>, MAC_TSCH_MAX_LINKS>,
    /// Timings used for communication inside a timeslot
    /// (macTimeslotTemplate)
    pub(crate) timeslot_timings: TschTimeslotTimings,
    /// The Absolute Slot Number, i.e., the number of slots that has elapsed since the start of the network.
    asn: u64,
}

impl<Neighbor> TschPib<Neighbor> {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    /// Add a given slotframe to the schedule.
    ///
    /// * `slotframe` - Slotframe to add
    pub(crate) fn create_slotframe(
        &mut self,
        handle: u16,
        size: u16,
    ) -> Result<u16, ScheduleError> {
        if self
            .pib
            .tsch
            .slotframes
            .push(TschSlotframe { handle, size })
            .is_err()
        {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.pib.tsch.slotframes.len() as u16 - 1)
        }
    }
    /// Add the given link to the slotframe
    ///
    /// * `link` - Link to add
    pub fn add_link(&mut self, link: TschLink<()>) -> Result<u16, ScheduleError> {
        if let Some(slotframe) = self.pib.tsch.slotframes.get(link.slotframe_handle as usize) {
            if link.timeslot >= slotframe.size {
                Err(ScheduleError::InvalidTimeslot)
            } else if link.channel_offset as usize >= self.pib.hopping_sequence.len() {
                Err(ScheduleError::InvalidChannelOffset)
            } else if self.pib.tsch.links.push(link).is_err() {
                Err(ScheduleError::CapacityExceeded)
            } else {
                Ok(self.pib.tsch.links.len() as u16 - 1)
            }
        } else {
            Err(ScheduleError::InvalidSlotframe)
        }
    }

    pub fn next_advertisement_link(&self, _asn: TschAsn) -> Option<&TschLink<()>> {
        // TODO: for now we only support a single link
        self.pib.tsch.links.first()
    }

    pub(crate) fn next_asn_for_link(&self, link: &TschLink<()>, current_asn: TschAsn) -> TschAsn {
        if let Some(slotframe) = self.pib.tsch.slotframes.get(link.slotframe_handle as usize) {
            slotframe.next_asn(link, current_asn)
        } else {
            // TODO: handle invalid/outdated link
            panic!()
        }
    }

    /// Return the channel offset for a given link at a given ASN
    /// * `asn` - Absolute slot number
    /// * `link_channel_offset` - Channel offset of the link to consider
    pub(crate) fn channel(&self, asn: TschAsn, link: &TschLink<()>) -> Channel {
        let channel_offset =
            ((asn + link.channel_offset as u64) % self.pib.hopping_sequence.len() as u64) as usize;
        // Safety: index in range by using modulo
        self.pib.hopping_sequence[channel_offset]
    }

    /// Get an iterator over slotframes
    pub(crate) fn slotframes(&self) -> impl Iterator<Item = &TschSlotframe> {
        self.pib.tsch.slotframes.iter()
    }

    /// Get an iterator over links
    pub fn links(&self) -> impl Iterator<Item = &TschLink<()>> {
        self.pib.tsch.links.iter()
    }

    /// Get slotframe info for beacon IE generation
    /// Returns an iterator of (handle, size) tuples
    pub fn slotframe_info(&self) -> impl Iterator<Item = (u16, u16)> + '_ {
        self.pib
            .tsch
            .slotframes
            .iter()
            .enumerate()
            .map(|(idx, sf)| (idx as u16, sf.size))
    }

    /// Get number of slotframes
    pub fn num_slotframes(&self) -> usize {
        self.pib.tsch.slotframes.len()
    }

    /// Get number of links
    pub fn num_links(&self) -> usize {
        self.pib.tsch.links.len()
    }
}

impl<Neighbor> Default for TschPib<Neighbor> {
    fn default() -> Self {
        // Default values listed in IEEE802.15.4-2024 Table 10-16.
        Self {
            tsch_min_be: MAC_TSCH_MIN_BE,
            tsch_max_be: MAC_MAX_BE,
            slotframes: heapless::Vec::new(),
            links: heapless::Vec::new(),
            join_metric: MAC_JOIN_METRIC,
            disconnect_time: MAC_DISCONNECT_TIME,
            asn: 0,
            timeslot_timings: TschTimeslotTimings::default(),
        }
    }
}

#[cfg(test)]
pub mod tests {
    use dot15d4_driver::radio::config::Channel;
    use dot15d4_driver::timer::NsInstant;

    use crate::mac::frame::fields::TschLinkOption;
    use crate::mac::neighbors::tests::TestNeighbor;
    use crate::mac::neighbors::MacNeighbor;

    use super::{ScheduleError, TschLink, TschLinkType, TschSchedule, TschSlotframe};

    #[test]
    fn schedule() {
        const MAX_SLOTFRAMES: usize = 1;
        const MAX_LINKS: usize = 2;
        let hopping_sequence = [Channel::_15, Channel::_25];

        let nbr1 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 1]);
        let nbr2 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 2]);

        let mut schedule = TschSchedule::<MAX_SLOTFRAMES, MAX_LINKS, _>::new(hopping_sequence);

        let slotframe_handle = schedule.create_slotframe(3).unwrap();

        schedule
            .add_link(TschLink {
                slotframe_handle,
                channel_offset: 0,
                timeslot: 0,
                link_options: TschLinkOption::Tx,
                neighbor: Some(nbr1),
                ..Default::default()
            })
            .unwrap();

        assert_eq!(schedule.links.len(), 1);

        schedule
            .add_link(TschLink {
                slotframe_handle,
                channel_offset: 0,
                timeslot: 2,
                link_options: TschLinkOption::Rx,
                neighbor: Some(nbr2),
                ..Default::default()
            })
            .unwrap();
        assert_eq!(schedule.links.len(), 2);

        let res = schedule.add_link(TschLink {
            slotframe_handle,
            channel_offset: 0,
            timeslot: 1,
            link_options: TschLinkOption::Rx,
            ..Default::default()
        });
        match res.unwrap_err() {
            ScheduleError::CapacityExceeded => (),
            _ => panic!(),
        };

        assert_eq!(schedule.links.len(), 2);
    }

    #[test]
    fn invalid_links() {
        const MAX_SLOTFRAMES: usize = 2;
        const MAX_LINKS: usize = 5;

        let hopping_sequence = [Channel::_15, Channel::_25];

        let mut schedule =
            TschSchedule::<MAX_SLOTFRAMES, MAX_LINKS, TestNeighbor>::new(hopping_sequence);

        let slotframe_handle = schedule.create_slotframe(11).unwrap();

        let res = schedule.add_link(TschLink {
            slotframe_handle,
            channel_offset: 0,
            timeslot: 12,
            link_options: TschLinkOption::Tx,
            ..Default::default()
        });
        match res.unwrap_err() {
            ScheduleError::InvalidTimeslot => (),
            _ => panic!(),
        };

        let res = schedule.add_link(TschLink {
            slotframe_handle,
            channel_offset: 10,
            timeslot: 8,
            link_options: TschLinkOption::Rx,
            ..Default::default()
        });
        match res.unwrap_err() {
            ScheduleError::InvalidChannelOffset => (),
            _ => panic!(),
        };

        let res = schedule.add_link(TschLink {
            slotframe_handle,
            channel_offset: 0,
            timeslot: 10,
            link_options: TschLinkOption::Rx,
            ..Default::default()
        });
        assert!(res.is_ok());
    }
    #[test]
    fn multiple_slotframes() {
        const MAX_SLOTFRAMES: usize = 2;
        const MAX_LINKS: usize = 2;

        let hopping_sequence = [Channel::_15, Channel::_25];
        let mut schedule =
            TschSchedule::<MAX_SLOTFRAMES, MAX_LINKS, TestNeighbor>::new(hopping_sequence);

        let slotframe1_handle = schedule.create_slotframe(3).unwrap();

        let slotframe2_handle = schedule.create_slotframe(2).unwrap();

        let _res = schedule.add_link(TschLink {
            slotframe_handle: slotframe1_handle,
            channel_offset: 0,
            timeslot: 0,
            link_options: TschLinkOption::Tx,
            ..Default::default()
        });

        // Create a link that will overlap with link from SF 1
        let _res = schedule.add_link(TschLink {
            slotframe_handle: slotframe2_handle,
            channel_offset: 1,
            timeslot: 0,
            link_options: TschLinkOption::Rx,
            ..Default::default()
        });

        // Adding a third slotframe should not work
        let res = schedule.create_slotframe(3);
        match res.unwrap_err() {
            ScheduleError::CapacityExceeded => (),
            _ => panic!(),
        };
    }
}
