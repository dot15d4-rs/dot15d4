#![allow(dead_code)]
use crate::mac::{
    frame::fields::{TschLinkOption, TschTimeslotTimings},
    neighbors::MacNeighbor,
};

use super::asn::AbsoluteSlotNumber;

pub enum ScheduleError {
    InvalidTimeslot,
    InvalidChannelOffset,
    CapacityExceeded,
    HandleDuplicate,
}

/// A TSCH link is a pairwise assignment of a directed communication between
/// devices for a given slotframe, in a given timeslot on a given channel offset.
#[allow(dead_code)]
pub struct TschLink<T: MacNeighbor> {
    /// Link Identifier
    handle: u16,
    /// Associated timeslot in the slotframe
    timeslot: u16,
    /// Associated Channel offset for the given timeslot for the link
    channel_offset: u16,
    /// Link communication option
    link_options: TschLinkOption,
    /// Type of link (normal or advertising)
    link_type: TschLinkType,
    /// Neighbor assigned to the link for communication. None if not a
    /// dedicated link
    neighbor: Option<T>,
}

/// Type of link
pub enum TschLinkType {
    Advertising,
    Normal,
}

/// Represents a channel hopping sequence
pub type TschHoppingSequence = [u8; 4];

/// A TSCH slotframe collection of timeslots repeating in time, analogous to a
/// superframe in that it defines periods of communication opportunities.
#[allow(dead_code)]
pub struct TschSlotframe<const N: usize, T: MacNeighbor> {
    /// Slotframe Identifier
    handle: u16,
    /// The number of timeslots in a given slotframe, representing of often a
    /// timeslot repeats.
    size: u16,
    /// Sequence of PHY channels that allows for a different channel to be
    /// used at a given ASN
    hopping_sequence: TschHoppingSequence,
    /// Sequence of links configured for the slotframe.
    links: heapless::Vec<TschLink<T>, N>,
}

#[allow(dead_code)]
impl<const N: usize, T: MacNeighbor> TschSlotframe<N, T> {
    /// Creates a new [`TschSlotframe`].
    pub fn new(handle: u16, size: u16, hopping_sequence: TschHoppingSequence) -> Self {
        Self {
            handle,
            size,
            hopping_sequence,
            links: heapless::Vec::new(),
        }
    }

    /// Add the given link to the slotframe
    ///
    /// * `link` - Link to add
    pub fn add_link(&mut self, link: TschLink<T>) -> Result<(), ScheduleError> {
        if link.timeslot >= self.size {
            Err(ScheduleError::InvalidTimeslot)
        } else if link.channel_offset as usize >= self.hopping_sequence.len() {
            Err(ScheduleError::InvalidChannelOffset)
        } else if self.links.iter().any(|l| l.handle == link.handle) {
            Err(ScheduleError::HandleDuplicate)
        } else if self.links.push(link).is_err() {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(())
        }
    }

    /// Return the link associated to the given ASN, if any.
    ///
    /// * `asn` - Absolute slot number
    pub fn get_link(&self, asn: AbsoluteSlotNumber) -> Option<&TschLink<T>> {
        let timeslot = self.timeslot(asn);
        self.links.iter().find(|l| l.timeslot == timeslot)
    }

    /// Return the timeslot within the slotframe for a given ASN
    ///
    /// * `asn` - Absolute slot number
    fn timeslot(&self, asn: AbsoluteSlotNumber) -> u16 {
        asn % self.size
    }

    /// Return the channel offset for a given link at a given ASN
    /// * `asn` - Absolute slot number
    /// * `link` - Link to consider
    fn channel_offset(&self, asn: AbsoluteSlotNumber, link: TschLink<T>) -> u16 {
        (asn + link.channel_offset) % self.size
    }
}

/// Entity that allows for managing a TSCH schedule composed of multiple
/// slotframes. It used for iterating over slots and identifying link to
/// consider for the current slot.
#[allow(dead_code)]
pub struct TschSchedule<const S: usize, const L: usize, T: MacNeighbor> {
    /// Sequence of slotframes associated to the schedule.
    slotframes: heapless::Vec<TschSlotframe<L, T>, S>,
    /// The total number of timeslots that has elapsed since the start of the
    /// network.
    asn: AbsoluteSlotNumber,
    /// Metric used when selecting and joining a TSCH network
    join_metric: u16,
    /// Timings used for communication inside a timeslot
    timeslot_timings: TschTimeslotTimings,
}

#[allow(dead_code)]
impl<const S: usize, const L: usize, T: MacNeighbor> TschSchedule<S, L, T> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a given slotframe to the schedule.
    ///
    /// * `slotframe` - Slotframe to add
    pub(crate) fn add_slotframe(
        &mut self,
        slotframe: TschSlotframe<L, T>,
    ) -> Result<(), ScheduleError> {
        if self.slotframes.iter().any(|s| s.handle == slotframe.handle) {
            Err(ScheduleError::HandleDuplicate)
        } else if self.slotframes.push(slotframe).is_err() {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(())
        }
    }

    /// Return the link associated to the current ASN, if any, and then
    /// increment the ASN.
    pub(crate) fn next_slot(&mut self) -> Option<&TschLink<T>> {
        let mut slot = None;
        for slotframe in &self.slotframes {
            if let Some(link) = slotframe.get_link(self.asn) {
                slot = Some(link);
                break;
            }
        }
        self.asn.increment();
        slot
    }

    /// Increment ASN until a link is found. Return the link.
    pub(crate) fn next_active_slot(&mut self) -> Option<&TschLink<T>> {
        if self.slotframes.iter().all(|s| s.links.is_empty()) {
            None
        } else {
            loop {
                let mut slot = None;
                for slotframe in &self.slotframes {
                    if let Some(link) = slotframe.get_link(self.asn) {
                        slot = Some(link);
                        break;
                    }
                }
                self.asn.increment();
                if slot.is_some() {
                    return slot;
                }
            }
        }
    }

    /// Set the absolute slot number.
    pub(crate) fn set_asn(&mut self, asn: AbsoluteSlotNumber) {
        self.asn = asn;
    }
}

impl<const S: usize, const L: usize, T: MacNeighbor> Default for TschSchedule<S, L, T> {
    fn default() -> Self {
        Self {
            slotframes: heapless::Vec::new(),
            join_metric: 1,
            asn: AbsoluteSlotNumber::try_from(0).unwrap(),
            timeslot_timings: TschTimeslotTimings::default(),
        }
    }
}

#[cfg(test)]
pub mod tests {
    use crate::mac::{frame::fields::TschLinkOption, neighbors::tests::TestNeighbor};

    use super::{ScheduleError, TschLink, TschLinkType, TschSchedule, TschSlotframe};

    #[test]
    fn schedule() {
        let hopping_sequence = [15, 25, 26, 20];
        let nbr1 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 1]);
        let nbr2 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 2]);
        let mut sf = TschSlotframe::new(1, 3, hopping_sequence);

        let res = sf.add_link(TschLink {
            handle: 0,
            channel_offset: 0,
            timeslot: 0,
            link_options: TschLinkOption::Tx,
            link_type: TschLinkType::Normal,
            neighbor: Some(nbr1),
        });

        assert!(res.is_ok());
        assert_eq!(sf.links.len(), 1);

        let res = sf.add_link(TschLink {
            handle: 1,
            channel_offset: 0,
            timeslot: 2,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: Some(nbr2),
        });
        assert!(res.is_ok());
        assert_eq!(sf.links.len(), 2);

        let res = sf.add_link(TschLink {
            handle: 2,
            channel_offset: 0,
            timeslot: 1,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });
        match res.unwrap_err() {
            ScheduleError::CapacityExceeded => (),
            _ => panic!(),
        };

        assert_eq!(sf.links.len(), 2);

        let mut schedule = TschSchedule::<1, 2, _>::new();
        let res = schedule.add_slotframe(sf);
        assert!(res.is_ok());

        {
            let slot = schedule.next_slot().unwrap();
            assert_eq!(slot.timeslot, 0);
        }
        {
            let inactive_slot = schedule.next_slot();
            assert!(inactive_slot.is_none());
        }
        {
            let slot = schedule.next_slot().unwrap();
            assert_eq!(slot.timeslot, 2);
        }
        {
            let slot = schedule.next_slot().unwrap();
            assert_eq!(slot.timeslot, 0);
        }
        {
            // Next active slot is two slots away. Should skip one slot.
            let active_slot = schedule.next_active_slot().unwrap();
            assert_eq!(active_slot.timeslot, 2);
        }
    }

    #[test]
    fn invalid_links() {
        let hopping_sequence = [15, 25, 26, 20];
        let mut sf = TschSlotframe::<2, TestNeighbor>::new(1, 11, hopping_sequence);

        let res = sf.add_link(TschLink {
            handle: 0,
            channel_offset: 0,
            timeslot: 12,
            link_options: TschLinkOption::Tx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });
        match res.unwrap_err() {
            ScheduleError::InvalidTimeslot => (),
            _ => panic!(),
        };

        let res = sf.add_link(TschLink {
            handle: 1,
            channel_offset: 10,
            timeslot: 8,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });
        match res.unwrap_err() {
            ScheduleError::InvalidChannelOffset => (),
            _ => panic!(),
        };

        let res = sf.add_link(TschLink {
            handle: 0,
            channel_offset: 0,
            timeslot: 10,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });
        assert!(res.is_ok());

        let res = sf.add_link(TschLink {
            handle: 0,
            channel_offset: 0,
            timeslot: 10,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });
        match res.unwrap_err() {
            ScheduleError::HandleDuplicate => (),
            _ => panic!(),
        };
    }
    #[test]
    fn multiple_slotframes() {
        let hopping_sequence = [15, 25, 26, 20];
        let mut sf1 = TschSlotframe::new(1, 3, hopping_sequence);
        let mut sf2 = TschSlotframe::new(2, 2, hopping_sequence);

        let _res = sf1.add_link(TschLink {
            handle: 1,
            channel_offset: 0,
            timeslot: 0,
            link_options: TschLinkOption::Tx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });

        // Create a link that will overlap with link from SF 1
        let _res = sf2.add_link(TschLink {
            handle: 2,
            channel_offset: 1,
            timeslot: 0,
            link_options: TschLinkOption::Rx,
            link_type: TschLinkType::Normal,
            neighbor: None,
        });

        let mut schedule = TschSchedule::<2, 2, TestNeighbor>::new();
        let res = schedule.add_slotframe(sf1);
        assert!(res.is_ok());

        // Invalid slotframe (handle already used)
        let invalid_sf = TschSlotframe::new(1, 3, hopping_sequence);
        let res = schedule.add_slotframe(invalid_sf);
        match res.unwrap_err() {
            ScheduleError::HandleDuplicate => (),
            _ => panic!(),
        };

        let res = schedule.add_slotframe(sf2);
        assert!(res.is_ok());

        {
            // Two links for the current ASN, should be a link from SF1
            let slot = schedule.next_slot().unwrap();
            assert_eq!(slot.handle, 1);
        }

        {
            // Next active slot is from SF 2
            let active_slot = schedule.next_active_slot().unwrap();
            assert_eq!(active_slot.handle, 2);
        }
        {
            // Next active slot is from SF 1
            let active_slot = schedule.next_active_slot().unwrap();
            assert_eq!(active_slot.handle, 1);
        }

        // Adding a third slotframe should not work
        let sf3 = TschSlotframe::new(3, 3, hopping_sequence);
        let res = schedule.add_slotframe(sf3);
        match res.unwrap_err() {
            ScheduleError::CapacityExceeded => (),
            _ => panic!(),
        };
    }
}
