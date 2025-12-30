use core::time::Duration;

use dot15d4_driver::radio::config::Channel;
use dot15d4_driver::radio::frame::FrameType;
use dot15d4_driver::timer::{NsDuration, NsInstant, OptionalNsInstant};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;
use heapless::Vec;

use crate::scheduler::SchedulerRequest;

use super::schedule::{TschAsn, TschSchedule};

/// Guard time before a timeslot starts, used to wake up early and prepare for the slot.
const TIMESLOT_GUARD_TIME: NsDuration = NsDuration::micros(2000);

/// Expected clock drift per timeslot, used to compensate for clock inaccuracies.
// const CLOCK_DRIFT_PER_SLOT: NsDuration = NsDuration::nanos(520);
const CLOCK_DRIFT_PER_SLOT: NsDuration = NsDuration::nanos(0);

/// Represents an infinite deadline, used when no pending operations exist.
const INFINITE_DEADLINE: NsInstant = NsInstant::from_ticks(u64::MAX);

/// Represents a TSCH operation to be executed during a timeslot.
pub(super) enum TschOperation {
    /// Transmission slot operation.
    ///
    /// Fields:
    /// - `MpduFrame`: The frame to transmit
    /// - `TschAsn`: The Absolute Slot Number for this transmission
    /// - `Channel`: The channel to transmit on
    /// - `bool`: Whether to perform Clear Channel Assessment (CCA) before transmitting
    /// - `ResponseToken`: Token to signal completion back to the requester
    TxSlot(MpduFrame, TschAsn, Channel, bool, ResponseToken),
    /// Reception slot operation.
    ///
    /// Fields:
    /// - `TschAsn`: The Absolute Slot Number for this reception
    /// - `Channel`: The channel to listen on
    /// - `ResponseToken`: Token to signal completion back to the requester
    RxSlot(TschAsn, Channel, ResponseToken),
    AdvertisementSlot(TschAsn, Channel),
    /// Idle state, no operation scheduled.
    Idle,
}

/// Represents the operating mode of a TSCH device in the network.
pub(crate) enum TschDeviceMode {
    /// Device mode: this node synchronizes to a coordinator.
    ///
    /// Fields:
    /// - `TschAsn`: The Absolute Slot Number observed from the coordinator
    /// - `NsInstant`: The timestamp at which the ASN was observed
    Device(TschAsn, NsInstant),
    /// Coordinator mode: this node initiates and maintains the TSCH network.
    ///
    /// Fields:
    /// - `NsInstant`: The instant at which the TSCH network starts (i.e., ASN = 0)
    Coordinator(NsInstant),
}

/// Executes TSCH operations according to the configured schedule.
///
/// The runner manages timeslot scheduling, tracks the Absolute Slot Number (ASN),
/// and queues pending operations for transmission and reception.
///
/// # Type Parameters
/// - `MAX_SLOTFRAMES`: Maximum number of slotframes in the schedule
/// - `MAX_LINKS`: Maximum number of links in total, shared among slotframes
/// - `MAX_OPERATIONS`: Maximum number of pending operations that can be queued
/// - `Neighbor`: Type representing neighbor node information
pub(super) struct TschRunner<
    const MAX_SLOTFRAMES: usize,
    const MAX_LINKS: usize,
    const MAX_OPERATIONS: usize,
    Neighbor,
> {
    /// The TSCH schedule containing slotframes and links.
    schedule: TschSchedule<MAX_SLOTFRAMES, MAX_LINKS, Neighbor>,
    /// Queue of pending operations to be executed in upcoming timeslots.
    pending_operations: Vec<TschOperation, MAX_OPERATIONS>,
    /// The timestamp of the last known timeslot start, used as a reference for ASN calculations.
    last_base_time: NsInstant,
    /// Whether this device is operating as a TSCH coordinator.
    is_coordinator: bool,
    /// The last known Absolute Slot Number.
    last_asn: TschAsn,
    /// History of recent ASN values for synchronization tracking.
    asn_history: Vec<TschAsn, 10>,
}

impl<
        const MAX_SLOTFRAMES: usize,
        const MAX_LINKS: usize,
        const MAX_OPERATIONS: usize,
        Neighbor,
    > TschRunner<MAX_SLOTFRAMES, MAX_LINKS, MAX_OPERATIONS, Neighbor>
{
    pub(super) fn new(
        schedule: TschSchedule<MAX_SLOTFRAMES, MAX_LINKS, Neighbor>,
        mode: TschDeviceMode,
    ) -> Self {
        let (last_base_time, last_asn, is_coordinator) = match mode {
            TschDeviceMode::Device(asn, instant) => (instant, asn, false),
            TschDeviceMode::Coordinator(instant) => (instant, 0, true),
        };
        let mut runner = Self {
            schedule,
            pending_operations: Vec::<TschOperation, MAX_OPERATIONS>::new(),
            last_asn,
            last_base_time,
            is_coordinator,
            asn_history: Vec::new(),
        };
        runner.queue_next_advertisement();
        runner
    }

    fn queue_next_advertisement(&mut self) {
        if self.is_coordinator {
            let current_asn = self.last_asn;
            if let Some(advertisement_link) = self.schedule.next_advertisement_link(current_asn) {
                let next_asn = self
                    .schedule
                    .next_asn_for_link(advertisement_link, current_asn);
                let channel = self.schedule.channel(next_asn, advertisement_link);
                let operation = TschOperation::AdvertisementSlot(next_asn, channel);
                let _ = self.pending_operations.push(operation);
            }
        }
    }

    // TODO: should be a result if no capacity
    pub(super) fn queue_scheduler_request(
        &mut self,
        request: SchedulerRequest,
        response_token: ResponseToken,
        current_time: NsInstant,
    ) -> NsInstant {
        match request {
            SchedulerRequest::Transmission(mpdu) => {
                let current_asn = self.asn(current_time);
                let link = match mpdu.frame_control().frame_type() {
                    FrameType::Beacon => self.schedule.next_advertisement_link(current_asn),
                    // FrameType::Data => todo!(),
                    _ => unreachable!(),
                };

                if let Some(link) = link {
                    let asn = self.schedule.next_asn_for_link(link, current_asn);
                    // TODO: handle CCA
                    let cca = false;
                    let channel = self.schedule.channel(asn, link);

                    // TODO: handle capacity exceeded
                    self.pending_operations
                        .push(TschOperation::TxSlot(
                            mpdu,
                            asn,
                            channel,
                            cca,
                            response_token,
                        ))
                        .unwrap_or_default();
                    // TODO: sort operations
                } else {
                    // TODO: handle no link
                }

                self.next_deadline()
            }
            SchedulerRequest::Command(_) => todo!(),
            _ => unreachable!(),
        }
    }

    pub(super) fn next_operation(&mut self) -> (NsInstant, NsInstant, TschOperation) {
        match self.pending_operations.pop() {
            Some(operation) => {
                match &operation {
                    TschOperation::TxSlot(_, asn, _, _, _) | TschOperation::RxSlot(asn, _, _) => {
                        self.last_base_time = self.expected_slot_start(*asn);
                        self.last_asn = *asn;
                    }
                    TschOperation::AdvertisementSlot(asn, _channel) => {
                        self.last_base_time = self.expected_slot_start(*asn);
                        self.last_asn = *asn;
                        self.queue_next_advertisement();
                    }
                    _ => unreachable!(),
                }
                (self.next_deadline(), self.last_base_time, operation)
            }
            None => {
                // No pending operation. Next deadline is never and we'll just
                // be waiting for next scheduler request
                (INFINITE_DEADLINE, self.last_base_time, TschOperation::Idle)
            }
        }
    }

    pub(super) fn next_deadline(&self) -> NsInstant {
        match self.pending_operations.last() {
            Some(TschOperation::AdvertisementSlot(asn, _))
            | Some(TschOperation::TxSlot(_, asn, _, _, _))
            | Some(TschOperation::RxSlot(asn, _, _)) => {
                let instant = self.expected_slot_start(*asn);
                instant - TIMESLOT_GUARD_TIME
            }
            _ => INFINITE_DEADLINE,
        }
    }

    fn asn(&self, instant: NsInstant) -> TschAsn {
        // TODO: check if +1 or not with division
        self.last_asn
            + (instant - self.last_base_time).to_micros()
                / self.schedule.timeslot_timings.timeslot_length() as u64
    }

    fn expected_slot_start(&self, asn: TschAsn) -> NsInstant {
        let slot_duration =
            NsDuration::micros(self.schedule.timeslot_timings.timeslot_length() as u64);
        self.last_base_time + (asn - self.last_asn) as u32 * (slot_duration - CLOCK_DRIFT_PER_SLOT)
    }
}
