use core::cell::Cell;
use core::time::Duration;

use dot15d4_driver::radio::config::Channel;
use dot15d4_driver::radio::frame::{FrameType, RadioFrame, RadioFrameSized};
use dot15d4_driver::radio::DriverConfig;
use dot15d4_driver::timer::{NsDuration, NsInstant, OptionalNsInstant};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;
use heapless::Vec;

use crate::constants::MAC_TSCH_MAX_PENDING_OPERATIONS;
use crate::scheduler::{SchedulerRequest, SchedulerService};

use super::beacon::EnhancedBeaconBuilder;
use super::pib::TschAsn;

/// Guard time before a timeslot starts, used to wake up early and prepare for the slot.
const TIMESLOT_GUARD_TIME: NsDuration = NsDuration::micros(2000);

/// Expected clock drift per timeslot, used to compensate for clock inaccuracies.
// const CLOCK_DRIFT_PER_SLOT: NsDuration = NsDuration::nanos(520);
const CLOCK_DRIFT_PER_SLOT: NsDuration = NsDuration::nanos(0);

/// Represents an infinite deadline, used when no pending operations exist.
const INFINITE_DEADLINE: NsInstant = NsInstant::from_ticks(u64::MAX);

pub struct TschState<RadioDriverImpl: DriverConfig> {
    /// Queue of pending operations to be executed in upcoming timeslots.
    pending_operations: Vec<TschOperation, MAC_TSCH_MAX_PENDING_OPERATIONS>,
    /// The timestamp of the last known timeslot start, used as a reference for ASN calculations.
    pub(crate) last_base_time: NsInstant,
    /// Whether this device is operating as a TSCH coordinator.
    is_coordinator: bool,
    /// The last known Absolute Slot Number.
    pub(crate) last_asn: TschAsn,
    // TODO: feature tsch-coordinator
    pub(crate) beacon_frame: Cell<Option<RadioFrame<RadioFrameSized>>>,
    pub(crate) beacon_builder: EnhancedBeaconBuilder<'static, RadioDriverImpl>,
}

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

impl<RadioDriverImpl: DriverConfig> TschState<RadioDriverImpl> {
    pub(crate) fn new() -> Self {
        Self {
            pending_operations: Vec::<TschOperation, MAC_TSCH_MAX_PENDING_OPERATIONS>::new(),
            last_asn: 0,
            last_base_time: NsInstant::from_ticks(0),
            is_coordinator: false,
            beacon_frame: Cell::new(None),
            beacon_builder: EnhancedBeaconBuilder::new(),
        }
    }
}

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    fn queue_next_advertisement(&mut self) {
        if self.tsch_state.is_coordinator {
            let current_asn = self.tsch_state.last_asn;
            if let Some(advertisement_link) = self.next_advertisement_link(current_asn) {
                let next_asn = self.next_asn_for_link(advertisement_link, current_asn);
                let channel = self.channel(next_asn, advertisement_link);
                let operation = TschOperation::AdvertisementSlot(next_asn, channel);
                let _ = self.tsch_state.pending_operations.push(operation);
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
                    FrameType::Beacon => self.next_advertisement_link(current_asn),
                    // FrameType::Data => todo!(),
                    _ => unreachable!(),
                };

                if let Some(link) = link {
                    let asn = self.next_asn_for_link(link, current_asn);
                    // TODO: handle CCA
                    let cca = false;
                    let channel = self.channel(asn, link);

                    // TODO: handle capacity exceeded
                    self.tsch_state
                        .pending_operations
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
        match self.tsch_state.pending_operations.pop() {
            Some(operation) => {
                match &operation {
                    TschOperation::TxSlot(_, asn, _, _, _) | TschOperation::RxSlot(asn, _, _) => {
                        self.tsch_state.last_base_time = self.expected_slot_start(*asn);
                        self.tsch_state.last_asn = *asn;
                    }
                    TschOperation::AdvertisementSlot(asn, _channel) => {
                        self.tsch_state.last_base_time = self.expected_slot_start(*asn);
                        self.tsch_state.last_asn = *asn;
                        self.queue_next_advertisement();
                    }
                    _ => unreachable!(),
                }
                (
                    self.next_deadline(),
                    self.tsch_state.last_base_time,
                    operation,
                )
            }
            None => {
                // No pending operation. Next deadline is never and we'll just
                // be waiting for next scheduler request
                (
                    INFINITE_DEADLINE,
                    self.tsch_state.last_base_time,
                    TschOperation::Idle,
                )
            }
        }
    }

    pub(super) fn next_deadline(&self) -> NsInstant {
        match self.tsch_state.pending_operations.last() {
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
        self.tsch_state.last_asn
            + (instant - self.tsch_state.last_base_time).to_micros()
                / self.pib.tsch.timeslot_timings.timeslot_length() as u64
    }

    fn expected_slot_start(&self, asn: TschAsn) -> NsInstant {
        let slot_duration =
            NsDuration::micros(self.pib.tsch.timeslot_timings.timeslot_length() as u64);
        self.tsch_state.last_base_time
            + (asn - self.tsch_state.last_asn) as u32 * (slot_duration - CLOCK_DRIFT_PER_SLOT)
    }
}
