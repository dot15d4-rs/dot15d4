#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::config::Channel;
use dot15d4_driver::radio::DriverConfig;
#[cfg(feature = "tsch")]
use dot15d4_driver::timer::NsInstant;
use heapless::Vec;

use crate::{
    constants::MAC_HOPPING_SEQUENCE_MAX_LENGTH,
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::pib::{PibCommand, PibCommandResult, SetPibResult},
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

#[cfg(feature = "tsch")]
use crate::mac::frame::fields::TschTimeslotTimings;

#[cfg(feature = "rtos-trace")]
use crate::trace::MAC_REQUEST;

pub enum SetError {
    #[allow(dead_code)]
    InvalidParameter,
}

/// Attributes that may be written by an upper layer
pub enum SetRequestAttribute {
    // IEEE 802.15.4-2020, section 8.4.3.1, table 8-94
    MacExtendedAddress([u8; 8]),
    MacAssociationPermit(bool),
    MacCoordExtendedAddress([u8; 8]),
    MacPanId(u16),
    MacShortAddress(u16),
    #[cfg(feature = "tsch")]
    MacAsn(u64, NsInstant),
    #[cfg(feature = "tsch")]
    MacTimeslotTemplate(TschTimeslotTimings),
    /// The neighbor (extended address, little-endian) this node synchronizes
    /// to, or `None` to clear it. The neighbor is created in the neighbor
    /// table if needed and protected from eviction.
    #[cfg(feature = "tsch")]
    MacTschTimeSource(Option<[u8; 8]>),
    /// The join metric this node advertises in its Enhanced Beacons
    /// (typically parent metric + 1, so it grows along the chain).
    #[cfg(feature = "tsch")]
    MacJoinMetric(u16),
    /// Keep-alive period in timeslots (macKeepAlivePeriod, IEEE
    /// 802.15.4-2024, 10.3.10.8); `0` disables the stack-internal keep-alive
    /// toward the time source. Takes effect from the next keep-alive
    /// re-arming (a synchronizing exchange or a keep-alive transmission).
    #[cfg(feature = "tsch")]
    MacKeepAlivePeriod(u16),
    /// Hopping sequence list used for TSCH channel hopping.
    MacHoppingSequenceList(Vec<Channel, MAC_HOPPING_SEQUENCE_MAX_LENGTH>),
}

/// MLME-SET.request primitive
pub struct SetRequest {
    pub attribute: SetRequestAttribute,
}

impl SetRequest {
    pub fn new(attribute: SetRequestAttribute) -> Self {
        Self { attribute }
    }
}

/// MLME-SET.confirm primitive
pub enum SetConfirm {
    Success,
    InvalidParameter,
}

pub(crate) struct SetRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: SetRequestState,
    task_marker: PhantomData<&'task u8>,
    radio_marker: PhantomData<RadioDriverImpl>,
}

pub(crate) enum SetRequestState {
    Initial(SetRequestAttribute),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> SetRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: SetRequest) -> Self {
        Self {
            state: SetRequestState::Initial(request.attribute),
            task_marker: PhantomData,
            radio_marker: PhantomData,
        }
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for SetRequestTask<'_, RadioDriverImpl> {
    type Result = SetConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            SetRequestState::Initial(attribute) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = SetRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::PibCommand(PibCommand::Set(
                        attribute,
                    ))),
                    None,
                )
            }
            SetRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::PibCommand(PibCommandResult::Set(result)),
                )) => {
                    let confirm = match result {
                        SetPibResult::Success => SetConfirm::Success,
                        SetPibResult::InvalidParameter => SetConfirm::InvalidParameter,
                    };
                    MacTaskTransition::Terminated(confirm)
                }
                _ => unreachable!(),
            },
        }
    }
}
