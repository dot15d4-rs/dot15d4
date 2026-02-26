#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::{frame::Address, DriverConfig};

use crate::{
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::pib::{GetPibResult, PibCommand, PibCommandResult},
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

#[cfg(feature = "rtos-trace")]
use crate::trace::MAC_REQUEST;

pub enum GetError {
    #[allow(dead_code)]
    InvalidParameter,
}

/// Attributes that may be written by an upper layer
pub enum GetRequestAttribute {
    // IEEE 802.15.4-2020, section 8.4.3.1, table 8-94
    MacExtendedAddress,
    MacCoordExtendedAddress,
    MacAssociationPermit,
    MacPanId,
    MacShortAddress,
    #[cfg(feature = "tsch")]
    MacAsn,
}

/// MLME-SET.request primitive
pub struct GetRequest {
    pub attribute: GetRequestAttribute,
}

impl GetRequest {
    pub fn new(attribute: GetRequestAttribute) -> Self {
        Self { attribute }
    }
}

/// MLME-SET.confirm primitive
pub enum GetConfirm {
    MacExtendedAddress(Address<[u8; 8]>),
    MacCoordExtendedAddress(Address<[u8; 8]>),
    MacAssociationPermit(bool),
    MacPanId(u16),
    MacShortAddress(u16),
    InvalidParameter,
}

pub(crate) struct GetRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: GetRequestState,
    task_marker: PhantomData<&'task u8>,
    radio_marker: PhantomData<RadioDriverImpl>,
}

pub(crate) enum GetRequestState {
    Initial(GetRequestAttribute),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> GetRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: GetRequest) -> Self {
        Self {
            state: GetRequestState::Initial(request.attribute),
            task_marker: PhantomData,
            radio_marker: PhantomData,
        }
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for GetRequestTask<'_, RadioDriverImpl> {
    type Result = GetConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            GetRequestState::Initial(attribute) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = GetRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::PibCommand(PibCommand::Get(
                        attribute,
                    ))),
                    None,
                )
            }
            GetRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::PibCommand(PibCommandResult::Get(result)),
                )) => {
                    let confirm = match result {
                        GetPibResult::MacExtendedAddress(addr) => {
                            GetConfirm::MacExtendedAddress(addr)
                        }
                        GetPibResult::MacCoordExtendedAddress(addr) => {
                            GetConfirm::MacCoordExtendedAddress(addr)
                        }
                        GetPibResult::MacPanId(pan_id) => GetConfirm::MacPanId(pan_id),
                        GetPibResult::MacShortAddress(addr) => GetConfirm::MacShortAddress(addr),
                        _ => todo!(),
                    };
                    MacTaskTransition::Terminated(confirm)
                }
                _ => unreachable!(),
            },
        }
    }
}
