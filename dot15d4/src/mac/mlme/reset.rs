#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::DriverConfig;

use crate::{
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::pib::{PibCommand, PibCommandResult, ResetPibResult},
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

#[cfg(feature = "rtos-trace")]
use crate::trace::MAC_REQUEST;

/// MLME-RESET.request primitive
///
/// IEEE 802.15.4-2024, section 10.2.9
pub struct ResetRequest {
    /// If TRUE, the MAC sublayer is reset and all PIB attributes are set to
    /// their default values. If FALSE, the MAC sublayer is reset but all PIB
    /// attributes retain their values prior to the generation of the
    /// MLME-RESET.request primitive.
    pub set_default_pib: bool,
}

impl ResetRequest {
    pub fn new(set_default_pib: bool) -> Self {
        Self { set_default_pib }
    }
}

/// MLME-RESET.confirm primitive
pub struct ResetConfirm {
    pub status: ResetStatus,
}

pub enum ResetStatus {
    Success,
}

pub(crate) struct ResetRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: ResetRequestState,
    task_marker: PhantomData<&'task u8>,
    radio_marker: PhantomData<RadioDriverImpl>,
}

pub(crate) enum ResetRequestState {
    Initial(bool),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> ResetRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: ResetRequest) -> Self {
        Self {
            state: ResetRequestState::Initial(request.set_default_pib),
            task_marker: PhantomData,
            radio_marker: PhantomData,
        }
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for ResetRequestTask<'_, RadioDriverImpl> {
    type Result = ResetConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            ResetRequestState::Initial(_set_default_pib) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = ResetRequestState::SendingRequest;
                // Always send Reset command - the scheduler will handle the PIB reset
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::PibCommand(PibCommand::Reset)),
                    None,
                )
            }
            ResetRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::PibCommand(PibCommandResult::Reset(result)),
                )) => {
                    let status = match result {
                        ResetPibResult::Success => ResetStatus::Success,
                    };
                    MacTaskTransition::Terminated(ResetConfirm { status })
                }
                _ => unreachable!(),
            },
        }
    }
}
