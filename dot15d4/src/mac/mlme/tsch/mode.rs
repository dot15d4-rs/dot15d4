#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::DriverConfig;

use crate::{
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::tsch::{TschCommand, UseTschCommandResult},
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

pub struct TschModeRequest {
    /// Indication of whether the device is joining (or associated to) a TSCH
    /// network (i.e. not using unslotted CSMA-CA)
    pub tsch_mode: bool,
    /// Indication of whether the device should use CCA during CSMA-CA TSCH
    pub tsch_cca: bool,
}

impl TschModeRequest {
    pub fn new(tsch_mode: bool, tsch_cca: bool) -> Self {
        Self {
            tsch_mode,
            tsch_cca,
        }
    }
}

pub(crate) struct TschModeRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: TschModeRequestState,
    task: PhantomData<&'task u8>,
    radio: PhantomData<RadioDriverImpl>,
}

pub(crate) enum TschModeRequestState {
    Initial(bool, bool),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> TschModeRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: TschModeRequest) -> Self {
        Self {
            state: TschModeRequestState::Initial(request.tsch_mode, request.tsch_cca),
            task: PhantomData,
            radio: PhantomData,
        }
    }
}

/// Final result of a data request task.
pub enum TschModeConfirm {
    Started,
    Stopped,
}

impl<RadioDriverImpl: DriverConfig> MacTask for TschModeRequestTask<'_, RadioDriverImpl> {
    type Result = TschModeConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            TschModeRequestState::Initial(tsch_mode, tsch_cca) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = TschModeRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::TschCommand(TschCommand::UseTsch(
                        tsch_mode, tsch_cca,
                    ))),
                    None,
                )
            }
            TschModeRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::TschCommand(
                        crate::scheduler::command::tsch::TschCommandResult::UseTsch(command_result),
                    ),
                )) => {
                    let task_result = match command_result {
                        UseTschCommandResult::StartedTsch => TschModeConfirm::Started,
                        UseTschCommandResult::StoppedTsch => TschModeConfirm::Stopped,
                    };
                    MacTaskTransition::Terminated(task_result)
                }
                _ => unreachable!(),
            },
        }
    }
}
