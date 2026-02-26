#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::DriverConfig;

use crate::{
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::tsch::{SetTschSlotframeResult, TschCommand, TschCommandResult},
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

use super::TschScheduleOperation;

pub struct SetSlotframeRequest {
    /// Slotframe Identifier
    pub handle: u8,
    /// The number of timeslots in a given slotframe, representing of often a
    /// timeslot repeats.
    pub size: u16,
    /// The type of operation to be performed for the slotframe (add, modify, delete).
    pub operation: TschScheduleOperation,
    /// Wether this slotframe shall be advertised in Enhanced beacon frames
    /// using the TSCH Slotframe and Link IE. If not, this link shall
    /// be added locally only.
    pub advertise: bool,
}

pub(crate) struct SetSlotframeRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: SetSlotframeRequestState,
    task: PhantomData<&'task u8>,
    radio: PhantomData<RadioDriverImpl>,
}

pub(crate) enum SetSlotframeRequestState {
    Initial(SetSlotframeRequest),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> SetSlotframeRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: SetSlotframeRequest) -> Self {
        Self {
            state: SetSlotframeRequestState::Initial(request),
            task: PhantomData,
            radio: PhantomData,
        }
    }
}

/// Final result of a data request task.
pub enum SetSlotframeConfirm {
    Success,
    SlotframeNotFound,
    MaxSlotframesExceeded,
}

impl<RadioDriverImpl: DriverConfig> MacTask for SetSlotframeRequestTask<'_, RadioDriverImpl> {
    type Result = SetSlotframeConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            SetSlotframeRequestState::Initial(request) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = SetSlotframeRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::TschCommand(
                        TschCommand::SetTschSlotframe(request),
                    )),
                    None,
                )
            }
            SetSlotframeRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::TschCommand(TschCommandResult::SetTschSlotframe(
                        command_result,
                    )),
                )) => {
                    // TODO: implement into()
                    let task_result = match command_result {
                        SetTschSlotframeResult::Success => SetSlotframeConfirm::Success,
                        SetTschSlotframeResult::SlotframeNotFound => {
                            SetSlotframeConfirm::SlotframeNotFound
                        }
                        SetTschSlotframeResult::MaxSlotframesExceeded => {
                            SetSlotframeConfirm::MaxSlotframesExceeded
                        }
                    };
                    MacTaskTransition::Terminated(task_result)
                }
                _ => unreachable!(),
            },
        }
    }
}
