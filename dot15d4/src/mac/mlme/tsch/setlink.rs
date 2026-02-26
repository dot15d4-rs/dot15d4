#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::radio::{frame::Address, DriverConfig};
use dot15d4_frame::fields::TschLinkOption;

use crate::{
    mac::task::{MacTask, MacTaskEvent, MacTaskTransition},
    scheduler::{
        command::tsch::{SetTschLinkResult, TschCommandResult},
        tsch::pib::TschLinkType,
        SchedulerCommand, SchedulerCommandResult, SchedulerRequest, SchedulerResponse,
    },
};

pub struct SetLinkRequest {
    /// Slotframe identifier of the slotframe to which the link is associated.
    pub slotframe_handle: u8,
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
    pub neighbor: Option<Address<[u8; 4]>>,
    /// Wether this link shall be advertised in Enhanced beacon frames
    /// using the TSCH Slotframe and Link IE. If not, this link shall
    /// be added locally only.
    pub advertise: bool,
}

pub(crate) struct SetLinkRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: SetLinkRequestState,
    task: PhantomData<&'task u8>,
    radio: PhantomData<RadioDriverImpl>,
}

pub(crate) enum SetLinkRequestState {
    Initial(SetLinkRequest),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> SetLinkRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: SetLinkRequest) -> Self {
        Self {
            state: SetLinkRequestState::Initial(request),
            task: PhantomData,
            radio: PhantomData,
        }
    }
}

/// Final result of a data request task.
pub enum SetLinkConfirm {
    Success,
    UnknownLink,
    MaxLinksExceeded,
}

impl<RadioDriverImpl: DriverConfig> MacTask for SetLinkRequestTask<'_, RadioDriverImpl> {
    type Result = SetLinkConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            SetLinkRequestState::Initial(request) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = SetLinkRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::TschCommand(
                        crate::scheduler::command::tsch::TschCommand::SetTschLink(request),
                    )),
                    None,
                )
            }
            SetLinkRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::TschCommand(TschCommandResult::SetTschLink(
                        command_result,
                    )),
                )) => {
                    // TODO: implement into()
                    let task_result = match command_result {
                        SetTschLinkResult::Success => SetLinkConfirm::Success,
                        SetTschLinkResult::UnknownLink => SetLinkConfirm::UnknownLink,
                        SetTschLinkResult::MaxLinksExceeded => SetLinkConfirm::MaxLinksExceeded,
                    };
                    MacTaskTransition::Terminated(task_result)
                }
                _ => unreachable!(),
            },
        }
    }
}
