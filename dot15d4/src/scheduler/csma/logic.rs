//! CSMA scheduler logic - refactored for efficiency.

use dot15d4_driver::{
    radio::{
        config,
        frame::{Address, PanId, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::RadioTimerApi,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};

#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::UseTschCommandResult;
#[cfg(feature = "tsch")]
use crate::scheduler::{
    command::tsch::{TschCommand, TschCommandResult},
    tsch::TschPib,
};
use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    mac::mlme::set::SetRequestAttribute,
    scheduler::task::SchedulerTaskCompletion,
};
use crate::{
    pib::Pib,
    scheduler::{
        action::SchedulerAction,
        command::pib::*,
        task::{SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition},
        SchedulerCommandResult, SchedulerContext, SchedulerRequest, SchedulerResponse,
        SchedulerTransmissionResult,
    },
};

use super::task::{CsmaState, CsmaTask, Pipelined};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for CsmaTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.state {
            CsmaState::Idle => self.execute_idle(event, context),
            CsmaState::WaitingForTxStart => self.execute_waiting_for_tx_start(event, context),
            CsmaState::Transmitting => self.execute_transmitting(event, context),
            CsmaState::Listening => self.execute_listening(event, context),
            CsmaState::Receiving => self.execute_receiving(event, context),
            #[cfg(feature = "tsch")]
            CsmaState::Terminating => self.execute_terminating(event, context),
        }
    }
}

// ============================================================================
// States Execution
// ============================================================================
impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn execute_idle(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::Entry => {
                // Decide what to do next: pending TX, channel TX, or start RX.
                SchedulerTaskTransition::Execute(self.next_action(context), None)
            }
            _ => unreachable!(),
        }
    }

    fn execute_listening(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            // A Scheduler Request arrived while listening for frame
            SchedulerTaskEvent::SchedulerRequest { token, request } => match request {
                SchedulerRequest::Transmission(mpdu) => {
                    SchedulerTaskTransition::Execute(self.schedule_tx(token, mpdu, context), None)
                }
                SchedulerRequest::Command(cmd) => {
                    self.on_scheduler_command_request(token, cmd, context)
                }
                _ => unreachable!(),
            },
            // Incoming frame detected
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::FrameStarted) => {
                self.state = CsmaState::Receiving;
                // Frame arriving - pipeline next RX
                let frame = self
                    .rx_frame
                    .take()
                    .unwrap_or_else(|| context.allocate_frame());
                let req = self.build_rx_request(frame, None);
                self.send_driver_request_and_wait(req)
            }
            _ => unreachable!(),
        }
    }

    fn execute_receiving(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Received(frame, instant)) => {
                self.base_time = instant;
                self.state = CsmaState::Listening;

                // Check for pending RX request
                if let Some((token, _)) = context.try_receive_rx_request() {
                    self.rx_frame = Some(context.allocate_frame());
                    let response = Some((token, SchedulerResponse::Reception(frame, instant)));
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SelectDriverEventOrRequest,
                        response,
                    )
                } else {
                    self.rx_frame = Some(frame.forget_size::<RadioDriverImpl>());
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SelectDriverEventOrRequest,
                        None,
                    )
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CrcError(frame, instant)) => {
                self.base_time = instant;
                self.rx_frame = Some(frame);
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            _ => unreachable!(),
        }
    }

    fn execute_waiting_for_tx_start(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::TxStarted(instant)) => {
                self.base_time = instant;
                self.pipeline_next_operation(context)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CcaBusy(frame, instant)) => {
                self.base_time = instant;
                self.backoff_or_fail(frame, context)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(radio_frame)) => {
                // RX window ended during TX transition - save frame for later
                self.rx_frame = Some(radio_frame);
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            _ => unreachable!(),
        }
    }

    fn execute_transmitting(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Sent(frame, instant)) => {
                self.base_time = instant;

                let token = self.take_tx_token();
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                    frame.forget_size::<RadioDriverImpl>(),
                    instant,
                ));
                self.continue_with_pipelined((token, resp))
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Nack(frame, instant, recovered)) => {
                self.base_time = instant;

                match recovered {
                    Some(DrvSvcRequest::CompleteThenStartTx(recovered_task)) => {
                        // Store recovered TX for after retry
                        if let Some(Pipelined::Tx(token)) = self.pipelined.take() {
                            self.pending_tx = Some((token, recovered_task.mpdu));
                        }
                        self.retransmit_or_fail(frame, context)
                    }
                    Some(DrvSvcRequest::CompleteThenStartRx(task)) => {
                        self.rx_frame = Some(task.radio_frame);
                        self.pipelined = None;
                        self.retransmit_or_fail(frame, context)
                    }
                    None => {
                        // No recovery - next op already started, report failure
                        let token = self.take_tx_token();
                        let resp = SchedulerResponse::Transmission(
                            SchedulerTransmissionResult::NoAck(frame, instant),
                        );
                        self.continue_with_pipelined((token, resp))
                    }
                    _ => unreachable!(),
                }
            }
            _ => unreachable!(),
        }
    }

    #[cfg(feature = "tsch")]
    fn execute_terminating(
        &mut self,
        event: SchedulerTaskEvent,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(radio_frame)) => {
                unsafe {
                    context
                        .buffer_allocator
                        .deallocate_buffer(radio_frame.into_buffer());
                }
                let response = SchedulerResponse::Command(SchedulerCommandResult::TschCommand(
                    TschCommandResult::UseTsch(UseTschCommandResult::StartedTsch),
                ));
                SchedulerTaskTransition::Completed(
                    SchedulerTaskCompletion::SwitchToTsch,
                    Some((self.take_tx_token(), response)),
                )
            }
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// Handling Driver Service events from TX request
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn retransmit_or_fail(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        if self.tx_retries < context.pib.max_frame_retries {
            self.state = CsmaState::WaitingForTxStart;
            self.base_time = context.timer.now();
            self.tx_retries += 1;
            self.backoff.reset(context.pib.min_be);

            let at = Timestamp::Scheduled(self.backoff_time(context.rng));
            let fallback = self.tx_retries < context.pib.max_frame_retries;
            let mpdu = MpduFrame::from_radio_frame(frame);
            let req = self.build_tx_request(mpdu, at, Some(self.channel), fallback);

            self.send_driver_request_and_wait(req)
        } else {
            let token = self.take_tx_token();
            let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                frame,
                self.base_time,
            ));
            self.state = CsmaState::Idle;
            SchedulerTaskTransition::Execute(self.next_action(context), Some((token, resp)))
        }
    }

    fn backoff_or_fail(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        // Try another backoff
        if self
            .backoff
            .on_failure(context.pib.max_csma_backoffs, context.pib.max_be)
        {
            let at = Timestamp::Scheduled(self.backoff_time(context.rng));
            let fallback = self.tx_retries < context.pib.max_frame_retries;
            let mpdu = MpduFrame::from_radio_frame(frame);
            let req = self.build_tx_request(mpdu, at, Some(self.channel), fallback);
            return self.send_driver_request_and_wait(req);
        }

        // Transmission attempt ended with Channel Access failure
        let token = self.take_tx_token();
        let resp = SchedulerResponse::Transmission(
            SchedulerTransmissionResult::ChannelAccessFailure(frame),
        );
        self.state = CsmaState::Idle;
        SchedulerTaskTransition::Execute(self.next_action(context), Some((token, resp)))
    }

    /// Pipeline the next operation after TX started.
    fn pipeline_next_operation(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let fallback = context.pib.max_frame_retries > 0;

        let request = if let Some((token, mpdu)) = self.next_tx_request(context) {
            self.pipelined = Some(Pipelined::Tx(token));
            //TODO: should be scheduled timestamp, but OK for best effort for now
            self.build_tx_request(mpdu, Timestamp::BestEffort, None, fallback)
        } else {
            let frame = self.rx_frame.take().expect("no rx frame");
            self.pipelined = Some(Pipelined::Rx);
            self.build_rx_request(frame, None)
        };

        self.state = CsmaState::Transmitting;
        self.send_driver_request_and_wait(request)
    }
}

// ============================================================================
// Commands
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn on_scheduler_command_request(
        &mut self,
        token: ResponseToken,
        cmd: crate::scheduler::command::SchedulerCommand,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        use crate::scheduler::command::*;

        match cmd {
            SchedulerCommand::CsmaCommand(CsmaCommand::UseCsma(ch)) => {
                self.channel = ch;
                let resp = SchedulerResponse::Command(SchedulerCommandResult::CsmaCommand(
                    CsmaCommandResult::UseCsma(UseCsmaResult::Success),
                ));
                self.state = CsmaState::Idle;
                SchedulerTaskTransition::Execute(self.next_action(context), Some((token, resp)))
            }

            SchedulerCommand::PibCommand(cmd) => self.on_pib_cmd(token, cmd, context),

            #[cfg(feature = "tsch")]
            SchedulerCommand::TschCommand(cmd) => {
                self.on_tsch_cmd(token, cmd, &mut context.pib.tsch)
            }
        }
    }

    fn on_pib_cmd(
        &mut self,
        token: ResponseToken,
        cmd: PibCommand,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match cmd {
            PibCommand::Set(attribute) => {
                let result = match attribute {
                    SetRequestAttribute::MacExtendedAddress(addr) => {
                        context.pib.extended_address = Address::from_le_bytes(&addr);
                        SetPibResult::Success
                    }
                    SetRequestAttribute::MacAssociationPermit(permit) => {
                        context.pib.association_permit = permit;
                        SetPibResult::Success
                    }
                    SetRequestAttribute::MacPanId(pan_id) => {
                        context.pib.pan_id = PanId::new_owned(pan_id.to_le_bytes());
                        SetPibResult::Success
                    }
                    SetRequestAttribute::MacShortAddress(short_addr) => {
                        context.pib.short_address = short_addr;
                        SetPibResult::Success
                    }
                };
                let resp = SchedulerResponse::Command(SchedulerCommandResult::PibCommand(
                    PibCommandResult::Set(result),
                ));
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    Some((token, resp)),
                )
            }
            PibCommand::Reset => {
                let addr_bytes: [u8; 8] = match &context.pib.extended_address {
                    Address::Extended(ext) => ext
                        .as_ref()
                        .try_into()
                        .expect("extended address is 8 bytes"),
                    _ => [0u8; 8], // fallback to zero address if somehow not extended
                };
                context.pib = Pib::new(&addr_bytes);
                let resp = SchedulerResponse::Command(SchedulerCommandResult::PibCommand(
                    PibCommandResult::Reset(ResetPibResult::Success),
                ));
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    Some((token, resp)),
                )
            }
        }
    }

    #[cfg(feature = "tsch")]
    fn on_tsch_cmd(
        &mut self,
        token: ResponseToken,
        cmd: TschCommand,
        tsch: &mut TschPib<()>,
    ) -> SchedulerTaskTransition {
        use crate::{
            mac::mlme::tsch::TschScheduleOperation,
            scheduler::{
                command::{tsch::*, SchedulerCommandResult},
                tsch::pib::{ScheduleError, TschLink},
            },
        };

        match cmd {
            TschCommand::UseTsch(enabled, _) => {
                if enabled {
                    self.state = CsmaState::Terminating;
                    self.tx_token = Some(token);
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SendDriverRequestThenWait(
                            DrvSvcRequest::CompleteThenGoIdle,
                        ),
                        None,
                    )
                } else {
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SelectDriverEventOrRequest,
                        None,
                    )
                }
            }
            TschCommand::SetTschSlotframe(cmd) => {
                let result = match cmd.operation {
                    TschScheduleOperation::Add => {
                        match tsch.create_slotframe(cmd.handle, cmd.size) {
                            Ok(_) | Err(ScheduleError::HandleDuplicate) => {
                                SetTschSlotframeResult::Success
                            }
                            Err(ScheduleError::CapacityExceeded) => {
                                SetTschSlotframeResult::MaxSlotframesExceeded
                            }
                            Err(_) => SetTschSlotframeResult::SlotframeNotFound,
                        }
                    }
                    TschScheduleOperation::Modify => tsch
                        .slotframes
                        .iter_mut()
                        .find(|s| s.handle == cmd.handle)
                        .map(|s| {
                            s.size = cmd.size;
                            SetTschSlotframeResult::Success
                        })
                        .unwrap_or(SetTschSlotframeResult::SlotframeNotFound),
                    TschScheduleOperation::Delete => tsch
                        .slotframes
                        .iter()
                        .position(|s| s.handle == cmd.handle)
                        .map(|pos| {
                            tsch.slotframes.remove(pos);
                            tsch.links.retain(|l| l.slotframe_handle != cmd.handle);
                            SetTschSlotframeResult::Success
                        })
                        .unwrap_or(SetTschSlotframeResult::SlotframeNotFound),
                };
                let resp = SchedulerResponse::Command(SchedulerCommandResult::TschCommand(
                    TschCommandResult::SetTschSlotframe(result),
                ));
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    Some((token, resp)),
                )
            }
            TschCommand::SetTschLink(cmd) => {
                let link = TschLink {
                    slotframe_handle: cmd.slotframe_handle,
                    timeslot: cmd.timeslot,
                    channel_offset: cmd.channel_offset,
                    link_options: cmd.link_options,
                    link_type: cmd.link_type,
                    neighbor: None,
                    link_advertise: cmd.advertise,
                };
                let result = match tsch.add_link(link) {
                    Ok(_) => SetTschLinkResult::Success,
                    Err(ScheduleError::InvalidSlotframe) => SetTschLinkResult::UnknownLink,
                    Err(ScheduleError::CapacityExceeded) => SetTschLinkResult::MaxLinksExceeded,
                    Err(_) => SetTschLinkResult::UnknownLink,
                };
                let resp = SchedulerResponse::Command(SchedulerCommandResult::TschCommand(
                    TschCommandResult::SetTschLink(result),
                ));
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    Some((token, resp)),
                )
            }
        }
    }
}

// ============================================================================
// Transition Helpers
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Send driver request and wait for driver event.
    #[inline]
    fn send_driver_request_and_wait(&self, req: DrvSvcRequest) -> SchedulerTaskTransition {
        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(req), None)
    }

    fn next_action(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) -> SchedulerAction {
        // If a TX request is available in channel (or pending)
        if let Some((tx_token, mpdu)) = self.next_tx_request(context) {
            self.schedule_tx(tx_token, mpdu, context)
        } else {
            let frame = self.rx_frame.take().unwrap();
            self.state = CsmaState::Listening;
            let req = self.build_rx_request(frame, Some(self.channel));
            SchedulerAction::SendDriverRequestThenSelect(req)
        }
    }

    fn schedule_tx(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerAction {
        self.state = CsmaState::WaitingForTxStart;

        self.base_time = context.timer.now();
        self.backoff.reset(context.pib.min_be);
        self.tx_token = Some(token);
        self.tx_retries = 0;

        // FIXME: fix scheduled timings
        let at = Timestamp::BestEffort;
        let fallback = self.tx_retries < context.pib.max_frame_retries;
        let req = self.build_tx_request(mpdu, at, Some(self.channel), fallback);

        SchedulerAction::SendDriverRequestThenWait(req)
    }

    /// Continue with next pipelined operation after TX completes.
    fn continue_with_pipelined(
        &mut self,
        response: (ResponseToken, SchedulerResponse),
    ) -> SchedulerTaskTransition {
        match self.pipelined.take() {
            Some(Pipelined::Tx(token)) => {
                self.state = CsmaState::WaitingForTxStart;
                self.tx_token = Some(token);
                self.tx_retries = 0;
                SchedulerTaskTransition::Execute(
                    SchedulerAction::WaitForDriverEvent,
                    Some(response),
                )
            }
            _ => {
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    Some(response),
                )
            }
        }
    }

    /// Retrieves the next TX request to process, if any.
    fn next_tx_request(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> Option<(ResponseToken, MpduFrame)> {
        // Priorities:
        // 1. Pending TX from NACK recovery
        // 2. Tx request from channel
        if let Some(pending) = self.pending_tx.take() {
            Some(pending)
        } else if let Some((tx_token, mpdu)) = context.try_receive_tx_request() {
            Some((tx_token, mpdu))
        } else {
            None
        }
    }
}

// ============================================================================
// Request Builders
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    #[inline]
    fn build_tx_request(
        &self,
        mpdu: MpduFrame,
        at: Timestamp,
        channel: Option<config::Channel>,
        fallback: bool,
    ) -> DrvSvcRequest {
        DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
            at,
            mpdu,
            cca: true,
            channel,
            fallback_on_nack: fallback,
        })
    }

    #[inline]
    fn build_rx_request(
        &self,
        radio_frame: RadioFrame<RadioFrameUnsized>,
        channel: Option<config::Channel>,
    ) -> DrvSvcRequest {
        DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
            start: Timestamp::BestEffort,
            radio_frame,
            channel,
            rx_window: None,
        })
    }
}
