//! CSMA scheduler logic - refactored for efficiency.

use dot15d4_driver::{
    radio::{
        config,
        frame::{Address, PanId, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsInstant, RadioTimerApi},
};
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

use super::task::{CsmaState, CsmaTask, PipelinedInfo};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for CsmaTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::Entry => self.on_entry(context),
            SchedulerTaskEvent::DriverEvent(e) => self.on_driver_event(e, context),
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.on_scheduler_request(token, request, context)
            }
            #[cfg(feature = "tsch")]
            SchedulerTaskEvent::TimerExpired => unreachable!(),
        }
    }
}

// ============================================================================
// Entry & Dispatch
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn on_entry(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match &self.state {
            CsmaState::Idle => self.decide_next_action(context),
            CsmaState::Listening => {
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            _ => SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None),
        }
    }

    fn on_driver_event(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.state {
            CsmaState::Cca => self.on_cca(event, context),
            CsmaState::WaitingForTxResult => self.on_tx_result(event, context),
            CsmaState::Listening => self.on_listening(event, context),
            CsmaState::Receiving => self.on_receiving(event, context),
            CsmaState::Idle => {
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            #[cfg(feature = "tsch")]
            CsmaState::Terminating => self.on_terminating(event, context),
        }
    }

    fn on_scheduler_request(
        &mut self,
        token: ResponseToken,
        request: SchedulerRequest,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match request {
            SchedulerRequest::Transmission(mpdu) => {
                self.start_tx(token, mpdu.into_radio_frame::<RadioDriverImpl>(), context)
            }
            SchedulerRequest::Command(cmd) => self.on_command(token, cmd, context),
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// TX Initiation & Decision
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Decide what to do next: pending TX, channel TX, or start RX.
    fn decide_next_action(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        if let Some((token, frame)) = self.next_tx_request(context) {
            return self.start_tx(token, frame, context);
        } else {
            self.start_rx()
        }
    }

    /// Start TX with an already-prepared radio frame.
    fn start_tx(
        &mut self,
        token: ResponseToken,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        self.base_time = context.timer.now();
        self.prepare_tx(token, context.pib.min_be);

        // FIXME: fix scheduled timings
        let at = Timestamp::BestEffort;
        let fallback = self.can_retry(context.pib.max_frame_retries);
        let req = self.build_tx_request(frame, at, Some(self.channel), fallback);

        self.send_driver_request_and_wait(req)
    }
}

// ============================================================================
// CCA Handling
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn on_cca(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::TxStarted(instant) => {
                self.base_time = instant;
                self.pipeline_next_operation(context)
            }
            DrvSvcEvent::CcaBusy(frame, instant) => {
                self.base_time = instant;
                self.handle_cca_busy(frame, context)
            }
            DrvSvcEvent::RxWindowEnded(radio_frame) => {
                // RX window ended during TX transition - save frame for later
                self.rx_frame = Some(radio_frame);
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            _ => unreachable!(),
        }
    }

    fn handle_cca_busy(
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
            let fallback = self.can_retry(context.pib.max_frame_retries);
            let req = self.build_tx_request(frame, at, Some(self.channel), fallback);
            return self.send_driver_request_and_wait(req);
        }

        // Try retransmission
        if self.can_retry(context.pib.max_frame_retries) {
            return self.do_retransmit(frame, context);
        }

        // Complete failure
        self.complete_tx_failure(frame, context)
    }
}

// ============================================================================
// TX Result Handling
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn on_tx_result(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::Sent(frame, instant) => {
                self.base_time = instant;
                self.complete_tx_success(frame, instant)
            }
            DrvSvcEvent::Nack(frame, instant, recovered) => {
                self.base_time = instant;
                self.handle_nack(frame, instant, recovered, context)
            }
            _ => unreachable!(),
        }
    }

    fn handle_nack(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
        recovered: Option<DrvSvcRequest>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match recovered {
            Some(DrvSvcRequest::CompleteThenStartTx(recovered_task)) => {
                // Store recovered TX for after retry
                if let Some(PipelinedInfo::Tx(token)) = self.pipelined_info.take() {
                    self.pending_tx = Some((token, recovered_task.radio_frame));
                }
                self.retry_or_fail(frame, context)
            }
            Some(DrvSvcRequest::CompleteThenStartRx(task)) => {
                self.rx_frame = Some(task.radio_frame);
                self.pipelined_info = None;
                self.retry_or_fail(frame, context)
            }
            None => {
                // No recovery - next op already started, report failure
                let token = self.take_tx_token();
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                    frame, instant,
                ));
                self.continue_after_tx(Some((token, resp)))
            }
            _ => unreachable!(),
        }
    }

    fn retry_or_fail(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        if self.can_retry(context.pib.max_frame_retries) {
            self.do_retransmit(frame, context)
        } else {
            let token = self.take_tx_token();
            let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                frame,
                self.base_time,
            ));
            self.state = CsmaState::Idle;
            self.transition_with_response(context, token, resp)
        }
    }

    fn do_retransmit(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        self.prepare_retransmit(context.pib.min_be);
        self.base_time = context.timer.now();

        let at = Timestamp::Scheduled(self.backoff_time(context.rng));
        let fallback = self.can_retry(context.pib.max_frame_retries);
        let req = self.build_tx_request(frame, at, Some(self.channel), fallback);

        self.send_driver_request_and_wait(req)
    }

    fn complete_tx_success(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
    ) -> SchedulerTaskTransition {
        let token = self.take_tx_token();
        let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
            frame.forget_size::<RadioDriverImpl>(),
            instant,
        ));
        self.continue_after_tx(Some((token, resp)))
    }

    fn complete_tx_failure(
        &mut self,
        frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let token = self.take_tx_token();
        let resp = SchedulerResponse::Transmission(
            SchedulerTransmissionResult::ChannelAccessFailure(frame),
        );
        self.state = CsmaState::Idle;
        self.transition_with_response(context, token, resp)
    }
}

// ============================================================================
// Pipelining
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Pipeline the next operation after TX started.
    fn pipeline_next_operation(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let fallback = context.pib.max_frame_retries > 0;

        let request = if let Some((token, frame)) = self.next_tx_request(context) {
            self.pipelined_info = Some(PipelinedInfo::Tx(token));
            self.build_tx_request(frame, Timestamp::BestEffort, None, fallback)
        } else {
            let frame = self.rx_frame.take().expect("no rx frame");
            self.pipelined_info = Some(PipelinedInfo::Rx);
            self.build_rx_request(frame, None)
        };

        self.state = CsmaState::WaitingForTxResult;
        self.send_driver_request_and_wait(request)
    }

    /// Continue with next pipelined operation after TX completes.
    fn continue_after_tx(
        &mut self,
        response: Option<(ResponseToken, SchedulerResponse)>,
    ) -> SchedulerTaskTransition {
        match self.pipelined_info.take() {
            Some(PipelinedInfo::Tx(token)) => {
                self.tx_token = Some(token);
                self.tx_retries = 0;
                self.state = CsmaState::Cca;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, response)
            }
            _ => {
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    response,
                )
            }
        }
    }
}

// ============================================================================
// RX Operations
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn start_rx(&mut self) -> SchedulerTaskTransition {
        let frame = match self.rx_frame.take() {
            Some(f) => f,
            None => {
                self.state = CsmaState::Idle;
                return SchedulerTaskTransition::Execute(
                    SchedulerAction::WaitForSchedulerRequest,
                    None,
                );
            }
        };

        self.state = CsmaState::Listening;
        let req = self.build_rx_request(frame, Some(self.channel));
        self.send_driver_request_and_select(req)
    }

    fn on_listening(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::FrameStarted => {
                // Frame arriving - pipeline next RX
                let frame = self
                    .rx_frame
                    .take()
                    .unwrap_or_else(|| context.allocate_frame());
                self.state = CsmaState::Receiving;
                let req = self.build_rx_request(frame, None);
                self.send_driver_request_and_wait(req)
            }
            DrvSvcEvent::RxWindowEnded(frame) => {
                self.rx_frame = Some(frame);
                self.state = CsmaState::Idle;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            _ => unreachable!(),
        }
    }

    fn on_receiving(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::Received(frame, instant) => {
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
            DrvSvcEvent::CrcError(frame, instant) => {
                self.base_time = instant;
                self.rx_frame = Some(frame);
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// Commands
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    fn on_command(
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
                self.transition_with_response(context, token, resp)
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

    #[cfg(feature = "tsch")]
    fn on_terminating(
        &mut self,
        event: DrvSvcEvent,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::RxWindowEnded(radio_frame) => {
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
// Transition Helpers
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Send driver request and wait for driver event.
    #[inline]
    fn send_driver_request_and_wait(&self, req: DrvSvcRequest) -> SchedulerTaskTransition {
        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(req), None)
    }

    /// Send driver request and select on driver event OR scheduler request.
    #[inline]
    fn send_driver_request_and_select(&self, req: DrvSvcRequest) -> SchedulerTaskTransition {
        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenSelect(req), None)
    }

    /// Transition with response after state change.
    fn transition_with_response(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        token: ResponseToken,
        resp: SchedulerResponse,
    ) -> SchedulerTaskTransition {
        let transition = if let Some((tx_token, frame)) = self.next_tx_request(context) {
            self.start_tx(tx_token, frame, context)
        } else {
            self.start_rx()
        };
        match transition {
            SchedulerTaskTransition::Execute(action, _) => {
                SchedulerTaskTransition::Execute(action, Some((token, resp)))
            }
            other => other,
        }
    }

    /// Retrieves the next TX request to process, if any.
    fn next_tx_request(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> Option<(ResponseToken, RadioFrame<RadioFrameSized>)> {
        // Priorities:
        // 1. Pending TX from NACK recovery
        // 2. Tx request from channel
        if let Some(pending) = self.pending_tx.take() {
            Some(pending)
        } else if let Some((tx_token, mpdu)) = context.try_receive_tx_request() {
            Some((tx_token, mpdu.into_radio_frame::<RadioDriverImpl>()))
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
        frame: RadioFrame<RadioFrameSized>,
        at: Timestamp,
        channel: Option<config::Channel>,
        fallback: bool,
    ) -> DrvSvcRequest {
        DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
            at,
            radio_frame: frame,
            cca: true,
            channel,
            fallback_on_nack: fallback,
        })
    }

    #[inline]
    fn build_rx_request(
        &self,
        frame: RadioFrame<RadioFrameUnsized>,
        channel: Option<config::Channel>,
    ) -> DrvSvcRequest {
        DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
            start: Timestamp::BestEffort,
            radio_frame: frame,
            channel,
        })
    }
}
