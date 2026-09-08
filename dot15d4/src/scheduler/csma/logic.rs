//! CSMA-CA scheduler state machine logic.
//!
//! This module implements the state transition logic for the CSMA-CA scheduler.

use dot15d4_driver::{
    radio::{
        config,
        frame::{RadioFrame, RadioFrameSized, RadioFrameUnsized},
        phy::PhyConfig,
        DriverConfig, PhyOf,
    },
    timer::{NsDuration, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};

#[cfg(feature = "tsch")]
use crate::scheduler::command::scan::ScanCommand;
use crate::scheduler::{
    command::pib::PibCommand, SchedulerCommandResult, SchedulerContext, SchedulerRequest,
    SchedulerResponse, SchedulerTransmissionResult,
};
#[cfg(feature = "tsch")]
use crate::scheduler::{command::tsch::UseTschCommandResult, SchedulerTaskCompletion};
#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::{TschCommand, TschCommandResult};
use crate::{
    driver::{AckWindow, DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    scheduler::{SchedulerAction, SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition},
    utils,
};

use super::task::{CsmaState, CsmaTask, Pipelined};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for CsmaTask<RadioDriverImpl> {
    /// Main state machine entry point - dispatches to state-specific handlers.
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
            CsmaState::Terminating(completion) => {
                self.execute_terminating(event, context, completion)
            }
        }
    }
}

// ============================================================================
// State Execution Methods
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Handle Idle state - decide next action (TX or RX).
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

    /// Handle Listening state - waiting for frames or TX requests.
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
            // Any other driver event while merely listening is unexpected; keep
            // selecting rather than aborting.
            _ => {
                dot15d4_util::warn!("CSMA: unexpected event in Listening; continuing to select.");
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
        }
    }

    /// Handle Receiving state - frame reception in progress.
    fn execute_receiving(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Received(rx_frame, instant)) => {
                self.base_time = instant;
                self.state = CsmaState::Listening;

                let mpdu = MpduFrame::from_radio_frame(rx_frame);

                // Neighbor statistics: attribute the frame to its source
                // neighbor.
                if let Ok(parsed) = mpdu.reader().parse_addressing() {
                    if let Ok(addressing) = parsed.try_addressing_fields() {
                        let src_id = addressing
                            .try_src_address()
                            .and_then(|src| context.pib.neighbors.intern(&src, instant));
                        if let Some(id) = src_id {
                            context.pib.neighbors.note_rx(id, instant);
                        }
                    }
                }

                match utils::process_rx_frame(mpdu, instant, context) {
                    Ok((response, token)) => {
                        self.rx_frame = Some(context.allocate_frame());
                        SchedulerTaskTransition::Execute(
                            SchedulerAction::SelectDriverEventOrRequest,
                            Some((token, response)),
                        )
                    }
                    Err(recovered_mpdu) => {
                        self.rx_frame = Some(
                            recovered_mpdu
                                .into_radio_frame::<RadioDriverImpl>()
                                .forget_size::<RadioDriverImpl>(),
                        );
                        SchedulerTaskTransition::Execute(
                            SchedulerAction::SelectDriverEventOrRequest,
                            None,
                        )
                    }
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CrcError(frame, instant)) => {
                self.base_time = instant;
                self.rx_frame = Some(frame);
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::ReceptionAborted) => {
                // The reception terminated without a frame (Enh-ACK CRC rollback
                // or a frame starting as the window closed). The driver kept the
                // RX buffer and resumed listening, so we just return to Listening
                // and keep selecting, instead of aborting.
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
            // Receiving legitimately only sees the events above; recover to
            // listening rather than aborting.
            _ => {
                dot15d4_util::warn!("CSMA: unexpected event in Receiving; recovering to listening.");
                self.state = CsmaState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::SelectDriverEventOrRequest, None)
            }
        }
    }

    /// Handle WaitingForTxStart state - CCA in progress.
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
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::SchedulingFailed(frame)) => {
                // The driver could not schedule the transmission (timer rejection
                // / deadline in the past). Route it through the same backoff path
                // as a CCA-busy result so the frame is retried or surfaced as a
                // channel-access failure, instead of aborting.
                self.backoff_or_fail(frame, context)
            }
            // WaitingForTxStart legitimately only sees the events above; keep
            // waiting rather than aborting.
            _ => {
                dot15d4_util::warn!("CSMA: unexpected event in WaitingForTxStart; continuing to wait.");
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
        }
    }

    /// Handle Transmitting state - frame transmission in progress.
    fn execute_transmitting(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Sent(frame, instant, _)) => {
                self.base_time = instant;

                let token = self.take_tx_token();
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                    frame.forget_size::<RadioDriverImpl>(),
                    instant,
                ));
                self.continue_with_pipelined((token, resp), context)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Nack(frame, instant, recovered, _)) => {
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
                        self.continue_with_pipelined((token, resp), context)
                    }
                    // Any other recovered request kind is unexpected; surface the
                    // TX as failed (consuming the frame) rather than aborting.
                    _ => {
                        dot15d4_util::warn!(
                            "CSMA: unexpected recovered request on Nack; reporting TX failure."
                        );
                        let token = self.take_tx_token();
                        let resp = SchedulerResponse::Transmission(
                            SchedulerTransmissionResult::NoAck(frame, instant),
                        );
                        self.continue_with_pipelined((token, resp), context)
                    }
                }
            }
            // Transmitting legitimately only sees Sent / Nack; keep waiting for a
            // terminal TX event rather than aborting.
            _ => {
                dot15d4_util::warn!("CSMA: unexpected event in Transmitting; continuing to wait.");
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
        }
    }

    /// Handle Terminating state - shutting down CSMA to switch modes.
    #[cfg(feature = "tsch")]
    fn execute_terminating(
        &mut self,
        event: SchedulerTaskEvent,
        context: &SchedulerContext<RadioDriverImpl>,
        completion: SchedulerTaskCompletion,
    ) -> SchedulerTaskTransition {
        use crate::scheduler::command::scan::ScanCommandResult;

        let recovered_frame = match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(radio_frame)) => {
                Some(radio_frame)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CrcError(radio_frame, _)) => {
                Some(radio_frame)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Received(radio_frame, _)) => {
                Some(radio_frame.forget_size::<RadioDriverImpl>())
            }
            // The radio kept the rx buffer, so there is nothing to give back.
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::ReceptionAborted) => None,
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::FrameStarted) => {
                return SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None);
            }
            // Anything else leaves the mode switch unresolved. Keep waiting for
            // the driver to go idle rather than aborting: completing here would
            // hand the token back before the radio is actually free.
            _ => {
                dot15d4_util::warn!(
                    "CSMA: unexpected event in Terminating; still waiting for the driver to go idle."
                );
                return SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None);
            }
        };

        if let Some(radio_frame) = recovered_frame {
            unsafe {
                context
                    .buffer_allocator
                    .deallocate_buffer(radio_frame.into_buffer());
            }
        }

        let response = match &completion {
            SchedulerTaskCompletion::SwitchToCsma => {
                unreachable!()
            }
            SchedulerTaskCompletion::SwitchToTsch => {
                SchedulerResponse::Command(SchedulerCommandResult::TschCommand(
                    TschCommandResult::UseTsch(UseTschCommandResult::StartedTsch),
                ))
            }
            SchedulerTaskCompletion::SwitchToScanning(_, _) => SchedulerResponse::Command(
                SchedulerCommandResult::ScanCommand(ScanCommandResult::StartedScanning),
            ),
        };
        SchedulerTaskTransition::Completed(
            completion,
            Some((self.take_tx_token(), response)),
        )
    }
}

// ============================================================================
// TX Event Handlers (retransmission and backoff)
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Handle NACK - attempt retransmission or report failure.
    ///
    /// If retries remain, schedules another transmission attempt with reset backoff.
    /// Otherwise, reports No Ack failure and transitions to next operation.
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

    /// Handle CCA busy - attempt another backoff or report channel access failure.
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

    /// Pipeline the next operation while transmission is in progress.
    ///
    /// Called after TxStarted to prepare the next operation (TX or RX)
    /// to minimize idle time between operations.
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
// Command Handlers
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Handle incoming scheduler commands.
    ///
    /// Dispatches to specific command handlers for CSMA, PIB, and TSCH commands.
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
                self.on_tsch_cmd(token, cmd, context)
            }
            #[cfg(feature = "tsch")]
            SchedulerCommand::ScanCommand(cmd) => self.on_scan_command(token, cmd),
        }
    }

    /// Handle PIB attribute commands.
    ///
    /// Delegates to the centralised handler on [`SchedulerContext`].
    fn on_pib_cmd(
        &mut self,
        token: ResponseToken,
        cmd: PibCommand,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let response = context.process_pib_command(cmd);
        SchedulerTaskTransition::Execute(
            SchedulerAction::SelectDriverEventOrRequest,
            Some((token, response)),
        )
    }

    #[cfg(feature = "tsch")]
    pub(crate) fn on_scan_command(
        &mut self,
        token: ResponseToken,
        cmd: ScanCommand,
    ) -> crate::scheduler::SchedulerTaskTransition {
        match cmd {
            ScanCommand::StartScanning(channels, max_pan_descriptors) => {
                self.state = CsmaState::Terminating(SchedulerTaskCompletion::SwitchToScanning(
                    channels,
                    max_pan_descriptors,
                ));
                self.tx_token = Some(token);
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(DrvSvcRequest::CompleteThenGoIdle),
                    None,
                )
            }
            ScanCommand::StopScanning => {
                unreachable!()
            }
        }
    }

    /// Handle TSCH commands while in CSMA mode.
    ///
    /// Handles UseTsch to switch modes, and slotframe/link configuration
    /// commands that can be processed while still in CSMA mode.
    #[cfg(feature = "tsch")]
    fn on_tsch_cmd(
        &mut self,
        token: ResponseToken,
        cmd: TschCommand,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        use crate::{
            mac::mlme::tsch::TschScheduleOperation,
            scheduler::{
                command::{tsch::*, SchedulerCommandResult},
                tsch::pib::{ScheduleError, TschLink},
            },
        };

        let tsch = &mut context.pib.tsch;
        match cmd {
            TschCommand::UseTsch { enabled, cca, role } => {
                // Record macTschCca and the session role (MLME-TSCH-MODE
                // .request). This handler runs before the mode switch, so it
                // is the one that takes effect for the upcoming TSCH session:
                // TSCH entry reads the role from the PIB.
                tsch.tsch_cca = cca;
                tsch.role = role;
                if enabled {
                    self.state = CsmaState::Terminating(SchedulerTaskCompletion::SwitchToTsch);
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
                // Resolve the neighbor address to a table handle. A link
                // neighbor is a long-lived reference, so pin the entry.
                let neighbor = cmd.neighbor.and_then(|ext_addr| {
                    let now = context.timer.now();
                    let address = dot15d4_driver::radio::frame::Address::Extended(
                        dot15d4_driver::radio::frame::ExtendedAddress::new(&ext_addr[..]),
                    );
                    let id = context.pib.neighbors.intern(&address, now);
                    if let Some(id) = id {
                        context.pib.neighbors.pin(id);
                    } else {
                        dot15d4_util::warn!(
                            "SetTschLink: neighbor table full, link stored without neighbor"
                        );
                    }
                    id
                });
                let tsch = &mut context.pib.tsch;
                let link = TschLink {
                    slotframe_handle: cmd.slotframe_handle,
                    timeslot: cmd.timeslot,
                    channel_offset: cmd.channel_offset,
                    link_options: cmd.link_options,
                    link_type: cmd.link_type,
                    neighbor,
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
    /// Helper to create a "send request then wait" transition.
    #[inline]
    fn send_driver_request_and_wait(&self, req: DrvSvcRequest) -> SchedulerTaskTransition {
        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(req), None)
    }

    /// Determine the next action based on pending operations.
    ///
    /// Priority order: pending TX > channel TX request > start RX
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

    /// Schedule a new transmission.
    ///
    /// Initializes backoff state and submits TX request to driver.
    fn schedule_tx(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerAction {
        self.state = CsmaState::WaitingForTxStart;

        // Stamp macDsn on the fresh request. Retransmissions re-use the frame
        // as it stands and therefore keep this sequence number.
        let mut mpdu = mpdu;
        let _ = mpdu.try_set_sequence_number(context.pib.next_dsn());

        // macPanId is MAC state, not something the upper layer passes down, so
        // it is stamped here as well. The call is a no-op for frames whose
        // addressing configuration elides the Destination PAN ID.
        let _ = mpdu.try_set_dst_pan_id(&context.pib.pan_id);

        self.base_time = context.timer.now();
        self.backoff.reset(context.pib.min_be);
        self.tx_token = Some(token);
        self.tx_retries = 0;

        // The random(2^BE-1) unit-backoff delay precedes every CCA, including
        // the first. `base_time` was just set to `now()`, so schedule the first
        // attempt at now + backoff.
        let at = Timestamp::Scheduled(self.backoff_time(context.rng));
        let fallback = self.tx_retries < context.pib.max_frame_retries;
        let req = self.build_tx_request(mpdu, at, Some(self.channel), fallback);

        SchedulerAction::SendDriverRequestThenWait(req)
    }

    /// Continue with the next pipelined operation after TX completes.
    ///
    /// If a TX was pipelined, wait for its completion.
    /// Otherwise, transition to listening state.
    fn continue_with_pipelined(
        &mut self,
        response: (ResponseToken, SchedulerResponse),
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pipelined.take() {
            Some(Pipelined::Tx(token)) => {
                self.state = CsmaState::WaitingForTxStart;
                self.tx_token = Some(token);
                self.tx_retries = 0;
                // The pipelined frame is a new transmission attempt and must
                // start the CSMA-CA algorithm from scratch (NB=0, BE=macMinBe);
                // inheriting the previous frame's backoff would let one busy CCA
                // trigger a spurious CHANNEL_ACCESS_FAILURE with inflated BE.
                self.backoff.reset(context.pib.min_be);
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

    /// Get the next TX request to process, if any.
    ///
    /// Checks pending TX (from NACK recovery) first, then channel requests.
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
// Driver Request Builders
// ============================================================================

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    const ACK_RX_GUARD: NsDuration = NsDuration::micros(32);

    const ACK_WINDOW: AckWindow = AckWindow {
        delay: <PhyOf<RadioDriverImpl> as PhyConfig>::AIFS
            .checked_sub(Self::ACK_RX_GUARD)
            .unwrap(),
        width: <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_UNIT_BACKOFF_PERIOD
            .checked_add(<PhyOf<RadioDriverImpl> as PhyConfig>::RMARKER_OFFSET)
            .unwrap()
            .checked_add(Self::ACK_RX_GUARD)
            .unwrap(),
    };

    /// Build a TX request for the driver service.
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
            ack_window: Self::ACK_WINDOW,
        })
    }

    /// Build an RX request for the driver service.
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
            rx_window_end: None,
            expected_rx_framestart: None,
            ack_delay: <PhyOf<RadioDriverImpl> as PhyConfig>::AIFS,
        })
    }
}
