//! TSCH scheduler state machine logic.
//!
//! This module implements the state transition logic for the TSCH scheduler.
//! Operations are queued and executed at their scheduled times.
//! The scheduler wakes up slightly before each timeslot (guard time)
//! to prepare for the operation.

use core::mem;

use dot15d4_driver::{
    radio::{
        frame::{RadioFrame, RadioFrameSized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;

#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::TschCommand::UseTsch;
use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    scheduler::{SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition},
};
use crate::{
    scheduler::{
        command::SchedulerCommand, SchedulerAction, SchedulerContext, SchedulerRequest,
        SchedulerResponse, SchedulerTransmissionResult,
    },
    utils,
};

use super::task::{TschCsmaBackoff, TschOperation, TschState, TschTask};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for TschTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match mem::replace(&mut self.state, TschState::Placeholder) {
            TschState::Idle => self.execute_idle(event, context),
            TschState::WaitingForTxStart {
                response_token,
                shared,
                backoff,
            } => self.execute_waiting_for_tx_start(response_token, shared, backoff, event, context),
            TschState::Transmitting {
                response_token,
                shared,
                backoff,
            } => self.execute_transmitting(response_token, shared, backoff, event, context),
            TschState::Listening => self.execute_listening(event, context),
            TschState::Receiving => self.execute_receiving(event, context),
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// State Execution Methods
// ============================================================================

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Handle Idle state - waiting for timeslot deadline or requests.
    ///
    /// In idle state, the scheduler waits for:
    /// - Timer expiration to execute the next scheduled operation
    /// - New TX requests from the MAC layer
    /// - Commands to configure TSCH parameters
    fn execute_idle(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::Entry => {
                // Sort links by (slotframe_handle, timeslot)
                // so that link selection can rely on iteration
                // order for priority rules.
                context.pib.tsch.sort_links();

                if context.pib.coord_extended_address.is_broadcast() {
                    self.init_coordinator(context, Some(9));
                } else {
                    self.init_device(context);
                }
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            // Incoming Scheduler Tx/Command Request from upper layer
            SchedulerTaskEvent::SchedulerRequest { token, request } => match request {
                SchedulerRequest::Transmission(mpdu) => {
                    self.on_scheduler_tx_request(token, mpdu, context)
                }
                SchedulerRequest::Command(command) => {
                    self.on_scheduler_command(token, command, context)
                }
                SchedulerRequest::Reception(_) => {
                    unreachable!()
                }
            },
            // Timer expired: time to execute next operation in the upcoming timeslot
            SchedulerTaskEvent::TimerExpired => match self.pending_operations.last() {
                Some(TschOperation::TxSlot { .. }) => self.schedule_tx_operation(context),
                Some(TschOperation::RxSlot { .. }) => self.schedule_rx_operation(context),
                Some(TschOperation::AdvertisementSlot { .. }) => {
                    self.schedule_advertisement_operation(context)
                }
                _ => unreachable!(),
            },
            _ => unreachable!(),
        }
    }

    /// Handle WaitingForTxStart state - CCA/TX preparation in progress.
    fn execute_waiting_for_tx_start(
        &mut self,
        response_token: Option<ResponseToken>,
        shared: bool,
        backoff: TschCsmaBackoff,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::TxStarted(_instant)) => {
                self.state = TschState::Transmitting {
                    response_token,
                    shared,
                    backoff,
                };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CcaBusy(radio_frame, _instant)) => {
                // CCA reported the channel busy before the frame went out.
                self.handle_tx_start_failure(radio_frame, shared, backoff, response_token, context)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::SchedulingFailed(radio_frame)) => {
                // The driver could not schedule the transmission (timer rejection,
                // deadline in the past, or guard-time overrun). The frame never
                // went out, so handle it like a CCA-busy failure instead of
                // aborting.
                self.handle_tx_start_failure(radio_frame, shared, backoff, response_token, context)
            }
            // WaitingForTxStart legitimately only sees the events above; recover
            // to idle rather than aborting.
            _ => {
                dot15d4_util::warn!("TSCH: unexpected event in WaitingForTxStart; recovering to idle.");
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
        }
    }

    /// Handle a pre-transmission failure in `WaitingForTxStart`: CCA reported the
    /// channel busy, or the driver could not schedule the TX. Both mean the frame
    /// never went out, so they are handled identically: a shared link applies the
    /// CSMA-CA backoff, a dedicated link reports channel-access failure to the
    /// upper layer, and a beacon restores the frame and reschedules the
    /// advertisement.
    fn handle_tx_start_failure(
        &mut self,
        radio_frame: RadioFrame<RadioFrameSized>,
        shared: bool,
        backoff: TschCsmaBackoff,
        response_token: Option<ResponseToken>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match response_token {
            // A frame that never went out (CCA busy or scheduling failure)
            // retries on both shared and dedicated links, up to
            // macMaxFrameRetries; only the backoff delay and BE increase are
            // shared-specific. The dedicated and early-attempt cases add no delay
            // and leave BE unchanged.
            Some(response_token) => self.handle_tx_failure(
                radio_frame,
                backoff,
                response_token,
                None,
                shared,
                context,
            ),
            // Beacon: restore the frame and reschedule the advertisement.
            None => {
                self.recover_beacon(radio_frame, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
        }
    }

    /// Handle Transmitting state - frame transmission in progress.
    fn execute_transmitting(
        &mut self,
        response_token: Option<ResponseToken>,
        shared: bool,
        backoff: TschCsmaBackoff,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Sent(tx_frame, instant, _timesync_us)) => {
                if let Some(response_token) = response_token {
                    let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                        tx_frame.forget_size::<RadioDriverImpl>(),
                        instant,
                    ));
                    // Acknowledgment-based synchronization
                    #[cfg(not(feature = "no-tsch-ack-sync"))]
                    if let Some(timesync_us) = _timesync_us {
                        context.pib.tsch.sync_ack(timesync_us);
                    }
                    let action = self.go_idle(context);
                    SchedulerTaskTransition::Execute(action, Some((response_token, resp)))
                } else {
                    // Record the beacon transmission time (used for the next
                    // period calculation), then put the frame back and
                    // schedule the next advertisement.
                    self.on_beacon_sent(instant);
                    self.recover_beacon(tx_frame, context);
                    SchedulerTaskTransition::Execute(self.go_idle(context), None)
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Nack(
                tx_frame,
                instant,
                _request,
                _timesync_us,
            )) => {
                match response_token {
                    // Both shared and dedicated links retransmit on a missing
                    // ACK, up to macMaxFrameRetries; the dedicated path adds no
                    // delay and does not bump BE.
                    Some(response_token) => self.handle_tx_failure(
                        tx_frame,
                        backoff,
                        response_token,
                        Some(instant),
                        shared,
                        context,
                    ),
                    // Beacon: restore the frame and reschedule the
                    // advertisement.
                    None => {
                        self.recover_beacon(tx_frame, context);
                        SchedulerTaskTransition::Execute(self.go_idle(context), None)
                    }
                }
            }
            _ => unreachable!(),
        }
    }

    fn handle_tx_failure(
        &mut self,
        tx_frame: RadioFrame<RadioFrameSized>,
        backoff: TschCsmaBackoff,
        response_token: ResponseToken,
        noack_instant: Option<NsInstant>,
        failed_on_shared: bool,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let is_cca_busy = noack_instant.is_none();
        let nb = backoff.nb.saturating_add(1);

        if nb > context.pib.max_frame_retries {
            let result = match noack_instant {
                Some(instant) => SchedulerTransmissionResult::NoAck(tx_frame, instant),
                None => SchedulerTransmissionResult::ChannelAccessFailure(tx_frame),
            };
            let action = self.go_idle(context);
            return SchedulerTaskTransition::Execute(
                action,
                Some((response_token, SchedulerResponse::Transmission(result))),
            );
        }

        let (be, backoff_remaining) = if failed_on_shared && !(is_cca_busy && backoff.nb == 0) {
            // Shared-link retransmission: the delay window comes from the current
            // BE, which is advanced afterwards for the next round.
            let window = 1u32 << backoff.be.min(31);
            let backoff_remaining = (context.rng.next_u32() % window) as u16;
            let be = backoff
                .be
                .saturating_add(1)
                .min(context.pib.tsch.tsch_max_be);
            (be, backoff_remaining)
        } else {
            // First channel-access failure, or a dedicated-link failure: no wait,
            // BE unchanged.
            (backoff.be, 0)
        };

        let backoff = TschCsmaBackoff {
            be,
            nb,
            backoff_remaining,
        };
        let mpdu = MpduFrame::from_radio_frame(tx_frame);
        let current_asn = context.pib.tsch.asn_at(context.timer.now());
        self.queue_tx_with_backoff(response_token, mpdu, current_asn, backoff, context);

        SchedulerTaskTransition::Execute(self.go_idle(context), None)
    }

    /// Handle Listening state - RX window active, waiting for frames.
    fn execute_listening(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::FrameStarted) => {
                // The slot is not complete yet; do not queue the next RX here
                // (that would over-queue duplicate ops). Queue it once the slot
                // actually completes (RX window end or terminal RX event).
                self.state = TschState::Receiving;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(rx_frame)) => {
                // No frame received in this slot: the slot is complete, so queue
                // the next RX exactly once here.
                self.put_rx_frame(rx_frame);
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            // Listening legitimately only sees the two events above. Recover to
            // idle rather than aborting.
            _ => {
                dot15d4_util::warn!("TSCH: unexpected event in Listening; recovering to idle.");
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
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
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Received(rx_frame, rx_instant)) => {
                // Slot complete: queue the next RX exactly once.
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                let action = self.go_idle(context);

                // Convert to an MPDU once; the sync decision and the frame
                // routing below both work on this view.
                let mpdu = MpduFrame::from_radio_frame(rx_frame);

                // Frame-based synchronization
                #[cfg(not(feature = "no-tsch-frame-sync"))]
                if !self.is_coordinator {
                    let asn = context.pib.tsch.asn;
                    // Only steer the local clock from frames received on a
                    // time-keeping link (a designated time-source neighbor),
                    // not from arbitrary RX links.
                    if context.pib.tsch.rx_link_is_time_keeping(asn) {
                        context.pib.tsch.sync_asn(asn, rx_instant);
                    }
                }

                match utils::process_rx_frame(mpdu, rx_instant, context) {
                    Ok((response, token)) => {
                        // Allocate new frame for next RX
                        self.put_rx_frame(context.allocate_frame());

                        SchedulerTaskTransition::Execute(action, Some((token, response)))
                    }
                    Err(recovered_mpdu) => {
                        // No receiver, reuse frame
                        self.put_rx_frame(
                            recovered_mpdu
                                .into_radio_frame::<RadioDriverImpl>()
                                .forget_size::<RadioDriverImpl>(),
                        );
                        SchedulerTaskTransition::Execute(action, None)
                    }
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CrcError(rx_frame, _instant)) => {
                // Slot complete: queue the next RX exactly once.
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                self.put_rx_frame(rx_frame);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::ReceptionAborted) => {
                // The reception terminated without a frame (Enh-ACK CRC rollback
                // or a frame starting as the window closed). The driver kept the
                // RX buffer and resumed listening, so the slot is not complete:
                // go back to Listening and wait for the next driver event (a new
                // FrameStarted or the eventual RxWindowEnded) instead of aborting.
                // The next RX is queued when the slot completes.
                self.state = TschState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            // Receiving legitimately only sees the events above. Recover to idle
            // rather than aborting.
            _ => {
                dot15d4_util::warn!("TSCH: unexpected event in Receiving; recovering to idle.");
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
        }
    }
}

// ============================================================================
// Scheduler Requests handling
// ============================================================================

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Handle a TX request from the MAC layer.
    ///
    /// Finds an appropriate link based on frame type and schedules
    /// the transmission for the next occurrence of that link.
    ///
    /// Transform that request into a TSCH operation that is inserted
    /// in the pending operations queue.
    fn on_scheduler_tx_request(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let current_time = context.timer.now();
        let current_asn = context.pib.tsch.asn_at(current_time);

        // When no suitable link exists, queue_tx fails the request itself
        // (ChannelAccessFailure), so the task simply returns to idle.
        self.queue_tx(token, mpdu, current_asn, context);
        SchedulerTaskTransition::Execute(self.go_idle(context), None)
    }

    /// Handle a command request (mode switching, configuration).
    fn on_scheduler_command(
        &mut self,
        token: ResponseToken,
        command: SchedulerCommand,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        use crate::scheduler::command::*;

        self.state = TschState::Idle;

        match command {
            #[cfg(feature = "tsch")]
            SchedulerCommand::TschCommand(tsch_cmd) => match tsch_cmd {
                UseTsch(enabled, cca) => {
                    use crate::scheduler::{
                        command::tsch::TschCommandResult::UseTsch, SchedulerTaskCompletion,
                    };

                    // Keep macTschCca in sync (the CSMA-side handler is the one
                    // that takes effect on entry, but mirror it here too).
                    context.pib.tsch.tsch_cca = cca;

                    let result = if enabled {
                        tsch::UseTschCommandResult::StartedTsch
                    } else {
                        self.terminate(context);
                        tsch::UseTschCommandResult::StoppedTsch
                    };

                    let response = SchedulerResponse::Command(SchedulerCommandResult::TschCommand(
                        UseTsch(result),
                    ));

                    SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((token, response)),
                    )
                }
                tsch::TschCommand::SetTschSlotframe(_) => {
                    // Slotframe commands are handled in CSMA mode before switching
                    todo!()
                }
                tsch::TschCommand::SetTschLink(_) => {
                    // Link commands are handled in CSMA mode before switching
                    todo!()
                }
            },
            SchedulerCommand::CsmaCommand(_) => {
                // CSMA commands not handled in TSCH mode
                todo!()
            }
            SchedulerCommand::PibCommand(cmd) => {
                let response = context.process_pib_command(cmd);
                let deadline = self.peek_deadline(context);
                SchedulerTaskTransition::Execute(
                    SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline },
                    Some((token, response)),
                )
            }
            SchedulerCommand::ScanCommand(_scan_command) => todo!(),
        }
    }
}

// ============================================================================
// TSCH Operation Scheduling
// ============================================================================

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Schedule a TX operation for the current timeslot.
    ///
    /// Pops the operation from the queue (which is assumed to be of TX type)
    /// and submits the driver request with precise timing.
    fn schedule_tx_operation(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::TxSlot {
                mpdu,
                asn,
                channel,
                cca,
                shared,
                backoff,
                response_token,
                ..
            } => {
                self.state = TschState::WaitingForTxStart {
                    response_token: Some(response_token),
                    shared,
                    backoff,
                };

                context.pib.tsch.update_timing(asn);

                // Update TSCH Synchronization IE with current ASN and join metric
                let join_metric = context.pib.tsch.join_metric as u8;
                let mpdu =
                    super::task::update_tsch_sync_ie::<RadioDriverImpl>(mpdu, asn, join_metric);

                // Calculate TX start time: timeslot start + macTsTxOffset
                // TODO: from method, remove inline calculation
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let tx_instant = context.pib.tsch.last_base_time + NsDuration::micros(tx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                    at: Timestamp::Scheduled(tx_instant),
                    mpdu,
                    cca,
                    channel: Some(channel),
                    fallback_on_nack: false,
                });

                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(request),
                    None,
                )
            }
            _ => unreachable!(),
        }
    }

    /// Schedule an RX operation for the current timeslot.
    ///
    /// Pops the operation (which is assumed to be of TX type) from the queue
    /// and submits the driver request with precise timing.
    fn schedule_rx_operation(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::RxSlot { asn, channel, .. } => {
                context.pib.tsch.update_timing(asn);
                let frame = self
                    .take_rx_frame()
                    .unwrap_or_else(|| context.allocate_frame());

                self.state = TschState::Listening;

                // Calculate RX start time: timeslot start + macTsRxOffset
                let rx_offset_us = context.pib.tsch.timeslot_timings.rx_offset() as u64;
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let rx_wait_us = context.pib.tsch.timeslot_timings.rx_wait() as u64;
                let rx_instant = context.pib.tsch.last_base_time + NsDuration::micros(rx_offset_us);
                let max_rx_start = rx_instant + NsDuration::micros(rx_wait_us);
                let expected_rx_framestart =
                    context.pib.tsch.last_base_time + NsDuration::micros(tx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                    start: Timestamp::Scheduled(rx_instant),
                    radio_frame: frame,
                    channel: Some(channel),
                    rx_window_end: Some(max_rx_start),
                    expected_rx_framestart: Some(expected_rx_framestart),
                });

                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(request),
                    None,
                )
            }
            _ => unreachable!(),
        }
    }

    /// Schedule an advertisement (beacon) operation for the current timeslot.
    ///
    /// Updates the beacon frame with current ASN and schedules transmission.
    fn schedule_advertisement_operation(
        &mut self,
        // TODO: pass tsch context
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::AdvertisementSlot { asn, channel, .. } => {
                context.pib.tsch.update_timing(asn);
                // Take beacon frame and update ASN
                let beacon_frame = self
                    .beacon_mpdu
                    .take()
                    .expect("no beacon frame for advertisement");

                let join_metric = context.pib.tsch.join_metric as u8;
                let updated_frame = super::task::update_tsch_sync_ie::<RadioDriverImpl>(
                    beacon_frame,
                    asn,
                    join_metric,
                );

                self.state = TschState::WaitingForTxStart {
                    response_token: None,
                    shared: false,
                    backoff: TschCsmaBackoff::new(0),
                };

                // Calculate TX start time: timeslot start + macTsTxOffset
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let tx_instant = context.pib.tsch.last_base_time + NsDuration::micros(tx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                    at: Timestamp::Scheduled(tx_instant),
                    mpdu: updated_frame,
                    cca: context.pib.tsch.tsch_cca,
                    channel: Some(channel),
                    fallback_on_nack: false,
                });

                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(request),
                    None,
                )
            }
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// Helper Methods
// ============================================================================

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Put the beacon frame back after a transmission attempt (sent, CCA busy,
    /// scheduling failure or missing ACK) and schedule the next advertisement.
    ///
    /// This is the single recovery path for the beacon buffer, shared by all
    /// beacon outcome branches.
    fn recover_beacon(
        &mut self,
        radio_frame: RadioFrame<RadioFrameSized>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) {
        self.beacon_mpdu
            .set(Some(MpduFrame::from_radio_frame(radio_frame)));
        self.queue_next_beacon(context);
    }

    /// Transition to idle state and wait for the next deadline by configuring
    /// timer action accordingly.
    fn go_idle(&mut self, context: &SchedulerContext<RadioDriverImpl>) -> SchedulerAction {
        let deadline = self.peek_deadline(context);
        self.state = TschState::Idle;

        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline }
    }
}
