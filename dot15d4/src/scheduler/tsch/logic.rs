//! TSCH scheduler logic implementation.

use core::mem;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{FrameType, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;

#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::TschCommand::UseTsch;
use crate::scheduler::task::{SchedulerTaskEvent, SchedulerTaskTransition};
use crate::scheduler::{
    action::SchedulerAction, command::SchedulerCommand, SchedulerContext, SchedulerRequest,
    SchedulerResponse, SchedulerTransmissionResult,
};
use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    scheduler::task::SchedulerTask,
};

use super::task::{TschOperation, TschState, TschTask, INFINITE_DEADLINE};
#[cfg(feature = "tsch-coordinator")]
use super::TschAsn;

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for TschTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::Entry => self.on_entry(context),
            SchedulerTaskEvent::DriverEvent(event) => self.on_driver_event(event, context),
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.on_scheduler_request(token, request, context)
            }
            SchedulerTaskEvent::TimerExpired => self.on_timer_expired_event(context),
        }
    }
}

// ============================================================================
// Entry & Dispatch
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    pub fn on_entry(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        #[cfg(feature = "tsch-coordinator")]
        {
            let current_time = context.timer.now();
            self.init_coordinator(context, current_time, Some(3));
            self.schedule_beacon(context);
        }
        assert!(matches!(
            self.state,
            TschState::WaitingForDeadlineOrRequest { .. }
        ));
        SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
    }

    /// Handle a scheduler request.
    pub fn on_scheduler_request(
        &mut self,
        token: ResponseToken,
        request: SchedulerRequest,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match request {
            SchedulerRequest::Transmission(mpdu) => {
                self.on_scheduler_tx_request(token, mpdu, context)
            }
            SchedulerRequest::Command(command) => self.on_scheduler_command(token, command),
            SchedulerRequest::Reception => todo!(),
        }
    }

    /// Handle a driver event.
    pub fn on_driver_event(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::CcaBusy(tx_frame, instant) => self.on_cca_busy_event(tx_frame, instant),
            DrvSvcEvent::TxStarted(instant) => self.on_tx_started_event(instant),
            DrvSvcEvent::Sent(tx_frame, instant) => self.on_sent_event(context, tx_frame, instant),
            DrvSvcEvent::Nack(tx_frame, instant, drv_svc_request) => {
                self.on_nack_event(context, tx_frame, instant, drv_svc_request)
            }
            DrvSvcEvent::FrameStarted => self.on_frame_started_event(),
            DrvSvcEvent::Received(rx_frame, instant) => {
                self.on_frame_received_event(context, rx_frame, instant)
            }
            DrvSvcEvent::RxWindowEnded(rx_frame) => {
                self.on_rx_window_ended_event(context, rx_frame)
            }
            DrvSvcEvent::CrcError(rx_frame, instant) => {
                self.on_crc_error_event(context, rx_frame, instant)
            }
            DrvSvcEvent::SchedulingFailed(_tx_frame) => todo!(),
        }
    }

    /// Handle timer expiry - time to execute next operation.
    pub fn on_timer_expired_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        assert!(matches!(
            self.state,
            TschState::WaitingForDeadlineOrRequest { .. }
        ));
        // peek next operation to execute in the upcoming timeslot
        match self.pending_operations.last() {
            Some(TschOperation::TxSlot { .. }) => self.schedule_tx_operation(context),
            Some(TschOperation::RxSlot { .. }) => self.schedule_rx_operation(context),
            #[cfg(feature = "tsch-coordinator")]
            Some(TschOperation::AdvertisementSlot { .. }) => {
                self.schedule_advertisement_operation(context)
            }
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// Driver Events handling
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    pub fn on_tx_started_event(&mut self, instant: NsInstant) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::WaitingForTxStartInTxSlot { response_token } => {
                self.state = TschState::TransmittingInTxSlot { response_token };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            TschState::WaitingForTxStartInAdvertisementSlot => {
                self.state = TschState::TransmittingInAdvertisementSlot;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            _ => unreachable!(),
        }
    }

    pub fn on_cca_busy_event(
        &mut self,
        tx_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        //TODO: support tsch csma/retransmission
        match state {
            TschState::WaitingForTxStartInTxSlot { response_token } => {
                todo!()
            }
            TschState::WaitingForTxStartInAdvertisementSlot => {
                todo!()
            }
            _ => unreachable!(),
        }
    }

    pub fn on_frame_started_event(&mut self) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::WaitingForFrameInRxSlot { response_token } => {
                self.state = TschState::ReceivingFrameInRxSlot { response_token };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            _ => unreachable!(),
        }
    }

    pub fn on_frame_received_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        rx_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::ReceivingFrameInRxSlot { response_token } => {
                let action = self.wait_for_timeout_or_request(context);

                if let Some(token) = response_token {
                    // Allocate new frame for next RX
                    self.put_rx_frame(context.allocate_frame());

                    let response = SchedulerResponse::Reception(rx_frame, instant);
                    SchedulerTaskTransition::Execute(action, Some((token, response)))
                } else {
                    // No receiver, reuse frame
                    self.put_rx_frame(rx_frame.forget_size::<RadioDriverImpl>());
                    SchedulerTaskTransition::Execute(action, None)
                }
            }
            _ => unreachable!(),
        }
    }

    pub fn on_rx_window_ended_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        rx_frame: RadioFrame<RadioFrameUnsized>,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::WaitingForFrameInRxSlot { response_token } => {
                // No frame received in this slot
                self.put_rx_frame(rx_frame);
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }
            _ => unreachable!(),
        }
    }

    pub fn on_nack_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        tx_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
        drv_svc_request: Option<DrvSvcRequest>,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::TransmittingInTxSlot { response_token } => {
                // TODO: reschedule retransmission
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                    tx_frame, instant,
                ));
                let action = self.wait_for_timeout_or_request(context);
                SchedulerTaskTransition::Execute(action, Some((response_token, resp)))
            }
            TschState::TransmittingInAdvertisementSlot => {
                // TODO: check retransmission handling if unicast
                todo!()
            }
            _ => unreachable!(),
        }
    }

    pub fn on_sent_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        tx_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::TransmittingInTxSlot { response_token } => {
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                    tx_frame.forget_size::<RadioDriverImpl>(),
                    instant,
                ));
                let action = self.wait_for_timeout_or_request(context);
                SchedulerTaskTransition::Execute(action, Some((response_token, resp)))
            }
            TschState::TransmittingInAdvertisementSlot => {
                // Put beacon frame back for next advertisement
                self.beacon_mpdu
                    .set(Some(MpduFrame::from_radio_frame(tx_frame)));

                // Record beacon transmission time for period calculation
                self.on_beacon_sent(instant);
                self.schedule_beacon(context);

                // Return to idle - next beacon will be scheduled automatically
                // in get_initial_action when the period expires
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }
            _ => unreachable!(),
        }
    }

    pub fn on_crc_error_event(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        rx_frame: RadioFrame<RadioFrameUnsized>,
        instant: NsInstant,
    ) -> SchedulerTaskTransition {
        let state = mem::replace(&mut self.state, TschState::Placeholder);

        match state {
            TschState::ReceivingFrameInRxSlot { response_token } => {
                self.put_rx_frame(rx_frame);
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }
            _ => unreachable!(),
        }
    }
}

// ============================================================================
// Scheduler Requests handling
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Handle a TX request by finding appropriate link and scheduling.
    fn on_scheduler_tx_request(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let current_time = context.timer.now();
        // Get current ASN
        let current_asn = self.asn_at(current_time, context);

        // Find appropriate link based on frame type
        let link = match mpdu.frame_control().frame_type() {
            FrameType::Beacon => context.pib.tsch.next_advertisement_link(current_asn),
            // TODO: For now, use first available link
            _ => context.pib.tsch.links().next(),
        };

        if let Some(link) = link {
            // Calculate next ASN for this link
            if let Some(next_asn) = context.pib.tsch.next_asn_for_link(link, current_asn) {
                // Calculate channel
                let channel = context.pib.tsch.channel_for_link(
                    next_asn,
                    link,
                    &context.pib.hopping_sequence,
                );

                // Create and queue the operation
                let operation = TschOperation::TxSlot {
                    mpdu,
                    asn: next_asn,
                    channel,
                    cca: false, // TSCH typically doesn't use CCA
                    response_token: token,
                };

                let _ = self.push_operation(operation);

                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            } else {
                todo!()
            }
        } else {
            todo!()
        }
    }
    fn on_scheduler_command(
        &mut self,
        token: ResponseToken,
        command: SchedulerCommand,
    ) -> SchedulerTaskTransition {
        use crate::scheduler::command::*;

        match command {
            #[cfg(feature = "tsch")]
            SchedulerCommand::TschCommand(tsch_cmd) => match tsch_cmd {
                UseTsch(enabled, _cca) => {
                    use crate::scheduler::command::tsch::TschCommandResult::UseTsch;
                    use crate::scheduler::task::SchedulerTaskCompletion;

                    let result = if enabled {
                        tsch::UseTschCommandResult::StartedTsch
                    } else {
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
            SchedulerCommand::PibCommand(_cmd) => {
                // PIB commands should be forwarded to the root scheduler or handled here
                // For now, we handle them similarly to CSMA mode
                todo!("PibCommand not yet implemented in TSCH mode")
            }
        }
    }
}

// ============================================================================
// TSCH Operations Scheduling
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn schedule_tx_operation(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::TxSlot {
                mpdu,
                asn,
                channel,
                cca,
                response_token,
            } => {
                self.state = TschState::WaitingForTxStartInTxSlot { response_token };

                self.update_timing(asn, context);

                // Calculate TX start time: timeslot start + macTsTxOffset
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let tx_instant = self.last_base_time + NsDuration::micros(tx_offset_us);

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
    fn schedule_rx_operation(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::RxSlot {
                asn,
                channel,
                response_token,
            } => {
                self.update_timing(asn, context);
                let frame = self.take_rx_frame().expect("no rx_frame for TSCH RX slot");

                self.state = TschState::WaitingForFrameInRxSlot { response_token };

                // Calculate RX start time: timeslot start + macTsRxOffset
                let rx_offset_us = context.pib.tsch.timeslot_timings.rx_offset() as u64;
                let rx_instant = self.last_base_time + NsDuration::micros(rx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                    start: Timestamp::Scheduled(rx_instant),
                    radio_frame: frame,
                    channel: Some(channel),
                });

                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(request),
                    None,
                )
            }
            _ => unreachable!(),
        }
    }
    #[cfg(feature = "tsch-coordinator")]
    fn schedule_advertisement_operation(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match self.pop_operation() {
            TschOperation::AdvertisementSlot { asn, channel } => {
                self.update_timing(asn, context);
                // Take beacon frame and update ASN
                let beacon_frame = self
                    .beacon_mpdu
                    .take()
                    .expect("no beacon frame for advertisement");

                let updated_frame = self
                    .beacon_builder
                    .update_beacon(beacon_frame, asn)
                    .expect("failed to update beacon ASN");

                self.state = TschState::WaitingForTxStartInAdvertisementSlot;

                // Calculate TX start time: timeslot start + macTsTxOffset
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let tx_instant = self.last_base_time + NsDuration::micros(tx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                    at: Timestamp::Scheduled(tx_instant),
                    radio_frame: updated_frame,
                    cca: false,
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
// Helpers
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn wait_for_timeout_or_request(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerAction {
        let deadline = self.peek_deadline(context);
        self.state = TschState::WaitingForDeadlineOrRequest {
            next_deadline: deadline,
        };

        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline }
    }
}
