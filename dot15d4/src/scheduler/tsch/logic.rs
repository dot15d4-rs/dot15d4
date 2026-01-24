//! TSCH scheduler logic implementation.

use core::mem;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{FrameType, RadioFrame, RadioFrameSized},
        DriverConfig,
    },
    timer::{NsDuration, RadioTimerApi},
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
            // TODO: Data traffic not supported yet
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.on_scheduler_request(token, request, context)
            }
            SchedulerTaskEvent::TimerExpired => self.on_timer_expired(context),
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

        match self.state {
            TschState::Idle { .. } => {
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }

            _ => SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None),
        }
    }

    /// Handle a scheduler request.
    pub fn on_scheduler_request(
        &mut self,
        token: ResponseToken,
        request: SchedulerRequest,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match request {
            SchedulerRequest::Transmission(mpdu) => self.handle_tx_request(token, mpdu, context),

            SchedulerRequest::Command(command) => self.handle_command(token, command),

            SchedulerRequest::Reception => {
                // RX request - schedule in appropriate slot
                // For now, just continue waiting
                let deadline = self.peek_deadline(context);
                SchedulerTaskTransition::Execute(
                    SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline },
                    None,
                )
            }
        }
    }

    /// Handle a driver event.
    pub fn on_driver_event(
        &mut self,
        event: DrvSvcEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        // Take ownership of current mode using mem::replace
        let placeholder = TschState::Idle {
            next_deadline: INFINITE_DEADLINE,
        };
        let current_mode = mem::replace(&mut self.state, placeholder);

        match current_mode {
            TschState::TxSlotWaitingForStart { response_token } => {
                self.on_tx_slot_start_event(event, response_token)
            }
            TschState::TxSlotWaitingForResult { response_token } => {
                self.on_tx_slot_result_event(event, response_token, context)
            }
            TschState::RxSlotWaitingForFrame { response_token } => {
                self.on_rx_slot_start_event(event, response_token, context)
            }
            TschState::RxSlotReceivingFrame { response_token } => {
                self.on_rx_slot_result_event(event, response_token, context)
            }
            #[cfg(feature = "tsch-coordinator")]
            TschState::AdvertisementWaitingForStart => self.on_advertisement_start_event(event),
            #[cfg(feature = "tsch-coordinator")]
            TschState::AdvertisementWaitingForResult => {
                self.on_advertisement_result_event(event, context)
            }
            #[cfg(not(feature = "tsch-coordinator"))]
            TschState::AdvertisementWaitingForStart | TschState::AdvertisementWaitingForResult => {
                unreachable!("advertisement states without tsch-coordinator feature")
            }
            TschState::Idle { .. } => {
                unreachable!("driver event in Idle mode")
            }
        }
    }

    /// Handle timer expiry - time to execute next operation.
    pub fn on_timer_expired(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        let operation = self.pop_operation();

        match operation {
            TschOperation::TxSlot {
                radio_frame,
                asn,
                channel,
                cca,
                response_token,
            } => {
                self.update_timing(asn, timeslot_length_us);
                self.schedule_tx_slot(radio_frame, channel, cca, response_token, context)
            }
            TschOperation::RxSlot {
                asn,
                channel,
                response_token,
            } => {
                self.update_timing(asn, timeslot_length_us);
                self.schedule_rx_slot(channel, response_token, context)
            }
            #[cfg(feature = "tsch-coordinator")]
            TschOperation::AdvertisementSlot { asn, channel } => {
                self.update_timing(asn, timeslot_length_us);
                self.schedule_advertisement_slot(asn, channel, context)
            }
            #[cfg(not(feature = "tsch-coordinator"))]
            TschOperation::AdvertisementSlot { .. } => {
                // Advertisement slots are not supported without tsch-coordinator feature
                self.state = TschState::Idle {
                    next_deadline: INFINITE_DEADLINE,
                };
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }
            TschOperation::Idle => {
                // No operation - shouldn't happen if timer fired
                self.state = TschState::Idle {
                    next_deadline: INFINITE_DEADLINE,
                };
                SchedulerTaskTransition::Execute(
                    SchedulerAction::WaitForTimeoutOrSchedulerRequest {
                        deadline: INFINITE_DEADLINE,
                    },
                    None,
                )
            }
        }
    }
}

// ============================================================================
// TSCH Transmission-related operations
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Handle a TX request by finding appropriate link and scheduling.
    fn handle_tx_request(
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

        let link = match link {
            Some(l) => l,
            None => {
                // No link available
                todo!()
            }
        };

        // Calculate next ASN for this link
        let next_asn = match context.pib.tsch.next_asn_for_link(link, current_asn) {
            Some(asn) => asn,
            None => {
                todo!()
            }
        };

        // Calculate channel
        let channel =
            context
                .pib
                .tsch
                .channel_for_link(next_asn, link, &context.pib.hopping_sequence);

        // Convert MPDU to radio frame
        let radio_frame = mpdu.into_radio_frame::<RadioDriverImpl>();

        // Create and queue the operation
        let op = TschOperation::TxSlot {
            radio_frame,
            asn: next_asn,
            channel,
            cca: false, // TSCH typically doesn't use CCA
            response_token: token,
        };

        let _ = self.push_operation(op);

        // Update deadline
        let deadline = self.peek_deadline(context);
        self.state = TschState::Idle {
            next_deadline: deadline,
        };

        SchedulerTaskTransition::Execute(
            SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline },
            None,
        )
    }

    fn schedule_tx_slot(
        &mut self,
        radio_frame: RadioFrame<RadioFrameSized>,
        channel: Channel,
        cca: bool,
        response_token: ResponseToken,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        self.state = TschState::TxSlotWaitingForStart { response_token };

        // Calculate TX start time: timeslot start + macTsTxOffset
        let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
        let tx_instant = self.last_base_time + NsDuration::micros(tx_offset_us);

        let request = DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
            at: Timestamp::Scheduled(tx_instant),
            radio_frame,
            cca,
            channel: Some(channel),
            fallback_on_nack: false,
        });

        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(request), None)
    }

    fn on_tx_slot_start_event(
        &mut self,
        event: DrvSvcEvent,
        response_token: ResponseToken,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::TxStarted(_instant) => {
                self.state = TschState::TxSlotWaitingForResult { response_token };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            _ => unreachable!("unexpected event in TxSlotWaitingForStart"),
        }
    }

    fn on_tx_slot_result_event(
        &mut self,
        event: DrvSvcEvent,
        response_token: ResponseToken,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let (response, action) = match event {
            DrvSvcEvent::Sent(radio_frame, instant) => {
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                    radio_frame.forget_size::<RadioDriverImpl>(),
                    instant,
                ));
                (resp, self.wait_for_timeout_or_request(context))
            }

            DrvSvcEvent::Nack(radio_frame, instant, _) => {
                // TODO: reschedule retransmission
                let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                    radio_frame,
                    instant,
                ));
                (resp, self.wait_for_timeout_or_request(context))
            }

            _ => unreachable!("unexpected event in TxSlotWaitingForResult"),
        };

        SchedulerTaskTransition::Execute(action, Some((response_token, response)))
    }
}

// ============================================================================
// TSCH Reception-related operations
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn schedule_rx_slot(
        &mut self,
        channel: Channel,
        response_token: Option<ResponseToken>,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let frame = self.take_rx_frame().expect("no rx_frame for TSCH RX slot");

        self.state = TschState::RxSlotWaitingForFrame { response_token };

        // Calculate RX start time: timeslot start + macTsRxOffset
        let rx_offset_us = context.pib.tsch.timeslot_timings.rx_offset() as u64;
        let rx_instant = self.last_base_time + NsDuration::micros(rx_offset_us);

        let request = DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
            start: Timestamp::Scheduled(rx_instant),
            radio_frame: frame,
            channel: Some(channel),
        });

        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(request), None)
    }

    fn on_rx_slot_start_event(
        &mut self,
        event: DrvSvcEvent,
        response_token: Option<ResponseToken>,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::FrameStarted => {
                self.state = TschState::RxSlotReceivingFrame { response_token };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }

            DrvSvcEvent::RxWindowEnded(frame) => {
                // No frame received in this slot
                self.put_rx_frame(frame);
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }

            _ => unreachable!("unexpected event in RxSlotWaitingForFrame"),
        }
    }

    fn on_rx_slot_result_event(
        &mut self,
        event: DrvSvcEvent,
        response_token: Option<ResponseToken>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::Received(frame, instant) => {
                let action = self.wait_for_timeout_or_request(context);

                if let Some(token) = response_token {
                    // Allocate new frame for next RX
                    self.put_rx_frame(context.allocate_frame());

                    let response = SchedulerResponse::Reception(frame, instant);
                    SchedulerTaskTransition::Execute(action, Some((token, response)))
                } else {
                    // No receiver, reuse frame
                    self.put_rx_frame(frame.forget_size::<RadioDriverImpl>());
                    SchedulerTaskTransition::Execute(action, None)
                }
            }

            DrvSvcEvent::CrcError(frame, _instant) => {
                self.put_rx_frame(frame);
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }

            _ => unreachable!("unexpected event in RxSlotReceivingFrame"),
        }
    }
}

// ============================================================================
// Commands & Helpers
// ============================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn handle_command(
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
        }
    }
    fn wait_for_timeout_or_request(
        &mut self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerAction {
        let deadline = self.peek_deadline(context);
        self.state = TschState::Idle {
            next_deadline: deadline,
        };

        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline }
    }
}

// ========================================================================
// TSCH Coordinator
// ========================================================================
#[cfg(feature = "tsch-coordinator")]
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn schedule_advertisement_slot(
        &mut self,
        asn: TschAsn,
        channel: Channel,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        // Take beacon frame and update ASN
        let beacon_frame = self
            .beacon_frame
            .take()
            .expect("no beacon frame for advertisement");

        let updated_frame = self
            .beacon_builder
            .update_beacon(beacon_frame, asn)
            .expect("failed to update beacon ASN");

        self.state = TschState::AdvertisementWaitingForStart;

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

        SchedulerTaskTransition::Execute(SchedulerAction::SendDriverRequestThenWait(request), None)
    }

    fn on_advertisement_start_event(&mut self, event: DrvSvcEvent) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::TxStarted(_instant) => {
                self.state = TschState::AdvertisementWaitingForResult;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            DrvSvcEvent::CcaBusy(_radio_frame, _instant) => {
                //TODO: support tsch csma/retransmission
                todo!()
            }
            _ => unreachable!(),
        }
    }

    fn on_advertisement_result_event(
        &mut self,
        event: DrvSvcEvent,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            DrvSvcEvent::Sent(beacon_frame, instant) => {
                // Put beacon frame back for next advertisement
                self.beacon_frame.set(Some(beacon_frame));

                // Record beacon transmission time for period calculation
                self.on_beacon_sent(instant);
                self.schedule_beacon(context);

                // Return to idle - next beacon will be scheduled automatically
                // in get_initial_action when the period expires
                SchedulerTaskTransition::Execute(self.wait_for_timeout_or_request(context), None)
            }
            // TODO: check retransmission handling if unicast
            DrvSvcEvent::Nack(_radio_frame, _instant, _drv_svc_request) => todo!(),
            _ => unreachable!(),
        }
    }
}
