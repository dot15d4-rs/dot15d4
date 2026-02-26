//! TSCH scheduler state machine logic.
//!
//! This module implements the state transition logic for the TSCH scheduler.
//! Operations are queued and executed at their scheduled times.
//! The scheduler wakes up slightly before each timeslot (guard time)
//! to prepare for the operation.

use core::mem;

use dot15d4_driver::{
    radio::{frame::Address, DriverConfig},
    timer::{NsDuration, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;

#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::TschCommand::UseTsch;
use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    mac::mlme::{get::GetRequestAttribute, set::SetRequestAttribute},
    scheduler::{
        command::pib::{GetPibResult, SetPibResult},
        SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition,
    },
};
use crate::{
    scheduler::{
        command::SchedulerCommand, SchedulerAction, SchedulerContext, SchedulerRequest,
        SchedulerResponse, SchedulerTransmissionResult,
    },
    utils,
};

use super::task::{TschOperation, TschState, TschTask};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for TschTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match mem::replace(&mut self.state, TschState::Placeholder) {
            TschState::Idle => self.execute_idle(event, context),
            TschState::WaitingForTxStart { response_token } => {
                self.execute_waiting_for_tx_start(response_token, event)
            }
            TschState::Transmitting { response_token } => {
                self.execute_transmitting(response_token, event, context)
            }
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

                #[cfg(feature = "tsch-coordinator")]
                {
                    if context.pib.coord_extended_address.is_broadcast() {
                        self.init_coordinator(context, Some(9));
                    } else {
                        self.init_device(context);
                    }
                }
                #[cfg(not(feature = "tsch-coordinator"))]
                {
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
                SchedulerRequest::Reception(_reception_type) => todo!(),
            },
            // Timer expired: time to execute next operation in the upcoming timeslot
            SchedulerTaskEvent::TimerExpired => match self.pending_operations.last() {
                Some(TschOperation::TxSlot { .. }) => self.schedule_tx_operation(context),
                Some(TschOperation::RxSlot { .. }) => self.schedule_rx_operation(context),
                #[cfg(feature = "tsch-coordinator")]
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
        event: SchedulerTaskEvent,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::TxStarted(_instant)) => {
                self.state = TschState::Transmitting { response_token };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CcaBusy(_radio_frame, _instant)) => {
                todo!()
            }
            _ => unreachable!(),
        }
    }

    /// Handle Transmitting state - frame transmission in progress.
    fn execute_transmitting(
        &mut self,
        response_token: Option<ResponseToken>,
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
                    #[cfg(feature = "tsch-coordinator")]
                    {
                        // Put beacon frame back for next advertisement
                        self.beacon_mpdu
                            .set(Some(MpduFrame::from_radio_frame(tx_frame)));

                        // Record beacon transmission time for period calculation
                        self.on_beacon_sent(instant);
                        self.queue_next_beacon(context);
                    }
                    // Return to idle - next beacon will be scheduled automatically
                    // in get_initial_action when the period expires
                    SchedulerTaskTransition::Execute(self.go_idle(context), None)
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Nack(
                tx_frame,
                instant,
                _request,
                _timesync_us,
            )) => {
                if let Some(response_token) = response_token {
                    // TODO: reschedule retransmission
                    let resp = SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                        tx_frame, instant,
                    ));
                    let action = self.go_idle(context);
                    SchedulerTaskTransition::Execute(action, Some((response_token, resp)))
                } else {
                    // TODO: Unicast Beacon ?
                    todo!()
                }
            }
            _ => unreachable!(),
        }
    }

    /// Handle Listening state - RX window active, waiting for frames.
    fn execute_listening(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let current_asn = context.pib.tsch.asn;
        self.queue_next_rx(current_asn, context);
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::FrameStarted) => {
                self.state = TschState::Receiving;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(rx_frame)) => {
                // No frame received in this slot
                self.put_rx_frame(rx_frame);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            _ => unreachable!(),
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
                let action = self.go_idle(context);

                // Frame-based synchronization
                #[cfg(not(feature = "no-tsch-frame-sync"))]
                if !self.is_coordinator {
                    let asn = context.pib.tsch.asn;
                    context.pib.tsch.sync_asn(asn, rx_instant);
                }

                match utils::process_rx_frame(rx_frame, rx_instant, context) {
                    Ok((response, token)) => {
                        // Allocate new frame for next RX
                        self.put_rx_frame(context.allocate_frame());

                        SchedulerTaskTransition::Execute(action, Some((token, response)))
                    }
                    Err(recovered_frame) => {
                        // No receiver, reuse frame
                        self.put_rx_frame(recovered_frame);
                        SchedulerTaskTransition::Execute(action, None)
                    }
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CrcError(rx_frame, _instant)) => {
                self.put_rx_frame(rx_frame);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            _ => unreachable!(),
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

        if self.queue_tx(token, mpdu, current_asn, context) {
            SchedulerTaskTransition::Execute(self.go_idle(context), None)
        } else {
            todo!()
        }
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
                UseTsch(enabled, _cca) => {
                    use crate::scheduler::{
                        command::tsch::TschCommandResult::UseTsch, SchedulerTaskCompletion,
                    };

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
            SchedulerCommand::PibCommand(cmd) => match cmd {
                PibCommand::Set(attribute) => {
                    let result = match attribute {
                        SetRequestAttribute::MacShortAddress(short_addr) => {
                            context.pib.short_address = short_addr;
                            SetPibResult::Success
                        }
                        SetRequestAttribute::MacCoordExtendedAddress(addr) => {
                            context.pib.coord_extended_address = Address::from_le_bytes(&addr);
                            SetPibResult::Success
                        }
                        // NOTE: we do not need/support other attributes in
                        //       TSCH mode for now.
                        _ => unreachable!(),
                    };
                    let resp = SchedulerResponse::Command(SchedulerCommandResult::PibCommand(
                        PibCommandResult::Set(result),
                    ));
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SelectDriverEventOrRequest,
                        Some((token, resp)),
                    )
                }
                PibCommand::Get(attribute) => {
                    let result = match attribute {
                        GetRequestAttribute::MacExtendedAddress => {
                            GetPibResult::MacExtendedAddress(context.pib.extended_address)
                        }
                        GetRequestAttribute::MacCoordExtendedAddress => {
                            GetPibResult::MacCoordExtendedAddress(
                                context.pib.coord_extended_address,
                            )
                        }
                        GetRequestAttribute::MacAssociationPermit => {
                            GetPibResult::MacAssociationPermit(context.pib.association_permit)
                        }
                        GetRequestAttribute::MacPanId => {
                            GetPibResult::MacPanId(context.pib.pan_id.into_u16())
                        }
                        GetRequestAttribute::MacShortAddress => {
                            GetPibResult::MacShortAddress(context.pib.short_address)
                        }
                        _ => todo!(),
                    };
                    let resp = SchedulerResponse::Command(SchedulerCommandResult::PibCommand(
                        PibCommandResult::Get(result),
                    ));
                    SchedulerTaskTransition::Execute(
                        SchedulerAction::SelectDriverEventOrRequest,
                        Some((token, resp)),
                    )
                }
                PibCommand::Reset => todo!(),
            },
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
                response_token,
                ..
            } => {
                self.state = TschState::WaitingForTxStart {
                    response_token: Some(response_token),
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
                let frame = self.take_rx_frame().expect("no rx_frame for TSCH RX slot");

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
    /// Coordinator-only. Updates the beacon frame with current ASN and
    /// schedules transmission.
    #[cfg(feature = "tsch-coordinator")]
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
                };

                // Calculate TX start time: timeslot start + macTsTxOffset
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
                let tx_instant = context.pib.tsch.last_base_time + NsDuration::micros(tx_offset_us);

                let request = DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                    at: Timestamp::Scheduled(tx_instant),
                    mpdu: updated_frame,
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
// Helper Methods
// ============================================================================

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Transition to idle state and wait for the next deadline by configuring
    /// timer action accordingly.
    fn go_idle(&mut self, context: &SchedulerContext<RadioDriverImpl>) -> SchedulerAction {
        let deadline = self.peek_deadline(context);
        self.state = TschState::Idle;

        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline }
    }
}
