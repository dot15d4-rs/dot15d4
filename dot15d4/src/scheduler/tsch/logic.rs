//! TSCH scheduler state machine logic.
//!
//! This module implements the state transition logic for the TSCH scheduler.
//! Operations are queued and executed at their scheduled times.
//! The scheduler wakes up slightly before each timeslot (guard time)
//! to prepare for the operation.

use core::mem;

use dot15d4_driver::{
    radio::{
        frame::{FrameType, RadioFrame, RadioFrameSized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};

#[cfg(feature = "tsch")]
use crate::scheduler::command::tsch::TschCommand::UseTsch;
use crate::{
    constants::MAC_TSCH_MAX_KEEP_ALIVE_MISSES,
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    mac::neighbors::NeighborId,
    scheduler::{
        SchedulerTask, SchedulerTaskCompletion, SchedulerTaskEvent, SchedulerTaskTransition,
    },
};
use crate::{
    scheduler::{
        command::SchedulerCommand, SchedulerAction, SchedulerContext, SchedulerRequest,
        SchedulerResponse, SchedulerTransmissionResult,
    },
    utils,
};

use super::log::{ch, ft, tsch_debug, tsch_info, tsch_trace};
use super::task::{TschCsmaBackoff, TschOperation, TschState, TschTask, TxOrigin};

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for TschTask<RadioDriverImpl> {
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match mem::replace(&mut self.state, TschState::Placeholder) {
            TschState::Idle => self.execute_idle(event, context),
            TschState::WaitingForTxStart {
                origin,
                shared,
                backoff,
            } => self.execute_waiting_for_tx_start(origin, shared, backoff, event, context),
            TschState::Transmitting {
                origin,
                shared,
                backoff,
            } => self.execute_transmitting(origin, shared, backoff, event, context),
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

                self.init(context);
                tsch_info!(
                    "tsch start root={} beacon={} cca={} links={} asn={}",
                    self.is_time_root,
                    self.beacon_config.enabled,
                    context.pib.tsch.tsch_cca,
                    context.pib.tsch.num_links(),
                    context.pib.tsch.asn
                );
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
            // Timer expired: queue a due keep-alive first, then execute the
            // next pending operation. When the wake-up was for the keep-alive
            // deadline alone (keep-alive due, no operation due yet, the
            // freshly queued keep-alive included), return to idle instead so
            // the deadline is re-armed for the keep-alive's own timeslot.
            SchedulerTaskEvent::TimerExpired => {
                let now = context.timer.now();
                let keep_alive_was_due = matches!(
                    self.keep_alive_deadline(context),
                    Some(deadline) if deadline <= now
                );
                self.maybe_queue_keep_alive(context);
                // `maybe_queue_keep_alive` is the only place the miss counter
                // moves, so this is the one point that has to test it. A
                // threshold of zero disables the mechanism, as elsewhere in
                // the TSCH configuration.
                if MAC_TSCH_MAX_KEEP_ALIVE_MISSES > 0
                    && self.missed_keep_alives >= MAC_TSCH_MAX_KEEP_ALIVE_MISSES
                {
                    return self.disconnect(context);
                }
                if keep_alive_was_due && self.op_deadline(context) > now {
                    return SchedulerTaskTransition::Execute(self.go_idle(context), None);
                }
                match self.pending_operations.last() {
                    Some(TschOperation::TxSlot { .. }) => self.schedule_tx_operation(context),
                    Some(TschOperation::RxSlot { .. }) => self.schedule_rx_operation(context),
                    Some(TschOperation::AdvertisementSlot { .. }) => {
                        self.schedule_advertisement_operation(context)
                    }
                    // Nothing pending : re-arm and keep waiting.
                    None => SchedulerTaskTransition::Execute(self.go_idle(context), None),
                    _ => unreachable!(),
                }
            }
            _ => unreachable!(),
        }
    }

    /// Handle WaitingForTxStart state - CCA/TX preparation in progress.
    fn execute_waiting_for_tx_start(
        &mut self,
        origin: TxOrigin,
        shared: bool,
        backoff: TschCsmaBackoff,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::TxStarted(_instant)) => {
                tsch_trace!("tsch tx-started asn={}", context.pib.tsch.asn);
                self.state = TschState::Transmitting {
                    origin,
                    shared,
                    backoff,
                };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::CcaBusy(radio_frame, _instant)) => {
                // CCA reported the channel busy before the frame went out.
                tsch_debug!("tsch tx-abort asn={} cause=cca-busy", context.pib.tsch.asn);
                self.handle_tx_start_failure(radio_frame, shared, backoff, origin, context)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::SchedulingFailed(radio_frame)) => {
                // The driver could not schedule the transmission (timer rejection,
                // deadline in the past, or guard-time overrun). The frame never
                // went out, so handle it like a CCA-busy failure instead of
                // aborting.
                tsch_debug!(
                    "tsch tx-abort asn={} cause=sched-failed",
                    context.pib.tsch.asn
                );
                self.handle_tx_start_failure(radio_frame, shared, backoff, origin, context)
            }
            // A TX outcome that skipped TxStarted.
            event @ SchedulerTaskEvent::DriverEvent(
                DrvSvcEvent::Sent(..) | DrvSvcEvent::Nack(..),
            ) => self.execute_transmitting(origin, shared, backoff, event, context),
            // Any other driver event: keep waiting, so that the TX outcome
            // returns the frame to its origin.
            SchedulerTaskEvent::DriverEvent(event) => {
                dot15d4_util::warn!(
                    "TSCH: unexpected event in WaitingForTxStart; continuing to wait."
                );
                Self::release_buffers(event, context);
                self.state = TschState::WaitingForTxStart {
                    origin,
                    shared,
                    backoff,
                };
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            // WaitingForTxStart legitimately only sees the events above; recover
            // to idle rather than aborting.
            _ => {
                dot15d4_util::warn!("TSCH: unexpected event in WaitingForTxStart; recovering to idle.");
                self.recover(event, context)
            }
        }
    }

    /// Handle a pre-transmission failure in `WaitingForTxStart`: CCA reported the
    /// channel busy, or the driver could not schedule the TX. Both mean the frame
    /// never went out, so they are handled identically: MAC and keep-alive
    /// frames enter the channel-access/retransmission procedure (a shared link
    /// applies the CSMA-CA backoff, a dedicated link retries without delay),
    /// and a beacon restores the frame and reschedules the advertisement.
    fn handle_tx_start_failure(
        &mut self,
        radio_frame: RadioFrame<RadioFrameSized>,
        shared: bool,
        backoff: TschCsmaBackoff,
        origin: TxOrigin,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match origin {
            // A frame that never went out (CCA busy or scheduling failure)
            // retries on both shared and dedicated links, up to
            // macMaxFrameRetries; only the backoff delay and BE increase are
            // shared-specific. The dedicated and early-attempt cases add no delay
            // and leave BE unchanged.
            origin @ (TxOrigin::Mac(_) | TxOrigin::KeepAlive) => {
                self.handle_tx_failure(radio_frame, backoff, origin, None, shared, context)
            }
            // Beacon: restore the frame and reschedule the advertisement.
            TxOrigin::Beacon => {
                self.recover_beacon(radio_frame, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
        }
    }

    /// Handle Transmitting state - frame transmission in progress.
    fn execute_transmitting(
        &mut self,
        origin: TxOrigin,
        shared: bool,
        backoff: TschCsmaBackoff,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Sent(tx_frame, instant, _timesync_us)) => {
                match origin {
                    TxOrigin::Mac(response_token) => {
                        tsch_info!(
                            "tsch tx sent asn={} shared={} attempts={}",
                            context.pib.tsch.asn,
                            shared,
                            backoff.nb.saturating_add(1)
                        );
                        // Neighbor statistics: the exchange completed, score it on
                        // the destination neighbor (nb failures + this attempt).
                        let mpdu = MpduFrame::from_radio_frame(tx_frame);
                        let dst_id = self.note_unicast_tx_result(
                            &mpdu,
                            true,
                            backoff.nb.saturating_add(1),
                            instant,
                            context,
                        );
                        let resp =
                            SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                                mpdu.into_radio_frame::<RadioDriverImpl>()
                                    .forget_size::<RadioDriverImpl>(),
                                instant,
                            ));
                        // Acknowledgment-based synchronization: only from an ACK
                        // sent by the time source, and never on the time root.
                        // An ACK from a child must not steer this node's clock.
                        if !self.is_time_root
                            && dst_id.is_some()
                            && dst_id == context.pib.tsch.time_source
                        {
                            // The exchange reached the time source: a keep-alive
                            // is not needed for another period, whether or not
                            // the ack-sync correction is compiled in, and the
                            // node is demonstrably still synchronized.
                            self.note_sync(instant, context);
                            if let Some(timesync_us) = _timesync_us {
                                context.pib.tsch.sync_ack(timesync_us);
                            }
                        }
                        let action = self.go_idle(context);
                        SchedulerTaskTransition::Execute(action, Some((response_token, resp)))
                    }
                    TxOrigin::KeepAlive => {
                        tsch_info!(
                            "tsch keep-alive sent asn={} attempts={}",
                            context.pib.tsch.asn,
                            backoff.nb.saturating_add(1)
                        );
                        // Same bookkeeping as a MAC frame - neighbor statistics
                        // and acknowledgment-based synchronization (the whole
                        // point of the keep-alive) - but the outcome is consumed
                        // internally: release the frame, respond to nobody.
                        let mpdu = MpduFrame::from_radio_frame(tx_frame);
                        let dst_id = self.note_unicast_tx_result(
                            &mpdu,
                            true,
                            backoff.nb.saturating_add(1),
                            instant,
                            context,
                        );
                        if !self.is_time_root
                            && dst_id.is_some()
                            && dst_id == context.pib.tsch.time_source
                        {
                            self.note_sync(instant, context);
                            if let Some(timesync_us) = _timesync_us {
                                context.pib.tsch.sync_ack(timesync_us);
                            }
                        }
                        // Safety: the keep-alive frame was allocated from this
                        // allocator by `build_keep_alive_frame`.
                        unsafe {
                            context
                                .buffer_allocator
                                .deallocate_buffer(mpdu.into_buffer())
                        };
                        SchedulerTaskTransition::Execute(self.go_idle(context), None)
                    }
                    TxOrigin::Beacon => {
                        // Advance the beacon grid past the ASN this one used
                        // (the slot the advertisement executed in, which
                        // `update_timing` made current), then put the frame
                        // back and schedule the next advertisement.
                        tsch_info!("tsch beacon sent asn={}", context.pib.tsch.asn);
                        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
                        self.on_beacon_sent(context.pib.tsch.asn, timeslot_length_us);
                        self.recover_beacon(tx_frame, context);
                        SchedulerTaskTransition::Execute(self.go_idle(context), None)
                    }
                }
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::Nack(
                tx_frame,
                instant,
                _request,
                _timesync_us,
            )) => {
                tsch_debug!("tsch tx no-ack asn={}", context.pib.tsch.asn);
                match origin {
                    // Both shared and dedicated links retransmit on a missing
                    // ACK, up to macMaxFrameRetries; the dedicated path adds no
                    // delay and does not bump BE. 
                    origin @ (TxOrigin::Mac(_) | TxOrigin::KeepAlive) => self.handle_tx_failure(
                        tx_frame,
                        backoff,
                        origin,
                        Some(instant),
                        shared,
                        context,
                    ),
                    // Beacon: restore the frame and reschedule the
                    // advertisement.
                    TxOrigin::Beacon => {
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
        origin: TxOrigin,
        noack_instant: Option<NsInstant>,
        failed_on_shared: bool,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let is_cca_busy = noack_instant.is_none();
        let nb = backoff.nb.saturating_add(1);

        if nb > context.pib.max_frame_retries {
            tsch_info!(
                "tsch tx drop asn={} nb={} max={} result={}",
                context.pib.tsch.asn,
                nb,
                context.pib.max_frame_retries,
                if is_cca_busy {
                    "channel-access-failure"
                } else {
                    "no-ack"
                }
            );
            // Neighbor statistics: the exchange failed for good, score it on
            // the destination neighbor with the no-ack penalty.
            let now = noack_instant.unwrap_or_else(|| context.timer.now());
            let mpdu = MpduFrame::from_radio_frame(tx_frame);
            let _ = self.note_unicast_tx_result(&mpdu, false, nb, now, context);
            let action = self.go_idle(context);
            return match origin {
                TxOrigin::Mac(response_token) => {
                    let tx_frame = mpdu.into_radio_frame::<RadioDriverImpl>();
                    let result = match noack_instant {
                        Some(instant) => SchedulerTransmissionResult::NoAck(tx_frame, instant),
                        None => SchedulerTransmissionResult::ChannelAccessFailure(tx_frame),
                    };
                    SchedulerTaskTransition::Execute(
                        action,
                        Some((response_token, SchedulerResponse::Transmission(result))),
                    )
                }
                TxOrigin::KeepAlive => {
                    // The time source acknowledged none of the attempts.
                    // Release the frame; the deadline was re-armed when this
                    // keep-alive was queued, so the next one goes out a 
                    // period later.
                    //
                    // Safety: the keep-alive frame was allocated from this
                    // allocator by `build_keep_alive_frame`.
                    unsafe {
                        context
                            .buffer_allocator
                            .deallocate_buffer(mpdu.into_buffer())
                    };
                    SchedulerTaskTransition::Execute(action, None)
                }
                TxOrigin::Beacon => unreachable!(),
            };
        }

        let (be, backoff_remaining, retransmitting) = if is_cca_busy && !backoff.retransmitting {
            // Channel-access failure: the frame has never been on the air, so
            // this is still the channel-access algorithm, which uses no
            // backoff. Retry at the next TX link with no delay and BE
            // unchanged, however many busy channels in a row are seen.
            (backoff.be, 0, false)
        } else {
            // Retransmission: entered at the first missing acknowledgment and
            // re-entered on every subsequent failure. Each round draws a fresh
            // delay of random(2^BE - 1) shared links; the wait only defers
            // shared links, a dedicated link to the destination is never
            // delayed and preempts it. BE is advanced before the draw when the
            // failed attempt was an unacknowledged transmission in a shared
            // link; dedicated-link failures and busy channels leave it
            // unchanged, so the round entering the procedure draws from
            // macTschMinBe.
            let be = if backoff.retransmitting && failed_on_shared && !is_cca_busy {
                backoff
                    .be
                    .saturating_add(1)
                    .min(context.pib.tsch.tsch_max_be)
            } else {
                backoff.be
            };
            let window = 1u32 << be.min(15);
            let backoff_remaining = (context.rng.next_u32() % window) as u16;
            (be, backoff_remaining, true)
        };

        tsch_debug!(
            "tsch tx retry nb={} be={} skip-shared={} on-shared={} cause={}",
            nb,
            be,
            backoff_remaining,
            failed_on_shared,
            if is_cca_busy { "cca" } else { "no-ack" }
        );

        let backoff = TschCsmaBackoff {
            be,
            nb,
            backoff_remaining,
            retransmitting,
        };
        let mpdu = MpduFrame::from_radio_frame(tx_frame);
        let current_asn = context.pib.tsch.asn_at(context.timer.now());
        self.queue_tx_with_backoff(origin, mpdu, current_asn, backoff, context);

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
                tsch_trace!("tsch rx frame-start asn={}", context.pib.tsch.asn);
                self.state = TschState::Receiving;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            SchedulerTaskEvent::DriverEvent(DrvSvcEvent::RxWindowEnded(rx_frame)) => {
                // No frame received in this slot: the slot is complete, so queue
                // the next RX exactly once here.
                tsch_trace!("tsch rx-slot empty asn={}", context.pib.tsch.asn);
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
                self.recover(event, context)
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
                // Convert to an MPDU once; the sync decision and the frame
                // routing below both work on this view.
                let mpdu = MpduFrame::from_radio_frame(rx_frame);

                // Neighbor statistics + source identification: attribute the
                // frame to its source neighbor (created on first contact).
                // Enhanced Beacons also record the advertised join metric
                // (the IE walk only runs for the rare beacon frames).
                let mut beacon_asn = None;
                let src_id = if let Ok(parsed) = mpdu.reader().parse_addressing() {
                    let src_id = match parsed.try_addressing_fields() {
                        Ok(addressing) => addressing
                            .try_src_address()
                            .and_then(|src| context.pib.neighbors.intern(&src, rx_instant)),
                        Err(_) => None,
                    };
                    if let Some(id) = src_id {
                        let join_metric =
                            if parsed.frame_control().frame_type() == FrameType::Beacon {
                                parsed
                                    .parse_security()
                                    .and_then(|r| r.parse_ies::<RadioDriverImpl>())
                                    .ok()
                                    .and_then(|with_ies| {
                                        with_ies.try_ies_fields()?.tsch_sync().map(|ts| {
                                            beacon_asn = Some(ts.asn());
                                            ts.join_metric()
                                        })
                                    })
                            } else {
                                None
                            };
                        match join_metric {
                            Some(metric) => {
                                context.pib.neighbors.note_beacon(id, metric, rx_instant)
                            }
                            None => context.pib.neighbors.note_rx(id, rx_instant),
                        }
                    }
                    src_id
                } else {
                    None
                };

                // Frame-based synchronization: only steer the local clock
                // from frames sent by the designated time source, never from
                // arbitrary neighbors, and never on the time root.
                if !self.is_time_root
                    && src_id.is_some()
                    && src_id == context.pib.tsch.time_source
                {
                    let asn = beacon_asn.unwrap_or(context.pib.tsch.asn);
                    context.pib.tsch.sync_asn(asn, rx_instant);
                    // Heard from the time source: a keep-alive is not needed
                    // for another period, whether or not the frame-sync
                    // correction is compiled in, and the node is demonstrably
                    // still synchronized.
                    self.note_sync(rx_instant, context);
                }

                #[cfg(feature = "sync-error-log")]
                if src_id.is_some() && src_id != context.pib.tsch.time_source {
                    let peer = src_id
                        .and_then(|id| context.pib.neighbors.get(id))
                        .map(|neighbor| *neighbor.ext_addr());
                    if let Some(peer) = peer {
                        context.pib.tsch.log_peer_sync_error(rx_instant, &peer);
                    }
                }

                // Slot complete: queue the next RX exactly once.
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                let action = self.go_idle(context);

                tsch_debug!(
                    "tsch rx asn={} type={} known-src={} from-timesrc={}",
                    current_asn,
                    ft(mpdu.frame_control().frame_type()),
                    src_id.is_some(),
                    src_id.is_some() && src_id == context.pib.tsch.time_source
                );

                match utils::process_rx_frame(mpdu, rx_instant, context) {
                    Ok((response, token)) => {
                        // Allocate new frame for next RX
                        self.put_rx_frame(context.allocate_frame());

                        SchedulerTaskTransition::Execute(action, Some((token, response)))
                    }
                    Err(recovered_mpdu) => {
                        let dropped_type = recovered_mpdu.frame_control().frame_type();
                        if matches!(dropped_type, FrameType::Beacon) {
                            tsch_trace!(
                                "tsch rx dropped asn={} type=beacon cause=no-consumer",
                                current_asn
                            );
                        } else {
                            tsch_debug!(
                                "tsch rx dropped asn={} type={} cause=no-receiver",
                                current_asn,
                                ft(dropped_type)
                            );
                        }
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
                tsch_debug!("tsch rx dropped asn={} cause=crc", current_asn);
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
                tsch_trace!("tsch rx aborted asn={}", context.pib.tsch.asn);
                self.state = TschState::Listening;
                SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
            }
            // Receiving legitimately only sees the events above. Recover to idle
            // rather than aborting.
            _ => {
                dot15d4_util::warn!("TSCH: unexpected event in Receiving; recovering to idle.");
                let current_asn = context.pib.tsch.asn;
                self.queue_next_rx(current_asn, context);
                self.recover(event, context)
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
                UseTsch { enabled, cca, .. } => {
                    use crate::scheduler::command::tsch::TschCommandResult::UseTsch;

                    // Keep macTschCca in sync (the CSMA-side handler is the one
                    // that takes effect on entry, but mirror it here too).
                    context.pib.tsch.tsch_cca = cca;

                    let result = if enabled {
                        tsch::UseTschCommandResult::StartedTsch
                    } else {
                        tsch_info!(
                            "tsch stop asn={} pending={}",
                            context.pib.tsch.asn,
                            self.pending_operation_count()
                        );
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
                slotframe_handle,
                channel,
                cca,
                shared,
                backoff,
                origin,
            } => {
                tsch_debug!(
                    "tsch tx-slot asn={} sf={} ch={} shared={} cca={} nb={} be={}",
                    asn,
                    slotframe_handle,
                    ch(channel),
                    shared,
                    cca,
                    backoff.nb,
                    backoff.be
                );
                self.state = TschState::WaitingForTxStart {
                    origin,
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
                    ack_window: context.pib.tsch.ack_window(),
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
            TschOperation::RxSlot {
                asn,
                slotframe_handle,
                channel,
            } => {
                tsch_debug!(
                    "tsch rx-slot asn={} sf={} ch={}",
                    asn,
                    slotframe_handle,
                    ch(channel)
                );
                context.pib.tsch.update_timing(asn);
                let frame = self
                    .take_rx_frame()
                    .unwrap_or_else(|| context.allocate_frame());

                self.state = TschState::Listening;

                let (rx_offset_us, rx_wait_us) = context
                    .pib
                    .tsch
                    .rx_window_us(RadioDriverImpl::RX_ARM_LEAD.to_micros());
                let tx_offset_us = context.pib.tsch.timeslot_timings.tx_offset() as u64;
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
                    ack_delay: context.pib.tsch.ack_delay(),
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
            TschOperation::AdvertisementSlot {
                asn,
                slotframe_handle,
                channel,
            } => {
                tsch_debug!(
                    "tsch adv-slot asn={} sf={} ch={}",
                    asn,
                    slotframe_handle,
                    ch(channel)
                );
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
                    origin: TxOrigin::Beacon,
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
                    ack_window: context.pib.tsch.ack_window(),
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

    /// Score a completed unicast exchange (success or retry exhaustion) on
    /// the destination neighbor's statistics.
    ///
    /// Only frames that requested an acknowledgment and go to a unicast
    /// destination already present in the neighbor table are scored:
    /// transmissions never create table entries, reception does.
    fn note_unicast_tx_result(
        &self,
        mpdu: &MpduFrame,
        acked: bool,
        attempts_used: u8,
        now: NsInstant,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Option<NeighborId> {
        let reader = mpdu.reader();
        if !reader.frame_control().ack_request() {
            return None;
        }
        let Ok(parsed) = reader.parse_addressing() else {
            return None;
        };
        let Ok(addressing) = parsed.try_addressing_fields() else {
            return None;
        };
        let dst_id = addressing
            .try_dst_address()
            .and_then(|dst| context.pib.neighbors.find(&dst));
        if let Some(id) = dst_id {
            context
                .pib
                .neighbors
                .note_tx_result(id, acked, attempts_used, now);
        }
        dst_id
    }

    /// Return to idle after an unexpected event, giving back what it carries.
    fn recover(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            // Serve a request as the idle state does.
            SchedulerTaskEvent::SchedulerRequest { .. } => self.execute_idle(event, context),
            SchedulerTaskEvent::DriverEvent(event) => {
                Self::release_buffers(event, context);
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
            SchedulerTaskEvent::Entry | SchedulerTaskEvent::TimerExpired => {
                SchedulerTaskTransition::Execute(self.go_idle(context), None)
            }
        }
    }

    /// Return the buffers that a driver event carries to the pool.
    fn release_buffers(event: DrvSvcEvent, context: &SchedulerContext<RadioDriverImpl>) {
        let (frame, request) = match event {
            DrvSvcEvent::CcaBusy(frame, _)
            | DrvSvcEvent::Sent(frame, ..)
            | DrvSvcEvent::Received(frame, _)
            | DrvSvcEvent::SchedulingFailed(frame) => (Some(frame.into_buffer()), None),
            DrvSvcEvent::RxWindowEnded(frame) | DrvSvcEvent::CrcError(frame, _) => {
                (Some(frame.into_buffer()), None)
            }
            DrvSvcEvent::Nack(frame, _, request, _) => (Some(frame.into_buffer()), request),
            DrvSvcEvent::TxStarted(_)
            | DrvSvcEvent::FrameStarted
            | DrvSvcEvent::ReceptionAborted => (None, None),
        };
        let request = request.and_then(|request| match request {
            DrvSvcRequest::CompleteThenStartTx(tx) => Some(tx.mpdu.into_buffer()),
            DrvSvcRequest::CompleteThenStartRx(rx) => Some(rx.radio_frame.into_buffer()),
            DrvSvcRequest::CompleteThenGoIdle => None,
        });
        for buffer in [frame, request].into_iter().flatten() {
            // Safety: every frame buffer of the stack comes from this allocator.
            unsafe { context.buffer_allocator.deallocate_buffer(buffer) };
        }
    }

    /// Transition to idle state and wait for the next deadline by configuring
    /// timer action accordingly.
    fn go_idle(&mut self, context: &SchedulerContext<RadioDriverImpl>) -> SchedulerAction {
        let deadline = self.peek_deadline(context);
        self.state = TschState::Idle;

        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline }
    }

    fn disconnect(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        tsch_info!(
            "tsch disconnect asn={} missed-ka={} pending={}",
            context.pib.tsch.asn,
            self.missed_keep_alives,
            self.pending_operation_count()
        );

        // `step` took the state out to dispatch this event; leave a valid one
        // behind. The root scheduler drops this task on `SwitchToCsma`, but a
        // task left holding `Placeholder` would panic in `step` if anything
        // ever did reach it again.
        self.state = TschState::Idle;

        // The former time source no longer needs its neighbor table entry
        // protected from eviction.
        let former_time_source = context.pib.tsch.time_source.take();
        if let Some(id) = former_time_source {
            context.pib.neighbors.unpin(id);
        }
        self.terminate(context);

        SchedulerTaskTransition::Completed(SchedulerTaskCompletion::SwitchToCsma, None)
    }
}
