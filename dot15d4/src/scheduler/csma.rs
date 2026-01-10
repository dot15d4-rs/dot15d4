use dot15d4_driver::radio::{
    frame::{RadioFrame, RadioFrameSized},
    DriverConfig,
};
use dot15d4_util::{
    allocator::IntoBuffer,
    sync::{select, ConsumerToken, Either, ResponseToken},
};

#[cfg(feature = "tsch")]
use super::command::UseTschCommandResult;
use super::{SchedulerService, SchedulerState};
use crate::driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp};
use crate::scheduler::{
    command::SchedulerCommand, SchedulerRequest, SchedulerResponse, SchedulerTransmissionResult,
    TaskDirection,
};

pub enum CsmaSchedulerState {
    Initial,
    Transmitting(ResponseToken),
    WaitingForFrame,
    Terminating(SchedulerState),
}

const CSMA_MAX_RETRANSMISSIONS: i32 = 4;
const CSMA_MAX_ATTEMPTS: i32 = 4;

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    pub(super) async fn run_csma(
        &mut self,
        mut consumer_token: ConsumerToken,
    ) -> (SchedulerState, ConsumerToken) {
        let mut state = CsmaSchedulerState::Initial;

        loop {
            state = match state {
                CsmaSchedulerState::Initial => self.initial().await,
                CsmaSchedulerState::Transmitting(sched_response_token) => {
                    self.complete_csma_ca_tx(sched_response_token).await
                }
                CsmaSchedulerState::WaitingForFrame => {
                    self.waiting_for_frame(&mut consumer_token).await
                }
                CsmaSchedulerState::Terminating(scheduler_state) => {
                    break (scheduler_state, consumer_token)
                }
            }
        }
    }

    pub(super) async fn complete_csma_ca_tx(
        &mut self,
        mut response_token: ResponseToken,
    ) -> CsmaSchedulerState {
        let mut pending_tx_frame = None;
        let mut pending_response_token = None;

        // Outer loop for consecutive transmission requests
        loop {
            let mut retransmissions = 0;
            'retransmissions: loop {
                let mut attempts = 1;
                let cca_result = 'attempts: loop {
                    match self.driver_event_receiver.receive().await {
                        DrvSvcEvent::TxStarted(_instant) => break 'attempts Ok(()),
                        DrvSvcEvent::CcaBusy(radio_frame, instant) => {
                            if attempts == CSMA_MAX_ATTEMPTS {
                                // ChannelAccessFailure => retransmit
                                break 'attempts Err((radio_frame, instant));
                            }
                            // TODO update timestamp with backoff
                            self.driver_request_sender
                                .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                                    at: Timestamp::BestEffort,
                                    radio_frame,
                                    cca: true,
                                    channel: None,
                                    fallback_on_nack: retransmissions
                                        != CSMA_MAX_RETRANSMISSIONS - 1,
                                }))
                                .await;
                        }
                        _ => unreachable!(),
                    }
                    attempts += 1;
                };
                match cca_result {
                    // CCA Idle or no CCA
                    Ok(_) => {
                        // Set/update pending TX request (and associated response token)
                        // used for scheduling
                        (pending_tx_frame, pending_response_token) =
                            self.update_pending_tx(pending_tx_frame, pending_response_token);

                        // If there is a pending TX request, use it, otherwise we want
                        // to wait for a frame
                        let (request, is_tx) = self.next_driver_request(pending_tx_frame);

                        self.driver_request_sender.send(request).await;
                        let result = match self.driver_event_receiver.receive().await {
                            // Completion of previous request resulted in NACK. If `fallback_on_nack` is enabled,
                            // current request is recovered in `request`, otherwise `request` is
                            // None. In CSMA-CA transmission, `fallback_on_nack` is disabled only
                            // for last retransmission since no further driver TX request will be
                            // necessary for that transmission, i.e. no need to wait for driver
                            // result to schedule next driver request.
                            DrvSvcEvent::Nack(radio_frame, instant, request) => match request {
                                Some(DrvSvcRequest::CompleteThenStartTx(task_tx)) => {
                                    pending_tx_frame = Some(task_tx.radio_frame);
                                    Err(radio_frame)
                                }
                                Some(DrvSvcRequest::CompleteThenStartRx(task_rx)) => {
                                    self.rx_frame.set(Some(task_rx.radio_frame));
                                    pending_tx_frame = None;
                                    Err(radio_frame)
                                }
                                // None is expected for last retransmission since we disabled
                                // `fallback_on_nack` which results in next driver request being
                                // scheduled (i.e. not recovered)
                                None => {
                                    debug_assert!(retransmissions == CSMA_MAX_RETRANSMISSIONS);
                                    pending_tx_frame = None;
                                    Ok(SchedulerResponse::Transmission(
                                        SchedulerTransmissionResult::NoAck(radio_frame, instant),
                                    ))
                                }
                                _ => unreachable!(),
                            },
                            DrvSvcEvent::Sent(radio_frame, instant) => {
                                pending_tx_frame = None;
                                Ok(SchedulerResponse::Transmission(
                                    SchedulerTransmissionResult::Sent(
                                        radio_frame.forget_size::<RadioDriverImpl>(),
                                        instant,
                                    ),
                                ))
                            }
                            _ => unreachable!(),
                        };
                        match result {
                            Ok(response) => {
                                self.request_receiver.received(response_token, response);
                                if is_tx {
                                    response_token = pending_response_token.unwrap();
                                    pending_response_token = None;
                                    // We just scheduled transmission for next request,
                                    break 'retransmissions;
                                } else {
                                    // We scheduled a RX request (since no TX request available)
                                    return CsmaSchedulerState::WaitingForFrame;
                                }
                            }
                            Err(radio_frame) => {
                                self.driver_request_sender
                                    .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                                        at: Timestamp::BestEffort,
                                        radio_frame,
                                        cca: true,
                                        channel: None,
                                        fallback_on_nack: retransmissions
                                            != CSMA_MAX_RETRANSMISSIONS - 1,
                                    }))
                                    .await;
                            }
                        }
                    }
                    // Channel Access Failure (Cca busy for all attempts)
                    Err((recovered_radio_frame, instant)) => {
                        // Next retransmission
                        if retransmissions == CSMA_MAX_RETRANSMISSIONS - 1 {
                            self.request_receiver.received(
                                response_token,
                                SchedulerResponse::Transmission(
                                    SchedulerTransmissionResult::NoAck(
                                        recovered_radio_frame,
                                        instant,
                                    ),
                                ),
                            );

                            (pending_tx_frame, pending_response_token) =
                                self.update_pending_tx(pending_tx_frame, pending_response_token);

                            // If there is a pending TX request, use it, otherwise we want
                            // to wait for a frame
                            let (request, is_tx) = self.next_driver_request(pending_tx_frame);

                            if is_tx {
                                self.driver_request_sender.send(request).await;
                                // Safety: we expect an tx token associated to a TX radio frame
                                response_token = pending_response_token.unwrap();
                                pending_response_token = None;
                                pending_tx_frame = None;
                            } else {
                                self.driver_request_sender.send(request).await;
                                return CsmaSchedulerState::WaitingForFrame;
                            }

                            break 'retransmissions;
                        } else {
                            self.driver_request_sender
                                .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                                    at: Timestamp::BestEffort,
                                    radio_frame: recovered_radio_frame,
                                    cca: true,
                                    channel: None,
                                    fallback_on_nack: retransmissions
                                        != CSMA_MAX_RETRANSMISSIONS - 1,
                                }))
                                .await;
                        }
                    }
                }
                retransmissions += 1;
            }
        }
    }

    pub(super) async fn waiting_for_frame(
        &mut self,
        consumer_token: &mut ConsumerToken,
    ) -> CsmaSchedulerState {
        loop {
            // schedule next
            match select(
                self.request_receiver
                    .receive_request_async(consumer_token, &TaskDirection::Outbound),
                self.driver_event_receiver.receive(),
            )
            .await
            {
                Either::First((response_token, request)) => {
                    match request {
                        SchedulerRequest::Transmission(mpdu_frame) => {
                            self.driver_request_sender
                                .send(Self::initial_driver_tx_request(
                                    mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                                ))
                                .await;
                            return match self.driver_event_receiver.receive().await {
                                DrvSvcEvent::RxWindowEnded(radio_frame) => {
                                    // If RX frame is from back-to-back RX, we deallocate this extra buffer
                                    if let Some(rx_frame) = self.rx_frame.replace(Some(radio_frame))
                                    {
                                        unsafe {
                                            self.buffer_allocator
                                                .deallocate_buffer(rx_frame.into_buffer());
                                        }
                                    }
                                    CsmaSchedulerState::Transmitting(response_token)
                                }
                                // TODO: handle unexpected RX
                                DrvSvcEvent::FrameStarted => todo!(),
                                DrvSvcEvent::Received(_radio_frame, _instant) => todo!(),
                                _ => unreachable!(),
                            };
                        }
                        SchedulerRequest::Command(command) => {
                            match command {
                                #[cfg(feature = "tsch")]
                                // TODO: use tsch feature
                                SchedulerCommand::UseTsch(_tsch_mode, _tsch_cca) => {
                                    // TODO: handle config change, define pub fn in tsch ? set_mode ?
                                    return self.terminate(response_token).await;
                                }
                                #[cfg(feature = "tsch")]
                                SchedulerCommand::SetTschSlotframe(_)
                                | SchedulerCommand::SetTschLink(_) => {
                                    self.handle_tsch_command(command);
                                }
                                _ => unreachable!(),
                            }
                        }
                        // TODO: support UseCsma Command to enable channel switching
                        _ => unreachable!(),
                    }
                }
                Either::Second(DrvSvcEvent::FrameStarted) => {
                    // schedule next RX (because no tx request prending)
                    let back_to_back_rx_frame = match self.rx_frame.take() {
                        Some(rx_frame) => rx_frame,
                        None => Self::allocate_frame(self.buffer_allocator),
                    };
                    self.driver_request_sender
                        .send(DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                            start: Timestamp::BestEffort,
                            radio_frame: back_to_back_rx_frame,
                            channel: None,
                        }))
                        .await;
                    // process current RX
                    match self.driver_event_receiver.receive().await {
                        DrvSvcEvent::Received(radio_frame, instant) => {
                            // Safety: we expect the MAC service to always send a
                            // RX request
                            if let Some((response_token, SchedulerRequest::Reception)) = self
                                .request_receiver
                                .try_receive_request(&TaskDirection::Inbound)
                            {
                                self.request_receiver.received(
                                    response_token,
                                    SchedulerResponse::Reception(radio_frame, instant),
                                );
                                self.rx_frame
                                    .set(Some(Self::allocate_frame(self.buffer_allocator)));
                            } else {
                                // No RX request available, we drop the frame by reusing it for
                                // next RX
                                self.rx_frame
                                    .set(Some(radio_frame.forget_size::<RadioDriverImpl>()));
                            }
                        }
                        DrvSvcEvent::CrcError(radio_frame, _instant) => {
                            self.rx_frame.set(Some(radio_frame));
                        }
                        _ => unreachable!(),
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    fn update_pending_tx(
        &self,
        pending_tx_frame: Option<RadioFrame<RadioFrameSized>>,
        pending_response_token: Option<ResponseToken>,
    ) -> (Option<RadioFrame<RadioFrameSized>>, Option<ResponseToken>) {
        if pending_tx_frame.is_none() {
            match self
                .request_receiver
                .try_receive_request(&TaskDirection::Outbound)
            {
                Some((token, SchedulerRequest::Transmission(mpdu))) => (
                    Some(mpdu.into_radio_frame::<RadioDriverImpl>()),
                    Some(token),
                ),
                None => (None, None),
                _ => unreachable!(),
            }
        } else {
            (pending_tx_frame, pending_response_token)
        }
    }

    fn next_driver_request(
        &self,
        pending_tx_frame: Option<RadioFrame<RadioFrameSized>>,
    ) -> (DrvSvcRequest, bool) {
        match pending_tx_frame {
            Some(radio_frame) => (Self::initial_driver_tx_request(radio_frame), true),
            None => {
                let inbound_frame = self.rx_frame.take().unwrap();
                (
                    DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                        start: Timestamp::BestEffort,
                        radio_frame: inbound_frame,
                        channel: None,
                    }),
                    false,
                )
            }
        }
    }

    fn initial_driver_tx_request(radio_frame: RadioFrame<RadioFrameSized>) -> DrvSvcRequest {
        DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
            at: Timestamp::BestEffort,
            radio_frame,
            cca: true,
            channel: None,
            fallback_on_nack: CSMA_MAX_RETRANSMISSIONS > 0,
        })
    }

    async fn initial(&mut self) -> CsmaSchedulerState {
        match self
            .request_receiver
            .try_receive_request(&TaskDirection::Outbound)
        {
            Some((sched_response_token, request)) => match request {
                SchedulerRequest::Transmission(mpdu_frame) => {
                    self.driver_request_sender
                        .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                            at: Timestamp::BestEffort,
                            radio_frame: mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                            cca: false,
                            channel: None,
                            // First try so we expect to retransmit on NACK
                            fallback_on_nack: true,
                        }))
                        .await;
                    CsmaSchedulerState::Transmitting(sched_response_token)
                }
                #[cfg(feature = "tsch")]
                SchedulerRequest::Command(SchedulerCommand::UseTsch(_, _)) => {
                    self.terminate(sched_response_token).await
                }
                _ => unreachable!(),
            },
            None => {
                let inbound_frame = self.rx_frame.take().unwrap();
                self.driver_request_sender
                    .send(DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                        start: Timestamp::BestEffort,
                        radio_frame: inbound_frame,
                        channel: None,
                    }))
                    .await;
                CsmaSchedulerState::WaitingForFrame
            }
        }
    }

    #[cfg(feature = "tsch")]
    async fn terminate(&self, command_response_token: ResponseToken) -> CsmaSchedulerState {
        // // Handle last operation result
        // self.driver_request_sender
        //     .send(DrvSvcRequest::CompleteThenGoIdle)
        //     .await;
        // // process current RX
        // match self.driver_event_receiver.receive().await {
        //     DrvSvcEvent::Received(radio_frame, instant) => unsafe {
        //         // Safety: we expect the MAC service to always send a
        //         // RX request
        //         if let Some((response_token, SchedulerRequest::Reception)) = self
        //             .request_receiver
        //             .try_receive_request(&TaskDirection::Inbound)
        //         {
        //             self.request_receiver.received(
        //                 response_token,
        //                 SchedulerResponse::Reception(radio_frame, instant),
        //             );
        //         } else {
        //             unsafe {
        //                 self.buffer_allocator
        //                     .deallocate_buffer(radio_frame.into_buffer());
        //             }
        //         }
        //     },
        //     DrvSvcEvent::RxWindowEnded(radio_frame) => unsafe {
        //         unsafe {
        //             self.buffer_allocator
        //                 .deallocate_buffer(radio_frame.into_buffer());
        //         }
        //     },
        //     DrvSvcEvent::CrcError(radio_frame, instant) => {
        //         self.rx_frame.set(Some(radio_frame));
        //     }
        //     _ => unreachable!(),
        // }
        // if let Some(rx_frame) = self.rx_frame.take() {
        //     unsafe {
        //         self.buffer_allocator
        //             .deallocate_buffer(rx_frame.into_buffer());
        //     }
        // }
        self.request_receiver.received(
            // TODO: implement from/into for SchedulerCommandResult
            command_response_token,
            SchedulerResponse::Command(super::SchedulerCommandResult::UseTsch(
                UseTschCommandResult::StartedTsch,
            )),
        );
        CsmaSchedulerState::Terminating(SchedulerState::UsingTsch)
    }
}
