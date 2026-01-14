#![allow(unused_imports)]
pub mod asn;
pub mod beacon;
pub mod runner;
pub mod schedule;

use core::cell::Cell;
use heapless::Vec;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{Address, ExtendedAddress, PanId, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::{fields::TschLinkOption, mpdu::MpduFrame, repr::mpdu_repr};
use dot15d4_util::{
    allocator::IntoBuffer,
    sync::{select, ConsumerToken, Either, ResponseToken},
};

use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, DrvSvcTaskTx, Timestamp},
    mac::mlme::tsch::TschScheduleOperation,
    scheduler::{SchedulerResponse, SchedulerTransmissionResult, TaskDirection},
};

use self::{
    beacon::EnhancedBeaconBuilder,
    runner::{TschDeviceMode, TschOperation},
    schedule::{TschAsn, TschLink, TschLinkType},
};

pub use self::runner::TschState;

use super::{SchedulerCommand, SchedulerService, SchedulerState};

const MAX_SLOTFRAMES: usize = 1;
const MAX_LINKS: usize = 1;
const MAX_OPERATIONS: usize = 5;

// TODO: configurable Coordinator MAC address
const COORD_MAC_ADDR: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0xfe, 0xd6, 0x1f, 0xaf, 0x7d, 0x5a, 0x36, 0xfc,
]));

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    pub(super) async fn run_tsch(
        &mut self,
        mode: TschDeviceMode,
        mut consumer_token: ConsumerToken,
    ) -> (SchedulerState, ConsumerToken) {
        let (last_base_time, last_asn, is_coordinator) = match mode {
            TschDeviceMode::Device(asn, instant) => (instant, asn, false),
            TschDeviceMode::Coordinator(instant) => (instant, 0, true),
        };
        self.tsch_state.last_base_time = last_base_time;
        self.tsch_state.last_asn = last_asn;

        if is_coordinator {
            self.init_beacon_frame();
        }

        let mut next_deadline = self.next_deadline();

        loop {
            match select::select(
                // Waiting for next pending operation deadline
                unsafe { self.timer.wait_until(next_deadline) },
                // Waiting for next Scheduler Transmission request
                self.request_receiver
                    .receive_request_async(&mut consumer_token, &TaskDirection::Outbound),
            )
            .await
            {
                Either::First(_timer_result) => {
                    // Timer expired, we proceed with scheduling the operation that triggered
                    // the timeout.
                    let (new_deadline, instant, operation) = self.next_operation();
                    match operation {
                        TschOperation::TxSlot(mpdu_frame, _, channel, cca, response_token) => {
                            self.tx_slot(
                                mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                                instant,
                                channel,
                                cca,
                                response_token,
                            )
                            .await
                        }
                        TschOperation::RxSlot(_, channel, response_token) => {
                            self.rx_slot(instant, channel, response_token).await
                        }
                        TschOperation::AdvertisementSlot(asn, channel) => {
                            self.advertisement_slot(instant, asn, channel).await
                        }
                        TschOperation::Idle => {}
                    }
                    next_deadline = new_deadline;
                }
                Either::Second((response_token, request)) => {
                    // We received a TX request before we wake up. We must update next operation in
                    // case the new one is due earlier than the current next operation
                    next_deadline =
                        self.queue_scheduler_request(request, response_token, self.timer.now());
                }
            }
        }
    }

    fn init_beacon_frame(&mut self) {
        let radio_frame = Self::allocate_frame(self.buffer_allocator);

        let beacon_frame = self.tsch_state.beacon_builder.build_enhanced_beacon(
            &self,
            radio_frame,
            &COORD_MAC_ADDR,
            &self.pib.pan_id,
        );
        if let Some(beacon_frame) = beacon_frame {
            self.tsch_state.beacon_frame.set(Some(beacon_frame));
        } else {
            panic!("Enhanced beacon could not be initialized");
        }
    }

    async fn tx_slot(
        &self,
        radio_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
        channel: Channel,
        cca: bool,
        response_token: ResponseToken,
    ) {
        self.driver_request_sender
            .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                at: Timestamp::Scheduled(instant),
                radio_frame,
                cca,
                channel: Some(channel),
                // First try so we expect to retransmit on NACK
                fallback_on_nack: false,
            }))
            .await;
        match self.driver_event_receiver.receive().await {
            DrvSvcEvent::TxStarted(_) => {}
            _ => unreachable!(),
        };
        let scheduler_response = match self.driver_event_receiver.receive().await {
            DrvSvcEvent::Nack(radio_frame, instant, _drv_svc_request) => {
                // TODO: reschedule
                SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(
                    radio_frame,
                    instant,
                ))
            }
            DrvSvcEvent::Sent(radio_frame, instant) => {
                SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(
                    radio_frame.forget_size::<RadioDriverImpl>(),
                    instant,
                ))
            }
            _ => unreachable!(),
        };
        self.request_receiver
            .received(response_token, scheduler_response);
    }

    async fn rx_slot(&self, instant: NsInstant, channel: Channel, response_token: ResponseToken) {
        // TODO: safety: check if really available because previous slot may be used for RX also
        // and we may not have finished RX when scheduling this succesive RX
        let radio_frame = self.rx_frame.take().unwrap();
        self.driver_request_sender
            .send(DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                start: Timestamp::Scheduled(instant),
                radio_frame,
                channel: Some(channel),
            }))
            .await;
        match self.driver_event_receiver.receive().await {
            DrvSvcEvent::FrameStarted => {
                match self.driver_event_receiver.receive().await {
                    DrvSvcEvent::Received(radio_frame, instant) => {
                        let scheduler_response = SchedulerResponse::Reception(radio_frame, instant);
                        self.request_receiver
                            .received(response_token, scheduler_response);
                        self.rx_frame
                            .set(Some(Self::allocate_frame(self.buffer_allocator)));
                    }
                    DrvSvcEvent::CrcError(radio_frame, _) => self.rx_frame.set(Some(radio_frame)),
                    _ => unreachable!(),
                };
            }
            DrvSvcEvent::RxWindowEnded(radio_frame) => {
                self.rx_frame.set(Some(radio_frame));
            }
            _ => unreachable!(),
        };
    }

    async fn advertisement_slot(&self, instant: NsInstant, asn: TschAsn, channel: Channel) {
        let radio_frame = self.tsch_state.beacon_frame.take().unwrap();

        let updated_frame = self
            .tsch_state
            .beacon_builder
            .update_beacon(radio_frame, asn)
            .unwrap();

        self.driver_request_sender
            .send(DrvSvcRequest::CompleteThenStartTx(DrvSvcTaskTx {
                at: Timestamp::Scheduled(instant),
                radio_frame: updated_frame,
                cca: false,
                channel: Some(channel),
                fallback_on_nack: false,
            }))
            .await;
        match self.driver_event_receiver.receive().await {
            DrvSvcEvent::TxStarted(_) => {}
            _ => unreachable!(),
        };
        match self.driver_event_receiver.receive().await {
            DrvSvcEvent::Sent(beacon_frame, _instant) => {
                // set back beacon frame
                self.tsch_state.beacon_frame.set(Some(beacon_frame));
            }
            _ => unreachable!(),
        }
    }

    pub(super) fn handle_tsch_command(&mut self, command: SchedulerCommand) {
        match command {
            SchedulerCommand::SetTschSlotframe(request) => match request.operation {
                TschScheduleOperation::Add => {}
                _ => todo!(),
            },
            SchedulerCommand::SetTschLink(_set_link_request) => todo!(),
            _ => unreachable!(),
        }
    }
}
