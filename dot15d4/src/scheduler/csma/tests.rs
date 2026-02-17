//! Integration tests for the CSMA-CA scheduler task.

use dot15d4_driver::timer::NsInstant;

use crate::driver::DrvSvcEvent;
use crate::scheduler::csma::task::CsmaState;
use crate::scheduler::tests::{
    create_sized_radio_frame, create_test_mpdu_with_ack, ActionType, DeterministicRng,
    FakeDriverConfig, TestRunner,
};
use crate::scheduler::tests::{create_test_allocator, FakeRadioTimer};
use crate::scheduler::SchedulerRequest;

use crate::driver::Timestamp;
use crate::scheduler::tests::create_unsized_radio_frame;

use super::task::Backoff;
use super::CsmaTask;

impl<'a> TestRunner<'a, CsmaTask<FakeDriverConfig>> {
    pub fn state(&self) -> &CsmaState {
        &self.task.state
    }

    pub fn backoff(&self) -> &Backoff {
        &self.task.backoff
    }

    pub fn tx_retries(&self) -> u8 {
        self.task.tx_retries
    }

    pub fn base_time(&self) -> NsInstant {
        self.task.base_time
    }

    pub fn set_min_be(&mut self, value: u8) {
        self.context.pib.min_be = value;
        self.task.backoff = Backoff::new(value);
    }
}

#[cfg(test)]
mod single_tx_tests {

    use core::cell::Cell;

    use dot15d4_driver::radio::config::Channel;

    use crate::driver::{DrvSvcRequest, DrvSvcTaskRx};
    use crate::scheduler::csma::CsmaTask;
    use crate::scheduler::tests::{
        get_scheduler_channel, FakeDriverConfig, DRIVER_EVENT_CHANNEL, DRIVER_REQUEST_CHANNEL,
    };
    use crate::scheduler::SchedulerContext;

    use super::*;

    fn setup() -> TestRunner<'static, CsmaTask<FakeDriverConfig>> {
        let timer = FakeRadioTimer::new();
        let allocator = create_test_allocator();

        static mut RNG: DeterministicRng = DeterministicRng {
            values: vec![],
            index: Cell::new(0),
            call_count: Cell::new(0),
            default_value: 1,
        };
        let rng = unsafe { &mut RNG };
        rng.values = vec![1];
        rng.reset_index();
        rng.reset_count();

        let address = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

        let channel = get_scheduler_channel();
        let request_receiver = channel.receiver();
        let driver_request_sender = DRIVER_REQUEST_CHANNEL.sender();
        let driver_event_receiver = DRIVER_EVENT_CHANNEL.receiver();

        let mut context = SchedulerContext::new(
            allocator,
            rng,
            timer,
            &address,
            request_receiver,
            driver_request_sender,
            driver_event_receiver,
        );

        let task = CsmaTask::new(Channel::_12, &mut context);

        TestRunner::new(context, task)
    }

    #[test]
    fn test_tx_success_first_try() {
        let mut runner = setup();

        runner.step_entry();
        assert!(matches!(runner.state(), CsmaState::Listening));

        let mpdu = create_test_mpdu_with_ack(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        assert!(matches!(runner.state(), CsmaState::WaitingForTxStart));
        assert!(runner.last_tx_is_best_effort());

        // Simulate driver returning RX frame before starting TX
        let rx_ended = runner.create_rx_window_ended_event();
        runner.step_driver_event(rx_ended);

        let instant = NsInstant::from_ticks(1000);
        runner.step_driver_event(DrvSvcEvent::TxStarted(instant));
        assert!(matches!(runner.state(), CsmaState::Transmitting));
        assert_eq!(runner.base_time(), instant);

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        let sent_instant = NsInstant::from_ticks(2000);
        runner.step_driver_event(DrvSvcEvent::Sent(frame, sent_instant, None));

        assert!(matches!(runner.state(), CsmaState::Listening));
        assert!(runner.last_response_is_sent());
        assert_eq!(runner.base_time(), sent_instant);
    }

    #[test]
    fn test_tx_after_single_cca_failure() {
        let mut runner = setup();
        runner.set_max_csma_backoffs(4);

        runner.step_entry();
        let mpdu = create_test_mpdu_with_ack(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // RX window ended before CCA result
        let rx_ended = runner.create_rx_window_ended_event();
        runner.step_driver_event(rx_ended);

        let cca_instant = NsInstant::from_ticks(500);
        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(frame, cca_instant));

        assert!(matches!(runner.state(), CsmaState::WaitingForTxStart));
        assert_eq!(runner.backoff().nb, 1);
        assert_eq!(runner.base_time(), cca_instant);

        let instant = NsInstant::from_ticks(1000);
        runner.step_driver_event(DrvSvcEvent::TxStarted(instant));
        assert!(matches!(runner.state(), CsmaState::Transmitting));

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(frame, NsInstant::from_ticks(2000), None));
        assert!(runner.last_response_is_sent());
    }

    #[test]
    fn test_tx_channel_access_failure() {
        let mut runner = setup();
        runner.set_max_csma_backoffs(1);

        runner.step_entry();
        let mpdu = create_test_mpdu_with_ack(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // RX window ended before CCA result
        let rx_ended = runner.create_rx_window_ended_event();
        runner.step_driver_event(rx_ended);

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(frame, NsInstant::from_ticks(500)));
        assert_eq!(runner.backoff().nb, 1);

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(frame, NsInstant::from_ticks(1000)));

        assert!(matches!(runner.state(), CsmaState::Listening));
        assert!(runner.last_response_is_channel_access_failure());
    }

    #[test]
    fn test_tx_max_retries_exceeded() {
        let mut runner = setup();
        runner.set_max_frame_retries(1);

        runner.step_entry();
        let mpdu = create_test_mpdu_with_ack(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // RX window ended before TX start
        let rx_ended = runner.create_rx_window_ended_event();
        runner.step_driver_event(rx_ended);

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1000)));

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1500),
            Some(DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                start: Timestamp::BestEffort,
                radio_frame: rx_frame,
                channel: None,
                rx_window_end: None,
                expected_rx_framestart: None,
            })),
            None,
        ));
        assert_eq!(runner.tx_retries(), 1);

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(2000)));

        let frame = create_sized_radio_frame(&runner.context.buffer_allocator, 20);
        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(2500),
            Some(DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
                start: Timestamp::BestEffort,
                radio_frame: rx_frame,
                channel: None,
                rx_window_end: None,
                expected_rx_framestart: None,
            })),
            None,
        ));

        assert!(matches!(runner.state(), CsmaState::Listening));
        assert!(runner.last_response_is_no_ack());
    }
}
