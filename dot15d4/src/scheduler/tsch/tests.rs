//! Integration tests for the TSCH scheduler task.

use core::cell::Cell;
use core::num::NonZero;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{
            Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
            RadioFrame, RadioFrameSized, RadioFrameUnsized,
        },
    },
    timer::{NsDuration, NsInstant},
};
use dot15d4_frame::{
    fields::TschLinkOption,
    mpdu::MpduFrame,
    repr::{MpduRepr, SeqNrRepr},
    MpduWithIes,
};

use crate::driver::{DrvSvcEvent, DrvSvcRequest, Timestamp};
use crate::mac::MacBufferAllocator;
use crate::scheduler::command::pib::PibCommand;
use crate::scheduler::command::tsch::TschCommand;
use crate::scheduler::tests::{
    create_sized_radio_frame, create_test_allocator, create_test_mpdu_with_ack,
    create_unsized_radio_frame, get_scheduler_channel, ActionType, DeterministicRng,
    DriverRequestInfo, FakeDriverConfig, FakeRadioTimer, ResponseType, Sleeping, TestRunner,
    DRIVER_EVENT_CHANNEL, DRIVER_REQUEST_CHANNEL,
};
use crate::scheduler::tsch::{
    TschLink, TschLinkType, TschOperation, TschState, TschTask, INFINITE_DEADLINE,
    TIMESLOT_GUARD_TIME_US,
};
use crate::scheduler::{
    SchedulerAction, SchedulerCommand, SchedulerContext, SchedulerRequest, SchedulerResponse,
    SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition,
};

use super::pib::TschAsn;
use super::TschPib;

// ============================================================================
// Timing Constants (from default TschTimeslotTimings)
// ============================================================================

/// Default timeslot length in microseconds.
const TIMESLOT_LENGTH_US: u64 = 10_000;
/// Default TX offset in microseconds.
const TX_OFFSET_US: u64 = 2120;
/// Default RX offset in microseconds (tx_offset - guard_time/2).
const RX_OFFSET_US: u64 = 1020;
/// Default RX wait in microseconds (guard_time).
const RX_WAIT_US: u64 = 2200;

// ============================================================================
// Role Configuration
// ============================================================================

/// Extended address of the coordinator (used in device-mode tests to indicate
/// that the device has a known coordinator).
const COORD_ADDRESS: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

// ============================================================================
// TestRunner TSCH Extensions
// ============================================================================

impl<'a> TestRunner<'a, TschTask<FakeDriverConfig>> {
    pub fn state(&self) -> &TschState {
        &self.task.state
    }

    pub fn is_idle(&self) -> bool {
        self.task.is_idle()
    }

    pub fn has_pending_operations(&self) -> bool {
        self.task.has_pending_operations()
    }

    pub fn pending_operation_count(&self) -> usize {
        self.task.pending_operation_count()
    }

    pub fn last_asn(&self) -> TschAsn {
        self.context.pib.tsch.asn
    }

    pub fn last_base_time(&self) -> NsInstant {
        self.context.pib.tsch.last_base_time
    }

    pub fn drift_ns(&self) -> i32 {
        self.context.pib.tsch.drift_ns
    }

    pub fn set_timer_time(&mut self, instant: NsInstant) {
        self.context.timer.set_time(instant);
    }

    pub fn advance_timer(&mut self, duration: NsDuration) {
        self.context.timer.advance(duration);
    }

    pub fn terminate(&mut self) {
        self.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::TschCommand(
            TschCommand::UseTsch(false, false),
        )));
        assert!(self.last_response_is_command());
    }

    /// Initialize and enter as coordinator node.
    pub fn enter_as_coordinator(&mut self) {
        self.context.pib.tsch.asn = 0;
        self.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        self.context.pib.tsch.sort_links();
        self.step_entry();
    }

    /// Configure PIB for device node and step_entry.
    pub fn enter_as_device(&mut self) {
        self.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        self.context.pib.tsch.asn = 0;
        self.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        self.step_entry();
    }
}

// ============================================================================
// Schedule Setup Helpers
// ============================================================================

fn setup() -> TestRunner<'static, TschTask<FakeDriverConfig>> {
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

    let task = TschTask::new(&mut context);

    TestRunner::new(context, task)
}

/// Minimal TSCH schedule: one shared advertising link at timeslot 0 in a
/// slotframe of size 100. The link supports TX, RX, Shared, and TimeKeeping.
fn setup_one_link_schedule(pib: &mut TschPib<()>) {
    assert!(pib.create_slotframe(0, 100).is_ok());
    let link = TschLink {
        slotframe_handle: 0,
        timeslot: 0,
        channel_offset: 0,
        link_options: TschLinkOption::Tx
            | TschLinkOption::Rx
            | TschLinkOption::Shared
            | TschLinkOption::TimeKeeping,
        link_type: TschLinkType::Advertising,
        neighbor: None,
        link_advertise: true,
    };
    assert!(pib.add_link(link).is_ok());
}

/// Schedule with separate TX and RX links at different timeslots.
fn setup_two_link_schedule(pib: &mut TschPib<()>) {
    assert!(pib.create_slotframe(0, 100).is_ok());
    // TX link at timeslot 10
    let tx_link = TschLink {
        slotframe_handle: 0,
        timeslot: 10,
        channel_offset: 0,
        link_options: TschLinkOption::Tx | TschLinkOption::Shared,
        link_type: TschLinkType::Advertising,
        neighbor: None,
        link_advertise: true,
    };
    assert!(pib.add_link(tx_link).is_ok());
    // RX link at timeslot 20
    let rx_link = TschLink {
        slotframe_handle: 0,
        timeslot: 20,
        channel_offset: 0,
        link_options: TschLinkOption::Rx | TschLinkOption::TimeKeeping,
        link_type: TschLinkType::Normal,
        neighbor: None,
        link_advertise: false,
    };
    assert!(pib.add_link(rx_link).is_ok());
}

/// Schedule with short slotframe for quick cycling.
fn setup_short_schedule(pib: &mut TschPib<()>) {
    assert!(pib.create_slotframe(0, 5).is_ok());
    let link = TschLink {
        slotframe_handle: 0,
        timeslot: 0,
        channel_offset: 0,
        link_options: TschLinkOption::Tx
            | TschLinkOption::Rx
            | TschLinkOption::Shared
            | TschLinkOption::TimeKeeping,
        link_type: TschLinkType::Advertising,
        neighbor: None,
        link_advertise: true,
    };
    assert!(pib.add_link(link).is_ok());
}

// ============================================================================
// Frame Creation Helpers
// ============================================================================

fn create_test_data_frame(allocator: &MacBufferAllocator) -> MpduFrame {
    create_test_mpdu_with_ack(allocator)
}

fn create_data_radio_frame(
    allocator: &MacBufferAllocator,
    size: u16,
) -> RadioFrame<RadioFrameSized> {
    create_sized_radio_frame(allocator, size)
}

// ============================================================================
// Timing Computation Helpers
// ============================================================================

/// Compute the expected slot start time for a given ASN relative to base.
fn expected_slot_start(base_time: NsInstant, base_asn: TschAsn, target_asn: TschAsn) -> NsInstant {
    let slots_diff = target_asn.saturating_sub(base_asn) as u32;
    base_time + slots_diff * NsDuration::micros(TIMESLOT_LENGTH_US)
}

/// Compute the expected deadline (slot_start - guard_time).
fn expected_deadline(base_time: NsInstant, base_asn: TschAsn, target_asn: TschAsn) -> NsInstant {
    expected_slot_start(base_time, base_asn, target_asn)
        - NsDuration::micros(TIMESLOT_GUARD_TIME_US)
}

/// Compute the expected TX instant (slot_start + tx_offset).
fn expected_tx_instant(slot_start: NsInstant) -> NsInstant {
    slot_start + NsDuration::micros(TX_OFFSET_US)
}

/// Compute the expected RX instant (slot_start + rx_offset).
fn expected_rx_instant(slot_start: NsInstant) -> NsInstant {
    slot_start + NsDuration::micros(RX_OFFSET_US)
}

/// Compute the expected RX window end (rx_instant + rx_wait).
fn expected_rx_window_end(slot_start: NsInstant) -> NsInstant {
    expected_rx_instant(slot_start) + NsDuration::micros(RX_WAIT_US)
}

/// Compute the expected RX framestart (slot_start + tx_offset, peer's TX time).
fn expected_rx_framestart(slot_start: NsInstant) -> NsInstant {
    slot_start + NsDuration::micros(TX_OFFSET_US)
}

// ============================================================================
// State Machine Tests
// ============================================================================

#[cfg(test)]
mod state_tests {
    use super::*;

    #[test]
    fn test_initial_state_is_idle() {
        let mut runner = setup();
        assert!(runner.is_idle());
        assert!(!runner.has_pending_operations());

        runner.terminate();
    }

    #[test]
    fn test_idle_deadline_is_infinite_initially() {
        let mut runner = setup();
        assert_eq!(
            runner.task.peek_deadline(&runner.context),
            INFINITE_DEADLINE
        );
        runner.terminate();
    }

    #[test]
    fn test_entry_transitions_to_idle() {
        let mut runner = setup();
        runner.step_entry();
        let action = runner.last_action().unwrap();

        assert!(runner.is_idle());
        assert!(matches!(
            action,
            ActionType::WaitForTimeoutOrSchedulerRequest
        ));
        runner.terminate();
    }

    #[test]
    fn test_entry_with_schedule_queues_operations() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        assert!(runner.is_idle());
        assert!(runner.has_pending_operations());
        // Coordinator queues beacon + RX
        assert_eq!(runner.pending_operation_count(), 2);

        runner.terminate();
    }

    #[test]
    fn test_entry_deadline_matches_earliest_operation() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        let deadline = runner.task.peek_deadline(&runner.context);
        let expected = expected_deadline(NsInstant::from_ticks(0), 0, 100);
        assert_eq!(deadline, expected);

        runner.terminate();
    }
}

// ============================================================================
// Operation Queue Tests
// ============================================================================

#[cfg(test)]
mod operation_queue_tests {
    use super::*;

    #[test]
    fn test_push_operation_adds_to_queue() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        let op = TschOperation::RxSlot {
            asn: 300,
            channel: Channel::_12,
            slotframe_handle: 0,
        };

        assert!(runner.task.push_operation(op).is_ok());
        assert_eq!(runner.pending_operation_count(), 3);
        runner.terminate();
    }

    #[test]
    fn test_multiple_operations_maintain_order() {
        let mut runner = setup();

        runner
            .task
            .push_operation(TschOperation::RxSlot {
                asn: 200,
                channel: Channel::_12,
                slotframe_handle: 0,
            })
            .unwrap();

        runner
            .task
            .push_operation(TschOperation::RxSlot {
                asn: 100,
                channel: Channel::_12,
                slotframe_handle: 0,
            })
            .unwrap();

        runner
            .task
            .push_operation(TschOperation::RxSlot {
                asn: 150,
                channel: Channel::_12,
                slotframe_handle: 0,
            })
            .unwrap();

        assert_eq!(runner.pending_operation_count(), 3);

        let op1 = runner.task.pop_operation();
        assert!(matches!(op1, TschOperation::RxSlot { asn: 100, .. }));

        runner.terminate();
    }
}

// ============================================================================
// Timing Calculation Tests
// ============================================================================

#[cfg(test)]
mod timing_tests {
    use super::*;

    #[test]
    fn test_expected_slot_start_from_base() {
        let mut runner = setup();
        runner.step_entry();
        let pib = &mut runner.context.pib.tsch;
        let timeslot_length_us = pib.timeslot_length_us();

        assert_eq!(
            pib.expected_slot_start(0, timeslot_length_us),
            pib.last_base_time
        );
        assert_eq!(
            pib.expected_slot_start(1, timeslot_length_us),
            pib.last_base_time + NsDuration::micros(timeslot_length_us)
        );
        assert_eq!(
            pib.expected_slot_start(10, timeslot_length_us),
            pib.last_base_time + NsDuration::micros(10 * timeslot_length_us)
        );

        runner.terminate();
    }

    #[test]
    fn test_asn_at_instant() {
        let mut runner = setup();
        runner.step_entry();

        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(1_000_000_000);
        runner.context.pib.tsch.asn = 0;
        let timeslot_length_us = runner.context.pib.tsch.timeslot_length_us();

        assert_eq!(
            runner
                .context
                .pib
                .tsch
                .asn_at(runner.context.pib.tsch.last_base_time),
            0
        );

        let one_slot =
            runner.context.pib.tsch.last_base_time + NsDuration::micros(timeslot_length_us);
        assert_eq!(runner.context.pib.tsch.asn_at(one_slot), 1);

        let ten_slots =
            runner.context.pib.tsch.last_base_time + NsDuration::micros(10 * timeslot_length_us);
        assert_eq!(runner.context.pib.tsch.asn_at(ten_slots), 10);

        runner.terminate();
    }

    #[test]
    fn test_asn_at_before_base_returns_last_asn() {
        let mut runner = setup();
        runner.step_entry();

        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(1_000_000_000);
        runner.context.pib.tsch.asn = 50;

        assert_eq!(
            runner
                .context
                .pib
                .tsch
                .asn_at(NsInstant::from_ticks(500_000_000)),
            50
        );

        runner.terminate();
    }

    #[test]
    fn test_update_timing_updates_base_and_asn() {
        let mut runner = setup();
        runner.step_entry();

        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.context.pib.tsch.asn = 0;

        runner.context.pib.tsch.update_timing(100);

        assert_eq!(runner.context.pib.tsch.asn, 100);
        let timeslot_length_us = runner.context.pib.tsch.timeslot_length_us();
        assert_eq!(
            runner.context.pib.tsch.last_base_time,
            NsInstant::from_ticks(0) + NsDuration::micros(100 * timeslot_length_us)
        );

        runner.terminate();
    }
}

// ============================================================================
// RX Frame Management Tests
// ============================================================================

#[cfg(test)]
mod rx_frame_tests {
    use super::*;

    #[test]
    fn test_take_and_put_rx_frame() {
        let mut runner = setup();
        runner.step_entry();
        let frame = runner.task.take_rx_frame();
        assert!(frame.is_some());
        assert!(runner.task.take_rx_frame().is_none());
        runner.task.put_rx_frame(frame.unwrap());
        runner.terminate();
    }
}

// ============================================================================
// Beacon Config Tests
// ============================================================================

#[cfg(test)]
mod beacon_config_tests {
    use crate::scheduler::tsch::BeaconConfig;

    #[test]
    fn test_beacon_config_default_is_disabled() {
        let config = BeaconConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.period_secs, 10);
    }

    #[test]
    fn test_beacon_config_new_is_enabled() {
        let config = BeaconConfig::new(5);
        assert!(config.enabled);
        assert_eq!(config.period_secs, 5);
    }

    #[test]
    fn test_beacon_config_disabled() {
        let config = BeaconConfig::disabled();
        assert!(!config.enabled);
        assert_eq!(config.period_secs, 0);
    }

    #[test]
    fn test_beacon_period_ns_conversion() {
        let config = BeaconConfig::new(3);
        assert_eq!(config.period_ns(), 3_000_000_000);
    }
}

// ============================================================================
// Integration Tests: Single TX Operation Flow
// ============================================================================

#[cfg(test)]
mod single_tx_tests {
    use super::*;

    #[test]
    fn test_tx_complete_success_flow() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        assert!(runner.is_idle());
        assert_eq!(runner.pending_operation_count(), 1);

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        assert!(runner.is_idle());
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );

        runner.step_timer_expired();
        assert!(matches!(
            runner.state(),
            TschState::WaitingForTxStart { .. }
        ));
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::SendDriverRequestThenWait
        );
        assert!(runner.last_request_is_tx());
        assert!(!runner.last_tx_is_best_effort());
        assert!(runner.last_request().unwrap().scheduled_instant().is_some());

        let tx_start = NsInstant::from_ticks(1_000_000_000);
        runner.step_driver_event(DrvSvcEvent::TxStarted(tx_start));
        assert!(matches!(runner.state(), TschState::Transmitting { .. }));
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForDriverEvent
        );

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_002_000_000),
            None,
        ));
        assert!(runner.is_idle());
        assert!(runner.last_action_has_response());
        assert!(runner.last_response_is_sent());

        runner.terminate();
    }

    #[test]
    fn test_tx_nack_returns_no_ack_response() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));

        assert!(runner.is_idle());
        assert!(runner.last_response_is_no_ack());
        runner.terminate();
    }

    #[test]
    fn test_tx_scheduled_instant_matches_expected() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        let slot_start_100 = expected_slot_start(NsInstant::from_ticks(0), 0, 100);
        let expected_tx = expected_tx_instant(slot_start_100);

        runner.step_timer_expired();
        assert_eq!(
            runner.last_request().unwrap().scheduled_instant(),
            Some(expected_tx)
        );

        runner.step_driver_event(DrvSvcEvent::TxStarted(expected_tx));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(frame, expected_tx, None));
        runner.terminate();
    }

    #[test]
    fn test_tx_channel_from_hopping_sequence() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        // ASN 100, channel_offset 0, hopping [_26, _12]: (100+0)%2=0 -> _26
        assert_eq!(runner.last_request().unwrap().channel(), Some(Channel::_26));

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_tx_cca_disabled() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        assert!(!runner.last_request().unwrap().cca_enabled());

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_tx_updates_asn_and_base_time() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let initial_bt = runner.last_base_time();
        let initial_asn = runner.last_asn();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        assert_eq!(runner.last_asn(), initial_asn);

        runner.step_timer_expired();
        assert_eq!(runner.last_asn(), 100);
        assert_eq!(
            runner.last_base_time(),
            expected_slot_start(initial_bt, initial_asn, 100)
        );

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_tx_sent_reduces_pending_and_reschedules() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        let after_tx = runner.pending_operation_count();

        runner.step_timer_expired();
        assert!(runner.pending_operation_count() < after_tx);

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        assert!(runner.is_idle());
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: RX Operation Flow
// ============================================================================

#[cfg(test)]
mod rx_operation_tests {
    use super::*;

    fn skip_beacon(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) {
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
    }

    #[test]
    fn test_rx_full_receive_flow() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        runner.step_timer_expired();
        assert!(matches!(runner.state(), TschState::Listening));
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::SendDriverRequestThenWait
        );
        assert!(runner.last_request_is_rx());

        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        assert!(matches!(runner.state(), TschState::Receiving));
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForDriverEvent
        );

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(
            frame,
            NsInstant::from_ticks(2_002_120_000),
        ));
        assert!(runner.is_idle());
        runner.terminate();
    }

    #[test]
    fn test_rx_window_ended_returns_to_idle() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        runner.step_timer_expired();
        assert!(matches!(runner.state(), TschState::Listening));

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        assert!(runner.is_idle());
        assert!(!runner.last_action_has_response());
        runner.terminate();
    }

    #[test]
    fn test_rx_crc_error_returns_to_idle() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        assert!(matches!(runner.state(), TschState::Receiving));

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::CrcError(
            rx_frame,
            NsInstant::from_ticks(2_002_000_000),
        ));
        assert!(runner.is_idle());
        runner.terminate();
    }

    #[test]
    fn test_rx_scheduled_timing_matches_expected() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        let bt = runner.last_base_time();
        let asn = runner.last_asn();

        runner.step_timer_expired();
        let req = runner.last_request().unwrap();
        let slot_start = expected_slot_start(bt, asn, 200);

        assert_eq!(
            req.rx_scheduled_instant(),
            Some(expected_rx_instant(slot_start))
        );
        assert_eq!(
            req.rx_window_end(),
            Some(expected_rx_window_end(slot_start))
        );
        assert_eq!(
            req.expected_rx_framestart(),
            Some(expected_rx_framestart(slot_start))
        );

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        runner.terminate();
    }

    #[test]
    fn test_rx_channel_from_hopping_sequence() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        // RX at ASN 200: (200+0)%2=0 -> _26
        runner.step_timer_expired();
        assert_eq!(runner.last_request().unwrap().channel(), Some(Channel::_26));

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Advertisement / Beacon Flow
// ============================================================================

#[cfg(test)]
mod advertisement_tests {
    use super::*;

    #[test]
    fn test_coordinator_entry_queues_beacon() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        assert_eq!(runner.pending_operation_count(), 2);
        assert!(runner.task.is_coordinator);
        assert!(runner.task.beacon_config.enabled);
        runner.terminate();
    }

    #[test]
    fn test_advertisement_tx_no_response_token() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        runner.step_timer_expired();
        assert!(matches!(
            runner.state(),
            TschState::WaitingForTxStart { .. }
        ));
        assert!(runner.last_request_is_tx());

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_002_120_000),
            None,
        ));

        assert!(runner.is_idle());
        assert!(!runner.last_action_has_response());
        runner.terminate();
    }

    #[test]
    fn test_beacon_sent_updates_last_beacon_time() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        assert!(runner.task.last_beacon_time.is_none());

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let sent_instant = NsInstant::from_ticks(1_002_120_000);
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(frame, sent_instant, None));

        assert_eq!(runner.task.last_beacon_time, Some(sent_instant));
        runner.terminate();
    }

    #[test]
    fn test_beacon_sent_reschedules_next_beacon() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_002_120_000),
            None,
        ));

        assert!(runner.has_pending_operations());
        assert!(runner.pending_operation_count() >= 2);
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Acknowledgement-based Synchronization
// ============================================================================

#[cfg(test)]
mod ack_sync_tests {
    use super::*;

    #[test]
    fn test_ack_positive_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(50i16),
        ));

        assert!(runner.last_response_is_sent());
        assert_eq!(runner.last_base_time(), bt_before - NsDuration::micros(50));
        runner.terminate();
    }

    #[test]
    fn test_ack_negative_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(-30i16),
        ));

        assert_eq!(runner.last_base_time(), bt_before + NsDuration::micros(30));
        runner.terminate();
    }

    #[test]
    fn test_ack_zero_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(0i16),
        ));

        assert_eq!(runner.last_base_time(), bt_before);
        runner.terminate();
    }

    #[test]
    fn test_ack_none_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        assert_eq!(runner.last_base_time(), bt_before);
        runner.terminate();
    }

    #[test]
    fn test_nack_timesync_not_applied() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            Some(100),
        ));

        assert_eq!(runner.last_base_time(), bt_before);
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Frame-based Synchronization
// ============================================================================

#[cfg(test)]
mod frame_sync_tests {
    use super::*;

    fn setup_device_runner() -> TestRunner<'static, TschTask<FakeDriverConfig>> {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.task.init_device(&mut runner.context);
        runner.task.state = TschState::Idle;
        runner
    }

    #[test]
    fn test_device_rx_frame_sync_updates_base_time() {
        let mut runner = setup_device_runner();

        runner.step_timer_expired();
        assert!(matches!(runner.state(), TschState::Listening));

        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        let rx_instant = NsInstant::from_ticks(1_002_120_000);
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(frame, rx_instant));

        assert!(runner.is_idle());
        let expected_base = rx_instant - NsDuration::micros(TX_OFFSET_US);
        assert_eq!(runner.last_base_time(), expected_base);
        assert_eq!(runner.last_asn(), 100);
        runner.terminate();
    }

    #[test]
    fn test_device_rx_calculates_drift() {
        let mut runner = setup_device_runner();

        // First RX at ASN 100
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        let first_rx = NsInstant::from_ticks(1_002_120_000);
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(frame, first_rx));
        assert_eq!(runner.drift_ns(), 0);

        // Second RX at ASN 200, 30us late
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        let second_rx = NsInstant::from_ticks(2_002_120_000 + 30_000);
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(frame, second_rx));

        assert_ne!(runner.drift_ns(), 0);
        runner.terminate();
    }

    #[test]
    fn test_coordinator_rx_does_not_sync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        // Skip beacon
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        // Trigger RX and receive
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(
            frame,
            NsInstant::from_ticks(2_102_200_000),
        ));

        assert!(runner.task.is_coordinator);
        // base_time set by update_timing(200), NOT by sync_asn
        assert_eq!(
            runner.last_base_time(),
            expected_slot_start(NsInstant::from_ticks(0), 0, 200)
        );
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Deadline and Timer
// ============================================================================

#[cfg(test)]
mod deadline_tests {
    use super::*;

    #[test]
    fn test_no_pending_infinite_deadline() {
        let mut runner = setup();
        runner.step_entry();
        assert_eq!(runner.last_deadline().unwrap(), INFINITE_DEADLINE);
        runner.terminate();
    }

    #[test]
    fn test_deadline_includes_guard_time() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        assert_eq!(
            runner.task.peek_deadline(&runner.context),
            expected_deadline(NsInstant::from_ticks(0), 0, 100)
        );
        runner.terminate();
    }

    #[test]
    fn test_deadline_updates_after_completion() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        let first = runner.task.peek_deadline(&runner.context);

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        let second = runner.last_deadline().unwrap();
        assert!(second > first);
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Command Handling
// ============================================================================

#[cfg(test)]
mod command_tests {
    use crate::mac::mlme::{get::GetRequestAttribute, set::SetRequestAttribute};

    use super::*;

    #[test]
    fn test_set_pib_short_address() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Set(SetRequestAttribute::MacShortAddress(0x1234)),
        )));

        assert!(runner.last_response_is_command());
        assert_eq!(runner.context.pib.short_address, 0x1234);
        runner.terminate();
    }

    #[test]
    fn test_get_pib_short_address() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.context.pib.short_address = 0xABCD;

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Get(GetRequestAttribute::MacShortAddress),
        )));

        assert!(runner.last_response_is_command());
        runner.terminate();
    }

    #[test]
    fn test_use_tsch_false_switches_to_csma() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::TschCommand(
            TschCommand::UseTsch(false, false),
        )));

        assert_eq!(runner.last_action().unwrap(), ActionType::Completed);
        assert!(runner.last_response_is_command());
    }
}

// ============================================================================
// Integration Tests: Multiple Operations and Priority
// ============================================================================

#[cfg(test)]
mod multi_operation_tests {
    use super::*;

    #[test]
    fn test_tx_wins_over_rx_at_same_asn() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        runner.task.init_device(&mut runner.context);
        runner.task.state = TschState::Idle;

        assert_eq!(runner.pending_operation_count(), 1); // RX at ASN 100

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        runner.step_timer_expired();
        assert!(matches!(
            runner.state(),
            TschState::WaitingForTxStart { .. }
        ));
        assert!(runner.last_request_is_tx());

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_short_slotframe_cycles() {
        let mut runner = setup();
        setup_short_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        let deadline = runner.task.peek_deadline(&runner.context);
        assert_eq!(deadline, expected_deadline(NsInstant::from_ticks(0), 0, 5));
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Action Type Verification
// ============================================================================

#[cfg(test)]
mod action_type_tests {
    use super::*;

    fn skip_beacon(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) {
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
    }

    #[test]
    fn test_entry_action_type() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );
        runner.terminate();
    }

    #[test]
    fn test_tx_scheduling_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::SendDriverRequestThenWait
        );

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_tx_started_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForDriverEvent
        );

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_rx_scheduling_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);
        runner.step_timer_expired();

        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::SendDriverRequestThenWait
        );

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        runner.terminate();
    }

    #[test]
    fn test_frame_started_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForDriverEvent
        );

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Received(
            frame,
            NsInstant::from_ticks(2_002_120_000),
        ));
        runner.terminate();
    }

    #[test]
    fn test_sent_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );
        runner.terminate();
    }

    #[test]
    fn test_set_pib_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Set(crate::mac::mlme::set::SetRequestAttribute::MacShortAddress(
                0x1234,
            )),
        )));

        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::SelectDriverEventOrRequest
        );
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Full Cycle Scenarios
// ============================================================================

#[cfg(test)]
mod full_cycle_tests {
    use super::*;

    #[test]
    fn test_coordinator_beacon_rx_tx_cycle() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        // Beacon at ASN 100
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        assert!(runner.is_idle());

        // RX at ASN 200
        runner.step_timer_expired();
        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        assert!(runner.is_idle());

        // TX request
        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        assert!(runner.last_request_is_tx());

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
        ));
        assert!(runner.is_idle());
        assert!(runner.last_response_is_sent());
        runner.terminate();
    }

    #[test]
    fn test_multiple_empty_rx_windows() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        // Skip beacon
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        runner.step_timer_expired();
        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        assert!(runner.is_idle());
        assert!(runner.has_pending_operations());

        runner.step_timer_expired();
        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        assert!(runner.is_idle());
        assert!(runner.has_pending_operations());
        runner.terminate();
    }

    #[test]
    fn test_channel_alternates_with_asn() {
        let mut runner = setup();
        setup_short_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();

        // ASN 5: (5+0)%2=1 -> _12
        runner.step_timer_expired();
        assert_eq!(runner.last_request().unwrap().channel(), Some(Channel::_12));

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(50_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(51_000_000),
            None,
        ));

        // ASN 10: (10+0)%2=0 -> _26
        runner.step_timer_expired();
        assert_eq!(runner.last_request().unwrap().channel(), Some(Channel::_26));

        let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
        runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: TX Request During Idle
// ============================================================================

#[cfg(test)]
mod idle_request_tests {
    use super::*;

    #[test]
    fn test_tx_request_updates_deadline() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        runner.task.init_device(&mut runner.context);
        runner.task.state = TschState::Idle;

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        assert_eq!(
            runner.last_deadline().unwrap(),
            expected_deadline(NsInstant::from_ticks(0), 0, 100)
        );

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_tx_request_idle_action() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        runner.task.init_device(&mut runner.context);
        runner.task.state = TschState::Idle;

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );
        assert!(runner.is_idle());

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
        runner.terminate();
    }

    #[test]
    fn test_nack_response_type() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert!(runner.last_response_is_no_ack());
        runner.terminate();
    }

    #[test]
    fn test_large_positive_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(800i16),
        ));

        assert_eq!(runner.last_base_time(), bt_before - NsDuration::micros(800));
        runner.terminate();
    }

    #[test]
    fn test_large_negative_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(-500i16),
        ));

        assert_eq!(runner.last_base_time(), bt_before + NsDuration::micros(500));
        runner.terminate();
    }
}

// ============================================================================
// Clock Drift Correction Tests
// ============================================================================

#[cfg(test)]
mod clock_drift_tests {
    use super::*;

    const RMARKER_OFFSET_NS: u64 = (TX_OFFSET_US) * 1_000;

    fn rmarker_for_slot_start(slot_start_ns: u64) -> NsInstant {
        NsInstant::from_ticks(slot_start_ns + RMARKER_OFFSET_NS)
    }

    /// Helper: compute the expected slot start with drift applied.
    fn expected_slot_start_with_drift(
        base_time: NsInstant,
        base_asn: TschAsn,
        target_asn: TschAsn,
        drift_ns: i32,
    ) -> NsInstant {
        let adjusted_slot_ns = (TIMESLOT_LENGTH_US * 1_000) as i64 - drift_ns as i64;
        let slots_diff = target_asn.saturating_sub(base_asn) as u64;
        base_time + slots_diff as u32 * NsDuration::nanos(adjusted_slot_ns as u64)
    }

    #[test]
    fn test_first_sync_does_not_update_drift() {
        let mut pib: TschPib<()> = TschPib::new();
        assert_eq!(pib.drift_ns, 0);
        assert!(pib.last_rx.is_none());

        pib.sync_asn(100, rmarker_for_slot_start(1_000_000_000));

        assert_eq!(pib.drift_ns, 0);
        assert!(pib.last_rx.is_some());
    }

    #[test]
    fn test_positive_drift_clock_faster() {
        let mut pib: TschPib<()> = TschPib::new();

        // First sync at ASN 100
        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));
        assert_eq!(pib.drift_ns, 0);

        // Second sync at ASN 300, with 60µs less elapsed than expected.
        // delta_asn = 200, expected = 200 * 10_000µs = 2_000_000µs
        // elapsed = 2_000_000µs - 60µs = 1_999_940µs
        // drift = 60_000ns / 200 = 300 ns/slot
        let slot_start_2: u64 = slot_start_1 + 1_999_940_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns, 300);
    }

    #[test]
    fn test_negative_drift_clock_slower() {
        let mut pib: TschPib<()> = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // 60µs more elapsed than expected -> drift = -(60_000 / 200) = -300
        let slot_start_2: u64 = slot_start_1 + 2_000_060_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns, -300);
    }

    #[test]
    fn test_zero_drift_synchronized() {
        let mut pib: TschPib<()> = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // Elapsed exactly matches expected: 200 * 10_000µs = 2_000_000µs
        let slot_start_2: u64 = slot_start_1 + 2_000_000_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns, 0);
    }

    #[test]
    fn test_drift_overwritten_on_each_sync() {
        let mut pib: TschPib<()> = TschPib::new();

        // First pair: establish +300 drift
        pib.sync_asn(100, rmarker_for_slot_start(1_000_000_000));
        pib.sync_asn(300, rmarker_for_slot_start(1_000_000_000 + 1_999_940_000));
        assert_eq!(pib.drift_ns, 300);

        // Second pair: now clocks are perfectly aligned -> drift becomes 0
        // last_rx is (slot_start of ASN 300, 300)
        // New sync at ASN 500: delta = 200, expect 2_000_000_000ns
        let last_slot_start = 1_000_000_000 + 1_999_940_000;
        pib.sync_asn(500, rmarker_for_slot_start(last_slot_start + 2_000_000_000));
        assert_eq!(pib.drift_ns, 0);
    }

    #[test]
    fn test_drift_small_delta_asn() {
        let mut pib: TschPib<()> = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // delta_asn = 5, expected = 50_000µs = 50_000_000ns
        // elapsed = 50_000_000 - 2_500 = 49_997_500ns -> drift = 2500/5 = 500
        let slot_start_2: u64 = slot_start_1 + 49_997_500;
        pib.sync_asn(105, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns, 500);
    }

    #[test]
    fn test_expected_slot_start_zero_drift() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = 0;

        let result = pib.expected_slot_start(200, TIMESLOT_LENGTH_US);
        let expected = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 100, 200);
        assert_eq!(result, expected);
    }

    #[test]
    fn test_expected_slot_start_positive_drift() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = 300;

        let result = pib.expected_slot_start(200, TIMESLOT_LENGTH_US);
        let nominal = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 100, 200);

        // 100 slots * 300ns/slot = 30_000ns earlier than nominal
        assert_eq!(result, nominal - NsDuration::nanos(30_000));
        assert_eq!(
            result,
            expected_slot_start_with_drift(NsInstant::from_ticks(1_000_000_000), 100, 200, 300,)
        );
    }

    #[test]
    fn test_expected_slot_start_negative_drift() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = -300;

        let result = pib.expected_slot_start(200, TIMESLOT_LENGTH_US);
        let nominal = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 100, 200);

        // 100 slots * 300ns/slot = 30_000ns later than nominal
        assert_eq!(result, nominal + NsDuration::nanos(30_000));
    }

    #[test]
    fn test_drift_accumulates_over_distance() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 0;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = 100;

        // 1000 slots ahead: 1000 * 100ns = 100µs total correction
        let result = pib.expected_slot_start(1000, TIMESLOT_LENGTH_US);
        let nominal = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 0, 1000);
        assert_eq!(result, nominal - NsDuration::nanos(100_000));
    }

    #[test]
    fn test_expected_slot_start_same_asn() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = 500; // drift should not matter for zero distance

        let result = pib.expected_slot_start(100, TIMESLOT_LENGTH_US);
        assert_eq!(result, NsInstant::from_ticks(1_000_000_000));
    }

    #[test]
    fn test_update_timing_uses_drift() {
        let mut pib: TschPib<()> = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.drift_ns = 300;

        pib.update_timing(200);

        let expected_base =
            expected_slot_start_with_drift(NsInstant::from_ticks(1_000_000_000), 100, 200, 300);
        assert_eq!(pib.asn, 200);
        assert_eq!(pib.last_base_time, expected_base);
    }
}
