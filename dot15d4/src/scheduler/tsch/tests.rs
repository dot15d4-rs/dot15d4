//! Integration tests for the TSCH scheduler task.

#![cfg_attr(
    all(feature = "_fuzz-harness", not(test)),
    allow(
        dead_code,
        unused_imports,
        unused_variables,
        unused_mut,
        unused_attributes,
        non_snake_case,
        clippy::all
    )
)]

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{Address, ExtendedAddress, RadioFrame, RadioFrameSized},
    },
    timer::{NsDuration, NsInstant},
};
use dot15d4_frame::{
    fields::{TschLinkOption, TschTimeslotTimings},
    mpdu::MpduFrame,
};
use dot15d4_util::allocator::IntoBuffer;

use crate::driver::DrvSvcEvent;
use crate::mac::neighbors::NeighborId;
use crate::mac::MacBufferAllocator;
use crate::scheduler::command::pib::PibCommand;
use crate::scheduler::command::tsch::{TschCommand, TschRoleConfig};
use crate::scheduler::command::{CsmaCommandResult, UseCsmaResult};
use crate::scheduler::tests::{
    create_test_allocator, create_test_mpdu_with_ack, create_test_rng, create_unsized_radio_frame,
    driver_event_channel, driver_request_channel, get_scheduler_channel, ActionType,
    FakeDriverConfig, FakeRadioTimer, TestRunner,
};
use crate::scheduler::tsch::{
    TschCsmaBackoff, TschLink, TschLinkType, TschOperation, TschState, TschTask, TxOrigin,
    INFINITE_DEADLINE, TIMESLOT_GUARD_TIME_US,
};
use crate::scheduler::SchedulerCommandResult;
use crate::scheduler::{SchedulerCommand, SchedulerContext, SchedulerRequest, SchedulerResponse};

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

    /// Return `(be, nb)` of the pending TX operation's TSCH CSMA-CA backoff
    /// state, if any.
    pub fn pending_tx_backoff(&self) -> Option<(u8, u8)> {
        self.task.pending_operations.iter().find_map(|op| match op {
            TschOperation::TxSlot { backoff, .. } => Some((backoff.be, backoff.nb)),
            _ => None,
        })
    }

    /// Return `(be, nb, backoff_remaining)` of the pending TX operation's TSCH
    /// CSMA-CA backoff state, if any. `backoff_remaining` is the number of
    /// shared TX links the retransmission still has to skip.
    pub fn pending_tx_backoff_full(&self) -> Option<(u8, u8, u16)> {
        self.task.pending_operations.iter().find_map(|op| match op {
            TschOperation::TxSlot { backoff, .. } => {
                Some((backoff.be, backoff.nb, backoff.backoff_remaining))
            }
            _ => None,
        })
    }

    /// Return the ASN of the pending TX operation, if any.
    pub fn pending_tx_asn(&self) -> Option<TschAsn> {
        self.task.pending_operations.iter().find_map(|op| match op {
            TschOperation::TxSlot { asn, .. } => Some(*asn),
            _ => None,
        })
    }

    /// Return the ASN of the pending advertisement operation, if any.
    pub fn pending_advertisement_asn(&self) -> Option<TschAsn> {
        self.task.pending_operations.iter().find_map(|op| match op {
            TschOperation::AdvertisementSlot { asn, .. } => Some(*asn),
            _ => None,
        })
    }

    pub fn last_asn(&self) -> TschAsn {
        self.context.pib.tsch.asn
    }

    pub fn last_base_time(&self) -> NsInstant {
        self.context.pib.tsch.last_base_time
    }

    pub fn drift_ns(&self) -> i32 {
        self.context.pib.tsch.drift_ns()
    }

    pub fn set_timer_time(&mut self, instant: NsInstant) {
        self.context.timer.set_time(instant);
    }

    pub fn advance_timer(&mut self, duration: NsDuration) {
        self.context.timer.advance(duration);
    }

    pub fn terminate(&mut self) {
        while let Some(op) = self.task.pending_operations.pop() {
            if let TschOperation::TxSlot { mpdu, origin, .. } = op {
                // Safety: the buffer was allocated from this allocator.
                unsafe {
                    self.context
                        .buffer_allocator
                        .deallocate_buffer(mpdu.into_buffer())
                };
                if let TxOrigin::Mac(response_token) = origin {
                    self.context.request_receiver.received(
                        response_token,
                        SchedulerResponse::Command(SchedulerCommandResult::CsmaCommand(
                            CsmaCommandResult::UseCsma(UseCsmaResult::Success),
                        )),
                    );
                }
            }
        }
        self.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::TschCommand(
            TschCommand::UseTsch {
                enabled: false,
                cca: false,
                role: Default::default(),
            },
        )));
        assert!(self.last_response_is_command());
    }

    /// Initialize and enter as coordinator node (time root, 9 s beacon).
    pub fn enter_as_coordinator(&mut self) {
        self.context.pib.tsch.role = TschRoleConfig::time_root(9);
        self.context.pib.tsch.asn = 0;
        self.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        self.context.pib.tsch.sort_links();
        self.step_entry();
    }

    /// Intern `addr_le` as a neighbor, pin it and make it this node's time
    /// source.
    pub fn set_time_source(&mut self, addr_le: &[u8; 8]) -> NeighborId {
        let id = self
            .context
            .pib
            .neighbors
            .intern(
                &Address::Extended(ExtendedAddress::new(&addr_le[..])),
                NsInstant::from_ticks(0),
            )
            .unwrap();
        self.context.pib.neighbors.pin(id);
        self.context.pib.tsch.time_source = Some(id);
        id
    }

    /// Configure PIB for device node and step_entry.
    pub fn enter_as_device(&mut self) {
        self.context.pib.tsch.role = TschRoleConfig::device();
        self.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        self.context.pib.tsch.asn = 0;
        self.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        self.step_entry();
    }
}


/// Like `create_test_data_frame`, but with deterministic extended source
/// and destination addresses (little-endian bytes).
fn create_addressed_data_frame(
    allocator: &MacBufferAllocator,
    src_le: &[u8; 8],
    dst_le: &[u8; 8],
) -> MpduFrame {
    let mut mpdu = create_test_mpdu_with_ack(allocator);
    let mut writer = mpdu.writer().parse_addressing_mut().unwrap();
    let mut addressing = writer.try_addressing_fields_mut().unwrap();
    addressing
        .try_src_address_mut()
        .unwrap()
        .try_set(&Address::Extended(ExtendedAddress::new(&src_le[..])))
        .unwrap();
    addressing
        .try_dst_address_mut()
        .unwrap()
        .try_set(&Address::Extended(ExtendedAddress::new(&dst_le[..])))
        .unwrap();
    drop(addressing);
    drop(writer);
    mpdu
}

// ============================================================================
// Schedule Setup Helpers
// ============================================================================

fn setup() -> TestRunner<'static, TschTask<FakeDriverConfig>> {
    let timer = FakeRadioTimer::new();
    let allocator = create_test_allocator();
    let rng = create_test_rng();

    let address = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    let channel = get_scheduler_channel();
    let request_receiver = channel.receiver();
    let driver_request_sender = driver_request_channel().sender();
    let driver_event_receiver = driver_event_channel().receiver();

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
fn setup_one_link_schedule(pib: &mut TschPib) {
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
fn setup_two_link_schedule(pib: &mut TschPib) {
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
fn setup_short_schedule(pib: &mut TschPib) {
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

/// Schedule with a single shared TX-only link at timeslot 0 (no RX links).
///
/// Used by TSCH CSMA-CA tests that step through several TX attempts: with no
/// RX links, every timer expiry pops the TX operation, without interleaved
/// RX slots between retransmission attempts.
fn setup_shared_tx_only_schedule(pib: &mut TschPib) {
    assert!(pib.create_slotframe(0, 100).is_ok());
    let link = TschLink {
        slotframe_handle: 0,
        timeslot: 0,
        channel_offset: 0,
        link_options: TschLinkOption::Tx | TschLinkOption::Shared,
        link_type: TschLinkType::Advertising,
        neighbor: None,
        link_advertise: true,
    };
    assert!(pib.add_link(link).is_ok());
}

/// Schedule with a dedicated (non-shared) TX link at timeslot 10.
/// Used to test non-shared link behavior where NACK is reported directly.
fn setup_dedicated_tx_link_schedule(pib: &mut TschPib) {
    assert!(pib.create_slotframe(0, 100).is_ok());
    // Dedicated TX link (not shared): NACK should produce NoAck response
    let tx_link = TschLink {
        slotframe_handle: 0,
        timeslot: 10,
        channel_offset: 0,
        link_options: TschLinkOption::Tx,
        link_type: TschLinkType::Advertising,
        neighbor: None,
        link_advertise: true,
    };
    assert!(pib.add_link(tx_link).is_ok());
    // RX link at timeslot 50
    let rx_link = TschLink {
        slotframe_handle: 0,
        timeslot: 50,
        channel_offset: 0,
        link_options: TschLinkOption::Rx | TschLinkOption::TimeKeeping,
        link_type: TschLinkType::Normal,
        neighbor: None,
        link_advertise: false,
    };
    assert!(pib.add_link(rx_link).is_ok());
}

/// Schedule with a single dedicated (non-shared) TX-only link at timeslot 0
/// (no RX links).
///
/// Counterpart to [`setup_shared_tx_only_schedule`] for dedicated-link
/// retransmission tests: with no RX link, every timer expiry pops the TX
/// operation, so consecutive retransmission attempts are not interleaved with
/// RX slots.
fn setup_dedicated_tx_only_schedule(pib: &mut TschPib) {
    assert!(pib.create_slotframe(0, 100).is_ok());
    let link = TschLink {
        slotframe_handle: 0,
        timeslot: 0,
        channel_offset: 0,
        link_options: TschLinkOption::Tx,
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

/// Create a well-formed data radio frame, as handed back by the driver in
/// `Sent`/`Nack`/`CcaBusy` events.
///
/// Built through the MPDU writer rather than from a raw sized buffer: the
/// test allocator recycles buffers across tests, and a raw frame would expose
/// stale bytes as its frame control field (e.g. a spurious "IE present" bit),
/// breaking frames that round-trip through `MpduFrame::from_radio_frame` on
/// the TSCH CSMA-CA requeue path.
fn create_data_radio_frame(
    allocator: &MacBufferAllocator,
    _size: u16,
) -> RadioFrame<RadioFrameSized> {
    create_test_data_frame(allocator).into_radio_frame::<FakeDriverConfig>()
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

    #[test]
    fn test_queue_when_full_gives_up_instead_of_recursing() {
        const CAP: usize = crate::constants::MAC_TSCH_MAX_PENDING_OPERATIONS;

        let mut runner = setup();
        // An RX link must exist so `next_rx_operation` keeps returning a
        // candidate slot; otherwise the helper bails immediately and the
        // recursion under test is never reached.
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.sort_links();

        // Fill the queue to capacity. The filler ASNs (1..=CAP) deliberately do
        // not coincide with the link's slot occurrences (multiples of the
        // slotframe size), so the next queue attempt fails with "full" rather
        // than a survivable conflict.
        for asn in 1..=CAP as TschAsn {
            runner
                .task
                .push_operation(TschOperation::RxSlot {
                    asn,
                    channel: Channel::_12,
                    slotframe_handle: 0,
                })
                .unwrap();
        }
        assert_eq!(runner.pending_operation_count(), CAP);

        // Would recurse without bound (stack overflow) prior to the fix.
        let queued = runner.task.queue_next_rx(0, &mut runner.context);
        assert!(!queued, "a full queue must reject, not re-queue endlessly");
        assert_eq!(runner.pending_operation_count(), CAP);

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
    fn test_beacon_period_slots_conversion() {
        let config = BeaconConfig::new(3);
        // 3 s of 10 ms timeslots.
        assert_eq!(config.period_slots(10_000), 300);
        // A period shorter than a timeslot still advances by one slot.
        assert_eq!(BeaconConfig::new(0).period_slots(10_000), 1);
    }

    #[test]
    fn test_beacon_jitter_slots_conversion() {
        let config = BeaconConfig::new(10).with_jitter_ms(500);
        // +/- 500 ms of 10 ms timeslots.
        assert_eq!(config.jitter_slots(10_000), 50);
        assert_eq!(BeaconConfig::new(10).jitter_slots(10_000), 0);
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
        // With TSCH CSMA-CA, NACK on a shared link re-queues the frame (no NoAck
        // response). NoAck is only produced on dedicated links.
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
        // Shared link: frame re-queued for CSMA-CA retransmission, no response to upper layer
        assert!(!runner.last_action_has_response());
        assert!(runner.has_pending_operations());
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
        // CCA is disabled on dedicated (non-shared) links. Shared links use CCA.
        let mut runner = setup();
        setup_dedicated_tx_link_schedule(&mut runner.context.pib.tsch);
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
        assert!(runner.task.is_time_root);
        assert!(runner.task.beacon_config.enabled);
        runner.terminate();
    }

    #[test]
    fn test_device_with_beacon_role_queues_beacon_but_keeps_syncing() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        // A joined forwarding node: beacons, but is not the time root.
        runner.context.pib.tsch.role = TschRoleConfig::device().with_beacon(9);
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.context.pib.tsch.sort_links();
        runner.step_entry();

        // Both an advertisement and an rx slot are queued, like for the
        // coordinator, but the node still adjusts its clock.
        assert_eq!(runner.pending_operation_count(), 2);
        assert!(runner
            .task
            .pending_operations
            .iter()
            .any(|op| op.is_advertisement()));
        assert!(!runner.task.is_time_root);
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
    fn test_beacon_sent_arms_next_beacon_asn() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        // `enter_as_coordinator` beacons every 9 s: 900 timeslots of 10 ms.
        runner.enter_as_coordinator();
        assert!(runner.task.next_beacon_asn.is_none());

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 50);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_002_120_000),
            None,
        ));

        // The beacon went out in the advertising cell at ASN 100; the next one
        // is due at the first point of the 900-timeslot grid past it. The ASN
        // the beacon occupied is what the cadence is built on, never the
        // instant it was transmitted at.
        assert_eq!(runner.task.next_beacon_asn, Some(900));
        runner.terminate();
    }

    /// Drive four beacons, standing in for the advertisement slot executing
    /// and the beacon going out in it, and return the ASN each one used.
    /// Neither pending operation owns a buffer, so clearing the queue between
    /// cycles leaks nothing.
    fn beacon_asns(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) -> [TschAsn; 4] {
        let mut asns = [0; 4];
        for beacon_asn in asns.iter_mut() {
            *beacon_asn = runner
                .pending_advertisement_asn()
                .expect("an advertisement is queued");
            runner.context.pib.tsch.update_timing(*beacon_asn);
            runner.task.pending_operations.clear();
            runner.task.on_beacon_sent(*beacon_asn, TIMESLOT_LENGTH_US);
            assert!(runner.task.queue_next_beacon(&mut runner.context));
        }
        asns
    }

    #[test]
    fn test_beacon_lands_on_the_absolute_asn_grid() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.role = TschRoleConfig::time_root(10);
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.context.pib.tsch.sort_links();
        runner.step_entry();

        // The first beacon goes out at the advertising cell right after entry
        // rather than waiting for the grid, then every later one is a
        // multiple of the period.
        assert_eq!(beacon_asns(&mut runner), [100, 1000, 2000, 3000]);
        runner.terminate();
    }

    /// Same, with the advertising cell away from timeslot 0: the grid point
    /// itself is not an advertising cell, so each beacon takes the first cell
    /// at or after it and the spacing is still exactly one period.
    #[test]
    fn test_beacon_cadence_is_exactly_one_period() {
        let mut runner = setup();
        // Advertising cell at timeslot 10, RX at timeslot 20, so the two
        // never contend for the same ASN.
        setup_two_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.role = TschRoleConfig::time_root(10);
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.context.pib.tsch.sort_links();
        runner.step_entry();

        assert_eq!(beacon_asns(&mut runner), [10, 1010, 2010, 3010]);
        runner.terminate();
    }

    /// A beacon that cannot go out at its due ASN does not drag the cadence
    /// with it: the grid is absolute, so the next beacon is placed from the
    /// period alone.
    #[test]
    fn test_late_beacon_does_not_shift_the_grid() {
        let mut runner = setup();
        setup_two_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.role = TschRoleConfig::time_root(10);
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.context.pib.tsch.sort_links();
        runner.step_entry();

        // A first beacon at ASN 10, before the grid has started, still arms
        // the grid point.
        runner.task.on_beacon_sent(10, TIMESLOT_LENGTH_US);
        assert_eq!(runner.task.next_beacon_asn, Some(1000));

        // One that slipped three slotframes past its grid point does not
        // carry the following one with it.
        runner.task.on_beacon_sent(1310, TIMESLOT_LENGTH_US);
        assert_eq!(runner.task.next_beacon_asn, Some(2000));

        // Several silent periods are caught up in one step.
        runner.task.on_beacon_sent(5000, TIMESLOT_LENGTH_US);
        assert_eq!(runner.task.next_beacon_asn, Some(6000));
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

    /// This node's own address in the exchanged frames.
    const SRC: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    #[test]
    fn test_ack_positive_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(50i16),
        ));

        assert!(runner.last_response_is_sent());
        // The Time Correction IE is applied additively, so a positive value
        // pushes last_base_time forward.
        assert_eq!(runner.last_base_time(), bt_before + NsDuration::micros(50));
        runner.terminate();
    }

    #[test]
    fn test_ack_negative_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(-30i16),
        ));

        // Additive application: a negative value pulls last_base_time back.
        assert_eq!(runner.last_base_time(), bt_before - NsDuration::micros(30));
        runner.terminate();
    }

    #[test]
    fn test_ack_zero_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
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
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
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
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            Some(100),
        ));

        assert_eq!(runner.last_base_time(), bt_before);
        runner.terminate();
    }
    #[test]
    fn test_ack_from_non_time_source_does_not_sync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);
        // The acknowledged destination is a child, not the time source.
        let child = [0x0c; 8];
        runner
            .context
            .pib
            .neighbors
            .intern(
                &Address::Extended(ExtendedAddress::new(&child[..])),
                NsInstant::from_ticks(0),
            )
            .unwrap();

        let mpdu = create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &child);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &child)
            .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(50i16),
        ));

        // The ACK of a child must not steer this node's clock.
        assert_eq!(runner.last_base_time(), bt_before);
        runner.terminate();
    }
}
// Integration Tests: Frame-based Synchronization
// ============================================================================

#[cfg(test)]
mod frame_sync_tests {
    use super::*;

    /// This node's own address in the received frames.
    const DST: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    fn setup_device_runner() -> TestRunner<'static, TschTask<FakeDriverConfig>> {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.coord_extended_address =
            Address::Extended(ExtendedAddress::new_owned(COORD_ADDRESS));
        runner.context.pib.tsch.asn = 0;
        runner.context.pib.tsch.last_base_time = NsInstant::from_ticks(0);
        runner.task.init(&mut runner.context);
        runner.task.state = TschState::Idle;
        runner.set_time_source(&COORD_ADDRESS);
        runner
    }

    #[test]
    fn test_device_rx_frame_sync_updates_base_time() {
        let mut runner = setup_device_runner();

        runner.step_timer_expired();
        assert!(matches!(runner.state(), TschState::Listening));

        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        let rx_instant = NsInstant::from_ticks(1_002_120_000);
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &DST)
                .into_radio_frame::<FakeDriverConfig>();
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
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &DST)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Received(frame, first_rx));
        assert_eq!(runner.drift_ns(), 0);

        // Second RX at ASN 200, 30us late
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        let second_rx = NsInstant::from_ticks(2_002_120_000 + 30_000);
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &DST)
                .into_radio_frame::<FakeDriverConfig>();
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

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &DST)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Received(
            frame,
            NsInstant::from_ticks(2_102_200_000),
        ));

        assert!(runner.task.is_time_root);
        // base_time set by update_timing(200), NOT by sync_asn
        assert_eq!(
            runner.last_base_time(),
            expected_slot_start(NsInstant::from_ticks(0), 0, 200)
        );
        runner.terminate();
    }

    #[test]
    fn test_device_rx_from_other_neighbor_does_not_sync() {
        let mut runner = setup_device_runner();

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        // A frame from some other neighbor: it feeds the neighbor table but
        // must not steer the clock.
        let other = [0x0b; 8];
        let rx_instant = NsInstant::from_ticks(1_002_120_000 + 30_000);
        let frame = create_addressed_data_frame(&runner.context.buffer_allocator, &other, &[0x01; 8])
            .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Received(frame, rx_instant));

        // base_time follows the nominal schedule (update_timing), not the
        // reception time of the foreign frame.
        assert_eq!(
            runner.last_base_time(),
            expected_slot_start(NsInstant::from_ticks(0), 0, 100)
        );
        assert!(runner
            .context
            .pib
            .neighbors
            .find_ext(&other)
            .is_some());
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
            TschCommand::UseTsch {
                enabled: false,
                cca: false,
                role: Default::default(),
            },
        )));

        assert_eq!(runner.last_action().unwrap(), ActionType::Completed);
        assert!(runner.last_response_is_command());
    }

    /// Convenience for the timeslot-template tests below.
    fn set_timeslot_template(
        runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>,
        timings: TschTimeslotTimings,
    ) {
        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Set(SetRequestAttribute::MacTimeslotTemplate(timings)),
        )));
        assert!(runner.last_response_is_command());
    }

    #[test]
    fn test_set_timeslot_template_honors_custom_ie_timings() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mut timings = TschTimeslotTimings::default();
        timings.set_timeslot_length(15_000);
        timings.set_tx_offset(3000);
        timings.set_cca_offset(1234);
        timings.set_rx_offset(1900);
        timings.set_ack_wait(800);
        set_timeslot_template(&mut runner, timings);

        let applied = &runner.context.pib.tsch.timeslot_timings;
        assert_eq!(applied.timeslot_length(), 15_000);
        assert_eq!(applied.tx_offset(), 3000);
        assert_eq!(applied.cca_offset(), 1234);
        assert_eq!(applied.ack_wait(), 800);
        runner.terminate();
    }

    #[test]
    fn test_set_timeslot_template_rejects_too_short_cca() {
        // CCA needs at least phyCcaDuration (128 us for O-QPSK); a template
        // asking for less cannot be honored, so the current one is kept.
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mut timings = TschTimeslotTimings::default();
        timings.set_cca(64);
        set_timeslot_template(&mut runner, timings);

        assert_eq!(runner.context.pib.tsch.timeslot_timings.cca(), 128);
        runner.terminate();
    }

    #[test]
    fn test_set_timeslot_template_rejects_too_short_tx_offset() {
        // The RMARKER cannot precede a full macUnitBackoffPeriod (320 us for
        // O-QPSK) of wake-up and CCA, so a shorter TX offset is refused.
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mut timings = TschTimeslotTimings::default();
        timings.set_tx_offset(200);
        set_timeslot_template(&mut runner, timings);

        assert_eq!(runner.context.pib.tsch.timeslot_timings.tx_offset(), 2120);
        runner.terminate();
    }

    #[test]
    fn test_set_timeslot_template_rejects_too_short_rx_offset() {
        // The receiver needs at least one aTurnaroundTime (192 us for O-QPSK)
        // to bring its front end up before listening; a shorter RX offset is
        // refused.
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mut timings = TschTimeslotTimings::default();
        timings.set_rx_offset(100);
        set_timeslot_template(&mut runner, timings);

        assert_eq!(runner.context.pib.tsch.timeslot_timings.rx_offset(), 1020);
        runner.terminate();
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
        runner.task.init(&mut runner.context);
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
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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
            ActionType::WaitForTimeoutOrSchedulerRequest
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
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &[0x01; 8])
                .into_radio_frame::<FakeDriverConfig>();
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
        runner.task.init(&mut runner.context);
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
        runner.task.init(&mut runner.context);
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
        // Dedicated link; with the retry budget pinned to zero the first NACK
        // exhausts it immediately and surfaces a NoAck response (F3 made
        // dedicated links retry by default, so without this they would re-queue).
        setup_dedicated_tx_link_schedule(&mut runner.context.pib.tsch);
        runner.set_max_frame_retries(0);
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
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &[0x01; 8],
            &COORD_ADDRESS,
        );
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &[0x01; 8],
            &COORD_ADDRESS,
        )
        .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(800i16),
        ));

        // Additive application of the Time Correction IE.
        assert_eq!(runner.last_base_time(), bt_before + NsDuration::micros(800));
        runner.terminate();
    }

    #[test]
    fn test_large_negative_timesync() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);

        let mpdu = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &[0x01; 8],
            &COORD_ADDRESS,
        );
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        let bt_before = runner.last_base_time();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &[0x01; 8],
            &COORD_ADDRESS,
        )
        .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            Some(-500i16),
        ));

        // Additive application of the Time Correction IE.
        assert_eq!(runner.last_base_time(), bt_before - NsDuration::micros(500));
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
        let mut pib: TschPib = TschPib::new();
        assert_eq!(pib.drift_ns(), 0);
        assert!(pib.last_rx.is_none());

        pib.sync_asn(100, rmarker_for_slot_start(1_000_000_000));

        assert_eq!(pib.drift_ns(), 0);
        assert!(pib.last_rx.is_some());
    }

    #[test]
    fn test_positive_drift_clock_faster() {
        let mut pib: TschPib = TschPib::new();

        // First sync at ASN 100
        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));
        assert_eq!(pib.drift_ns(), 0);

        // Second sync at ASN 300, with 60µs less elapsed than expected.
        // delta_asn = 200, expected = 200 * 10_000µs = 2_000_000µs
        // elapsed = 2_000_000µs - 60µs = 1_999_940µs
        // drift = 60_000ns / 200 = 300 ns/slot
        let slot_start_2: u64 = slot_start_1 + 1_999_940_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns(), 300);
    }

    #[test]
    fn test_negative_drift_clock_slower() {
        let mut pib: TschPib = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // 60µs more elapsed than expected -> drift = -(60_000 / 200) = -300
        let slot_start_2: u64 = slot_start_1 + 2_000_060_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns(), -300);
    }

    #[test]
    fn test_zero_drift_synchronized() {
        let mut pib: TschPib = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // Elapsed exactly matches expected: 200 * 10_000µs = 2_000_000µs
        let slot_start_2: u64 = slot_start_1 + 2_000_000_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns(), 0);
    }

    #[test]
    fn test_drift_averages_recent_samples() {
        let mut pib: TschPib = TschPib::new();

        // First pair: establish +300 drift (one sample of 60_000ns over 200 slots).
        pib.sync_asn(100, rmarker_for_slot_start(1_000_000_000));
        pib.sync_asn(300, rmarker_for_slot_start(1_000_000_000 + 1_999_940_000));
        assert_eq!(pib.drift_ns(), 300);

        // Second pair: clocks now perfectly aligned -> a 0ns sample over 200 slots.
        // The estimate is the interval-weighted mean of both samples, not just
        // the latest one: (60_000 + 0) / (200 + 200) = 150 ns/slot.
        let last_slot_start = 1_000_000_000 + 1_999_940_000;
        pib.sync_asn(500, rmarker_for_slot_start(last_slot_start + 2_000_000_000));
        assert_eq!(pib.drift_ns(), 150);
    }

    #[test]
    fn test_drift_ignores_short_baseline() {
        let mut pib: TschPib = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // delta_asn = 5 < MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS: a 2.5µs deviation
        // over 5 slots would read as 500 ns/slot (50 ppm) although it is
        // indistinguishable from per-event phase noise. The sample must be
        // rejected; the sync still corrects phase (anchor moves).
        let slot_start_2: u64 = slot_start_1 + 49_997_500;
        pib.sync_asn(105, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns(), 0);
        assert_eq!(
            pib.last_base_time,
            NsInstant::from_ticks(slot_start_2),
            "phase must still be corrected even when the drift sample is rejected"
        );
    }

    #[test]
    fn test_join_then_ack_correction_does_not_fake_drift() {
        let mut pib: TschPib = TschPib::new();

        // Join: anchor ASN 1100 to a scan timestamp 61µs late of the grid.
        let grid = |asn: u64| asn * TIMESLOT_LENGTH_US * 1_000;
        pib.reset_time_base(1100, rmarker_for_slot_start(grid(1100) + 61_000));
        assert!(pib.last_rx.is_none(), "join must not seed a resync baseline");
        assert_eq!(pib.drift_ns(), 0);

        // TX slots 1102/1103, then the Enh-ACK carries TC = -59µs.
        pib.update_timing(1102);
        pib.update_timing(1103);
        pib.sync_ack(-59);

        assert_eq!(
            pib.drift_ns(),
            0,
            "a phase correction over a 3-slot baseline must not become drift"
        );

        // The schedule must stay on the (phase-corrected) 10ms grid: 97 slots
        // later, slot 1200 starts 97 * 10ms after slot 1103, not ~1.9ms early.
        let base_1103 = pib.last_base_time;
        let start_1200 = pib.expected_slot_start(1200, TIMESLOT_LENGTH_US);
        assert_eq!(
            start_1200,
            base_1103 + NsDuration::micros(97 * TIMESLOT_LENGTH_US)
        );
    }

    #[test]
    fn test_drift_sample_clamped_to_max_ppm() {
        let mut pib: TschPib = TschPib::new();

        let slot_start_1: u64 = 1_000_000_000;
        pib.sync_asn(100, rmarker_for_slot_start(slot_start_1));

        // A 20ms deviation over 200 slots (10_000 ppm) is physically
        // impossible crystal drift; the estimator clamps it to
        // MAC_TSCH_MAX_DRIFT_PPM (200 ppm = 2_000 ns/slot at 10ms slots).
        let slot_start_2: u64 = slot_start_1 + 2_000_000_000 - 20_000_000;
        pib.sync_asn(300, rmarker_for_slot_start(slot_start_2));

        assert_eq!(pib.drift_ns(), 2_000);
    }

    #[test]
    fn test_reset_time_base_clears_sync_history() {
        let mut pib: TschPib = TschPib::new();

        pib.sync_asn(100, rmarker_for_slot_start(1_000_000_000));
        pib.sync_asn(300, rmarker_for_slot_start(1_000_000_000 + 1_999_940_000));
        assert_eq!(pib.drift_ns(), 300);
        assert!(pib.last_rx.is_some());

        pib.reset_time_base(1000, rmarker_for_slot_start(5_000_000_000));

        assert_eq!(pib.asn, 1000);
        assert_eq!(pib.last_base_time, NsInstant::from_ticks(5_000_000_000));
        assert!(pib.last_rx.is_none());
        assert_eq!(pib.drift_ns(), 0);
    }

    #[test]
    fn test_expected_slot_start_zero_drift() {
        let mut pib: TschPib = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(0);

        let result = pib.expected_slot_start(200, TIMESLOT_LENGTH_US);
        let expected = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 100, 200);
        assert_eq!(result, expected);
    }

    #[test]
    fn test_expected_slot_start_positive_drift() {
        let mut pib: TschPib = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(300);

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
        let mut pib: TschPib = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(-300);

        let result = pib.expected_slot_start(200, TIMESLOT_LENGTH_US);
        let nominal = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 100, 200);

        // 100 slots * 300ns/slot = 30_000ns later than nominal
        assert_eq!(result, nominal + NsDuration::nanos(30_000));
    }

    #[test]
    fn test_drift_accumulates_over_distance() {
        let mut pib: TschPib = TschPib::new();
        pib.asn = 0;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(100);

        // 1000 slots ahead: 1000 * 100ns = 100µs total correction
        let result = pib.expected_slot_start(1000, TIMESLOT_LENGTH_US);
        let nominal = expected_slot_start(NsInstant::from_ticks(1_000_000_000), 0, 1000);
        assert_eq!(result, nominal - NsDuration::nanos(100_000));
    }

    #[test]
    fn test_expected_slot_start_same_asn() {
        let mut pib: TschPib = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(500); // drift should not matter for zero distance

        let result = pib.expected_slot_start(100, TIMESLOT_LENGTH_US);
        assert_eq!(result, NsInstant::from_ticks(1_000_000_000));
    }

    #[test]
    fn test_update_timing_uses_drift() {
        let mut pib: TschPib = TschPib::new();
        pib.asn = 100;
        pib.last_base_time = NsInstant::from_ticks(1_000_000_000);
        pib.set_drift_ns(300);

        pib.update_timing(200);

        let expected_base =
            expected_slot_start_with_drift(NsInstant::from_ticks(1_000_000_000), 100, 200, 300);
        assert_eq!(pib.asn, 200);
        assert_eq!(pib.last_base_time, expected_base);
    }
}

// ============================================================================
// TSCH CSMA-CA Retransmission Algorithm Tests
// ============================================================================

#[cfg(test)]
mod tsch_csma_ca_tests {
    use super::*;

    // -------------------------------------------------------------------
    // Shared link CCA enabled
    // -------------------------------------------------------------------

    #[test]
    fn test_shared_link_tx_enables_cca() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();
        runner.context.pib.tsch.tsch_cca = true;

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        // Shared link with macTschCca on must request CCA.
        assert!(runner.last_request().unwrap().cca_enabled());

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
    fn test_dedicated_link_tx_disables_cca() {
        let mut runner = setup();
        setup_dedicated_tx_link_schedule(&mut runner.context.pib.tsch);
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

    // -------------------------------------------------------------------
    // Shared link NACK retransmission
    // -------------------------------------------------------------------

    #[test]
    fn test_shared_link_nack_requeues_frame() {
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

        // Shared link NACK should NOT produce a response to upper layer
        assert!(!runner.last_action_has_response());
        assert!(runner.is_idle());
        // Frame was re-queued for retransmission
        assert!(runner.has_pending_operations());
        runner.terminate();
    }

    #[test]
    fn test_dedicated_link_nack_reports_no_ack() {
        let mut runner = setup();
        setup_dedicated_tx_only_schedule(&mut runner.context.pib.tsch);
        let min_be = runner.context.pib.tsch.tsch_min_be;
        let max_retries = runner.context.pib.max_frame_retries as u64; // default: 3
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        let mut time_base = 1_000_000_000u64;
        for nb in 1..=(max_retries + 1) {
            runner.step_timer_expired();
            runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(time_base)));
            let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
            runner.step_driver_event(DrvSvcEvent::Nack(
                frame,
                NsInstant::from_ticks(time_base + 1_000_000),
                None,
                None,
            ));

            if nb <= max_retries {
                // Re-queued: no response, BE unchanged (dedicated), NB counts up.
                assert!(!runner.last_action_has_response());
                assert!(runner.has_pending_operations());
                assert_eq!(runner.pending_tx_backoff(), Some((min_be, nb as u8)));
            }
            time_base += 20_000_000_000;
        }

        // Budget exhausted (NB > macMaxFrameRetries): NoAck reported, queue empty.
        assert!(runner.last_action_has_response());
        assert!(runner.last_response_is_no_ack());
        assert!(!runner.has_pending_operations());
        runner.terminate();
    }

    #[test]
    fn test_shared_link_nack_retransmission_at_later_asn() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // Execute the first TX attempt at ASN 100
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));

        // Frame was re-queued: the retransmission ASN must be > 100
        assert!(runner.has_pending_operations());
        let next_deadline = runner.last_deadline().unwrap();
        let first_tx_deadline = expected_deadline(NsInstant::from_ticks(0), 0, 100);
        assert!(next_deadline > first_tx_deadline);
        runner.terminate();
    }

    // -------------------------------------------------------------------
    // CCA busy retransmission
    // -------------------------------------------------------------------

    #[test]
    fn test_shared_link_cca_busy_requeues_frame() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();

        // CCA busy on shared link
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(1_000_000_000),
        ));

        // No response to upper layer, frame re-queued
        assert!(!runner.last_action_has_response());
        assert!(runner.is_idle());
        assert!(runner.has_pending_operations());
        runner.terminate();
    }

    #[test]
    fn test_dedicated_link_cca_busy_reports_failure() {
        let mut runner = setup();
        setup_dedicated_tx_only_schedule(&mut runner.context.pib.tsch);
        // CCA only runs when macTschCca is on; enable it for this scenario.
        runner.context.pib.tsch.tsch_cca = true;
        let min_be = runner.context.pib.tsch.tsch_min_be;
        let max_retries = runner.context.pib.max_frame_retries as u64; // default: 3
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        let mut time_base = 1_000_000_000u64;
        for nb in 1..=(max_retries + 1) {
            runner.step_timer_expired();
            let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
            runner.step_driver_event(DrvSvcEvent::CcaBusy(
                frame,
                NsInstant::from_ticks(time_base),
            ));

            if nb <= max_retries {
                assert!(!runner.last_action_has_response());
                assert!(runner.has_pending_operations());
                assert_eq!(runner.pending_tx_backoff(), Some((min_be, nb as u8)));
            }
            time_base += 20_000_000_000;
        }

        // Budget exhausted: ChannelAccessFailure reported, queue empty.
        assert!(runner.last_action_has_response());
        assert!(runner.last_response_is_channel_access_failure());
        assert!(!runner.has_pending_operations());
        runner.terminate();
    }

    // -------------------------------------------------------------------
    // Backoff state transitions
    // -------------------------------------------------------------------

    #[test]
    fn test_backoff_be_increments_on_each_failure() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let min_be = runner.context.pib.tsch.tsch_min_be; // default: 1

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // First TX attempt: NACK
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        // After first failure: the procedure is entered, BE = min_be, NB = 1
        assert_eq!(runner.pending_tx_backoff(), Some((min_be, 1)));

        // Second TX attempt: NACK
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
            None,
        ));
        // After second failure: BE = min_be + 1, NB = 2
        assert_eq!(runner.pending_tx_backoff(), Some((min_be + 1, 2)));

        // Third TX attempt: success
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(5_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(5_001_000_000),
            None,
        ));
        assert!(runner.last_response_is_sent());
        runner.terminate();
    }

    #[test]
    fn test_backoff_be_capped_at_max_be() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        // Allow enough retries to drive BE past its cap.
        runner.set_max_frame_retries(15);
        runner.enter_as_device();

        let min_be = runner.context.pib.tsch.tsch_min_be; // default: 1
        let max_be = runner.context.pib.tsch.tsch_max_be; // default: 7

        // Generate enough failures to hit max BE
        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        let mut time_base = 1_000_000_000u64;
        for i in 0..(max_be as u64 + 2) {
            runner.step_timer_expired();
            runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(time_base)));
            let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
            runner.step_driver_event(DrvSvcEvent::Nack(
                frame,
                NsInstant::from_ticks(time_base + 1_000_000),
                None,
                None,
            ));
            // The entry round keeps BE = macTschMinBe; BE then grows by one
            // per further shared failure, capped at macTschMaxBe. NB counts
            // every failure.
            let expected_be = core::cmp::min(min_be as u64 + i, max_be as u64) as u8;
            assert_eq!(
                runner.pending_tx_backoff(),
                Some((expected_be, i as u8 + 1))
            );
            time_base += 20_000_000_000; // big enough jump for next slot
        }

        // After max_be+2 failures, BE stayed capped and the frame is still
        // re-queued (retry budget not exhausted).
        assert!(runner.has_pending_operations());

        // Final success to clean up
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(time_base)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(time_base + 1_000_000),
            None,
        ));
        assert!(runner.last_response_is_sent());
        runner.terminate();
    }

    // -------------------------------------------------------------------
    // select_nth_link PIB tests
    // -------------------------------------------------------------------

    #[test]
    fn test_select_nth_link_skip_zero_same_as_select_next() {
        let mut pib: TschPib = TschPib::new();
        setup_one_link_schedule(&mut pib);

        let next = pib.select_next_link(0, |l| l.link_options.contains(TschLinkOption::Tx));
        let nth = pib.select_nth_link(0, 0, |l| l.link_options.contains(TschLinkOption::Tx));
        assert_eq!(next.map(|(_, asn)| asn), nth.map(|(_, asn)| asn));
    }

    #[test]
    fn test_select_nth_link_skip_one() {
        let mut pib: TschPib = TschPib::new();
        setup_one_link_schedule(&mut pib);

        // First occurrence at ASN 100 (timeslot 0, slotframe size 100)
        let first = pib
            .select_nth_link(0, 0, |l| l.link_options.contains(TschLinkOption::Tx))
            .unwrap();
        assert_eq!(first.1, 100);

        // Skip 1 --> second occurrence at ASN 200
        let second = pib
            .select_nth_link(0, 1, |l| l.link_options.contains(TschLinkOption::Tx))
            .unwrap();
        assert_eq!(second.1, 200);
    }

    #[test]
    fn test_select_nth_link_skip_multiple() {
        let mut pib: TschPib = TschPib::new();
        setup_one_link_schedule(&mut pib);

        // Skip 3 --> fourth occurrence at ASN 400
        let fourth = pib
            .select_nth_link(0, 3, |l| l.link_options.contains(TschLinkOption::Tx))
            .unwrap();
        assert_eq!(fourth.1, 400);
    }

    // -------------------------------------------------------------------
    // TschCsmaBackoff unit tests
    // -------------------------------------------------------------------

    #[test]
    fn test_csma_backoff_new() {
        let backoff = TschCsmaBackoff::new(3);
        assert_eq!(backoff.be, 3);
        assert_eq!(backoff.nb, 0);
        assert_eq!(backoff.backoff_remaining, 0);
        assert!(!backoff.retransmitting);
    }

    // -------------------------------------------------------------------
    // Retry budget exhaustion
    // -------------------------------------------------------------------

    #[test]
    fn test_shared_link_retries_exhausted_reports_no_ack() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.set_max_frame_retries(1);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // First failure: NB = 1 <= macMaxFrameRetries, re-queued.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert!(!runner.last_action_has_response());
        assert!(runner.has_pending_operations());

        // Second failure: NB = 2 > macMaxFrameRetries, frame is dropped and
        // NoAck is reported to the upper layer.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
            None,
        ));
        assert!(runner.last_action_has_response());
        assert!(runner.last_response_is_no_ack());
        assert!(!runner.has_pending_operations());
        assert!(runner.is_idle());
        runner.terminate();
    }

    #[test]
    fn test_shared_link_retries_exhausted_on_cca_reports_failure() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.set_max_frame_retries(1);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // First failure: CCA busy, re-queued.
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(1_000_000_000),
        ));
        assert!(!runner.last_action_has_response());
        assert!(runner.has_pending_operations());

        // Second failure: CCA busy again, retry budget exhausted, the
        // failure is reported as ChannelAccessFailure.
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(3_000_000_000),
        ));
        assert!(runner.last_action_has_response());
        assert!(runner.last_response_is_channel_access_failure());
        assert!(!runner.has_pending_operations());
        assert!(runner.is_idle());
        runner.terminate();
    }

    // -------------------------------------------------------------------
    // Retransmission eventually succeeds after backoff
    // -------------------------------------------------------------------

    #[test]
    fn test_shared_link_nack_then_success() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // First attempt fails (NACK)
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert!(!runner.last_action_has_response());
        assert!(runner.has_pending_operations());

        // Retransmission succeeds
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(2_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(2_001_000_000),
            None,
        ));

        assert!(runner.last_response_is_sent());
        runner.terminate();
    }

    #[test]
    fn test_shared_link_cca_busy_then_success() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // First attempt fails (CCA busy)
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(1_000_000_000),
        ));
        assert!(!runner.last_action_has_response());
        assert!(runner.has_pending_operations());

        // Retransmission succeeds
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(2_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(2_001_000_000),
            None,
        ));

        assert!(runner.last_response_is_sent());
        runner.terminate();
    }

    // -------------------------------------------------------------------
    // Mixed CCA and NACK failures
    // -------------------------------------------------------------------

    #[test]
    fn test_cca_busy_then_nack_then_success() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // CCA failure
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(1_000_000_000),
        ));
        assert!(runner.has_pending_operations());

        // NACK failure
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
            None,
        ));
        assert!(runner.has_pending_operations());

        // Success
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(5_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(5_001_000_000),
            None,
        ));
        assert!(runner.last_response_is_sent());
        runner.terminate();
    }
}

#[cfg(test)]
mod tsch_csma_ca_state_machine_tests {
    use super::*;

    #[test]
    fn test_cca_busy_never_backs_off_before_first_transmission() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        // The channel-access/retransmission split is only observable when CCA runs.
        runner.context.pib.tsch.tsch_cca = true;
        let min_be = runner.context.pib.tsch.tsch_min_be; // default 1
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // As long as the frame has never been on the air, every channel-access
        // failure retries at the next TX link: NB counts up, BE stays at
        // macTschMinBe and no shared-link backoff window is drawn, on the
        // first busy channel and on every consecutive one.
        let mut time_base = 1_000_000_000u64;
        for nb in 1..=3u8 {
            runner.step_timer_expired();
            let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
            runner.step_driver_event(DrvSvcEvent::CcaBusy(
                frame,
                NsInstant::from_ticks(time_base),
            ));
            assert!(!runner.last_action_has_response());
            assert!(runner.is_idle());
            assert!(runner.has_pending_operations());
            assert_eq!(runner.pending_tx_backoff_full(), Some((min_be, nb, 0)));
            time_base += 20_000_000_000;
        }
        runner.terminate();
    }

    #[test]
    fn test_shared_link_nack_sets_backoff_remaining() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        let min_be = runner.context.pib.tsch.tsch_min_be; // default 1
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // The first missing ACK enters the retransmission procedure: a window
        // is drawn from the entry BE (macTschMinBe), which stays put until a
        // subsequent failed transmission in a shared link advances it.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));

        let (be, nb, rem) = runner.pending_tx_backoff_full().unwrap();
        assert_eq!((be, nb), (min_be, 1));
        assert_eq!(rem, 1 % (1u16 << min_be));
        runner.terminate();
    }

    #[test]
    fn test_dedicated_link_failures_keep_be_and_redraw_delay() {
        let mut runner = setup();
        setup_dedicated_tx_only_schedule(&mut runner.context.pib.tsch);
        let min_be = runner.context.pib.tsch.tsch_min_be;
        runner.set_max_frame_retries(3);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        // Timeslot 0 in a slotframe of 100: first attempt at ASN 100.
        assert_eq!(runner.pending_tx_asn(), Some(100));

        let mut time_base = 1_000_000_000u64;
        for nb in 1..=3u8 {
            runner.step_timer_expired();
            runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(time_base)));
            let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
            runner.step_driver_event(DrvSvcEvent::Nack(
                frame,
                NsInstant::from_ticks(time_base + 1_000_000),
                None,
                None,
            ));
            // BE stays at macTschMinBe, NB counts up, and a fresh window is
            // drawn from the unchanged BE each round.
            assert_eq!(
                runner.pending_tx_backoff_full(),
                Some((min_be, nb, 1 % (1u16 << min_be)))
            );
            // The dedicated link is never delayed: the retry takes the next
            // occurrence of the cell, one slotframe later.
            assert_eq!(runner.pending_tx_asn(), Some(100 * (nb as u64 + 1)));
            time_base += 20_000_000_000;
        }
        runner.terminate();
    }

    #[test]
    fn test_cca_busy_during_retransmission_redraws_without_be_bump() {
        // Once the retransmission procedure is entered, a busy channel counts
        // toward NB and re-enters the delay box with a fresh window, but only
        // an unacknowledged transmission in a shared link advances BE.
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.tsch_cca = true;
        let min_be = runner.context.pib.tsch.tsch_min_be;
        runner.set_max_frame_retries(3);
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // (1) First missing ACK: enter the procedure, window from macTschMinBe.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert_eq!(
            runner.pending_tx_backoff_full(),
            Some((min_be, 1, 1 % (1u16 << min_be)))
        );

        // (2) Busy channel during the retransmission: NB advances, a fresh
        //     window is drawn, BE is unchanged.
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(3_000_000_000),
        ));
        assert_eq!(
            runner.pending_tx_backoff_full(),
            Some((min_be, 2, 1 % (1u16 << min_be)))
        );

        // (3) Another missing ACK in a shared link: now BE advances and the
        //     window is drawn from the advanced BE.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(5_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(5_001_000_000),
            None,
            None,
        ));
        assert_eq!(
            runner.pending_tx_backoff_full(),
            Some((min_be + 1, 3, 1 % (1u16 << (min_be + 1))))
        );
        runner.terminate();
    }

    #[test]
    fn test_first_cca_busy_then_nack_be_progression() {
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.tsch_cca = true;
        let min_be = runner.context.pib.tsch.tsch_min_be;
        runner.enter_as_device();

        let mpdu = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // (1) First failure is a channel-access failure: BE unchanged.
        runner.step_timer_expired();
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::CcaBusy(
            frame,
            NsInstant::from_ticks(1_000_000_000),
        ));
        assert_eq!(runner.pending_tx_backoff_full(), Some((min_be, 1, 0)));

        // (2) Second failure is the first missing ACK: it enters the
        //     retransmission procedure, drawing a window from the entry BE
        //     (still macTschMinBe) without advancing it yet.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
            None,
        ));
        let (be, nb, rem) = runner.pending_tx_backoff_full().unwrap();
        assert_eq!((be, nb), (min_be, 2));
        assert_eq!(rem, 1 % (1u16 << min_be));
        runner.terminate();
    }

    #[test]
    fn test_backoff_resets_for_new_frame() {
        // The backoff is kept per frame: a fresh transmission must restart from
        // the PIB defaults rather than inherit the previous frame's BE/NB.
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        let min_be = runner.context.pib.tsch.tsch_min_be;
        runner.enter_as_device();

        // Frame 1: two NACKs (the second advances BE to min_be + 1), then
        // success.
        let mpdu1 = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu1));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert_eq!(runner.pending_tx_backoff(), Some((min_be, 1)));

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(2_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(2_001_000_000),
            None,
            None,
        ));
        assert_eq!(runner.pending_tx_backoff(), Some((min_be + 1, 2)));

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(3_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(3_001_000_000),
            None,
        ));
        assert!(runner.last_response_is_sent());
        assert!(!runner.has_pending_operations());
        assert!(runner.is_idle());

        // Frame 2: a brand-new transmission. Its first NACK must yield
        // (min_be, 1), proving the state did not carry over from frame 1.
        let mpdu2 = create_test_data_frame(&runner.context.buffer_allocator);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu2));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(4_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(4_001_000_000),
            None,
            None,
        ));
        assert_eq!(runner.pending_tx_backoff(), Some((min_be, 1)));
        runner.terminate();
    }
}

#[cfg(test)]
mod tx_link_destination_tests {
    use super::*;

    const N1_ADDRESS: [u8; 8] = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8];
    const N2_ADDRESS: [u8; 8] = [0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8];
    const SRC_ADDRESS: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    fn intern_neighbor(
        runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>,
        addr_le: &[u8; 8],
    ) -> NeighborId {
        runner
            .context
            .pib
            .neighbors
            .intern(
                &Address::Extended(ExtendedAddress::new(&addr_le[..])),
                NsInstant::from_ticks(0),
            )
            .unwrap()
    }

    /// Two dedicated TX cells assigned to two different neighbors (timeslots
    /// 10 and 20) plus one unassigned shared TX cell (timeslot 30), in a
    /// slotframe of size 100.
    fn setup_two_neighbor_schedule(pib: &mut TschPib, n1: NeighborId, n2: NeighborId) {
        assert!(pib.create_slotframe(0, 100).is_ok());
        for (timeslot, neighbor) in [(10, Some(n1)), (20, Some(n2))] {
            let link = TschLink {
                slotframe_handle: 0,
                timeslot,
                channel_offset: 0,
                link_options: TschLinkOption::Tx,
                link_type: TschLinkType::Normal,
                neighbor,
                link_advertise: false,
            };
            assert!(pib.add_link(link).is_ok());
        }
        let shared = TschLink {
            slotframe_handle: 0,
            timeslot: 30,
            channel_offset: 0,
            link_options: TschLinkOption::Tx | TschLinkOption::Shared,
            link_type: TschLinkType::Normal,
            neighbor: None,
            link_advertise: false,
        };
        assert!(pib.add_link(shared).is_ok());
    }

    #[test]
    fn test_unicast_waits_for_link_to_destination() {
        // A frame to the second neighbor must skip the first neighbor's
        // earlier dedicated cell (nobody would listen there) and take the
        // destination's own cell.
        let mut runner = setup();
        let n1 = intern_neighbor(&mut runner, &N1_ADDRESS);
        let n2 = intern_neighbor(&mut runner, &N2_ADDRESS);
        setup_two_neighbor_schedule(&mut runner.context.pib.tsch, n1, n2);
        runner.enter_as_device();

        let mpdu = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &SRC_ADDRESS,
            &N2_ADDRESS,
        );
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // Timeslot 20, not the earlier timeslot 10 assigned to n1.
        assert_eq!(runner.pending_tx_asn(), Some(20));
        runner.terminate();
    }

    #[test]
    fn test_unresolved_destination_uses_unassigned_links_only() {
        // A destination that maps to no neighbor table entry (as for a
        // broadcast) must not borrow another neighbor's dedicated cell; only
        // links without an assigned neighbor qualify.
        let mut runner = setup();
        let n1 = intern_neighbor(&mut runner, &N1_ADDRESS);
        let n2 = intern_neighbor(&mut runner, &N2_ADDRESS);
        setup_two_neighbor_schedule(&mut runner.context.pib.tsch, n1, n2);
        runner.enter_as_device();

        const UNKNOWN: [u8; 8] = [0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8];
        let mpdu = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &SRC_ADDRESS,
            &UNKNOWN,
        );
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));

        // The unassigned shared cell at timeslot 30.
        assert_eq!(runner.pending_tx_asn(), Some(30));
        runner.terminate();
    }

    #[test]
    fn test_retransmission_never_preempts_into_other_neighbors_cell() {
        // The dedicated-link preemption of the retransmission wait only
        // applies to dedicated links to the destination. A retry of a frame
        // whose destination has no dedicated cell must stay on unassigned
        // shared cells even when another neighbor's dedicated cell comes
        // first.
        let mut runner = setup();
        let n1 = intern_neighbor(&mut runner, &N1_ADDRESS);
        let _n2 = intern_neighbor(&mut runner, &N2_ADDRESS);
        // Only n1 has a dedicated cell (timeslot 10); the shared cell is at
        // timeslot 30.
        assert!(runner.context.pib.tsch.create_slotframe(0, 100).is_ok());
        let dedicated = TschLink {
            slotframe_handle: 0,
            timeslot: 10,
            channel_offset: 0,
            link_options: TschLinkOption::Tx,
            link_type: TschLinkType::Normal,
            neighbor: Some(n1),
            link_advertise: false,
        };
        assert!(runner.context.pib.tsch.add_link(dedicated).is_ok());
        let shared = TschLink {
            slotframe_handle: 0,
            timeslot: 30,
            channel_offset: 0,
            link_options: TschLinkOption::Tx | TschLinkOption::Shared,
            link_type: TschLinkType::Normal,
            neighbor: None,
            link_advertise: false,
        };
        assert!(runner.context.pib.tsch.add_link(shared).is_ok());
        runner.enter_as_device();

        // Fresh frame to n2: only the shared cell qualifies (ASN 30).
        let mpdu = create_addressed_data_frame(
            &runner.context.buffer_allocator,
            &SRC_ADDRESS,
            &N2_ADDRESS,
        );
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        assert_eq!(runner.pending_tx_asn(), Some(30));

        // First attempt at ASN 30 fails (no ACK): the retry draws a one-slot
        // shared-link skip (deterministic RNG) and must land on the shared
        // cell after the skipped one (ASN 230), not on n1's dedicated cell at
        // ASN 110.
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_data_radio_frame(&runner.context.buffer_allocator, 20);
        runner.step_driver_event(DrvSvcEvent::Nack(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
            None,
        ));
        assert_eq!(runner.pending_tx_asn(), Some(230));
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Neighbor statistics hooks
// ============================================================================

#[cfg(test)]
mod neighbor_stats_tests {
    use super::*;

    fn skip_beacon(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) {
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let frame = create_test_mpdu_with_ack(&runner.context.buffer_allocator)
            .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));
    }

    #[test]
    fn test_rx_frame_feeds_neighbor_table() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_coordinator();
        skip_beacon(&mut runner);

        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        let src = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let dst = [0xaa; 8];
        let frame = create_addressed_data_frame(&runner.context.buffer_allocator, &src, &dst)
            .into_radio_frame::<FakeDriverConfig>();
        let rx_instant = NsInstant::from_ticks(2_002_120_000);
        runner.step_driver_event(DrvSvcEvent::Received(frame, rx_instant));

        let id = runner
            .context
            .pib
            .neighbors
            .find_ext(&src)
            .expect("the source neighbor must have been created");
        let neighbor = runner.context.pib.neighbors.get(id).unwrap();
        assert_eq!(neighbor.rx_count(), 1);
        assert_eq!(neighbor.last_heard(), rx_instant);
        runner.terminate();
    }

    #[test]
    fn test_tx_sent_scores_destination_neighbor() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let src = [0x01; 8];
        let dst = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        // The destination must already be known: transmissions never create
        // table entries, receptions do.
        let dst_id = runner
            .context
            .pib
            .neighbors
            .intern(
                &Address::Extended(ExtendedAddress::new(&dst[..])),
                NsInstant::from_ticks(0),
            )
            .unwrap();

        let mpdu = create_addressed_data_frame(&runner.context.buffer_allocator, &src, &dst);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let sent_instant = NsInstant::from_ticks(1_001_000_000);
        let frame = create_addressed_data_frame(&runner.context.buffer_allocator, &src, &dst)
            .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(frame, sent_instant, None));

        let neighbor = runner.context.pib.neighbors.get(dst_id).unwrap();
        assert_eq!(neighbor.tx_acked(), 1);
        assert_eq!(neighbor.tx_attempts(), 1);
        assert_eq!(neighbor.etx_q7(), 128, "one attempt, one ack: ETX 1.0");
        assert_eq!(neighbor.last_sent_ok(), sent_instant);
        runner.terminate();
    }

    #[test]
    fn test_tx_without_ack_request_is_not_scored() {
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.enter_as_device();

        let src = [0x01; 8];
        let dst = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let dst_id = runner
            .context
            .pib
            .neighbors
            .intern(
                &Address::Extended(ExtendedAddress::new(&dst[..])),
                NsInstant::from_ticks(0),
            )
            .unwrap();

        let mut mpdu = create_addressed_data_frame(&runner.context.buffer_allocator, &src, &dst);
        mpdu.writer().set_ack_request(false);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired();
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let mut sent_frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &src, &dst);
        sent_frame.writer().set_ack_request(false);
        runner.step_driver_event(DrvSvcEvent::Sent(
            sent_frame.into_radio_frame::<FakeDriverConfig>(),
            NsInstant::from_ticks(1_001_000_000),
            None,
        ));

        let neighbor = runner.context.pib.neighbors.get(dst_id).unwrap();
        assert_eq!(neighbor.tx_acked(), 0);
        assert_eq!(neighbor.tx_attempts(), 0);
        assert_eq!(neighbor.etx_q7(), 0);
        runner.terminate();
    }
}

// ============================================================================
// Integration Tests: Time-Source Keep-Alive (IEEE 802.15.4-2024, 10.3.10.8)
// ============================================================================

#[cfg(test)]
mod keep_alive_tests {
    use super::*;
    use crate::constants::MAC_TSCH_MAX_KEEP_ALIVE_MISSES;
    use crate::mac::mlme::set::SetRequestAttribute;

    /// This node's own address (matches the address `setup()` writes to the
    /// PIB).
    const SRC: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    /// Keep-alive period used by most tests, in timeslots: 50 slots = 500 ms
    /// with the 10 ms test timeslot.
    const PERIOD_SLOTS: u16 = 50;

    fn period(period_slots: u16) -> NsDuration {
        NsDuration::micros(period_slots as u64 * TIMESLOT_LENGTH_US)
    }

    /// Device runner with the given schedule, keep-alive period and the
    /// coordinator as its time source, entered at t = 0. The period is set
    /// before entry so the entry-time re-arming already uses it.
    fn setup_keep_alive_runner(
        period_slots: u16,
        install: fn(&mut TschPib),
    ) -> TestRunner<'static, TschTask<FakeDriverConfig>> {
        let mut runner = setup();
        install(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.keep_alive_period = period_slots;
        runner.enter_as_device();
        runner.set_time_source(&COORD_ADDRESS);
        runner
    }

    /// Assert that exactly one keep-alive is pending and that its frame is
    /// an acknowledged Data frame addressed to the time source.
    fn assert_pending_keep_alive_frame(runner: &TestRunner<'static, TschTask<FakeDriverConfig>>) {
        let keep_alives = runner
            .task
            .pending_operations
            .iter()
            .filter(|op| op.is_keep_alive())
            .count();
        assert_eq!(keep_alives, 1);
        let mpdu = runner
            .task
            .pending_operations
            .iter()
            .find_map(|op| match op {
                TschOperation::TxSlot {
                    mpdu,
                    origin: TxOrigin::KeepAlive,
                    ..
                } => Some(mpdu),
                _ => None,
            })
            .unwrap();
        let reader = mpdu.reader();
        assert!(reader.frame_control().ack_request());
        let parsed = reader.parse_addressing().unwrap();
        let addressing = parsed.try_addressing_fields().unwrap();
        let dst = addressing.try_dst_address().unwrap();
        assert!(matches!(&dst, Address::Extended(ext) if ext.as_ref() == COORD_ADDRESS));
    }

    #[test]
    fn test_keep_alive_disarmed_on_time_root() {
        // Time root: a period is configured but no time source exists, so
        // the keep-alive is disarmed.
        let mut runner = setup();
        setup_one_link_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.keep_alive_period = PERIOD_SLOTS;
        runner.enter_as_coordinator();
        assert!(runner.task.keep_alive_deadline(&runner.context).is_none());
        runner.terminate();
    }

    #[test]
    fn test_keep_alive_disarmed_with_zero_period() {
        // Device with a time source but a zero period: also disarmed. With a
        // non-zero period it is armed.
        let mut runner = setup_keep_alive_runner(0, setup_one_link_schedule);
        assert!(runner.task.keep_alive_deadline(&runner.context).is_none());
        runner.context.pib.tsch.keep_alive_period = PERIOD_SLOTS;
        assert!(runner.task.keep_alive_deadline(&runner.context).is_some());
        runner.terminate();
    }

    #[test]
    fn test_peek_deadline_includes_keep_alive() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_one_link_schedule);

        // Entry at t=0 armed the keep-alive at t=500ms; the first RX
        // operation (ASN 100) is only due at ~998ms, so the keep-alive
        // deadline is the earlier wake-up.
        let keep_alive_at = NsInstant::from_ticks(0) + period(PERIOD_SLOTS);
        assert_eq!(runner.task.next_keep_alive, keep_alive_at);
        assert_eq!(runner.task.peek_deadline(&runner.context), keep_alive_at);
        assert!(runner.task.op_deadline(&runner.context) > keep_alive_at);
        runner.terminate();
    }

    #[test]
    fn test_keep_alive_queued_after_idle_period() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_one_link_schedule);
        assert_eq!(runner.pending_operation_count(), 1); // RX at ASN 100

        // Wake at the keep-alive deadline: the keep-alive is built and
        // queued (displacing the same-ASN RX to the next cycle), but its own
        // timeslot is not due yet, so the scheduler re-arms and keeps
        // waiting.
        runner.set_timer_time(NsInstant::from_ticks(0) + period(PERIOD_SLOTS));
        runner.step_timer_expired();

        assert!(runner.is_idle());
        assert_eq!(
            runner.last_action().unwrap(),
            ActionType::WaitForTimeoutOrSchedulerRequest
        );
        assert_eq!(runner.pending_operation_count(), 2); // keep-alive + displaced RX
        assert_pending_keep_alive_frame(&runner);
        // Queueing re-armed the deadline one period ahead of "now".
        assert_eq!(
            runner.task.next_keep_alive,
            NsInstant::from_ticks(0) + period(PERIOD_SLOTS) + period(PERIOD_SLOTS)
        );
        runner.terminate();
    }

    #[test]
    fn test_keep_alive_sent_resyncs_and_responds_to_nobody() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_one_link_schedule);
        runner.set_timer_time(NsInstant::from_ticks(0) + period(PERIOD_SLOTS));
        runner.step_timer_expired(); // queue the keep-alive (ASN 100)
        runner.step_timer_expired(); // dispatch it

        assert!(matches!(
            runner.state(),
            TschState::WaitingForTxStart { .. }
        ));
        assert!(runner.last_request_is_tx());

        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
        let bt_before = runner.last_base_time();

        let sent_instant = NsInstant::from_ticks(1_002_120_000);
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(frame, sent_instant, Some(40i16)));

        assert!(runner.is_idle());
        assert!(!runner.last_action_has_response());
        assert_eq!(runner.last_base_time(), bt_before + NsDuration::micros(40));
        assert_eq!(runner.task.next_keep_alive, sent_instant + period(PERIOD_SLOTS));
        assert!(!runner
            .task
            .pending_operations
            .iter()
            .any(|op| op.is_keep_alive()));
        runner.terminate();
    }

    #[test]
    fn test_time_source_rx_defers_keep_alive() {
        // Period longer than the first RX slot so no keep-alive interferes.
        const LONG_PERIOD_SLOTS: u16 = 150;
        let mut runner = setup_keep_alive_runner(LONG_PERIOD_SLOTS, setup_one_link_schedule);

        runner.step_timer_expired(); // dispatch RX at ASN 100
        assert!(matches!(runner.state(), TschState::Listening));
        runner.step_driver_event(DrvSvcEvent::FrameStarted);

        let rx_instant = NsInstant::from_ticks(1_002_120_000);
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &SRC)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Received(frame, rx_instant));

        assert!(runner.is_idle());
        assert_eq!(
            runner.task.next_keep_alive,
            rx_instant + period(LONG_PERIOD_SLOTS)
        );
        assert!(!runner
            .task
            .pending_operations
            .iter()
            .any(|op| op.is_keep_alive()));
        runner.terminate();
    }

    #[test]
    fn test_keep_alive_retry_exhaustion_is_internal() {
        // Shared TX-only schedule: no RX links, so the keep-alive is the
        // only pending operation and consecutive retransmission attempts are
        // not interleaved with RX slots.
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        assert_eq!(runner.pending_operation_count(), 0);

        // Wake at the keep-alive deadline: queue it (its slot is not due).
        runner.set_timer_time(NsInstant::from_ticks(0) + period(PERIOD_SLOTS));
        runner.step_timer_expired();
        assert_pending_keep_alive_frame(&runner);
        let armed_at = runner.task.next_keep_alive;

        // macMaxFrameRetries is 3 in the test config: attempts 1..=3 fail
        // and re-queue silently, the 4th failure drops the frame - still
        // without any upper-layer response.
        for attempt in 1..=4u8 {
            runner.step_timer_expired();
            assert!(runner.last_request_is_tx());
            runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(
                1_000_000_000,
            )));
            let frame = create_addressed_data_frame(
                &runner.context.buffer_allocator,
                &SRC,
                &COORD_ADDRESS,
            )
            .into_radio_frame::<FakeDriverConfig>();
            runner.step_driver_event(DrvSvcEvent::Nack(
                frame,
                NsInstant::from_ticks(1_001_000_000),
                None,
                None,
            ));

            assert!(runner.is_idle());
            assert!(!runner.last_action_has_response());
            if attempt < 4 {
                assert_pending_keep_alive_frame(&runner);
            }
        }

        // Dropped internally: nothing pending, and the deadline armed when
        // the keep-alive was queued still stands, so the next keep-alive
        // goes out a full period later.
        assert_eq!(runner.pending_operation_count(), 0);
        assert_eq!(runner.task.next_keep_alive, armed_at);
        runner.terminate();
    }

    // ========================================================================
    // Desynchronization: missed keep-alive counting
    // ========================================================================

    /// Wake the runner at the current keep-alive deadline, so that
    /// `maybe_queue_keep_alive` sees the period as elapsed, and step the
    /// state machine on it.
    fn wake_at_keep_alive_deadline(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) {
        let deadline = runner.task.next_keep_alive;
        runner.set_timer_time(deadline);
        runner.step_timer_expired();
    }

    /// Let one keep-alive period elapse and run the bookkeeping alone, without
    /// stepping the state machine into the slot that follows.
    fn elapse_keep_alive_period(runner: &mut TestRunner<'static, TschTask<FakeDriverConfig>>) {
        let deadline = runner.task.next_keep_alive;
        runner.set_timer_time(deadline);
        runner.task.maybe_queue_keep_alive(&mut runner.context);
    }

    #[test]
    fn test_missed_keep_alives_counts_silent_periods() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        assert_eq!(runner.task.missed_keep_alives, 0);

        // An elapsed period with no exchange counts once, when the keep-alive
        // it triggers is queued.
        elapse_keep_alive_period(&mut runner);
        assert_eq!(runner.task.missed_keep_alives, 1);
        assert_pending_keep_alive_frame(&runner);

        // A keep-alive still pending is the same outstanding probe, not a new
        // silent period: the count does not move.
        elapse_keep_alive_period(&mut runner);
        assert_eq!(runner.task.missed_keep_alives, 1);

        runner.terminate();
    }

    #[test]
    fn test_time_source_rx_clears_missed_keep_alives() {
        const LONG_PERIOD_SLOTS: u16 = 150;
        let mut runner = setup_keep_alive_runner(LONG_PERIOD_SLOTS, setup_one_link_schedule);
        runner.task.missed_keep_alives = 2;

        runner.step_timer_expired(); // dispatch RX at ASN 100
        runner.step_driver_event(DrvSvcEvent::FrameStarted);
        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &COORD_ADDRESS, &SRC)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Received(
            frame,
            NsInstant::from_ticks(1_002_120_000),
        ));

        // A frame heard from the time source proves the node is synchronized.
        assert_eq!(runner.task.missed_keep_alives, 0);
        runner.terminate();
    }

    #[test]
    fn test_acked_data_frame_to_time_source_clears_missed_keep_alives() {
        // The other loss source the mechanism has to see: an ordinary data
        // frame that the time source acknowledged is as good as a keep-alive.
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        runner.task.missed_keep_alives = 2;

        let mpdu =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS);
        runner.step_scheduler_request(SchedulerRequest::Transmission(mpdu));
        runner.step_timer_expired(); // dispatch the TX
        runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));

        let frame =
            create_addressed_data_frame(&runner.context.buffer_allocator, &SRC, &COORD_ADDRESS)
                .into_radio_frame::<FakeDriverConfig>();
        runner.step_driver_event(DrvSvcEvent::Sent(
            frame,
            NsInstant::from_ticks(1_002_120_000),
            None,
        ));

        assert_eq!(runner.task.missed_keep_alives, 0);
        runner.terminate();
    }

    #[test]
    fn test_missed_keep_alives_frozen_without_time_source() {
        // Time root: the keep-alive is disarmed, so no silence is ever
        // counted and the node can never disconnect itself.
        let mut runner = setup();
        setup_shared_tx_only_schedule(&mut runner.context.pib.tsch);
        runner.context.pib.tsch.keep_alive_period = PERIOD_SLOTS;
        runner.enter_as_coordinator();

        assert!(runner.task.keep_alive_deadline(&runner.context).is_none());
        for _ in 0..(MAC_TSCH_MAX_KEEP_ALIVE_MISSES as usize + 2) {
            runner.advance_timer(period(PERIOD_SLOTS));
            runner.task.maybe_queue_keep_alive(&mut runner.context);
            assert_eq!(runner.task.missed_keep_alives, 0);
        }
        runner.terminate();
    }

    // ========================================================================
    // Desynchronization: disconnect
    // ========================================================================

    #[test]
    fn test_disconnect_after_max_missed_keep_alives() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);

        // One short of the threshold: still in TSCH.
        runner.task.missed_keep_alives = MAC_TSCH_MAX_KEEP_ALIVE_MISSES - 1;
        wake_at_keep_alive_deadline(&mut runner);
        assert_eq!(runner.task.missed_keep_alives, MAC_TSCH_MAX_KEEP_ALIVE_MISSES);
        assert_eq!(runner.last_action().unwrap(), ActionType::Completed);

        assert_eq!(runner.pending_operation_count(), 0);
        assert!(runner.context.pib.tsch.time_source.is_none());

        // The former time source is evictable again, so a node that joins a
        // different parent later does not accumulate un-evictable entries.
        let former = runner
            .context
            .pib
            .neighbors
            .find_ext(&COORD_ADDRESS[..])
            .unwrap();
        assert!(!runner
            .context
            .pib
            .neighbors
            .get(former)
            .unwrap()
            .is_pinned());
    }

    // ========================================================================
    // Guard time: relaxation and recovery
    // ========================================================================

    /// Guard time short enough that `rx_window_us` widens it while
    /// uncommitted: below `MAC_TSCH_GUARD_FLOOR_US`.
    const SHORT_GUARD_US: u16 = 80;

    #[test]
    fn test_missed_period_relaxes_committed_guard() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        runner.context.pib.tsch.timeslot_timings = TschTimeslotTimings::new(1, SHORT_GUARD_US);
        runner.context.pib.tsch.force_guard_committed();

        // Committed: the node listens with the advertised (short) window.
        let (_, rx_wait_us) = runner.context.pib.tsch.rx_window_us(0);
        assert_eq!(rx_wait_us, SHORT_GUARD_US as u64);

        wake_at_keep_alive_deadline(&mut runner);

        // One silent period is enough to widen the window again, well before
        // the node gives up on the network.
        assert!(!runner.context.pib.tsch.is_guard_committed());
        let (_, rx_wait_us) = runner.context.pib.tsch.rx_window_us(0);
        assert!(rx_wait_us > SHORT_GUARD_US as u64);
        assert!(runner.task.missed_keep_alives < MAC_TSCH_MAX_KEEP_ALIVE_MISSES);

        runner.terminate();
    }

    #[test]
    fn test_guard_relaxation_keeps_the_drift_estimate() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        runner.context.pib.tsch.timeslot_timings = TschTimeslotTimings::new(1, SHORT_GUARD_US);
        runner.context.pib.tsch.set_drift_ns(7);
        runner.context.pib.tsch.force_guard_committed();

        wake_at_keep_alive_deadline(&mut runner);

        assert!(!runner.context.pib.tsch.is_guard_committed());
        assert_eq!(runner.drift_ns(), 7);
        runner.terminate();
    }

    #[test]
    fn test_long_guard_is_unaffected_by_relaxation() {
        // A network advertising the default 2200 us guard never widens, so
        // relaxing is a no-op on the receive window.
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_shared_tx_only_schedule);
        let before = runner.context.pib.tsch.rx_window_us(0);

        wake_at_keep_alive_deadline(&mut runner);

        assert_eq!(runner.context.pib.tsch.rx_window_us(0), before);
        assert_eq!(before.1, RX_WAIT_US);
        runner.terminate();
    }

    #[test]
    fn test_set_keep_alive_period_attribute() {
        let mut runner = setup_keep_alive_runner(PERIOD_SLOTS, setup_one_link_schedule);

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Set(SetRequestAttribute::MacKeepAlivePeriod(0)),
        )));
        assert!(runner.last_response_is_command());
        assert_eq!(runner.context.pib.tsch.keep_alive_period, 0);
        assert!(runner.task.keep_alive_deadline(&runner.context).is_none());

        runner.step_scheduler_request(SchedulerRequest::Command(SchedulerCommand::PibCommand(
            PibCommand::Set(SetRequestAttribute::MacKeepAlivePeriod(75)),
        )));
        assert!(runner.last_response_is_command());
        assert_eq!(runner.context.pib.tsch.keep_alive_period, 75);
        assert!(runner.task.keep_alive_deadline(&runner.context).is_some());
        runner.terminate();
    }
}

// ============================================================================
// Fuzz harness: TSCH state machine under adversarial events (target T4)
// ============================================================================

#[cfg(feature = "_fuzz-harness")]
pub(crate) fn fuzz_step(data: &[u8]) {
    const ADDR_A: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const ADDR_B: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];

    let mut u = arbitrary::Unstructured::new(data);

    let mut runner = setup();
    setup_one_link_schedule(&mut runner.context.pib.tsch);
    runner.enter_as_coordinator();

    // Send the first scheduled beacon, so the machine settles into its slot
    // cycle. This mirrors the beacon-skip helper used by the tests.
    runner.step_timer_expired();
    runner.step_driver_event(DrvSvcEvent::TxStarted(NsInstant::from_ticks(1_000_000_000)));
    let beacon = create_addressed_data_frame(&runner.context.buffer_allocator, &ADDR_A, &ADDR_B)
        .into_radio_frame::<FakeDriverConfig>();
    runner.step_driver_event(DrvSvcEvent::Sent(
        beacon,
        NsInstant::from_ticks(1_001_000_000),
        None,
    ));

    let leaked_before = dot15d4_util::tokens::leaked_tokens();

    let rounds = u.arbitrary::<u8>().unwrap_or(0) % 32;
    let mut tick: u64 = 2_000_000_000;
    for _ in 0..rounds {
        runner.step_timer_expired();

        // Only a listening slot accepts receive events. Other slots (for
        // example a beacon transmission) are left to run on their own.
        if matches!(runner.state(), TschState::Listening) {
            match u.arbitrary::<u8>().unwrap_or(0) % 4 {
                0 => {
                    // A frame is received. Source and destination are arbitrary,
                    // which exercises neighbor handling and address filtering.
                    runner.step_driver_event(DrvSvcEvent::FrameStarted);
                    let mut src = [0u8; 8];
                    let mut dst = [0u8; 8];
                    let _ = u.fill_buffer(&mut src);
                    let _ = u.fill_buffer(&mut dst);
                    let frame = create_addressed_data_frame(
                        &runner.context.buffer_allocator,
                        &src,
                        &dst,
                    )
                    .into_radio_frame::<FakeDriverConfig>();
                    runner.step_driver_event(DrvSvcEvent::Received(
                        frame,
                        NsInstant::from_ticks(tick + 120_000),
                    ));
                }
                1 => {
                    // The radio reports a CRC error and returns the buffer.
                    runner.step_driver_event(DrvSvcEvent::FrameStarted);
                    let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
                    runner.step_driver_event(DrvSvcEvent::CrcError(
                        rx_frame,
                        NsInstant::from_ticks(tick),
                    ));
                }
                2 => {
                    // The receive window ends with no frame.
                    let rx_frame = create_unsized_radio_frame(&runner.context.buffer_allocator);
                    runner.step_driver_event(DrvSvcEvent::RxWindowEnded(rx_frame));
                }
                _ => {
                    // Reception starts, then aborts.
                    runner.step_driver_event(DrvSvcEvent::FrameStarted);
                    runner.step_driver_event(DrvSvcEvent::ReceptionAborted);
                }
            }
        }

        tick += 1_000_000;
    }

    // Clean teardown returns every outstanding buffer to the pool.
    runner.terminate();

    let leaked_after = dot15d4_util::tokens::leaked_tokens();
    assert_eq!(
        leaked_after, leaked_before,
        "a frame buffer was dropped instead of returned"
    );
}
