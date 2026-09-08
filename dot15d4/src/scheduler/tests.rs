//! Test infrastructure and helpers for scheduler testing.
//!
//! This module provides common test utilities used by both CSMA and TSCH
//! scheduler tests. It is also compiled as library code by the `_fuzz-harness`
//! feature, which reuses this infrastructure to drive the fuzz targets.

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

use core::{cell::Cell, marker::PhantomData, num::NonZero};

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{
            AddressingMode, AddressingRepr, FrameType, FrameVersion, PanIdCompressionRepr,
            RadioFrame, RadioFrameSized, RadioFrameUnsized,
        },
        phy::{OQpsk250KBit, Phy},
        DriverConfig, FcsTwoBytes,
    },
    timer::{
        HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
        OptionalNsInstant, RadioTimerApi, RadioTimerError, TimedSignal,
    },
};
use dot15d4_frame::{
    mpdu::MpduFrame,
    repr::{MpduRepr, SeqNrRepr},
    MpduWithIes,
};
use dot15d4_util::{
    allocator::{BufferAllocator, BufferAllocatorBackend, BufferToken, IntoBuffer},
    sync::Channel as SyncChannel,
};
use embassy_sync::channel::Channel as EmbassyChannel;
use rand_core::RngCore;

use crate::{
    driver::{DriverEventChannel, DriverRequestChannel, DrvSvcEvent, DrvSvcRequest, Timestamp},
    mac::MacBufferAllocator,
    scheduler::{
        MessageType, SchedulerAction, SchedulerCommandResult, SchedulerContext, SchedulerRequest,
        SchedulerResponse, SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition,
        SchedulerTransmissionResult, SCHEDULER_CHANNEL_BACKLOG, SCHEDULER_CHANNEL_CAPACITY,
    },
};

use super::{
    command::{CsmaCommandResult, UseCsmaResult},
    SchedulerReceptionResult,
};

/// Fake radio timer for testing.
#[derive(Clone, Copy)]
pub struct FakeRadioTimer<State: Clone> {
    state: PhantomData<State>,
    current_time: NsInstant,
}

impl<State: Clone> FakeRadioTimer<State> {
    pub fn new() -> Self {
        Self {
            state: PhantomData,
            current_time: NsInstant::from_ticks(0),
        }
    }

    pub fn set_time(&mut self, instant: NsInstant) {
        self.current_time = instant;
    }

    #[allow(dead_code)]
    pub fn advance(&mut self, duration: NsDuration) {
        let new_time = self.current_time + duration;
        self.current_time = new_time;
    }
}

#[derive(Clone, Copy)]
pub struct Sleeping;

#[derive(Clone, Copy)]
pub struct Running;

impl RadioTimerApi for FakeRadioTimer<Sleeping> {
    const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);
    const GUARD_TIME: NsDuration = NsDuration::from_ticks(0);
    type HighPrecisionTimer = FakeRadioTimer<Running>;

    fn now(&self) -> NsInstant {
        self.current_time
    }

    async unsafe fn wait_until(&mut self, _instant: NsInstant) -> Result<(), RadioTimerError> {
        todo!()
    }

    fn start_high_precision_timer(
        &self,
        _start_at: OptionalNsInstant,
    ) -> Result<Self::HighPrecisionTimer, RadioTimerError> {
        todo!()
    }
}

impl HighPrecisionTimer for FakeRadioTimer<Running> {
    const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);

    fn schedule_timed_signal(&self, _timed_signal: TimedSignal) -> Result<&Self, RadioTimerError> {
        todo!()
    }

    fn schedule_timed_signal_unless(
        &self,
        _timed_signal: TimedSignal,
        _event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        todo!()
    }

    fn schedule_timed_signal_if(
        &self,
        _timed_signal: TimedSignal,
        _event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        todo!()
    }

    fn commit_timed_signal(&self) {
        todo!()
    }

    async unsafe fn wait_for(&mut self, _signal: HardwareSignal) {
        todo!()
    }

    fn observe_event(&self, _event: HardwareEvent) -> Result<&Self, RadioTimerError> {
        todo!()
    }

    fn poll_event(&self, _event: HardwareEvent) -> OptionalNsInstant {
        todo!()
    }

    fn reset(&self) {
        todo!()
    }
}

// ============================================================================
// Fake Driver Config
// ============================================================================

pub struct FakeDriverConfig;

impl DriverConfig for FakeDriverConfig {
    type Phy = Phy<OQpsk250KBit>;
    type Fcs = FcsTwoBytes;
    type Timer = FakeRadioTimer<Sleeping>;

    const HEADROOM: u8 = 1;
    const TAILROOM: u8 = 2;
    const MAX_SDU_LENGTH: u16 = 127;
}

// ============================================================================
// Deterministic RNG
// ============================================================================

pub struct DeterministicRng {
    pub(crate) values: Vec<u32>,
    pub(crate) index: Cell<usize>,
    pub(crate) call_count: Cell<usize>,
    pub(crate) default_value: u32,
}

impl DeterministicRng {
    pub fn new(values: Vec<u32>) -> Self {
        Self {
            values,
            index: Cell::new(0),
            call_count: Cell::new(0),
            default_value: 0,
        }
    }

    pub fn constant(value: u32) -> Self {
        Self {
            values: vec![value],
            index: Cell::new(0),
            call_count: Cell::new(0),
            default_value: value,
        }
    }

    pub fn call_count(&self) -> usize {
        self.call_count.get()
    }

    pub fn reset_count(&self) {
        self.call_count.set(0);
    }

    pub fn reset_index(&self) {
        self.index.set(0);
    }
}

impl RngCore for DeterministicRng {
    fn next_u32(&mut self) -> u32 {
        self.call_count.set(self.call_count.get() + 1);
        if self.values.is_empty() {
            return self.default_value;
        }
        let idx = self.index.get() % self.values.len();
        self.index.set(idx + 1);
        self.values[idx]
    }

    fn next_u64(&mut self) -> u64 {
        self.next_u32() as u64
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for byte in dest.iter_mut() {
            *byte = self.next_u32() as u8;
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

// ============================================================================
// Test Buffer Allocator
// ============================================================================

const TEST_BUFFER_SIZE: usize = 130;
const TEST_BUFFER_COUNT: usize = 16;

type TestAllocatorBackend = BufferAllocatorBackend<TEST_BUFFER_SIZE, TEST_BUFFER_COUNT>;

static INTENTIONAL_LEAKS: std::sync::Mutex<Vec<LeakedAllocation>> =
    std::sync::Mutex::new(Vec::new());

struct LeakedAllocation {
    _keep_alive: *const (),
}

// Safety: The pointer is never dereferenced; it only keeps the allocation
// visible to miri's leak checker.
unsafe impl Send for LeakedAllocation {}

fn leak_for_test<T: 'static>(value: T) -> &'static mut T {
    let leaked = Box::leak(Box::new(value));
    INTENTIONAL_LEAKS.lock().unwrap().push(LeakedAllocation {
        _keep_alive: leaked as *mut T as *const (),
    });
    leaked
}

pub(crate) fn create_test_allocator() -> MacBufferAllocator {
    thread_local! {
        static TEST_ALLOCATOR: MacBufferAllocator = {
            let backend: &'static mut TestAllocatorBackend =
                leak_for_test(BufferAllocatorBackend::new());
            let pinned: &'static core::pin::Pin<&'static TestAllocatorBackend> =
                leak_for_test(TestAllocatorBackend::pin(backend));
            BufferAllocator::new(pinned)
        };
    }

    TEST_ALLOCATOR.with(|allocator| *allocator)
}

// ============================================================================
// Per-Thread Channels and RNG for Testing
// ============================================================================

type TestSchedulerChannel = SyncChannel<
    MessageType,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;

pub(crate) fn get_scheduler_channel() -> &'static TestSchedulerChannel {
    thread_local! {
        static SCHEDULER_REQUEST_CHANNEL: &'static TestSchedulerChannel =
            leak_for_test(SyncChannel::new());
    }

    SCHEDULER_REQUEST_CHANNEL.with(|channel| *channel)
}

pub(crate) fn driver_request_channel() -> &'static DriverRequestChannel {
    thread_local! {
        static DRIVER_REQUEST_CHANNEL: &'static DriverRequestChannel =
            leak_for_test(EmbassyChannel::new());
    }

    DRIVER_REQUEST_CHANNEL.with(|channel| *channel)
}

pub(crate) fn driver_event_channel() -> &'static DriverEventChannel {
    thread_local! {
        static DRIVER_EVENT_CHANNEL: &'static DriverEventChannel =
            leak_for_test(EmbassyChannel::new());
    }

    DRIVER_EVENT_CHANNEL.with(|channel| *channel)
}

pub(crate) fn create_test_rng() -> &'static mut DeterministicRng {
    thread_local! {
        static TEST_RNG: *mut DeterministicRng =
            leak_for_test(DeterministicRng::constant(1));
    }

    let rng = TEST_RNG.with(|rng| *rng);
    // Safety: The pointer comes from a leaked box owned by this thread alone, so
    // it stays valid for the lifetime of the thread and no other thread can
    // reach it. The setup helpers build one runner at a time, so no second
    // reference to the RNG is alive while this one is in use.
    let rng = unsafe { &mut *rng };
    rng.values = vec![1];
    rng.reset_index();
    rng.reset_count();
    rng
}

// ============================================================================
// Timing Calculation Helpers
// ============================================================================

pub mod timing {
    use dot15d4_driver::timer::NsDuration;

    pub const SYMBOL_PERIOD_US: u64 = 16;
    pub const MAC_UNIT_BACKOFF_PERIOD: NsDuration = NsDuration::micros(20 * SYMBOL_PERIOD_US);
    pub const FIXED_RMARKER_OVERHEAD: NsDuration = NsDuration::micros(580);
    pub const MIN_DELAY: NsDuration = NsDuration::micros(730);

    pub fn calculate_backoff_delay(be: u8, rng_value: u32) -> NsDuration {
        let max = (1u32 << be).saturating_sub(1);
        let backoff_periods = if max == 0 { 0 } else { rng_value % (max + 1) };
        let standard_delay_us = MAC_UNIT_BACKOFF_PERIOD.to_micros() * backoff_periods as u64;
        let total_delay_us = standard_delay_us + FIXED_RMARKER_OVERHEAD.to_micros();

        if total_delay_us < MIN_DELAY.to_micros() {
            MIN_DELAY
        } else {
            NsDuration::micros(total_delay_us)
        }
    }
}

// ============================================================================
// Driver Request Info - Rich introspection of driver requests
// ============================================================================

/// Detailed information about a driver request, extracted for test verification.
#[derive(Debug, Clone)]
pub enum DriverRequestInfo {
    /// TX request details
    Tx {
        /// Timestamp for transmission
        timestamp: Timestamp,
        /// Whether CCA is enabled
        cca: bool,
        /// Channel (if specified)
        channel: Option<Channel>,
        /// Whether driver should fallback on NACK
        fallback_on_nack: bool,
    },
    /// RX request details
    Rx {
        /// Start timestamp
        start: Timestamp,
        /// Channel (if specified)
        channel: Option<Channel>,
        /// RX window end instant (if specified)
        rx_window_end: Option<NsInstant>,
        /// Expected RX framestart instant (TSCH)
        #[cfg(feature = "tsch")]
        expected_rx_framestart: Option<NsInstant>,
    },
    /// Go idle request
    GoIdle,
}

impl DriverRequestInfo {
    /// Check if this is a TX request
    pub fn is_tx(&self) -> bool {
        matches!(self, Self::Tx { .. })
    }

    /// Check if this is an RX request
    pub fn is_rx(&self) -> bool {
        matches!(self, Self::Rx { .. })
    }

    /// Check if this is a go-idle request
    pub fn is_go_idle(&self) -> bool {
        matches!(self, Self::GoIdle)
    }

    /// Get TX timestamp if this is a TX request
    pub fn tx_timestamp(&self) -> Option<Timestamp> {
        match self {
            Self::Tx { timestamp, .. } => Some(*timestamp),
            _ => None,
        }
    }

    /// Check if TX is best effort
    pub fn is_best_effort(&self) -> bool {
        matches!(
            self,
            Self::Tx {
                timestamp: Timestamp::BestEffort,
                ..
            }
        )
    }

    /// Get scheduled instant if TX is scheduled
    pub fn scheduled_instant(&self) -> Option<NsInstant> {
        match self {
            Self::Tx {
                timestamp: Timestamp::Scheduled(instant),
                ..
            } => Some(*instant),
            _ => None,
        }
    }

    /// Check if CCA is enabled (for TX)
    pub fn cca_enabled(&self) -> bool {
        matches!(self, Self::Tx { cca: true, .. })
    }

    /// Get channel if specified
    pub fn channel(&self) -> Option<Channel> {
        match self {
            Self::Tx { channel, .. } | Self::Rx { channel, .. } => *channel,
            Self::GoIdle => None,
        }
    }

    /// Get RX window end instant (if specified)
    pub fn rx_window_end(&self) -> Option<NsInstant> {
        match self {
            Self::Rx { rx_window_end, .. } => *rx_window_end,
            _ => None,
        }
    }

    /// Get RX start timestamp
    pub fn rx_start_timestamp(&self) -> Option<Timestamp> {
        match self {
            Self::Rx { start, .. } => Some(*start),
            _ => None,
        }
    }

    /// Get scheduled RX start instant
    pub fn rx_scheduled_instant(&self) -> Option<NsInstant> {
        match self {
            Self::Rx {
                start: Timestamp::Scheduled(instant),
                ..
            } => Some(*instant),
            _ => None,
        }
    }

    /// Get expected RX framestart (TSCH)
    #[cfg(feature = "tsch")]
    pub fn expected_rx_framestart(&self) -> Option<NsInstant> {
        match self {
            Self::Rx {
                expected_rx_framestart,
                ..
            } => *expected_rx_framestart,
            _ => None,
        }
    }
}

/// Scheduler action type for test verification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionType {
    SendDriverRequestThenWait,
    WaitForDriverEvent,
    WaitForSchedulerRequest,
    SelectDriverEventOrRequest,
    #[cfg(feature = "tsch")]
    WaitForTimeoutOrSchedulerRequest,
    SendDriverRequestThenSelect,
    Completed,
}

/// Response type for test verification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResponseType {
    Sent,
    ChannelAccessFailure,
    NoAck,
    Reception,
    Command,
}

// ============================================================================
// Synchronous Test Runner
// ============================================================================

/// Synchronous test runner for CSMA task.
pub struct TestRunner<'a, Task: SchedulerTask<FakeDriverConfig>> {
    pub task: Task,
    pub context: SchedulerContext<'a, FakeDriverConfig>,

    // Collected information from last transition
    last_request_info: Option<DriverRequestInfo>,
    last_action_type: Option<ActionType>,
    last_response_type: Option<ResponseType>,
    #[cfg(feature = "tsch")]
    last_deadline: Option<NsInstant>,

    // Buffer management - all buffers that need deallocation
    pending_buffers: Vec<BufferToken>,
}

impl<'a, Task: SchedulerTask<FakeDriverConfig>> TestRunner<'a, Task> {
    pub fn new(context: SchedulerContext<'a, FakeDriverConfig>, task: Task) -> Self {
        Self {
            task,
            context,
            last_request_info: None,
            last_action_type: None,
            last_response_type: None,
            #[cfg(feature = "tsch")]
            last_deadline: None,
            pending_buffers: Vec::new(),
        }
    }

    // ========================================================================
    // Step Methods
    // ========================================================================

    /// Step with Entry event.
    pub fn step_entry(&mut self) {
        let transition = self.task.step(SchedulerTaskEvent::Entry, &mut self.context);
        self.process_transition(transition);
    }

    /// Step with a driver event.
    pub fn step_driver_event(&mut self, event: DrvSvcEvent) {
        let transition = self
            .task
            .step(SchedulerTaskEvent::DriverEvent(event), &mut self.context);
        self.process_transition(transition);
    }

    /// Step with a scheduler request.
    pub fn step_scheduler_request(&mut self, request: SchedulerRequest) {
        let channel = get_scheduler_channel();
        let sender = channel.sender();
        if let Some(token) = sender.try_allocate_request_token() {
            sender.send_request_no_response(token, request);
        } else {
            panic!("Failed to allocate request token");
        }

        let (token, request) = self
            .context
            .request_receiver
            .try_receive_request(&MessageType::TxOrCommand)
            .expect("no request available in channel");

        let transition = self.task.step(
            SchedulerTaskEvent::SchedulerRequest { token, request },
            &mut self.context,
        );
        self.process_transition(transition);
    }

    /// Step with timer expired event.
    pub fn step_timer_expired(&mut self) {
        let transition = self
            .task
            .step(SchedulerTaskEvent::TimerExpired, &mut self.context);
        self.process_transition(transition);
    }

    // ========================================================================
    // Transition Processing
    // ========================================================================

    fn process_transition(&mut self, transition: SchedulerTaskTransition) {
        match transition {
            SchedulerTaskTransition::Execute(action, response) => {
                self.last_action_type = Some(Self::action_to_type(&action));
                self.process_action(action);

                if let Some((token, resp)) = response {
                    self.last_response_type = Some(Self::response_to_type(&resp));
                    self.process_response_with_token(token, resp);
                }
            }
            SchedulerTaskTransition::Completed(_, response) => {
                self.last_action_type = Some(ActionType::Completed);
                self.last_request_info = None;

                if let Some((token, resp)) = response {
                    self.last_response_type = Some(Self::response_to_type(&resp));
                    self.process_response_with_token(token, resp);
                }
            }
        }
    }

    fn process_action(&mut self, action: SchedulerAction) {
        #[cfg(feature = "tsch")]
        {
            if let SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline } = &action {
                self.last_deadline = Some(*deadline);
            } else {
                self.last_deadline = None;
            }
        }
        match action {
            SchedulerAction::SendDriverRequestThenWait(req)
            | SchedulerAction::SendDriverRequestThenSelect(req) => {
                self.process_driver_request(req);
            }
            _ => {
                self.last_request_info = None;
            }
        }
    }

    fn process_driver_request(&mut self, request: DrvSvcRequest) {
        match request {
            DrvSvcRequest::CompleteThenStartTx(tx) => {
                self.last_request_info = Some(DriverRequestInfo::Tx {
                    timestamp: tx.at,
                    cca: tx.cca,
                    channel: tx.channel,
                    fallback_on_nack: tx.fallback_on_nack,
                });
                // Store the TX buffer for later deallocation
                self.pending_buffers.push(tx.mpdu.into_buffer());
            }
            DrvSvcRequest::CompleteThenStartRx(rx) => {
                self.last_request_info = Some(DriverRequestInfo::Rx {
                    start: rx.start,
                    channel: rx.channel,
                    rx_window_end: rx.rx_window_end,
                    #[cfg(feature = "tsch")]
                    expected_rx_framestart: rx.expected_rx_framestart,
                });
                // Store the RX buffer for later deallocation
                self.pending_buffers.push(rx.radio_frame.into_buffer());
            }
            DrvSvcRequest::CompleteThenGoIdle => {
                self.last_request_info = Some(DriverRequestInfo::GoIdle);
            }
        }
    }

    fn process_response_with_token(
        &mut self,
        response_token: dot15d4_util::sync::ResponseToken,
        response: SchedulerResponse,
    ) {
        // Extract and store any buffers from the response
        match response {
            SchedulerResponse::Transmission(result) => match result {
                SchedulerTransmissionResult::Sent(frame, _) => {
                    self.pending_buffers.push(frame.into_buffer());
                }
                SchedulerTransmissionResult::ChannelAccessFailure(frame) => {
                    self.pending_buffers.push(frame.into_buffer());
                }
                SchedulerTransmissionResult::NoAck(frame, _) => {
                    self.pending_buffers.push(frame.into_buffer());
                }
            },
            SchedulerResponse::Reception(reception_result) => match reception_result {
                SchedulerReceptionResult::Data(radio_frame, _)
                | SchedulerReceptionResult::Beacon(radio_frame, _)
                | SchedulerReceptionResult::Command(radio_frame, _) => {
                    self.pending_buffers.push(radio_frame.into_buffer());
                }
            },
            SchedulerResponse::Command(_) => {}
        }

        // Return the response token to the channel using a dummy response
        // (the response is dropped anyway for RequestNoResponse slots)
        self.context.request_receiver.received(
            response_token,
            SchedulerResponse::Command(SchedulerCommandResult::CsmaCommand(
                CsmaCommandResult::UseCsma(UseCsmaResult::Success),
            )),
        );
    }

    fn action_to_type(action: &SchedulerAction) -> ActionType {
        match action {
            SchedulerAction::SendDriverRequestThenWait(_) => ActionType::SendDriverRequestThenWait,
            SchedulerAction::WaitForDriverEvent => ActionType::WaitForDriverEvent,
            SchedulerAction::WaitForSchedulerRequest => ActionType::WaitForSchedulerRequest,
            SchedulerAction::SelectDriverEventOrRequest => ActionType::SelectDriverEventOrRequest,
            #[cfg(feature = "tsch")]
            SchedulerAction::WaitForTimeoutOrSchedulerRequest { .. } => {
                ActionType::WaitForTimeoutOrSchedulerRequest
            }
            SchedulerAction::SendDriverRequestThenSelect(_) => {
                ActionType::SendDriverRequestThenSelect
            }
        }
    }

    fn response_to_type(resp: &SchedulerResponse) -> ResponseType {
        match resp {
            SchedulerResponse::Transmission(SchedulerTransmissionResult::Sent(..)) => {
                ResponseType::Sent
            }
            SchedulerResponse::Transmission(SchedulerTransmissionResult::ChannelAccessFailure(
                ..,
            )) => ResponseType::ChannelAccessFailure,
            SchedulerResponse::Transmission(SchedulerTransmissionResult::NoAck(..)) => {
                ResponseType::NoAck
            }
            SchedulerResponse::Reception(..) => ResponseType::Reception,
            SchedulerResponse::Command(..) => ResponseType::Command,
        }
    }

    // ========================================================================
    // Introspection - Last Request
    // ========================================================================

    /// Get detailed info about the last driver request.
    pub fn last_request(&self) -> Option<&DriverRequestInfo> {
        self.last_request_info.as_ref()
    }

    /// Check if the last request was TX.
    pub fn last_request_is_tx(&self) -> bool {
        self.last_request_info.as_ref().map_or(false, |r| r.is_tx())
    }

    /// Check if the last request was RX.
    pub fn last_request_is_rx(&self) -> bool {
        self.last_request_info.as_ref().map_or(false, |r| r.is_rx())
    }

    /// Check if the last TX was best effort.
    pub fn last_tx_is_best_effort(&self) -> bool {
        self.last_request_info
            .as_ref()
            .map_or(false, |r| r.is_best_effort())
    }

    /// Get the scheduled instant from the last TX request.
    pub fn last_tx_scheduled_instant(&self) -> Option<NsInstant> {
        self.last_request_info
            .as_ref()
            .and_then(|r| r.scheduled_instant())
    }

    /// Verify that the last TX was scheduled at the expected time.
    pub fn verify_tx_scheduled_at(&self, expected: NsInstant) -> bool {
        self.last_tx_scheduled_instant() == Some(expected)
    }

    /// Get the type of the last action.
    pub fn last_action(&self) -> Option<ActionType> {
        self.last_action_type
    }

    /// Check if the last action has a response.
    pub fn last_action_has_response(&self) -> bool {
        self.last_response_type.is_some()
    }

    /// Check if the last response was Sent.
    pub fn last_response_is_sent(&self) -> bool {
        self.last_response_type == Some(ResponseType::Sent)
    }

    /// Check if the last response was ChannelAccessFailure.
    pub fn last_response_is_channel_access_failure(&self) -> bool {
        self.last_response_type == Some(ResponseType::ChannelAccessFailure)
    }

    /// Check if the last response was NoAck.
    pub fn last_response_is_no_ack(&self) -> bool {
        self.last_response_type == Some(ResponseType::NoAck)
    }

    /// Check if the last response was NoAck.
    pub fn last_response_is_command(&self) -> bool {
        self.last_response_type == Some(ResponseType::Command)
    }

    /// Check if the last response was a Reception.
    pub fn last_response_is_reception(&self) -> bool {
        self.last_response_type == Some(ResponseType::Reception)
    }

    /// Get the deadline from the last WaitForTimeoutOrSchedulerRequest action.
    #[cfg(feature = "tsch")]
    pub fn last_deadline(&self) -> Option<NsInstant> {
        self.last_deadline
    }

    // ========================================================================
    // PIB Configuration
    // ========================================================================

    pub fn set_max_csma_backoffs(&mut self, value: u8) {
        self.context.pib.max_csma_backoffs = value;
    }

    pub fn set_max_frame_retries(&mut self, value: u8) {
        self.context.pib.max_frame_retries = value;
    }

    pub fn set_max_be(&mut self, value: u8) {
        self.context.pib.max_be = value;
    }

    // ========================================================================
    // Helper Methods
    // ========================================================================

    /// Create an RxWindowEnded event with a fresh frame.
    /// Call this before TxStarted when transitioning from RX to TX.
    pub fn create_rx_window_ended_event(&self) -> DrvSvcEvent {
        let frame = create_unsized_radio_frame(&self.context.buffer_allocator);
        DrvSvcEvent::RxWindowEnded(frame)
    }

    /// Deallocate all pending buffers.
    pub(super) fn deallocate_all_buffers(&mut self) {
        for buffer in self.pending_buffers.drain(..) {
            unsafe {
                self.context.buffer_allocator.deallocate_buffer(buffer);
            }
        }
    }

    /// Clear collected state for next test phase.
    pub fn clear_collected(&mut self) {
        self.last_request_info = None;
        self.last_action_type = None;
        self.last_response_type = None;
        #[cfg(feature = "tsch")]
        {
            self.last_deadline = None;
        }
    }
}

impl<'a, Task: SchedulerTask<FakeDriverConfig>> Drop for TestRunner<'a, Task> {
    fn drop(&mut self) {
        self.deallocate_all_buffers();
    }
}

// ============================================================================
// Frame Creation Helpers
// ============================================================================

pub fn create_test_mpdu_with_ack(allocator: &MacBufferAllocator) -> MpduFrame {
    let buffer = allocator
        .try_allocate_buffer(TEST_BUFFER_SIZE)
        .expect("allocation failed");

    const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
        .with_frame_control(SeqNrRepr::Yes)
        .with_addressing(AddressingRepr::new(
            AddressingMode::Extended,
            AddressingMode::Extended,
            false,
            PanIdCompressionRepr::No,
        ))
        .without_security()
        .without_ies();

    let payload: [u8; 10] = [0u8; 10];

    let mut mpdu_writer = MPDU_REPR
        .into_writer::<FakeDriverConfig>(
            FrameVersion::Ieee802154,
            FrameType::Data,
            payload.len() as u16,
            buffer,
        )
        .unwrap();

    mpdu_writer.set_sequence_number(42);
    mpdu_writer.set_ack_request(true);
    mpdu_writer.frame_payload_mut().copy_from_slice(&payload);

    mpdu_writer.into_mpdu_frame()
}

pub fn create_sized_radio_frame(
    allocator: &MacBufferAllocator,
    size: u16,
) -> RadioFrame<RadioFrameSized> {
    let buffer = allocator
        .try_allocate_buffer(TEST_BUFFER_SIZE)
        .expect("allocation failed");
    let frame = RadioFrame::<RadioFrameUnsized>::new::<FakeDriverConfig>(buffer);
    frame.with_size(NonZero::new(size).unwrap())
}

pub fn create_unsized_radio_frame(allocator: &MacBufferAllocator) -> RadioFrame<RadioFrameUnsized> {
    let buffer = allocator
        .try_allocate_buffer(TEST_BUFFER_SIZE)
        .expect("allocation failed");
    RadioFrame::<RadioFrameUnsized>::new::<FakeDriverConfig>(buffer)
}

// ============================================================================
// Fuzz harness: acknowledgment validation (target T3)
// ============================================================================

#[cfg(feature = "_fuzz-harness")]
pub(crate) fn fuzz_validate_ack(data: &[u8]) {
    if data.len() < 10 {
        return;
    }

    let mut own_address = [0u8; 8];
    own_address.copy_from_slice(&data[0..8]);
    let expected_seq_nb = data[8];
    let frame_bytes = &data[9..];

    let headroom = <FakeDriverConfig as DriverConfig>::HEADROOM as usize;
    let max_len = <FakeDriverConfig as DriverConfig>::MAX_SDU_LENGTH as usize
        - core::mem::size_of::<FcsTwoBytes>();
    let n = frame_bytes.len().min(max_len);

    let allocator = create_test_allocator();
    let mut buffer = allocator
        .try_allocate_buffer(TEST_BUFFER_SIZE)
        .expect("allocation failed");
    buffer[headroom..headroom + n].copy_from_slice(&frame_bytes[..n]);

    let rx_ack_frame = RadioFrame::<RadioFrameUnsized>::new::<FakeDriverConfig>(buffer)
        .with_size(NonZero::new(n as u16).unwrap());

    let recovered =
        match crate::driver::validate_ack_frame::<FakeDriverConfig>(
            rx_ack_frame,
            expected_seq_nb,
            &own_address,
        ) {
            Ok((frame, _time_sync)) => frame,
            Err(frame) => frame,
        };

    // Return the buffer to the pool.
    // Safety: the buffer was allocated from this allocator.
    unsafe {
        allocator.deallocate_buffer(recovered.into_buffer());
    }
}

#[cfg(test)]
mod reception_address_tests {
    use crate::scheduler::{MacCommandType, MessageType, ReceptionType, SchedulerRequest};
    use dot15d4_util::sync::HasAddress;

    #[test]
    fn mac_command_requests_match_their_own_command_type() {
        let request = SchedulerRequest::Reception(ReceptionType::MacCommand(
            MacCommandType::AssociateRequest,
        ));
        assert!(request.matches(&MessageType::Rx(ReceptionType::MacCommand(
            MacCommandType::AssociateRequest
        ))));
        assert!(!request.matches(&MessageType::Rx(ReceptionType::MacCommand(
            MacCommandType::AssociateResponse
        ))));
    }
}
