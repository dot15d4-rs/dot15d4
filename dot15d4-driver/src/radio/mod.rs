#![cfg_attr(not(feature = "nrf"), allow(unused))]

use core::{fmt::Debug, marker::PhantomData, mem::MaybeUninit};

use crate::{
    radio::tasks::{AnyTask, RadioDriverApi, RadioTask},
    timer::{HighPrecisionTimer, NsInstant, OptionalNsInstant, RadioTimerApi, RadioTimerError},
};

use self::{
    phy::PhyConfig,
    tasks::{
        RadioState, RadioTransitionResult, ReceivingRxState, RxResult, StopListeningResult,
        TaskOff, TaskRx,
    },
};

pub mod config;
pub mod constants;
pub mod frame;
pub mod phy;
pub mod tasks;

// TODO: Move this to an external per-driver config.
pub const MAX_DRIVER_OVERHEAD: usize = 2;

/// Type allowed for [`DriverConfig::Fcs`]
/// Drivers for LECIM, TVWS and SUN PHYs may be configured with a 4-byte FCS, all
pub type FcsFourBytes = u32;

/// Type allowed for [`DriverConfig::Fcs`]
/// Most drivers/PHYs use two bytes.
pub type FcsTwoBytes = u16;

/// Type allowed for [`DriverConfig::Fcs`]
/// Drivers that offload FCS (=CRC) checking to hardware will neither require
/// nor include an FCS in the frame.
pub type FcsNone = ();

// TODO: Convert into a runtime construct so that we can address multiple
//       radios and get rid of the generic. This can be done with minimal
//       overhead as higher-layer representations need to save headroom,
//       tailroom and FCS ranges anyway.
pub trait DriverConfig {
    type Phy: PhyConfig;

    /// Any buffer headroom required by the driver.
    const HEADROOM: u8;

    /// Any buffer tailroom required by the driver. If the driver takes care of
    /// FCS handling (see [`FcsNone`]), then the tailroom may have to include
    /// the required bytes to let the hardware add the FCS.
    const TAILROOM: u8;

    /// aMaxPhyPacketSize if the FCS is handled by the MAC, otherwise
    /// aMaxPhyPacketSize minus the FCS size.
    const MAX_SDU_LENGTH: u16;

    /// FCS handling:
    ///  - [`FcsTwoBytes`]: No FCS handling inside the driver or hardware. The
    ///    driver expects the framework to calculate and inject a 2-byte FCS
    ///    into the frame.
    ///  - [`FcsFourBytes`]: No FCS handling inside the driver or hardware. The
    ///    driver expects the framework to calculate and inject a 4-byte FCS
    ///    into the frame.
    ///  - [`FcsNone`]: FCS handling is offloaded to the driver or hardware. The
    ///    driver expects the framework to end the MPDU after the frame payload
    ///    without any FCS. If the driver or hardware requires buffer space for
    ///    its own FCS handling, then it must be included in the tailroom.
    type Fcs: Copy + Debug;

    /// The radio timer implementation.
    type Timer: RadioTimerApi;
}

pub type HighPrecisionTimerOf<RadioDriverImpl> =
    <<RadioDriverImpl as DriverConfig>::Timer as RadioTimerApi>::HighPrecisionTimer;

pub type PhyOf<RadioDriverImpl> = <RadioDriverImpl as DriverConfig>::Phy;

/// Generic IEEE 802.15.4 radio driver state machine.
///
/// This structure represents a typestate based radio driver state machine
/// implementation.
///
/// The implementation is contingent on the `RadioDriverImpl` parameter. The
/// current state machine state is encoded by the `Task` parameter.
///
/// The radio driver state machine is modeled after UML behavior state machine
/// concepts (see UML 2.5.1, section 14.2):
/// - It is a single-region, non-hierarchical state machine (section 14.2.3)
///   with a fixed set of "simple" states (section 14.2.3.4.1) as well as
///   well-defined "external" and "internal" transitions (section 14.2.3.8.1)
///   that have to be implemented by all radio driver implementations.
/// - As each state corresponds to a well-defined abstract radio task, we use
///   the radio task name to designate the state. This doesn't mean that state
///   and task may be equated, see below.
/// - Each state MAY define entry and exit behavior as well as a "do activity",
///   i.e. the actual radio task (section 14.2.3.4.3). The "do activity"
///   finishes with a "completion event" (section 14.2.3.8.3).
/// - The transition from the current radio task to the next is scheduled by a
///   radio task scheduler. The scheduler calls one of the typestate-specific
///   methods on the radio driver state machine. Conceptually this is an event
///   occurrence (section 13.3.3.1) that will be stored ("pooled") by the state
///   machine until it reaches a well-defined ("stable") state configuration at
///   which point the transition to the next state ("state machine step") will
///   be triggered (section 14.2.3.9.1). We use async functions and futures to
///   await stable state configurations and transition completion (section
///   14.2.3.8).
/// - To fully benefit from the performance-oriented, precision-timing design of
///   the driver state machine, schedulers SHOULD typically schedule the next
///   task while the current tasks "do activity" is still ongoing. This allows
///   driver implementations to pre-program transitions in hardware so that they
///   can be executed without CPU interaction and deterministic timing as soon
///   as the current task finishes. In this case the lifetime of the task
///   corresponds exactly to the lifetime of the state.
/// - Nevertheless state machine implementations SHALL be able to deal with late
///   scheduling without introducing data races or other undefined behavior. In
///   this case the state outlives the task.
/// - From a state machine's perspective, transitions between radio states are
///   atomic "steps" in the sense that a transition triggered by some event will
///   be run-to-completion (section 14.2.3.9.1) before a new event can be
///   dispatched. From a wall-clock's perspective the execution time of
///   transitions MAY nevertheless be non-zero (section 14.2.3.8). In real-world
///   radio driver implementations this will typically be the case. We implement
///   this by alternating between distinct objects representing the state
///   machine "in state" and "in transit" (section 14.2.3.1) one consuming the
///   other so that they can never exist concurrently.
/// - Transitions between radio peripheral states may have attached
///   transition-specific "effect" behavior (section 14.2.3.8). This allows
///   driver implementations to execute transition-specific code on top of
///   state-specific code. This is regularly required when pre-programming
///   deterministically timed transitions and is the _raison d'être_ of the
///   typestate based radio driver design in the first place.
/// - We extend the UML transition execution model to allow for sophisticated,
///   deterministically-timed execution of transition-related behavior. Drivers
///   MAY define transition-specific behavior in callbacks defined within
///   transition implementations:
///   1. "on_scheduled" behavior: Immediately executes when a transition is
///      scheduled. Not defined in the UML standard but required in practice to
///      pre-program the transition effect or to trigger the subsequent state's
///      entry behavior or do activity.
///   2. "on_completed": Executes when the transition is actually triggered
///      (either on "do activity" completion or immediately when the "do
///      activity" already finished). Albeit similar, this does NOT corresponds
///      to UML's notion of a transition effect as it is executed _before_ any
///      state-specific exit behavior.
///   3. "cleanup": Executed after the target state entered or if the transition
///      needs to be rolled back. Not defined in the UML standard but required
///      in practice to clean up any left-overs from prior transition behavior.
///
///   Note that none of these behaviors can be considered equivalent to UML
///   transition effects, they are non-standard extensions specific to our
///   execution model.
/// - All behaviors defined for states and transitions may fail in practice.
///   While the UML standard defines exceptions (section 13.2.3.1) it mentions
///   exceptions during transition execution only briefly (section 14.2.3.9.1)
///   and doesn't explicitly define exception handling. As exceptions may
///   regularly occur during transitions, we implicitly define a "choice"
///   pseudostate (section 14.2.3.5) after each behavior that is executed during
///   a transition.
/// - If one of the transition behaviors signals an error _before_ the target
///   state has entered, the transition will be "rolled back", i.e.
///   conceptually each external transition implies several compound
///   self-transitions with a zero net effect routed through the "failure"
///   branches of the corresponding choice pseudostates placed after each
///   transition behavior. Implementations will have to ensure that all prior
///   effects of the transition will be neutralized before returning an error
///   from a transition-related behavior. See
///   [`tasks::CompletedRadioTransition::Rollback`].
/// - A rollback is typically not possible if one of the transition behaviors
///   signals an error _after_ the source state has been left (i.e. the
///   state-specific transition() method has been called). Such exceptions
///   SHALL NOT leave the driver in an undefined state. Implementations SHALL
///   fall back to the off state if the target state cannot be reached, see
///   [`tasks::CompletedRadioTransition::Fallback`].
/// - We further extend the UML state machine model by defining a "do activity
///   result", i.e. the radio task MAY produce a result (e.g. a transmission
///   result code or a received radio frame). While the result will typically
///   be available after scheduling a transition and before the next state
///   enters, the framework will NOT wake the CPU immediately when the result
///   becomes available but only after the next state entered:
///   - simplified execution model: The radio scheduler only needs to take
///     action once per task, i.e. it can deal with the result of the previous
///     task and schedule the next task in a single step.
///   - energy efficiency: The CPU only needs to be woken up once. This saves
///     unnecessary CPU startup and shutdown cost e.g. due to async executor
///     overhead.
///   - deterministic timing: Dealing with the result before scheduling the next
///     radio task may risk deterministic execution timing if overstretching the
///     possibly short scheduling window.
///
///   See [`tasks::CompletedRadioTransition::Entered`].
///
/// SAFETY: Radio drivers are not synchronized. All its methods SHALL be called
///         from a single scheduler.
pub struct RadioDriver<RadioDriverImpl, Task> {
    config: PhantomData<RadioDriverImpl>,
    task: PhantomData<Task>,
}

impl<RadioDriverImpl: DriverConfig, Task> RadioDriver<RadioDriverImpl, Task> {
    /// Constructor used to instantiate a new typestate.
    ///
    /// SHALL only be used within framework-internal methods that provably
    /// switch state. External clients of the state machine SHALL rely on
    /// [`tasks::RadioTransition``]'s public API to step the radio state
    /// machine.
    pub(crate) fn new_internal() -> Self {
        Self {
            config: PhantomData,
            task: PhantomData,
        }
    }
}

impl<RadioDriverImpl: DriverConfig> RadioDriver<RadioDriverImpl, TaskOff>
where
    Self: RadioState<RadioDriverImpl, TaskOff>,
{
    pub(crate) async fn wait_until_off(mut self) -> (Self, NsInstant) {
        let off_entry = self.transition().await;
        debug_assert!(off_entry.is_ok());
        (self, off_entry.unwrap())
    }
}

impl<RadioDriverImpl: DriverConfig> RadioDriver<RadioDriverImpl, TaskRx>
where
    Self: RadioState<RadioDriverImpl, TaskRx>,
    RadioDriver<RadioDriverImpl, TaskOff>: RadioState<RadioDriverImpl, TaskOff>,
{
    pub(crate) async fn rx_window_ended<RxState: ReceivingRxState<RadioDriverImpl>>(
        mut self,
        disabled_at: OptionalNsInstant,
    ) -> StopListeningResult<RadioDriverImpl, RxState> {
        let rx_result = RxResult::RxWindowEnded(self.take_task().radio_frame);
        let (off_state, measured_entry) = RadioDriver::new_internal().wait_until_off().await;
        StopListeningResult::RxWindowEnded(RadioTransitionResult::new(
            rx_result,
            off_state,
            measured_entry,
            disabled_at,
        ))
    }
}

/// Implementation-independent radio driver state.
///
/// Radio driver state is kept separately from the typestate modelling access to
/// it. This is an optimization to reduce size of asynchronous tasks handling
/// radio state.
pub struct RadioDriverState<RadioDriverImpl: DriverConfig> {
    /// Possibly uninitialized field may only be accessed once this field is
    /// true.
    is_initialized: bool,

    /// Any private state used by a specific radio driver implementation.
    inner: MaybeUninit<RadioDriverImpl>,

    // An instance of the radio sleep timer.
    sleep_timer: MaybeUninit<RadioDriverImpl::Timer>,

    /// An instance of the high precision timer while it is running.
    high_precision_timer: Option<HighPrecisionTimerOf<RadioDriverImpl>>,

    /// The scheduled entry time. Kept for debugging and error handling
    /// purposes. Shall be set while the task is being scheduled. See
    /// [`tasks::RadioState::transition`] for a definition of the semantics of
    /// this timestamp.
    pub(crate) scheduled_entry: OptionalNsInstant,

    /// The time the task entered. This timestamp SHALL be measured and set as
    /// soon as the task entered. See [`tasks::RadioState::transition`] for a
    /// definition of the semantics of this timestamp.
    pub(crate) measured_entry: OptionalNsInstant,

    /// The RMARKER of an incoming frame.
    ///
    /// This value is observed at the end of the listening rx state and
    /// available throughout the receiving rx state. Not available in any other
    /// radio state.
    rx_frame_started: OptionalNsInstant,

    /// The currently active task which may be consumed by the driver at any
    /// time during task execution.
    ///
    /// Note: Storing the task here rather than in [`RadioDriver`] is an
    ///       optimization that makes the typestate a ZST for zero-cost use in
    ///       async functions and blocks.
    task: Option<AnyTask>,
}

impl<RadioDriverImpl: DriverConfig> RadioDriverState<RadioDriverImpl> {
    pub(crate) const fn new() -> Self {
        Self {
            is_initialized: false,
            inner: MaybeUninit::uninit(),
            sleep_timer: MaybeUninit::uninit(),
            high_precision_timer: None,
            scheduled_entry: OptionalNsInstant::none(),
            measured_entry: OptionalNsInstant::none(),
            rx_frame_started: OptionalNsInstant::none(),
            task: Some(AnyTask::Off(TaskOff)),
        }
    }

    pub(crate) fn init(&mut self, inner: RadioDriverImpl, sleep_timer: RadioDriverImpl::Timer) {
        assert!(!self.is_initialized);
        self.is_initialized = true;
        self.inner.write(inner);
        self.sleep_timer.write(sleep_timer);
    }

    pub(crate) fn enter_next_task<Task: RadioTask>(&mut self, next_task: Task) {
        self.measured_entry = None.into();
        self.rx_frame_started = None.into();
        self.task = Some(next_task.into());
    }

    pub(crate) fn inner(&mut self) -> &mut RadioDriverImpl {
        debug_assert!(self.is_initialized);
        // Safety: The driver is initialized.
        unsafe { self.inner.assume_init_mut() }
    }

    pub(crate) fn sleep_timer(&self) -> RadioDriverImpl::Timer {
        debug_assert!(self.is_initialized);
        // Safety: The driver is initialized.
        unsafe { self.sleep_timer.assume_init() }
    }

    pub(crate) fn start_timer(
        &mut self,
        start_at: OptionalNsInstant,
    ) -> Result<&HighPrecisionTimerOf<RadioDriverImpl>, RadioTimerError> {
        debug_assert!(self.high_precision_timer.is_none());

        self.high_precision_timer = Some(self.sleep_timer().start_high_precision_timer(start_at)?);
        Ok(self.timer())
    }

    pub(crate) fn timer(&self) -> &HighPrecisionTimerOf<RadioDriverImpl> {
        self.high_precision_timer.as_ref().unwrap()
    }

    pub(crate) fn stop_timer(&mut self) {
        drop(self.high_precision_timer.take());
    }

    pub(crate) fn reset_timer(&self) {
        self.high_precision_timer.as_ref().unwrap().reset();
    }

    pub(crate) fn set_rx_frame_started(&mut self, rx_frame_started: NsInstant) {
        debug_assert!(self.rx_frame_started.is_none());

        self.rx_frame_started = rx_frame_started.into();
    }

    pub(crate) fn rx_frame_started(&self) -> OptionalNsInstant {
        self.rx_frame_started
    }

    pub(crate) fn take_task<Task: RadioTask>(&mut self) -> Task {
        self.task.take().unwrap().try_into_task().unwrap()
    }

    pub(crate) fn ref_task<Task: RadioTask>(&self) -> &Task {
        self.task.as_ref().unwrap().try_into_task_ref().unwrap()
    }
}

#[cfg(feature = "rtos-trace")]
pub(crate) mod trace {
    use dot15d4_util::trace::TraceOffset;

    const OFFSET: TraceOffset = TraceOffset::Dot15d4DriverRadio;

    // Tasks
    pub const TASK_OFF_SCHEDULE: u32 = OFFSET.wrap(0);
    pub const TASK_OFF_ENTRY: u32 = OFFSET.wrap(1);
    pub const TASK_OFF_COMPLETE: u32 = OFFSET.wrap(2);

    pub const TASK_RX_SCHEDULE: u32 = OFFSET.wrap(3);
    pub const TASK_RX_ENTRY: u32 = OFFSET.wrap(4);
    pub const TASK_RX_COMPLETE: u32 = OFFSET.wrap(5);

    pub const TASK_TX_SCHEDULE: u32 = OFFSET.wrap(6);
    pub const TASK_TX_ENTRY: u32 = OFFSET.wrap(7);
    pub const TASK_TX_COMPLETE: u32 = OFFSET.wrap(8);

    pub const TASK_FALL_BACK: u32 = OFFSET.wrap(9);

    // Markers
    pub const MARKER_RX_FRAME_STARTED: u32 = OFFSET.wrap(0);
    pub const MARKER_RX_FRAME_INFO: u32 = OFFSET.wrap(1);
    pub const MARKER_RX_WINDOW_ENDED: u32 = OFFSET.wrap(2);

    /// Instruments the driver for task tracing.
    pub fn instrument() {
        rtos_trace::trace::task_new_stackless(TASK_OFF_SCHEDULE, "Schedule Off\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_OFF_ENTRY, "Off\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_OFF_COMPLETE, "Off->*\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_RX_SCHEDULE, "Schedule Rx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_RX_ENTRY, "Rx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_RX_COMPLETE, "Rx->*\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TX_SCHEDULE, "Schedule Tx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TX_ENTRY, "Tx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TX_COMPLETE, "Tx->*\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_FALL_BACK, "Fall back to Off\0", 0);
        rtos_trace::trace::name_marker(MARKER_RX_FRAME_STARTED, "Frame Started\0");
        rtos_trace::trace::name_marker(MARKER_RX_FRAME_INFO, "Preliminary Frame Info\0");
        rtos_trace::trace::name_marker(MARKER_RX_WINDOW_ENDED, "Rx Window Ended\0");
    }
}
