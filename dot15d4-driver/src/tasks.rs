// TODO: This is a generic, vendor-independent API. Move this file to a place
//       where it can be accessed by all HALs and the scheduler.
use core::{convert::Infallible, future::Future, marker::PhantomData};

use crate::{
    config::Channel,
    constants::A_MAX_SIFS_FRAME_SIZE,
    frame::{AddressingFields, FrameControl, RadioFrame, RadioFrameSized, RadioFrameUnsized},
};

/// Tasks can be scheduled as fast as possible ("best effort") or at a
/// well-defined tick of the local radio clock ("scheduled")
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Timestamp {
    /// A task with this timestamp will be executed back-to-back to the previous
    /// task with minimal standard-conforming inter-frame spacing.
    BestEffort,
    /// A task with this timestamp will be executed by the driver at a precisely
    /// defined time. The semantics of the timestamp depends on the task that's
    /// being scheduled:
    /// - TX: Designates the time at which the RMARKER SHALL pass the local
    ///   antenna.
    /// - RX: Designates the time at which the RMARKER is expected to pass the
    ///   local antenna.
    /// - Radio Off: Designates the time at which the radio will start to
    ///   ramp-down.
    Scheduled(u64),
}

/// Generic representation of a radio task.
///
/// Some features of radio tasks are mandatory, others are optional (see the
/// documentation of structs implementing this trait).
///
/// Mandatory features of radio tasks SHALL be implemented by all drivers while
/// optional features SHOULD be implemented if the radio peripheral offers the
/// corresponding functionality ("hardware offloading").
pub trait RadioTask {
    /// Whenever a radio task finishes without error (i.e. the state's "do
    /// activity" successfully runs to completion), it SHALL produce a task
    /// result, e.g. a task status code or structured result.
    ///
    /// A task MAY produce distinct results depending on external contingencies,
    /// e.g. a valid frame arrived, a frame arrived but its CRC or signature
    /// does not match, it cannot be decrypted or doesn't match filtering
    /// criteria or a frame was expected but it didn't arrive.
    ///
    /// The transition to the next scheduled task SHALL only proceed if the "do
    /// activity" produces a task result. If it produces a task error, the
    /// transition SHALL be rolled back, see
    /// [`CompletedRadioTransition::Rollback`].
    ///
    /// If the task produces a result and the transition to the following task
    /// also succeeds, the result will be reported with the
    /// [`CompletedRadioTransition::Entered`] variant. Otherwise the result will
    /// be contained in the [`CompletedRadioTransition::Rollback`] variant.
    ///
    /// This type SHALL be the unit type if the task does not produce any
    /// result.
    ///
    /// Note: The same task outcome (e.g. a CRC error) MAY be interpreted as
    ///       both, a [`RadioTask::Result`] or a [`RadioTask::Error`], depending
    ///       on the context: If an independent Tx frame is scheduled after an
    ///       Rx task ending in a CRC error, then the Tx frame must be scheduled
    ///       nevertheless. The same is not true for an acknowledgment frame
    ///       scheduled after an incoming frame whose CRC check fails. This
    ///       explains why the "CRC not ok" outcome exists as both, a result and
    ///       an error. The scheduling API will allow to pass in flags that
    ///       allow the driver to distinguish between those cases.
    type Result;

    /// A transition MAY fail if it the source state's task or the transition to
    /// the target state produce an error (e.g.  due to a failed precondition
    /// like a busy channel in the TX case or even due to message collision on
    /// the API or I2C bus).
    ///
    /// If the error occurs while still in the source state, the driver SHALL
    /// roll back the transaction (see [`CompletedRadioTransition::Rollback`])
    /// and remain in the source state. If the error occurs after leaving the
    /// source state but before entering the target state, then the scheduler
    /// SHALL place the driver in the off state (see
    /// [`CompletedRadioTransition::Fallback`]).
    ///
    /// This type SHALL be the never type (i.e. "Infallible") if starting the
    /// task cannot fail.
    type Error;
}

/// Task: switch to low energy state
#[derive(Debug, PartialEq, Eq)]
pub struct TaskOff {
    /// Designates the time at which the radio will start to ramp-down.
    pub at: Timestamp,
}
#[derive(Debug, PartialEq, Eq)]
pub enum OffResult {
    Off,
}
impl RadioTask for TaskOff {
    type Result = OffResult;
    type Error = Infallible;
}

/// Task: receive a single frame
///
/// This task is mandatory and SHALL be implemented by all drivers.
///
/// A driver MAY offload acknowledgement to hardware (automatic acknowledgement)
/// or rely on the client for manual acknowledgement.
///
/// # Manual Acknowledgement
///
/// If the RX task receives a non-ACK frame, the driver SHALL store the frame's
/// sequence number (if present) on-the-fly. Actual acknowledgement will be
/// scheduled subsequently by the client via a regular TX task containing the
/// ACK frame using the stored sequence number on-the-fly ("soft MAC").
///
/// This feature is mandatory as AIFS is generally too short to set the sequence
/// number with CPU intervention after a frame was received.
///
/// # Automatic Acknowledgement
///
/// This feature is optional and SHOULD only be implemented by drivers that
/// cover hardware with ACK offloading ("hard MAC").
///
/// If the RX task receives a data, multi-purpose or command frame with the AR
/// flag set and matching all filtering and security criteria (see IEEE
/// 802.15.4-2024, section 6.6.2), then the driver SHALL auto-acknowledge the
/// frame.
#[derive(Debug, PartialEq, Eq)]
pub struct TaskRx {
    /// the time at which the RMARKER is expected to pass the local antenna
    pub start: Timestamp,

    /// radio frame allocated to receive incoming frames
    pub radio_frame: RadioFrame<RadioFrameUnsized>,
}
/// RX task result
#[derive(Debug, PartialEq, Eq)]
pub enum RxResult {
    /// A valid frame was successfully received and acknowledged if requested.
    Frame(
        /// received radio frame
        RadioFrame<RadioFrameSized>,
    ),
    /// A new task was scheduled before a frame was received.
    RxWindowEnded(
        /// recovered radio frame
        RadioFrame<RadioFrameUnsized>,
    ),
    /// A frame was received but the CRC didn't match.
    ///
    /// Note: This result is returned if the driver was programmed to switch to
    ///       the next radio task on CRC error, e.g. when scheduling a regular
    ///       Off, RX or TX task back-to-back to an RX task.
    CrcError(
        /// recovered radio frame
        RadioFrame<RadioFrameUnsized>,
    ),
    /// A frame with correct CRC was received but didn't match the filtering
    /// requirements, see IEEE 802.15.4-2024, section 6.6.2. This can be useful
    /// to implement promiscuous mode.
    FilteredFrame(
        /// received radio frame
        RadioFrame<RadioFrameSized>,
    ),
}
#[derive(Debug, PartialEq, Eq)]
pub enum RxError {
    /// A frame was received but the CRC didn't match.
    ///
    /// Note: This error is returned if the driver was programmed to remain in
    ///       the RX state on CRC error, e.g. to ensure that an ACK frame
    ///       scheduled back-to-back to an RX frame is not being sent when the
    ///       checksum doesn't match.
    CrcError,
}
impl RadioTask for TaskRx {
    type Result = RxResult;
    type Error = RxError;
}

/// Task: send a single frame
///
/// This task is mandatory and SHALL be implemented by all drivers.
///
/// A driver MAY offload acknowledgement to hardware (automatic acknowledgement)
/// or rely on the client for manual acknowledgement.
///
/// # Manual Acknowledgement
///
/// If the TX task represents a non-ACK TX frame then the frame SHALL be sent
/// unchanged. If the frame requires acknowledgment, a regular RX task will be
/// scheduled subsequently by the client awaiting the ACK frame ("soft MAC").
///
/// If the TX task represents an ACK TX frame, then the driver SHALL set the
/// sequence number from the preceding RX frame on-the-fly and respect the AIFS.
///
/// This feature is mandatory as AIFS is generally too short to set the sequence
/// number with CPU intervention during an intermittent off task after a frame
/// was received.
///
/// # Automatic Acknowledgement
///
/// This feature is optional and SHOULD only be implemented by drivers that
/// cover hardware with ACK offloading ("hard MAC").
///
/// A driver implementing this capability SHALL wait for ACK after sending a
/// frame requiring acknowledgement. It is the responsibility of the client to
/// ensure that the AR flag is properly set in the frame header.
#[derive(Debug, PartialEq, Eq)]
pub struct TaskTx {
    /// the time at which the RMARKER of the sent frame SHALL pass the local
    /// antenna
    pub at: Timestamp,

    /// radio frame to be sent
    pub radio_frame: RadioFrame<RadioFrameSized>,

    /// whether CCA is to be performed as a precondition to send out the frame
    pub cca: bool,
}
/// TX task result
#[derive(Debug, PartialEq, Eq)]
pub enum TxResult {
    /// The frame was successfully sent and acknowledged if requested.
    /// Does not yet carry any data but MAY do so in the future.
    Sent(RadioFrame<RadioFrameSized>), // TODO: Support returning an optional Enh-Ack frame.
    /// The frame was sent but the ACK timeout expired or an Enh-ACK frame was
    /// received but its content indicates a NACK (used, e.g. in TSCH to signal
    /// NACK while still transporting time synchronization info).
    Nack(
        /// The radio frame that was not ack'ed.
        RadioFrame<RadioFrameSized>,
    ), // TODO: Support returning an optional Enh-Ack frame.
}
#[derive(Debug, PartialEq, Eq)]
/// TX task error
pub enum TxError {
    /// CCA detected a busy medium.
    CcaBusy(
        /// The radio frame that could not be sent.
        RadioFrame<RadioFrameSized>,
    ),
}
impl RadioTask for TaskTx {
    type Result = TxResult;
    type Error = TxError;
}

/// Currently just a placeholder - may report more specific scheduling errors
/// later on.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SchedulingError;

/// Represents a radio task or scheduling error.
#[derive(Debug, PartialEq, Eq)]
pub enum RadioTaskError<Task: RadioTask> {
    /// Any interaction with the radio may fail and the scheduler will have to
    /// deal with this.
    Scheduling(SchedulingError),

    /// The radio task itself failed.
    Task(Task::Error),
}

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
///   2. "on_task_complete": Executes when the transition is actually triggered
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
///   [`CompletedRadioTransition::Rollback`].
/// - A rollback is typically not possible if one of the transition behaviors
///   signals an error _after_ the source state has been left (i.e. the
///   state-specific transition() method has been called). Such exceptions
///   SHALL NOT leave the driver in an undefined state. Implementations SHALL
///   fall back to the off state if the target state cannot be reached, see
///   [`CompletedRadioTransition::Fallback`].
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
///   See [`CompletedRadioTransition::Entered`].
///
/// SAFETY: Radio drivers are not synchronized. All its methods SHALL be called
///         from a single scheduler.
pub struct RadioDriver<RadioDriverImpl, Task> {
    /// Any private state used by a specific radio driver implementation.
    pub(super) inner: RadioDriverImpl,
    /// The currently active task which may be consumed by the driver at any
    /// time during task execution.
    #[allow(dead_code)]
    pub(super) task: Option<Task>,
}

/// Generic IEEE 802.15.4 radio driver state machine state.
///
/// This trait must be implemented by all radio states. It defines the template
/// for generic entry and exit behavior as well as behavior triggered by the
/// "completion event" of the state.
pub trait RadioState<Task: RadioTask> {
    /// Waits until the state's state invariants have been established (i.e. the
    /// peripheral fully reached the target state) and the state specific task
    /// ("do activity") started.
    ///
    /// MAY additionally implement the state specific entry behavior of the
    /// state (UML 2.5.1, section 14.2.3.4.3).
    ///
    /// Any transition-specific `cleanup` behavior will be executed right after
    /// this method returns.
    ///
    /// This means that the method strictly returns an undefined time _after_
    /// the state entered from a conceptual viewpoint (section 14.2.3.4.5).
    /// Returning from this method signals to the radio task scheduler that the
    /// state machine is ready to receive the next scheduling event, i.e. the
    /// next task can be scheduled/pre-programmed.
    ///
    /// In practice this method SHOULD return such that the radio task
    /// scheduler has sufficient time to schedule the next task before the
    /// current task ends. This is required to guarantee deterministic,
    /// CPU-independent timing of radio tasks.
    ///
    /// Returns `Ok` if the state could was successfully entered, `Err`
    /// otherwise.
    ///
    /// SHALL NOT be executed on self-transitions.
    ///
    /// Note: Implementations SHOULD ensure that this method is being called
    ///       before the driver actually switched state internally.
    ///       Implementations SHALL ensure that nevertheless the method
    ///       terminates right away if the driver has already switched state
    ///       internally.
    fn transition(&mut self) -> impl Future<Output = Result<(), RadioTaskError<Task>>>;

    /// Waits until the current state's task ("do activity") is complete.
    ///
    /// Any transition-specific "on_task_complete" behavior will be executed
    /// right after this method returns.
    ///
    /// Produces the task result or fails with a scheduling error.
    ///
    /// If the `alt_outcome_is_error` flag is true, then the alternate outcome
    /// (e.g. CRC not ok) should be treated as a task error rather than a task
    /// result thereby triggering a transition rollback.
    ///
    /// Note: Implementations SHALL NOT assume anything about the status of the
    ///       state's task - it MAY be running or already complete when this
    ///       method is being called.
    fn run(
        &mut self,
        alt_outcome_is_error: bool,
    ) -> impl Future<Output = Result<Task::Result, RadioTaskError<Task>>>;

    /// Ensures leftovers from task execution have been cleaned up before the
    /// state is left.
    ///
    /// May fail with a scheduling error.
    ///
    /// SHALL NOT be executed on self-transitions.
    ///
    /// Note: Implementations SHALL ensure that this method is being called
    ///       _after_ the state's task completed, i.e. after awaiting `run()``.
    fn exit(&mut self) -> Result<(), SchedulingError>;
}

/// Generic characterization of the "Radio Off" state. Drivers MAY either
/// implement these methods individually to take advantage of hardware
/// optimizations or they MAY delegate to a common implementation shared between
/// states for simpler implementation and maintenance.
///
/// This allows driver maintainers to provide an initial "minimal"
/// implementation and optimize for performance and energy efficiency later on
/// while still guaranteeing that a single scheduler can drive all kinds of
/// radio hardware.
///
/// This is true similarly for all other state characterizations.
pub trait OffState<RadioDriverImpl>: RadioState<TaskOff> {
    /// Set the default radio channel.
    ///
    /// This channel will be used for Rx and Tx if no task-specific channel was
    /// set.
    fn set_channel(&mut self, channel: Channel);

    /// Schedules a transition to the RX state.
    fn schedule_rx(
        self,
        rx_task: TaskRx,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskOff, TaskRx>;

    /// Schedules a transition to the TX state.
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else remains in the Radio Off state.
    fn schedule_tx(
        self,
        tx_task: TaskTx,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskOff, TaskTx>;

    /// Switches the radio off immediately and unconditionally.
    ///
    /// This method will be called whenever a non-recoverable error is
    /// encountered. The method must place the driver into the well-defined off
    /// state under all conditions. If this is not possible, it SHALL panic.
    ///
    /// Note: May panic.
    fn switch_off(inner: RadioDriverImpl) -> impl Future<Output = Self>;
}

pub struct PreliminaryFrameInfo<'frame> {
    pub mpdu_length: u16,
    pub frame_control: Option<FrameControl<[u8; 2]>>,
    pub seq_nr: Option<u8>,
    pub addressing_fields: Option<AddressingFields<&'frame [u8]>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ifs {
    Aifs,
    Sifs,
    Lifs,
    None,
}

impl Ifs {
    pub fn from_mpdu_length(mpdu_length: u16) -> Ifs {
        if mpdu_length <= A_MAX_SIFS_FRAME_SIZE {
            Ifs::Sifs
        } else {
            Ifs::Lifs
        }
    }
}

/// Generic characterization of the "Receiver ON" (RX) state.
///
/// Drivers will occupy this state when waiting for frames or while receiving a
/// frame.
///
/// Transition away from this state depend on the outcome of the RX task in
/// combination with the `rollback_on_crcerror` flag:
/// - In case of a CRC error, the transition will be aborted if the
///   `rollback_on_crcerror` flag is true. This is useful in the case of an
///   acknowledgment scheduled back-to-back to the corresponding Rx frame.
/// - If the flag is false then the transition takes place independently of
///   a CRC match. This is the correct behavior in case of an independent Tx
///   frame being scheduled back-to-back to an incoming frame that does not
///   request acknowledgment.
pub trait RxState<RadioDriverImpl>: RadioState<TaskRx> {
    /// Wait until a frame is being received. This function SHOULD return as
    /// quickly as possible once a synchronization header is recognized by the
    /// receiver. This is required for frame validation and RX back-to-back
    /// scheduling, see below.
    ///
    /// The returned future SHALL be cancelable.
    ///
    /// Note: It is not guaranteed that a frame will be returned when the RX
    ///       state completes. A CRC, signature or decryption error may occur or
    ///       the frame might be filtered by the driver if the driver implements
    ///       destination address filter offloading.
    fn frame_started(&mut self) -> impl Future<Output = ()>;

    /// Wait until the destination pan id and address of an incoming frame has
    /// been received or the frame ends prematurely. This is required for frame
    /// validation.
    ///
    /// Note: It is not guaranteed that a frame will be returned when the RX
    ///       state completes. A CRC, signature or decryption error may occur or
    ///       the frame might be filtered by the driver if the driver implements
    ///       destination address filter offloading.
    fn preliminary_frame_info(&mut self) -> impl Future<Output = PreliminaryFrameInfo<'_>>;

    /// Schedules reception of a frame back-to-back to the frame currently
    /// being received (if any).
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    ///
    /// Note: When scheduling RX frame back-to-back, then only "best effort"
    ///       scheduling SHALL be allowed and the current RX window SHALL NOT be
    ///       ended. This is to avoid that schedulers enter an endless
    ///       Rx-Rx-loop. Schedulers MAY await the `frame_started()` future to
    ///       ensure that another RX frame will only be scheduled when the
    ///       current task is guaranteed to do some work and completes soon.
    ///
    /// Note: The RX task undergoes several sub-states. We have to deal with the
    ///       following cases:
    ///       1. A frame has already been fully received before calling this
    ///          method. Implementations SHALL complete the previous task with
    ///          that frame and set up and start reception of the next one
    ///          immediately.
    ///       2. The last bit of a frame is received just as we set up the new
    ///          task. As in the first case, implementations SHALL return that
    ///          frame and re-start reception immediately while guarding
    ///          against possible race conditions when setting up the new task.
    ///       3. The receiver is still waiting to receive a frame or a frame
    ///          is currently being received but its last bit is only received
    ///          after we set up the new task. Implementation SHALL complete the
    ///          task with the previous frame while preparing everything such
    ///          that the receiver will be ready to receive the next frame
    ///          immediately after receiving the previous one.
    ///
    /// For best performance and energy efficiency, a scheduler SHOULD always
    /// schedule the next RX task early enough such that condition 3 holds. But
    /// for stability and correctness we SHALL deal with the other two
    /// (exceptional) cases, too. We'll emit a warning, though.
    fn schedule_rx(
        self,
        rx_task: TaskRx,
        rollback_on_crcerror: bool,
    ) -> impl SelfRadioTransition<RadioDriverImpl, TaskRx, TaskRx>;

    /// Schedules a transition to the TX state.
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else switches to the Radio Off state.
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        ifs: Ifs,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskRx, TaskTx>;

    /// Schedules a transition to the Radio Off state independently.
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    fn schedule_off(
        self,
        off_task: TaskOff,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskRx, TaskOff>;
}

/// Generic characterization of the "Transmitter ON" (TX) state.
///
/// Drivers will occupy this state while sending a frame or after sending when
/// the transmitter is idle but the radio is still powered (TX idle).
pub trait TxState<RadioDriverImpl>: RadioState<TaskTx> {
    /// Schedules a transition to the RX state.
    fn schedule_rx(
        self,
        task: TaskRx,
        ifs: Ifs,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskTx, TaskRx>;

    /// Schedules transmission of a frame back-to-back to the frame currently
    /// being sent (if any).
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else switches to the Radio Off state.
    ///
    /// Note: The TX state undergoes several sub-states. We have to deal with
    ///       the following cases:
    ///       1. A frame has already been fully sent before calling this
    ///          method. Implementations SHALL complete the previous task with
    ///          that frame and set up and start transmission of the next one
    ///          immediately.
    ///       2. The last bit of a frame is sent just as we set up the new
    ///          task. As in the first case, implementations SHALL return that
    ///          frame and start transmission immediately while guarding
    ///          against possible race conditions when setting up the new task.
    ///       3. The radio is still sending a frame and its last bit will only
    ///          be sent after we set up the new task. Implementation SHALL
    ///          complete the task with the previous frame while preparing
    ///          everything such that the transceiver will be ready to send the
    ///          next frame immediately after sending the previous one.
    ///
    /// For best performance and energy efficiency, a scheduler SHOULD always
    /// schedule the next TX task early enough such that condition 3 holds. But
    /// for stability and correctness we SHALL deal with the other two
    /// (exceptional) cases, too. We'll emit a warning, though.
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        ifs: Ifs,
    ) -> impl SelfRadioTransition<RadioDriverImpl, TaskTx, TaskTx>;

    /// Schedules a transitions to the Radio Off state.
    fn schedule_off(
        self,
        off_task: TaskOff,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskTx, TaskOff>;
}

/// Represents an active radio state transition while it is being traversed.
pub struct RadioTransition<
    RadioDriverImpl,
    ThisTask: RadioTask,
    NextTask: RadioTask,
    OnScheduled: Fn() -> Result<(), SchedulingError>,
    OnTaskComplete: Fn() -> Result<(), SchedulingError>,
    Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
> {
    /// The source radio peripheral state of the transition.
    from_radio: RadioDriver<RadioDriverImpl, ThisTask>,

    /// The target radio peripheral state of the transition.
    to_radio: PhantomData<RadioDriver<RadioDriverImpl, NextTask>>,

    /// Configuration and parameters of the target radio peripheral state.
    next_task: NextTask,

    /// Callback executed as soon as the transition is being scheduled.
    ///
    /// Prepares or starts the transition to the next radio peripheral state:
    /// - If the current state implements a "do activity" and this activity is
    ///   still ongoing (the default case), then this callback SHOULD
    ///   pre-program the hardware such that the transition to the next radio
    ///   peripheral state will be triggered without CPU interaction as soon as
    ///   the "do activity" of the source task finished successfully and
    ///   produced a result.
    /// - If the current state does not implement a "do activity" (e.g. the
    ///   off state) or if the "do activity" already completed, this callback
    ///   SHALL immediately start transitioning to the next radio peripheral
    ///   state.
    on_scheduled: OnScheduled,

    /// Callback executed as soon as the radio task completes.
    ///
    /// MAY start the transition to the next radio state if (and only if)
    /// deterministic CPU-less scheduling from the "on_scheduled" callback
    /// cannot be supported by the radio peripheral.
    ///
    /// MAY otherwise do transition-specific clean up after task completion or
    /// deal with transition-specific error handling depending on the task
    /// result.
    on_task_complete: OnTaskComplete,

    /// Callback to clean up any transition-specific setup or left-overs.
    ///
    /// If the transition succeeds or falls back due to an error in the target
    /// task's `transition()` method: executed as soon as the `transition()`
    /// method returned (i.e. the radio task entered the target state).
    ///
    /// If the transition is rolled back due to an error in the source task's
    /// `run()` or `exit()` methods: executed immediately after the `run()` or
    /// `exit()` method returns with an error.
    ///
    /// Note: This callback will _not_ be called when the transition's own
    ///       `on_scheduled` or `on_task_complete` callbacks fail. In that case
    ///       it is assumed that those callbacks will clean up after themselves.
    cleanup: Cleanup,

    /// Tasks MAY produce distinct outcomes depending on external contingencies
    /// that are known only after the task has already been scheduled. Currently
    /// this is true for "CRC ok" (main outcome) vs. "CRC not ok" (alternate
    /// outcome). This flag determines wether the alternate outcome is treated
    /// as a [`RadioTask::Result`] or as a [`RadioTask::Error`].
    ///
    /// Note: Currently only the RX task's "CRC not ok" outcome uses this flag.
    alt_outcome_is_error: bool,
}

impl<
        RadioDriverImpl,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError>,
        OnTaskComplete: Fn() -> Result<(), SchedulingError>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnTaskComplete, Cleanup>
{
    /// Instantiates a new radio transition.
    pub fn new(
        from_radio: RadioDriver<RadioDriverImpl, ThisTask>,
        next_task: NextTask,
        on_scheduled: OnScheduled,
        on_task_complete: OnTaskComplete,
        cleanup: Cleanup,
        alt_outcome_is_error: bool,
    ) -> Self {
        Self {
            from_radio,
            to_radio: PhantomData,
            next_task,
            on_scheduled,
            on_task_complete,
            cleanup,
            alt_outcome_is_error,
        }
    }
}

/// Represents an active external radio state transition while it is being
/// traversed.
///
/// External transitions have distinct source and target states.
pub trait ExternalRadioTransition<RadioDriverImpl, ThisTask: RadioTask, NextTask: RadioTask> {
    /// Executes the external radio transition. Returns the target state with a
    /// new task instance once the transition completed.
    ///
    /// Switching to the new state SHALL include completing the previous task,
    /// executing the transition behavior as well as `exit()` and `transition()`
    /// in the following order:
    /// 1. transition: on_scheduled() - non-blocking
    /// 2. source radio state: run() - blocking
    /// 3. transition: on_task_complete() - non-blocking
    /// 4. source radio state: exit() - non-blocking
    /// 5. target radio state: transition() - blocking
    /// 6. transition: cleanup() - non-blocking
    ///
    /// Note that the task result is known once run() finishes but will only be
    /// returned to the radio task scheduler once this method finishes, see the
    /// [`RadioDriver`] documentation for more details.
    ///
    /// The naming of this method was chosen to be readable and intuitive in
    /// client code: Once the async method completes, the new state was entered.
    fn execute_transition(
        self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask>>;
}

impl<
        RadioDriverImpl,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError>,
        OnTaskComplete: Fn() -> Result<(), SchedulingError>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > ExternalRadioTransition<RadioDriverImpl, ThisTask, NextTask>
    for RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnTaskComplete, Cleanup>
where
    RadioDriver<RadioDriverImpl, ThisTask>: RadioState<ThisTask>,
    RadioDriver<RadioDriverImpl, NextTask>: RadioState<NextTask>,
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
{
    async fn execute_transition(
        mut self,
    ) -> CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask> {
        if let Err(scheduling_error) = (self.on_scheduled)() {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error),
                None,
                self.next_task,
            );
        }

        let prev_task_result = match self.from_radio.run(self.alt_outcome_is_error).await {
            Err(task_error) => {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end();

                let _ = (self.cleanup)();
                return CompletedRadioTransition::Rollback(
                    self.from_radio,
                    task_error,
                    None,
                    self.next_task,
                );
            }
            Ok(prev_task_result) => prev_task_result,
        };

        if let Err(scheduling_error) = (self.on_task_complete)() {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error),
                Some(prev_task_result),
                self.next_task,
            );
        }

        if let Err(scheduling_error) = self.from_radio.exit() {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            let _ = (self.cleanup)();
            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error),
                Some(prev_task_result),
                self.next_task,
            );
        }

        let mut next_state = RadioDriver {
            inner: self.from_radio.inner,
            task: Some(self.next_task),
        };
        let next_state_entry = next_state.transition().await;

        let fallback = |next_task_error, prev_task_result, inner| async {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            CompletedRadioTransition::Fallback(
                RadioTransitionResult {
                    prev_task_result,
                    prev_state: PhantomData,
                    this_state: RadioDriver::<RadioDriverImpl, TaskOff>::switch_off(inner).await,
                },
                next_task_error,
            )
        };

        if let Err(next_task_error) = (self.cleanup)() {
            return fallback(next_task_error, prev_task_result, next_state.inner).await;
        }

        match next_state_entry {
            Ok(_) => CompletedRadioTransition::Entered(RadioTransitionResult {
                prev_task_result,
                prev_state: PhantomData,
                this_state: next_state,
            }),
            Err(next_task_error) => {
                fallback(next_task_error, prev_task_result, next_state.inner).await
            }
        }
    }
}

/// Represents an active radio state self-transition while it is being
/// traversed.
///
/// Self transitions have the same source and target states. They are also
/// called internal transitions.
pub trait SelfRadioTransition<RadioDriverImpl, ThisTask: RadioTask, NextTask: RadioTask> {
    /// Executes the internal radio self-transition. Returns the same state with
    /// a new task instance once the transition completed.
    ///
    /// Switching to the new state SHALL include completing the previous task,
    /// executing the full transition behavior but NOT `exit()` or
    /// `transition()` in the following order:
    /// 1. transition: on_scheduled() - non-blocking
    /// 2. source radio state: run() - blocking
    /// 3. transition: on_task_complete() - non-blocking
    /// 4. transition: cleanup() - non-blocking
    ///
    /// Note that - other than for external transitions - the task result will
    /// be available synchronously after task completion. This is due to the
    /// fact that no blocking entry behavior is required on self-transitions.
    ///
    /// The naming of this method was chosen to be readable and intuitive in
    /// client code: Once the async method completes, the previous task
    /// completed without changing the radio state.
    fn execute_transition(
        self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask>>;
}

impl<
        RadioDriverImpl,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError>,
        OnTaskComplete: Fn() -> Result<(), SchedulingError>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > SelfRadioTransition<RadioDriverImpl, ThisTask, NextTask>
    for RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnTaskComplete, Cleanup>
where
    RadioDriver<RadioDriverImpl, ThisTask>: RadioState<ThisTask>,
    RadioDriver<RadioDriverImpl, NextTask>: RadioState<NextTask>,
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
{
    async fn execute_transition(
        mut self,
    ) -> CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask> {
        if let Err(scheduling_error) = (self.on_scheduled)() {
            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error),
                None,
                self.next_task,
            );
        }

        let prev_task_result = match self.from_radio.run(self.alt_outcome_is_error).await {
            Err(scheduling_error) => {
                let _ = (self.cleanup)();
                return CompletedRadioTransition::Rollback(
                    self.from_radio,
                    scheduling_error,
                    None,
                    self.next_task,
                );
            }
            Ok(prev_task_result) => prev_task_result,
        };

        if let Err(scheduling_error) = (self.on_task_complete)() {
            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error),
                Some(prev_task_result),
                self.next_task,
            );
        }

        if let Err(next_task_error) = (self.cleanup)() {
            return CompletedRadioTransition::Fallback(
                RadioTransitionResult {
                    prev_task_result,
                    prev_state: PhantomData,
                    this_state: RadioDriver::<RadioDriverImpl, TaskOff>::switch_off(
                        self.from_radio.inner,
                    )
                    .await,
                },
                next_task_error,
            );
        }

        CompletedRadioTransition::Entered(RadioTransitionResult {
            prev_task_result,
            prev_state: PhantomData,
            this_state: RadioDriver {
                inner: self.from_radio.inner,
                task: Some(self.next_task),
            },
        })
    }
}

/// Represents the result of a successful radio transition.
pub struct RadioTransitionResult<RadioDriverImpl, PrevTask: RadioTask, ThisTask: RadioTask> {
    /// The result of the task that was completed by this transition.
    pub prev_task_result: PrevTask::Result,

    prev_state: PhantomData<RadioDriver<RadioDriverImpl, PrevTask>>,

    /// The currently active radio state that was entered through this
    /// transition.
    pub this_state: RadioDriver<RadioDriverImpl, ThisTask>,
}

/// Represents a completed non-deterministic active radio state transition.
pub enum CompletedRadioTransition<RadioDriverImpl, PrevTask: RadioTask, ThisTask: RadioTask> {
    /// The previous task ended and the next scheduled task was started
    /// successfully.
    Entered(RadioTransitionResult<RadioDriverImpl, PrevTask, ThisTask>),

    /// The scheduled transition to the next task could not be executed and was
    /// rolled back to the previous transition state. This happens if any of the
    /// source state's methods involved in task execution and transition - up to
    /// and including the source state's `exit()` method - returns an error.
    ///
    /// Note: The previous task may or may not have produced a result in this
    ///       case. If the result is `None` then the previous task SHALL remain
    ///       active otherwise it has completed. If the task produced a result
    ///       but the following `on_task_complete` or `exit()` methods fail,
    ///       then both, the task result and the subsequent error will be
    ///       reported.
    Rollback(
        RadioDriver<RadioDriverImpl, PrevTask>,
        RadioTaskError<PrevTask>,
        Option<PrevTask::Result>,
        ThisTask,
    ),

    /// The source state's task was successfully executed and left but the
    /// target state could not be entered because the target state's
    /// `transition()` or the transition's `cleanup()` method produced an error.
    ///
    /// To avoid leaving the driver in an undefined state, this will result in
    /// the radio to be switched off, i.e. it reaches a well-defined state that
    /// can be entered infallibly from which the scheduler can continue to
    /// operate.
    Fallback(
        RadioTransitionResult<RadioDriverImpl, PrevTask, TaskOff>,
        RadioTaskError<ThisTask>,
    ),
}
