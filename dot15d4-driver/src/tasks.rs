// TODO: This is a generic, vendor-independent API. Move this file to a place
//       where it can be accessed by all HALs and the scheduler.
use core::{convert::Infallible, future::Future, marker::PhantomData};

use crate::{
    config::Channel,
    constants::A_MAX_SIFS_FRAME_SIZE,
    frame::{AddressingFields, FrameControl, RadioFrame, RadioFrameSized, RadioFrameUnsized},
    radio::DriverConfig,
    timer::{SyntonizedInstant, TimedSignal},
};

use super::radio::RadioDriver;

/// Tasks can be scheduled as fast as possible ("best effort") or at a
/// well-defined tick of the local radio clock ("scheduled").
///
/// The timestamp is represented as a [`SyntonizedInstant`] in terms of the
/// radio driver's local timer, i.e. the timestamp must already have been
/// compensated for clock drift.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Timestamp {
    /// A task with this timestamp will be executed back-to-back to the previous
    /// task with minimal standard-conforming inter-frame spacing.
    BestEffort,
    /// A task with this timestamp will be executed by the driver at a precisely
    /// defined time. The semantics of the timestamp depends on the task that's
    /// being scheduled (see the corresponding task's definition).
    ///
    /// Usually the timestamp will be related to the RMARKER of a frame. For all
    /// PHYs the RMARKER is defined to be the time when the beginning of the
    /// first symbol following the SFD of the frame is at the local antenna.
    Scheduled(SyntonizedInstant),
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
    /// The driver SHALL cut off ongoing reception or transmission at the given
    /// time if an off task is scheduled with a timestamp. If this behavior is
    /// to be avoided, then Rx/Tx tasks SHALL be followed by a best-effort off
    /// task.
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
    /// The earliest time at which a frame with this RMARKER passing the local
    /// antenna SHALL be recognized. The receiver SHALL be switched on as late
    /// as possible to minimize energy consumption.
    ///
    /// Note: We do not define Rx window duration at the radio driver level.
    ///       Schedule a [`TaskOff`] instead to end an Rx window. It's the
    ///       responsibility of the driver service to compensate for clock drift
    ///       and insert guard times.
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
    /// the time at which the RMARKER of the outbound frame SHALL pass the local
    /// antenna.
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
pub struct SchedulingError<Task> {
    pub task: Task,
}

/// Represents a radio task or scheduling error.
#[derive(Debug, PartialEq, Eq)]
pub enum RadioTaskError<Task: RadioTask> {
    /// Any interaction with the radio may fail and the scheduler will have to
    /// deal with this.
    Scheduling(Task),

    /// The radio task itself failed.
    Task(Task::Error),
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
    /// This method SHALL be executed on both, external transitions and
    /// self-transitions.
    ///
    /// This means that the method strictly returns an undefined time _after_
    /// the transition executed from a conceptual viewpoint (section
    /// 14.2.3.4.5).  Returning from this method signals to the radio task
    /// scheduler that the state machine is ready to receive the next scheduling
    /// event, i.e. the next task can be scheduled/pre-programmed.
    ///
    /// In practice this method SHOULD return such that the radio task scheduler
    /// has sufficient time to schedule the next task before the current task
    /// ends. This is required to guarantee deterministic, CPU-independent
    /// timing of radio tasks.
    ///
    /// Self-transitions: Any transition-specific `cleanup` behavior will be
    /// executed right after this method returns.
    ///
    /// Returns `Ok` if the transition successfully executed, `Err` otherwise.
    ///
    /// Note: Implementations SHOULD ensure that this method is being called
    ///       before the driver actually switched state internally.
    ///       Implementations SHALL ensure that nevertheless the method
    ///       terminates right away if the driver has already switched state
    ///       internally.
    fn transition(
        &mut self,
        timed_transition: Option<TimedSignal>,
    ) -> impl Future<Output = Result<(), RadioTaskError<Task>>>;

    /// SHALL implement state specific entry behavior (UML 2.5.1, section
    /// 14.2.3.4.3).
    ///
    /// Executes right after the transition executed (section 14.2.3.4.5).
    ///
    /// SHALL NOT be executed on self-transitions (section 14.2.3.4.3) or if the
    /// transition failed.
    ///
    /// External transitions: Any transition-specific `cleanup` behavior will be
    /// executed right after this method returns.
    ///
    /// Returns `Ok` if the state was successfully entered, `Err` otherwise.
    fn entry(&mut self) -> Result<(), RadioTaskError<Task>>;

    /// Waits until the current state's task ("do activity") is complete.
    ///
    /// In case of a "best effort" task this waits until the current state exits
    /// by itself. Timed tasks MAY optionally schedule a timed completion event
    /// that MAY interrupt an ongoing do activity (e.g. reception of an incoming
    /// frame).
    ///
    /// Any transition-specific "on_completed" behavior will be executed right
    /// after this method returns.
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
    fn completion(
        &mut self,
        timed_completion: Option<TimedSignal>,
        alt_outcome_is_error: bool,
    ) -> impl Future<Output = Result<Task::Result, RadioTaskError<Task>>>;

    /// Ensures leftovers from task execution have been cleaned up before the
    /// state is left.
    ///
    /// MAY fail with a scheduling error.
    ///
    /// SHALL NOT be executed on self-transitions.
    ///
    /// Deviating from the UML standard, `exit()` SHALL NOT rely on state
    /// invariants being upheld while executing. This is due to the fact that
    /// task completion may have triggered immediate hardware-level
    /// transitioning to the next state in the background.
    ///
    /// Note: Implementations SHALL ensure that this method is being called
    ///       _after_ the state's task completed, i.e. after awaiting
    ///       `completion()`.
    fn exit(&mut self) -> Result<(), SchedulingError<Task>>;
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
pub trait OffState<RadioDriverImpl: DriverConfig>: RadioState<TaskOff> {
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
    fn switch_off<AnyState>(
        any_state: RadioDriver<RadioDriverImpl, AnyState>,
    ) -> impl Future<Output = Self>;
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
pub trait RxState<RadioDriverImpl: DriverConfig>: RadioState<TaskRx> {
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
    ///
    /// If the task is timed, i.e. defines a `start` timestamp, the IFS SHALL be
    /// ignored and SHOULD be set to [`Ifs::None`].
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
pub trait TxState<RadioDriverImpl: DriverConfig>: RadioState<TaskTx> {
    /// Schedules a transition to the RX state.
    ///
    /// If the task is timed, i.e. defines a `start` timestamp, the IFS SHALL be
    /// ignored and SHOULD be set to [`Ifs::None`].
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
    ///
    /// If the task is timed, i.e. defines an `at` timestamp, the IFS SHALL
    /// be ignored and SHOULD be set to [`Ifs::None`].
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        ifs: Ifs,
    ) -> impl SelfRadioTransition<RadioDriverImpl, TaskTx, TaskTx>;

    /// Schedules a transitions to the Radio Off state.
    ///
    /// Note: As Tx will end deterministically, it doesn't make sense to
    ///       schedule a "Tx window". Therefore only best-effort scheduling
    ///       SHALL be allowed.
    fn schedule_off(
        self,
        off_task: TaskOff,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskTx, TaskOff>;
}

/// Represents an active radio state transition while it is being traversed.
pub struct RadioTransition<
    RadioDriverImpl: DriverConfig,
    ThisTask: RadioTask,
    NextTask: RadioTask,
    OnScheduled: Fn() -> Result<(), SchedulingError<ThisTask>>,
    OnCompleted: Fn() -> Result<(), SchedulingError<ThisTask>>,
    Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
> {
    /// The source radio peripheral state of the transition.
    from_radio: RadioDriver<RadioDriverImpl, ThisTask>,

    /// The target radio peripheral state of the transition.
    to_radio: PhantomData<RadioDriver<RadioDriverImpl, NextTask>>,

    /// Configuration and parameters of the target radio peripheral state.
    next_task: NextTask,

    /// In case of a timed task: its (optional) timed completion event. This
    /// event signals completion of `ThisTask`.
    timed_completion: Option<TimedSignal>,

    /// In case of a timed task: its (optional) timed transition event. This
    /// event signals transition to `NextTask`.
    timed_transition: Option<TimedSignal>,

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
    ///
    /// SHALL NOT rely on state invariants being upheld while executing. This is
    /// due to the fact that task completion may have triggered immediate
    /// hardware-level transitioning to the next state in the background.
    on_completed: OnCompleted,

    /// Callback to clean up any transition-specific setup or left-overs.
    ///
    /// If the transition succeeds or falls back due to an error in the target
    /// task's `transition()` method: executed as soon as the `transition()`
    /// method returned (i.e. the radio task entered the target state).
    ///
    /// If the transition is rolled back due to an error in the source task's
    /// `completion()` or `exit()` methods: executed immediately after those
    /// methods.
    ///
    /// SHALL NOT rely on state invariants being upheld while executing. This is
    /// due to the fact that task completion may have triggered immediate
    /// hardware-level transitioning to the next state in the background.
    ///
    /// Note: This callback will _not_ be called when the transition's own
    ///       `on_scheduled` or `on_completed` callbacks fail. In that case it
    ///       is assumed that those callbacks will clean up after themselves.
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
        RadioDriverImpl: DriverConfig,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError<ThisTask>>,
        OnCompleted: Fn() -> Result<(), SchedulingError<ThisTask>>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnCompleted, Cleanup>
{
    /// Instantiates a new radio transition.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        from_radio: RadioDriver<RadioDriverImpl, ThisTask>,
        next_task: NextTask,
        timed_completion: Option<TimedSignal>,
        timed_transition: Option<TimedSignal>,
        on_scheduled: OnScheduled,
        on_completed: OnCompleted,
        cleanup: Cleanup,
        alt_outcome_is_error: bool,
    ) -> Self {
        Self {
            from_radio,
            to_radio: PhantomData,
            next_task,
            timed_completion,
            timed_transition,
            on_scheduled,
            on_completed,
            cleanup,
            alt_outcome_is_error,
        }
    }
}

/// Represents an active external radio state transition while it is being
/// traversed.
///
/// External transitions have distinct source and target states.
pub trait ExternalRadioTransition<
    RadioDriverImpl: DriverConfig,
    ThisTask: RadioTask,
    NextTask: RadioTask,
>
{
    /// Executes the current radio task (i.e. the current state's do activity)
    /// to completion. Then waits for the current state to exit and executes the
    /// external radio transition to the target state. Returns the target state
    /// with a new task instance once the transition completed.
    ///
    /// Switching to the new state SHALL include completing the source task and
    /// executing source- and target-state specific transition behavior in the
    /// following order:
    /// 1. transition: on_scheduled() - synchronous
    /// 2. source radio state: completion() - asynchronous
    /// 3. transition: on_completed() - synchronous
    /// 4. source radio state: exit() - synchronous
    /// 5. target radio state: transition() - asynchronous
    /// 6. target radio state: entry() - synchronous
    /// 7. transition: cleanup() - synchronous
    ///
    /// Note that the task result is known as soon as the `completion()` call
    /// (i.e.  the current state's "do activity") finishes but will only be
    /// returned to the radio task scheduler once the `transition()` to the
    /// target state also finishes, see the [`RadioDriver`] documentation for
    /// UML state machine compatibility.
    fn complete_and_transition(
        self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask>>;
}

impl<
        RadioDriverImpl: DriverConfig,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError<ThisTask>>,
        OnCompleted: Fn() -> Result<(), SchedulingError<ThisTask>>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > ExternalRadioTransition<RadioDriverImpl, ThisTask, NextTask>
    for RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnCompleted, Cleanup>
where
    RadioDriver<RadioDriverImpl, ThisTask>: RadioState<ThisTask>,
    RadioDriver<RadioDriverImpl, NextTask>: RadioState<NextTask>,
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
{
    async fn complete_and_transition(
        mut self,
    ) -> CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask> {
        if let Err(scheduling_error) = (self.on_scheduled)() {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error.task),
                None,
                self.next_task,
            );
        }

        let prev_task_result = match self
            .from_radio
            .completion(self.timed_completion, self.alt_outcome_is_error)
            .await
        {
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

        if let Err(scheduling_error) = (self.on_completed)() {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error.task),
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
                RadioTaskError::Scheduling(scheduling_error.task),
                Some(prev_task_result),
                self.next_task,
            );
        }

        let RadioDriver { inner, timer, .. } = self.from_radio;
        let mut next_state = RadioDriver {
            inner,
            timer,
            task: Some(self.next_task),
        };
        let next_state_entry = next_state.transition(self.timed_transition).await;

        let fallback = |next_task_error, prev_task_result, any_state| async {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            CompletedRadioTransition::Fallback(
                RadioTransitionResult {
                    prev_task_result,
                    prev_state: PhantomData,
                    this_state: RadioDriver::<RadioDriverImpl, TaskOff>::switch_off(any_state)
                        .await,
                },
                next_task_error,
            )
        };

        if next_state_entry.is_ok() {
            if let Err(next_task_error) = next_state.entry() {
                return fallback(next_task_error, prev_task_result, next_state).await;
            }
        }

        if let Err(next_task_error) = (self.cleanup)() {
            return fallback(next_task_error, prev_task_result, next_state).await;
        }

        match next_state_entry {
            Ok(_) => CompletedRadioTransition::Entered(RadioTransitionResult {
                prev_task_result,
                prev_state: PhantomData,
                this_state: next_state,
            }),
            Err(next_task_error) => fallback(next_task_error, prev_task_result, next_state).await,
        }
    }
}

/// Represents an active radio state self-transition while it is being
/// traversed.
///
/// Self transitions have the same source and target states. They are also
/// called internal transitions.
pub trait SelfRadioTransition<
    RadioDriverImpl: DriverConfig,
    ThisTask: RadioTask,
    NextTask: RadioTask,
>
{
    /// Executes the current radio task (i.e. the current state's do activity)
    /// to completion. Then executes the internal radio self-transition (i.e.
    /// without exiting/re-entering the state). Returns the same state with a
    /// new task instance once the transition completed.
    ///
    /// Switching to the new state SHALL include completing the current task and
    /// executing the full state-specific transition behavior but NOT `exit()`
    /// or `entry()` in the following order:
    /// 1. transition: on_scheduled() - synchronous
    /// 2. source radio state: completion() - asynchronous
    /// 3. transition: on_completed() - synchronous
    /// 4. target radio state: transition() - asynchronous
    /// 5. transition: cleanup() - synchronous
    ///
    /// Note that the task result is known as soon as the `completion()` call
    /// (i.e. the current state's "do activity") finishes but will only be
    /// returned to the radio task scheduler once the `transition()` to the
    /// target state also finishes, see the [`RadioDriver`] documentation for
    /// UML state machine compatibility.
    fn run_and_transition(
        self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask>>;
}

impl<
        RadioDriverImpl: DriverConfig,
        ThisTask: RadioTask,
        NextTask: RadioTask,
        OnScheduled: Fn() -> Result<(), SchedulingError<ThisTask>>,
        OnCompleted: Fn() -> Result<(), SchedulingError<ThisTask>>,
        Cleanup: Fn() -> Result<(), RadioTaskError<NextTask>>,
    > SelfRadioTransition<RadioDriverImpl, ThisTask, NextTask>
    for RadioTransition<RadioDriverImpl, ThisTask, NextTask, OnScheduled, OnCompleted, Cleanup>
where
    RadioDriver<RadioDriverImpl, ThisTask>: RadioState<ThisTask>,
    RadioDriver<RadioDriverImpl, NextTask>: RadioState<NextTask>,
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
{
    async fn run_and_transition(
        mut self,
    ) -> CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask> {
        if let Err(scheduling_error) = (self.on_scheduled)() {
            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error.task),
                None,
                self.next_task,
            );
        }

        let prev_task_result = match self
            .from_radio
            .completion(self.timed_completion, self.alt_outcome_is_error)
            .await
        {
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

        if let Err(scheduling_error) = (self.on_completed)() {
            return CompletedRadioTransition::Rollback(
                self.from_radio,
                RadioTaskError::Scheduling(scheduling_error.task),
                Some(prev_task_result),
                self.next_task,
            );
        }

        let RadioDriver { inner, timer, .. } = self.from_radio;
        let mut next_state = RadioDriver {
            inner,
            timer,
            task: Some(self.next_task),
        };
        let next_state_entry = next_state.transition(self.timed_transition).await;

        let fallback = |next_task_error, prev_task_result, any_state| async {
            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::task_exec_end();

            CompletedRadioTransition::Fallback(
                RadioTransitionResult {
                    prev_task_result,
                    prev_state: PhantomData,
                    this_state: RadioDriver::<RadioDriverImpl, TaskOff>::switch_off(any_state)
                        .await,
                },
                next_task_error,
            )
        };

        if let Err(next_task_error) = (self.cleanup)() {
            return fallback(next_task_error, prev_task_result, next_state).await;
        }

        match next_state_entry {
            Ok(_) => CompletedRadioTransition::Entered(RadioTransitionResult {
                prev_task_result,
                prev_state: PhantomData,
                this_state: next_state,
            }),
            Err(next_task_error) => fallback(next_task_error, prev_task_result, next_state).await,
        }
    }
}

/// Represents the result of a successful radio transition.
pub struct RadioTransitionResult<
    RadioDriverImpl: DriverConfig,
    PrevTask: RadioTask,
    ThisTask: RadioTask,
> {
    /// The result of the task that was completed by this transition.
    pub prev_task_result: PrevTask::Result,

    prev_state: PhantomData<RadioDriver<RadioDriverImpl, PrevTask>>,

    /// The currently active radio state that was entered through this
    /// transition.
    pub this_state: RadioDriver<RadioDriverImpl, ThisTask>,
}

/// Represents a completed non-deterministic active radio state transition.
pub enum CompletedRadioTransition<
    RadioDriverImpl: DriverConfig,
    PrevTask: RadioTask,
    ThisTask: RadioTask,
> {
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
    ///       but the following `on_completed` or `exit()` methods fail, then
    ///       both, the task result and the subsequent error will be reported.
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
