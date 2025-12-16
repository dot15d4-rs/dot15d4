use core::{convert::Infallible, future::Future, marker::PhantomData};

use crate::{
    radio::phy::PhyConfig,
    timer::{NsDuration, NsInstant, OptionalNsInstant, RadioTimerError},
};

use super::{
    config::Channel,
    frame::{AddressingFields, FrameControl, RadioFrame, RadioFrameSized, RadioFrameUnsized},
    phy::Ifs,
    DriverConfig, PhyOf, RadioDriver,
};

/// Generic representation of a radio task.
///
/// Some features of radio tasks are mandatory, others are optional (see the
/// documentation of structs implementing this trait).
///
/// Mandatory features of radio tasks SHALL be implemented by all drivers while
/// optional features SHOULD be implemented if the radio peripheral offers the
/// corresponding functionality ("hardware offloading").
pub trait RadioTask: Sized {
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
    ///       on the context: If an independent tx frame is scheduled after an
    ///       rx task ending in a CRC error, then the tx frame must be scheduled
    ///       nevertheless. The same is not true for an acknowledgment frame
    ///       scheduled after an incoming frame whose CRC check fails. This
    ///       explains why the "CRC not ok" outcome exists as both, a result and
    ///       an error. The scheduling API will allow to pass in flags that
    ///       allow the driver to distinguish between those cases.
    type Result;

    /// A transition MAY fail if it the source state's task or the transition to
    /// the target state produce an error (e.g.  due to a failed precondition
    /// like a busy channel in the tx case or even due to message collision on
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

    /// Convert this task into a task-agnostic representation.
    fn into_any_task(self) -> AnyTask;

    /// Construct this task from its task-agnostic representation.
    ///
    /// Returns [`None`] if the given task isn't representing this task.
    fn try_from_any_task(any_task: AnyTask) -> Option<Self>;

    /// Construct a reference to this task from a reference to its task-agnostic
    /// representation.
    ///
    /// Returns [`None`] if the given task isn't representing this task.
    fn try_from_any_task_ref(any_task: &AnyTask) -> Option<&Self>;
}

/// Task: switch to low energy state
#[derive(Debug, PartialEq, Eq)]
pub struct TaskOff;
#[derive(Debug, PartialEq, Eq)]
pub enum OffResult {
    Off,
}
impl RadioTask for TaskOff {
    type Result = OffResult;
    type Error = Infallible;

    fn into_any_task(self) -> AnyTask {
        AnyTask::Off(self)
    }

    fn try_from_any_task(any_task: AnyTask) -> Option<Self> {
        match any_task {
            AnyTask::Off(task_off) => Some(task_off),
            _ => None,
        }
    }

    fn try_from_any_task_ref(any_task: &AnyTask) -> Option<&Self> {
        match any_task {
            AnyTask::Off(task_off) => Some(task_off),
            _ => None,
        }
    }
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
/// If the rx task receives a non-ACK frame, the driver SHALL store the frame's
/// sequence number (if present) on-the-fly. Actual acknowledgement will be
/// scheduled subsequently by the client via a regular tx task containing the
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
/// If the rx task receives a data, multi-purpose or command frame with the AR
/// flag set and matching all filtering and security criteria (see IEEE
/// 802.15.4-2024, section 6.6.2), then the driver SHALL auto-acknowledge the
/// frame.
#[derive(Debug, PartialEq, Eq)]
pub struct TaskRx {
    /// radio frame allocated to receive incoming frames
    pub radio_frame: RadioFrame<RadioFrameUnsized>,
}
/// rx task result
#[derive(Debug, PartialEq, Eq)]
pub enum RxResult {
    /// A valid frame was successfully received and acknowledged if requested.
    Frame(
        /// received radio frame
        RadioFrame<RadioFrameSized>,
        /// the frame's RMARKER timestamp
        NsInstant,
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
    ///       off, rx or tx task back-to-back to an rx task.
    CrcError(
        /// recovered radio frame
        RadioFrame<RadioFrameUnsized>,
        /// the frame's RMARKER timestamp
        NsInstant,
    ),
}
#[derive(Debug, PartialEq, Eq)]
pub enum RxError {
    /// A frame was received but the CRC didn't match.
    ///
    /// Note: This error is returned if the driver was programmed to remain in
    ///       the rx state on CRC error, e.g. to ensure that an ACK frame
    ///       scheduled back-to-back to an rx frame is not being sent when the
    ///       checksum doesn't match.
    CrcError,
}
impl RadioTask for TaskRx {
    type Result = RxResult;
    type Error = RxError;

    fn into_any_task(self) -> AnyTask {
        AnyTask::Rx(self)
    }

    fn try_from_any_task(any_task: AnyTask) -> Option<Self> {
        match any_task {
            AnyTask::Rx(task_rx) => Some(task_rx),
            _ => None,
        }
    }

    fn try_from_any_task_ref(any_task: &AnyTask) -> Option<&Self> {
        match any_task {
            AnyTask::Rx(task_rx) => Some(task_rx),
            _ => None,
        }
    }
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
/// If the tx task represents a non-ACK tx frame then the frame SHALL be sent
/// unchanged. If the frame requires acknowledgment, a regular rx task will be
/// scheduled subsequently by the client awaiting the ACK frame ("soft MAC").
///
/// If the tx task represents an ACK tx frame, then the driver SHALL set the
/// sequence number from the preceding rx frame on-the-fly and respect the AIFS.
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
    /// radio frame to be sent
    pub radio_frame: RadioFrame<RadioFrameSized>,

    /// whether CCA is to be performed as a precondition to send out the frame
    pub cca: bool,
}
/// tx task result
#[derive(Debug, PartialEq, Eq)]
pub enum TxResult {
    /// The frame was successfully sent and acknowledged if requested.
    /// Does not yet carry any data but MAY do so in the future.
    Sent(
        /// The transmitted frame.
        RadioFrame<RadioFrameSized>,
        /// the frame's RMARKER timestamp
        NsInstant,
    ),
}
#[derive(Debug, PartialEq, Eq)]
/// tx task error
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

    fn into_any_task(self) -> AnyTask {
        AnyTask::Tx(self)
    }

    fn try_from_any_task(any_task: AnyTask) -> Option<Self> {
        match any_task {
            AnyTask::Tx(task_tx) => Some(task_tx),
            _ => None,
        }
    }

    fn try_from_any_task_ref(any_task: &AnyTask) -> Option<&Self> {
        match any_task {
            AnyTask::Tx(task_tx) => Some(task_tx),
            _ => None,
        }
    }
}

/// An enum able to represent any task.
pub enum AnyTask {
    Off(TaskOff),
    Rx(TaskRx),
    Tx(TaskTx),
}

impl AnyTask {
    /// Downcast a generic task into its concrete implementation.
    ///
    /// Note: We only need this as a consequence of having to keep task state in
    ///       static memory to avoid overhead of async blocks when constructing
    ///       concrete radio states on the stack.
    pub fn try_into_task<Task: RadioTask>(self) -> Option<Task> {
        Task::try_from_any_task(self)
    }

    /// Downcast a reference to a generic task into a reference to its concrete
    /// implementation.
    ///
    /// Note: We only need this as a consequence of having to keep task state in
    ///       static memory to avoid overhead of async blocks when constructing
    ///       concrete radio states on the stack.
    pub fn try_into_task_ref<Task: RadioTask>(&self) -> Option<&Task> {
        Task::try_from_any_task_ref(self)
    }
}

impl<Task: RadioTask> From<Task> for AnyTask {
    fn from(task: Task) -> Self {
        task.into_any_task()
    }
}

/// A scheduling error is produced whenever timing-related errors occur. It
/// optionally contains the offending timestamp that could not be scheduled.
///
/// Semantics of this timestamp depends on the context in which the error
/// occurs and will be documented there.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SchedulingError {
    pub instant: OptionalNsInstant,
}

impl From<RadioTimerError> for SchedulingError {
    fn from(value: RadioTimerError) -> Self {
        match value {
            RadioTimerError::Overdue(instant) => SchedulingError {
                instant: instant.into(),
            },
            RadioTimerError::Already | RadioTimerError::Busy => SchedulingError {
                instant: None.into(),
            },
        }
    }
}

/// Represents a radio task or scheduling error.
#[derive(Debug, PartialEq, Eq)]
pub enum RadioTaskError<Task: RadioTask> {
    /// Any interaction with the radio may fail and the scheduler will have to
    /// deal with this.
    Scheduling(Task, SchedulingError),

    /// The radio task itself failed.
    Task(Task::Error),
}

/// Basic features to be implemented by all radio drivers, independent of driver
/// state.
///
/// This object also acts as a zero-sized pointer (proxy) into the radio driver
/// state of the driver instance represented by the driver configuration.
pub trait RadioDriverApi<RadioDriverImpl: DriverConfig, Task: RadioTask> {
    /// Return the device's IEEE 802.15.4 extended address.
    fn ieee802154_address(&self) -> [u8; 8];

    /// Irreversibly switch to the next task state.
    fn enter_next_task<NextTask: RadioTask>(
        self,
        next_task: NextTask,
    ) -> RadioDriver<RadioDriverImpl, NextTask>
    where
        RadioDriver<RadioDriverImpl, NextTask>: RadioState<RadioDriverImpl, NextTask>;

    /// Switches the radio off immediately and unconditionally.
    ///
    /// This method will be called whenever a non-recoverable error is
    /// encountered. The method must place the driver into the well-defined off
    /// state under all conditions. If this is not possible, it SHALL panic.
    ///
    /// Returns the off state and the precise time at which the state entered.
    ///
    /// # Panics
    ///
    /// If the radio cannot fall back to off state at any time, this is
    /// considered a fatal error.
    fn switch_off(self)
        -> impl Future<Output = (RadioDriver<RadioDriverImpl, TaskOff>, NsInstant)>;

    /// Returns a copy of the radio's sleep timer for client use.
    fn sleep_timer(&self) -> RadioDriverImpl::Timer;

    /// Returns the scheduled entry timestamp of the radio state's task, if any.
    fn scheduled_entry(&self) -> OptionalNsInstant;

    /// Sets the measured entry time of the radio state's task as returned by
    /// [`RadioState::transition()`].
    fn set_measured_entry(&mut self, measured_entry: NsInstant);

    /// Consume the radio state's task.
    ///
    /// # Panics
    ///
    /// If the task had been consumed before.
    fn take_task(&mut self) -> Task;

    /// Consume the radio state's task and derive a scheduling error from it.
    ///
    /// # Panics
    ///
    /// If the task had been consumed before.
    fn scheduling_error(&mut self) -> RadioTaskError<Task> {
        RadioTaskError::Scheduling(
            self.take_task(),
            SchedulingError {
                instant: self.scheduled_entry(),
            },
        )
    }
}

/// Methods to be implemented by all an IEEE 802.15.4 radio driver state machine
/// states.
pub trait RadioState<RadioDriverImpl: DriverConfig, Task: RadioTask>:
    RadioDriverApi<RadioDriverImpl, Task>
{
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
    /// Returns the precise timestamp at which the state entered if the
    /// transition successfully executed, `Err` otherwise:
    ///
    /// - In case of an rx task: The measured radio clock instant representing
    ///   the rx task's start time, i.e. the earliest time at which a frame's
    ///   RMARKER could have passed the local antenna and reliably be observed
    ///   by the radio. In case of a timed task, this SHALL equal the scheduled
    ///   `start` time +/- rounding error.
    ///
    /// - In case of a tx task: The measured radio clock instant representing
    ///   the tx task's start time, i.e. the time at which the transmitted
    ///   frame's RMARKER passed the local antenna. In case of a timed task,
    ///   this SHALL equal the scheduled `at` time +/- rounding error.
    ///
    /// - In case of a "Radio Off" task: The measured radio clock instant
    ///   at which the radio was disabled. In case of a timed task, this SHALL
    ///   equal the scheduled `at` time +/- rounding error.
    ///
    /// The rounding error SHALL be at most the duration of one radio clock
    /// tick.
    ///
    /// Note: Implementations SHOULD ensure that this method is being called
    ///       before the driver actually switched state internally.
    ///       Implementations SHALL ensure that nevertheless the method
    ///       terminates right away if the driver has already switched state
    ///       internally.
    ///
    /// The returned future SHALL NOT be cancelled.
    fn transition(&mut self) -> impl Future<Output = Result<NsInstant, RadioTaskError<Task>>>;

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
    ///
    /// The returned future SHALL NOT be cancelled.
    fn completion(
        &mut self,
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
pub trait OffState<RadioDriverImpl: DriverConfig>:
    RadioState<RadioDriverImpl, TaskOff> + Sized
{
    /// Set the current radio channel.
    fn set_channel(&mut self, channel: Channel);

    /// Schedules a transition to the rx state.
    ///
    /// The (optional) start time designates the earliest expected time at which
    /// the RMARKER of an incoming frame may pass the local antenna to be
    /// observed within the scheduled rx window.
    fn schedule_rx(
        self,
        rx_task: TaskRx,
        start: OptionalNsInstant,
        channel: Option<Channel>,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskOff, TaskRx>;

    /// Schedules a transition to the tx state.
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else remains in the "Radio Off" state.
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        at: OptionalNsInstant,
        channel: Option<Channel>,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskOff, TaskTx>;
}

pub struct PreliminaryFrameInfo<'frame> {
    pub mpdu_length: u16,
    pub frame_control: Option<FrameControl<[u8; 2]>>,
    pub seq_nr: Option<u8>,
    pub addressing_fields: Option<AddressingFields<&'frame [u8]>>,
}

pub enum StopListeningResult<
    RadioDriverImpl: DriverConfig,
    RxState: ReceivingRxState<RadioDriverImpl>,
> {
    FrameStarted(NsInstant, RxState),
    RxWindowEnded(RadioTransitionResult<RadioDriverImpl, TaskRx, TaskOff>),
}

/// The radio reception state is split into two sub-states: listening state
/// (this state) and receiving state (see below).
///
/// The listening state is active while the receiver is on but no incoming frame
/// has been observed, yet. In this state scheduling of the subsequent state is
/// often not possible because it is not known whether a frame will arrive,
/// whether the arriving frame is valid and whether it requires acknowledgement
/// or contains IEs that possibly need to be reacted to by upper layers.
///
/// This is the generic characterization of the "Receiver ON" (rx) state, i.e.
/// the start of a frame SHALL be recognized while in this state.
pub trait ListeningRxState<RadioDriverImpl: DriverConfig>:
    RadioState<RadioDriverImpl, TaskRx> + Sized
{
    /// Utility method to calculate the PHY specific time that a PPDU with the
    /// given PSDU (=MPDU) size occupies the physical channel.
    ///
    /// See [`ListeningRxState::stop_listening`] for an application.
    fn ppdu_rx_duration(&self, psdu_size: u16) -> NsDuration;

    /// Utility method to calculate the latest possible frame start of an
    /// inbound frame (in terms of its RMARKER and of the given expected max
    /// PSDU size) such that a timed transmission at the given tx instant (in
    /// terms of its RMARKER and optionally including sufficient time for CCA)
    /// can still be scheduled in time.
    ///
    /// If no PSDU size is given, then the max PSDU size of the driver's PHY is
    /// used in the calculation.
    ///
    /// See [`ListeningRxState::stop_listening()`] and
    /// [`ReceivingRxState::schedule_tx()`] for further information about how
    /// this is calculated.
    fn latest_rx_frame_start_before_tx(
        &self,
        tx_at: NsInstant,
        cca: bool,
        expected_max_psdu_size: Option<u16>,
    ) -> NsInstant {
        // rx_task_end
        //   = tx_at - (cca ? macUnitBackoffPeriod : 0) - LIFS
        //     - rmarker_offset
        //
        // latest_frame_start
        //   = rx_task_end - ppdu_rx_time(phyMaxPacketSize)
        //     + rmarker_offset
        //   = tx_at - (cca ? macUnitBackoffPeriod : 0) - LIFS
        //     - ppdu_rx_time(phyMaxPacketSize)
        //
        // Note:
        //  - The RMARKER offsets cancel each other out.
        //  - As we may still receive a max-sized frame, we need
        //    to cater for LIFS. Shorter frames will end earlier
        //    anyway.
        let psdu_size = expected_max_psdu_size
            .unwrap_or(<PhyOf<RadioDriverImpl> as PhyConfig>::PHY_MAX_PACKET_SIZE);
        let max_ppdu_rx = self.ppdu_rx_duration(psdu_size);
        let mut latest_frame_start =
            tx_at - <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_LIFS_PERIOD - max_ppdu_rx;
        if cca {
            let back_off_period = <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_UNIT_BACKOFF_PERIOD;
            latest_frame_start -= back_off_period
        }
        latest_frame_start
    }

    /// Utility method to calculate the latest possible frame start of an
    /// inbound frame (in terms of its RMARKER and of the given expected max
    /// PSDU size) if the radio shall be switched off at the given instant.
    ///
    /// If no PSDU size is given, then the max PSDU size of the driver's PHY is
    /// used in the calculation.
    ///
    /// See [`ListeningRxState::stop_listening()`] and
    /// [`ReceivingRxState::schedule_off()`] for further information about how
    /// this is calculated.
    fn latest_rx_frame_start_before_off(
        &self,
        off_at: NsInstant,
        expected_max_psdu_size: Option<u16>,
    ) -> NsInstant {
        // latest_frame_start
        //   = rx_task_end - ppdu_rx_time(phyMaxPacketSize)
        //     + rmarker_offset
        //   = off_at - ppdu_rx_time(phyMaxPacketSize)
        //     + rmarker_offset
        let psdu_size = expected_max_psdu_size
            .unwrap_or(<PhyOf<RadioDriverImpl> as PhyConfig>::PHY_MAX_PACKET_SIZE);
        let max_ppdu_rx = self.ppdu_rx_duration(psdu_size);
        off_at - max_ppdu_rx + <PhyOf<RadioDriverImpl> as PhyConfig>::RMARKER_OFFSET
    }

    /// Waits indefinitely until the radio observes the start of a frame.
    /// Immediately returns the RMARKER timestamp of an incoming frame.
    ///
    /// This function SHOULD return as quickly as possible once a
    /// synchronization header has been observed by the receiver.
    ///
    /// The returned future SHALL be cancelable. Cancelling the future SHALL
    /// leave the radio in listening state.
    ///
    /// Note: This cannot be unified with [`ListeningRxState::stop_listening`]
    ///       as the latter must be non-cancellable to consume the listening
    ///       state while this method may only reference self.
    fn wait_for_frame_start(&mut self) -> impl Future<Output = NsInstant>;

    /// Calling this method irreversibly ends (and consumes) the listening state
    /// and transitions into a state that allows deterministic scheduling of
    /// subsequent tasks:
    ///
    /// - If frame reception is ongoing when calling this method: Immediately
    ///   returns with the RMARKER timestamp of the incoming frame and continues
    ///   in the receiving state.
    ///
    /// - If no frame has started and no timeout has been given: Immediately
    ///   turns the radio off and continues in the "Radio Off" state.
    ///
    /// - If no frame has started and a timeout has been given for the latest
    ///   frame start: Waits until the radio observes the start of a frame or
    ///   the timeout expires, whatever happens earlier. If a frame starts:
    ///   Immediately returns the RMARKER timestamp of the incoming frame and
    ///   continues in the receiving state. Otherwise stops listening at the
    ///   precise instant designated by the timeout and continues in the "Radio
    ///   Off" state.
    ///
    /// Inbound frames starting after this method returns SHALL be ignored.
    ///
    /// This function SHOULD return as quickly as possible once a
    /// synchronization header or timeout has been observed by the receiver.
    ///
    /// The returned future SHALL NOT be cancelled.
    ///
    /// In case a timed task is to be scheduled after this task then clients
    /// SHALL calculate and provide the latest frame start.
    ///
    /// The latest frame start designates the latest point in time at which the
    /// RMARKER of an incoming frame may pass the local antenna to be observed
    /// within this rx window:
    ///
    ///   latest_frame_start = rx_task_end
    ///                      - ppdu_rx_duration(max_expected_mpdu_size)
    ///                      + rmarker_offset
    ///
    /// rx_task_end:
    ///
    ///   Depends on the subsequent task, see [`ReceivingRxState::schedule_tx`]
    ///   and [`ReceivingRxState::schedule_off`]. Can be derived from the
    ///   earliest timed off and earliest timed tx respectively, see picture
    ///   below.
    ///
    /// ppdu_rx_duration(max_expected_mpdu_size):
    ///
    ///   Defines the time it takes to receive the full PPDU given an MPDU with
    ///   the max. expected size in this rx window.
    ///
    /// rmarker_offset:
    ///
    ///   The RMARKER offset is PHY-dependent, e.g. rx_time(SHR) in case of
    ///   O-QPSK.
    ///
    /// The following picture illustrates some of those notions:
    ///
    /// ```ignore
    ///     rx start                 latest frame start     rx task end
    ///     |                        |                      |
    ///     |                        |                      earliest off   earliest tx
    /// |SHR|<-earliest RMARKER  |SHR|<-latest RMARKER      v                        v
    /// |----- rx listening -----|-- max ppdu rx duration --|IFS+(CCA+TURNAROUND+)SHR|
    /// ```
    fn stop_listening(
        self,
        latest_frame_start: OptionalNsInstant,
    ) -> impl Future<
        Output = Result<
            StopListeningResult<RadioDriverImpl, impl ReceivingRxState<RadioDriverImpl>>,
            (SchedulingError, Self),
        >,
    >;
}

/// Receiving state
///
/// This is a generic characterization of the rx state after the start of an
/// incoming frame has been observed. The radio will receive the incoming frame
/// while in this state.
///
/// This state is different from the [`ListeningRxState`] in that the behavior
/// of external devices no longer imposes restrictions on the ability to decide
/// upon and schedule subsequent tasks.
///
/// Scheduling of subsequent tasks SHALL be done concurrently with frame
/// reception to ensure standard-conforming timing.
///
/// When transitioning away from the rx state, the target state depends on the
/// outcome of the rx task in combination with the `rollback_on_crcerror` flag:
///
/// - If the `rollback_on_crcerror` flag is true, the transition will be rolled
///   back in case of a CRC error. This is useful when acknowledgment is
///   required for a frame received by this rx task: In case of a CRC error an
///   already scheduled acknowledgement will not to be sent.
///
/// - If the `rollback_on_crcerror` flag is false then the transition takes
///   place independently of a CRC match. This is the correct behavior in case
///   of an independent rx or tx task being scheduled back-to-back to an
///   incoming frame that does not require acknowledgment. In this case we
///   expect the scheduled task to be executed independently of the preceding rx
///   result.
pub trait ReceivingRxState<RadioDriverImpl: DriverConfig>:
    RadioState<RadioDriverImpl, TaskRx>
{
    /// If a frame has started within the reception window then returns the
    /// preliminary frame information, otherwise returns [`None`].
    ///
    /// Preliminary frame information may be incomplete if frame reception ended
    /// prematurely or if the received frame does not contain one of the
    /// collected fields.
    ///
    /// Based on preliminary frame information, callers MAY validate the
    /// incoming frame and - in case of manual acknowledgement - construct an
    /// acknowledgement frame and schedule its transmission.
    ///
    /// Note: Reception will typically still be ongoing while calling this
    ///       method. This is important to ensure standard-conforming timing. It
    ///       is required that clients call this method and schedule subsequent
    ///       tasks as quickly as possible.
    fn preliminary_frame_info(&mut self) -> impl Future<Output = Option<PreliminaryFrameInfo<'_>>>;

    /// Schedules reception of a frame back-to-back to the frame currently being
    /// received with well-defined radio downtime.
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    ///
    /// If an IFS is given, then the driver SHALL deduce sufficient guard time,
    /// so that - given the local radio's IFS timer accuracy - a sent frame will
    /// be received, even assuming max clock drift.
    ///
    /// If no IFS is given then the radio will stay in rx mode and is
    /// immediately able to receive a subsequent frame.
    ///
    /// Note: When scheduling rx frame back-to-back, then only "best effort"
    ///       scheduling is allowed. The current rx window SHALL NOT be ended.
    ///       This is to avoid that schedulers enter an endless rx-rx-loop.
    ///       Schedulers SHALL await the `frame_started()` future to ensure that
    ///       another rx frame will only be scheduled when the current task is
    ///       guaranteed to do some work and completes soon.
    ///
    /// Note: The rx task undergoes several sub-states. We have to deal with the
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
    /// schedule the next rx task early enough such that condition 3 holds. But
    /// for stability and correctness we SHALL deal with the other two
    /// (exceptional) cases, too. We'll emit a warning, though.
    fn schedule_rx(
        self,
        rx_task: TaskRx,
        ifs: Option<Ifs<PhyOf<RadioDriverImpl>>>,
        rollback_on_crcerror: bool,
    ) -> impl SelfRadioTransition<RadioDriverImpl, TaskRx>;

    /// Ends the rx task and schedules a transition to the tx state with a
    /// well-defined IFS.
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else falls back to the "Radio Off"
    /// state.
    ///
    /// If the scheduled tx task is timed, i.e. an `at` timestamp is given, then
    /// the end of the rx task is calculated as follows:
    ///
    ///   rx_task_end = at
    ///               - rmarker_offset
    ///               - (cca ? macUnitBackoffPeriod : 0)
    ///               - IFS
    ///
    /// The receiver SHALL be disabled at this instant, even if the incoming
    /// frame has not ended, yet.
    ///
    /// Note that the RMARKER offset is PHY-dependent, e.g. rx_time(SHR) in case
    /// of O-QPSK.
    ///
    /// If the incoming frame ends before the given timeout then the receiver
    /// SHOULD be disabled as soon as the frame ends to optimize for energy
    /// efficiency.
    ///
    /// Note: It is the scheduler's task to cater for possible clock drift (i.e.
    ///       to widen the rx window) as appropriate.
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        at: OptionalNsInstant,
        ifs: Option<Ifs<PhyOf<RadioDriverImpl>>>,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskRx, TaskTx>;

    /// Ends the rx task and schedules a transition to the "Radio Off" state.
    ///
    /// If the task is timed, then the end of the rx task is precisely the given
    /// timestamp:
    ///
    ///   rx_task_end = at
    ///
    /// The receiver SHALL be disabled at this instant, even if the incoming
    /// frame has not ended, yet.
    ///
    /// If the incoming frame ends before the given timeout then the receiver
    /// SHOULD be disabled as soon as the frame ends to optimize for energy
    /// efficiency.
    ///
    /// See the trait documentation for an explanation of the
    /// `rollback_on_crcerror` flag.`
    fn schedule_off(
        self,
        at: OptionalNsInstant,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskRx, TaskOff>;
}

/// Generic characterization of the "Transmitter ON" (tx) state.
///
/// Drivers will occupy this state while sending a frame or after sending when
/// the transmitter is idle but the radio is still powered (tx idle).
pub trait TxState<RadioDriverImpl: DriverConfig>: RadioState<RadioDriverImpl, TaskTx> {
    /// Schedules a best-effort transition to the rx state.
    ///
    /// The driver SHALL deduce sufficient guard time from the IFS, so that -
    /// given the local radio's IFS timer accuracy - a frame can be received in
    /// time, even assuming max. clock drift.
    ///
    /// Note: If you need to start a timed rx task after a tx task, transition
    ///       to "Radio Off" state first. Keeping the radio in tx state while
    ///       waiting for an rx window would not be energy efficient.
    fn schedule_rx(
        self,
        task: TaskRx,
        ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> impl ExternalRadioTransition<RadioDriverImpl, TaskTx, TaskRx>;

    /// Schedules best-effort transmission of a frame back-to-back to the frame
    /// currently being sent.
    ///
    /// If the tx task's cca flag is set, then this transition will only be
    /// executed if the medium is idle, else switches to the "Radio Off" state.
    ///
    /// Note: The tx state undergoes several sub-states. We have to deal with
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
    /// schedule the next tx task early enough such that condition 3 holds. But
    /// for stability and correctness we SHALL deal with the other two
    /// (exceptional) cases, too. We'll emit a warning, though.
    ///
    /// Note: If you need to start a timed tx task after another tx task,
    ///       transition to "Radio Off" state first. Keeping the radio in tx
    ///       state while waiting for an rx window would not be energy
    ///       efficient.
    fn schedule_tx(
        self,
        tx_task: TaskTx,
        ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> impl SelfRadioTransition<RadioDriverImpl, TaskTx>;

    /// Schedules a transitions to the "Radio Off" state.
    ///
    /// Note: As tx will end deterministically, only best-effort scheduling is
    ///       allowed. Keeping the radio in tx state while waiting for an rx
    ///       window would not be energy efficient.
    fn schedule_off(self) -> impl ExternalRadioTransition<RadioDriverImpl, TaskTx, TaskOff>;
}

/// Represents an active radio state transition while it is being traversed.
pub trait RadioTransition<RadioDriverImpl: DriverConfig, ThisTask: RadioTask, NextTask: RadioTask>:
    Sized
{
    /// Provides mutable access to the source state.
    fn driver(&mut self) -> &mut RadioDriver<RadioDriverImpl, ThisTask>;

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
    ///
    /// In case of a timed task, returns the scheduled entry timestamp, see
    /// [`RadioState::transition`] for its semantics.
    fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError>;

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
    fn on_completed(&mut self) -> Result<(), SchedulingError>;

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
    fn cleanup() -> Result<(), RadioTaskError<NextTask>>;

    /// Tasks MAY produce distinct outcomes depending on external contingencies
    /// that are known only after the task has already been scheduled. Currently
    /// this is true for "CRC ok" (main outcome) vs. "CRC not ok" (alternate
    /// outcome). This flag determines wether the alternate outcome is treated
    /// as a [`RadioTask::Result`] or as a [`RadioTask::Error`].
    ///
    /// Note: Currently only the rx task's "CRC not ok" outcome uses this flag.
    fn alt_outcome_is_error(&self) -> bool;

    /// Consumes the transition to produce the corresponding source state and
    /// event (=next task) of the transition.
    fn consume(self) -> (RadioDriver<RadioDriverImpl, ThisTask>, NextTask);
}

/// Represents an active external radio state transition while it is being
/// traversed.
///
/// External transitions have distinct source and target states.
pub trait ExternalRadioTransition<
    RadioDriverImpl: DriverConfig,
    ThisTask: RadioTask,
    NextTask: RadioTask,
>: RadioTransition<RadioDriverImpl, ThisTask, NextTask>
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
        mut self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, ThisTask, NextTask>>
    where
        RadioDriver<RadioDriverImpl, ThisTask>: RadioState<RadioDriverImpl, ThisTask>,
        RadioDriver<RadioDriverImpl, NextTask>: RadioState<RadioDriverImpl, NextTask>,
        RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
    {
        async {
            let scheduled_entry = match self.on_scheduled() {
                Ok(scheduled_entry) => scheduled_entry,
                Err(scheduling_error) => {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::task_exec_end();

                    let (mut from_radio, next_task) = self.consume();
                    let task = from_radio.take_task();
                    return CompletedRadioTransition::Rollback(
                        from_radio,
                        RadioTaskError::Scheduling(task, scheduling_error),
                        None,
                        next_task,
                    );
                }
            };

            let alt_outcome_is_error = self.alt_outcome_is_error();
            let prev_task_result = match self.driver().completion(alt_outcome_is_error).await {
                Err(task_error) => {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::task_exec_end();

                    let _ = Self::cleanup();
                    let (from_radio, next_task) = self.consume();
                    return CompletedRadioTransition::Rollback(
                        from_radio, task_error, None, next_task,
                    );
                }
                Ok(prev_task_result) => prev_task_result,
            };

            if let Err(scheduling_error) = self.on_completed() {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end();

                let (mut from_radio, next_task) = self.consume();
                let task = from_radio.take_task();
                return CompletedRadioTransition::Rollback(
                    from_radio,
                    RadioTaskError::Scheduling(task, scheduling_error),
                    Some(prev_task_result),
                    next_task,
                );
            }

            if let Err(scheduling_error) = self.driver().exit() {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end();

                let _ = Self::cleanup();
                let (mut from_radio, next_task) = self.consume();
                let task = from_radio.take_task();
                return CompletedRadioTransition::Rollback(
                    from_radio,
                    RadioTaskError::Scheduling(task, scheduling_error),
                    Some(prev_task_result),
                    next_task,
                );
            }

            let (from_radio, next_task) = self.consume();
            let mut next_state = from_radio.enter_next_task(next_task);
            let next_state_entry = next_state.transition().await;

            let fallback =
                |next_task_error,
                 prev_task_result,
                 any_state: RadioDriver<RadioDriverImpl, NextTask>| async {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::task_exec_end();

                    let (mut off_state, entry) = any_state.switch_off().await;
                    off_state.set_measured_entry(entry);
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult::new(prev_task_result, off_state, entry, None.into()),
                        next_task_error,
                    )
                };

            if let Ok(entry_timestamp) = next_state_entry {
                next_state.set_measured_entry(entry_timestamp);
                if let Err(next_task_error) = next_state.entry() {
                    return fallback(next_task_error, prev_task_result, next_state).await;
                }
            };

            if let Err(next_task_error) = Self::cleanup() {
                return fallback(next_task_error, prev_task_result, next_state).await;
            }

            match next_state_entry {
                Ok(entry_timestamp) => {
                    CompletedRadioTransition::Entered(RadioTransitionResult::new(
                        prev_task_result,
                        next_state,
                        entry_timestamp,
                        scheduled_entry,
                    ))
                }
                Err(next_task_error) => {
                    fallback(next_task_error, prev_task_result, next_state).await
                }
            }
        }
    }
}

pub trait NotSame<ThisTask, NextTask> {}
impl NotSame<TaskOff, TaskRx> for () {}
impl NotSame<TaskOff, TaskTx> for () {}
impl NotSame<TaskRx, TaskTx> for () {}
impl NotSame<TaskRx, TaskOff> for () {}
impl NotSame<TaskTx, TaskRx> for () {}
impl NotSame<TaskTx, TaskOff> for () {}

impl<T, RadioDriverImpl: DriverConfig, ThisTask: RadioTask, NextTask: RadioTask>
    ExternalRadioTransition<RadioDriverImpl, ThisTask, NextTask> for T
where
    T: RadioTransition<RadioDriverImpl, ThisTask, NextTask>,
    (): NotSame<ThisTask, NextTask>,
{
}

/// Represents an active radio state self-transition while it is being
/// traversed.
///
/// Self transitions have the same source and target states. They are also
/// called internal transitions.
pub trait SelfRadioTransition<RadioDriverImpl: DriverConfig, Task: RadioTask>:
    RadioTransition<RadioDriverImpl, Task, Task>
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
    fn complete_and_transition(
        mut self,
    ) -> impl Future<Output = CompletedRadioTransition<RadioDriverImpl, Task, Task>>
    where
        RadioDriver<RadioDriverImpl, Task>: RadioState<RadioDriverImpl, Task>,
        RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
    {
        async {
            let scheduled_entry = match self.on_scheduled() {
                Ok(scheduled_entry) => scheduled_entry,
                Err(scheduling_error) => {
                    let (mut from_radio, next_task) = self.consume();
                    let task = from_radio.take_task();
                    return CompletedRadioTransition::Rollback(
                        from_radio,
                        RadioTaskError::Scheduling(task, scheduling_error),
                        None,
                        next_task,
                    );
                }
            };

            let alt_outcome_is_error = self.alt_outcome_is_error();
            let prev_task_result = match self.driver().completion(alt_outcome_is_error).await {
                Err(scheduling_error) => {
                    let _ = Self::cleanup();
                    let (from_radio, next_task) = self.consume();
                    return CompletedRadioTransition::Rollback(
                        from_radio,
                        scheduling_error,
                        None,
                        next_task,
                    );
                }
                Ok(prev_task_result) => prev_task_result,
            };

            if let Err(scheduling_error) = self.on_completed() {
                let (mut from_radio, next_task) = self.consume();
                let task = from_radio.take_task();
                return CompletedRadioTransition::Rollback(
                    from_radio,
                    RadioTaskError::Scheduling(task, scheduling_error),
                    Some(prev_task_result),
                    next_task,
                );
            }

            let (from_radio, next_task) = self.consume();
            let mut next_state = from_radio.enter_next_task(next_task);
            let next_state_entry = next_state.transition().await;

            let fallback =
                |next_task_error,
                 prev_task_result,
                 any_state: RadioDriver<RadioDriverImpl, Task>| async {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::task_exec_end();

                    let (mut off_state, entry) = any_state.switch_off().await;
                    off_state.set_measured_entry(entry);
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult::new(prev_task_result, off_state, entry, None.into()),
                        next_task_error,
                    )
                };

            if let Ok(entry_timestamp) = next_state_entry {
                next_state.set_measured_entry(entry_timestamp);
            }

            if let Err(next_task_error) = Self::cleanup() {
                return fallback(next_task_error, prev_task_result, next_state).await;
            }

            match next_state_entry {
                Ok(entry_timestamp) => {
                    CompletedRadioTransition::Entered(RadioTransitionResult::new(
                        prev_task_result,
                        next_state,
                        entry_timestamp,
                        scheduled_entry,
                    ))
                }
                Err(next_task_error) => {
                    fallback(next_task_error, prev_task_result, next_state).await
                }
            }
        }
    }
}

impl<T, RadioDriverImpl: DriverConfig, Task: RadioTask> SelfRadioTransition<RadioDriverImpl, Task>
    for T
where
    T: RadioTransition<RadioDriverImpl, Task, Task>,
    RadioDriver<RadioDriverImpl, Task>: RadioState<RadioDriverImpl, Task>,
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
{
}

/// Represents the result of a successful radio transition.
pub struct RadioTransitionResult<
    RadioDriverImpl: DriverConfig,
    PrevTask: RadioTask,
    ThisTask: RadioTask,
> {
    /// The result of the task that was completed by this transition.
    pub prev_task_result: PrevTask::Result,

    /// Represents the source state of this transition.
    prev_state: PhantomData<RadioDriver<RadioDriverImpl, PrevTask>>,

    /// The currently active radio state that was entered through this
    /// transition.
    pub this_state: RadioDriver<RadioDriverImpl, ThisTask>,

    /// The precise instant at which the currently active radio state was
    /// entered.
    pub measured_entry: NsInstant,

    /// The instant to which the current radio state was originally scheduled.
    /// Passed along for tuning and debugging purposes.
    pub scheduled_entry: OptionalNsInstant,
}

impl<RadioDriverImpl: DriverConfig, PrevTask: RadioTask, ThisTask: RadioTask>
    RadioTransitionResult<RadioDriverImpl, PrevTask, ThisTask>
{
    pub(crate) fn new(
        prev_task_result: PrevTask::Result,
        this_state: RadioDriver<RadioDriverImpl, ThisTask>,
        measured_entry: NsInstant,
        scheduled_entry: OptionalNsInstant,
    ) -> Self {
        Self {
            prev_task_result,
            prev_state: PhantomData,
            this_state,
            measured_entry,
            scheduled_entry,
        }
    }
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
