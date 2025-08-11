use crate::driver::{DrvSvcRequest, DrvSvcResponse};

/// A MAC task represents a - possibly infinite - stream of driver
/// request/response exchanges each of which MAY time out.
///
/// Alternatively a MAC task can be conceived as a state machine that steps one
/// driver task at a time, see the [`MacTaskTransition::DrvSvcRequest`] and
/// [`MacTaskEvent::DrvSvcResponse`] pair. A transition can be ended by a
/// [`MacTaskEvent::Timeout`] in case a response is not received in time from
/// the driver service.
///
/// The task is instantiated, entered (see [`MacTaskEvent::Entry`]) and driven
/// by the MAC service in its role as a MAC task executor.
pub trait MacTask {
    /// A task MAY produce intermediate and final results while being executed.
    type Result;

    /// This method must be called by the task executor (i.e. the MAC service)
    /// whenever the task becomes pending. It drives the task state machine
    /// until it terminates (see [`MacTaskTransition::Terminated`]).
    ///
    /// A task becomes pending when one of the following events occurs:
    /// - [`MacTaskEvent::Entry`]: The task has just been initialized.
    /// - [`MacTaskEvent::DrvSvcResponse`]: The driver service returned a
    ///   response to a pending request from the state machine.
    fn step(self, event: MacTaskEvent) -> MacTaskTransition<Self>
    where
        Self: Sized;
}

/// The set of events that may occur while executing a MAC task state machine.
pub enum MacTaskEvent {
    /// Event produced once by the executor immediately after a (sub-)state
    /// machine has been instantiated. Takes the state machine from the initial
    /// pseudostate to its initial state.
    Entry,

    /// The driver service has produced a response to the driver service request
    /// previously produced by the state machine's request.
    DrvSvcResponse(DrvSvcResponse),
}

/// Represents the transition triggered by a MAC task step.
///
/// A transition produces a driver service request and additionally intermediate
/// and/or final task results.
///
/// A transition MAY block if it cannot executed immediately because it requires
/// resources blocked by another state machine.
///
/// A transition MAY time out.
pub enum MacTaskTransition<Task: MacTask> {
    /// This result signals to the executor that a transition has been
    /// triggered, i.e. a driver service request needs to be sent to the driver
    /// service.
    ///
    /// A transition MAY yield an intermediate result to be handled by the
    /// executor.
    ///
    /// A transition MAY time out.
    DrvSvcRequest(
        /// The task's next state.
        Task,
        /// The driver service request produced by the transition.
        DrvSvcRequest,
        /// An optional intermediate task result.
        Option<Task::Result>,
    ),

    /// Signals to the executor that the state machine exited, possibly with a
    /// final result.
    Terminated(Task::Result),
}
