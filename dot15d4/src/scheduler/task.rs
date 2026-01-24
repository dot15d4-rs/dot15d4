//! Scheduler task trait and events.
//!
//! This module defines the core trait for scheduler tasks (CSMA, TSCH) and
//! the events/transitions they handle.

use dot15d4_driver::radio::DriverConfig;
use dot15d4_util::sync::ResponseToken;

use crate::driver::DrvSvcEvent;
use crate::scheduler::SchedulerRequest;

use super::action::SchedulerAction;
use super::{SchedulerContext, SchedulerResponse};

pub enum SchedulerTaskCompletion {
    SwitchToCsma,
    #[cfg(feature = "tsch")]
    SwitchToTsch,
}

/// Trait for scheduler tasks.
///
/// Tasks are pure state machines that receive events and produce transitions.
pub trait SchedulerTask<RadioDriverImpl: DriverConfig> {
    /// Process an event and return the next transition.
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition;
}

/// Events that can be delivered to a scheduler task.
pub enum SchedulerTaskEvent {
    /// Task is being entered (initial entry).
    Entry,
    /// A driver event was received.
    DriverEvent(DrvSvcEvent),
    /// A scheduler request was received from MAC layer.
    SchedulerRequest {
        token: ResponseToken,
        request: SchedulerRequest,
    },
    /// Timer expired (e.g. for TSCH slot timing).
    #[cfg(feature = "tsch")]
    TimerExpired,
}

/// Transitions returned by scheduler tasks.
pub enum SchedulerTaskTransition {
    /// Execute an action and optionally send a response.
    Execute(
        /// The action for the runner to execute.
        SchedulerAction,
        /// An optional response to send immediately.
        Option<(ResponseToken, SchedulerResponse)>,
    ),

    /// Task cycle completed
    Completed(
        SchedulerTaskCompletion,
        Option<(ResponseToken, SchedulerResponse)>,
    ),
}
