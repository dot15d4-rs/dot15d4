//! Top-level (root) Scheduler Task.
//!
//! This module implements the top-level scheduler task that manages
//! switching between different mode of operation (CSMA-CA and TSCH).
//!
//! # Architecture
//!
//! The [`RootSchedulerTask`] acts as a wrapper around the active scheduler
//! (either CSMA or TSCH). It delegates events to the active scheduler and
//! handles mode switching when a scheduler completes with a switch request.

use dot15d4_driver::radio::{config::Channel, DriverConfig};

#[cfg(feature = "tsch")]
use super::tsch::TschTask;
use super::{
    csma::CsmaTask, SchedulerContext, SchedulerTask, SchedulerTaskCompletion, SchedulerTaskEvent,
    SchedulerTaskTransition,
};

/// Enumeration of active scheduler implementations.
///
/// At any time, exactly one scheduler is active and processing events.
pub enum ActiveScheduler<RadioDriverImpl: DriverConfig> {
    /// CSMA-CA scheduler is active.
    Csma(CsmaTask<RadioDriverImpl>),
    /// TSCH scheduler is active (requires `tsch` feature).
    #[cfg(feature = "tsch")]
    Tsch(TschTask<RadioDriverImpl>),
}

/// Root scheduler task that manages scheduler switching.
///
/// This is the top-level task that wraps the actual scheduler implementations.
/// It handles mode switching requests and ensures proper initialization of
/// new schedulers when switching.
pub struct RootSchedulerTask<RadioDriverImpl: DriverConfig> {
    /// The currently active scheduler.
    pub inner_task: ActiveScheduler<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> RootSchedulerTask<RadioDriverImpl> {
    /// Create a new root scheduler task starting in CSMA mode.
    ///
    /// # Arguments
    ///
    /// * `initial_channel` - The PHY channel to operate on
    /// * `context` - Scheduler context for initialization
    pub fn new(initial_channel: Channel, context: &mut SchedulerContext<RadioDriverImpl>) -> Self {
        Self {
            inner_task: ActiveScheduler::Csma(CsmaTask::new(initial_channel, context)),
        }
    }
}

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl>
    for RootSchedulerTask<RadioDriverImpl>
{
    /// Process an event by delegating to the active scheduler.
    ///
    /// If the active scheduler completes with a switch request, this method
    /// handles creating the new scheduler and transitioning to it.
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        // Delegate to inner task
        let transition = match &mut self.inner_task {
            ActiveScheduler::Csma(csma_task) => csma_task.step(event, context),
            #[cfg(feature = "tsch")]
            ActiveScheduler::Tsch(tsch_task) => tsch_task.step(event, context),
        };

        // Handle scheduler switching
        match transition {
            SchedulerTaskTransition::Completed(completion_result, response) => {
                match completion_result {
                    SchedulerTaskCompletion::SwitchToCsma => {
                        // Get channel from current CSMA task or use default
                        let channel = match &self.inner_task {
                            ActiveScheduler::Csma(csma) => csma.channel,
                            #[cfg(feature = "tsch")]
                            // TODO: configurable default channel
                            ActiveScheduler::Tsch(_) => Channel::_12, // Default channel
                        };
                        self.inner_task = ActiveScheduler::Csma(CsmaTask::new(channel, context));
                        SchedulerTaskTransition::Completed(
                            SchedulerTaskCompletion::SwitchToCsma,
                            response,
                        )
                    }
                    #[cfg(feature = "tsch")]
                    SchedulerTaskCompletion::SwitchToTsch => {
                        self.inner_task = ActiveScheduler::Tsch(TschTask::new(context));
                        SchedulerTaskTransition::Completed(
                            SchedulerTaskCompletion::SwitchToTsch,
                            response,
                        )
                    }
                }
            }
            // Pass through all other transitions unchanged
            other => other,
        }
    }
}
