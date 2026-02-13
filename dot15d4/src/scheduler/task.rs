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
use super::scan::ScanTask;
#[cfg(feature = "tsch")]
use super::tsch::TschTask;
use super::{
    csma::CsmaTask, SchedulerContext, SchedulerTask, SchedulerTaskCompletion, SchedulerTaskEvent,
    SchedulerTaskTransition,
};

/// Root scheduler state machine states.
pub enum RootSchedulerState<RadioDriverImpl: DriverConfig> {
    /// CSMA-CA scheduler is active.
    UsingCsma(CsmaTask<RadioDriverImpl>),
    /// TSCH scheduler is active (requires `tsch` feature).
    #[cfg(feature = "tsch")]
    UsingTsch(TschTask<RadioDriverImpl>),
    #[cfg(feature = "tsch")]
    ScanningChannels(ScanTask<RadioDriverImpl>),
}

/// Root scheduler task that manages scheduler switching.
///
/// This is the top-level task that wraps the actual scheduler implementations.
/// It handles mode switching requests and ensures proper initialization of
/// new schedulers when switching.
pub struct RootSchedulerTask<RadioDriverImpl: DriverConfig> {
    /// Current scheduler state
    pub state: RootSchedulerState<RadioDriverImpl>,
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
            state: RootSchedulerState::UsingCsma(CsmaTask::new(initial_channel, context)),
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
        let transition = match &mut self.state {
            RootSchedulerState::UsingCsma(csma_task) => csma_task.step(event, context),
            #[cfg(feature = "tsch")]
            RootSchedulerState::UsingTsch(tsch_task) => tsch_task.step(event, context),
            #[cfg(feature = "tsch")]
            RootSchedulerState::ScanningChannels(scan_task) => scan_task.step(event, context),
        };

        // Handle scheduler switching
        match transition {
            SchedulerTaskTransition::Completed(completion_result, response) => {
                match completion_result {
                    SchedulerTaskCompletion::SwitchToCsma => {
                        // Get channel from current CSMA task or use default
                        let channel = match &self.state {
                            RootSchedulerState::UsingCsma(csma) => csma.channel,
                            #[cfg(feature = "tsch")]
                            // TODO: configurable default channel
                            _ => Channel::_26, // Default channel
                        };
                        self.state = RootSchedulerState::UsingCsma(CsmaTask::new(channel, context));
                        SchedulerTaskTransition::Completed(
                            SchedulerTaskCompletion::SwitchToCsma,
                            response,
                        )
                    }
                    #[cfg(feature = "tsch")]
                    SchedulerTaskCompletion::SwitchToTsch => {
                        self.state = RootSchedulerState::UsingTsch(TschTask::new(context));
                        SchedulerTaskTransition::Completed(
                            SchedulerTaskCompletion::SwitchToTsch,
                            response,
                        )
                    }
                    #[cfg(feature = "tsch")]
                    SchedulerTaskCompletion::SwitchToScanning(channels, max_pan_descriptors) => {
                        self.state = RootSchedulerState::ScanningChannels(ScanTask::new(
                            channels,
                            max_pan_descriptors,
                        ));
                        SchedulerTaskTransition::Completed(
                            SchedulerTaskCompletion::SwitchToScanning(
                                channels,
                                max_pan_descriptors,
                            ),
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
