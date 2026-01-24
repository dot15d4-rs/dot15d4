//! Shared scheduler service state types.

use dot15d4_driver::radio::{config::Channel, DriverConfig};

use super::task::{
    SchedulerTask, SchedulerTaskCompletion, SchedulerTaskEvent, SchedulerTaskTransition,
};
#[cfg(feature = "tsch")]
use super::tsch::TschTask;
use super::{csma::CsmaTask, SchedulerContext};

/// Active scheduler type.
pub enum ActiveScheduler<RadioDriverImpl: DriverConfig> {
    /// Using CSMA-CA
    Csma(CsmaTask<RadioDriverImpl>),
    /// Using TSCH
    #[cfg(feature = "tsch")]
    Tsch(TschTask<RadioDriverImpl>),
}

/// Complete scheduler service state.
pub struct RootSchedulerTask<RadioDriverImpl: DriverConfig> {
    /// Which scheduler is currently active.
    pub inner_task: ActiveScheduler<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> RootSchedulerTask<RadioDriverImpl> {
    /// Create new scheduler service state starting with CSMA.
    pub fn new(initial_channel: Channel, context: &mut SchedulerContext<RadioDriverImpl>) -> Self {
        Self {
            inner_task: ActiveScheduler::Csma(CsmaTask::new(initial_channel, context)),
        }
    }
}

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl>
    for RootSchedulerTask<RadioDriverImpl>
{
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
