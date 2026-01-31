//! Async runner for scheduler service.
//!
//! This is the async wrapper that executes I/O operations based on
//! actions returned by the sync logic.

use dot15d4_driver::{radio::DriverConfig, timer::RadioTimerApi};
use dot15d4_util::sync::{select, ConsumerToken, Either};

use super::{
    MessageType, SchedulerAction, SchedulerContext, SchedulerTask, SchedulerTaskEvent,
    SchedulerTaskTransition,
};

/// Run a scheduler task loop.
pub async fn run_task<'a, RadioDriverImpl, Task>(
    task: &mut Task,
    context: &mut SchedulerContext<'a, RadioDriverImpl>,
    consumer_token: &mut ConsumerToken,
) -> !
where
    RadioDriverImpl: DriverConfig,
    RadioDriverImpl::Timer: RadioTimerApi,
    Task: SchedulerTask<RadioDriverImpl>,
{
    let mut transition = task.step(SchedulerTaskEvent::Entry, context);

    loop {
        transition = match transition {
            SchedulerTaskTransition::Execute(action, response) => {
                // Handle any response first
                if let Some((token, response)) = response {
                    context.request_receiver.received(token, response);
                }

                // Execute the action and get next event
                execute_action(action, task, context, consumer_token).await
            }
            SchedulerTaskTransition::Completed(_result, response) => {
                // TODO: process result ? A priori, no since run_task is used with Root Scheduler
                // which already handles the result
                if let Some((token, response)) = response {
                    context.request_receiver.received(token, response);
                }
                // Break inner loop to re-enter task
                task.step(SchedulerTaskEvent::Entry, context)
            }
        }
    }
}

/// Execute a single scheduler action and return the next transition.
async fn execute_action<'a, RadioDriverImpl, Task>(
    action: SchedulerAction,
    task: &mut Task,
    context: &mut SchedulerContext<'a, RadioDriverImpl>,
    consumer_token: &mut ConsumerToken,
) -> SchedulerTaskTransition
where
    RadioDriverImpl: DriverConfig,
    RadioDriverImpl::Timer: RadioTimerApi,
    Task: SchedulerTask<RadioDriverImpl>,
{
    match action {
        SchedulerAction::SendDriverRequestThenWait(req) => {
            context.driver_request_sender.send(req).await;
            let event = context.driver_event_receiver.receive().await;
            task.step(SchedulerTaskEvent::DriverEvent(event), context)
        }
        SchedulerAction::SendDriverRequestThenSelect(req) => {
            context.driver_request_sender.send(req).await;
            // After sending the request, select on driver event OR scheduler request
            match select::select(
                context.driver_event_receiver.receive(),
                context
                    .request_receiver
                    .receive_request_async(consumer_token, &MessageType::TxOrCommand),
            )
            .await
            {
                Either::First(event) => task.step(SchedulerTaskEvent::DriverEvent(event), context),
                Either::Second((token, request)) => task.step(
                    SchedulerTaskEvent::SchedulerRequest { token, request },
                    context,
                ),
            }
        }
        SchedulerAction::WaitForDriverEvent => {
            let event = context.driver_event_receiver.receive().await;
            task.step(SchedulerTaskEvent::DriverEvent(event), context)
        }
        SchedulerAction::WaitForSchedulerRequest => {
            let (token, request) = context
                .request_receiver
                .receive_request_async(consumer_token, &MessageType::TxOrCommand)
                .await;
            task.step(
                SchedulerTaskEvent::SchedulerRequest { token, request },
                context,
            )
        }
        SchedulerAction::SelectDriverEventOrRequest => {
            match select::select(
                context.driver_event_receiver.receive(),
                context
                    .request_receiver
                    .receive_request_async(consumer_token, &MessageType::TxOrCommand),
            )
            .await
            {
                Either::First(event) => task.step(SchedulerTaskEvent::DriverEvent(event), context),
                Either::Second((token, request)) => task.step(
                    SchedulerTaskEvent::SchedulerRequest { token, request },
                    context,
                ),
            }
        }
        #[cfg(feature = "tsch")]
        SchedulerAction::WaitForTimeoutOrSchedulerRequest { deadline } => {
            match select::select(
                // Safety: timer API requires unsafe for wait_until
                unsafe { context.timer.wait_until(deadline) },
                context
                    .request_receiver
                    .receive_request_async(consumer_token, &MessageType::TxOrCommand),
            )
            .await
            {
                Either::First(_) => {
                    // Timer expired
                    task.step(SchedulerTaskEvent::TimerExpired, context)
                }
                Either::Second((token, request)) => task.step(
                    SchedulerTaskEvent::SchedulerRequest { token, request },
                    context,
                ),
            }
        }
    }
}
