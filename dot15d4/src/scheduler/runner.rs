//! Async runner for scheduler service.
//!
//! This module provides the async execution environment for scheduler tasks.
//! It translates synchronous state machine transitions into actual async
//! operations, maintaining a clean separation between logic and execution.
//!
//! # Architecture
//!
//! The runner implements a simple loop:
//! 1. Task returns a transition with an action to execute
//! 2. Runner executes the async operation
//! 3. Runner delivers the resulting event back to the task
//! 4. Repeat

use dot15d4_driver::{radio::DriverConfig, timer::RadioTimerApi};
use dot15d4_util::sync::{select, ConsumerToken, Either};

use super::{
    MessageType, SchedulerAction, SchedulerContext, SchedulerTask, SchedulerTaskEvent,
    SchedulerTaskTransition,
};

/// Run a scheduler task in an infinite loop.
///
/// This is the main async entry point for executing scheduler tasks.
/// It continuously processes transitions from the task, executing the
/// requested actions and delivering events back to the task.
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
///
/// This function performs the actual async I/O operation requested by
/// the task and converts the result into an event for the task.
///
/// # Arguments
///
/// * `action` - The action to execute
/// * `task` - The scheduler task to delivr the event to
/// * `context` - Shared scheduler context
/// * `consumer_token` - Token for consuming channel requests
///
/// # Returns
///
/// The next transition from the task after processing the event.
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
