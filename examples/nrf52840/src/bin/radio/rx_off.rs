use dot15d4::{
    driver::{
        radio::{
            tasks::{
                CompletedRadioTransition::*, ExternalRadioTransition, ListeningRxState, OffState,
                RxResult, StopListeningResult, TaskOff, TaskRx, TaskTx, TxState,
            },
            DriverConfig, RadioDriver,
        },
        timer::{export::ExtU64, LocalClockInstant, RadioTimerApi},
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::util::{log_timing, rx_task};

pub async fn scenarios<Config: DriverConfig>(
    radio: RadioDriver<Config, TaskOff>,
    timer: Config::Timer,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let anchor_time = timer.now();

    // timed
    let radio = timed(radio, anchor_time, buffer_allocator).await;

    // best effort
    best_effort(radio, buffer_allocator).await
}

async fn best_effort<Config: DriverConfig>(
    off_radio: RadioDriver<Config, TaskOff>,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    // off -> rx
    let listening_rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), None)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Off->Rx(BE)", &radio_transition_result, false);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // rx -> rx window ended
    match listening_rx_radio.stop_listening(None).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_timing("Rx->End(BE)", &radio_transition_result, false);
            match radio_transition_result.prev_task_result {
                RxResult::RxWindowEnded(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    }
}

async fn timed<Config: DriverConfig>(
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: LocalClockInstant,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let frame_period = 10.millis();

    // off -> rx
    let rx_start = anchor_time + frame_period;
    let listening_rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Some(rx_start))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Off->Rx(T )", &radio_transition_result, false);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // rx -> rx window ended
    let off_at = rx_start + frame_period;
    match listening_rx_radio.stop_listening(Some(off_at)).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_timing("Rx->End(T )", &radio_transition_result, false);
            match radio_transition_result.prev_task_result {
                RxResult::RxWindowEnded(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    }
}
