use dot15d4::{
    driver::{
        radio::{DriverConfig, RadioDriver},
        tasks::{
            CompletedRadioTransition::*, ExternalRadioTransition, Ifs, OffState, RxResult, RxState,
            TaskOff, TaskRx, TaskTx, TxResult, TxState,
        },
        timer::{export::ExtU64, LocalClockInstant, RadioTimerApi},
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::util::{off_task, rx_task, tx_task};

pub async fn scenarios<Config: DriverConfig>(
    radio: RadioDriver<Config, TaskOff>,
    timer: Config::Timer,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: RxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let anchor_time = timer.now();

    // timed, no CCA
    let (radio, anchor_time) = timed(radio, anchor_time, false, buffer_allocator).await;

    // timed, CCA
    let (radio, _) = timed(radio, anchor_time, true, buffer_allocator).await;

    // best effort, no CCA
    let radio = best_effort(radio, false, buffer_allocator).await;

    // best effort, CCA
    best_effort(radio, true, buffer_allocator).await
}

async fn best_effort<Config: DriverConfig>(
    off_radio: RadioDriver<Config, TaskOff>,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: RxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    // Off -> Rx
    let rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(None, buffer_allocator))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => radio_transition_result.this_state,
        _ => unreachable!(),
    };

    // Rx -> Tx
    let tx_radio = match rx_radio
        .schedule_tx(
            tx_task::<Config>(None, cca, buffer_allocator),
            Ifs::Sifs,
            false,
        )
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            match radio_transition_result.prev_task_result {
                RxResult::RxWindowEnded(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // Tx -> Off
    match tx_radio
        .schedule_off(off_task(None))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            match radio_transition_result.prev_task_result {
                TxResult::Sent(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            }
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    }
}

async fn timed<Config: DriverConfig>(
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: LocalClockInstant,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> (RadioDriver<Config, TaskOff>, LocalClockInstant)
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: RxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let frame_period = 10.millis();

    // Off -> Rx
    let rx_start = anchor_time + frame_period;
    let rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(Some(rx_start), buffer_allocator))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => radio_transition_result.this_state,
        _ => unreachable!(),
    };

    // Rx -> Tx
    let tx_at = rx_start + frame_period;
    let tx_radio = match rx_radio
        .schedule_tx(
            tx_task::<Config>(Some(tx_at), cca, buffer_allocator),
            Ifs::None,
            false,
        )
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            match radio_transition_result.prev_task_result {
                RxResult::RxWindowEnded(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // Tx -> Off
    let off_radio = match tx_radio
        .schedule_off(off_task(None))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            match radio_transition_result.prev_task_result {
                TxResult::Sent(radio_frame) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
                _ => unreachable!(),
            }
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    (off_radio, tx_at)
}
