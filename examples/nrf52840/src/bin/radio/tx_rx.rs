use dot15d4::{
    driver::{
        radio::{
            phy::Ifs,
            tasks::{
                CompletedRadioTransition::*, ExternalRadioTransition, ListeningRxState, OffState,
                RxResult, StopListeningResult, TaskOff, TaskRx, TaskTx, TxResult, TxState,
            },
            DriverConfig, RadioDriver,
        },
        timer::{export::ExtU64, LocalClockInstant, RadioTimerApi},
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::util::{log_timing, rx_task, tx_task};

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
    // off -> tx
    let tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(false, buffer_allocator), None)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Off->Tx(BE)", &radio_transition_result, false);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> rx
    let listening_rx_radio = match tx_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Tx->Rx (BE)", &radio_transition_result, false);
            match radio_transition_result.prev_task_result {
                TxResult::Sent(radio_frame, ..) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
            };
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

    // off -> tx
    let tx_at = anchor_time + frame_period;
    let tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(false, buffer_allocator), Some(tx_at))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Off->Tx(T )", &radio_transition_result, false);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> rx
    let (listening_rx_radio, rx_start) = match tx_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Tx->Rx (T )", &radio_transition_result, false);
            match radio_transition_result.prev_task_result {
                TxResult::Sent(radio_frame, ..) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
            };
            (
                radio_transition_result.this_state,
                radio_transition_result.measured_entry,
            )
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
