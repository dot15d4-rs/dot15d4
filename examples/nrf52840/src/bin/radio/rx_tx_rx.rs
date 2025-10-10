use dot15d4::{
    driver::{
        radio::{
            phy::Ifs,
            tasks::{
                CompletedRadioTransition::*, CompletingRxState, ExternalRadioTransition,
                ListeningRxState, OffState, RxResult, TaskOff, TaskRx, TaskTx, TxResult, TxState,
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
            log_timing("Off->Rx (BE)", &radio_transition_result, cca);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };
    let completing_rx_radio = match listening_rx_radio.stop_listening(None).await {
        Ok((_, completing_rx_radio)) => completing_rx_radio,
        Err(_) => unreachable!(),
    };

    // rx -> tx
    let tx_radio = match completing_rx_radio
        .schedule_tx(
            tx_task::<Config>(cca, buffer_allocator),
            None,
            Some(Ifs::short()),
            false,
        )
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Rx->Tx  (BE)", &radio_transition_result, cca);
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

    // tx -> rx
    let listening_rx_radio = match tx_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Tx->Rx  (BE)", &radio_transition_result, cca);
            match radio_transition_result.prev_task_result {
                TxResult::Sent(radio_frame, ..) => {
                    unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
                }
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };
    let completing_rx_radio = match listening_rx_radio.stop_listening(None).await {
        Ok((_, completing_rx_radio)) => completing_rx_radio,
        Err(_) => unreachable!(),
    };

    // rx -> off
    match completing_rx_radio
        .schedule_off(None, false)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Rx->Off (BE)", &radio_transition_result, cca);
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
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> (RadioDriver<Config, TaskOff>, LocalClockInstant)
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
            log_timing("Off->Rx (T)", &radio_transition_result, cca);
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    let completing_rx_radio = match listening_rx_radio
        .stop_listening(Some(rx_start + 5.millis()))
        .await
    {
        Ok((_, completing_rx_radio)) => completing_rx_radio,
        Err(_) => unreachable!(),
    };

    // rx -> tx
    let tx_at = rx_start + frame_period;
    let tx_radio = match completing_rx_radio
        .schedule_tx(
            tx_task::<Config>(cca, buffer_allocator),
            Some(tx_at),
            None,
            false,
        )
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Rx->Tx  (T)", &radio_transition_result, cca);
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

    // tx -> rx
    let (listening_rx_radio, rx_start) = match tx_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Tx->Rx  (T)", &radio_transition_result, cca);
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

    let completing_rx_radio = match listening_rx_radio
        .stop_listening(Some(rx_start + 5.millis()))
        .await
    {
        Ok((_, completing_rx_radio)) => completing_rx_radio,
        Err(_) => unreachable!(),
    };

    // rx -> off
    let off_at = rx_start + 10.millis();
    let off_radio = match completing_rx_radio
        .schedule_off(Some(off_at), false)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing("Rx->Off (T)", &radio_transition_result, cca);
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

    (off_radio, off_at)
}
