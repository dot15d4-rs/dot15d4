use dot15d4::{
    driver::{
        radio::{
            phy::Ifs,
            tasks::{
                CompletedRadioTransition::*, ExternalRadioTransition, OffState,
                SelfRadioTransition, TaskOff, TaskTx, TxResult, TxState,
            },
            DriverConfig, RadioDriver,
        },
        timer::{export::ExtU64, LocalClockInstant, RadioTimerApi},
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::util::tx_task;

pub async fn scenarios<Config: DriverConfig>(
    radio: RadioDriver<Config, TaskOff>,
    timer: Config::Timer,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let anchor_time = timer.now();

    // timed, no CCA
    let (radio, anchor_time) = timed(radio, anchor_time, false, buffer_allocator).await;

    // timed, CCA
    let (radio, _) = timed(radio, anchor_time, true, buffer_allocator).await;

    // best effort, no CCA, SIFS
    let radio = best_effort(radio, false, buffer_allocator).await;

    // best effort, CCA, SIFS
    best_effort(radio, true, buffer_allocator).await

    // TODO: Test LIFS
}

async fn best_effort<Config: DriverConfig>(
    off_radio: RadioDriver<Config, TaskOff>,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    // off -> tx
    let mut tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), None)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => radio_transition_result.this_state,
        _ => unreachable!(),
    };

    // tx -> tx
    tx_radio = match tx_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> off
    match tx_radio.schedule_off().complete_and_transition().await {
        Entered(radio_transition_result) => {
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
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
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let frame_period = 10.millis();

    // off -> tx
    let tx_at = anchor_time + frame_period;
    let mut tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), Some(tx_at))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => radio_transition_result.this_state,
        _ => unreachable!(),
    };

    // tx -> tx
    tx_radio = match tx_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> off
    let off_radio = match tx_radio.schedule_off().complete_and_transition().await {
        Entered(radio_transition_result) => {
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    (off_radio, tx_at)
}
