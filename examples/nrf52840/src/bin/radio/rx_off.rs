use dot15d4::{
    driver::{
        radio::{
            tasks::{
                CompletedRadioTransition::*, ExternalRadioTransition, ListeningRxState, OffState,
                RxResult, StopListeningResult, TaskOff, TaskRx, TaskTx, TxState,
            },
            DriverConfig, RadioDriver,
        },
        timer::LocalClockInstant,
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::{
    util::{allocate_test_slot, log_timing, rx_task},
    TestSuite,
};

pub async fn best_effort<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: LocalClockInstant,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let _ = allocate_test_slot(
        timer,
        anchor_time,
        TestSuite::SingleBestEffortRxOff,
        0,
        true,
    )
    .await;

    // off -> rx
    let listening_rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), None)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing(
                "Off->Rx(BE)",
                anchor_time,
                TestSuite::SingleBestEffortRxOff,
                0,
                1,
                &radio_transition_result,
                false,
            );
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // rx -> rx window ended
    match listening_rx_radio.stop_listening(None).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_timing(
                "Rx->End(BE)",
                anchor_time,
                TestSuite::SingleBestEffortRxOff,
                0,
                2,
                &radio_transition_result,
                false,
            );
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

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Test {
    OffToRx,
    RxToRxWindowEnd,
    NumSlots,
}

pub async fn timed<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: LocalClockInstant,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    // off -> rx
    let rx_start = allocate_test_slot(
        timer,
        anchor_time,
        TestSuite::SingleTimedRxOff,
        Test::OffToRx as usize,
        false,
    )
    .await;
    let listening_rx_radio = match off_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Some(rx_start))
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_timing(
                "Off->Rx(T)",
                anchor_time,
                TestSuite::SingleTimedRxOff,
                Test::OffToRx as usize,
                1,
                &radio_transition_result,
                false,
            );
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // rx -> rx window ended
    let off_at = allocate_test_slot(
        timer,
        anchor_time,
        TestSuite::SingleTimedRxOff,
        Test::RxToRxWindowEnd as usize,
        false,
    )
    .await;
    match listening_rx_radio.stop_listening(Some(off_at)).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_timing(
                "Rx->End(T)",
                anchor_time,
                TestSuite::SingleTimedRxOff,
                Test::RxToRxWindowEnd as usize,
                1,
                &radio_transition_result,
                false,
            );
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
