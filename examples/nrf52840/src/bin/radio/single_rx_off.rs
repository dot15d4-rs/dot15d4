use dot15d4::{
    driver::{
        radio::{
            tasks::{
                CompletedRadioTransition::*, ExternalRadioTransition, ListeningRxState, OffState,
                RxResult, StopListeningResult, TaskOff, TaskRx, TaskTx, TxState,
            },
            DriverConfig, RadioDriver,
        },
        timer::NsInstant,
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::{
    util::{allocate_test_slot, log_transition_result, rx_task},
    TestSuite,
};

pub async fn best_effort<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: NsInstant,
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
        .schedule_rx(rx_task::<Config>(buffer_allocator), None.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Rx(BE)",
                TestSuite::SingleBestEffortRxOff,
                0,
                1,
                anchor_time,
                &radio_transition_result,
                false,
            );
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // rx -> rx window ended
    match listening_rx_radio.stop_listening(None.into()).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_transition_result(
                "Rx->End(BE)",
                TestSuite::SingleBestEffortRxOff,
                0,
                2,
                anchor_time,
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
    anchor_time: NsInstant,
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
        .schedule_rx(rx_task::<Config>(buffer_allocator), rx_start.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Rx(T)",
                TestSuite::SingleTimedRxOff,
                Test::OffToRx as usize,
                1,
                anchor_time,
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
    match listening_rx_radio.stop_listening(off_at.into()).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_transition_result(
                "Rx->End(T)",
                TestSuite::SingleTimedRxOff,
                Test::RxToRxWindowEnd as usize,
                1,
                anchor_time,
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
