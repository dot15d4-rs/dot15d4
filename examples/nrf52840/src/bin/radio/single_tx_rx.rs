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
        timer::NsInstant,
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::{
    util::{allocate_test_slot, log_transition_result, rx_task, tx_task},
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
    let _ = allocate_test_slot(timer, anchor_time, TestSuite::SingleBestEffortTxRx, 0, true).await;

    // off -> tx
    let tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(false, buffer_allocator), None.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Tx(BE)",
                TestSuite::SingleBestEffortTxRx,
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

    // tx -> rx
    let listening_rx_radio = match tx_radio
        .schedule_rx(rx_task::<Config>(buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Rx (BE)",
                TestSuite::SingleBestEffortTxRx,
                0,
                2,
                anchor_time,
                &radio_transition_result,
                false,
            );
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
    match listening_rx_radio.stop_listening(None.into()).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_transition_result(
                "Rx->End(BE)",
                TestSuite::SingleBestEffortTxRx,
                0,
                3,
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
    OffToTxToRx,
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
    // off -> tx
    let tx_at = allocate_test_slot(
        timer,
        anchor_time,
        TestSuite::SingleTimedTxRx,
        Test::OffToTxToRx as usize,
        false,
    )
    .await;
    let tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(false, buffer_allocator), tx_at.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Tx(T)",
                TestSuite::SingleTimedTxRx,
                Test::OffToTxToRx as usize,
                1,
                anchor_time,
                &radio_transition_result,
                false,
            );
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
            log_transition_result(
                "Tx->Rx (T)",
                TestSuite::SingleTimedTxRx,
                Test::OffToTxToRx as usize,
                2,
                anchor_time,
                &radio_transition_result,
                false,
            );
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
    let off_at = allocate_test_slot(
        timer,
        anchor_time,
        TestSuite::SingleTimedTxRx,
        Test::RxToRxWindowEnd as usize,
        false,
    )
    .await;
    match listening_rx_radio.stop_listening(off_at.into()).await {
        Ok(StopListeningResult::RxWindowEnded(radio_transition_result)) => {
            log_transition_result(
                "Rx->End(T)",
                TestSuite::SingleTimedTxRx,
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
