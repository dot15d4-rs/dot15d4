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
        timer::NsInstant,
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

use crate::{
    util::{allocate_test_slot, log_transition_result, tx_task},
    TestSuite,
};

pub async fn best_effort<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: NsInstant,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let test_suite = if cca {
        TestSuite::SingleBestEffortTxTxWithCca
    } else {
        TestSuite::SingleBestEffortTxTxWithoutCca
    };
    let _ = allocate_test_slot(timer, anchor_time, test_suite, 0, true).await;

    // off -> tx
    let mut tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), None.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Tx(BE)",
                test_suite,
                0,
                1,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> tx
    tx_radio = match tx_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Tx (BE)",
                test_suite,
                0,
                2,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> off
    match tx_radio.schedule_off().complete_and_transition().await {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Off(BE)",
                test_suite,
                0,
                3,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    }
}

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Test {
    OffToTxToTxToOff,
    NumSlots,
}

pub async fn timed<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: NsInstant,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskTx>: TxState<Config>,
{
    let test_suite = if cca {
        TestSuite::SingleTimedTxTxWithCca
    } else {
        TestSuite::SingleTimedTxTxWithoutCca
    };

    // off -> tx
    let tx_at = allocate_test_slot(
        timer,
        anchor_time,
        test_suite,
        Test::OffToTxToTxToOff as usize,
        false,
    )
    .await;
    let mut tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), tx_at.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Tx(T)",
                test_suite,
                Test::OffToTxToTxToOff as usize,
                1,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> tx
    tx_radio = match tx_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), Ifs::short())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Tx (T)",
                test_suite,
                Test::OffToTxToTxToOff as usize,
                2,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    // tx -> off
    let off_radio = match tx_radio.schedule_off().complete_and_transition().await {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Off(T)",
                test_suite,
                Test::OffToTxToTxToOff as usize,
                3,
                anchor_time,
                &radio_transition_result,
                cca,
            );
            let TxResult::Sent(radio_frame, ..) = radio_transition_result.prev_task_result;
            unsafe { buffer_allocator.deallocate_buffer(radio_frame.into_buffer()) };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    };

    off_radio
}
