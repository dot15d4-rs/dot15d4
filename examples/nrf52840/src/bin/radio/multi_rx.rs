#[cfg(feature = "device-sync-client")]
use dot15d4::driver::radio::tasks::{TaskTx, TxResult, TxState};
#[cfg(not(feature = "device-sync-client"))]
use dot15d4::{
    driver::radio::{
        phy::PhyConfig,
        tasks::{ListeningRxState, ReceivingRxState, RxResult, StopListeningResult, TaskRx},
        PhyOf,
    },
    mac::frame::mpdu::MpduFrame,
};
use dot15d4::{
    driver::{
        radio::{
            tasks::{CompletedRadioTransition::*, ExternalRadioTransition, OffState, TaskOff},
            DriverConfig, RadioDriver,
        },
        timer::{NsDuration, NsInstant},
    },
    util::allocator::{BufferAllocator, IntoBuffer},
};

#[cfg(feature = "device-sync-client")]
use crate::util::tx_task;
#[cfg(not(feature = "device-sync-client"))]
use crate::util::{log_test_result, rx_task, TestResult, PAYLOAD};
use crate::{
    util::{allocate_test_slot, log_transition_result},
    TestSuite,
};

const RX_WINDOW_DURATION: NsDuration = NsDuration::micros(10);

#[cfg(not(feature = "device-sync-client"))]
pub async fn server<Config: DriverConfig>(
    timer: &mut Config::Timer,
    off_radio: RadioDriver<Config, TaskOff>,
    anchor_time: NsInstant,
    cca: bool,
    buffer_allocator: BufferAllocator,
) -> RadioDriver<Config, TaskOff>
where
    RadioDriver<Config, TaskOff>: OffState<Config>,
    RadioDriver<Config, TaskRx>: ListeningRxState<Config>,
{
    // off -> rx
    let mut earliest_frame_start =
        allocate_test_slot(timer, anchor_time, TestSuite::MultiTimedRx, 0, false).await;
    if cca {
        earliest_frame_start += <PhyOf<Config> as PhyConfig>::A_TURNAROUND_TIME;
    }
    let listening_rx_radio = match off_radio
        .schedule_rx(
            rx_task::<Config>(buffer_allocator),
            earliest_frame_start.into(),
        )
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Rx(M)",
                TestSuite::MultiTimedRx,
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

    // rx -> rx frame reception

    let latest_frame_start = earliest_frame_start + RX_WINDOW_DURATION;
    let receiving_rx_radio = match listening_rx_radio
        .stop_listening(latest_frame_start.into())
        .await
    {
        Ok(StopListeningResult::FrameStarted(measured_rx_start, receiving_rx_radio)) => {
            let test_result = TestResult {
                label: "Rx->Rcv(M)",
                test_suite: TestSuite::MultiTimedRx,
                test_slot: 0,
                test_step: 2,
                anchor_time,
                expected_timestamp: latest_frame_start.into(),
                measured_timestamp: measured_rx_start,
                cca,
            };
            log_test_result::<Config>(test_result);

            receiving_rx_radio
        }
        _ => panic!("Expected frame not received."),
    };

    match receiving_rx_radio
        .schedule_off(None.into(), false)
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Rcv->Off(M)",
                TestSuite::MultiTimedRx,
                0,
                3,
                anchor_time,
                &radio_transition_result,
                false,
            );
            match radio_transition_result.prev_task_result {
                RxResult::Frame(radio_frame, ..) => {
                    let mpdu = MpduFrame::from_radio_frame(radio_frame);

                    let mpdu_reader = mpdu.reader();
                    assert!(mpdu_reader.is_valid());

                    let mpdu_reader = mpdu_reader
                        .parse_addressing()
                        .unwrap()
                        .parse_security()
                        .parse_ies::<Config>()
                        .unwrap();

                    let payload = mpdu_reader.try_frame_payload().unwrap();
                    assert_eq!(payload, PAYLOAD);

                    unsafe { buffer_allocator.deallocate_buffer(mpdu.into_buffer()) };
                }
                _ => unreachable!(),
            };
            radio_transition_result.this_state
        }
        _ => unreachable!(),
    }
}

#[cfg(feature = "device-sync-client")]
pub async fn client<Config: DriverConfig>(
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
    // off -> tx
    let mut tx_at = allocate_test_slot(timer, anchor_time, TestSuite::MultiTimedRx, 0, false).await;
    tx_at += RX_WINDOW_DURATION / 2;
    let tx_radio = match off_radio
        .schedule_tx(tx_task::<Config>(cca, buffer_allocator), tx_at.into())
        .complete_and_transition()
        .await
    {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Off->Tx(M)",
                TestSuite::MultiTimedRx,
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

    // tx -> off
    match tx_radio.schedule_off().complete_and_transition().await {
        Entered(radio_transition_result) => {
            log_transition_result(
                "Tx->Off(M)",
                TestSuite::MultiTimedRx,
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
    }
}
