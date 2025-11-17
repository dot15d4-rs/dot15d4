//! Low level radio driver tests.
//!
//! Also shows how the interrupt executor can be used stand-alone in a plain
//! cortex-m project.

#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

#[cfg(feature = "device-sync")]
mod multi_rx;

#[cfg(not(feature = "device-sync-client"))]
mod single_rx_off;
#[cfg(not(feature = "device-sync-client"))]
mod single_tx_rx;
#[cfg(not(feature = "device-sync-client"))]
mod single_tx_tx;

mod util;

#[cfg(not(feature = "device-sync"))]
use dot15d4::driver::timer::RadioTimerApi;
use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        radio::{
            phy::{OQpsk250KBit, Phy, PhyConfig},
            RadioDriver,
        },
        timer::NsDuration,
    },
    util::{buffer_allocator, info},
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor, AvailableResources};
#[cfg(feature = "device-sync")]
use dot15d4_examples_nrf52840::{gpiote_executor, observe_gpio_event, wait_for_gpio_event};

use self::util::done;

const TEST_SLOT_DURATION_MS: usize = 10;
const TEST_SLOT_DURATION: NsDuration = NsDuration::millis(TEST_SLOT_DURATION_MS as u64);

#[cfg(not(feature = "device-sync-client"))]
use single_rx_off::Test as RxOffTest;
#[cfg(not(feature = "device-sync-client"))]
use single_tx_rx::Test as TxRxTest;
#[cfg(not(feature = "device-sync-client"))]
use single_tx_tx::Test as TxTxTest;

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TestSuite {
    #[cfg(feature = "device-sync")]
    MultiTimedRx,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedRxOff,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxRx,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxTxWithoutCca,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxTxWithCca,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortRxOff,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxRx,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxTxWithoutCca,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxTxWithCca,
}

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TestSuiteSlot {
    // We place multi-device tests at the beginning to reduce the probability of
    // synchronization errors due to missed clock ticks.
    #[cfg(feature = "device-sync")]
    MultiTimedRx,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedRxOff,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxRx = Self::SingleTimedRxOff as usize + RxOffTest::NumSlots as usize,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxTxWithoutCca = Self::SingleTimedTxRx as usize + TxRxTest::NumSlots as usize,
    #[cfg(not(feature = "device-sync-client"))]
    SingleTimedTxTxWithCca = Self::SingleTimedTxTxWithoutCca as usize + TxTxTest::NumSlots as usize,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortRxOff,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxRx,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxTxWithoutCca,
    #[cfg(not(feature = "device-sync-client"))]
    SingleBestEffortTxTxWithCca,
}

impl TestSuite {
    pub fn slot(&self) -> usize {
        (match self {
            #[cfg(feature = "device-sync")]
            TestSuite::MultiTimedRx => TestSuiteSlot::MultiTimedRx,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleTimedRxOff => TestSuiteSlot::SingleTimedRxOff,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleTimedTxRx => TestSuiteSlot::SingleTimedTxRx,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleTimedTxTxWithoutCca => TestSuiteSlot::SingleTimedTxTxWithoutCca,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleTimedTxTxWithCca => TestSuiteSlot::SingleTimedTxTxWithCca,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleBestEffortRxOff => TestSuiteSlot::SingleBestEffortRxOff,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleBestEffortTxRx => TestSuiteSlot::SingleBestEffortTxRx,
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleBestEffortTxTxWithoutCca => {
                TestSuiteSlot::SingleBestEffortTxTxWithoutCca
            }
            #[cfg(not(feature = "device-sync-client"))]
            TestSuite::SingleBestEffortTxTxWithCca => TestSuiteSlot::SingleBestEffortTxTxWithCca,
        }) as usize
            + 1
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let AvailableResources {
        radio,
        #[cfg(feature = "device-sync")]
        gpiote,
        mut timer,
        ..
    } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    #[cfg(feature = "executor-trace")]
    let executor_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        radio,
        timer,
        #[cfg(feature = "executor-trace")]
        executor_trace_channel,
        #[cfg(feature = "radio-trace")]
        radio_tracing_config(),
    );
    let swi_executor = swi_executor();
    #[cfg(feature = "device-sync")]
    let gpiote_executor = gpiote_executor();

    let buffer_allocator = buffer_allocator!(
        { <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize },
        2
    );

    #[cfg(feature = "device-sync")]
    {
        #[cfg(not(feature = "device-sync-client"))]
        info!("Multi-Device Test: Server");
        #[cfg(feature = "device-sync-client")]
        info!("Multi-Device Test: Client");
    }

    #[cfg(not(feature = "device-sync"))]
    info!("Single-Device Test");

    swi_executor.block_on(async {
        #[cfg(not(feature = "device-sync"))]
        let anchor_time = timer.now();
        #[cfg(feature = "device-sync")]
        let anchor_time = {
            info!("Waiting for timer synchronization.");
            wait_for_gpio_event(gpiote_executor, &gpiote).await;
            observe_gpio_event(gpiote_executor, &timer, &gpiote).await
        };

        // Single Device and Multi-Device Server-Side Tests
        #[cfg(not(feature = "device-sync-client"))]
        {
            // Multi-Device Tests
            // no CCA
            #[cfg(feature = "device-sync")]
            let radio =
                multi_rx::server(&mut timer, radio, anchor_time, false, buffer_allocator).await;

            // Timed Tests

            let radio =
                single_rx_off::timed(&mut timer, radio, anchor_time, buffer_allocator).await;
            let radio = single_tx_rx::timed(&mut timer, radio, anchor_time, buffer_allocator).await;
            // no CCA
            let radio =
                single_tx_tx::timed(&mut timer, radio, anchor_time, false, buffer_allocator).await;
            // CCA
            let radio =
                single_tx_tx::timed(&mut timer, radio, anchor_time, true, buffer_allocator).await;

            // Best Effort Tests

            let radio =
                single_rx_off::best_effort(&mut timer, radio, anchor_time, buffer_allocator).await;
            let radio =
                single_tx_rx::best_effort(&mut timer, radio, anchor_time, buffer_allocator).await;
            // no CCA, SIFS
            let radio =
                single_tx_tx::best_effort(&mut timer, radio, anchor_time, false, buffer_allocator)
                    .await;
            // CCA, SIFS
            let _ =
                single_tx_tx::best_effort(&mut timer, radio, anchor_time, true, buffer_allocator)
                    .await;

            // TODO: Test LIFS
        }

        // Multi-Device Client-Side Tests
        #[cfg(feature = "device-sync-client")]
        {
            // Multi-Device Tests
            // no CCA
            let _ = multi_rx::client(&mut timer, radio, anchor_time, false, buffer_allocator).await;
        }
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    done();
}
