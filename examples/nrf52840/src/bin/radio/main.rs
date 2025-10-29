//! Low level radio driver tests.
//!
//! Also shows how the interrupt executor can be used stand-alone in a plain
//! cortex-m project.

#![no_std]
#![no_main]
#![cfg(feature = "nrf52840")]
#![allow(clippy::uninlined_format_args)]

mod rx_off;
mod tx_rx;
mod tx_tx;
mod util;

#[cfg(not(feature = "device-sync"))]
use dot15d4::driver::timer::RadioTimerApi;
#[cfg(feature = "device-sync")]
use dot15d4::{driver::nrf_interrupt_executor, util::info};
use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        radio::{
            phy::{OQpsk250KBit, Phy, PhyConfig},
            RadioDriver,
        },
        timer::LocalClockDuration,
    },
    util::buffer_allocator,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor, AvailableResources};
#[cfg(feature = "device-sync")]
use dot15d4_examples_nrf52840::{observe_gpio_event, wait_for_gpio_event};

use self::util::done;

#[cfg(feature = "device-sync")]
nrf_interrupt_executor!(gpiote_executor, GPIOTE);

const TEST_SLOT_DURATION_MS: usize = 10;
const TEST_SLOT_DURATION: LocalClockDuration =
    LocalClockDuration::millis(TEST_SLOT_DURATION_MS as u64);

use rx_off::Test as RxOffTest;
use tx_rx::Test as TxRxTest;
use tx_tx::Test as TxTxTest;

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TestSuite {
    SingleTimedRxOff,
    SingleTimedTxRx,
    SingleTimedTxTxWithoutCca,
    SingleTimedTxTxWithCca,
    SingleBestEffortRxOff,
    SingleBestEffortTxRx,
    SingleBestEffortTxTxWithoutCca,
    SingleBestEffortTxTxWithCca,
}

#[repr(usize)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TestSuiteSlot {
    // Timed tests occupy a well-defined number of time slots.
    SingleTimedRxOff = 0,
    SingleTimedTxRx = RxOffTest::NumSlots as usize,
    SingleTimedTxTxWithoutCca = Self::SingleTimedTxRx as usize + TxRxTest::NumSlots as usize,
    SingleTimedTxTxWithCca = Self::SingleTimedTxTxWithoutCca as usize + TxTxTest::NumSlots as usize,
    // Best effort tests occupy one time slot each.
    SingleBestEffortRxOff,
    SingleBestEffortTxRx,
    SingleBestEffortTxTxWithoutCca,
    SingleBestEffortTxTxWithCca,
}

impl TestSuite {
    pub fn slot(&self) -> usize {
        (match self {
            TestSuite::SingleTimedRxOff => TestSuiteSlot::SingleTimedRxOff,
            TestSuite::SingleTimedTxRx => TestSuiteSlot::SingleTimedTxRx,
            TestSuite::SingleTimedTxTxWithoutCca => TestSuiteSlot::SingleTimedTxTxWithoutCca,
            TestSuite::SingleTimedTxTxWithCca => TestSuiteSlot::SingleTimedTxTxWithCca,
            TestSuite::SingleBestEffortRxOff => TestSuiteSlot::SingleBestEffortRxOff,
            TestSuite::SingleBestEffortTxRx => TestSuiteSlot::SingleBestEffortTxRx,
            TestSuite::SingleBestEffortTxTxWithoutCca => {
                TestSuiteSlot::SingleBestEffortTxTxWithoutCca
            }
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
    let gpiote_executor = gpiote_executor((swi_executor.priority().one_higher()).unwrap());

    let buffer_allocator = buffer_allocator!(
        { <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize },
        2
    );

    swi_executor.block_on(async {
        #[cfg(not(feature = "device-sync"))]
        let anchor_time = timer.now();
        #[cfg(feature = "device-sync")]
        let anchor_time = {
            info!("Waiting for timer synchronization.");
            wait_for_gpio_event(gpiote_executor, &gpiote).await;
            observe_gpio_event(gpiote_executor, &timer, &gpiote).await
        };

        // Timed Tests

        let radio = rx_off::timed(&mut timer, radio, anchor_time, buffer_allocator).await;
        let radio = tx_rx::timed(&mut timer, radio, anchor_time, buffer_allocator).await;
        // no CCA
        let radio = tx_tx::timed(&mut timer, radio, anchor_time, false, buffer_allocator).await;
        // CCA
        let radio = tx_tx::timed(&mut timer, radio, anchor_time, true, buffer_allocator).await;

        // Best Effort Tests

        let radio = rx_off::best_effort(&mut timer, radio, anchor_time, buffer_allocator).await;
        let radio = tx_rx::best_effort(&mut timer, radio, anchor_time, buffer_allocator).await;
        // no CCA, SIFS
        let radio =
            tx_tx::best_effort(&mut timer, radio, anchor_time, false, buffer_allocator).await;
        // CCA, SIFS
        let _ = tx_tx::best_effort(&mut timer, radio, anchor_time, true, buffer_allocator).await;

        // TODO: Test LIFS
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    done();
}
