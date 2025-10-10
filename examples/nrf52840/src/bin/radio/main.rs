//! Low level radio driver tests.
//!
//! Also shows how the interrupt executor can be used stand-alone in a plain
//! cortex-m project.

#![no_std]
#![no_main]
#![cfg(feature = "nrf52840")]
#![allow(clippy::uninlined_format_args)]

#[cfg(feature = "rx_tx_rx")]
mod rx_tx_rx;
#[cfg(feature = "tx_tx")]
mod tx_tx;
mod util;

use dot15d4::driver::{executor::InterruptExecutor, radio::RadioDriver};
#[cfg(any(feature = "rx_tx_rx", feature = "tx_tx"))]
use dot15d4::{
    driver::radio::phy::{OQpsk250KBit, Phy, PhyConfig},
    util::buffer_allocator,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor, AvailableResources};

use util::done;
#[cfg(feature = "sync")]
use util::sync::TestSynchronization;

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let AvailableResources {
        radio,
        clocks,
        timer,
        #[cfg(feature = "sync")]
        sync_in,
        #[cfg(feature = "sync")]
        sync_out,
        ..
    } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    #[cfg(feature = "sync")]
    let mut test_sync = {
        let mut test_sync = TestSynchronization::new(sync_in, sync_out);
        test_sync.start();
        test_sync
    };

    #[cfg(feature = "executor-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        radio,
        clocks,
        timer,
        #[cfg(feature = "executor-trace")]
        gpiote_trace_channel,
    );
    let executor = swi_executor();

    #[cfg(any(feature = "rx_tx_rx", feature = "tx_tx"))]
    let buffer_allocator = buffer_allocator!(
        { <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize },
        2
    );
    executor.block_on(async {
        #[cfg(feature = "rx_tx_rx")]
        let radio = rx_tx_rx::scenarios(radio, timer, buffer_allocator).await;
        #[cfg(feature = "tx_tx")]
        let radio = tx_tx::scenarios(radio, timer, buffer_allocator).await;
        let _ = radio;
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    #[cfg(feature = "sync")]
    test_sync.done();

    done();
}
