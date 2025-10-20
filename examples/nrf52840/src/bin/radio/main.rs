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

use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        radio::{
            phy::{OQpsk250KBit, Phy, PhyConfig},
            RadioDriver,
        },
    },
    util::buffer_allocator,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor, AvailableResources};

use util::done;
#[cfg(feature = "terminal")]
use util::terminal::Terminal;

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let AvailableResources {
        radio,
        timer,
        #[cfg(feature = "terminal")]
        sync_in,
        ..
    } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    #[cfg(feature = "terminal")]
    let terminal = Terminal::new(sync_in).start();

    #[cfg(feature = "executor-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        radio,
        timer,
        #[cfg(feature = "executor-trace")]
        gpiote_trace_channel,
    );
    let executor = swi_executor();

    let buffer_allocator = buffer_allocator!(
        { <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize },
        2
    );
    executor.block_on(async {
        let radio = rx_off::scenarios(radio, timer, buffer_allocator).await;
        let radio = tx_rx::scenarios(radio, timer, buffer_allocator).await;
        let _ = tx_tx::scenarios(radio, timer, buffer_allocator).await;
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    #[cfg(feature = "terminal")]
    terminal.done();

    done();
}
