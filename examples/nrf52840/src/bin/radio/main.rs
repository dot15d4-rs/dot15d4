//! Low level radio driver tests.
//!
//! Also shows how the interrupt executor can be used stand-alone in a plain
//! cortex-m project.

#![no_std]
#![no_main]
#![cfg(feature = "nrf52840")]

use panic_probe as _;

use cortex_m::asm::wfe;
use dot15d4::driver::{executor::InterruptExecutor, radio::RadioDriver};
#[cfg(any(feature = "rx_to_tx", feature = "tx_to_tx"))]
use dot15d4::{
    driver::radio::phy::{OQpsk250KBit, Phy, PhyConfig},
    util::buffer_allocator,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor};

#[cfg(feature = "rx_to_tx")]
mod rx_to_tx;
#[cfg(feature = "tx_to_tx")]
mod tx_to_tx;
mod util;

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    #[cfg(any(feature = "rx_to_tx", feature = "tx_to_tx"))]
    let _buffer_allocator = buffer_allocator!(
        { <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize },
        2
    );

    let (peripherals, clocks, timer) = config_peripherals();
    #[cfg(feature = "executor-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        peripherals.radio,
        clocks,
        timer,
        #[cfg(feature = "executor-trace")]
        gpiote_trace_channel,
    );
    let executor = swi_executor();

    executor.block_on(async {
        #[cfg(feature = "rx_to_tx")]
        let radio = rx_to_tx::scenarios(radio, timer, _buffer_allocator).await;
        #[cfg(feature = "tx_to_tx")]
        let radio = tx_to_tx::scenarios(radio, timer, _buffer_allocator).await;

        let _ = radio;
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    loop {
        wfe();
    }
}
