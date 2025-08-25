//! Low level radio driver tests.
//!
//! Also shows how the interrupt executor can be used stand-alone in a plain
//! cortex-m project.

#![no_std]
#![no_main]

use panic_probe as _;

#[cfg(feature = "_cortex-m")]
use cortex_m::asm::wfe;
use dot15d4::{
    driver::{constants::PHY_MAX_PACKET_SIZE_127, executor::InterruptExecutor, radio::RadioDriver},
    util::buffer_allocator,
};
#[cfg(feature = "gpio-trace")]
use dot15d4_examples_nrf52840::PIN_EXECUTOR;
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor};

#[cfg(feature = "rx_to_tx")]
mod rx_to_tx;
#[cfg(feature = "tx_to_tx")]
mod tx_to_tx;
mod util;

#[cfg_attr(feature = "_cortex-m", cortex_m_rt::entry)]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let buffer_allocator = buffer_allocator!(PHY_MAX_PACKET_SIZE_127, 2);

    let (peripherals, clocks, timer) = config_peripherals();
    #[cfg(feature = "gpio-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        peripherals.radio,
        clocks,
        timer,
        #[cfg(feature = "gpio-trace")]
        &peripherals.gpiote,
        #[cfg(feature = "gpio-trace")]
        gpiote_trace_channel,
    );
    let executor = swi_executor(&peripherals.gpiote);

    executor.block_on(async {
        #[cfg(feature = "rx_to_tx")]
        let radio = rx_to_tx::scenarios(radio, timer, buffer_allocator).await;
        #[cfg(feature = "tx_to_tx")]
        let radio = tx_to_tx::scenarios(radio, timer, buffer_allocator).await;

        let _ = radio;
    });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    #[allow(clippy::empty_loop)]
    loop {
        #[cfg(feature = "_cortex-m")]
        wfe();
    }
}
