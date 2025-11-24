//! Example demonstrating capturing timestamps of hardware events.

#![no_std]
#![no_main]
#![cfg(feature = "timer-trace")]
#![allow(clippy::uninlined_format_args)]

use dot15d4::{driver::executor::InterruptExecutor, util::info};
use dot15d4_examples_nrf52840::{
    config_peripherals, gpiote_executor, observe_gpio_event, swi_executor,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let resources = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let swi_executor = swi_executor();
    let gpiote_executor = gpiote_executor();

    let gpiote = resources.gpiote;
    let timer = resources.timer;

    swi_executor.block_on(async {
        loop {
            let observed_timestamp = observe_gpio_event(gpiote_executor, &timer, &gpiote).await;
            let _uptime_micros = observed_timestamp.duration_since_epoch().to_micros();

            info!("Captured instant: {}\0", _uptime_micros);
        }
    });

    unreachable!()
}
