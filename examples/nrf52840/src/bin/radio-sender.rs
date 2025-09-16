//! A helper application that is used to trigger precisely timed tx frames to
//! test radio reception from the radio test application.

#![no_std]
#![no_main]

use panic_probe as _;

#[cfg(feature = "_cortex-m")]
use cortex_m::asm::wfe;
use dot15d4::{
    driver::{constants::PHY_MAX_PACKET_SIZE_127, executor::InterruptExecutor, radio::RadioDriver},
    util::buffer_allocator,
};
use dot15d4_examples_nrf52840::{config_peripherals, gpio_trace::PIN_EXECUTOR, swi_executor};

#[cfg_attr(feature = "_cortex-m", cortex_m_rt::entry)]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let _buffer_allocator = buffer_allocator!(PHY_MAX_PACKET_SIZE_127, 2);

    let (peripherals, clocks, timer) = config_peripherals();

    #[cfg(feature = "gpio-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let _radio = RadioDriver::new(
        peripherals.radio,
        clocks,
        timer,
        #[cfg(feature = "gpio-trace")]
        &peripherals.gpiote,
        #[cfg(feature = "gpio-trace")]
        gpiote_trace_channel,
    );
    let executor = swi_executor(&peripherals.gpiote);

    executor.block_on(async {});

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    loop {
        wfe();
    }
}
