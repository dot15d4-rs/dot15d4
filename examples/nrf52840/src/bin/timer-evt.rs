//! Example demonstrating capturing timestamps of hardware events.

#![no_std]
#![no_main]

use panic_probe as _;

use cortex_m::peripheral::NVIC;
use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        socs::nrf::{export::pac::interrupt, NrfRadioTimer},
        timer::{export::ExtU64, HardwareEvent, RadioTimerApi},
    },
    util::info,
};
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor};

#[cfg_attr(feature = "_cortex-m", cortex_m_rt::entry)]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let (peripherals, _, timer) = config_peripherals();

    // Clear and enable the GPIOTE interrupt.
    NVIC::unpend(interrupt::GPIOTE);
    unsafe { NVIC::unmask(interrupt::GPIOTE) };

    let executor = swi_executor(&peripherals.gpiote);
    executor.block_on(async {
        loop {
            let timeout = timer.now() + 1.millis();

            // Safety: We run at lower priority than the timer interrupt and we
            //         run from a single task.
            let result = unsafe { timer.wait_for_event(timeout, HardwareEvent::GpioToggle) }
                .await
                .unwrap();
            info!(
                "Captured instant: {}\0",
                result.duration_since_epoch().to_micros()
            );
        }
    });
    unreachable!()
}

#[interrupt]
fn GPIOTE() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_enter();

    NrfRadioTimer::on_gpiote_interrupt();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_exit();
}
