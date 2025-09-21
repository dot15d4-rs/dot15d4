//! Example demonstrating capturing timestamps of hardware events.

#![no_std]
#![no_main]
#![cfg(feature = "nrf52840")]

use core::{future::poll_fn, task::Poll};

use panic_probe as _;

use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        socs::nrf::executor::nrf_interrupt_executor,
        timer::{export::ExtU64, HardwareEvent, HighPrecisionTimer, RadioTimerApi},
    },
    util::info,
};
use dot15d4_examples_nrf52840::{config_peripherals, gpio_trace::GpioteChannel, swi_executor};

nrf_interrupt_executor!(gpiote_executor, GPIOTE);

#[cortex_m_rt::entry]
fn main() -> ! {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(bare_metal cpu_freq: 64_000_000 Hz);

    let (peripherals, _, timer) = config_peripherals();

    let swi_executor = swi_executor();
    let gpiote_executor = gpiote_executor((swi_executor.priority().one_higher()).unwrap());

    let gpiote = peripherals.gpiote;
    let gpiote_channel = GpioteChannel::TimerEvent as usize;
    let gpiote_mask = 1 << gpiote_channel;
    let gpiote_event = &gpiote.events_in[gpiote_channel];

    swi_executor.block_on(async {
        loop {
            let timeout = timer.now() + 1.millis();
            let high_precision_timer = timer.start_high_precision_timer(Some(timeout)).unwrap();

            gpiote_event.reset();
            high_precision_timer
                .observe_event(HardwareEvent::GpioToggled)
                .unwrap();

            let wait_for_event = poll_fn(|_| {
                if gpiote_event.read().events_in().bit_is_set() {
                    gpiote.intenclr.write(|w| unsafe { w.bits(gpiote_mask) });
                    gpiote_event.reset();
                    Poll::Ready(())
                } else {
                    gpiote.intenset.write(|w| unsafe { w.bits(gpiote_mask) });
                    Poll::Pending
                }
            });
            unsafe { gpiote_executor.spawn(wait_for_event) }.await;

            let result = high_precision_timer
                .poll_event(HardwareEvent::GpioToggled)
                .unwrap();
            let result = result.duration_since_epoch().to_micros();
            info!("Captured instant: {result}\0");
        }
    });

    unreachable!()
}
