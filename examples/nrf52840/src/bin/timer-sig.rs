//! Example demonstrating timed hardware signals.

#![no_std]
#![no_main]
#![cfg(feature = "timer-trace")]

#[cfg(feature = "rtos-trace")]
use embassy_nrf as _;

use dot15d4::driver::{
    executor::InterruptExecutor,
    socs::nrf::NrfRadioSleepTimer,
    timer::{HardwareSignal, HighPrecisionTimer, NsDuration, RadioTimerApi, TimedSignal},
};
use dot15d4_examples_nrf52840::{
    config_peripherals, gpio_trace::PIN_TIMER_SIGNAL, swi_executor, toggle_gpiote_pin,
};
#[cfg(feature = "device-sync")]
use dot15d4_examples_nrf52840::{gpiote_executor, observe_gpio_event, wait_for_gpio_event};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);

    let resources = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let swi_executor = swi_executor();
    #[cfg(feature = "device-sync")]
    let gpiote_executor = gpiote_executor();

    let mut timer = resources.timer;
    let gpiote = resources.gpiote;

    let toggle_alarm_pin = || {
        toggle_gpiote_pin(&gpiote, PIN_TIMER_SIGNAL.gpiote_channel as usize);
    };

    let timer_task = async {
        // We specify absolute uptime values so that we can test timer
        // synchronization when working with synchronized devices.
        #[cfg(not(feature = "device-sync"))]
        let mut timeout = timer.now();
        #[cfg(feature = "device-sync")]
        let mut timeout = {
            wait_for_gpio_event(gpiote_executor, &gpiote).await;
            observe_gpio_event(gpiote_executor, &timer, &gpiote).await
        };

        for _ in 0..10 {
            const PERIOD: NsDuration = NsDuration::micros(500);

            timeout += PERIOD;

            unsafe {
                timer
                    .wait_until(timeout - NrfRadioSleepTimer::GUARD_TIME)
                    .await
                    .unwrap()
            };

            let mut high_precision_timer =
                timer.start_high_precision_timer(timeout.into()).unwrap();
            high_precision_timer
                .schedule_timed_signal(TimedSignal::new(timeout, HardwareSignal::GpioToggle))
                .unwrap();

            unsafe {
                high_precision_timer
                    .wait_for(HardwareSignal::GpioToggle)
                    .await
            };

            // The high precision timer is being dropped (and thereby stopped
            // and de-allocated) at the end of the scope.
            drop(high_precision_timer);
        }

        toggle_alarm_pin();
    };

    unsafe { swi_executor.spawn(timer_task).await };

    toggle_alarm_pin();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();
}
