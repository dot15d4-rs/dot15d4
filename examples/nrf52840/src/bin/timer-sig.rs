//! Example demonstrating timed hardware signals.

#![no_std]
#![no_main]

#[cfg(feature = "rtos-trace")]
use embassy_nrf as _;

use dot15d4::driver::{
    executor::InterruptExecutor,
    socs::nrf::NrfRadioSleepTimer,
    timer::{HardwareSignal, HighPrecisionTimer, LocalClockDuration, RadioTimerApi, TimedSignal},
};
use dot15d4_examples_nrf52840::{
    config_peripherals, gpio_trace::PIN_TIMER_SIGNAL, swi_executor, toggle_gpiote_pin,
};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);

    let resources = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let toggle_alarm_pin = || {
        toggle_gpiote_pin(&resources.gpiote, PIN_TIMER_SIGNAL.gpiote_channel as usize);
    };

    let executor = swi_executor();
    let mut timer = resources.timer;

    let timer_task = async {
        let mut timeout = timer.now();

        for _ in 0..10 {
            const PERIOD: LocalClockDuration = LocalClockDuration::micros(500);

            timeout += PERIOD;

            unsafe {
                timer
                    .wait_until(timeout - NrfRadioSleepTimer::GUARD_TIME)
                    .await
                    .unwrap()
            };

            let mut high_precision_timer = timer.start_high_precision_timer(Some(timeout)).unwrap();
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

    unsafe { executor.spawn(timer_task).await };

    toggle_alarm_pin();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();
}
