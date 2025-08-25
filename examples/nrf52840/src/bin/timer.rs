#![no_std]
#![no_main]

use panic_probe as _;

#[cfg(feature = "rtos-trace")]
use embassy_nrf as _;

use dot15d4::driver::{
    executor::InterruptExecutor,
    timer::{HardwareSignal, LocalClockDuration, Pin, RadioTimerApi, RadioTimerResult},
};
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor, toggle_gpiote_pin, PIN_ALARM};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);

    let (peripherals, _, timer) = config_peripherals();

    let toggle_alarm_pin = || {
        toggle_gpiote_pin(&peripherals.gpiote, PIN_ALARM.gpiote_channel as usize);
    };

    let executor = swi_executor(&peripherals.gpiote);

    let timer_task = async {
        let mut timeout = timer.now();
        for _ in 0..10 {
            const DELAY: LocalClockDuration = LocalClockDuration::nanos(4 * 30518);
            timeout += DELAY;

            // Safety: We run at lower priority than the timer interrupt and we
            //         run from a single task.
            let result =
                unsafe { timer.wait_until(timeout, Some(HardwareSignal::GpioToggle(Pin::Pin0))) }
                    .await;
            assert!(matches!(result, RadioTimerResult::Ok));
        }
        toggle_alarm_pin();
    };

    unsafe { executor.spawn(timer_task).await };

    toggle_alarm_pin();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();
}
