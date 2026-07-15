#![no_std]
#![no_main]
#![cfg(feature = "timer-trace")]
#![allow(clippy::uninlined_format_args)]

#[cfg(feature = "rtos-trace")]
use embassy_nrf as _;

use dot15d4::{
    driver::{
        executor::InterruptExecutor,
        socs::nrf::{HfclkGuard, NrfRadioSleepTimer},
        timer::{HardwareSignal, HighPrecisionTimer, NsDuration, RadioTimerApi, TimedSignal},
    },
    util::info,
};
use dot15d4_examples_nrf52840::{config_peripherals, swi_executor};
use embassy_executor::Spawner;

const PERIOD: NsDuration = NsDuration::secs(1);

const HFXO_WARMUP_LEAD: NsDuration = NsDuration::micros(1500);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);

    let resources = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let swi_executor = swi_executor();
    let mut timer = resources.timer;

    let timer_task = async {
        let mut timeout = timer.now();
        info!("timer-toggle: PERIOD = {} ns\0", PERIOD.ticks());

        let mut period: u32 = 0;
        loop {
            timeout += PERIOD;

            let _ = unsafe { timer.wait_until(timeout - HFXO_WARMUP_LEAD).await };
            let hfclk = HfclkGuard::new();

            let _ = unsafe {
                timer
                    .wait_until(timeout - NrfRadioSleepTimer::GUARD_TIME)
                    .await
            };
            let mut hp_timer = timer.start_high_precision_timer(timeout.into()).unwrap();

            hp_timer
                .schedule_timed_signal(TimedSignal::new(timeout, HardwareSignal::GpioToggle))
                .unwrap();

            let d = decomp::decompose(timeout.ticks());
            info!(
                "p={} target={}us rtc_tick={} epoch={}us timer_ticks={} gap={}ns err={}ns\0",
                period,
                timeout.ticks() / 1_000,
                d.rtc_tick,
                d.epoch_ns / 1_000,
                d.timer_ticks,
                d.alarm_to_signal_ns,
                d.err_ns
            );

            unsafe { hp_timer.wait_for(HardwareSignal::GpioToggle).await };
            drop(hp_timer);
            drop(hfclk);

            period = period.wrapping_add(1);
        }
    };

    unsafe { swi_executor.spawn(timer_task).await };
}

mod decomp {
    pub struct Decomp {
        pub rtc_tick: u64,
        pub epoch_ns: u64,
        pub timer_ticks: u32,
        pub alarm_to_signal_ns: u64,
        pub err_ns: i64,
    }

    const fn instant_to_rtc_tick(ns: u64) -> u64 {
        const N: u32 = 78;
        const MULTIPLIER: u128 = 2u128.pow(6 + N) / 5u128.pow(9);
        ((ns as u128 * MULTIPLIER) >> N) as u64
    }
    const fn rtc_tick_to_instant(rtc_tick: u64) -> u64 {
        const MULTIPLIER: u128 = 5u128.pow(9);
        ((rtc_tick as u128 * MULTIPLIER) >> 6) as u64
    }
    const fn duration_to_timer_ticks(ns: u64) -> u32 {
        const N: u32 = 35;
        const MULTIPLIER: u128 = 2u128.pow(1 + N) / 5u128.pow(3);
        ((ns as u128 * MULTIPLIER) >> N) as u32
    }
    const fn timer_ticks_to_duration(ticks: u32) -> u64 {
        const MULTIPLIER: u64 = 5u64.pow(3);
        ((ticks as u64 * MULTIPLIER) >> 1) as u64
    }

    const TIMER_OFFSET_NS: u64 = 125 + 1;

    pub fn decompose(at_ns: u64) -> Decomp {
        let rtc_tick = instant_to_rtc_tick(at_ns - TIMER_OFFSET_NS);
        let epoch_ns = rtc_tick_to_instant(rtc_tick);
        let raw = duration_to_timer_ticks(at_ns - epoch_ns);
        let timer_ticks = raw - 1;
        let alarm_to_signal_ns = timer_ticks_to_duration(raw);
        let fire_ns = epoch_ns + alarm_to_signal_ns;
        let err_ns = fire_ns as i64 - at_ns as i64;
        Decomp {
            rtc_tick,
            epoch_ns,
            timer_ticks,
            alarm_to_signal_ns,
            err_ns,
        }
    }
}
