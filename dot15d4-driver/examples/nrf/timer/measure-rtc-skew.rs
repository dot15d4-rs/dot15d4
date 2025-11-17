//! Example used to measure RTC tick skew.
//!
//! Usage: See README.md

#![no_std]
#![no_main]
#![cfg(feature = "nrf")]
#![allow(clippy::uninlined_format_args)]

use core::{future::poll_fn, task::Poll};

use panic_probe as _;

use cortex_m::asm::wfe;
use dot15d4_driver::{
    executor::InterruptExecutor,
    socs::nrf::{
        executor::{nrf_interrupt_executor, NrfInterruptPriority},
        export::pac::interrupt,
        start_hf_oscillator, start_lf_clock, stop_lf_clock, LfClockSource,
    },
};
#[cfg(feature = "defmt")]
use dot15d4_util::rtt::export::set_defmt_channel;
use dot15d4_util::{info, init_rtt_channels};
use heapless::Vec;
use nrf52840_pac::{Peripherals, NVIC};

#[cfg(all(not(feature = "defmt"), not(feature = "log")))]
compile_info!("Requires either the 'log' or 'defmt' feature to be enabled.");

nrf_interrupt_executor!(rtc_executor, RTC0);

const NUM_SAMPLES: usize = 10000;
const S_PER_TICK: f32 = 0.0000000625;
const NOMINAL_FREQUENCY: i32 = 2i32.pow(15);

struct Results {
    last: u32,
    data: Vec<u16, NUM_SAMPLES>,
}

impl Results {
    const fn new() -> Self {
        Self {
            last: 0,
            data: Vec::new(),
        }
    }

    fn record(&mut self, timer_ticks: u32) -> Result<(), ()> {
        if self.last > 0 {
            let diff = timer_ticks - self.last;
            if self.data.push(diff as u16).is_err() {
                return Err(());
            };
        }
        self.last = timer_ticks;
        Ok(())
    }

    fn sort(&mut self) -> &[u16] {
        self.data.sort_unstable();
        self.data.as_slice()
    }

    fn clear(&mut self) {
        self.data.clear();
        self.last = 0;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let _channels = init_rtt_channels!();
    #[cfg(feature = "defmt")]
    set_defmt_channel(_channels.up.0);

    #[cfg(feature = "rtos-trace")]
    dot15d4_util::trace::instrument!(bare_metal cpu_freq: 64000000 Hz)();

    let Peripherals {
        POWER: power,
        CLOCK: clock,
        PPI: ppi,
        RTC0: rtc,
        TIMER0: timer,
        ..
    } = Peripherals::take().unwrap();

    power.dcdcen.write(|w| w.dcdcen().enabled());

    start_hf_oscillator(&clock);

    NVIC::unpend(interrupt::RTC0);
    unsafe { NVIC::unmask(interrupt::RTC0) };

    let ch0 = &ppi.ch[0];
    ch0.eep
        .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
    ch0.tep
        .write(|w| w.tep().variant(timer.tasks_capture[0].as_ptr() as u32));
    ppi.chen.write(|w| w.ch0().enabled());

    timer.bitmode.write(|w| w.bitmode()._32bit());
    timer.prescaler.write(|w| w.prescaler().variant(0)); // 16 MHz

    let rtc_executor = rtc_executor(
        NrfInterruptPriority::LOWEST,
        #[cfg(feature = "executor-trace")]
        0,
    );

    let clock_sources = [
        LfClockSource::Rc,
        LfClockSource::Rc,
        LfClockSource::Synth,
        #[cfg(not(feature = "ext-lf-clk"))]
        LfClockSource::Xtal,
        #[cfg(feature = "ext-lf-clk")]
        LfClockSource::External,
    ];

    let mut results = Results::new();

    let mut calibrate_rc = false;
    for clock_source in clock_sources {
        match clock_source {
            LfClockSource::Rc => {
                if calibrate_rc {
                    info!("=========== RC (calibrated) =========")
                } else {
                    info!("=========== RC (not calibrated) =====")
                }
            }
            LfClockSource::Synth => info!("=========== Synthesized ============="),
            #[cfg(not(feature = "ext-lf-clk"))]
            LfClockSource::Xtal => info!("=========== Oscillator =============="),
            #[cfg(feature = "ext-lf-clk")]
            LfClockSource::External => info!("=========== External ================"),
        }

        start_lf_clock(&clock, clock_source);

        if matches!(clock_source, LfClockSource::Rc) && calibrate_rc {
            clock.tasks_cal.write(|w| w.tasks_cal().set_bit());
            while clock.events_done.read().events_done().bit_is_clear() {}
            clock.events_done.reset();
        }

        let collect_data = poll_fn(|_| {
            if rtc.events_tick.read().events_tick().bit_is_clear() {
                // As the tick event is the only enabled event, the executor
                // will be woken exactly once into this branch. That's why we
                // can safely start the RTC here. Starting the RTC here ensures
                // that we don't race to the first tick when starting the RTC.
                rtc.intenset.write(|w| w.tick().set_bit());
                rtc.evtenset.write(|w| w.tick().set_bit());
                rtc.tasks_start.write(|w| w.tasks_start().set_bit());
                Poll::Pending
            } else {
                rtc.events_tick.reset();
                if results.record(timer.cc[0].read().cc().bits()).is_ok() {
                    Poll::Pending
                } else {
                    rtc.intenclr.write(|w| w.tick().set_bit());
                    Poll::Ready(())
                }
            }
        });

        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());
        timer.tasks_start.write(|w| w.tasks_start().set_bit());

        rtc.events_tick.reset();

        rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
        while rtc.counter.read().counter() != 0 {}

        rtc_executor.block_on(collect_data);

        rtc.evtenclr.write(|w| w.tick().set_bit());
        rtc.tasks_stop.write(|w| w.tasks_stop().set_bit());
        timer.tasks_stop.write(|w| w.tasks_stop().set_bit());

        let mut sum_ticks: usize = 0;
        for chunk in results.sort().chunk_by(|a, b| a == b) {
            let timer_ticks = chunk.first().unwrap();
            let occurrences = chunk.len();
            // Safety: All chunks contain at least one value.
            info!("{} ticks: {} times\0", timer_ticks, occurrences);
            sum_ticks += occurrences * *timer_ticks as usize;
        }

        let avg_ticks = sum_ticks as f32 / NUM_SAMPLES as f32;
        info!("average: {} ticks", avg_ticks);

        let period = avg_ticks * S_PER_TICK;
        let frequency = (period.recip() * 1000f32) as usize;
        let frequency_error = frequency as i32 - (NOMINAL_FREQUENCY * 1000);

        let frequency = (frequency as f32) / 1000f32;
        let frequency_error = (frequency_error as f32) / 1000f32;
        info!(
            "frequency: {} Hz (error: {} Hz)",
            frequency, frequency_error
        );

        stop_lf_clock(&clock);
        results.clear();
        calibrate_rc = true;
    }

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::stop();

    loop {
        wfe();
    }
}
