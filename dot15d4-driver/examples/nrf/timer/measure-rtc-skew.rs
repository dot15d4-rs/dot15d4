//! Example used to measure RTC tick skew.
//!
//! Usage:
//!   $> cd examples/nrf
//!   $> cargo build --examples --features rtos-trace,nrf52840,executor --no-default-features

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
    },
};
use dot15d4_util::{info, init_rtt_channels, rtt::export::set_defmt_channel};
use heapless::Vec;
use nrf52840_pac::{Peripherals, CLOCK, NVIC};

nrf_interrupt_executor!(rtc_executor, RTC0);

struct Results {
    last: u32,
    data: Vec<u16, 10000>,
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
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let channels = init_rtt_channels!();
    set_defmt_channel(channels.up.0);

    let Peripherals {
        POWER: power,
        CLOCK: clock,
        PPI: ppi,
        RTC0: rtc,
        TIMER0: timer,
        ..
    } = Peripherals::take().unwrap();

    power.dcdcen.write(|w| w.dcdcen().enabled());
    config_clock(clock);

    NVIC::unpend(interrupt::RTC0);
    unsafe { NVIC::unmask(interrupt::RTC0) };

    let ch0 = &ppi.ch[0];
    ch0.eep
        .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
    ch0.tep
        .write(|w| w.tep().variant(timer.tasks_capture[0].as_ptr() as u32));
    ppi.chen.write(|w| w.ch0().enabled());

    rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
    while rtc.counter.read().counter() != 0 {}
    rtc.tasks_start.write(|w| w.tasks_start().set_bit());
    while rtc.counter.read().counter() == 0 {}

    timer.bitmode.write(|w| w.bitmode()._32bit());
    timer.prescaler.write(|w| w.prescaler().variant(0)); // 16 MHz
    timer.tasks_start.write(|w| w.tasks_start().set_bit());

    let rtc_executor = rtc_executor(
        NrfInterruptPriority::LOWEST_PRIORITY,
        #[cfg(feature = "executor-trace")]
        0,
    );

    let mut results = Results::new();

    let collect_data = poll_fn(|_| {
        if rtc.events_tick.read().events_tick().bit_is_clear() {
            rtc.intenset.write(|w| w.tick().set_bit());
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

    rtc.evtenset.write(|w| w.tick().set_bit());
    rtc_executor.block_on(collect_data);
    rtc.evtenclr.write(|w| w.tick().set_bit());
    rtc.tasks_stop.write(|w| w.tasks_stop().set_bit());
    timer.tasks_stop.write(|w| w.tasks_stop().set_bit());

    for chunk in results.sort().chunk_by(|a, b| a == b) {
        let timer_ticks = chunk.first().unwrap();
        let occurrences = chunk.len();
        // Safety: All chunks contain at least one value.
        info!("bucket {}: {}\0", timer_ticks, occurrences);
    }

    loop {
        wfe();
    }
}

fn config_clock(clock: CLOCK) {
    clock
        .tasks_hfclkstart
        .write(|w| w.tasks_hfclkstart().set_bit());
    clock.lfclksrc.write(move |w| w.src().xtal());
    clock
        .tasks_lfclkstart
        .write(|w| w.tasks_lfclkstart().set_bit());

    while clock.events_hfclkstarted.read().bits() != 1
        || clock.events_lfclkstarted.read().bits() != 1
    {}

    clock.events_lfclkstarted.reset();
    clock.events_hfclkstarted.reset();
}
