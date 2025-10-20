#[cfg(feature = "executor")]
pub mod executor;
#[cfg(feature = "radio")]
mod radio;
#[cfg(feature = "timer")]
mod timer;

pub mod export {
    pub use nrf52840_pac as pac;
}

use nrf52840_pac::CLOCK;
#[cfg(feature = "radio")]
pub use radio::*;
#[cfg(feature = "timer")]
pub use timer::*;

#[derive(Clone, Copy, Debug)]
pub enum LfClockSource {
    // Uses the internal RC oscillator as sleep timer clock source (test only:
    // not precise enough).
    Rc,

    // Synthesizes the clock signal from the HF clock signal (test only: not
    // energy-efficient).
    Synth,

    // Expects a 32.768 kHz crystal oscillator to be installed.
    #[cfg(not(feature = "ext-lf-clk"))]
    Xtal,

    // Expects a full-swing clock signal to be applied to pin X1 (P0.00):
    // required for multi-device tests to synchronize the sleep timer across
    // devices.
    #[cfg(feature = "ext-lf-clk")]
    External,
}

pub fn start_hf_oscillator(clock: &CLOCK) {
    debug_assert!(!clock.hfclkstat.read().src().is_xtal());
    clock
        .tasks_hfclkstart
        .write(|w| w.tasks_hfclkstart().set_bit());
    while clock.events_hfclkstarted.read().bits() != 1 {}
    clock.events_hfclkstarted.reset();
}

pub fn start_lf_clock(clock: &CLOCK, sleep_timer_clk_src: LfClockSource) {
    // When debugging, the LF clock may continue to run across restarts.
    stop_lf_clock(clock);
    match sleep_timer_clk_src {
        LfClockSource::Rc => clock
            .lfclksrc
            .write(|w| w.src().rc().external().disabled().bypass().disabled()),
        LfClockSource::Synth => clock
            .lfclksrc
            .write(|w| w.src().synth().external().disabled().bypass().disabled()),
        #[cfg(not(feature = "ext-lf-clk"))]
        LfClockSource::Xtal => clock
            .lfclksrc
            .write(|w| w.src().xtal().external().disabled().bypass().disabled()),
        #[cfg(feature = "ext-lf-clk")]
        LfClockSource::External => clock
            .lfclksrc
            .write(|w| w.src().xtal().external().enabled().bypass().enabled()),
    }
    clock
        .tasks_lfclkstart
        .write(|w| w.tasks_lfclkstart().set_bit());
    // When connected to an external clock source, this loop will block
    // indefinitely until the external clock has been started.
    while clock.events_lfclkstarted.read().bits() != 1 {}
    clock.events_lfclkstarted.reset();
}

pub fn stop_lf_clock(clock: &CLOCK) {
    if clock.lfclkstat.read().state().is_running() {
        clock
            .tasks_lfclkstop
            .write(|w| w.tasks_lfclkstop().set_bit());
    }
}
