#![no_std]

use dot15d4_driver::socs::nrf::{
    export::{
        pac::{
            CorePeripherals, Peripherals, CLOCK, GPIOTE, NVMC, PPI, RADIO, RNG, RTC0, SCB, SWI0,
            UICR,
        },
        Clocks, ExternalOscillator, LfOscConfiguration, LfOscStarted,
    },
    NrfRadioTimer,
};

#[allow(dead_code)]
pub enum GpioPort {
    P0,
    P1,
}

pub enum GpioteChannel {
    Alarm,
    Executor,
    Tick,
}

pub struct GpioteConfig {
    pub gpiote_channel: GpioteChannel,
    pub port: GpioPort,
    pub pin: u8,
}

impl GpioteConfig {
    const fn new(gpiote_channel: GpioteChannel, port: GpioPort, pin: u8) -> Self {
        Self {
            gpiote_channel,
            port,
            pin,
        }
    }
}

#[allow(clippy::enum_variant_names)]
enum PpiChannel {
    TimerGpiote,
    RtcTickGpiote,
}

pub const PIN_ALARM: GpioteConfig = GpioteConfig::new(GpioteChannel::Alarm, GpioPort::P0, 27);
#[cfg(feature = "gpio-trace")]
pub const PIN_EXECUTOR: GpioteConfig = GpioteConfig::new(GpioteChannel::Executor, GpioPort::P0, 26);
pub const PIN_TICK: GpioteConfig = GpioteConfig::new(GpioteChannel::Tick, GpioPort::P0, 2);

pub struct AvailablePeripherals {
    pub gpiote: GPIOTE,
    pub radio: RADIO,
    pub rng: RNG,
    pub swi0: SWI0,
}

pub fn config_peripherals() -> (
    AvailablePeripherals,
    Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted>,
    NrfRadioTimer,
) {
    let peripherals = Peripherals::take().unwrap();
    let core_peripherals = CorePeripherals::take().unwrap();

    config_reset(&peripherals.UICR, &peripherals.NVMC, &core_peripherals.SCB);

    // Enable the DC/DC converter
    peripherals.POWER.dcdcen.write(|w| w.dcdcen().enabled());

    let clocks = config_clock(peripherals.CLOCK);
    config_gpiote(&peripherals.GPIOTE, PIN_ALARM);
    #[cfg(feature = "gpio-trace")]
    config_gpiote(&peripherals.GPIOTE, PIN_EXECUTOR);
    config_gpiote(&peripherals.GPIOTE, PIN_TICK);
    config_tick_ppi(
        &peripherals.PPI,
        &peripherals.GPIOTE,
        PIN_TICK.gpiote_channel as usize,
        &peripherals.RTC0,
        PpiChannel::RtcTickGpiote as usize,
    );

    let timer = NrfRadioTimer::new(
        peripherals.RTC0,
        peripherals.TIMER0,
        &peripherals.GPIOTE,
        PIN_ALARM.gpiote_channel as usize,
        &peripherals.PPI,
        PpiChannel::TimerGpiote as usize,
    );

    let available_peripherals = AvailablePeripherals {
        gpiote: peripherals.GPIOTE,
        radio: peripherals.RADIO,
        rng: peripherals.RNG,
        swi0: peripherals.SWI0,
    };
    (available_peripherals, clocks, timer)
}

fn config_reset(uicr: &UICR, nvmc: &NVMC, scb: &SCB) {
    if uicr.pselreset[0].read().connect().is_connected() {
        // UICR is already configured.
        return;
    }

    // The UICR registers in flash are pristine or were erased. We need to
    // re-configure them. No need to erase the register to satisfy n_write
    // requirements: It just seems to have been erased by someone else.

    nvmc.config.write(|w| w.wen().wen());
    // Both pselreset configs must be the same for the configuration to take
    // effect.
    for reg in 0..=1 {
        uicr.pselreset[reg].write(|w| {
            // Use the DK's default reset pin P0.18.
            w.port().clear_bit();
            w.pin().variant(18);
            w.connect().connected()
        });
        while nvmc.ready.read().ready().bit_is_clear() {}
    }
    nvmc.config.reset();

    // UICR changes only take effect after a reset.
    soft_reset(scb);
}

fn soft_reset(scb: &SCB) {
    const AIRCR_VECTKEY_MASK: u32 = 0x05FA << 16;
    const SYSRESETREQ: u32 = 1 << 2;
    unsafe { scb.aircr.write(AIRCR_VECTKEY_MASK | SYSRESETREQ) };
}

fn config_clock(clock: CLOCK) -> Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted> {
    // Enable external oscillators.
    Clocks::new(clock)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk()
}

fn config_gpiote(gpiote: &GPIOTE, config: GpioteConfig) {
    gpiote.config[config.gpiote_channel as usize].write(|w| {
        w.mode().task();
        w.port().bit(matches!(config.port, GpioPort::P1));
        w.psel().variant(config.pin);
        w.polarity().toggle()
    });
}

fn config_tick_ppi(
    ppi: &PPI,
    gpiote: &GPIOTE,
    gpiote_channel: usize,
    rtc: &RTC0,
    ppi_rtc_tick_gpiote: usize,
) {
    debug_assert!(ppi_rtc_tick_gpiote <= 19);
    ppi.ch[ppi_rtc_tick_gpiote]
        .eep
        .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
    ppi.ch[ppi_rtc_tick_gpiote].tep.write(|w| {
        w.tep()
            .variant(gpiote.tasks_out[gpiote_channel].as_ptr() as u32)
    });
    // Safety: We checked the PPI channel range.
    ppi.chenset
        .write(|w| unsafe { w.bits(1 << ppi_rtc_tick_gpiote) });
}

pub fn toggle_gpiote_pin(gpiote: &GPIOTE, gpiote_channel: usize) {
    gpiote.tasks_out[gpiote_channel].write(|w| w.tasks_out().set_bit());
}
