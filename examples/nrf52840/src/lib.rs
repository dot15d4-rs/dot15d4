#![no_std]

use dot15d4::driver::socs::nrf::{
    executor::{self as executor, swi0::NrfInterruptExecutor, NrfInterruptPriority},
    export::{
        pac::{CorePeripherals, Peripherals, CLOCK, GPIOTE, NVMC, RADIO, SCB, UICR},
        Clocks, ExternalOscillator, LfOscConfiguration, LfOscStarted,
    },
    NrfRadioTimer,
};

#[cfg(feature = "gpio-trace")]
pub mod gpio_trace {
    use dot15d4::driver::socs::nrf::export::pac::{Peripherals, GPIOTE, PPI, RTC0};

    pub enum GpioPort {
        P0,
        P1,
    }
    use GpioPort::*;

    /// GPIOTE channel allocation across example applications.
    ///
    /// A maximum of eight channels is available.
    #[derive(Clone, Copy)]
    pub enum GpioteChannel {
        /// Interrupt executor tracing.
        Executor,

        /// Timer alarm tracing.
        Alarm,

        /// Timer tick tracing.
        Tick,

        /// Synchronization signal, e.g. for radio packet synchronization across
        /// devices (inbound or outbound).
        Sync,
    }
    use GpioteChannel::*;

    pub enum GpioteDirection {
        In,
        Out,
    }
    use GpioteDirection::*;

    pub struct GpioteConfig {
        pub gpiote_channel: GpioteChannel,
        pub port: GpioPort,
        pub pin: u8,
        pub direction: GpioteDirection,
    }

    impl GpioteConfig {
        const fn new(
            gpiote_channel: GpioteChannel,
            port: GpioPort,
            pin: u8,
            direction: GpioteDirection,
        ) -> Self {
            Self {
                gpiote_channel,
                port,
                pin,
                direction,
            }
        }
    }

    // Tracing pins.
    pub const PIN_EXECUTOR: GpioteConfig = GpioteConfig::new(Executor, P0, 26, Out);
    pub const PIN_TICK: GpioteConfig = GpioteConfig::new(Tick, P0, 27, Out);

    // Timer pins.
    pub const PIN_ALARM: GpioteConfig = GpioteConfig::new(Alarm, P0, 2, Out);

    // Cross-device synchronization pins.
    pub const PIN_SYNC_OUT: GpioteConfig = GpioteConfig::new(Sync, P1, 14, Out);
    pub const PIN_SYNC_IN: GpioteConfig = GpioteConfig::new(Sync, P1, 15, In);

    pub(super) fn config_gpiote(peripherals: &Peripherals, config: &GpioteConfig) {
        if matches!(config.direction, In) {
            let pin_cnf = match config.port {
                P0 => &peripherals.P0.pin_cnf,
                P1 => &peripherals.P1.pin_cnf,
            };
            pin_cnf[config.pin as usize].write(|w| {
                w.pull().pullup();
                w.input().connect()
            });
        }
        peripherals.GPIOTE.config[config.gpiote_channel as usize].write(|w| {
            match config.direction {
                In => w.mode().event(),
                Out => w.mode().task(),
            };
            w.port().bit(matches!(config.port, P1));
            w.psel().variant(config.pin);
            w.polarity().toggle()
        });
    }

    pub(super) fn config_tick_ppi(
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
}

#[cfg(feature = "gpio-trace")]
use gpio_trace::*;

enum PpiChannel {
    Timer,
    #[cfg(feature = "gpio-trace")]
    RtcTickGpiote,
}

pub struct AvailablePeripherals {
    #[cfg(feature = "gpio-trace")]
    pub gpiote: GPIOTE,
    pub radio: RADIO,
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

    #[cfg(feature = "gpio-trace")]
    {
        for pin in [&PIN_EXECUTOR, &PIN_TICK, &PIN_ALARM, &PIN_SYNC_IN] {
            config_gpiote(&peripherals, pin);
        }
        config_tick_ppi(
            &peripherals.PPI,
            &peripherals.GPIOTE,
            PIN_TICK.gpiote_channel as usize,
            &peripherals.RTC0,
            PpiChannel::RtcTickGpiote as usize,
        );
    }

    let clocks = config_clock(peripherals.CLOCK);

    #[cfg(feature = "gpio-trace")]
    let pin_alarm_channel = PIN_ALARM.gpiote_channel as usize;
    #[cfg(feature = "gpio-trace")]
    let sync_in_channel = PIN_SYNC_IN.gpiote_channel as usize;
    let timer = NrfRadioTimer::new(
        peripherals.RTC0,
        peripherals.TIMER0,
        #[cfg(feature = "gpio-trace")]
        pin_alarm_channel,
        #[cfg(feature = "gpio-trace")]
        sync_in_channel,
        PpiChannel::Timer as usize,
    );

    let available_peripherals = AvailablePeripherals {
        #[cfg(feature = "gpio-trace")]
        gpiote: peripherals.GPIOTE,
        radio: peripherals.RADIO,
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

pub fn toggle_gpiote_pin(gpiote: &GPIOTE, gpiote_channel: usize) {
    gpiote.tasks_out[gpiote_channel].write(|w| w.tasks_out().set_bit());
}

pub fn swi_executor(
    #[cfg(feature = "gpio-trace")] gpiote: &GPIOTE,
) -> &'static mut NrfInterruptExecutor {
    // Safety: We don't expose SWI0 as available peripheral, so we can own it
    //         here.
    let swi = unsafe { Peripherals::steal() }.SWI0;
    #[cfg(feature = "gpio-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    executor::swi0(
        swi,
        NrfInterruptPriority::LOWEST_PRIORITY,
        #[cfg(feature = "gpio-trace")]
        gpiote,
        #[cfg(feature = "gpio-trace")]
        gpiote_trace_channel,
    )
}
