#![no_std]
#![cfg(feature = "nrf52840")]

use panic_probe as _;

#[cfg(feature = "timer-trace")]
use dot15d4::driver::socs::nrf::NrfRadioTimerTracingConfig;
use dot15d4::driver::{
    executor::{InterruptExecutor, PB3},
    socs::nrf::{
        executor::{nrf_interrupt_executor, NrfInterruptPriority},
        export::{
            pac::{CorePeripherals, Peripherals, CLOCK, GPIOTE, NVMC, RADIO, SCB, UICR},
            Clocks, ExternalOscillator, LfOscConfiguration, LfOscStarted,
        },
        NrfRadioSleepTimer,
    },
};
#[cfg(feature = "defmt")]
use dot15d4::util::rtt::export::set_defmt_channel;
#[cfg(feature = "terminal")]
use dot15d4::util::rtt::export::DownChannel;
#[cfg(any(feature = "log", feature = "defmt", feature = "rtos-trace",))]
use dot15d4::util::rtt::init_rtt_channels;

#[cfg(feature = "_gpio-trace")]
pub mod gpio_trace {
    use dot15d4::driver::socs::nrf::export::pac::Peripherals;

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
        #[cfg(feature = "executor-trace")]
        Executor,

        /// Timer tick tracing.
        #[cfg(feature = "timer-trace")]
        TimerTick,

        /// Timer GPIO signal.
        #[cfg(feature = "timer-trace")]
        TimerSignal,

        /// Timer GPIO event tracing.
        #[cfg(feature = "timer-trace")]
        TimerEvent,
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
    #[cfg(feature = "executor-trace")]
    pub const PIN_EXECUTOR: GpioteConfig = GpioteConfig::new(Executor, P0, 26, Out);

    // Timer pins.
    #[cfg(feature = "timer-trace")]
    pub const PIN_TIMER_TICK: GpioteConfig = GpioteConfig::new(TimerTick, P0, 27, Out);
    #[cfg(feature = "timer-trace")]
    pub const PIN_TIMER_SIGNAL: GpioteConfig = GpioteConfig::new(TimerSignal, P0, 2, Out);
    #[cfg(feature = "timer-trace")]
    pub const PIN_TIMER_EVENT: GpioteConfig = GpioteConfig::new(TimerEvent, P1, 15, In);

    // Synchronization pin to trigger a timer event on another device.
    #[cfg(feature = "timer-trace")]
    pub const PIN_SYNC_OUT: GpioteConfig = GpioteConfig::new(TimerEvent, P1, 14, Out);

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
}

#[cfg(feature = "_gpio-trace")]
use gpio_trace::*;

pub enum PpiChannel {
    RadioTimer1,
    RadioTimer2,
    #[cfg(feature = "timer-trace")]
    RadioTimerTick,
}

/// PPI channel group required to implement the "timed signal unless event"
/// feature of the timer.
const TIMER_PPI_CHANNEL_GROUP: usize = 0;

pub struct AvailableResources {
    #[cfg(feature = "_gpio-trace")]
    pub gpiote: GPIOTE,
    pub radio: RADIO,
    pub clocks: Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted>,
    pub timer: NrfRadioSleepTimer,
    #[cfg(feature = "terminal")]
    pub sync_in: DownChannel,
}

pub fn config_peripherals(
    #[cfg(feature = "rtos-trace")] start_tracing: fn(),
) -> AvailableResources {
    let peripherals = Peripherals::take().unwrap();
    let core_peripherals = CorePeripherals::take().unwrap();

    config_reset(&peripherals.UICR, &peripherals.NVMC, &core_peripherals.SCB);

    // Enable the DC/DC converter
    peripherals.POWER.dcdcen.write(|w| w.dcdcen().enabled());

    #[cfg(any(feature = "log", feature = "defmt", feature = "rtos-trace",))]
    let _channels = init_rtt_channels!();

    #[cfg(feature = "defmt")]
    set_defmt_channel(_channels.up.0);

    #[cfg(feature = "rtos-trace")]
    start_tracing();

    #[cfg(feature = "_gpio-trace")]
    {
        #[allow(clippy::single_element_loop)]
        for pin in [
            #[cfg(feature = "executor-trace")]
            &PIN_EXECUTOR,
            #[cfg(feature = "timer-trace")]
            &PIN_TIMER_TICK,
            #[cfg(feature = "timer-trace")]
            &PIN_TIMER_SIGNAL,
            #[cfg(feature = "timer-trace")]
            &PIN_TIMER_EVENT,
        ] {
            config_gpiote(&peripherals, pin);
        }
    }

    let clocks = config_clock(peripherals.CLOCK);

    #[cfg(feature = "timer-trace")]
    let timer_tracing_config = NrfRadioTimerTracingConfig {
        gpiote_out_channel: PIN_TIMER_SIGNAL.gpiote_channel as usize,
        gpiote_in_channel: PIN_TIMER_EVENT.gpiote_channel as usize,
        gpiote_tick_channel: PIN_TIMER_TICK.gpiote_channel as usize,
        ppi_tick_channel: PpiChannel::RadioTimerTick as usize,
    };
    let timer = NrfRadioSleepTimer::new(
        peripherals.RTC0,
        peripherals.TIMER0,
        [
            PpiChannel::RadioTimer1 as usize,
            PpiChannel::RadioTimer2 as usize,
        ],
        TIMER_PPI_CHANNEL_GROUP,
        #[cfg(feature = "timer-trace")]
        timer_tracing_config,
    );

    AvailableResources {
        #[cfg(feature = "_gpio-trace")]
        gpiote: peripherals.GPIOTE,
        radio: peripherals.RADIO,
        clocks,
        timer,
        #[cfg(feature = "terminal")]
        sync_in: _channels.down.0,
    }
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

nrf_interrupt_executor!(executor, SWI0_EGU0);

pub fn swi_executor() -> &'static mut impl InterruptExecutor<PB = PB3> {
    #[cfg(feature = "executor-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    executor(
        NrfInterruptPriority::LOWEST_PRIORITY,
        #[cfg(feature = "executor-trace")]
        gpiote_trace_channel,
    )
}
