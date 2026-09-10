//! Radio timer implementation for nRF SoCs.

mod queue;

// No need to use portable atomics in the driver as this is platform-specific
// code.
use core::{
    cell::{Cell, UnsafeCell},
    future::{poll_fn, Future},
    marker::PhantomPinned,
    num::{NonZero, NonZeroU32},
    pin::Pin,
    sync::atomic::{compiler_fence, AtomicU32, AtomicU8, AtomicUsize, Ordering},
    task::{Context, Poll, Waker},
};

use cortex_m::{
    asm::wfe,
    peripheral::{scb::VectActive, Peripherals as CorePeripherals, SCB},
    register::{basepri, basepri_max},
};
use dot15d4_util::sync::CancellationGuard;
use fugit::TimerRateU64;
#[cfg(feature = "timer-trace")]
use nrf52840_pac::GPIOTE;
use nrf52840_pac::{interrupt, Peripherals, CLOCK, NVIC, PPI, RADIO, RTC2, TIMER0};

use crate::{
    socs::nrf::executor::NrfInterruptPriority,
    timer::{
        HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
        OptionalNsInstant, RadioTimerApi, RadioTimerError, TimedSignal,
    },
};

const RTC_CC_HP_TIMER_START: usize = 0;
const RTC_CC_SLEEP_QUEUE: usize = 1;
const RTC_CC_HFXO_PREWARM: usize = 2;
const RTC_CC_HALF_PERIOD: usize = 3;

/// Priority of the TIMER0 interrupt: one below the radio's interrupt executor.
const TIMER_IRQ_PRIORITY: NrfInterruptPriority = NrfInterruptPriority::HIGHEST.lower_by(1);

/// Priority of the RTC2 interrupt.
const RTC_IRQ_PRIORITY: NrfInterruptPriority = TIMER_IRQ_PRIORITY.lower_by(1);

const _: () = assert!(RTC_IRQ_PRIORITY.to_arm_nvic_repr() != 0);

/// This flag atomically represents the current state of the high-precision
/// timer's RTC alarm (CC0).
///
/// # Safety
///
/// This flag synchronizes ownership of shared resources between the RTC
/// interrupt and scheduling context:
///
/// - While the alarm is active, an RTC interrupt may fire at any time, preempt
///   the scheduling context and access and mutate the alarm as well as related
///   resources (e.g. RAM or MMIO registers).
///
/// - While the alarm is pending or after it fired, shared resources must not be
///   accessed from RTC interrupt context. The scheduling context can then
///   access and mutate the alarm as well as related resources.
///
/// Shared resources must not be accessed or mutated from any other than
/// scheduling or interrupt context.
///
/// Considered alternatives, that don't work:
///
/// Synchronization via intenclr/set:
/// - LDREX/STREX are disallowed on device memory
/// - RTC interrupts remain disabled while we wait for the active half period
/// - Depending on the configuration, several distinct RTC interrupts may be
///   involved.
///
/// Synchronization via a special value of the RTC tick (Self::OFF):
/// - The overflow-protected RTC tick is 64 bits wide. A 64 bit value cannot
///   be accessed atomically on a 32 bit platform. Portable atomics would
///   introduce critical sections which we want to avoid.
///
/// Note that the safety conditions of the [`RadioTimerApi`] require the RTC
/// interrupt handler to run at a higher priority than the scheduling thread.
/// This guarantees that any ongoing interrupt execution continues to own shared
/// resources even if it releases them by changing the alarm state. Ownership
/// transfer takes place at the end of such an interrupt execution as it is
/// atomic from the perspective of scheduling context.
///
/// In case of TIMER interrupts, we can use the interrupt register to
/// synchronize resources: Only a globally unique instance of the high precision
/// timer and only a single interrupt can be active.
#[repr(u8)]
enum AlarmState {
    /// The alarm is currently unused and may be acquired by any scheduling
    /// context.
    Unused,
    /// The alarm has been acquired by some scheduling context. It is owned by
    /// that scheduling context.
    Pending,
    /// High-precision synchronization channel only: The high-precision timer is
    /// currently being synchronized to the RTC. The alarm is exclusively owned
    /// by interrupt context. It must be ensured that only a single interrupt
    /// handler is accessing the alarm.
    Synchronizing,
    /// The alarm is currently running and exclusively owned by interrupt
    /// context. Interrupts may preempt scheduling context at any time. It must
    /// be ensured that only a single interrupt handler is accessing the alarm.
    Active,
    /// The alarm has fired and exclusive ownership was transferred back to the
    /// scheduling context.
    Fired,
}

/// State of the dedicated HFXO pre-warm compare (CC2), which drives
/// CLOCK.TASKS_HFCLKSTART through PPI.
///
/// # Safety
///
/// While `Idle`, the compare and the pre-warm tick are exclusively owned by
/// scheduling context (only a single high-precision timer can exist, and only
/// its start arms the pre-warm). While `Armed`, the compare interrupt is
/// enabled so the channel frees itself early once it fired; release is decided
/// by an `Armed -> Idle` compare-exchange electing exactly one winner between
/// the interrupt's early release and the timer token's drop.
#[repr(u8)]
enum PrewarmState {
    Idle,
    Armed,
}

struct Alarm {
    /// The current alarm state. See [`AlarmState`] for details and safety
    /// considerations.
    state: AtomicU8,

    /// The RTC tick of a pending alarm.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    rtc_tick: Cell<u64>,

    /// Waker for the high-precision timer's `wait_for()`.
    ///
    /// Safety: Access is synchronized via the TIMER interrupt-enable register,
    ///         see [`State::timer_wait_for`]. The waker itself is [`Sync`].
    ///         It's therefore ok, to wake it from interrupt context.
    ///
    /// Note that the safety conditions of the [`HighPrecisionTimer`] disallow
    /// migrating an active timer to a different task. This is required to grant
    /// exclusive access to the waker from the interrupt as long as it is
    /// active.
    waker: UnsafeCell<Option<Waker>>,
}

/// Safety: See safety comments in the implementation.
unsafe impl Sync for Alarm {}

impl Alarm {
    const fn new() -> Self {
        Self {
            // The state is initially 'unused' to signal that the scheduling
            // thread has exclusive access to this alarm but still needs to
            // program it.
            state: AtomicU8::new(AlarmState::Unused as u8),
            rtc_tick: Cell::new(0),
            waker: UnsafeCell::new(None),
        }
    }
}

/// The sleep-timer waiter queue behind a BASEPRI priority-ceiling lock.
struct SleepQueue(UnsafeCell<queue::Inner>);

// Safety: The inner queue is only ever accessed through `with()`, which masks
//         every context that is allowed to access it (see above); wakers taken
//         out of the queue are `Sync` and may be woken from any context. On
//         the single-core nRF platform, compiler fences suffice as barriers.
unsafe impl Sync for SleepQueue {}

impl SleepQueue {
    const fn new() -> Self {
        Self(UnsafeCell::new(queue::Inner::new()))
    }

    /// Runs `f` with exclusive access to the queue.
    fn with<R>(&self, f: impl FnOnce(&mut queue::Inner) -> R) -> R {
        let restore = basepri::read();
        basepri_max::write(RTC_IRQ_PRIORITY.to_arm_nvic_repr());
        compiler_fence(Ordering::SeqCst);

        // Safety: All other queue users are masked, see the struct docs.
        let result = f(unsafe { &mut *self.0.get() });

        compiler_fence(Ordering::SeqCst);
        // Safety: Restores a previously read value; `basepri_max` never
        //         lowered it below an outer ceiling in the meantime.
        unsafe { basepri::write(restore) };
        result
    }
}

/// The driver initialization state.
#[repr(u8)]
enum InitializationState {
    /// The driver has not yet been initialized.
    Uninitialized,
    /// The initialization method is currently executing.
    Initializing,
    /// The driver has been initialized.
    Initialized,
}

/// The nRF radio timer implements a local (i.e. non-syntonized), monotonic,
/// overflow-protected uptime counter. It combines a low-energy RTC sleep timer
/// with a high-precision wake-up TIMER.
///
/// The timer can trigger asynchronous CPU wake-ups, fire PPI-backed hardware
/// signals or capture hardware event timestamps.
///
/// Safety: As we are on the single-core nRF platform we don't need to
///         synchronize atomic operations via CPU memory barriers. It is
///         sufficient to place appropriate compiler fences.
struct State {
    /// The driver's initialization state, see [`InitializationState`].
    init_state: AtomicU8,

    /// Number of half counter periods elapsed since boot.
    ///
    /// Safety: This needs to be atomic as it will be shared between the
    ///         interrupt and the application threads.
    //
    // Overflow protection is heavily inspired by embassy_nrf. Kudos to the
    // embassy contributors!
    half_period: AtomicU32,

    /// Synchronizes the high-precision timer's RTC channel (CC0).
    ///
    /// Safety: RTC alarms are Sync.
    hp_alarm: Alarm,

    /// The sleep-timer waiter queue (CC1).
    sleep_queue: SleepQueue,

    /// State of the dedicated HFXO pre-warm compare (CC2), see [`PrewarmState`].
    prewarm_state: AtomicU8,

    /// The RTC tick at which the pre-warm compare is set to fire.
    ///
    /// Safety: Written from scheduling context while the pre-warm state is
    ///         [`PrewarmState::Idle`], read-only in both contexts while
    ///         [`PrewarmState::Armed`]. The state store releases/acquires it.
    prewarm_tick: Cell<u64>,

    /// Contains the reference overflow-protected RTC tick (losslessly
    /// represented as the corresponding radio clock instant) at which the
    /// high-precision timer was started.
    ///
    /// This value is initialized while the [`RtcChannel::TimerSynchronization`]
    /// channel is in state [`AlarmState::Pending`] (when synchronizing to a
    /// specific RTC tick) or [`AlarmState::Synchronizing`] (when synchronizing
    /// to the next RTC tick). It is only valid to read in state
    /// [`AlarmState::Active`].
    ///
    /// Safety: Guarded by the [`RtcChannel::TimerSynchronization`] alarm's
    ///         [`AlarmState`]: May be written from scheduling context while
    ///         that state is neither synchronizing nor active. May be written
    ///         from interrupt context while synchronizing. Read-only access
    ///         from all contexts while active.
    timer_epoch: Cell<NsInstant>,

    /// Whether the configured LF sleep clock depends on the HF clock.
    ///
    /// This is the case for the synthesized LF clock source. In that mode, the
    /// HFXO cannot be power-gated while the sleep timer is meant to keep
    /// running.
    sleep_clock_uses_hfclk: AtomicU8,

    /// Counter of active HFXO "users" managed by this driver while the LF sleep
    /// clock does not itself require HFCLK.
    hfclk_active_use: AtomicUsize,

    /// GPIOTE channel used for GPIO signal triggering.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "timer-trace")]
    gpiote_out_channel: AtomicUsize,

    /// GPIOTE channel used for GPIO event capturing.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "timer-trace")]
    gpiote_in_channel: AtomicUsize,

    /// GPIOTE channel toggled at the RTC alarm / TIMER-start boundary.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "timer-trace")]
    gpiote_alarm_channel: AtomicUsize,

    /// Primary PPI channel used for signal triggering (timer synchronization,
    /// gpio toggle) and event capturing (rx/tx enabled).
    ///
    /// We assume that these signals and events will not be used concurrently.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel1: AtomicUsize,

    /// Secondary PPI channel used for event capturing (frame started,
    /// radio disabled, gpio toggled).
    ///
    /// We assume that these events will not be used concurrently.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel2: AtomicUsize,

    /// PPI channel group used for signal triggering (disable unless
    /// framestart, txen if crcok).
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel_group: AtomicUsize,

    /// PPI channel used to gate a timed signal on a hardware event
    /// (event -> enable channel group).
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    ppi_gate_channel: AtomicUsize,

    /// PPI channel that routes TIMER CC0 -> RADIO TASKS_START.
    ppi_txstart_channel: AtomicUsize,

    /// PPI channel that routes TIMER CC1 -> RADIO TASKS_TXEN.
    ppi_txramp_channel: AtomicUsize,

    /// The PPI channel mask containing all PPI channels used by this driver.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel_mask: AtomicU32,
}

/// Safety: See safety comments in the implementation.
unsafe impl Sync for State {}

impl State {
    // See the high-precision timer trait implementation docs for resource
    // assignments.

    const RTC_THREE_QUARTERS_PERIOD: u64 = 0xc00000;
    const RTC_GUARD_TICKS: u64 = 2;

    // Pre-assigned timer channels.
    const TIMER_TEST_CHANNEL: usize = 2;
    const TIMER_OVERFLOW_PROTECTION_CHANNEL: usize = 3;

    // Pre-programmed PPI channels.
    const TIMER_CC0_RADIO_TXEN_CHANNEL: usize = 20;
    const TIMER_CC0_RADIO_RXEN_CHANNEL: usize = 21;
    const TIMER_CC1_RADIO_DISABLE_CHANNEL: usize = 22;
    const RTC_CC0_TIMER_START_CHANNEL: usize = 19;

    const RTC_HFCLKSTART_PPI_CHANNEL: usize = 18;

    const HFXO_PREWARM_LEAD_TICKS: u64 = 14;

    const PPI_CHANNEL_MASK: u32 = 1 << Self::TIMER_CC0_RADIO_TXEN_CHANNEL
        | 1 << Self::TIMER_CC0_RADIO_RXEN_CHANNEL
        | 1 << Self::TIMER_CC1_RADIO_DISABLE_CHANNEL
        | 1 << Self::RTC_CC0_TIMER_START_CHANNEL;

    const fn new() -> Self {
        Self {
            init_state: AtomicU8::new(InitializationState::Uninitialized as u8),
            half_period: AtomicU32::new(0),
            hp_alarm: Alarm::new(),
            sleep_queue: SleepQueue::new(),
            prewarm_state: AtomicU8::new(PrewarmState::Idle as u8),
            prewarm_tick: Cell::new(0),
            timer_epoch: Cell::new(NsInstant::from_ticks(0)),
            sleep_clock_uses_hfclk: AtomicU8::new(0),
            hfclk_active_use: AtomicUsize::new(0),
            #[cfg(feature = "timer-trace")]
            gpiote_out_channel: AtomicUsize::new(0),
            #[cfg(feature = "timer-trace")]
            gpiote_in_channel: AtomicUsize::new(1),
            #[cfg(feature = "timer-trace")]
            gpiote_alarm_channel: AtomicUsize::new(2),
            ppi_channel1: AtomicUsize::new(0),
            ppi_channel2: AtomicUsize::new(0),
            ppi_channel_group: AtomicUsize::new(0),
            ppi_gate_channel: AtomicUsize::new(0),
            ppi_txstart_channel: AtomicUsize::new(0),
            ppi_txramp_channel: AtomicUsize::new(0),
            ppi_channel_mask: AtomicU32::new(0),
        }
    }

    fn nvic() -> NVIC {
        // Safety: We only use the peripheral to adjust properties of interrupts
        //         we exclusively own.
        unsafe { CorePeripherals::steal() }.NVIC
    }

    fn rtc() -> RTC2 {
        // We own the RTC peripheral exclusively.
        unsafe { Peripherals::steal() }.RTC2
    }

    fn timer() -> TIMER0 {
        // We own the TIMER peripheral exclusively.
        unsafe { Peripherals::steal() }.TIMER0
    }

    fn clock() -> CLOCK {
        // We own the CLOCK peripheral exclusively.
        unsafe { Peripherals::steal() }.CLOCK
    }

    fn ppi() -> PPI {
        // We only access PPI channels that we exclusively own.
        unsafe { Peripherals::steal() }.PPI
    }

    #[cfg(feature = "timer-trace")]
    fn gpiote() -> GPIOTE {
        // We only access GPIOTE channels that we exclusively own.
        unsafe { Peripherals::steal() }.GPIOTE
    }

    fn radio() -> RADIO {
        // We only access RADIO resources when asked by the peripheral's owner
        // to do so.
        unsafe { Peripherals::steal() }.RADIO
    }

    /// Takes exclusive ownership of the RTC and TIMER peripherals and
    /// initializes the driver.
    ///
    /// This must be called during early initialization while no concurrent
    /// critical sections are active.
    pub fn init(&self, config: NrfRadioTimerConfig) {
        assert_eq!(
            self.init_state
                .swap(InitializationState::Initializing as u8, Ordering::AcqRel),
            InitializationState::Uninitialized as u8
        );

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::instrument();

        let sleep_clock_uses_hfclk =
            matches!(config.sleep_timer_clk_src, super::LfClockSource::Synth);
        self.sleep_clock_uses_hfclk
            .store(sleep_clock_uses_hfclk as u8, Ordering::Relaxed);
        if sleep_clock_uses_hfclk {
            super::start_hf_oscillator(&config.clock);
        }
        super::start_lf_clock(&config.clock, config.sleep_timer_clk_src);

        let [ppi_channel1, ppi_channel2] = config.ppi_channels;
        let ppi_gate_channel = config.ppi_gate_channel;
        let ppi_txstart_channel = config.ppi_txstart_channel;
        let ppi_txramp_channel = config.ppi_txramp_channel;
        debug_assert!(ppi_channel1 <= 19);
        debug_assert!(ppi_channel2 <= 19);
        debug_assert!(ppi_gate_channel <= 19);
        debug_assert!(ppi_txstart_channel <= 19);
        debug_assert!(ppi_txramp_channel <= 19);
        debug_assert_ne!(ppi_channel1, ppi_channel2);
        debug_assert_ne!(ppi_gate_channel, ppi_channel1);
        debug_assert_ne!(ppi_gate_channel, ppi_channel2);
        debug_assert_ne!(ppi_txstart_channel, ppi_channel1);
        debug_assert_ne!(ppi_txstart_channel, ppi_channel2);
        debug_assert_ne!(ppi_txstart_channel, ppi_gate_channel);
        debug_assert_ne!(ppi_txramp_channel, ppi_channel1);
        debug_assert_ne!(ppi_txramp_channel, ppi_channel2);
        debug_assert_ne!(ppi_txramp_channel, ppi_gate_channel);
        debug_assert_ne!(ppi_txramp_channel, ppi_txstart_channel);
        self.ppi_channel1.store(ppi_channel1, Ordering::Relaxed);
        self.ppi_channel2.store(ppi_channel2, Ordering::Relaxed);
        self.ppi_gate_channel
            .store(ppi_gate_channel, Ordering::Relaxed);
        self.ppi_txstart_channel
            .store(ppi_txstart_channel, Ordering::Relaxed);
        self.ppi_txramp_channel
            .store(ppi_txramp_channel, Ordering::Relaxed);
        self.ppi_channel_mask.store(
            1 << ppi_channel1
                | 1 << ppi_channel2
                | 1 << ppi_gate_channel
                | 1 << ppi_txstart_channel
                | 1 << ppi_txramp_channel
                | Self::PPI_CHANNEL_MASK,
            Ordering::Relaxed,
        );

        debug_assert!(config.ppi_channel_group <= 5);
        self.ppi_channel_group
            .store(config.ppi_channel_group, Ordering::Relaxed);

        let rtc = config.rtc;
        let timer = config.timer;

        #[cfg(feature = "timer-trace")]
        {
            let NrfRadioTimerTracingConfig {
                gpiote_out_channel,
                gpiote_in_channel,
                gpiote_alarm_channel,
                gpiote_tick_channel,
                ppi_tick_channel,
            } = config.tracing_config;

            debug_assert!(gpiote_out_channel <= 7);
            debug_assert!(gpiote_in_channel <= 7);
            debug_assert!(gpiote_alarm_channel <= 7);
            debug_assert!(gpiote_tick_channel <= 7);
            debug_assert_ne!(gpiote_in_channel, gpiote_out_channel);
            debug_assert_ne!(gpiote_in_channel, gpiote_tick_channel);
            debug_assert_ne!(gpiote_tick_channel, gpiote_out_channel);
            debug_assert_ne!(gpiote_alarm_channel, gpiote_out_channel);
            debug_assert_ne!(gpiote_alarm_channel, gpiote_in_channel);
            debug_assert_ne!(gpiote_alarm_channel, gpiote_tick_channel);

            debug_assert!(ppi_tick_channel <= 19);
            debug_assert_ne!(ppi_channel1, ppi_tick_channel);
            debug_assert_ne!(ppi_channel2, ppi_tick_channel);

            self.gpiote_out_channel
                .store(gpiote_out_channel, Ordering::Relaxed);
            self.gpiote_in_channel
                .store(gpiote_in_channel, Ordering::Relaxed);
            self.gpiote_alarm_channel
                .store(gpiote_alarm_channel, Ordering::Relaxed);

            let ppi = Self::ppi();
            let gpiote = Self::gpiote();

            ppi.ch[ppi_tick_channel]
                .eep
                .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
            ppi.ch[ppi_tick_channel].tep.write(|w| {
                w.tep()
                    .variant(gpiote.tasks_out[gpiote_tick_channel].as_ptr() as u32)
            });
            // Safety: We checked the PPI channel range.
            ppi.chenset
                .write(|w| unsafe { w.bits(1 << ppi_tick_channel) });

            rtc.evtenset.write(|w| w.tick().set_bit());
        }

        timer.tasks_stop.write(|w| w.tasks_stop().set_bit());
        timer.mode.reset();
        timer.bitmode.write(|w| w.bitmode()._32bit());

        // The prescaler has a non-zero reset value.
        timer.prescaler.write(|w| w.prescaler().variant(0));
        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());

        // The timer must not overflow: stop it when it reaches its max value.
        timer.cc[Self::TIMER_OVERFLOW_PROTECTION_CHANNEL].write(|w| w.cc().variant(u32::MAX));
        timer
            .shorts
            // Safety: The overflow protection channel is in range. Stop shorts start at bit 8.
            .write(|w| unsafe { w.bits(1 << (8 + Self::TIMER_OVERFLOW_PROTECTION_CHANNEL)) });

        {
            let ppi = Self::ppi();
            ppi.ch[Self::RTC_CC0_TIMER_START_CHANNEL]
                .eep
                .write(|w| w.eep().variant(rtc.events_compare[0].as_ptr() as u32));
            ppi.ch[Self::RTC_CC0_TIMER_START_CHANNEL]
                .tep
                .write(|w| w.tep().variant(timer.tasks_start.as_ptr() as u32));
            #[cfg(feature = "timer-trace")]
            {
                let gpiote = Self::gpiote();
                let gpiote_alarm_channel = self.gpiote_alarm_channel.load(Ordering::Relaxed);
                ppi.fork[Self::RTC_CC0_TIMER_START_CHANNEL].tep.write(|w| {
                    w.tep()
                        .variant(gpiote.tasks_out[gpiote_alarm_channel].as_ptr() as u32)
                });
            }
            #[cfg(not(feature = "timer-trace"))]
            ppi.fork[Self::RTC_CC0_TIMER_START_CHANNEL].tep.reset();

            // HFXO pre-warm: the dedicated pre-warm COMPARE (CC2) -> CLOCK
            // TASKS_HFCLKSTART. Both endpoints are fixed; the channel is only
            // enabled while a pre-warm is armed (arrange_hfclk_prewarm()).
            // Safety: CLOCK is owned only by this subsystem; we just take the
            // address of its HFCLKSTART task register.
            let hfclkstart = unsafe { &*nrf52840_pac::CLOCK::ptr() }
                .tasks_hfclkstart
                .as_ptr() as u32;
            ppi.ch[Self::RTC_HFCLKSTART_PPI_CHANNEL].eep.write(|w| {
                w.eep()
                    .variant(rtc.events_compare[RTC_CC_HFXO_PREWARM].as_ptr() as u32)
            });
            ppi.ch[Self::RTC_HFCLKSTART_PPI_CHANNEL]
                .tep
                .write(|w| w.tep().variant(hfclkstart));
            ppi.fork[Self::RTC_HFCLKSTART_PPI_CHANNEL].tep.reset();

            // Timed TX start: TIMER CC0 compare -> RADIO TASKS_START.
            // Endpoints are fixed; the channel is enabled per scheduled signal.
            // Safety: we only take the address of the RADIO task register.
            let radio_start = Self::radio().tasks_start.as_ptr() as u32;
            let ch = &ppi.ch[ppi_txstart_channel];
            ch.eep
                .write(|w| w.eep().variant(timer.events_compare[0].as_ptr() as u32));
            ch.tep.write(|w| w.tep().variant(radio_start));
            ppi.fork[ppi_txstart_channel].tep.reset();

            // Timed TX ramp: TIMER CC1 compare -> RADIO TASKS_TXEN. The
            // pre-programmed TXEN route (PPI 20) is wired to CC0, which a timed
            // transmission already needs for TASKS_START; this second route
            // frees the ramp from that conflict so the transmitter can be
            // brought up just before the launch instead of in scheduling
            // context. Endpoints are fixed; the channel is enabled per
            // scheduled signal.
            // Safety: we only take the address of the RADIO task register.
            let radio_txen = Self::radio().tasks_txen.as_ptr() as u32;
            let ch = &ppi.ch[ppi_txramp_channel];
            ch.eep
                .write(|w| w.eep().variant(timer.events_compare[1].as_ptr() as u32));
            ch.tep.write(|w| w.tep().variant(radio_txen));
            ppi.fork[ppi_txramp_channel].tep.reset();
        }

        rtc.tasks_stop.write(|w| w.tasks_stop().set_bit());
        loop {
            rtc.prescaler.reset();
            if rtc.prescaler.read().prescaler().bits() == 0 {
                break;
            }
        }

        rtc.intenclr.write(|w| {
            w.tick().set_bit();
            w.ovrflw().set_bit();
            w.compare0().set_bit();
            w.compare1().set_bit();
            w.compare2().set_bit();
            w.compare3().set_bit()
        });
        rtc.evtenclr.write(|w| {
            w.ovrflw().set_bit();
            w.compare0().set_bit();
            w.compare1().set_bit();
            w.compare2().set_bit();
            w.compare3().set_bit()
        });
        rtc.events_ovrflw.reset();
        rtc.events_tick.reset();
        for events_compare in rtc.events_compare.iter() {
            events_compare.reset();
        }

        rtc.cc[RTC_CC_HALF_PERIOD].write(|w| w.compare().variant(0x800000));
        rtc.intenset.write(|w| {
            w.ovrflw().set_bit();
            w.compare3().set_bit()
        });

        rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
        while rtc.counter.read().counter() != 0 {}

        rtc.tasks_start.write(|w| w.tasks_start().set_bit());
        while rtc.counter.read().counter() == 0 {}

        let mut nvic = Self::nvic();

        // Safety: See the documented synchronization approach.
        unsafe {
            nvic.set_priority(interrupt::TIMER0, TIMER_IRQ_PRIORITY.to_arm_nvic_repr());
            nvic.set_priority(interrupt::RTC2, RTC_IRQ_PRIORITY.to_arm_nvic_repr());
        }

        // Clear and enable timer interrupts.
        NVIC::unpend(interrupt::RTC2);
        NVIC::unpend(interrupt::TIMER0);
        // Safety: We're in early initialization, so there should be no
        //         concurrent critical sections.
        unsafe { NVIC::unmask(interrupt::RTC2) };
        unsafe { NVIC::unmask(interrupt::TIMER0) };

        self.init_state
            .store(InitializationState::Initialized as u8, Ordering::Release);
    }

    /// Ensures that the driver has been properly initialized.
    ///
    /// # Panics
    ///
    /// Panics if any of the public driver methods is called before it has been
    /// initialized.
    fn assert_initialized(&self) {
        debug_assert_eq!(
            self.init_state.load(Ordering::Acquire),
            InitializationState::Initialized as u8
        );
    }

    fn sleep_clock_uses_hfclk(&self) -> bool {
        self.sleep_clock_uses_hfclk.load(Ordering::Relaxed) != 0
    }

    fn hfclk_request(&self) {
        if self.sleep_clock_uses_hfclk() {
            return;
        }

        self.hfclk_active_use.fetch_add(1, Ordering::Relaxed);
    }

    /// Start the HFXO now and block until it is running.
    fn hfclk_start_blocking(&self) {
        if self.sleep_clock_uses_hfclk() {
            return;
        }
        super::start_hf_oscillator(&Self::clock());
        super::wait_hf_oscillator_ready(&Self::clock());
    }

    fn hfclk_release(&self) {
        if self.sleep_clock_uses_hfclk() {
            return;
        }

        let prev = self.hfclk_active_use.fetch_sub(1, Ordering::Relaxed);
        assert!(prev > 0);
        if prev == 1 {
            super::stop_hf_oscillator(&Self::clock());
        }
    }

    /// Make sure the HFXO is running by the time the timer starts at
    /// `timer_start_tick` (RTC2 CC0).
    fn arrange_hfclk_prewarm(&self, timer_start_tick: u64) -> bool {
        if Self::clock().hfclkrun.read().status().is_triggered() {
            return false;
        }

        let rtc_now_tick = self.rtc_now_tick();
        if !self.sleep_clock_uses_hfclk()
            && timer_start_tick
                > rtc_now_tick + Self::HFXO_PREWARM_LEAD_TICKS + Self::RTC_GUARD_TICKS
            && timer_start_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD
        {
            debug_assert_eq!(
                self.prewarm_state.load(Ordering::Relaxed),
                PrewarmState::Idle as u8
            );

            let prewarm_tick = timer_start_tick - Self::HFXO_PREWARM_LEAD_TICKS;
            let rtc = Self::rtc();

            rtc.events_compare[RTC_CC_HFXO_PREWARM].reset();
            rtc.cc[RTC_CC_HFXO_PREWARM]
                .write(|w| w.compare().variant(prewarm_tick as u32 & 0xFFFFFF));

            // The ISR uses this tick to tell the real compare from a
            // spurious early match on the stale CC value.
            // Safety: The pre-warm is idle, i.e. exclusively owned by this
            //         scheduling context; the value is released to interrupt
            //         context by the `Armed` store below.
            self.prewarm_tick.set(prewarm_tick);

            rtc.evtenset.write(|w| w.compare2().set_bit());

            Self::ppi()
                .chenset
                .write(|w| unsafe { w.bits(1 << Self::RTC_HFCLKSTART_PPI_CHANNEL) });

            compiler_fence(Ordering::Release);
            self.prewarm_state
                .store(PrewarmState::Armed as u8, Ordering::Relaxed);
            rtc.intenset.write(|w| w.compare2().set_bit());

            if self.rtc_now_tick() + Self::RTC_GUARD_TICKS <= prewarm_tick {
                return true;
            }
            self.rtc_release_prewarm();
        }

        self.hfclk_start_blocking();
        false
    }

    /// Tear down the HFXO pre-warm.
    fn rtc_release_prewarm(&self) {
        compiler_fence(Ordering::Release);
        let won = self
            .prewarm_state
            .compare_exchange(
                PrewarmState::Armed as u8,
                PrewarmState::Idle as u8,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_ok();
        compiler_fence(Ordering::Acquire);

        if won {
            let rtc = Self::rtc();
            rtc.intenclr.write(|w| w.compare2().set_bit());
            rtc.evtenclr.write(|w| w.compare2().set_bit());
            Self::ppi()
                .chenclr
                .write(|w| unsafe { w.bits(1 << Self::RTC_HFCLKSTART_PPI_CHANNEL) });
            rtc.events_compare[RTC_CC_HFXO_PREWARM].reset();
        }
    }

    /// Sets the high-precision timer alarm's tick value.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    unsafe fn hp_set_alarm_tick(&self, rtc_tick: u64) {
        self.hp_alarm.rtc_tick.set(rtc_tick);
    }

    /// Read the high-precision timer alarm's RTC tick.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    /// - Compiler fences are required to acquire/release this value.
    unsafe fn hp_alarm_tick(&self) -> u64 {
        self.hp_alarm.rtc_tick.get()
    }

    /// Returns `true` while the high-precision timer alarm is active or
    /// synchronizing (and therefore owned by interrupt context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_hp_alarm_owned_by_interrupt(&self) -> bool {
        let state = self.hp_alarm.state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Active as u8 || state == AlarmState::Synchronizing as u8
    }

    /// Returns `true` while the high-precision timer alarm is pending (and
    /// owned by scheduling context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_hp_alarm_pending(&self) -> bool {
        let state = self.hp_alarm.state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Pending as u8
    }

    /// Returns 'true' while the high-precision timer is being synchronized to
    /// the RTC via the "next tick" synchronization approach.
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_pending_immediate_synchronization(&self) -> bool {
        let state = self.hp_alarm.state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Synchronizing as u8
    }

    /// Mark the high-precision timer alarm as pending if it is unused.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn hp_try_acquire_alarm(&self) -> Result<(), RadioTimerError> {
        compiler_fence(Ordering::Release);
        let result = self.hp_alarm.state.compare_exchange(
            AlarmState::Unused as u8,
            AlarmState::Pending as u8,
            Ordering::Relaxed,
            Ordering::Relaxed,
        );
        compiler_fence(Ordering::Acquire);
        if result.is_ok() {
            Ok(())
        } else {
            Err(RadioTimerError::Busy)
        }
    }

    /// Transfer ownership of the high-precision timer alarm to interrupt
    /// context.
    fn rtc_start_immediate_timer_synchronization(&self) {
        compiler_fence(Ordering::Release);
        self.hp_alarm
            .state
            .store(AlarmState::Synchronizing as u8, Ordering::Relaxed);
    }

    /// Transfer ownership of the high-precision timer alarm to interrupt
    /// context.
    ///
    /// Releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// Note: This does _not_ also activate the compare event. It may have to
    ///       remain inactive if we've not yet reached the target timer period.
    fn hp_activate_alarm(&self) {
        compiler_fence(Ordering::Release);
        self.hp_alarm
            .state
            .store(AlarmState::Active as u8, Ordering::Relaxed);
    }

    /// Disables the timer-start compare event and signals to the scheduling
    /// context that the high-precision timer alarm has been fired and is now
    /// inactive.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn hp_fire_and_inactivate_alarm(&self) {
        // Safety: We need to disable the event before we transfer ownership of
        //         the alarm to the scheduling context. We disable it early, as
        //         it may take up to four cycles before this operation takes
        //         effect.
        Self::rtc().evtenclr.write(|w| w.compare0().set_bit());
        self.hp_fire_alarm();
    }

    /// Mark the high-precision timer alarm as fired.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn hp_fire_alarm(&self) {
        compiler_fence(Ordering::Release);
        self.hp_alarm
            .state
            .store(AlarmState::Fired as u8, Ordering::Relaxed);
    }

    /// Mark the high-precision timer alarm as unused. Expects the alarm to be
    /// fired.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// # Panics
    ///
    /// Panics if the alarm was not previously fired.
    fn hp_release_alarm(&self) {
        compiler_fence(Ordering::Release);
        self.hp_alarm
            .state
            .compare_exchange(
                AlarmState::Fired as u8,
                AlarmState::Unused as u8,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .unwrap();
        compiler_fence(Ordering::Acquire);
    }

    /// Reads the high-precision timer's epoch.
    ///
    /// # Safety
    ///
    /// - Shared read-only access while the high-precision timer alarm is
    ///   active.
    /// - Must not be called in any other state.
    unsafe fn timer_epoch(&self) -> NsInstant {
        debug_assert_eq!(
            self.hp_alarm.state.load(Ordering::Relaxed),
            AlarmState::Active as u8
        );
        self.timer_epoch.get()
    }

    /// Sets the high-precision timer's epoch from an overflow-protected RTC
    /// tick.
    ///
    /// # Safety
    ///
    /// - Writable from scheduling context while the high-precision timer alarm
    ///   is neither synchronizing nor active.
    /// - Writable from interrupt context while the alarm is synchronizing.
    /// - Must not be called in active state.
    unsafe fn set_timer_epoch(&self, rtc_tick: u64) {
        debug_assert!(
            self.is_hp_alarm_pending() || self.is_rtc_pending_immediate_synchronization()
        );
        self.timer_epoch
            .set(TickConversion::rtc_tick_to_instant(rtc_tick));
    }

    /// Retrieve a captured timer value and disables the corresponding PPI
    /// channel. If the event was not observed, the function returns zero and
    /// leaves the PPI channel enabled.
    fn timer_get_and_clear_captured_ticks(&self, event: HardwareEvent) -> u32 {
        let event_timer_channel = Self::timer_event_channel(event);

        // Safety: Reading a single 32 bit register is atomic.
        let mut timer_ticks = Self::timer().cc[event_timer_channel].read().bits();

        if timer_ticks > 0 {
            // An event has been observed and the corresponding PPI channel may
            // be re-used.
            let event_ppi_channel = self.ppi_event_channel(event);
            let event_ppi_channel_mask = 1 << event_ppi_channel;
            Self::ppi()
                .chenclr
                .write(|w| unsafe { w.bits(event_ppi_channel_mask) });

            // Account for PPI delay, see nRF52840 PS, section 6.16: The events we
            // observe will be delayed by up to one 16 MHz clock period (i.e. one
            // timer tick). This is not exact science as radio events may not be
            // aligned to the 16 MHz clock. The excess probabilistic reduction
            // somewhat counters the probabilistic delay introduced by RTC-to-timer
            // synchronization, so at least errors shouldn't add up.
            timer_ticks -= 1;
        }

        timer_ticks
    }

    // Called exclusively from interrupt context.
    fn on_rtc_interrupt(&self) {
        // Performance:
        // - As the SWI runs at a lower priority than the RTC interrupt handler,
        //   order doesn't matter within this handler.

        let rtc = Self::rtc();

        if rtc.events_ovrflw.read().events_ovrflw().bit_is_set() {
            rtc.events_ovrflw.reset();
            self.rtc_increment_half_period();
        }

        if rtc.events_compare[RTC_CC_HALF_PERIOD]
            .read()
            .events_compare()
            .bit_is_set()
        {
            rtc.events_compare[RTC_CC_HALF_PERIOD].reset();
            self.rtc_increment_half_period();
        }

        if rtc.events_compare[RTC_CC_HFXO_PREWARM]
            .read()
            .events_compare()
            .bit_is_set()
            && self.prewarm_state.load(Ordering::Relaxed) == PrewarmState::Armed as u8
        {
            compiler_fence(Ordering::Acquire);
            // Safety: While the pre-warm is armed its tick was written once
            //         before the state store and is read-only in both contexts
            //         afterwards.
            let prewarm_tick = self.prewarm_tick.get();
            if self.rtc_now_tick() < prewarm_tick {
                rtc.events_compare[RTC_CC_HFXO_PREWARM].reset();
            } else {
                let clock = Self::clock();
                if clock.hfclkrun.read().status().is_not_triggered() {
                    super::start_hf_oscillator(&clock);
                }

                self.rtc_release_prewarm();
            }
        }

        if rtc.events_compare[RTC_CC_SLEEP_QUEUE]
            .read()
            .events_compare()
            .bit_is_set()
        {
            self.service_sleep_queue();
        }
    }

    // Called exclusively from interrupt context.
    fn on_timer_interrupt(&self) {
        let timer = Self::timer();

        if self.is_rtc_pending_immediate_synchronization() {
            self.timer_synchronize();

            // While we synchronize, no other interrupts must be enabled.
            debug_assert_eq!(timer.intenset.read().bits(), 0);

            return;
        }

        let enabled_interrupts_mask = timer.intenset.read().bits();

        // Ensure that we've been triggered by a capture event from appropriate
        // timer channels which asserts that we own shared memory.
        debug_assert_ne!(enabled_interrupts_mask & (0b11 << 16), 0);
        debug_assert_eq!(enabled_interrupts_mask & !(0b11 << 16), 0);

        // Disable all interrupts so the interrupt won't be pended again. We
        // leave the compare event active to ensure that subsequent invocations
        // of the wait method won't require interrupt execution.

        // Safety: The intenclr and intenset registers have the same layout.
        timer
            .intenclr
            .write(|w| unsafe { w.bits(enabled_interrupts_mask) });

        // Interrupt execution is atomic from the perspective of the scheduling
        // context, therefore we continue to own the waker even with the
        // interrupt now cleared.
        let waker = unsafe { self.hp_alarm.waker.get().as_mut() }
            .unwrap()
            .take();
        if let Some(waker) = waker {
            waker.wake();
        }
    }

    /// Calculate the timestamp from the period count and the tick count.
    ///
    /// The RTC counter is 24 bit. Ticking at 32768 Hz, it overflows every ~8
    /// minutes. This is too short. We must protect it against overflow.
    ///
    /// The obvious way would be to count overflow periods. Every time the
    /// counter overflows, increase a `periods` variable. `now()` simply does
    /// `periods << 24 + counter`. So, the logic around an overflow would look
    /// like this:
    ///
    /// ```not_rust
    /// periods = 1, counter = 0xFF_FFFE --> now = 0x1FF_FFFE
    /// periods = 1, counter = 0xFF_FFFF --> now = 0x1FF_FFFF
    /// **OVERFLOW**
    /// periods = 2, counter = 0x00_0000 --> now = 0x200_0000
    /// periods = 2, counter = 0x00_0001 --> now = 0x200_0001
    /// ```
    ///
    /// The problem is that this is vulnerable to race conditions if `now()`
    /// runs at the exact time an overflow happens.
    ///
    /// If `now()` reads `periods` first and `counter` later, and overflow
    /// happens between the reads, it would return a wrong value:
    ///
    /// ```not_rust
    /// periods = 1 (OLD), counter = 0x00_0000 (NEW) --> now = 0x100_0000 -> WRONG
    /// ```
    ///
    /// It fails similarly if it reads `counter` first and `periods` second.
    ///
    /// To fix this, we define a "half period" to be 2^23 ticks (instead of
    /// 2^24). One "overflow cycle" is 2 periods.
    ///
    /// - `half period` is incremented on overflow (at counter value 0)
    /// - `half period` is incremented "midway" between overflows (at counter
    ///   value 0x80_0000)
    ///
    /// Therefore, when `half period` is even, the counter is expected to be in
    /// the range 0..0x7f_ffff, when odd, in the range 0x80_0000..0xff_ffff.
    ///
    /// To get `now()`, the `half period` is read first, then the `counter`. If
    /// the counter value range matches the expected `half period` parity, we're
    /// done. If it doesn't, we know that a new half period has started between
    /// reading `period` and `counter`. We then assume that the `counter` value
    /// corresponds to the next half period.
    ///
    /// The `half period` has 32 bits and a single half period is represented by
    /// 23 bits. The counter ticks at 32768 Hz. The overflow protected counter
    /// therefore wraps after (2^55-1) / 32768 seconds of uptime, which
    /// corresponds to ~34865 years.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn rtc_now_tick(&self) -> u64 {
        let half_period = self.half_period.load(Ordering::Relaxed);
        compiler_fence(Ordering::SeqCst);
        let counter = Self::rtc().counter.read().counter().bits();
        ((half_period as u64) << 23) + ((counter ^ ((half_period & 1) << 23)) as u64)
    }

    // Called exclusively from interrupt context.
    fn rtc_increment_half_period(&self) {
        let next_half_period = self.half_period.fetch_add(1, Ordering::Relaxed) + 1;
        // Note: The acquire part of the fence protects the read to the alarm's
        //       RTC tick below. The release part ensures that the updated
        //       period becomes visible to all clients. Inside the interrupt
        //       this fence is not strictly necessary but we add it as it is
        //       essentially free, documents intent and protects us from UB.
        compiler_fence(Ordering::AcqRel);
        let next_half_period_start_tick = (next_half_period as u64) << 23;

        // Only the high-precision timer start (CC0) may be armed beyond the
        // current three-quarters window: the sleep queue clamps its target to
        // within it (see `rearm`) and the pre-warm rejects longer leads (see
        // `arrange_hfclk_prewarm`).
        //
        // Safety: Ensure that we own the alarm before accessing it.
        if self.is_hp_alarm_owned_by_interrupt() {
            // Safety: The call to `is_hp_alarm_owned_by_interrupt()`
            //         atomically acquires the RTC tick value and ensures
            //         exclusive access.
            let pending_rtc_tick = unsafe { self.hp_alarm_tick() };
            if pending_rtc_tick < next_half_period_start_tick + Self::RTC_THREE_QUARTERS_PERIOD {
                // Just enable the compare event. The correct CC value has
                // already been set when scheduling the alarm.
                Self::rtc().evtenset.write(|w| w.compare0().set_bit());
            }
        }
    }

    /// Services the sleep-timer waiter queue: fires every due waiter and
    /// programs CC1 for the earliest remaining deadline.
    fn service_sleep_queue(&self) {
        loop {
            let mut wakers = queue::WakerBatch::new();
            let serviced = self.sleep_queue.with(|q| self.rearm(q, &mut wakers));
            wakers.wake_all();
            if serviced {
                return;
            }
        }
    }

    /// Program the sleep queue's compare (CC1) for the earliest queued
    /// deadline, firing every waiter that can no longer be armed.
    fn rearm(&self, q: &mut queue::Inner, wakers: &mut queue::WakerBatch) -> bool {
        let rtc = Self::rtc();
        loop {
            let now = self.rtc_now_tick();
            let min = match q.fire_and_min(now, Self::RTC_GUARD_TICKS, wakers) {
                queue::FireOutcome::Truncated => return false,
                queue::FireOutcome::Complete(min) => min,
            };
            let Some(min) = min else {
                rtc.intenclr.write(|w| w.compare1().set_bit());
                rtc.events_compare[RTC_CC_SLEEP_QUEUE].reset();
                return true;
            };

            let target = min.min(now + Self::RTC_THREE_QUARTERS_PERIOD);

            rtc.events_compare[RTC_CC_SLEEP_QUEUE].reset();
            rtc.cc[RTC_CC_SLEEP_QUEUE].write(|w| w.compare().variant(target as u32 & 0xFFFFFF));
            rtc.intenset.write(|w| w.compare1().set_bit());

            if self.rtc_now_tick() + Self::RTC_GUARD_TICKS <= target {
                return true;
            }
        }
    }

    // Called exclusively from scheduling context.
    #[inline(always)]
    fn hp_program_cc(&self, rtc_tick: u64) -> Result<u64, ()> {
        // Safety: Ensure that the scheduling context exclusively owns the alarm
        //         and corresponding registers.
        debug_assert!(self.is_hp_alarm_pending());

        // The nRF product spec says: If the COUNTER is N, writing N or N+1 to a
        // CC register may not trigger a COMPARE event.
        //
        // To work around this, we never program a tick smaller than N+3. N+2
        // is not safe because the RTC can tick from N to N+1 between calling
        // now() and writing to the CC register.
        let rtc_now_tick = self.rtc_now_tick();
        if rtc_tick <= rtc_now_tick + State::RTC_GUARD_TICKS {
            self.hp_fire_alarm();
            return Err(());
        }

        unsafe { self.hp_set_alarm_tick(rtc_tick) };

        // Safety: The flag must be set before enabling event routing to
        //         transfer ownership. Releases alarm memory to
        //         interrupt context.
        self.hp_activate_alarm();

        let rtc = Self::rtc();
        rtc.events_compare[RTC_CC_HP_TIMER_START].reset();
        rtc.cc[RTC_CC_HP_TIMER_START].write(|w| w.compare().variant(rtc_tick as u32 & 0xFFFFFF));

        Ok(rtc_now_tick)
    }

    #[inline(always)]
    fn hp_safely_schedule_alarm(&self, rtc_tick: u64, rtc_now_tick: u64) -> Result<(), ()> {
        let rtc = Self::rtc();

        if rtc_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD {
            // If the alarm is imminent (i.e. safely within the currently
            // running RTC period), enable the compare event right away.

            // Safety: From this point onwards we must no longer access the
            //         alarm until the alarm has been marked inactive again.
            rtc.evtenset.write(|w| w.compare0().set_bit());

            // Safety: This method may have been preempted by higher-priority
            //         interrupts. Also, its execution time depends on compiler
            //         optimization. Therefore we need to ensure that the alarm
            //         was safely scheduled _after_ enabling the corresponding
            //         event.
            let was_safely_scheduled = self.rtc_now_tick() + Self::RTC_GUARD_TICKS <= rtc_tick;
            if !was_safely_scheduled {
                // Safety: The alarm may or may not have already fired at this
                //         point. It may even spuriously fire later as disabling
                //         events is not immediate.
                self.hp_fire_and_inactivate_alarm();
                Err(())
            } else {
                Ok(())
            }
        } else {
            // If the alarm is too far into the future, don't enable the compare
            // event yet. It will be enabled by `rtc_increment_half_period()`.
            Ok(())
        }
    }

    // Called exclusively from scheduling context.
    fn ppi_program_timed_signal(
        &self,
        signal: HardwareSignal,
        disabled_by_event: Option<HardwareEvent>,
    ) -> Result<
        (
            // ppi channel mask
            NonZeroU32,
            // timer channel
            usize,
        ),
        RadioTimerError,
    > {
        let ppi = Self::ppi();
        let timer = Self::timer();

        let (ppi_channel, timer_channel) = self.timer_signal_channels(signal);
        let ppi_channel_mask = 1 << ppi_channel;

        let previous_timer_fired = timer.events_compare[timer_channel]
            .read()
            .events_compare()
            .bit_is_set();
        if previous_timer_fired {
            // The previously programmed signal was already emitted, the timer
            // is programmed not to overflow, i.e. the compare event cannot be
            // triggered again at this point. We reset the compare event but
            // leave the channel enabled so that a new compare value can be
            // programmed subsequently.
            timer.events_compare[timer_channel].write(|w| w.events_compare().clear_bit());
            debug_assert_ne!(ppi.chen.read().bits() & ppi_channel_mask, 0);
        } else {
            let timer_channel_busy = timer.cc[timer_channel].read().cc().bits() != 0;
            if timer_channel_busy {
                if matches!(disabled_by_event, Some(HardwareEvent::RadioFrameStarted))
                    && Self::radio()
                        .events_framestart
                        .read()
                        .events_framestart()
                        .bit_is_set()
                {
                    // We share a single timer channel between the disable
                    // signal and the framestarted event.
                    return Err(RadioTimerError::Already);
                };
                return Err(RadioTimerError::Busy);
            }

            // We've checked that the cc register is zero. The timer is programmed
            // not to overflow, i.e. the compare event cannot be triggered at
            // this point. Therefore we don't risk a race by enabling the
            // channel and resetting the compare event here.
            //
            // Safety: The ppi channel has been asserted to be in range and not
            //         currently in use.
            debug_assert_eq!(ppi.chen.read().bits() & ppi_channel_mask, 0);
            ppi.chenset.write(|w| unsafe { w.bits(ppi_channel_mask) });
        }

        #[cfg(feature = "timer-trace")]
        if matches!(signal, HardwareSignal::GpioToggle) {
            let ch = &ppi.ch[ppi_channel];

            let cc_event = timer.events_compare[timer_channel].as_ptr();
            ch.eep.write(|w| w.eep().variant(cc_event as u32));

            let gpiote_channel = self.gpiote_out_channel.load(Ordering::Relaxed);
            let gpiote_out_task = Self::gpiote().tasks_out[gpiote_channel].as_ptr();
            ch.tep.write(|w| w.tep().variant(gpiote_out_task as u32));
            ppi.fork[ppi_channel].tep.reset();
        }

        // Safety: The channel mask is always non-zero, see above.
        Ok((
            unsafe { NonZero::new_unchecked(ppi_channel_mask) },
            timer_channel,
        ))
    }

    fn ppi_event_channel(&self, event: HardwareEvent) -> usize {
        let ppi_channel1 = || self.ppi_channel1.load(Ordering::Relaxed);
        let ppi_channel2 = || self.ppi_channel2.load(Ordering::Relaxed);
        match event {
            HardwareEvent::RadioRxEnabled | HardwareEvent::RadioDisabled => ppi_channel1(),
            HardwareEvent::RadioFrameStarted | HardwareEvent::RadioCrcOk => ppi_channel2(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => ppi_channel2(),
        }
    }

    /// Returns the event's register address (for PPI EEP programming) and
    /// whether the event is currently latched.
    fn hardware_event_state(&self, event: HardwareEvent) -> (u32, bool) {
        let radio = Self::radio();
        match event {
            HardwareEvent::RadioRxEnabled => (
                radio.events_rxready.as_ptr() as u32,
                radio.events_rxready.read().events_rxready().bit_is_set(),
            ),
            HardwareEvent::RadioFrameStarted => (
                radio.events_framestart.as_ptr() as u32,
                radio
                    .events_framestart
                    .read()
                    .events_framestart()
                    .bit_is_set(),
            ),
            HardwareEvent::RadioDisabled => (
                radio.events_disabled.as_ptr() as u32,
                radio.events_disabled.read().events_disabled().bit_is_set(),
            ),
            HardwareEvent::RadioCrcOk => (
                radio.events_crcok.as_ptr() as u32,
                radio.events_crcok.read().events_crcok().bit_is_set(),
            ),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => {
                let gpiote_channel = self.gpiote_in_channel.load(Ordering::Relaxed);
                let event = &Self::gpiote().events_in[gpiote_channel];
                (event.as_ptr() as u32, event.read().events_in().bit_is_set())
            }
        }
    }

    // Called exclusively from scheduling context.
    fn ppi_program_event(
        &self,
        event: HardwareEvent,
        disables_signal_ppi_channel_mask: Option<NonZeroU32>,
    ) -> Result<(), RadioTimerError> {
        let ppi = Self::ppi();
        let timer = Self::timer();

        // See the high-precision timer trait implementation docs for resource
        // assignments.

        // We run in two distinct modes that allocate distinct resources.
        enum UseCase {
            EventDisablesSignal(
                // The signal to be disabled (as a mask).
                NonZeroU32,
                // The PPI channel group.
                usize,
            ),
            ObservesEvent(
                // The timer channel used to capture the event.
                usize,
            ),
        }
        use UseCase::*;

        let use_case = if let Some(disables_signal_ppi_channel_mask) =
            disables_signal_ppi_channel_mask
        {
            let disables_signal_ppi_channel_group = self.ppi_channel_group.load(Ordering::Relaxed);
            // Safety: The ppi channel group has been asserted to be in range.
            let ppi_chg_busy = ppi.chg[disables_signal_ppi_channel_group].read().bits() != 0;
            if ppi_chg_busy {
                return Err(RadioTimerError::Busy);
            }
            EventDisablesSignal(
                disables_signal_ppi_channel_mask,
                disables_signal_ppi_channel_group,
            )
        } else {
            // Corresponding signals and events (e.g. txen and tx enabled) use
            // the same timer channels. Therefore, we cannot identify duplicate
            // use by checking whether the cc register is zero: The register may
            // already contain a signal deadline.
            //
            // As we couple PPI channel 1 with CC0 and PPI channel 2 with CC1,
            // it is sufficient that we check availability of the PPI channel
            // registers instead, see below.
            ObservesEvent(Self::timer_event_channel(event))
        };

        let (event_ptr, _) = self.hardware_event_state(event);

        let event_ppi_channel = self.ppi_event_channel(event);
        let event_ppi_channel_mask = 1 << event_ppi_channel;
        let event_ch = &ppi.ch[event_ppi_channel];
        let event_fork = &ppi.fork[event_ppi_channel];

        let event_ppi_channel_enabled = ppi.chen.read().bits() & event_ppi_channel_mask != 0;
        if event_ppi_channel_enabled {
            // If the channel is already enabled it must observe the correct
            // event.
            let current_event = event_ch.eep.read().eep().bits();
            if current_event != event_ptr {
                return Err(RadioTimerError::Busy);
            }

            // The task register we're trying to program must be available.
            match use_case {
                EventDisablesSignal(..) => {
                    let disable_task = event_fork.tep.read().tep().bits();
                    if disable_task != 0 {
                        return Err(RadioTimerError::Busy);
                    }
                }
                ObservesEvent(..) => {
                    let event_capture_task = event_ch.tep.read().tep().bits();
                    if event_capture_task != 0 {
                        return Err(RadioTimerError::Busy);
                    }
                }
            }
        } else {
            event_ch.eep.write(|w| w.eep().variant(event_ptr));
            if matches!(use_case, EventDisablesSignal(..)) {
                event_ch.tep.reset();
            } else {
                event_fork.tep.reset();
            }
        }

        match use_case {
            EventDisablesSignal(
                disables_signal_ppi_channel_mask,
                disables_signal_ppi_channel_group,
            ) => {
                // Safety: The ppi channel group has been asserted to be in range and
                //         not in use. The event channel is not yet enabled.
                ppi.chg[disables_signal_ppi_channel_group]
                    .write(|w| unsafe { w.bits(disables_signal_ppi_channel_mask.get()) });
                let signal_disable_task = ppi.tasks_chg[disables_signal_ppi_channel_group]
                    .dis
                    .as_ptr();

                // Safety:
                // - The fork must be programmed _after_ the signal channel was already
                //   enabled: This ensures that we don't race to enable the signal
                //   channel if the event occurs after enabling the fork.
                // - The fork must be programmed _before_ the signal may actually
                //   be triggered (i.e. the compare register must be zero): This ensures
                //   that we don't race to enable the fork before the signal is
                //   triggered.
                event_fork
                    .tep
                    .write(|w| w.tep().variant(signal_disable_task as u32));
            }
            ObservesEvent(timer_channel) => {
                let event_capture_task = timer.tasks_capture[timer_channel].as_ptr();
                event_ch
                    .tep
                    .write(|w| w.tep().variant(event_capture_task as u32));
            }
        }

        if !event_ppi_channel_enabled {
            // Safety: The ppi channel is in range an not in use. At this point
            //         eep, ch.tep and fork.tep are guaranteed to be correctly
            //         programmed.
            ppi.chenset
                .write(|w| unsafe { w.bits(event_ppi_channel_mask) });
        }

        // Now that the fork is in place and the channel enabled we can check
        // whether the event may have been observed before we programmed the
        // fork and then disable the signal manually.
        let (_, event_has_been_observed) = self.hardware_event_state(event);

        if event_has_been_observed {
            match use_case {
                EventDisablesSignal(_, disables_signal_ppi_channel_group) => {
                    // The event had been observed before we programmed the
                    // fork: We have to disable the signal manually.
                    ppi.tasks_chg[disables_signal_ppi_channel_group]
                        .dis
                        .write(|w| w.dis().set_bit());
                }
                ObservesEvent(_) => return Err(RadioTimerError::Already),
            }
        }

        Ok(())
    }

    // Called exclusively from interrupt context.
    // The high-precision timer must be synchronizing.
    fn timer_synchronize(&self) {
        let rtc_tick = self.rtc_now_tick();

        // We capture the timer at each tick event to ensure that we read
        // the counter value before it ticked again. If we captured zero,
        // then we can be sure that the rtc tick is valid. Note that there
        // is no way to capture the RTC counter directly.
        let timer = Self::timer();
        assert_eq!(timer.cc[0].read().cc().bits(), 0);

        // Safety: If we're not tracing, the tick interrupt is only used to
        //         synchronize the timer with the RTC.
        #[cfg(not(feature = "timer-trace"))]
        Self::rtc().evtenclr.write(|w| w.tick().set_bit());

        // Clean up timer registers that we used for synchronization.
        timer.intenclr.write(|w| w.compare1().set_bit());
        timer.events_compare[1].reset();
        timer.cc[1].reset();

        // Safety: The channel has been asserted to be in range.
        let ppi = Self::ppi();
        let ppi_channel = self.ppi_channel1.load(Ordering::Relaxed);
        ppi.chenclr.write(|w| unsafe { w.bits(1 << ppi_channel) });

        // Safety: This method is only called while the alarm is synchronizing.
        //         Therefore interrupt context has exclusive write access.
        unsafe { self.set_timer_epoch(rtc_tick) };
        self.hp_activate_alarm();

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_start_hp_timer(NsInstant::from_ticks(0), rtc_tick as u32);
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_try_start_at_rtc_tick(&self, rtc_tick: u64) -> Result<(), ()> {
        // Safety: We're exclusively called from scheduling context which
        //         currently owns the high-precision timer alarm channel, see
        //         the check in hp_program_cc() which must be called _after_.
        unsafe { self.set_timer_epoch(rtc_tick) };
        let rtc_now_tick = self.hp_program_cc(rtc_tick)?;

        // Safety: This is a pre-programmed PPI channel.
        Self::ppi()
            .chenset
            .write(|w| unsafe { w.bits(1 << Self::RTC_CC0_TIMER_START_CHANNEL) });

        self.hp_safely_schedule_alarm(rtc_tick, rtc_now_tick)
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_start_at_next_rtc_tick(&self) {
        debug_assert!(self.is_hp_alarm_pending());

        // The immediate-start path never programs CC0, but
        // `rtc_increment_half_period` reads the alarm tick whenever the alarm
        // is interrupt-owned (synchronizing or active) to decide whether to
        // enable the CC0 compare event. Park the tick at the far future so a
        // value left over from a previous timed session cannot spuriously
        // re-enable a stale CC0 compare, upholding the documented alarm-tick
        // ownership invariant.
        // Safety: The alarm is pending, i.e. owned by this scheduling
        //         context; the value is released to interrupt context by
        //         `rtc_start_immediate_timer_synchronization` below.
        unsafe { self.hp_set_alarm_tick(u64::MAX) };

        let rtc = Self::rtc();
        let timer = Self::timer();

        let ppi = Self::ppi();
        let ppi_channel = self.ppi_channel1.load(Ordering::Relaxed);
        let ch = &ppi.ch[ppi_channel];

        // Mask the tick event while we're setting synchronization up to avoid
        // race conditions.
        #[cfg(feature = "timer-trace")]
        rtc.evtenclr.write(|w| w.tick().set_bit());

        ch.eep
            .write(|w| w.eep().variant(rtc.events_tick.as_ptr() as u32));
        ch.tep
            .write(|w| w.tep().variant(timer.tasks_start.as_ptr() as u32));

        // To ensure that we read the RTC counter value before it ticks again
        // we'll check that the timer's capture register is still zero after we
        // read the counter. Note that the tick event occurs 5 16MHz clock ticks
        // before the counter register actually wraps.
        debug_assert_eq!(timer.cc[0].read().cc().bits(), 0);
        ppi.fork[ppi_channel]
            .tep
            .write(|w| w.tep().variant(timer.tasks_capture[0].as_ptr() as u32));

        // Safety: The channel has been asserted to be in range.
        ppi.chenset.write(|w| unsafe { w.bits(1 << ppi_channel) });

        // We trigger from the timer's compare interrupt rather than from the
        // rtc's tick interrupt. This allows us to avoid a race condition when
        // the tick comes precisely between ungating the tick event line and
        // enabling the tick interrupt: One fires and the other not in that
        // case. Thus, un-gating the RTC event line will be the single atomic
        // operation that activates the combined process.
        timer.cc[1].write(|w| w.cc().variant(1));
        debug_assert!(timer.events_compare[1]
            .read()
            .events_compare()
            .bit_is_clear());
        timer.intenset.write(|w| w.compare1().set_bit());

        // Safety: Alarm ownership must be transferred to interrupt context
        //         before enabling event routing.
        self.rtc_start_immediate_timer_synchronization();
        rtc.evtenset.write(|w| w.tick().set_bit());
    }

    fn timer_signal_channels(
        &self,
        signal: HardwareSignal,
    ) -> (
        // PPI channel
        usize,
        // timer channel
        usize,
    ) {
        // See the high-precision timer trait implementation docs for resource
        // assignments.
        match signal {
            HardwareSignal::RadioRxEnable => (Self::TIMER_CC0_RADIO_RXEN_CHANNEL, 0),
            HardwareSignal::RadioTxEnable => (Self::TIMER_CC0_RADIO_TXEN_CHANNEL, 0),
            HardwareSignal::RadioDisable => (Self::TIMER_CC1_RADIO_DISABLE_CHANNEL, 1),
            #[cfg(feature = "timer-trace")]
            HardwareSignal::GpioToggle => (self.ppi_channel1.load(Ordering::Relaxed), 0),
            HardwareSignal::RadioTxStart => (self.ppi_txstart_channel.load(Ordering::Relaxed), 0),
            HardwareSignal::RadioTxRamp => (self.ppi_txramp_channel.load(Ordering::Relaxed), 1),
        }
    }

    // Called exclusively from scheduling context.
    fn timer_event_channel(event: HardwareEvent) -> usize {
        // See the high-precision timer trait implementation docs for resource
        // assignments.
        match event {
            HardwareEvent::RadioRxEnabled | HardwareEvent::RadioDisabled => 0,
            HardwareEvent::RadioFrameStarted | HardwareEvent::RadioCrcOk => 1,
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => 1,
        }
    }

    fn timer_safely_schedule_signal(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        timer_channel: usize,
        signal_ppi_channel_mask: NonZeroU32,
    ) -> Result<(), RadioTimerError> {
        let timer = Self::timer();

        // Setting the compare register atomically activates the signal.
        timer.cc[timer_channel].write(|w| w.cc().variant(timer_ticks.get()));

        // We capture the timer value _after_ activating the signal to check for
        // race conditions (see below).
        timer.tasks_capture[Self::TIMER_TEST_CHANNEL].write(|w| w.tasks_capture().set_bit());
        let timer_now = timer.cc[Self::TIMER_TEST_CHANNEL].read().bits();

        // If the captured timer counter is less than the programmed timer tick,
        // we safely scheduled the timed signal into the future. This is the
        // expected case and should therefore be checked first.
        if timer_now < timer_ticks.get() {
            return Ok(());
        }

        // Otherwise we may encounter two situations:
        if timer.events_compare[timer_channel]
            .read()
            .events_compare()
            .bit_is_set()
        {
            // 1. The compare event was triggered immediately. This is ok,
            //    because then the timed signal was also correctly triggered.
            Ok(())
        } else {
            // 2. The compare event was not triggered: This means that it was
            //    programmed too late and won't be triggered any more. We can
            //    safely reset state w/o risking a race.
            Self::ppi()
                .chenclr
                .write(|w| unsafe { w.bits(signal_ppi_channel_mask.get()) });
            timer.cc[timer_channel].reset();
            Err(RadioTimerError::Overdue(instant))
        }
    }

    // Called exclusively from scheduling context.
    fn timer_try_schedule_signal(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        signal: HardwareSignal,
    ) -> Result<(), RadioTimerError> {
        let (signal_ppi_channel_mask, timer_channel) =
            self.ppi_program_timed_signal(signal, None)?;
        self.timer_safely_schedule_signal(
            instant,
            timer_ticks,
            timer_channel,
            signal_ppi_channel_mask,
        )
    }

    // Called exclusively from scheduling context.
    fn timer_try_schedule_signal_unless(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        signal: HardwareSignal,
        event: HardwareEvent,
    ) -> Result<(), RadioTimerError> {
        // Safety: We need to program the timed signal w/o actually allowing the
        //         timer to trigger it before we connect the event to disable
        //         the signal. See more detailed safety conditions in
        //         sub-methods.
        let (signal_ppi_channel_mask, timer_channel) =
            self.ppi_program_timed_signal(signal, Some(event))?;
        self.ppi_program_event(event, Some(signal_ppi_channel_mask))?;

        self.timer_safely_schedule_signal(
            instant,
            timer_ticks,
            timer_channel,
            signal_ppi_channel_mask,
        )
    }

    fn timer_try_schedule_signal_if(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        signal: HardwareSignal,
        event: HardwareEvent,
    ) -> Result<(), RadioTimerError> {
        let ppi = Self::ppi();
        let timer = Self::timer();

        let (signal_ppi_channel, timer_channel) = self.timer_signal_channels(signal);
        let signal_ppi_channel_mask: u32 = 1 << signal_ppi_channel;

        if timer.events_compare[timer_channel]
            .read()
            .events_compare()
            .bit_is_set()
        {
            timer.events_compare[timer_channel].write(|w| w.events_compare().clear_bit());
        } else if timer.cc[timer_channel].read().cc().bits() != 0 {
            return Err(RadioTimerError::Busy);
        }

        let group = self.ppi_channel_group.load(Ordering::Relaxed);
        if ppi.chg[group].read().bits() != 0 {
            return Err(RadioTimerError::Busy);
        }
        let gate = self.ppi_gate_channel.load(Ordering::Relaxed);
        debug_assert_eq!(ppi.chen.read().bits() & (1 << gate), 0);

        ppi.chenclr
            .write(|w| unsafe { w.bits(signal_ppi_channel_mask) });

        // (1) Safety: The group has been asserted to be in range and free.
        //     The signal channel is disabled and its compare cannot fire.
        ppi.chg[group].write(|w| unsafe { w.bits(signal_ppi_channel_mask) });

        let (event_ptr, _) = self.hardware_event_state(event);
        let gate_ch = &ppi.ch[gate];
        gate_ch.eep.write(|w| w.eep().variant(event_ptr));
        let group_enable_task = ppi.tasks_chg[group].en.as_ptr();
        gate_ch
            .tep
            .write(|w| w.tep().variant(group_enable_task as u32));
        ppi.fork[gate].tep.reset();

        ppi.chenset.write(|w| unsafe { w.bits(1 << gate) });

        timer.cc[timer_channel].write(|w| w.cc().variant(timer_ticks.get()));

        timer.tasks_capture[Self::TIMER_TEST_CHANNEL].write(|w| w.tasks_capture().set_bit());
        let timer_now = timer.cc[Self::TIMER_TEST_CHANNEL].read().bits();
        let compare_armed = timer_now < timer_ticks.get()
            || timer.events_compare[timer_channel]
                .read()
                .events_compare()
                .bit_is_set();
        if !compare_armed {
            ppi.chenclr
                .write(|w| unsafe { w.bits(1 << gate | signal_ppi_channel_mask) });
            ppi.tasks_chg[group].dis.write(|w| w.dis().set_bit());
            ppi.chg[group].reset();
            timer.cc[timer_channel].reset();
            return Err(RadioTimerError::Overdue(instant));
        }

        let (_, event_has_been_observed) = self.hardware_event_state(event);
        if event_has_been_observed && ppi.chen.read().bits() & signal_ppi_channel_mask == 0 {
            return Err(RadioTimerError::Already);
        }

        Ok(())
    }

    // Called exclusively from scheduling context.
    fn timer_try_observe_event(&self, event: HardwareEvent) -> Result<(), RadioTimerError> {
        self.ppi_program_event(event, None)
    }

    // Called exclusively from scheduling context.
    async fn timer_wait_for(&self, signal: HardwareSignal) {
        let (ppi_channel, timer_channel) = self.timer_signal_channels(signal);
        let ppi = Self::ppi();
        let ppi_channel_is_enabled = (ppi.chen.read().bits() & 1 << ppi_channel) != 0;
        assert!(ppi_channel_is_enabled);

        let timer = Self::timer();
        let event_ptr = timer.events_compare[timer_channel].as_ptr();
        let event_ok = ppi.ch[ppi_channel].eep.read().eep().bits() == event_ptr as u32;
        assert!(event_ok);

        debug_assert!(timer.intenset.read().bits() == 0);

        let timer_interrupt_mask = 1 << (16 + timer_channel);

        let cancellation_token = CancellationGuard::new(|| {
            // Safety: Inactivating the interrupt is atomic and idempotent.
            timer
                .intenclr
                .write(|w| unsafe { w.bits(timer_interrupt_mask) });
        });

        poll_fn(|cx| {
            if timer.events_compare[timer_channel]
                .read()
                .events_compare()
                .bit_is_set()
            {
                // The interrupt must either not have been set at all or reset
                // by the interrupt handler.
                debug_assert!(timer.intenset.read().bits() == 0);

                // We don't reset the event so that the method can be called
                // repeatedly.
                return Poll::Ready(());
            }

            let is_pending = timer.intenset.read().bits() != 0;
            if is_pending {
                // While the interrupt might fire we must not access shared
                // memory.
                Poll::Pending
            } else {
                // Safety: Due to the `&mut self` requirement on this method and
                //         the assurance that only a single instance of the high
                //         precision timer may be created, only a globally
                //         unique call (and therefore interrupt) may be active.
                //         We just found that no interrupt is active, so
                //         scheduling context cannot be interrupted and can
                //         safely set the waker. (This differs from our approach
                //         to synchronizing the sleep-timer waiter queue, which
                //         can be shared across tasks.)
                let scheduled_waker = unsafe { self.hp_alarm.waker.get().as_mut() }.unwrap();
                if let Some(scheduled_waker) = scheduled_waker {
                    scheduled_waker.clone_from(cx.waker());
                } else {
                    *scheduled_waker = Some(cx.waker().clone());
                }

                // Ensure that setting the waker happens-before enabling the
                // interrupt.
                compiler_fence(Ordering::Release);

                // If the timer fired between our initial check of the event
                // flag and now, the interrupt will fire immediately, i.e. we
                // don't cause a race although we don't check the event flag
                // again.
                timer
                    .intenset
                    .write(|w| unsafe { w.bits(timer_interrupt_mask) });
                Poll::Pending
            }
        })
        .await;

        cancellation_token.inactivate();
    }
}

static STATE: State = State::new();

/// PPI channel carrying the deferred HFXO pre-warm (borrowed RTC compare ->
/// CLOCK TASKS_HFCLKSTART).
pub const HFXO_PREWARM_PPI_CHANNEL: usize = State::RTC_HFCLKSTART_PPI_CHANNEL;

#[interrupt]
fn RTC2() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_enter();

    STATE.on_rtc_interrupt();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_exit();
}

#[interrupt]
fn TIMER0() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_enter();

    STATE.on_timer_interrupt();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_exit();
}

/// Debug guard for the [`RadioTimerApi::wait_until`] safety condition: the
/// calling context must run below the RTC interrupt priority.
#[inline]
fn debug_assert_below_rtc_priority() {
    #[cfg(debug_assertions)]
    {
        let priority = match SCB::vect_active() {
            VectActive::ThreadMode => return,
            // Safety: Reading a single NVIC IPR byte.
            VectActive::Interrupt { irqn } => unsafe { (*NVIC::PTR).ipr[irqn as usize].read() },
            VectActive::Exception(exception) => {
                let shpr_index = exception.irqn() + 12;
                if shpr_index >= 0 {
                    // Safety: Reading a single SHPR byte at a checked index.
                    unsafe { (*SCB::PTR).shpr[shpr_index as usize].read() }
                } else {
                    0
                }
            }
        };
        debug_assert!(
            priority > RTC_IRQ_PRIORITY.to_arm_nvic_repr(),
            "wait_until() polled at or above the RTC interrupt priority"
        );
    }
}

/// The future returned by [`NrfRadioSleepTimer::wait_until`].
struct WaitUntil {
    node: queue::Node,
    _pin: PhantomPinned,
}

// Safety: Every access to `node` happens under the queue lock (see
//         [`SleepQueue`]), which any context polling or dropping this future
//         is allowed to take per the `wait_until` safety conditions; the
//         contained `Waker` is `Send + Sync`. The future may therefore move
//         between tasks and executors, unlike its predecessor.
unsafe impl Send for WaitUntil {}

impl WaitUntil {
    fn new(instant: NsInstant, rtc_tick: u64) -> Self {
        Self {
            node: queue::Node::new(rtc_tick, instant),
            _pin: PhantomPinned,
        }
    }
}

impl Future for WaitUntil {
    type Output = Result<(), RadioTimerError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: We only take the node's address and never move out of it.
        //         References into the node are formed under the queue lock
        //         only and never kept alive across an unlocked region: once
        //         the node is linked, the interrupt may mutate it through the
        //         queue's pointers whenever the lock is free.
        let node: *mut queue::Node = &mut unsafe { self.get_unchecked_mut() }.node;

        let (result, displaced, linked) = STATE.sleep_queue.with(|q| {
            // Safety: The lock is held, see above.
            let node = unsafe { &mut *node };
            match node.state() {
                queue::NodeState::Fired(result) => (Poll::Ready(result), None, false),
                queue::NodeState::Unlinked => {
                    if node.tick() <= STATE.rtc_now_tick() + State::RTC_GUARD_TICKS {
                        let result = Err(RadioTimerError::Overdue(node.instant()));
                        node.fire_unlinked(result);
                        (Poll::Ready(result), None, false)
                    } else {
                        let displaced = node.set_waker(cx.waker());
                        q.link(node);
                        (Poll::Pending, displaced, true)
                    }
                }
                queue::NodeState::Linked => {
                    (Poll::Pending, node.set_waker(cx.waker()), false)
                }
            }
        });
        drop(displaced);

        if linked {
            STATE.service_sleep_queue();

            return STATE.sleep_queue.with(|_q| {
                // Safety: The lock is held, see above.
                match unsafe { &*node }.state() {
                    queue::NodeState::Fired(result) => Poll::Ready(result),
                    _ => Poll::Pending,
                }
            });
        }
        result
    }
}

impl Drop for WaitUntil {
    fn drop(&mut self) {
        let was_linked = STATE.sleep_queue.with(|q| {
            if matches!(self.node.state(), queue::NodeState::Linked) {
                q.unlink(&mut self.node);
                true
            } else {
                false
            }
        });
        if was_linked {
            STATE.service_sleep_queue();
        }
    }
}

// Tick-to-ns conversion (and back).
struct TickConversion;
impl TickConversion {
    const NS_PER_S: u128 = 1_000_000_000;

    // The max number of RTC ticks representable in nanoseconds (~584 years):
    // max_ticks = ((2^64-1) ns / 10^9 ns/s) * rtc_frequency
    const MAX_RTC_INSTANT: NsInstant = NsInstant::from_ticks(u64::MAX);
    const MAX_RTC_TICKS: u64 = ((Self::MAX_RTC_INSTANT.ticks() as u128
        * NrfRadioSleepTimer::FREQUENCY.to_Hz() as u128)
        / Self::NS_PER_S) as u64;

    // The max duration convertible with a maximum rounding error of one tick at
    // 64 bit precision. See inline comments in `duration_to_timer_ticks()` for
    // more details.
    const MAX_TIMER_DURATION: NsDuration = NsDuration::micros(33554432);

    #[inline(always)]
    const fn instant_to_rtc_tick(instant: NsInstant) -> u64 {
        // To keep ns-to-tick conversion cheap we avoid division while
        // minimizing rounding errors:
        //
        // rtc_ticks = (timestamp_ns / (10^9 ns/s)) * rtc_frequency_hz
        //           = (timestamp_ns / (10^9 ns/s)) * 32768 Hz
        //           = timestamp_ns * (2^15 / 10^9 ns)
        //           = timestamp_ns * (2^6 / 5^9 ns)
        //           = timestamp_ns * ((2^6 * 2^N) / (5^9 * 2^N ns))
        //           = (timestamp_ns * (2^(6+N) / 5^9 ns)) >> N
        //           = (timestamp_ns * M(N)) >> N where M(N) := 2^(6+N) / 5^9 ns
        //
        // We can now choose M(N) such that it provides maximum precision, i.e.
        // the largest N is chosen such that timestamp_ns_max * M(N) remains
        // representable. Calculating in 64 bits is not possible as we want to
        // be able to convert u64::MAX. It turns out that the largest N
        // representable in 128 bits is 78.

        const N: u32 = 78;
        const MULTIPLIER: u128 = 2_u128.pow(6 + N) / 5_u128.pow(9);

        // Safety: We asserted above that the max representable instant in
        //         nanoseconds times the MULTIPLIER does not overflow.
        let integer_fraction = instant.ticks() as u128 * MULTIPLIER;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        (integer_fraction >> N) as u64
    }

    /// Like [`Self::instant_to_rtc_tick`], but rounds *up*: the returned tick
    /// is the smallest one whose instant is not before `instant`.
    #[inline(always)]
    const fn instant_to_rtc_tick_ceil(instant: NsInstant) -> u64 {
        let mut rtc_tick = Self::instant_to_rtc_tick(instant);

        const _: () = assert!(NrfRadioSleepTimer::FREQUENCY.to_Hz() == 2_u64.pow(15));
        while (rtc_tick as u128) * 5_u128.pow(9) < (instant.ticks() as u128) << 6 {
            rtc_tick += 1;
        }

        rtc_tick
    }

    #[inline(always)]
    const fn duration_to_timer_ticks(duration: NsDuration) -> u32 {

        // To keep ns-to-tick conversion cheap we avoid division while
        // compromising between range and rounding error:
        //
        // timer_ticks = (timestamp_ns / (10^9 ns/s)) * timer_frequency_hz
        //             = (timestamp_ns / (10^9 ns/s)) * 16 MHz
        //             = timestamp_ns * ((2^4 * 10^6) / (2^3 * 5^3 * 10^6) ns)
        //             = timestamp_ns * (2 / 5^3 ns)
        //             = timestamp_ns * ((2 * 2^N) / (5^3 * 2^N ns))
        //             = (timestamp_ns * (2^(1+N) / 5^3 ns)) >> N
        //             = (timestamp_ns * M(N)) >> N where M(N) := 2^(1+N) / 5^3 ns
        //
        //
        // As 128 bit multiplication is much more expensive than 64 bit
        // multiplication and high-precision timer activation should not exceed
        // a few seconds anyway, we choose M(N) and timestamp_ns_max such that
        // timestamp_ns_max * M(N) remains representable in 64 bit while
        // rounding error is not more than a single tick over the full range.
        //
        // A value of N:=35 and timestamp_ns_max ~ 33 seconds is the best
        // compromise that can be achieved under such restrictions.
        //
        // The conversion rounds to *nearest*, not down. Truncation used to bias
        // every timed signal early by half a tick on average (-31.25 ns) and,
        // worse, by a full tick for exact tick multiples 
        // 
        // Rounding to nearest requires the multiplier to be rounded *up*: with
        // a floored multiplier the product for an exact tick multiple lands
        // infinitesimally below the integer, so adding half an ulp would still
        // not reach it. Rounding M up bounds its own error at +0.12 ticks over
        // the full 33 s range and at well below a nanosecond over a slotframe.

        const N: u32 = 35;
        const MULTIPLIER: u128 = (2_u128.pow(1 + N) + 5_u128.pow(3) - 1) / 5_u128.pow(3);
        const HALF_ULP: u128 = 1 << (N - 1);

        // Safety: The max timer duration was chosen so that multiplying it with
        //         the MULTIPLIER does not overflow.
        let integer_fraction = duration.ticks() as u128 * MULTIPLIER + HALF_ULP;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        (integer_fraction >> N) as u32
    }

    #[inline(always)]
    const fn instant_to_timer_ticks_with_epoch(timer_epoch: NsInstant, instant: NsInstant) -> u32 {
        Self::duration_to_timer_ticks(NsDuration::from_ticks(
            instant.ticks().saturating_sub(timer_epoch.ticks()),
        ))
    }

    #[inline(always)]
    const fn rtc_tick_to_instant(rtc_tick: u64) -> NsInstant {
        debug_assert!(rtc_tick <= Self::MAX_RTC_TICKS);

        // To keep tick-to-ns conversion cheap we avoid division:
        //
        // timestamp_ns = ticks * (1 / rtc_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 32768 Hz) * 10^9 ns/s
        //              = (ticks * (10^9 / 2^15)) ns
        //              = (ticks * (5^9 / 2^6)) ns
        //              = ((ticks * 5^9) >> 6) ns
        const _: () = assert!(NrfRadioSleepTimer::FREQUENCY.to_Hz() == 2_u64.pow(15));

        // Safety: Representing MAX_RTC_TICKS requires 50 bits. Multiplying by
        //         5^9 requires another 21 bits. We therefore have to calculate
        //         in 128 bits to ensure that the calculation cannot overflow.
        const MULTIPLIER: u128 = 5_u128.pow(9);
        let ns = (rtc_tick as u128 * MULTIPLIER) >> 6;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        NsInstant::from_ticks(ns as u64)
    }

    #[inline(always)]
    const fn timer_ticks_to_duration(timer_ticks: u32) -> NsDuration {
        // To keep tick-to-ns conversion cheap we avoid division:
        //
        // timestamp_ns = ticks * (1 / timer_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 16 MHz) * 10^9 ns/s
        //              = (ticks * ((2^3 * 5^3 * 10^6) / (2^4 * 10^6))) ns
        //              = (ticks * (5^3 / 2)) ns
        //              = ((ticks * 5^3) >> 1) ns

        // Safety: Representing the timer ticks requires 32 bits. Multiplying by
        //         5^3 requires another 7 bits. Calculating in 64 bits is
        //         therefore sufficient to ensure that the calculation cannot
        //         overflow. Note that the max timer ticks represent ~268s.
        const MULTIPLIER: u64 = 5_u64.pow(3);
        let ns = (timer_ticks as u64 * MULTIPLIER) >> 1;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        NsDuration::from_ticks(ns)
    }

    #[inline(always)]
    const fn timer_ticks_to_instant_with_epoch(
        timer_epoch: NsInstant,
        timer_ticks: u32,
    ) -> NsInstant {
        let timer_duration = Self::timer_ticks_to_duration(timer_ticks);
        timer_epoch.checked_add_duration(timer_duration).unwrap()
    }
}

// Test conversion.
//
// Note: We do this in a const expression rather than a test so that we can also
//       prove proper "constification" of the conversion functions.
const _: () = {
    let max_rtc_tick = TickConversion::instant_to_rtc_tick(NsInstant::from_ticks(u64::MAX));
    assert!(max_rtc_tick == TickConversion::MAX_RTC_TICKS);

    // One RTC tick is ~30517 ns, the rounding error must be less.
    const EXPECTED_REMAINDER_RTC_NS: u64 = 17924;
    let rtc_tick_ns = TickConversion::rtc_tick_to_instant(max_rtc_tick).ticks();
    assert!(rtc_tick_ns == u64::MAX - EXPECTED_REMAINDER_RTC_NS);

    const EXPECTED_REMAINDER_TIMER_TICKS: u32 = ((NrfRadioHighPrecisionTimer::FREQUENCY.to_Hz()
        * EXPECTED_REMAINDER_RTC_NS
        + TickConversion::NS_PER_S as u64 / 2)
        / TickConversion::NS_PER_S as u64) as u32;
    let timer_ticks =
        TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(EXPECTED_REMAINDER_RTC_NS));
    assert!(timer_ticks == EXPECTED_REMAINDER_TIMER_TICKS);

    const EXPECTED_REMAINDER_TIMER_NS: i64 = -13;
    let timer_ticks_ns = TickConversion::timer_ticks_to_duration(timer_ticks).ticks();
    assert!(
        (u64::MAX - rtc_tick_ns) as i64 - timer_ticks_ns as i64 == EXPECTED_REMAINDER_TIMER_NS
    );
    assert!(EXPECTED_REMAINDER_TIMER_NS > -32 && EXPECTED_REMAINDER_TIMER_NS < 32);

    const EXPECTED_MAX_TIMER_TICKS: u32 = ((TickConversion::MAX_TIMER_DURATION.ticks()
        * NrfRadioHighPrecisionTimer::FREQUENCY.to_Hz())
        / TickConversion::NS_PER_S as u64) as u32;
    let max_timer_ticks = TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(
        TickConversion::MAX_TIMER_DURATION.ticks(),
    ));
    assert!(max_timer_ticks == EXPECTED_MAX_TIMER_TICKS);

    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(875)) == 14);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(1_000)) == 16);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(2_120_000)) == 33_920);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(1_020_000)) == 16_320);

    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(0)) == 0);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(31)) == 0);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(32)) == 1);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(93)) == 1);
    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(94)) == 2);

    let epoch = NsInstant::from_ticks(1_000_000);
    assert!(TickConversion::instant_to_timer_ticks_with_epoch(epoch, NsInstant::from_ticks(999)) == 0);
    assert!(TickConversion::instant_to_timer_ticks_with_epoch(epoch, epoch) == 0);
    assert!(
        TickConversion::instant_to_timer_ticks_with_epoch(epoch, NsInstant::from_ticks(1_000_094))
            == 2
    );

    assert!(TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(0)) == 0);
    assert!(TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(1)) == 1);
    assert!(TickConversion::instant_to_rtc_tick(NsInstant::from_ticks(1_953_125)) == 63);
    assert!(TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(1_953_124)) == 64);
    assert!(TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(1_953_125)) == 64);
    assert!(TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(1_953_126)) == 65);
    assert!(TickConversion::instant_to_rtc_tick(NsInstant::from_ticks(1_000_000_000)) == 32767);
    assert!(
        TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(1_000_000_000)) == 32768
    );
    assert!(
        TickConversion::instant_to_rtc_tick_ceil(NsInstant::from_ticks(u64::MAX))
            == TickConversion::MAX_RTC_TICKS + 1
    );
};

#[derive(Clone, Copy, Debug)]
pub struct NrfRadioSleepTimer {
    // Private field to block direct instantiation.
    _inner: (),
}

#[cfg(feature = "timer-trace")]
pub struct NrfRadioTimerTracingConfig {
    pub gpiote_out_channel: usize,
    pub gpiote_in_channel: usize,
    /// GPIOTE OUT channel toggled at the RTC alarm / TIMER-start boundary.
    pub gpiote_alarm_channel: usize,
    pub gpiote_tick_channel: usize,
    pub ppi_tick_channel: usize,
}

pub struct NrfRadioTimerConfig {
    pub rtc: RTC2,
    pub timer: TIMER0,
    // We take ownership of the clock to enforce clock policy:
    // - The radio requires an external HF crystal oscillator.
    // - The hybrid timer requires the LF oscillator to be continuously enabled.
    pub clock: CLOCK,
    pub sleep_timer_clk_src: super::LfClockSource,
    pub ppi_channels: [usize; 2],
    pub ppi_channel_group: usize,
    /// PPI channel used to gate a timed signal on a hardware event
    /// ("timed signal if event", e.g. Enh-ACK txen on CRCOK).
    pub ppi_gate_channel: usize,
    /// PPI channel for the timed TX start (TIMER CC0 -> RADIO TASKS_START).
    pub ppi_txstart_channel: usize,
    /// PPI channel for the timed TX ramp (TIMER CC1 -> RADIO TASKS_TXEN).
    pub ppi_txramp_channel: usize,
    #[cfg(feature = "timer-trace")]
    pub tracing_config: NrfRadioTimerTracingConfig,
}

impl NrfRadioSleepTimer {
    pub const FREQUENCY: TimerRateU64<32_768> = TimerRateU64::from_raw(1);

    /// Instantiate the timer for the first time. Consumes the required
    /// peripherals. Further copies can then be created.
    pub fn new(config: NrfRadioTimerConfig) -> Self {
        STATE.init(config);
        Self { _inner: () }
    }
}

impl RadioTimerApi for NrfRadioSleepTimer {
    const TICK_PERIOD: NsDuration = Self::FREQUENCY.into_duration();
    const GUARD_TIME: NsDuration = NsDuration::micros(150);

    type HighPrecisionTimer = NrfRadioHighPrecisionTimer;

    fn now(&self) -> NsInstant {
        STATE.assert_initialized();

        let rtc_tick = STATE.rtc_now_tick();
        TickConversion::rtc_tick_to_instant(rtc_tick)
    }

    /// See [`RadioTimerApi::wait_until`].
    async unsafe fn wait_until(&mut self, instant: NsInstant) -> Result<(), RadioTimerError> {
        STATE.assert_initialized();
        debug_assert_below_rtc_priority();

        let rtc_tick = TickConversion::instant_to_rtc_tick_ceil(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_rtc_alarm(instant, rtc_tick as u32);

        WaitUntil::new(instant, rtc_tick).await
    }

    fn start_high_precision_timer(
        &self,
        at: OptionalNsInstant,
    ) -> Result<Self::HighPrecisionTimer, RadioTimerError> {
        STATE.assert_initialized();

        STATE.hp_try_acquire_alarm()?;
        STATE.hfclk_request();

        let result = (|| {
            if let Some(at) = at.into() {
                // We need to start at least two timer ticks earlier so we can
                // schedule signals at non-zero ticks even accounting for a one-tick
                // PPI delay.
                const TIMER_OFFSET: NsDuration = TickConversion::timer_ticks_to_duration(2)
                    // Adding 1ns to avoid rounding errors.
                    .checked_add(NsDuration::from_ticks(1))
                    .unwrap();

                let rtc_tick = TickConversion::instant_to_rtc_tick(at - TIMER_OFFSET);

                #[cfg(feature = "rtos-trace")]
                crate::timer::trace::record_start_hp_timer(at, rtc_tick as u32);

                STATE
                    .timer_try_start_at_rtc_tick(rtc_tick)
                    .map_err(|_| RadioTimerError::Overdue(at))?;

                STATE.arrange_hfclk_prewarm(rtc_tick);
            } else {
                STATE.timer_start_at_next_rtc_tick();
                STATE.hfclk_start_blocking();
            }

            // The high-precision timer will automatically be stopped and the
            // corresponding alarm channel released when dropping the running
            // radio timer instance.
            Ok(NrfRadioHighPrecisionTimer { _inner: () })
        })();

        if result.is_err() {
            STATE.hfclk_release();
            STATE.hp_release_alarm();
        }

        result
    }
}

pub struct NrfRadioHighPrecisionTimer {
    // Private field to block direct instantiation.
    _inner: (),
}

impl NrfRadioHighPrecisionTimer {
    pub const FREQUENCY: TimerRateU64<16_000_000> = TimerRateU64::from_raw(1);

    fn timer_epoch() -> NsInstant {
        // Note: This only works if the timer interrupt is running at a higher
        //       priority than the scheduling context. Immediate synchronization
        //       blocks for at most one RTC tick.
        while STATE.is_rtc_pending_immediate_synchronization() {
            wfe();
        }
        // Safety: We waited until the timer is synchronized.
        unsafe { STATE.timer_epoch() }
    }

    fn timer_ticks(instant: NsInstant) -> u32 {
        let timer_epoch = Self::timer_epoch();
        debug_assert_ne!(timer_epoch.ticks(), 0);

        let timer_ticks = TickConversion::instant_to_timer_ticks_with_epoch(timer_epoch, instant);

        timer_ticks.saturating_sub(1)
    }
}

/// The timer implementation is a compromise between resource usage and
/// flexibility.
///
/// The radio currently requires the following combinations of signals and
/// events (read the "\" character as "unless"):
///
/// STATE | METHOD         | SIGNALS              | EVENTS
/// =========================================================================
/// off   | sched_rx       | rxen                 | rx_enabled, framestarted
/// off   | sched_tx       | txen, or tx_ramp +   | framestarted
///       |                | tx_start***          |
/// off   | switch_off     |                      | disabled
/// rx    | stop_listening | disable\framestarted | framestarted, disabled
/// rx    | sched_rx       |                      | rx_enabled*, framestarted
/// rx    | sched_tx       | disable, txen or     | framestarted
///       |                | tx_start***          |
/// rx    | sched_ack_tx   | txen if crcok        | framestarted
/// rx    | sched_off      | disable**            | disabled**
/// tx    | sched_rx       |                      | rx_enabled, framestarted
/// tx    | sched_tx       |                      | framestarted
/// tx    | sched_off      |                      | disabled
///
/// *) If an IFS is given we disable and re-enable the radio.
///
/// **) Only if a frame was observed, otherwise the disabled event has already
///     been observed by the `stop_listening()` method.
///
/// ***) "tx_start" for a timed transmission without CCA, "txen" otherwise.
///      "txen" and "tx_start" both occupy CC0, so only one of them is ever
///      armed per operation. A timed transmission from the "off" state pairs
///      "tx_start" (CC0) with "tx_ramp" (CC1) instead: the ramp has to be
///      timed as well there, because unlike the "rx" state there is no
///      DISABLED event left to hang a `disabled_txen` short on.
///
/// Additionally we implement a GPIO test mode that requires a GPIO toggle
/// signal and a GPIO toggled event, possibly used concurrently.
///
/// For the radio, we allocate the following resources:
///
/// signals:
/// - rxen/txen: pre-configured PPI-Chs 20/21 + CC0
/// - disable: pre-configured PPI-Ch 22 + CC1
/// - disable unless...: like "disable" + PPI-ChGroup
/// - txen if crcok: like "txen" + PPI-ChGroup + gate PPI-Ch
///   (crcok -> chg.en; the txen channel starts disabled inside the group)
/// - tx start: init-programmed PPI-Ch (CC0 -> RADIO.TASKS_START) + CC0, used
///   instead of "txen" for timed transmissions without CCA. The transmitter is
///   ramped up beforehand and parks in TXIDLE, so the compare alone determines
///   the on-air RMARKER.
/// - tx ramp: init-programmed PPI-Ch (CC1 -> RADIO.TASKS_TXEN) + CC1, the
///   deferred ramp that puts the transmitter into TXIDLE shortly before a
///   "tx start". Only needed from the "off" state; from "rx" the "disable"
///   signal plus the `disabled_txen` short does the same job on CC1.
///
/// events:
/// - rx_enabled/disabled: PPI-Ch1.tep + CC0
/// - framestarted: PPI-Ch2.tep + CC1
/// - ...unless framestarted: like "framestarted" + PPI-Ch2.fork
///
/// In test mode we use:
/// gpio toggle signal: PPI-Ch1 + CC0
/// gpio toggled event: PPI-Ch2 + CC1
///
/// As we only have four channels available, we need to assign several signals
/// and events to the same channels:
///
/// 1. Events will always occur after the corresponding signal, i.e. the signal
///    channel is systematically released before the event occurs. This allows
///    us to re-use the same channel for the rxen signal and the rx enabled
///    event pair (CC0).
///
/// 2. The rx enabled and disabled event pair is mutually exclusive which allows
///    us to let them occupy the same channel CC0.
///
/// 3. In the "disable unless framestart" case, CC1 initially contains the
///    timeout for the disable signal. If the framestart event triggers, it'll
///    overwrite the contents of CC1 which is then no longer needed.
///
/// We use one timer channel (CC2) to ensure that programmed timers are in the
/// future. We do so by capturing the current timer value _after_ programming
/// the signal.
///
/// The fourth timer channel (CC3) is used to ensure that the timer will never
/// overflow. It shorts a compare event for the max counter value with the stop
/// task.
///
/// We use the ppi channel enabled and timer interrupt bits to synchronize
/// resource access. This is possible as we ensure that there can be only a
/// single instance of the high-precision timer:
/// - We synchronize access to the high-precision timer via the sleep timer
///   interface.
/// - The high-precision timer cannot be cloned or copied.
///
/// Notably we have to ensure that programming a "signal unless event" plus also
/// programming an observation of the same event does not produce a race,
/// independently of the order in which these calls are made.
///
/// The following diagrams show in which order registers need to be programmed
/// to avoid races.
///
/// First consider the case where the call to `schedule_timed_signal_unless()`
/// precedes the call to `observe_event()`:
///
/// event_* ──> eep (3) --> chen (4) ─┬> tep (9) ──> task_capture
///                                   │
///                                   └> fork (3) ──> chg.task_dis ┐
///                                ┌───────────────────────────────┘
///                                ˅  
/// event_compare ──> eep (1) ──> chen (2) ──> tep (1) ──> task_*
///  ˄
///  │
/// cc (0/5)
///
/// Initially (0) the compare register responsible for triggering the signal
/// must be zero.
///
/// Then we program (1) and enable (2) the PPI channel responsible for
/// triggering the signal. As the compare register is zero and the timer cannot
/// overflow, the signal cannot fire although the PPI channel is already
/// enabled.
///
/// Next we set up (3) and enable (4) the PPI channel that captures the event
/// and disables the signal. We use the fork register of the PPI channel to
/// disable the signal via a channel group.
///
/// If the event occurs while we finish setting up the remaining registers, it
/// will disable the PPI channel that fires the signal thereby avoiding a race.
/// This is the reason why we need to enable the signal path before we set up
/// the event path.
///
/// In a last step we can now set the compare register responsible for
/// triggering the signal. This will atomically enable the signal unless the
/// compare event lies in the past or the PPI channel has already been disabled
/// by an incoming event.
///
/// The following call to "observe event" will use the event path's "tep"
/// register to atomically start capturing the event, independently of the
/// "signal unless event" path.
///
/// Now consider calling `observe_event()` before the call to
/// `schedule_timed_signal_unless()`:
///
/// event_* ──> eep (1) --> chen (2) ─┬> tep (1) ──> task_capture
///                                   │
///                                   └> fork (5) ──> chg.task_dis ┐
///                                ┌───────────────────────────────┘
///                                ˅  
/// event_compare ──> eep (3) ──> chen (4) ──> tep (3) ──> task_*
///  ˄
///  │
/// cc (0/6)
///
/// In this case the capturing task will first be connected to the event source
/// which immediately starts observing the event (1, 2).
///
/// Programming the signal is prepared like in the first case by initially
/// leaving the cc register zero (0), then setting up and enabling the signal
/// path (3, 4).
///
/// Next, setting the fork register (5) will ensure that an incoming event will
/// immediately disable the signal path. And finally, setting the compare
/// register for the signal (6) will enable it atomically unless the event had
/// occurred already.
impl HighPrecisionTimer for NrfRadioHighPrecisionTimer {
    const TICK_PERIOD: NsDuration = Self::FREQUENCY.into_duration();

    fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> Result<&Self, RadioTimerError> {
        let TimedSignal { instant, signal } = timed_signal;
        let timer_ticks = Self::timer_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_timed_signal(instant, timer_ticks, signal);

        let timer_ticks = NonZero::new(timer_ticks).ok_or(RadioTimerError::Overdue(instant))?;
        STATE
            .timer_try_schedule_signal(instant, timer_ticks, signal)
            .map(|_| self)
    }

    fn schedule_timed_signal_unless(
        &self,
        timed_signal: TimedSignal,
        event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        let TimedSignal { instant, signal } = timed_signal;
        let timer_ticks = Self::timer_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_timed_signal(instant, timer_ticks, signal);

        let timer_ticks = NonZero::new(timer_ticks).ok_or(RadioTimerError::Overdue(instant))?;
        STATE
            .timer_try_schedule_signal_unless(instant, timer_ticks, signal, event)
            .map(|_| self)
    }

    fn schedule_timed_signal_if(
        &self,
        timed_signal: TimedSignal,
        event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        let TimedSignal { instant, signal } = timed_signal;
        let timer_ticks = Self::timer_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_timed_signal(instant, timer_ticks, signal);

        let timer_ticks = NonZero::new(timer_ticks).ok_or(RadioTimerError::Overdue(instant))?;
        STATE
            .timer_try_schedule_signal_if(instant, timer_ticks, signal, event)
            .map(|_| self)
    }

    fn commit_timed_signal(&self) {
        let ppi = State::ppi();
        let group = STATE.ppi_channel_group.load(Ordering::Relaxed);
        debug_assert_ne!(ppi.chg[group].read().bits(), 0);
        ppi.tasks_chg[group].en.write(|w| w.en().set_bit());
    }

    async unsafe fn wait_for(&mut self, signal: HardwareSignal) {
        STATE.timer_wait_for(signal).await
    }

    fn observe_event(&self, event: HardwareEvent) -> Result<&Self, RadioTimerError> {
        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_observe_event(event);

        // Note: This only works if the timer interrupt is running at a higher
        //       priority than the scheduling context. Immediate synchronization
        //       blocks for at most one RTC tick.
        while STATE.is_rtc_pending_immediate_synchronization() {
            wfe();
        }

        STATE.timer_try_observe_event(event).map(|_| self)
    }

    fn poll_event(&self, event: HardwareEvent) -> OptionalNsInstant {
        let timer_ticks = STATE.timer_get_and_clear_captured_ticks(event);
        if timer_ticks > 0 {
            TickConversion::timer_ticks_to_instant_with_epoch(Self::timer_epoch(), timer_ticks)
                .into()
        } else {
            None.into()
        }
    }

    fn reset(&self) {
        let timer = State::timer();
        let ppi = State::ppi();

        // First disable all PPI channels so that no further signals and events
        // can be generated.
        ppi.chenclr
            .write(|w| unsafe { w.bits(STATE.ppi_channel_mask.load(Ordering::Relaxed)) });

        // Clean up timer resources.
        for timer_channel in 0..2 {
            timer.events_compare[timer_channel].reset();
            timer.cc[timer_channel].reset();
        }
        #[cfg(feature = "timer-trace")]
        {
            let gpiote_channel = STATE.gpiote_in_channel.load(Ordering::Relaxed);
            State::gpiote().events_in[gpiote_channel].reset();
        }

        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());

        // Release the PPI channel group.
        let ppi_channel_group = STATE.ppi_channel_group.load(Ordering::Relaxed);
        State::ppi().chg[ppi_channel_group].reset();
    }
}

impl Drop for NrfRadioHighPrecisionTimer {
    fn drop(&mut self) {
        let timer = State::timer();

        // First of all stop the timer to ensure that further signals won't be
        // generated. This is atomic and doesn't have to be synchronized.
        timer.tasks_stop.write(|w| w.tasks_stop().set_bit());

        // Atomically cancel ongoing timer synchronization and re-acquire alarm
        // memory to scheduling context in which the drop handler runs.
        STATE.hp_fire_and_inactivate_alarm();

        // In case synchronization was interrupted: Clean up registers that were
        // used for synchronization.
        #[cfg(not(feature = "timer-trace"))]
        State::rtc().evtenclr.write(|w| w.tick().set_bit());
        timer.intenclr.write(|w| w.compare1().set_bit());

        // Clean up timer state.
        self.reset();

        STATE.rtc_release_prewarm();

        // Atomically de-allocate the high precision timer.
        STATE.hp_release_alarm();
        STATE.hfclk_release();
    }
}

pub(crate) fn request_hfclk() {
    STATE.assert_initialized();
    STATE.hfclk_request();
}

pub(crate) fn release_hfclk() {
    STATE.assert_initialized();
    STATE.hfclk_release();
}

/// Start the HFXO now and block until it runs.
#[allow(dead_code)]
pub(crate) fn start_hfclk_blocking() {
    STATE.assert_initialized();
    STATE.hfclk_start_blocking();
}

/// RAII guard that keeps the HFXO (high-frequency crystal) running for as
/// long as it is held.
#[must_use = "the HFXO is released as soon as the guard is dropped"]
pub struct HfclkGuard(());

impl HfclkGuard {
    /// Request the HFXO and keep it running until the guard is dropped.
    pub fn new() -> Self {
        request_hfclk();
        STATE.hfclk_start_blocking();
        Self(())
    }
}

impl Drop for HfclkGuard {
    fn drop(&mut self) {
        release_hfclk();
    }
}
