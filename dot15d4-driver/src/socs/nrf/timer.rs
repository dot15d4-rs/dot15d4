//! Radio timer implementation for nRF SoCs.

// No need to use portable atomics in the driver as this is platform-specific
// code.
use core::{
    cell::{Cell, UnsafeCell},
    future::poll_fn,
    num::{NonZero, NonZeroU32},
    sync::atomic::{compiler_fence, AtomicU32, AtomicU8, AtomicUsize, Ordering},
    task::{Poll, Waker},
};

use cortex_m::{asm::wfe, peripheral::Peripherals as CorePeripherals};
use dot15d4_util::sync::CancellationGuard;
use fugit::TimerRateU64;
#[cfg(feature = "timer-trace")]
use nrf52840_pac::GPIOTE;
use nrf52840_pac::{interrupt, Peripherals, CLOCK, NVIC, PPI, RADIO, RTC0, TIMER0};

use crate::{
    socs::nrf::executor::NrfInterruptPriority,
    timer::{
        HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
        OptionalNsInstant, RadioTimerApi, RadioTimerError, TimedSignal,
    },
};

/// Alarm channels represent shared resources that need to be synchronized
/// between scheduling context and interrupts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmChannel {
    HighPrecisionTimer = 0,
    SleepTimer1,
    SleepTimer2,
    NumRtcChannels,
}

/// This flag atomically represents the current state of an alarm.
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
/// - Several instances of the sleep timer may concurrently access RTC alarms.
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

// Resources shared and synchronized between scheduling context and RTC
// interrupt context.
struct Alarm {
    /// The current alarm state. See [`AlarmState`] for details and safety
    /// considerations.
    state: AtomicU8,

    /// The RTC tick of a pending alarm.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    rtc_tick: Cell<u64>,

    /// Waker for the current alarm.
    ///
    /// Safety: In case of an RTC alarm, access is synchronized via the alarm
    ///         state, see [`AlarmState`] for details. This is required as
    ///         canceling the alarm races with firing (i.e.  waking) it. The
    ///         waker itself is [`Sync`]. It's therefore ok, to wake it from
    ///         interrupt context.
    ///
    /// Note that the safety conditions of the [`RadioTimerApi`] disallow
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

const NUM_RTC_CHANNELS: usize = AlarmChannel::NumRtcChannels as usize;
const RTC_CHANNELS: [AlarmChannel; NUM_RTC_CHANNELS] = [
    AlarmChannel::HighPrecisionTimer,
    AlarmChannel::SleepTimer1,
    AlarmChannel::SleepTimer2,
];

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

    /// Independent resource synchronization channels supported by the RTC
    /// driver.
    ///
    /// Safety: RTC alarms are Sync.
    alarms: [Alarm; NUM_RTC_CHANNELS],

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
    /// framestart).
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel_group: AtomicUsize,

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
    const RTC_CC0_TIMER_START_CHANNEL: usize = 31;

    const PPI_CHANNEL_MASK: u32 = 1 << Self::TIMER_CC0_RADIO_TXEN_CHANNEL
        | 1 << Self::TIMER_CC0_RADIO_RXEN_CHANNEL
        | 1 << Self::TIMER_CC1_RADIO_DISABLE_CHANNEL
        | 1 << Self::RTC_CC0_TIMER_START_CHANNEL;

    const fn new() -> Self {
        // In const-context we cannot use array::from_fn and need to initialize
        // array with non-copy values manually.
        Self {
            init_state: AtomicU8::new(InitializationState::Uninitialized as u8),
            half_period: AtomicU32::new(0),
            alarms: [Alarm::new(), Alarm::new(), Alarm::new()],
            timer_epoch: Cell::new(NsInstant::from_ticks(0)),
            #[cfg(feature = "timer-trace")]
            gpiote_out_channel: AtomicUsize::new(0),
            #[cfg(feature = "timer-trace")]
            gpiote_in_channel: AtomicUsize::new(1),
            ppi_channel1: AtomicUsize::new(0),
            ppi_channel2: AtomicUsize::new(0),
            ppi_channel_group: AtomicUsize::new(0),
            ppi_channel_mask: AtomicU32::new(0),
        }
    }

    fn nvic() -> NVIC {
        // Safety: We only use the peripheral to adjust properties of interrupts
        //         we exclusively own.
        unsafe { CorePeripherals::steal() }.NVIC
    }

    fn rtc() -> RTC0 {
        // We own the RTC peripheral exclusively.
        unsafe { Peripherals::steal() }.RTC0
    }

    fn timer() -> TIMER0 {
        // We own the TIMER peripheral exclusively.
        unsafe { Peripherals::steal() }.TIMER0
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

        super::start_hf_oscillator(&config.clock);
        super::start_lf_clock(&config.clock, config.sleep_timer_clk_src);

        let [ppi_channel1, ppi_channel2] = config.ppi_channels;
        debug_assert!(ppi_channel1 <= 19);
        debug_assert!(ppi_channel2 <= 19);
        debug_assert_ne!(ppi_channel1, ppi_channel2);
        self.ppi_channel1.store(ppi_channel1, Ordering::Relaxed);
        self.ppi_channel2.store(ppi_channel2, Ordering::Relaxed);
        self.ppi_channel_mask.store(
            1 << ppi_channel1 | 1 << ppi_channel2 | Self::PPI_CHANNEL_MASK,
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
                gpiote_tick_channel,
                ppi_tick_channel,
            } = config.tracing_config;

            debug_assert!(gpiote_out_channel <= 7);
            debug_assert!(gpiote_in_channel <= 7);
            debug_assert!(gpiote_tick_channel <= 7);
            debug_assert_ne!(gpiote_in_channel, gpiote_out_channel);
            debug_assert_ne!(gpiote_in_channel, gpiote_tick_channel);
            debug_assert_ne!(gpiote_tick_channel, gpiote_out_channel);

            debug_assert!(ppi_tick_channel <= 19);
            debug_assert_ne!(ppi_channel1, ppi_tick_channel);
            debug_assert_ne!(ppi_channel2, ppi_tick_channel);

            self.gpiote_out_channel
                .store(gpiote_out_channel, Ordering::Relaxed);
            self.gpiote_in_channel
                .store(gpiote_in_channel, Ordering::Relaxed);

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

        rtc.tasks_stop.write(|w| w.tasks_stop().set_bit());
        rtc.prescaler.reset();

        // CC3 and overflow events trigger half-period counting.
        rtc.cc[3].write(|w| w.compare().variant(0x800000));
        rtc.intenset.write(|w| {
            w.ovrflw().set_bit();
            w.compare3().set_bit()
        });

        rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
        while rtc.counter.read().counter() != 0 {}

        rtc.tasks_start.write(|w| w.tasks_start().set_bit());
        while rtc.counter.read().counter() == 0 {}

        let mut nvic = Self::nvic();

        let timer_interrupt_priority = NrfInterruptPriority::HIGHEST.one_lower().unwrap();
        // Safety: See the documented synchronization approach.
        unsafe {
            nvic.set_priority(
                interrupt::TIMER0,
                timer_interrupt_priority.to_arm_nvic_repr(),
            )
        };

        let rtc_interrupt_priority = timer_interrupt_priority.one_lower().unwrap();
        // Safety: See the documented synchronization approach.
        unsafe { nvic.set_priority(interrupt::RTC0, rtc_interrupt_priority.to_arm_nvic_repr()) };

        // Clear and enable timer interrupts.
        NVIC::unpend(interrupt::RTC0);
        NVIC::unpend(interrupt::TIMER0);
        // Safety: We're in early initialization, so there should be no
        //         concurrent critical sections.
        unsafe { NVIC::unmask(interrupt::RTC0) };
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

    /// Sets an RTC alarm's tick value.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    unsafe fn set_rtc_alarm_tick(&self, channel: AlarmChannel, rtc_tick: u64) {
        let alarm = &self.alarms[channel as usize];
        alarm.rtc_tick.set(rtc_tick);
    }

    /// Read the alarm's RTC tick.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    /// - Compiler fences are required to acquire/release this value.
    unsafe fn rtc_alarm_tick(&self, channel: AlarmChannel) -> u64 {
        self.alarms[channel as usize].rtc_tick.get()
    }

    /// Returns `true` while the RTC alarm is active or synchronizing (and
    /// therefore owned by interrupt context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_alarm_owned_by_interrupt(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Active as u8 || state == AlarmState::Synchronizing as u8
    }

    /// Returns `true` while the RTC alarm is pending (and owned by scheduling
    /// context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_alarm_pending(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
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
        let state = self.alarms[AlarmChannel::HighPrecisionTimer as usize]
            .state
            .load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Synchronizing as u8
    }

    /// Mark the RTC alarm as pending if it is unused.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn rtc_try_acquire_alarm(&self, channel: AlarmChannel) -> Result<(), RadioTimerError> {
        compiler_fence(Ordering::Release);
        let result = self.alarms[channel as usize].state.compare_exchange_weak(
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

    /// Transfer ownership of the RTC alarm to interrupt context to synchronize
    /// the high-precision timer via the "next tick" approach.
    ///
    /// Releases the high-precision timer alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn rtc_start_immediate_timer_synchronization(&self) {
        compiler_fence(Ordering::Release);
        self.alarms[AlarmChannel::HighPrecisionTimer as usize]
            .state
            .store(AlarmState::Synchronizing as u8, Ordering::Relaxed);
    }

    /// Transfer ownership of the RTC alarm to interrupt context.
    ///
    /// Releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// Note: This does _not_ also activate interrupts. These may have to remain
    ///       inactive if we've not yet reached the target timer period.
    fn rtc_activate_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Active as u8, Ordering::Relaxed);
    }

    /// Disables timer interrupts and signals to the scheduling context that the
    /// given RTC alarm has been fired and is now inactive.
    ///
    /// Transfers ownership of the RTC alarm from interrupt context to
    /// scheduling context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn rtc_fire_and_inactivate_alarm(&self, channel: AlarmChannel) {
        let rtc = Self::rtc();

        // Safety: We need to disable the interrupt before we transfer
        //         ownership of the alarm to the scheduling context. We disable
        //         the interrupt early, as it may take up to four cycles before
        //         this operation takes effect. Should the interrupt be
        //         spuriously woken it will additionally check alarm state.
        match channel {
            AlarmChannel::HighPrecisionTimer => {
                rtc.evtenclr.write(|w| w.compare0().set_bit());
            }
            AlarmChannel::SleepTimer1 => rtc.intenclr.write(|w| w.compare1().set_bit()),
            AlarmChannel::SleepTimer2 => rtc.intenclr.write(|w| w.compare2().set_bit()),
            _ => unreachable!(),
        }

        self.rtc_fire_alarm(channel);
    }

    /// Mark the RTC alarm as fired.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn rtc_fire_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Fired as u8, Ordering::Relaxed);
    }

    /// Mark the RTC alarm as unused. Expects the alarm to be fired.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// # Panics
    ///
    /// Panics if the alarm was not previously fired.
    fn rtc_release_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .compare_exchange_weak(
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
    /// - Shared read-only access while the [`RtcChannel::TimerSynchronization`]
    ///   is active.
    /// - Must not be called in any other state.
    unsafe fn timer_epoch(&self) -> NsInstant {
        debug_assert_eq!(
            self.alarms[AlarmChannel::HighPrecisionTimer as usize]
                .state
                .load(Ordering::Relaxed),
            AlarmState::Active as u8
        );
        self.timer_epoch.get()
    }

    /// Sets the high-precision timer's epoch from an overflow-protected RTC
    /// tick.
    ///
    /// # Safety
    ///
    /// - Writable from scheduling context while the
    ///   [`RtcChannel::TimerSynchronization`] is neither synchronizing nor
    ///   active.
    /// - Writable from interrupt context while the channel is synchronizing.
    /// - Must not be called in active state.
    unsafe fn set_timer_epoch(&self, rtc_tick: u64) {
        debug_assert!(
            self.is_rtc_alarm_pending(AlarmChannel::HighPrecisionTimer)
                || self.is_rtc_pending_immediate_synchronization()
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

        if rtc.events_compare[3].read().events_compare().bit_is_set() {
            rtc.events_compare[3].reset();
            self.rtc_increment_half_period();
        }

        for channel in [AlarmChannel::SleepTimer1, AlarmChannel::SleepTimer2] {
            if rtc.events_compare[channel as usize]
                .read()
                .events_compare()
                .bit_is_set()
            {
                // We don't reset the compare event here but only just before
                // scheduling the next timeout: The timer may otherwise trigger
                // the compare event again whenever it wraps.

                self.rtc_trigger_alarm(channel);
            }
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
        let waker = unsafe {
            self.alarms[AlarmChannel::HighPrecisionTimer as usize]
                .waker
                .get()
                .as_mut()
        }
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
        // The `half_period` MUST be read before `counter`, see method docs.
        let half_period = self.half_period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = Self::rtc().counter.read().counter().bits();
        ((half_period as u64) << 23) + ((counter ^ ((half_period & 1) << 23)) as u64)
    }

    // Called exclusively from interrupt context.
    fn rtc_increment_half_period(&self) {
        let next_half_period = self.half_period.load(Ordering::Relaxed) + 1;
        // Note: The acquire part of the fence protects the read to the alarm's
        //       RTC tick below. The release part ensures that the updated
        //       period becomes visible to all clients. Inside the interrupt
        //       this fence is not strictly necessary but we add it as it is
        //       essentially free, documents intent and protects us from UB.
        compiler_fence(Ordering::AcqRel);
        self.half_period.store(next_half_period, Ordering::Relaxed);
        let next_half_period_start_tick = (next_half_period as u64) << 23;

        for channel in RTC_CHANNELS {
            // Safety: Ensure that we own the alarm before accessing it.
            if self.is_rtc_alarm_owned_by_interrupt(channel) {
                // Safety: The call to `is_rtc_alarm_owned_by_interrupt()`
                //         atomically acquires the RTC tick value and ensures
                //         exclusive access.
                let pending_rtc_tick = unsafe { self.rtc_alarm_tick(channel) };
                if pending_rtc_tick < next_half_period_start_tick + Self::RTC_THREE_QUARTERS_PERIOD
                {
                    // Just enable the compare interrupt. The correct CC value
                    // has already been set when scheduling the alarm.
                    let rtc = Self::rtc();
                    match channel {
                        AlarmChannel::HighPrecisionTimer => {
                            rtc.evtenset.write(|w| w.compare0().set_bit())
                        }
                        AlarmChannel::SleepTimer1 => rtc.intenset.write(|w| w.compare1().set_bit()),
                        AlarmChannel::SleepTimer2 => rtc.intenset.write(|w| w.compare2().set_bit()),
                        _ => unreachable!(),
                    }
                }
            }
        }
    }

    // Called exclusively from interrupt context.
    //
    // Note: May be preempted by higher-priority interrupts but _not_ by the
    //       scheduling context.
    //
    // Tuning notes:
    fn rtc_trigger_alarm(&self, channel: AlarmChannel) {
        // Performance:
        // - Switching if/else below doesn't yield a measurable improvement.
        // - Using a waker over pending the SWI directly costs us ~60ns.

        // Safety: Acquires alarm memory and ensures exclusive ownership. As the
        //         scheduling context runs at a lower priority, the interrupt
        //         operates atomically on alarm memory. We can therefore safely
        //         access the alarm until the interrupt handler ends.
        if !self.is_rtc_alarm_owned_by_interrupt(channel) {
            // Spurious compare interrupt, possibly due to a race on disabling
            // the interrupt when an overdue alarm is discovered.
            return;
        }

        // Safety: We acquired alarm memory and ensured ownership above.
        if self.rtc_now_tick() < unsafe { self.rtc_alarm_tick(channel) } {
            // Spurious compare interrupt: If the COUNTER is N and the
            // current CC register value is N+1 or N+2 when a new CC value
            // is written, a match may trigger on the previous CC value
            // before the new value takes effect, see nRF product
            // specification.
            Self::rtc().events_compare[channel as usize].reset();
            return;
        }

        // Interrupts must be disabled before we wake the scheduling context.
        self.rtc_fire_and_inactivate_alarm(channel);

        let waker = unsafe { self.alarms[channel as usize].waker.get().as_mut() }
            .unwrap()
            .take();
        if let Some(waker) = waker {
            waker.wake();
        } else {
            self.rtc_release_alarm(channel);
        }
    }

    // Called exclusively from scheduling context.
    #[inline(always)]
    fn rtc_program_cc(&self, rtc_tick: u64, channel: AlarmChannel) -> Result<u64, ()> {
        // Safety: Ensure that the scheduling context exclusively owns the alarm
        //         and corresponding registers.
        debug_assert!(self.is_rtc_alarm_pending(channel));

        // The nRF product spec says: If the COUNTER is N, writing N or N+1 to a
        // CC register may not trigger a COMPARE event.
        //
        // To work around this, we never program a tick smaller than N+3. N+2
        // is not safe because the RTC can tick from N to N+1 between calling
        // now() and writing to the CC register.
        let rtc_now_tick = self.rtc_now_tick();
        if rtc_tick <= rtc_now_tick + State::RTC_GUARD_TICKS {
            self.rtc_fire_alarm(channel);
            return Err(());
        }

        unsafe { self.set_rtc_alarm_tick(channel, rtc_tick) };

        // Safety: The flag must be set before enabling event routing to
        //         transfer ownership. Releases alarm memory to
        //         interrupt context.
        self.rtc_activate_alarm(channel);

        let rtc = Self::rtc();
        let cc = channel as usize;

        rtc.events_compare[cc].reset();
        rtc.cc[cc].write(|w| w.compare().variant(rtc_tick as u32 & 0xFFFFFF));

        Ok(rtc_now_tick)
    }

    #[inline(always)]
    fn rtc_safely_schedule_alarm(
        &self,
        rtc_tick: u64,
        rtc_now_tick: u64,
        channel: AlarmChannel,
    ) -> Result<(), ()> {
        let rtc = Self::rtc();

        if rtc_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD {
            // If the alarm is imminent (i.e. safely within the currently
            // running RTC period), enable the timer interrupt (resp. event)
            // right away.

            // Safety: From this point onwards we must no longer access the
            //         alarm until the alarm has been marked inactive again.
            match channel {
                AlarmChannel::HighPrecisionTimer => rtc.evtenset.write(|w| w.compare0().set_bit()),
                AlarmChannel::SleepTimer1 => rtc.intenset.write(|w| w.compare1().set_bit()),
                AlarmChannel::SleepTimer2 => rtc.intenset.write(|w| w.compare2().set_bit()),
                _ => unreachable!(),
            }

            // Safety: This method may have been preempted by higher-priority
            //         interrupts. Also, its execution time depends on compiler
            //         optimization. Therefore we need to ensure that the alarm
            //         was safely scheduled _after_ enabling the corresponding
            //         interrupt.
            let was_safely_scheduled = self.rtc_now_tick() + Self::RTC_GUARD_TICKS <= rtc_tick;
            if !was_safely_scheduled {
                // Safety: The alarm may or may not have already fired at this
                //         point. It may even spuriously fire later as disabling
                //         interrupts is not immediate. Therefore the interrupt
                //         handler additionally synchronizes on alarm state.
                self.rtc_fire_and_inactivate_alarm(channel);
                Err(())
            } else {
                Ok(())
            }
        } else {
            // If the alarm is too far into the future, don't enable the compare
            // interrupt yet. It will be enabled by `next_period()`.
            Ok(())
        }
    }

    // Called exclusively from scheduling context.
    fn rtc_try_activate_alarm(&self, rtc_tick: u64, channel: AlarmChannel) -> Result<(), ()> {
        let rtc_now_tick = self.rtc_program_cc(rtc_tick, channel)?;
        self.rtc_safely_schedule_alarm(rtc_tick, rtc_now_tick, channel)
    }

    // Called exclusively from scheduling context. Requires the given alarm
    // channel to be acquired/released before/after calling/awaiting the method.
    async fn rtc_wait_for(&self, rtc_tick: u64, channel: AlarmChannel) -> Result<(), ()> {
        let cleanup_on_drop = CancellationGuard::new(|| {
            // Safety: Clearing the interrupt is not immediate. It might still
            //         fire. That's why interrupt context additionally
            //         synchronizes on alarm state.
            self.rtc_fire_and_inactivate_alarm(channel);
            self.rtc_release_alarm(channel);

            // No need to drop the waker. It'll save us cloning if it is still
            // valid when scheduling the next alarm.
        });

        let result = poll_fn(|cx| {
            if self.is_rtc_alarm_owned_by_interrupt(channel) {
                // Safety: We must not access the waker as it is owned by the
                //         interrupt. We may assume that the waker is still
                //         valid, though, as it must not be migrated to a
                //         different task, see safety conditions on the
                //         `RadioTimerApi`.
                Poll::Pending
            } else {
                // Safety: We acquired and exclusively own the alarm at this point.
                let scheduled_waker =
                    unsafe { self.alarms[channel as usize].waker.get().as_mut() }.unwrap();
                if let Some(scheduled_waker) = scheduled_waker {
                    scheduled_waker.clone_from(cx.waker());
                } else {
                    *scheduled_waker = Some(cx.waker().clone());
                }

                if self.is_rtc_alarm_pending(channel) {
                    // Safety: To avoid a data race, we may only activate the
                    //         alarm once we're sure that the waker has been
                    //         safely installed. Activating the alarm
                    //         establishes a happens-before relationship with
                    //         all prior memory accesses and transfers ownership
                    //         of the alarm to interrupt context.
                    let result = self.rtc_try_activate_alarm(rtc_tick, channel);
                    if result.is_ok() {
                        Poll::Pending
                    } else {
                        Poll::Ready(result)
                    }
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        })
        .await;

        cleanup_on_drop.inactivate();

        result
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
            HardwareEvent::RadioFrameStarted => ppi_channel2(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => ppi_channel2(),
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

        #[cfg(feature = "timer-trace")]
        let gpiote_channel = self.gpiote_in_channel.load(Ordering::Relaxed);
        let radio = Self::radio();
        let event_ptr = match event {
            HardwareEvent::RadioRxEnabled => radio.events_rxready.as_ptr(),
            HardwareEvent::RadioFrameStarted => radio.events_framestart.as_ptr(),
            HardwareEvent::RadioDisabled => radio.events_disabled.as_ptr(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => Self::gpiote().events_in[gpiote_channel].as_ptr(),
        };

        let event_ppi_channel = self.ppi_event_channel(event);
        let event_ppi_channel_mask = 1 << event_ppi_channel;
        let event_ch = &ppi.ch[event_ppi_channel];
        let event_fork = &ppi.fork[event_ppi_channel];

        let event_ppi_channel_enabled = ppi.chen.read().bits() & event_ppi_channel_mask != 0;
        if event_ppi_channel_enabled {
            // If the channel is already enabled it must observe the correct
            // event.
            let current_event = event_ch.eep.read().eep().bits();
            if current_event != event_ptr as u32 {
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
            event_ch.eep.write(|w| w.eep().variant(event_ptr as u32));
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
        let event_has_been_observed = match event {
            HardwareEvent::RadioRxEnabled => {
                radio.events_rxready.read().events_rxready().bit_is_set()
            }
            HardwareEvent::RadioFrameStarted => radio
                .events_framestart
                .read()
                .events_framestart()
                .bit_is_set(),
            HardwareEvent::RadioDisabled => {
                radio.events_disabled.read().events_disabled().bit_is_set()
            }
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => Self::gpiote().events_in[gpiote_channel]
                .read()
                .events_in()
                .bit_is_set(),
        };

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
        self.rtc_activate_alarm(AlarmChannel::HighPrecisionTimer);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_start_hp_timer(NsInstant::from_ticks(0), rtc_tick as u32);
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_try_start_at_rtc_tick(&self, rtc_tick: u64) -> Result<(), ()> {
        // Safety: We're exclusively called from scheduling context which
        //         currently owns the high-precision timer alarm channel, see
        //         the check in rtc_program_cc() which must be called _after_.
        unsafe { self.set_timer_epoch(rtc_tick) };
        let rtc_now_tick = self.rtc_program_cc(rtc_tick, AlarmChannel::HighPrecisionTimer)?;

        // Safety: This is a pre-programmed PPI channel.
        Self::ppi()
            .chenset
            .write(|w| unsafe { w.bits(1 << Self::RTC_CC0_TIMER_START_CHANNEL) });

        self.rtc_safely_schedule_alarm(rtc_tick, rtc_now_tick, AlarmChannel::HighPrecisionTimer)
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_start_at_next_rtc_tick(&self) {
        debug_assert!(self.is_rtc_alarm_pending(AlarmChannel::HighPrecisionTimer));

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
        }
    }

    // Called exclusively from scheduling context.
    fn timer_event_channel(event: HardwareEvent) -> usize {
        // See the high-precision timer trait implementation docs for resource
        // assignments.
        match event {
            HardwareEvent::RadioRxEnabled | HardwareEvent::RadioDisabled => 0,
            HardwareEvent::RadioFrameStarted => 1,
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
                //         to synchronizing access to RTC alarms as the sleep
                //         timer can be shared across tasks.)
                let scheduled_waker = unsafe {
                    self.alarms[AlarmChannel::HighPrecisionTimer as usize]
                        .waker
                        .get()
                        .as_mut()
                }
                .unwrap();
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

#[interrupt]
fn RTC0() {
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

#[derive(Clone, Copy, Debug)]
pub struct NrfRadioTimer {
    // Private field to inhibit direct instantiation.
    _inner: (),
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

    #[inline(always)]
    const fn duration_to_timer_ticks(duration: NsDuration) -> u32 {
        debug_assert!(duration.ticks() <= TickConversion::MAX_TIMER_DURATION.ticks());

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

        const N: u32 = 35;
        const MULTIPLIER: u128 = 2_u128.pow(1 + N) / 5_u128.pow(3);

        // Safety: The max timer duration was chosen so that multiplying it with
        //         the MULTIPLIER does not overflow.
        let integer_fraction = duration.ticks() as u128 * MULTIPLIER;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        (integer_fraction >> N) as u32
    }

    #[inline(always)]
    const fn instant_to_timer_ticks_with_epoch(timer_epoch: NsInstant, instant: NsInstant) -> u32 {
        debug_assert!(instant.ticks() > timer_epoch.ticks());
        Self::duration_to_timer_ticks(NsDuration::from_ticks(
            instant.ticks() - timer_epoch.ticks(),
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
        * EXPECTED_REMAINDER_RTC_NS)
        / TickConversion::NS_PER_S as u64) as u32;
    let timer_ticks =
        TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(EXPECTED_REMAINDER_RTC_NS));
    assert!(timer_ticks == EXPECTED_REMAINDER_TIMER_TICKS);

    // One TIMER tick is 62.5 ns, the remaining rounding error must be less.
    const EXPECTED_REMAINDER_TIMER_NS: u64 = 49;
    let timer_ticks_ns = TickConversion::timer_ticks_to_duration(timer_ticks).ticks();
    assert!(u64::MAX - rtc_tick_ns - timer_ticks_ns == EXPECTED_REMAINDER_TIMER_NS);

    const EXPECTED_MAX_TIMER_TICKS: u32 = ((TickConversion::MAX_TIMER_DURATION.ticks()
        * NrfRadioHighPrecisionTimer::FREQUENCY.to_Hz())
        / TickConversion::NS_PER_S as u64) as u32;
    let max_timer_ticks = TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(
        TickConversion::MAX_TIMER_DURATION.ticks(),
    ));
    // We accept a rounding error of one tick.
    assert!(max_timer_ticks == EXPECTED_MAX_TIMER_TICKS - 1);

    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(875)) == 13);
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
    pub gpiote_tick_channel: usize,
    pub ppi_tick_channel: usize,
}

pub struct NrfRadioTimerConfig {
    pub rtc: RTC0,
    pub timer: TIMER0,
    // We take ownership of the clock to enforce clock policy:
    // - The radio requires an external HF crystal oscillator.
    // - The hybrid timer requires the LF oscillator to be continuously enabled.
    pub clock: CLOCK,
    pub sleep_timer_clk_src: super::LfClockSource,
    pub ppi_channels: [usize; 2],
    pub ppi_channel_group: usize,
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

    async unsafe fn wait_until(&mut self, instant: NsInstant) -> Result<(), RadioTimerError> {
        STATE.assert_initialized();

        let rtc_tick = TickConversion::instant_to_rtc_tick(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_rtc_alarm(instant, rtc_tick as u32);

        let channel = [AlarmChannel::SleepTimer1, AlarmChannel::SleepTimer2]
            .into_iter()
            .find(|&channel| STATE.rtc_try_acquire_alarm(channel).is_ok())
            .ok_or(RadioTimerError::Busy)?;

        let result = STATE
            .rtc_wait_for(rtc_tick, channel)
            .await
            .map_err(|_| RadioTimerError::Overdue(instant));

        STATE.rtc_release_alarm(channel);

        result
    }

    fn start_high_precision_timer(
        &self,
        at: OptionalNsInstant,
    ) -> Result<Self::HighPrecisionTimer, RadioTimerError> {
        STATE.assert_initialized();

        STATE.rtc_try_acquire_alarm(AlarmChannel::HighPrecisionTimer)?;

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
        } else {
            STATE.timer_start_at_next_rtc_tick();
        }

        // The high-precision timer will automatically be stopped and the
        // corresponding alarm channel released when dropping the running
        // radio timer instance.
        Ok(NrfRadioHighPrecisionTimer { _inner: () })
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

        // Account for PPI delay, see nRF52840 PS, section 6.16: The timer is
        // synchronized to the 16 MHz clock and runs at 16 MHz, therefore PPI
        // will delay the compare event by one timer tick.
        timer_ticks - 1
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
/// off   | sched_tx       | txen                 | framestarted
/// off   | switch_off     |                      | disabled
/// rx    | stop_listening | disable\framestarted | framestarted, disabled
/// rx    | sched_rx       |                      | rx_enabled*, framestarted
/// rx    | sched_tx       | disable, txen        | framestarted
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
/// Additionally we implement a GPIO test mode that requires a GPIO toggle
/// signal and a GPIO toggled event, possibly used concurrently.
///
/// For the radio, we allocate the following resources:
///
/// signals:
/// - rxen/txen: pre-configured PPI-Chs 20/21 + CC0
/// - disable: pre-configured PPI-Ch 22 + CC1
/// - disable unless...: like "disable" + PPI-ChGroup
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
        STATE.rtc_fire_and_inactivate_alarm(AlarmChannel::HighPrecisionTimer);

        // In case synchronization was interrupted: Clean up registers that were
        // used for synchronization.
        #[cfg(not(feature = "timer-trace"))]
        State::rtc().evtenclr.write(|w| w.tick().set_bit());
        timer.intenclr.write(|w| w.compare1().set_bit());

        // Clean up timer state.
        self.reset();

        // Atomically de-allocate the high precision timer.
        STATE.rtc_release_alarm(AlarmChannel::HighPrecisionTimer)
    }
}
