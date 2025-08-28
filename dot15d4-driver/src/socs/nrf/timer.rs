//! Radio timer implementation for nRF SoCs.
//!
//! The initial version of this driver was based on embassy_nrf. Kudos to the
//! embassy contributors!

// No need to use portable atomics in the driver as this is platform-specific
// code.
use core::{
    cell::{Cell, UnsafeCell},
    future::poll_fn,
    sync::atomic::{compiler_fence, AtomicU32, AtomicU8, AtomicUsize, Ordering},
    task::{Poll, Waker},
};

use dot15d4_util::sync::CancellationGuard;
use fugit::TimerRateU32;
#[cfg(feature = "gpio-trace")]
use nrf52840_pac::GPIOTE;
use nrf52840_pac::{interrupt, Peripherals, NVIC, PPI, RADIO, RTC0, TIMER0};

use crate::timer::{
    HardwareEvent, HardwareSignal, LocalClockDuration, LocalClockInstant, RadioTimerApi,
    RadioTimerResult, TimedSignal,
};

use super::AlarmChannel;

/// This flag atomically represents the current alarm state.
///
/// # Safety
///
/// This flag synchronizes exclusive ownership of the alarm between the
/// interrupt and the scheduling process:
///
/// - While the alarm is active, corresponding interrupts may fire at any
///   time, preempt the scheduling thread and access and mutate the alarm as
///   well as related hardware registers.
/// - While the alarm is pending or after it fired, the alarm data must not be
///   accessed from interrupt context. The scheduling thread can then access and
///   mutate the alarm as well as related hardware registers.
///
/// The alarm must not be accessed or mutated from any other than scheduling
/// or interrupt context.
///
/// Considered alternatives, that don't work:
///
/// Synchronization via interrupt registers:
/// - LDREX/STREX are disallowed on device memory
/// - interrupts remain disabled while we wait for the active half period
/// - depending on the configuration several distinct (RTC vs. TIMER) or no
///   interrupt at all (event triggering) may be involved.
///
/// Synchronization via a special value of the RTC tick (Self::OFF):
/// - The overflow-protected RTC tick is 64 bits wide. A 64 bit value cannot
///   be accessed atomically on a 32 bit platform. Portable atomics would
///   introduce a critical section. This is what we want to avoid.
///
/// Note that the safety conditions of the [`RadioTimerApi`] require the
/// timer interrupt to run at a higher priority than the scheduling thread.
/// This means that the interrupt continues to own the alarm while it is
/// active even if it sets the flag to `false`.
#[repr(u8)]
enum AlarmState {
    /// The alarm is currently unused and may be acquired by any scheduling
    /// context.
    Unused,
    /// The alarm is currently being scheduled but still owned by the scheduling
    /// context.
    Pending,
    /// The alarm is currently running and exclusively owned by interrupt
    /// context. Interrupts may preempt scheduling context at any time.
    Active,
    /// The alarm has fired and exclusive ownership was transferred back to the
    /// scheduling context.
    Fired,
}

struct Alarm {
    /// The current alarm state. See [`AlarmState`] for details and safety
    /// considerations.
    state: AtomicU8,

    /// The RTC tick of a pending alarm.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    rtc_tick: Cell<u64>,

    /// True if the RTC triggers the high frequency timer.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    triggers_timer: Cell<bool>,

    /// Waker for the current alarm.
    ///
    /// Safety: Access is synchronized via the alarm state, see above. This is
    ///         required as canceling the alarm races with firing (i.e.  waking)
    ///         it. The waker itself is [`Sync`]. It's therefore ok, to wake it
    ///         from interrupt context.
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
            // The state is initially 'fired' to signal that the scheduling
            // thread has exclusive access to this alarm but still needs to
            // program it.
            state: AtomicU8::new(AlarmState::Unused as u8),
            rtc_tick: Cell::new(0),
            triggers_timer: Cell::new(false),
            waker: UnsafeCell::new(None),
        }
    }
}

const NUM_ALARM_CHANNELS: usize = AlarmChannel::NumAlarmChannels as usize;
const ALARM_CHANNELS: [AlarmChannel; NUM_ALARM_CHANNELS] =
    [AlarmChannel::Timer, AlarmChannel::Rtc1, AlarmChannel::Rtc2];

#[repr(u8)]
enum InitializationState {
    Uninitialized,
    Initializing,
    Initialized,
}

/// The nRF radio timer implements a local (i.e. non-syntonized), monotonic,
/// overflow-protected uptime clock. It combines a low-energy RTC sleep timer
/// peripheral with a high-resolution wake-up TIMER peripheral.
///
/// The timer can trigger asynchronous CPU wake-ups and PPI-backed hardware
/// signals.
///
// Safety: As we are on the single-core nRF platform we don't need to
//         synchronize atomic operations via CPU memory barriers. It is
//         sufficient to place appropriate compiler fences.
struct State {
    init_state: AtomicU8,

    /// Number of half counter periods elapsed since boot.
    ///
    /// Safety: This needs to be atomic as it will be shared between the
    ///         interrupt and the application threads. (U32 would be atomic on
    ///         this platform anyway, but let's make the requirement explicit as
    ///         it compiles down to the same machine code).
    half_period: AtomicU32,

    /// Independent alarm channels supported by the RTC driver.
    ///
    /// Safety: Alarms are Sync.
    alarms: [Alarm; NUM_ALARM_CHANNELS],

    /// A GPIOTE channel used for GPIO event triggering.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "gpio-trace")]
    gpiote_out_channel: AtomicUsize,

    /// A GPIOTE channel used for GPIO event capturing.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "gpio-trace")]
    gpiote_in_channel: AtomicUsize,

    /// A PPI channel used for event triggering.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    ppi_channel: AtomicUsize,

    /// The PPI channel mask containing all PPI channels used by this driver.
    ppi_channel_mask: AtomicU32,
}

impl State {
    const RTC_FREQUENCY: TimerRateU32<32_768> = TimerRateU32::from_raw(1);

    const RTC_THREE_QUARTERS_PERIOD: u64 = 0xc00000;
    const RTC_GUARD_TICKS: u64 = 2;

    // Pre-programmed PPI channels.
    const TIMER_CC0_RADIO_TXEN_CHANNEL: usize = 20;
    const TIMER_CC0_RADIO_RXEN_CHANNEL: usize = 21;
    const RTC_CC0_RADIO_TXEN_CHANNEL: usize = 28;
    const RTC_CC0_RADIO_RXEN_CHANNEL: usize = 29;
    const RTC_CC0_TIMER_START_CHANNEL: usize = 31;
    const PPI_CHANNEL_MASK: u32 = 1 << Self::TIMER_CC0_RADIO_TXEN_CHANNEL
        | 1 << Self::TIMER_CC0_RADIO_RXEN_CHANNEL
        | 1 << Self::RTC_CC0_RADIO_TXEN_CHANNEL
        | 1 << Self::RTC_CC0_RADIO_RXEN_CHANNEL
        | 1 << Self::RTC_CC0_TIMER_START_CHANNEL;

    const fn new() -> Self {
        Self {
            init_state: AtomicU8::new(InitializationState::Uninitialized as u8),
            half_period: AtomicU32::new(0),
            alarms: [Alarm::new(), Alarm::new(), Alarm::new()],
            #[cfg(feature = "gpio-trace")]
            gpiote_out_channel: AtomicUsize::new(0),
            #[cfg(feature = "gpio-trace")]
            gpiote_in_channel: AtomicUsize::new(1),
            ppi_channel: AtomicUsize::new(0),
            ppi_channel_mask: AtomicU32::new(0),
        }
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

    #[cfg(feature = "gpio-trace")]
    fn gpiote() -> GPIOTE {
        // We only access GPIOTE channels that we exclusively own.
        unsafe { Peripherals::steal() }.GPIOTE
    }

    fn radio() -> RADIO {
        // We only access RADIO tasks when asked to do so.
        unsafe { Peripherals::steal() }.RADIO
    }

    /// Takes exclusive ownership of the RTC and TIMER peripherals and
    /// initializes the driver.
    ///
    /// This must be called during early initialization before any concurrent
    /// critical sections may be active.
    pub fn init(
        &self,
        rtc: RTC0,
        timer: TIMER0,
        #[cfg(feature = "gpio-trace")] gpiote_out_channel: usize,
        #[cfg(feature = "gpio-trace")] gpiote_in_channel: usize,
        ppi_channel: usize,
    ) {
        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::instrument();

        assert_eq!(
            STATE
                .init_state
                .swap(InitializationState::Initializing as u8, Ordering::AcqRel),
            InitializationState::Uninitialized as u8
        );

        debug_assert!(ppi_channel <= 19);
        STATE.ppi_channel.store(ppi_channel, Ordering::Relaxed);
        STATE
            .ppi_channel_mask
            .store(1 << ppi_channel | Self::PPI_CHANNEL_MASK, Ordering::Relaxed);

        #[cfg(feature = "gpio-trace")]
        {
            debug_assert!(gpiote_out_channel <= 7);
            debug_assert!(gpiote_in_channel <= 7);
            debug_assert_ne!(gpiote_in_channel, gpiote_out_channel);

            STATE
                .gpiote_out_channel
                .store(gpiote_out_channel, Ordering::Relaxed);
            STATE
                .gpiote_in_channel
                .store(gpiote_in_channel, Ordering::Relaxed);
        }

        timer.mode.reset();
        timer.bitmode.write(|w| w.bitmode()._32bit());

        // The prescaler has a non-zero reset value.
        timer.prescaler.write(|w| w.prescaler().variant(0));
        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());

        rtc.prescaler.reset();
        rtc.cc[3].write(|w| w.compare().variant(0x800000));

        rtc.intenset.write(|w| {
            w.ovrflw().set_bit();
            w.compare3().set_bit()
        });

        #[cfg(feature = "gpio-trace")]
        rtc.evtenset.write(|w| w.tick().set_bit());

        if rtc.counter.read().counter() != 0 {
            rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
            while rtc.counter.read().counter() != 0 {}
        }

        rtc.tasks_start.write(|w| w.tasks_start().set_bit());
        while rtc.counter.read().counter() == 0 {}

        // Clear and enable the timer interrupts.
        NVIC::unpend(interrupt::RTC0);
        NVIC::unpend(interrupt::TIMER0);
        // Safety: We're in early initialization, so there should be no
        //         concurrent critical sections.
        unsafe { NVIC::unmask(interrupt::RTC0) };
        unsafe { NVIC::unmask(interrupt::TIMER0) };

        STATE
            .init_state
            .store(InitializationState::Initialized as u8, Ordering::Release);
    }

    fn assert_initialized(&self) {
        debug_assert_eq!(
            self.init_state.load(Ordering::Acquire),
            InitializationState::Initialized as u8
        );
    }

    /// Sets the alarm's RTC and TIMER ticks.
    ///
    /// # Safety
    ///
    /// - The alarm state must indicate ownership for the calling context.
    /// - Must be called exclusively from scheduling context.
    unsafe fn set_alarm_ticks(&self, channel: AlarmChannel, rtc_tick: u64, triggers_timer: bool) {
        let alarm = &self.alarms[channel as usize];
        alarm.rtc_tick.set(rtc_tick);
        alarm.triggers_timer.set(triggers_timer);
    }

    /// Read the alarm's RTC tick.
    ///
    /// # Safety:
    ///
    /// - The alarm state must indicate ownership for the calling context.
    /// - Compiler fences are required to acquire/release this value.
    unsafe fn alarm_rtc_tick(&self, channel: AlarmChannel) -> u64 {
        self.alarms[channel as usize].rtc_tick.get()
    }

    /// Check wether the alarm triggers the high frequency timer.
    ///
    /// # Safety:
    ///
    /// - The alarm state must indicate ownership for the calling context.
    /// - Compiler fences are required to acquire/release this value.
    unsafe fn alarm_triggers_timer(&self, channel: AlarmChannel) -> bool {
        self.alarms[channel as usize].triggers_timer.get()
    }

    /// Retrieve the captured timer value.
    ///
    /// # Safety
    ///
    /// - The alarm state must indicate ownership of the timer alarm channel for
    ///   the calling context.
    unsafe fn get_and_clear_captured_timer_ticks(&self) -> u32 {
        let timer = Self::timer();
        let result = timer.cc[0].read().bits();
        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());
        result
    }

    /// Returns `true` while the alarm is active (and owned by interrupt
    /// context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_alarm_active(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Active as u8
    }

    /// Returns `true` while the alarm is pending (and owned by scheduling
    /// context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_alarm_pending(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Pending as u8
    }

    /// Disables timer interrupts and signals to the scheduling task that the
    /// alarm has been fired and is now inactive.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn fire_and_inactivate_alarm(&self, channel: AlarmChannel) {
        let rtc = Self::rtc();

        // Safety: We need to disable the interrupt before we transfer
        //         ownership of the alarm to the scheduling context. We disable
        //         the interrupt early, as it may take up to four cycles before
        //         this operation takes effect. Should the interrupt be
        //         spuriously woken it will additionally check alarm state.
        match channel {
            AlarmChannel::Timer => {
                rtc.evtenclr.write(|w| w.compare0().set_bit());
                rtc.intenclr.write(|w| w.compare0().set_bit());
                Self::ppi()
                    .chenclr
                    .write(|w| unsafe { w.bits(self.ppi_channel_mask.load(Ordering::Relaxed)) });
                Self::timer().intenclr.write(|w| w.compare0().set_bit());
                #[cfg(feature = "gpio-trace")]
                Self::gpiote().intenclr.write(|w| unsafe {
                    w.bits(1 << self.gpiote_in_channel.load(Ordering::Relaxed))
                });
            }
            AlarmChannel::Rtc1 => rtc.intenclr.write(|w| w.compare1().set_bit()),
            AlarmChannel::Rtc2 => rtc.intenclr.write(|w| w.compare2().set_bit()),
            _ => unreachable!(),
        }

        self.fire_alarm(channel);
    }

    /// Mark the alarm as pending. Expects the alarm to be unused.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// # Panics
    ///
    /// Panics if the scheduling context causes a race condition, i.e. tries to
    /// acquire an alarm that has been acquired concurrently by another
    /// scheduling context.
    fn acquire_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .compare_exchange_weak(
                AlarmState::Unused as u8,
                AlarmState::Pending as u8,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .unwrap();
        compiler_fence(Ordering::Acquire);
    }

    /// Mark the alarm as pending if it is unused.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn try_acquire_alarm(&self, channel: AlarmChannel) -> Result<(), ()> {
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
            Err(())
        }
    }

    /// Transfer ownership of the alarm to interrupt context.
    ///
    /// Releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// Note: This does _not_ also activate interrupts. These may have to remain
    ///       inactive if we've not yet reached the target timer period.
    fn activate_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Active as u8, Ordering::Relaxed);
    }

    /// Mark the alarm as fired.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn fire_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Fired as u8, Ordering::Relaxed);
    }

    /// Mark the alarm as unused. Expects the alarm to be fired.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// # Panics
    ///
    /// Panics if the alarm was not previously fired.
    fn release_alarm(&self, channel: AlarmChannel) {
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

    // Called exclusively from interrupt context.
    fn on_rtc_interrupt(&self) {
        // Perf: As the SWI runs at a lower priority than the RTC interrupt
        //       handler, order doesn't matter within this handler.

        let rtc = Self::rtc();

        if rtc.events_ovrflw.read().events_ovrflw().bit_is_set() {
            rtc.events_ovrflw.reset();
            self.increment_half_period();
        }

        if rtc.events_compare[3].read().events_compare().bit_is_set() {
            rtc.events_compare[3].reset();
            self.increment_half_period();
        }

        for channel in ALARM_CHANNELS {
            if rtc.events_compare[channel as usize]
                .read()
                .events_compare()
                .bit_is_set()
            {
                if self.rtc_now_tick() < unsafe { self.alarm_rtc_tick(channel) } {
                    // Spurious compare interrupt: If the COUNTER is N and the
                    // current CC register value is N+1 or N+2 when a new CC value
                    // is written, a match may trigger on the previous CC value
                    // before the new value takes effect, see nRF product
                    // specification.
                    rtc.events_compare[channel as usize].reset();
                    return;
                }

                // We don't reset the compare event here but only just before
                // scheduling the next timeout: The timer may otherwise trigger
                // the compare event again whenever it wraps.
                self.trigger_alarm(channel);
            }
        }
    }

    // Called exclusively from interrupt context.
    fn on_timer_interrupt(&self) {
        self.trigger_alarm(AlarmChannel::Timer);
    }

    #[cfg(feature = "gpio-trace")]
    fn on_gpiote_interrupt(&self) {
        let gpiote = Self::gpiote();
        let gpiote_channel = self.gpiote_in_channel.load(Ordering::Relaxed);
        let gpiote_mask = 1 << gpiote_channel;

        let is_waiting_for_event = gpiote.intenset.read().bits() & gpiote_mask > 0;
        if !is_waiting_for_event {
            return;
        }

        let is_gpiote_in_event = gpiote.events_in[gpiote_channel]
            .read()
            .events_in()
            .bit_is_set();
        if is_gpiote_in_event {
            gpiote.events_in[gpiote_channel].write(|w| w.events_in().clear_bit());
        } else {
            return;
        }

        // Safety: We checked the range of the gpiote channel.
        gpiote.intenclr.write(|w| unsafe { w.bits(gpiote_mask) });

        self.trigger_alarm(AlarmChannel::Timer);
    }

    // Called exclusively from interrupt context.
    fn increment_half_period(&self) {
        let next_half_period = self.half_period.load(Ordering::Relaxed) + 1;
        // Note: The acquire part of the fence protects the read to the alarm's
        //       RTC tick below. The release part ensures that the updated
        //       period becomes visible to all clients. Inside the interrupt
        //       this fence is not strictly necessary but we add it as it is
        //       essentially free, documents intent and protects us from UB.
        compiler_fence(Ordering::AcqRel);
        self.half_period.store(next_half_period, Ordering::Relaxed);
        let next_half_period_start_tick = (next_half_period as u64) << 23;

        for channel in ALARM_CHANNELS {
            // Safety: Ensure that we own the alarm before accessing it.
            if self.is_alarm_active(channel) {
                // Safety: The call to `is_alarm_active()` atomically acquires
                //         the RTC tick value and ensures exclusive access.
                let pending_rtc_tick = unsafe { self.alarm_rtc_tick(channel) };
                if pending_rtc_tick < next_half_period_start_tick + Self::RTC_THREE_QUARTERS_PERIOD
                {
                    // Just enable the compare interrupt. The correct CC value
                    // has already been set when scheduling the alarm.
                    let rtc = Self::rtc();
                    match channel {
                        AlarmChannel::Timer => {
                            if unsafe { self.alarm_triggers_timer(channel) } {
                                rtc.evtenset.write(|w| w.compare0().set_bit())
                            } else {
                                rtc.intenset.write(|w| w.compare0().set_bit())
                            }
                        }
                        AlarmChannel::Rtc1 => rtc.intenset.write(|w| w.compare1().set_bit()),
                        AlarmChannel::Rtc2 => rtc.intenset.write(|w| w.compare2().set_bit()),
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
    fn trigger_alarm(&self, channel: AlarmChannel) {
        // Performance:
        // - Switching if/else below doesn't yield a measurable improvement.
        // - Using a waker over pending the SWI directly costs us ~60ns.

        // Safety: Acquires alarm memory and ensures exclusive ownership. As the
        //         scheduling context runs at a lower priority, the interrupt
        //         operates atomically on alarm memory. We can therefore safely
        //         access the alarm until the interrupt handler ends.
        if !self.is_alarm_active(channel) {
            // Spurious compare interrupt, possibly due to a race on disabling
            // the interrupt when an overdue alarm is discovered.
            return;
        }

        // Interrupts must be disabled before we wake the scheduling context.
        self.fire_and_inactivate_alarm(channel);

        let waker = unsafe { self.alarms[channel as usize].waker.get().as_mut() }
            .unwrap()
            .take();
        if let Some(waker) = waker {
            waker.wake();
        } else {
            self.release_alarm(channel);
        }
    }

    // Called exclusively from scheduling context.
    #[inline(always)]
    fn prepare_rtc(
        &self,
        rtc_tick: u64,
        triggers_timer: bool,
        channel: AlarmChannel,
    ) -> (bool, u64) {
        // Safety: Ensure that the scheduling context exclusively owns the alarm
        //         and corresponding registers.
        debug_assert!(self.is_alarm_pending(channel));

        // The nRF product spec says: If the COUNTER is N, writing N or N+1 to a
        // CC register may not trigger a COMPARE event.
        //
        // To work around this, we never program a tick smaller than N+3. N+2
        // is not safe because the RTC can tick from N to N+1 between calling
        // now() and writing to the CC register.
        let rtc_now_tick = self.rtc_now_tick();
        if rtc_tick <= rtc_now_tick + State::RTC_GUARD_TICKS {
            self.fire_alarm(channel);
            return (false, rtc_now_tick);
        }

        unsafe { self.set_alarm_ticks(channel, rtc_tick, triggers_timer) };

        // Safety: The alarm must be activated before enabling the interrupt or
        //         event routing to transfer ownership. Releases alarm memory to
        //         interrupt context.
        self.activate_alarm(channel);

        let rtc = Self::rtc();
        let cc = channel as usize;

        rtc.events_compare[cc].reset();
        rtc.cc[cc].write(|w| w.compare().variant(rtc_tick as u32 & 0xFFFFFF));

        (true, rtc_now_tick)
    }

    #[inline(always)]
    fn was_safely_scheduled(
        &self,
        rtc_tick: u64,
        rtc_now_tick: u64,
        triggers_timer: bool,
        channel: AlarmChannel,
    ) -> RadioTimerResult {
        let rtc = Self::rtc();

        if rtc_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD {
            // If the alarm is imminent (i.e. safely within the currently
            // running RTC period), enable the timer interrupt right away.

            // Safety: From this point onwards we must no longer access the
            //         alarm until the alarm has been marked inactive again.
            match channel {
                AlarmChannel::Timer => {
                    if triggers_timer {
                        rtc.evtenset.write(|w| w.compare0().set_bit());
                    } else {
                        rtc.intenset.write(|w| w.compare0().set_bit())
                    }
                }
                AlarmChannel::Rtc1 => rtc.intenset.write(|w| w.compare1().set_bit()),
                AlarmChannel::Rtc2 => rtc.intenset.write(|w| w.compare2().set_bit()),
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
                self.fire_and_inactivate_alarm(channel);
                RadioTimerResult::Overdue
            } else {
                RadioTimerResult::Ok
            }
        } else {
            // If the alarm is too far into the future, don't enable the compare
            // interrupt yet. It will be enabled by `next_period()`.
            RadioTimerResult::Ok
        }
    }

    // Called exclusively from scheduling context.
    fn try_activate_alarm(
        &self,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
        channel: AlarmChannel,
        maybe_signal: Option<HardwareSignal>,
    ) -> RadioTimerResult {
        let triggers_timer = remaining_timer_ticks > 0;

        let (ok, rtc_now_tick) = self.prepare_rtc(rtc_tick, triggers_timer, channel);
        if !ok {
            return RadioTimerResult::Overdue;
        }

        let rtc = Self::rtc();
        let timer = Self::timer();
        let ppi = Self::ppi();

        if matches!(channel, AlarmChannel::Timer) {
            // cc == 0

            let cc_event = if triggers_timer {
                // Safety: This is a pre-programmed PPI channel.
                ppi.chenset
                    .write(|w| unsafe { w.bits(1 << Self::RTC_CC0_TIMER_START_CHANNEL) });
                timer.events_compare[0].reset();
                timer.shorts.write(|w| {
                    w.compare0_clear().set_bit();
                    w.compare0_stop().set_bit()
                });
                timer.cc[0].write(|w| w.cc().variant(remaining_timer_ticks as u32));
                timer.intenset.write(|w| w.compare0().set_bit());
                timer.events_compare[0].as_ptr()
            } else {
                rtc.events_compare[0].as_ptr()
            };

            let ppi_channel = match maybe_signal {
                Some(signal) => match signal {
                    #[cfg(feature = "gpio-trace")]
                    HardwareSignal::GpioToggle => {
                        let ppi_channel = self.ppi_channel.load(Ordering::Relaxed);
                        let ch = &ppi.ch[ppi_channel];

                        ch.eep.write(|w| w.eep().variant(cc_event as u32));

                        let gpiote_channel = self.gpiote_out_channel.load(Ordering::Relaxed);
                        let gpiote_out_task = Self::gpiote().tasks_out[gpiote_channel].as_ptr();
                        ch.tep.write(|w| w.tep().variant(gpiote_out_task as u32));
                        ppi.fork[ppi_channel].tep.reset();

                        ppi_channel
                    }
                    HardwareSignal::RadioRxEnable => {
                        if triggers_timer {
                            Self::TIMER_CC0_RADIO_RXEN_CHANNEL
                        } else {
                            Self::RTC_CC0_RADIO_RXEN_CHANNEL
                        }
                    }
                    HardwareSignal::RadioTxEnable => {
                        if triggers_timer {
                            Self::TIMER_CC0_RADIO_TXEN_CHANNEL
                        } else {
                            Self::RTC_CC0_RADIO_TXEN_CHANNEL
                        }
                    }
                    HardwareSignal::RadioDisable => {
                        let ppi_channel = self.ppi_channel.load(Ordering::Relaxed);
                        let ch = &ppi.ch[ppi_channel];

                        ch.eep.write(|w| w.eep().variant(cc_event as u32));

                        let radio_disable_task = Self::radio().tasks_disable.as_ptr();
                        ch.tep.write(|w| w.tep().variant(radio_disable_task as u32));
                        ppi.fork[ppi_channel].tep.reset();

                        ppi_channel
                    }
                },
                None => unreachable!(),
            };

            // Safety: The channel has been asserted to be in range.
            ppi.chenset.write(|w| unsafe { w.bits(1 << ppi_channel) });
        } else {
            debug_assert!(maybe_signal.is_none());
            debug_assert!(!triggers_timer);
        }

        self.was_safely_scheduled(rtc_tick, rtc_now_tick, triggers_timer, channel)
    }

    // Called exclusively from scheduling context.
    fn try_activate_capture(
        &self,
        _start_at_rtc_tick: u64,
        _event: HardwareEvent,
    ) -> RadioTimerResult {
        #[cfg(not(feature = "gpio-trace"))]
        todo!("not implemented");

        #[cfg(feature = "gpio-trace")]
        {
            let (ok, rtc_now_tick) =
                self.prepare_rtc(_start_at_rtc_tick, true, AlarmChannel::Timer);
            if !ok {
                return RadioTimerResult::Overdue;
            }

            let timer = Self::timer();
            timer.shorts.reset();

            let ppi = Self::ppi();
            let ppi_channel = self.ppi_channel.load(Ordering::Relaxed);
            let ch = &ppi.ch[ppi_channel];
            let fork = &ppi.fork[ppi_channel];

            let gpiote = Self::gpiote();
            let gpiote_channel = self.gpiote_in_channel.load(Ordering::Relaxed);

            match _event {
                HardwareEvent::GpioToggle => {
                    let gpiote_in_event = gpiote.events_in[gpiote_channel].as_ptr();
                    ch.eep.write(|w| w.eep().variant(gpiote_in_event as u32));

                    let timer_capture_task = timer.tasks_capture[0].as_ptr();
                    ch.tep.write(|w| w.tep().variant(timer_capture_task as u32));

                    let timer_stop_task = timer.tasks_stop.as_ptr();
                    fork.tep.write(|w| w.tep().variant(timer_stop_task as u32));

                    // Safety: We checked the range of the gpiote channel.
                    gpiote
                        .intenset
                        .write(|w| unsafe { w.bits(1 << gpiote_channel) });
                }
            };

            // Safety: These are a pre-programmed PPI channel and a channel that has
            //         been asserted to be in range.
            ppi.chenset.write(|w| unsafe {
                w.bits(1 << Self::RTC_CC0_TIMER_START_CHANNEL | 1 << ppi_channel)
            });

            let was_safely_scheduled = self.was_safely_scheduled(
                _start_at_rtc_tick,
                rtc_now_tick,
                true,
                AlarmChannel::Timer,
            );
            if gpiote.events_in[gpiote_channel]
                .read()
                .events_in()
                .bit_is_set()
            {
                gpiote.events_in[gpiote_channel].write(|w| w.events_in().clear_bit());
                RadioTimerResult::Overdue
            } else {
                was_safely_scheduled
            }
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

    // Called exclusively from scheduling context. Requires the given alarm
    // channel to be acquired/released before/after calling/awaiting the method.
    async fn wait_for<Activate: (FnOnce(AlarmChannel) -> RadioTimerResult) + Copy>(
        &self,
        channel: AlarmChannel,
        activate: Activate,
    ) -> RadioTimerResult {
        let cleanup_on_drop = CancellationGuard::new(|| {
            // Safety: Clearing the interrupt is not immediate. It might still
            //         fire. That's why interrupt context additionally
            //         synchronizes on alarm state.
            self.fire_and_inactivate_alarm(channel);
            self.release_alarm(channel);

            // No need to drop the waker. It'll save us cloning if it is still
            // valid when scheduling the next alarm.
        });

        let result = poll_fn(|cx| {
            if self.is_alarm_active(channel) {
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

                if self.is_alarm_pending(channel) {
                    // Safety: To avoid a data race, we may only activate the
                    //         alarm once we're sure that the waker has been
                    //         safely installed. Activating the alarm
                    //         establishes a happens-before relationship with
                    //         all prior memory accesses and transfers ownership
                    //         of the alarm to interrupt context.
                    let result = activate(channel);
                    if matches!(result, RadioTimerResult::Ok) {
                        Poll::Pending
                    } else {
                        Poll::Ready(result)
                    }
                } else {
                    Poll::Ready(RadioTimerResult::Ok)
                }
            }
        })
        .await;

        cleanup_on_drop.inactivate();

        result
    }

    // Called exclusively from scheduling context. Requires the given alarm
    // channel to be acquired before calling the method.
    fn schedule_event(
        &self,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
        signal: HardwareSignal,
    ) -> RadioTimerResult {
        // Safety: We own alarm memory at this point and the unsafe cell ensures
        //         a non-null pointer.
        unsafe {
            self.alarms[AlarmChannel::Timer as usize]
                .waker
                .get()
                .as_mut()
        }
        .unwrap()
        .take();

        self.try_activate_alarm(
            rtc_tick,
            remaining_timer_ticks,
            AlarmChannel::Timer,
            Some(signal),
        )
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

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NrfRadioTimer {
    // Private field to inhibit direct instantiation.
    inner: (),
}

impl NrfRadioTimer {
    /// Instantiate the timer for the first time. Consumes the required
    /// peripherals. Further copies can then be created.
    pub fn new(
        rtc: RTC0,
        timer: TIMER0,
        #[cfg(feature = "gpio-trace")] gpiote_out_channel: usize,
        #[cfg(feature = "gpio-trace")] gpiote_in_channel: usize,
        ppi_channel: usize,
    ) -> Self {
        STATE.init(
            rtc,
            timer,
            #[cfg(feature = "gpio-trace")]
            gpiote_out_channel,
            #[cfg(feature = "gpio-trace")]
            gpiote_in_channel,
            ppi_channel,
        );
        Self { inner: () }
    }

    /// Applications that require the gpio-trace feature and want to capture
    /// incoming GPIO toggle events SHALL call this method when the
    /// corresponding interrupt has been fired.
    ///
    /// Note: The global GPIOTE interrupt handler cannot be owned by the timer
    ///       as it only owns particular channels of the GPIOTE peripheral, not
    ///       the whole peripheral.
    #[cfg(feature = "gpio-trace")]
    pub fn on_gpiote_interrupt() {
        STATE.on_gpiote_interrupt();
    }
}

// Tick-to-ns conversion (and back).
impl NrfRadioTimer {
    // The max number of RTC/TIMER ticks representable in nanoseconds (~584 years):
    // max_ticks = ((2^64-1) ns / 10^9 ns/s) * frequency.
    const MAX_NS: u128 = u64::MAX as u128;
    const NS_PER_S: u128 = 1_000_000_000;
    const MAX_RTC_TICKS: u64 =
        ((Self::MAX_NS * State::RTC_FREQUENCY.to_Hz() as u128) / Self::NS_PER_S) as u64;

    const fn rtc_tick_to_instant(rtc_tick: u64) -> LocalClockInstant {
        debug_assert!(rtc_tick <= Self::MAX_RTC_TICKS);

        // To keep tick-to-ns conversion cheap we avoid division while
        // minimizing rounding errors:
        //
        // timestamp_ns = ticks * (1 / rtc_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 32768 Hz) * 10^9 ns/s
        //              = (ticks * (10^9 / 2^15)) ns
        //              = (ticks * (5^9 / 2^6)) ns
        //              = ((ticks * 5^9) >> 6) ns
        const _: () = assert!(State::RTC_FREQUENCY.to_Hz() == 2_u32.pow(15));

        // Safety: Representing MAX_RTC_TICKS requires 50 bits. Multiplying by
        //         5^9 requires another 21 bits. We therefore have to calculate
        //         in 128 bits to ensure that the calculation cannot overflow.
        const MULTIPLIER: u128 = 5_u128.pow(9);
        let ns = (rtc_tick as u128 * MULTIPLIER) >> 6;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        LocalClockInstant::from_ticks(ns as u64)
    }

    const fn timer_ticks_to_duration(timer_ticks: u32) -> LocalClockDuration {
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
        LocalClockDuration::from_ticks(ns)
    }

    const fn instant_to_alarm_ticks(ns: LocalClockInstant) -> (u64, u16) {
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
        let fraction = ns.ticks() as u128 * MULTIPLIER;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        let rtc_ticks = (fraction >> N) as u64;

        // To calculate the remainder in timer ticks, we re-use the remainder of
        // the ns-to-rtc_ticks calculation.
        //
        // With F(N) := timestamp_ns * M(N) and R := F(N) & (2^N - 1):
        //
        // remainder_s = (1/rtc_frequency_hz) * R/2^N where
        //
        // timer_ticks = remainder_s * timer_frequency_hz
        //             = (1/2^15) s * (R/2^N) * 16 MHz
        //             = (R * 2^10 * 5^6)/(2^(N + 15))
        //             = (R * 5^6)/(2^(N + 5))
        //             = (R * 5^6) >> N+5
        const FRACTION_MASK: u128 = 2_u128.pow(N) - 1u128;
        const FRACTION_MULTIPLIER: u128 = 5_u128.pow(6);

        // Safety: The max remainder multiplied by the fraction multiplier (i.e.
        //         the fraction mask times 5^6) can be represented in 92 bits,
        //         so calculating in 128 bits is safe. The remainder represents
        //         less than 1 RTC tick (~30.5µs) i.e. less than 489 timer ticks
        //         which we can safely cast down to 32 bits.
        let remainder = fraction & FRACTION_MASK;
        let remaining_timer_ticks = ((remainder * FRACTION_MULTIPLIER) >> (N + 5)) as u16;

        (rtc_ticks, remaining_timer_ticks)
    }
}

impl RadioTimerApi for NrfRadioTimer {
    fn now(&self) -> LocalClockInstant {
        STATE.assert_initialized();

        let rtc_tick = STATE.rtc_now_tick();
        Self::rtc_tick_to_instant(rtc_tick)
    }

    async unsafe fn wait_until(
        &self,
        instant: LocalClockInstant,
        signal: Option<HardwareSignal>,
    ) -> RadioTimerResult {
        STATE.assert_initialized();

        let (rtc_tick, mut remaining_timer_ticks) = Self::instant_to_alarm_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_wait_until(instant.ticks() as u32, rtc_tick as u32);

        let channel = if signal.is_some() {
            STATE.acquire_alarm(AlarmChannel::Timer);
            AlarmChannel::Timer
        } else {
            remaining_timer_ticks = 0;
            [AlarmChannel::Rtc1, AlarmChannel::Rtc2]
                .into_iter()
                .find(|&channel| STATE.try_acquire_alarm(channel).is_ok())
                .unwrap()
        };

        let result = STATE
            .wait_for(channel, |channel| {
                STATE.try_activate_alarm(rtc_tick, remaining_timer_ticks, channel, signal)
            })
            .await;

        STATE.release_alarm(channel);

        result
    }

    async unsafe fn wait_for_event(
        &self,
        start_at: LocalClockInstant,
        event: HardwareEvent,
    ) -> Result<LocalClockInstant, RadioTimerResult> {
        STATE.assert_initialized();

        let (rtc_tick, _) = Self::instant_to_alarm_ticks(start_at);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_wait_for(start_at.ticks() as u32, rtc_tick as u32);

        STATE.acquire_alarm(AlarmChannel::Timer);

        let result = match STATE
            .wait_for(AlarmChannel::Timer, |_| {
                STATE.try_activate_capture(rtc_tick, event)
            })
            .await
        {
            RadioTimerResult::Ok => {
                let captured_timer_ticks = STATE.get_and_clear_captured_timer_ticks();
                debug_assert!(captured_timer_ticks > 0);
                let captured_duration = Self::timer_ticks_to_duration(captured_timer_ticks);
                Ok(Self::rtc_tick_to_instant(rtc_tick) + captured_duration)
            }
            RadioTimerResult::Overdue => Err(RadioTimerResult::Overdue),
        };

        STATE.release_alarm(AlarmChannel::Timer);

        result
    }

    unsafe fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> RadioTimerResult {
        STATE.assert_initialized();

        let TimedSignal { instant, signal } = timed_signal;
        let (rtc_tick, remaining_timer_ticks) = Self::instant_to_alarm_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_event(
            instant.ticks() as u32,
            rtc_tick as u32,
            remaining_timer_ticks as u32,
        );

        STATE.acquire_alarm(AlarmChannel::Timer);

        STATE.schedule_event(rtc_tick, remaining_timer_ticks, signal)
    }
}

// Test conversion.
//
// Note: We do this in a const expression rather than a test so that we can also
//       prove proper "constification" of the conversion functions.
const _: () = {
    let (rtc_tick, remaining_timer_ticks) =
        NrfRadioTimer::instant_to_alarm_ticks(LocalClockInstant::from_ticks(u64::MAX));
    assert!(rtc_tick == NrfRadioTimer::MAX_RTC_TICKS);

    // One RTC tick is ~30517 ns, the rounding error must be less.
    const EXPECTED_REMAINDER_RTC_NS: u64 = 17924;
    let rtc_tick_ns = NrfRadioTimer::rtc_tick_to_instant(rtc_tick).ticks();
    assert!(rtc_tick_ns == u64::MAX - EXPECTED_REMAINDER_RTC_NS);

    const TIMER_FREQUENCY: u64 = 16_000_000;
    const EXPECTED_REMAINDER_TIMER_TICKS: u16 =
        ((TIMER_FREQUENCY * EXPECTED_REMAINDER_RTC_NS) / NrfRadioTimer::NS_PER_S as u64) as u16;
    assert!(remaining_timer_ticks == EXPECTED_REMAINDER_TIMER_TICKS);

    // One TIMER tick is 62.5 ns, the remaining rounding error must be less.
    const EXPECTED_REMAINDER_TIMER_NS: u64 = 49;
    let timer_ticks_ns =
        NrfRadioTimer::timer_ticks_to_duration(remaining_timer_ticks as u32).ticks();
    assert!(u64::MAX - rtc_tick_ns - timer_ticks_ns == EXPECTED_REMAINDER_TIMER_NS);
};
