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
use nrf52840_pac::{interrupt, Peripherals, GPIOTE, NVIC, PPI, RADIO, RTC0, TIMER0};

use crate::timer::{
    HardwareSignal, RadioTimerApi, RadioTimerResult, SyntonizedDuration, SyntonizedInstant,
    TimedSignal,
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

    /// The RTC tick of a pending alarm or OFF if the alarm is not pending.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    rtc_tick: Cell<u64>,

    /// The additional TIMER ticks of a pending alarm or OFF if the alarm is not
    /// pending.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    remaining_timer_ticks: Cell<u16>,

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
            state: AtomicU8::new(AlarmState::Fired as u8),
            rtc_tick: Cell::new(0),
            remaining_timer_ticks: Cell::new(0),
            waker: UnsafeCell::new(None),
        }
    }
}

const NUM_ALARM_CHANNELS: usize = AlarmChannel::NumAlarmChannels as usize;
const ALARM_CHANNELS: [AlarmChannel; NUM_ALARM_CHANNELS] = [AlarmChannel::Cpu, AlarmChannel::Event];

#[repr(u8)]
enum InitializationState {
    Uninitialized,
    Initializing,
    Initialized,
}

/// The nRF radio timer implements a shared, globally syntonized, monotonic,
/// overflow-protected uptime "wall clock". It combines a low-energy RTC sleep
/// timer peripheral with a high-resolution wake-up TIMER peripheral.
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

    /// A GPIOTE channel used for event triggering.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    gpiote_channel: AtomicUsize,

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
    const TIMER_CC_RADIO_TXEN_CHANNEL: usize = 20;
    const TIMER_CC_RADIO_RXEN_CHANNEL: usize = 21;
    const RTC_CC_RADIO_TXEN_CHANNEL: usize = 28;
    const RTC_CC_RADIO_RXEN_CHANNEL: usize = 29;
    const RTC_CC_TIMER_START_CHANNEL: usize = 31;
    const PPI_CHANNEL_MASK: u32 = 1 << Self::TIMER_CC_RADIO_TXEN_CHANNEL
        | 1 << Self::TIMER_CC_RADIO_RXEN_CHANNEL
        | 1 << Self::RTC_CC_RADIO_TXEN_CHANNEL
        | 1 << Self::RTC_CC_RADIO_RXEN_CHANNEL
        | 1 << Self::RTC_CC_TIMER_START_CHANNEL;

    const fn new() -> Self {
        Self {
            init_state: AtomicU8::new(InitializationState::Uninitialized as u8),
            half_period: AtomicU32::new(0),
            alarms: [Alarm::new(), Alarm::new()],
            gpiote_channel: AtomicUsize::new(0),
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
    pub fn init(&self, rtc: RTC0, timer: TIMER0, gpiote_channel: usize, ppi_channel: usize) {
        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::instrument();

        assert_eq!(
            STATE
                .init_state
                .swap(InitializationState::Initializing as u8, Ordering::AcqRel),
            InitializationState::Uninitialized as u8
        );

        debug_assert!(ppi_channel <= 19);

        STATE
            .gpiote_channel
            .store(gpiote_channel, Ordering::Relaxed);
        STATE.ppi_channel.store(ppi_channel, Ordering::Relaxed);
        STATE
            .ppi_channel_mask
            .store(1 << ppi_channel | Self::PPI_CHANNEL_MASK, Ordering::Relaxed);

        timer.mode.reset();
        // We need to represent up to two RTC ticks (976 timer ticks).
        timer.bitmode.reset(); // 16 bit by default
                               // The prescaler has a non-zero reset value.
        timer.prescaler.write(|w| w.prescaler().variant(0));
        timer.shorts.write(|w| {
            w.compare0_clear().set_bit();
            w.compare0_stop().set_bit()
        });
        timer.tasks_clear.write(|w| w.tasks_clear().set_bit());

        rtc.prescaler.reset();
        rtc.cc[3].write(|w| w.compare().variant(0x800000));

        rtc.intenset.write(|w| {
            w.ovrflw().set_bit();
            w.compare3().set_bit()
        });
        rtc.evtenset.write(|w| w.tick().set_bit());

        if rtc.counter.read().counter() != 0 {
            rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
            while rtc.counter.read().counter() != 0 {}
        }

        rtc.tasks_start.write(|w| w.tasks_start().set_bit());
        while rtc.counter.read().counter() == 0 {}

        // Clear and enable the timer interrupts
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
    unsafe fn set_alarm_ticks(
        &self,
        channel: AlarmChannel,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
    ) {
        let alarm = &self.alarms[channel as usize];
        alarm.remaining_timer_ticks.set(remaining_timer_ticks);
        alarm.rtc_tick.set(rtc_tick)
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
            AlarmChannel::Event => {
                rtc.evtenclr.write(|w| w.compare0().set_bit());
                rtc.intenclr.write(|w| w.compare0().set_bit());
                Self::ppi()
                    .chenclr
                    .write(|w| unsafe { w.bits(self.ppi_channel_mask.load(Ordering::Relaxed)) });
                Self::timer().intenclr.write(|w| w.compare0().set_bit());
            }
            AlarmChannel::Cpu => rtc.intenclr.write(|w| w.compare1().set_bit()),
            _ => unreachable!(),
        }

        self.fire_alarm(channel);
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

    /// Mark the alarm as pending.
    ///
    /// Releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn pend_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Pending as u8, Ordering::Relaxed);
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

    fn on_timer_interrupt(&self) {
        self.trigger_alarm(AlarmChannel::Event);
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
                        AlarmChannel::Event => rtc.intenset.write(|w| w.compare0().set_bit()),
                        AlarmChannel::Cpu => rtc.intenset.write(|w| w.compare1().set_bit()),
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
        }
    }

    // Called exclusively from scheduling context.
    fn try_activate_alarm(
        &self,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
        channel: AlarmChannel,
        signal: Option<HardwareSignal>,
    ) -> RadioTimerResult {
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
            return RadioTimerResult::Overdue;
        }

        unsafe { self.set_alarm_ticks(channel, rtc_tick, remaining_timer_ticks) };

        // Safety: The alarm must be activated before enabling the interrupt or
        //         event routing to transfer ownership. Releases alarm memory to
        //         interrupt context.
        self.activate_alarm(channel);

        let rtc = Self::rtc();
        let timer = Self::timer();
        let ppi = Self::ppi();
        let cc = channel as usize;

        rtc.events_compare[cc].reset();
        rtc.cc[cc].write(|w| w.compare().variant(rtc_tick as u32 & 0xFFFFFF));

        if matches!(channel, AlarmChannel::Event) {
            // cc == 0

            let cc_event = if remaining_timer_ticks > 0 {
                // Safety: This is a pre-programmed PPI channel.
                ppi.chenset
                    .write(|w| unsafe { w.bits(1 << Self::RTC_CC_TIMER_START_CHANNEL) });
                timer.events_compare[0].reset();
                timer.cc[0].write(|w| w.cc().variant(remaining_timer_ticks as u32));
                timer.intenset.write(|w| w.compare0().set_bit());
                timer.events_compare[0].as_ptr()
            } else {
                rtc.events_compare[0].as_ptr()
            };

            let ppi_channel = match signal {
                Some(signal) => match signal {
                    HardwareSignal::GpioToggle(_) => {
                        let ppi_channel = self.ppi_channel.load(Ordering::Relaxed);
                        let ch = &ppi.ch[ppi_channel];

                        ch.eep.write(|w| w.eep().variant(cc_event as u32));

                        let gpiote_channel = self.gpiote_channel.load(Ordering::Relaxed);
                        let gpiote_out_task = Self::gpiote().tasks_out[gpiote_channel].as_ptr();
                        ch.tep.write(|w| w.tep().variant(gpiote_out_task as u32));

                        ppi_channel
                    }
                    HardwareSignal::RadioRxEnable => {
                        if remaining_timer_ticks == 0 {
                            Self::RTC_CC_RADIO_RXEN_CHANNEL
                        } else {
                            Self::TIMER_CC_RADIO_RXEN_CHANNEL
                        }
                    }
                    HardwareSignal::RadioTxEnable => {
                        if remaining_timer_ticks == 0 {
                            Self::RTC_CC_RADIO_TXEN_CHANNEL
                        } else {
                            Self::TIMER_CC_RADIO_TXEN_CHANNEL
                        }
                    }
                    HardwareSignal::RadioDisable => {
                        let ppi_channel = self.ppi_channel.load(Ordering::Relaxed);
                        let ch = &ppi.ch[ppi_channel];

                        ch.eep.write(|w| w.eep().variant(cc_event as u32));

                        let radio_disable_task = Self::radio().tasks_disable.as_ptr();
                        ch.tep.write(|w| w.tep().variant(radio_disable_task as u32));

                        ppi_channel
                    }
                },
                None => unreachable!(),
            };

            // Safety: The channel has been asserted to be in range.
            ppi.chenset.write(|w| unsafe { w.bits(1 << ppi_channel) });

            rtc.evtenset.write(|w| w.compare0().set_bit());
        } else {
            debug_assert!(signal.is_none());
            debug_assert_eq!(remaining_timer_ticks, 0);
        }

        if rtc_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD {
            // If the alarm is imminent (i.e. safely within the currently
            // running RTC period), enable the timer interrupt right away.

            // Safety: From this point onwards we must no longer access the
            //         alarm until the alarm has been marked inactive again.
            match channel {
                AlarmChannel::Event => {
                    if remaining_timer_ticks == 0 {
                        rtc.intenset.write(|w| w.compare0().set_bit())
                    }
                }
                AlarmChannel::Cpu => rtc.intenset.write(|w| w.compare1().set_bit()),
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

    // Called exclusively from scheduling context.
    async fn wait_for_alarm(
        &self,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
        channel: AlarmChannel,
        signal: Option<HardwareSignal>,
    ) -> RadioTimerResult {
        let cleanup_on_drop = CancellationGuard::new(|| {
            // Safety: Clearing the interrupt is not immediate. It might still
            //         fire. That's why interrupt context additionally
            //         synchronizes on alarm state.
            self.fire_and_inactivate_alarm(channel);

            // No need to drop the waker. It'll save us cloning if it is still
            // valid when scheduling the next alarm.
        });

        // Safety: Ensure that we exclusively own the alarm.
        debug_assert!(!self.is_alarm_active(channel));

        self.pend_alarm(channel);

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
                    let result =
                        self.try_activate_alarm(rtc_tick, remaining_timer_ticks, channel, signal);
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

    // Called exclusively from scheduling context.
    fn schedule_event(
        &self,
        rtc_tick: u64,
        remaining_timer_ticks: u16,
        signal: HardwareSignal,
    ) -> RadioTimerResult {
        // Safety: Ensure that we exclusively own the alarm.
        debug_assert!(!self.is_alarm_active(AlarmChannel::Event));

        self.pend_alarm(AlarmChannel::Event);

        // Safety: Activating the alarm establishes a happens-before
        //         relationship with all prior memory accesses and transfers
        //         ownership of the alarm to interrupt context.
        self.try_activate_alarm(
            rtc_tick,
            remaining_timer_ticks,
            AlarmChannel::Event,
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
    pub fn new(rtc: RTC0, timer: TIMER0, gpiote_channel: usize, ppi_channel: usize) -> Self {
        STATE.init(rtc, timer, gpiote_channel, ppi_channel);
        Self { inner: () }
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

    const fn rtc_tick_to_instant(rtc_tick: u64) -> SyntonizedInstant {
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
        SyntonizedInstant::from_ticks(ns as u64)
    }

    #[allow(dead_code)]
    const fn timer_ticks_to_duration(timer_ticks: u32) -> SyntonizedDuration {
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
        SyntonizedDuration::from_ticks(ns)
    }

    const fn instant_to_alarm_ticks(ns: SyntonizedInstant) -> (u64, u16) {
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
    fn now(&self) -> SyntonizedInstant {
        STATE.assert_initialized();

        let rtc_tick = STATE.rtc_now_tick();
        Self::rtc_tick_to_instant(rtc_tick)
    }

    async unsafe fn wait_until(
        &self,
        instant: SyntonizedInstant,
        signal: Option<HardwareSignal>,
    ) -> RadioTimerResult {
        STATE.assert_initialized();

        let (rtc_tick, remaining_timer_ticks) = Self::instant_to_alarm_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_alarm(instant.ticks() as u32, rtc_tick as u32);

        if let Some(signal) = signal {
            STATE.wait_for_alarm(
                rtc_tick,
                remaining_timer_ticks,
                AlarmChannel::Event,
                Some(signal),
            )
        } else {
            STATE.wait_for_alarm(rtc_tick, 0, AlarmChannel::Cpu, None)
        }
        .await
    }

    unsafe fn schedule_event(&self, timed_signal: TimedSignal) -> RadioTimerResult {
        STATE.assert_initialized();

        let TimedSignal { instant, signal } = timed_signal;
        let (rtc_tick, remaining_timer_ticks) = Self::instant_to_alarm_ticks(instant);

        #[cfg(feature = "rtos-trace")]
        crate::timer::trace::record_schedule_signal(
            instant.ticks() as u32,
            rtc_tick as u32,
            remaining_timer_ticks as u32,
        );

        STATE.schedule_event(rtc_tick, remaining_timer_ticks, signal)
    }
}

// Test conversion.
//
// Note: We do this in a const expression rather than a test so that we can also
//       prove proper "constification" of the conversion functions.
const _: () = {
    let (rtc_tick, remaining_timer_ticks) =
        NrfRadioTimer::instant_to_alarm_ticks(SyntonizedInstant::from_ticks(u64::MAX));
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
