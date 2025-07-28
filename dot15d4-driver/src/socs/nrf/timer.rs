//! Radio timer implementation for nRF SoCs.
//!
//! A good part of this driver was copied verbatim from embassy_nrf. Kudos to
//! the embassy contributors!

use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};
use core::task::{Poll, Waker};

use critical_section::Mutex;
use dot15d4_util::sync::CancellationGuard;
use dot15d4_util::warn;
use nrf52840_hal::pac::{self, interrupt, RTC0};

use crate::timer::{RadioTimerApi, SyntonizedInstant};

/// Calculate the timestamp from the period count and the tick count.
///
/// The RTC counter is 24 bit. Ticking at 32768 Hz, it overflows every ~8
/// minutes. This is too short. We must protect it against overflow.
///
/// The obvious way would be to count overflow periods. Every time the counter
/// overflows, increase a `periods` variable. `now()` simply does `periods << 24
/// + counter`. So, the logic around an overflow would look like this:
///
/// ```not_rust
/// periods = 1, counter = 0xFF_FFFE --> now = 0x1FF_FFFE
/// periods = 1, counter = 0xFF_FFFF --> now = 0x1FF_FFFF
/// **OVERFLOW**
/// periods = 2, counter = 0x00_0000 --> now = 0x200_0000
/// periods = 2, counter = 0x00_0001 --> now = 0x200_0001
/// ```
///
/// The problem is that this is vulnerable to race conditions if `now()` runs at
/// the exact time an overflow happens.
///
/// If `now()` reads `periods` first and `counter` later, and overflow happens
/// between the reads, it would return a wrong value:
///
/// ```not_rust
/// periods = 1 (OLD), counter = 0x00_0000 (NEW) --> now = 0x100_0000 -> WRONG
/// ```
///
/// It fails similarly if it reads `counter` first and `periods` second.
///
/// To fix this, we define a "period" to be 2^23 ticks (instead of 2^24). One
/// "overflow cycle" is 2 periods.
///
/// - `period` is incremented on overflow (at counter value 0)
/// - `period` is incremented "midway" between overflows (at counter value
///   0x80_0000)
///
/// Therefore, when `period` is even, the counter is expected to be in the range
/// 0..0x7f_ffff, when odd, in the range 0x80_0000..0xff_ffff.
///
/// To get `now()`, the `period` is read first, then the `counter`. If the
/// counter value range matches the expected `period` parity, we're done.  If it
/// doesn't, we know that a new period has started between reading `period` and
/// `counter`. We then assume that the `counter` value corresponds to the next
/// period.
///
/// The `period` has 32 bits and a single period is represented by 23 bits. The
/// counter ticks at 32768 Hz. The overflow protected counter therefore wraps
/// after (2^55-1) / 32768 seconds of uptime, which corresponds to 34865 years.
///
/// Adopted from embassy_nrf. Kudos to the embassy contributors!
fn calc_now(period: u32, counter: u32) -> u64 {
    ((period as u64) << 23) + ((counter ^ ((period & 1) << 23)) as u64)
}

struct Alarms {
    pending: Cell<u64>,
    next: Cell<u64>,
    fired: Cell<u64>,
}

impl Alarms {
    const OFF: u64 = u64::MAX;

    const fn new() -> Self {
        Self {
            pending: Cell::new(Self::OFF),
            next: Cell::new(Self::OFF),
            fired: Cell::new(Self::OFF),
        }
    }

    fn get_pending(&self) -> u64 {
        self.pending.get()
    }

    /// Schedules the next timeout.
    ///
    /// Returns true if the pending timeout must be programmed into the
    /// peripheral.
    fn schedule(&self, timestamp: u64) -> bool {
        if self.pending.get() == Self::OFF {
            self.pending.set(timestamp);
            true
        } else {
            let previous = self.next.replace(timestamp);
            debug_assert_eq!(previous, Self::OFF);
            false
        }
    }

    fn fire_pending_and_get_next(&self) -> u64 {
        let next = self.next.replace(Self::OFF);
        let fired = self.pending.replace(next);
        let prev_fired = self.fired.replace(fired);
        if prev_fired != Self::OFF {
            warn!("missed timer event")
        }
        next
    }

    fn get_and_clear_fired(&self) -> u64 {
        self.fired.replace(Self::OFF)
    }
}

struct RtcDriver {
    /// Number of 2^23 periods elapsed since boot.
    period: AtomicU32,
    /// Pending alarms.
    alarms: Mutex<Alarms>,
    /// Waker for the current alarm.
    waker: Mutex<RefCell<Option<Waker>>>,
}

impl RtcDriver {
    const fn new() -> Self {
        Self {
            period: AtomicU32::new(0),
            alarms: Mutex::new(Alarms::new()),
            waker: Mutex::new(RefCell::new(None)),
        }
    }

    fn rtc() -> pac::RTC0 {
        // Safety: We let clients prove unique ownership of the peripheral by
        //         requiring an instance when initializing the driver.
        // TODO: Check whether this results in efficient assembly.
        unsafe { pac::Peripherals::steal() }.RTC0
    }

    fn init(&self, _rtc: RTC0) {
        let rtc = Self::rtc();
        rtc.cc[3].write(|w| w.compare().variant(0x800000));

        rtc.intenset.write(|w| {
            w.ovrflw().set_bit();
            w.compare3().set_bit()
        });

        rtc.tasks_clear.write(|w| w.tasks_clear().set_bit());
        rtc.tasks_start.write(|w| w.tasks_start().set_bit());

        // Wait for clear
        while rtc.counter.read().counter() != 0 {}

        // Clear and enable the timer interrupt
        pac::NVIC::unpend(pac::Interrupt::RTC0);
        // Safety: We're in early initialization, so there should be no
        //         concurrent critical sections.
        unsafe { pac::NVIC::unmask(pac::Interrupt::RTC0) };
    }

    fn on_interrupt(&self) {
        let rtc = Self::rtc();

        if rtc.events_ovrflw.read().events_ovrflw().bit_is_set() {
            rtc.events_ovrflw.reset();
            self.next_period();
        }

        if rtc.events_compare[3].read().events_compare().bit_is_set() {
            rtc.events_compare[3].reset();
            self.next_period();
        }

        if rtc.events_compare[0].read().events_compare().bit_is_set() {
            rtc.events_compare[0].reset();
            self.trigger_alarm();
        }
    }

    // Called exclusively from interrupt context.
    fn next_period(&self) {
        let next_period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(next_period, Ordering::Relaxed);
        let next_period_mask = (next_period as u64) << 23;

        // TODO: No critical section needed here as we're already in the
        //       interrupt handler.
        critical_section::with(|cs| {
            let pending_alarm = self.alarms.borrow(cs).get_pending();
            if pending_alarm < next_period_mask + 0xc00000 {
                // Just enable the compare interrupt. set_alarm() has already
                // set the correct CC value.
                Self::rtc().intenset.write(|w| w.compare0().set_bit());
            }
        })
    }

    // Called exclusively from interrupt context.
    fn trigger_alarm(&self) {
        Self::rtc().intenclr.write(|w| w.compare0().set_bit());

        // TODO: No critical section needed here as we're already in the
        //       interrupt handler.
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let next_alarm = alarms.fire_pending_and_get_next();
            if next_alarm != Alarms::OFF {
                let overdue = !self.try_program_alarm(next_alarm);
                if overdue {
                    // We lost an alarm. Clients will be able to discover this
                    // by comparing the fired timeout with the scheduled
                    // timeouts.
                    alarms.fire_pending_and_get_next();
                }
            }
            self.waker.borrow_ref(cs).as_ref().map_or_else(
                || {
                    alarms.get_and_clear_fired();
                },
                |waker| waker.wake_by_ref(),
            )
        });
    }

    fn try_program_alarm(&self, timestamp: u64) -> bool {
        let rtc = Self::rtc();

        loop {
            let now = self.now();
            if timestamp <= now {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.
                rtc.intenclr.write(|w| w.compare0().set_bit());
                return false;
            }

            // If it hasn't triggered yet, set it up in the compare channel.

            // Write the CC value regardless of whether we're going to enable it
            // now or not.  This way, when we enable it later, the right value
            // is already set.

            // The nRF docs say:
            //    If the COUNTER is N, writing N or N+1 to a CC register may not
            //    trigger a COMPARE event.
            // To work around this, we never write a timestamp smaller than N+3.
            // N+2 is not safe because rtc can tick from N to N+1 between
            // calling now() and writing CC.

            // Since the critical section does not guarantee that a higher prio
            // interrupt causes this to be delayed, we need to re-check how much
            // time actually passed after setting the alarm, and retry if we are
            // within the unsafe interval still.
            //
            // TODO: This means that an alarm can be delayed for up to 2 ticks
            //       (from t+1 to t+3).
            //
            // The alarm will not trigger *before* its scheduled time, though.
            let safe_timestamp = timestamp.max(now + 3);
            rtc.cc[0].write(|w| w.compare().variant(safe_timestamp as u32 & 0xFFFFFF));

            let diff = timestamp - now;
            if diff < 0xc00000 {
                rtc.intenset.write(|w| w.compare0().set_bit());

                // If we have not passed the timestamp, we can be sure the alarm will be invoked. Otherwise,
                // we need to retry setting the alarm.
                if self.now() + 2 <= timestamp {
                    return true;
                }
            } else {
                // If it's too far in the future, don't setup the compare channel yet.
                // It will be setup later by `next_period`.
                rtc.intenclr.write(|w| w.compare0().set_bit());
                return true;
            }
        }
    }

    fn now(&self) -> u64 {
        // `period` MUST be read before `counter`, see comment at the top for details.
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = Self::rtc().counter.read().counter().bits();
        calc_now(period, counter)
    }

    fn schedule_alarm(&self, at: u64) {
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let pending_alarm_changed = alarms.schedule(at);
            if pending_alarm_changed {
                let overdue = !self.try_program_alarm(at);
                if overdue {
                    alarms.fire_pending_and_get_next();
                }
            }
        })
    }

    async fn wait_for_alarm(&self) -> u64 {
        let cleanup_on_drop = CancellationGuard::new(|| {
            critical_section::with(|cs| {
                self.waker.borrow_ref_mut(cs).take();
            })
        });

        let fired_alarm = poll_fn(|cx| {
            critical_section::with(|cs| {
                let mut scheduled_waker = self.waker.borrow_ref_mut(cs);
                if let Some(scheduled_waker) = scheduled_waker.as_ref() {
                    debug_assert!(cx.waker().will_wake(scheduled_waker));
                } else {
                    *scheduled_waker = Some(cx.waker().clone());
                }

                let fired_alarm = self.alarms.borrow(cs).get_and_clear_fired();
                if fired_alarm == Alarms::OFF {
                    Poll::Pending
                } else {
                    Poll::Ready(fired_alarm)
                }
            })
        })
        .await;

        drop(cleanup_on_drop);

        fired_alarm
    }
}

static DRIVER: RtcDriver = RtcDriver::new();

#[interrupt]
fn RTC0() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_enter();

    DRIVER.on_interrupt();

    RtcDriver::rtc()
        .intenclr
        .write(|w| unsafe { w.bits(0xffff_ffff) });

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_exit_to_scheduler();
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct NrfRadioTimer;

// Assert that our formula works correctly for u64::MAX ns.
const _: () = assert!(
    NrfRadioTimer::ns_to_sleep_ticks(SyntonizedInstant::from_ticks(u64::MAX))
        == NrfRadioTimer::MAX_RTC_TICKS
);
const _: () = {
    const ROUNDING_ERROR: u64 = 17924; // one RTC tick is ~30517ns, the rounding
                                       // error must be less
    assert!(
        NrfRadioTimer::sleep_ticks_to_ns(NrfRadioTimer::MAX_RTC_TICKS).ticks()
            == u64::MAX - ROUNDING_ERROR
    );
};

impl NrfRadioTimer {
    // The max number of RTC ticks representable in nanoseconds (~584 years).
    const MAX_RTC_TICKS: u64 = 604462909807314;
    #[allow(dead_code)]
    const FREQUENCY: u32 = 32_768;

    pub fn init(rtc: RTC0) {
        DRIVER.init(rtc)
    }

    const fn sleep_ticks_to_ns(sleep_ticks: u64) -> SyntonizedInstant {
        debug_assert!(sleep_ticks <= Self::MAX_RTC_TICKS);

        // To keep tick-to-ns conversion cheap we avoid division while
        // minimizing rounding errors:
        //
        // timestamp_ns = ticks * (1 / timer_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 32768 Hz) * 10^9 ns/s
        //              = (ticks * (10^9 / 2^15)) ns
        //              = (ticks * (5^9 / 2^6)) ns
        //              = ((ticks * 5^9) >> 6) ns
        const _: () = assert!(NrfRadioTimer::FREQUENCY == 2_u32.pow(15));

        // Safety: The overflow protected tick counter uses 55 bits, see
        //         `now()`. Representing MAX_RTC_TICKS still requires 50 bits.
        //         Multiplying by 5^9 requires another 21 bits. We therefore
        //         have to calculate in 128 bits to ensure that the calculation
        //         cannot overflow.
        const RTC_FREQ_NS_GCD: u128 = 5_u128.pow(9);
        let ns = (sleep_ticks as u128 * RTC_FREQ_NS_GCD) >> 6;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        SyntonizedInstant::from_ticks(ns as u64)
    }

    const fn ns_to_sleep_ticks(ns: SyntonizedInstant) -> u64 {
        // To keep tick-to-ns conversion cheap we avoid division while
        // minimizing rounding errors:
        //
        // ticks = (timestamp_ns / (10^9 ns/s)) * timer_frequency_hz
        //       = (timestamp_ns / (10^9 ns/s)) * 32768 Hz
        //       = timestamp_ns * (2^15 / 10^9)
        //       = timestamp_ns * (2^6 / 5^9)
        //       = timestamp_ns * ((2^6 * 2^N) / (5^9 * 2^N))
        //       = (timestamp_ns * (2^(6+N) / 5^9)) >> N
        //       = (timestamp_ns * M(N)) >> N where M(N) = 2^(6+N) / 5^9
        //
        // We can now choose M(N) such that it provides maximum precision, i.e.
        // the largest N is chosen such that timestamp_ns_max * M(N) remains
        // representable. It turns out that the largest such N is 78.
        const N: u32 = 78;
        const MULTIPLIER: u128 = 2_u128.pow(6 + N) / 5_u128.pow(9);

        // Safety: We asserted above that the max representable instant in
        //         nanoseconds times the MULTIPLIER does not overflow. We can
        //         represent less nanoseconds in 64 bits than ticks, so casting
        //         down the end result is always safe.
        ((ns.ticks() as u128 * MULTIPLIER) >> N) as u64
    }
}

impl RadioTimerApi for NrfRadioTimer {
    fn now() -> SyntonizedInstant {
        Self::sleep_ticks_to_ns(DRIVER.now())
    }

    async fn wait_until(instant: SyntonizedInstant) {
        let scheduled_tick = Self::ns_to_sleep_ticks(instant);
        DRIVER.schedule_alarm(scheduled_tick);
        let now = DRIVER.wait_for_alarm().await;
        debug_assert_eq!(scheduled_tick, now);
    }
}
