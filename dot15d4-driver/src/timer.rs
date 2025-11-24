//! Time structures.
//!
//! - [`Instant`] is used to represent a point in time.
//! - [`Duration`] is used to represent a duration of time.

use core::{future::Future, num::NonZeroU64};

use fugit::NanosDurationU64;

pub mod export {
    pub use fugit::{Duration, ExtU64, Instant};
}

use export::*;

pub type NsInstant = Instant<u64, 1, 1_000_000_000>;
pub type NsDuration = NanosDurationU64;

// Local clock instants will almost never be zero. We often need optional
// instants, those use 16 bytes, even when non-zero, as rust is not able to see
// the niche. Therefore we provide a wrapper to save space.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct OptionalNsInstant {
    inner: NsInstant,
}

impl OptionalNsInstant {
    #[inline]
    pub const fn none() -> Self {
        Self {
            inner: NsInstant::from_ticks(0),
        }
    }

    #[inline]
    pub const fn from_ticks(ticks: NonZeroU64) -> Self {
        Self {
            inner: NsInstant::from_ticks(ticks.get()),
        }
    }

    #[inline]
    pub const fn try_from_ticks(ticks: u64) -> Self {
        Self {
            inner: NsInstant::from_ticks(ticks),
        }
    }

    #[inline]
    pub const fn is_some(&self) -> bool {
        self.inner.ticks() > 0
    }

    #[inline]
    pub const fn is_none(&self) -> bool {
        !self.is_some()
    }

    #[inline]
    pub const fn set(&mut self, instant: NsInstant) {
        assert!(instant.ticks() > 0);
        self.inner = instant
    }

    #[inline]
    pub const fn unwrap(self) -> NsInstant {
        if self.is_some() {
            self.inner
        } else {
            panic!()
        }
    }

    #[inline]
    pub fn map<U, F>(self, f: F) -> Option<U>
    where
        F: FnOnce(NsInstant) -> U,
    {
        match self.inner.ticks() {
            0 => None,
            ticks => Some(f(NsInstant::from_ticks(ticks))),
        }
    }

    #[inline]
    pub fn map_or_else<U, D, F>(self, default: D, f: F) -> U
    where
        D: FnOnce() -> U,
        F: FnOnce(NsInstant) -> U,
    {
        match self.inner.ticks() {
            0 => default(),
            ticks => f(NsInstant::from_ticks(ticks)),
        }
    }

    #[inline]
    pub fn ok_or_else<E, F>(self, err: F) -> Result<NsInstant, E>
    where
        F: FnOnce() -> E,
    {
        match self.inner.ticks() {
            0 => Err(err()),
            ticks => Ok(NsInstant::from_ticks(ticks)),
        }
    }
}

impl From<OptionalNsInstant> for Option<NsInstant> {
    #[inline]
    fn from(value: OptionalNsInstant) -> Self {
        let instant = value.inner;
        if instant.ticks() > 0 {
            Some(instant)
        } else {
            None
        }
    }
}

impl From<NsInstant> for OptionalNsInstant {
    #[inline]
    fn from(value: NsInstant) -> Self {
        assert!(value.ticks() > 0);
        Self { inner: value }
    }
}

impl From<Option<NsInstant>> for OptionalNsInstant {
    #[inline]
    fn from(value: Option<NsInstant>) -> Self {
        match value {
            Some(instant) => instant.into(),
            None => OptionalNsInstant::none(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioTimerError {
    /// The instant or signal could not be safely scheduled, e.g. due to guard
    /// time restrictions or because the scheduled instant is in the past.
    ///
    /// The operation returned at an arbitrary time before or after the
    /// scheduled instant or event occurrence.
    ///
    /// The offending instant is being returned.
    Overdue(NsInstant),

    /// The event cannot be observed as it already occurred.
    Already,

    /// The timer operation could not be scheduled due to lack of resources
    /// (e.g. lack of free timer channels).
    Busy,
}

/// Hardware signals are an abstraction over electrical signals that can be sent
/// across an event bus as usually found on radio hardware.
///
/// Note: The architecture and implementation of hardware signals and event
///       buses varies widely across SoCs and transceivers. A good abstraction
///       needs to emerge over time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum HardwareSignal {
    /// Enable radio reception.
    RadioRxEnable,

    /// Enable radio transmission.
    RadioTxEnable,

    /// Cancel any ongoing radio reception/transmission and transition the radio
    /// into a low-energy state.
    RadioDisable,

    /// Toggle the outbound alarm pin.
    #[cfg(feature = "timer-trace")]
    GpioToggle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum HardwareEvent {
    /// An event indicating that the radio receiver was enabled. Exact timing is
    /// implementation specific.
    RadioRxEnabled,

    /// An event indicating the start of frame reception. Exact timing is
    /// implementation specific.
    RadioFrameStarted,

    /// An event indicating that the radio was disabled. Exact timing is
    /// implementation specific.
    RadioDisabled,

    /// A toggle event on the inbound alarm pin.
    #[cfg(feature = "timer-trace")]
    GpioToggled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimedSignal {
    pub instant: NsInstant,
    pub signal: HardwareSignal,
}

impl TimedSignal {
    pub const fn new(instant: NsInstant, signal: HardwareSignal) -> Self {
        Self { instant, signal }
    }
}

pub trait RadioTimerApi: Copy {
    const TICK_PERIOD: NsDuration;
    const GUARD_TIME: NsDuration;

    type HighPrecisionTimer: HighPrecisionTimer;

    /// Returns a recent instant of the local radio clock's coarse sleep timer.
    ///
    /// Note: This method involves the CPU and therefore will always return a
    ///       past instant while the timer continues to tick concurrently.
    fn now(&self) -> NsInstant;

    /// Waits until the given instant using only the sleep timer, then wakes the
    /// current task.
    ///
    /// Implementations SHALL be cancel-safe. Cancelling the future will cancel
    /// the alarm.
    ///
    /// Note: This wakes the current task with latency and jitter depending on
    ///       the surrounding executor implementation. To reduce latency and
    ///       (almost) eliminate jitter, use the [`InterruptExecutor`].
    ///
    /// [`InterruptExecutor`]: crate::executor::InterruptExecutor
    ///
    /// # Panics
    ///
    /// Trying to create another future while the previous has not been driven
    /// to completion and dropped, will panic.
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s).
    /// - The resulting future SHALL always be polled with the same waker, i.e.
    ///   it SHALL NOT be migrated to a different task. Wakers MAY change on
    ///   subsequent invocations of the method, though.
    unsafe fn wait_until(
        &mut self,
        instant: NsInstant,
    ) -> impl Future<Output = Result<(), RadioTimerError>>;

    /// Tries to allocate and start a high precision timer instance that is
    /// synchronized with the sleep clock. Returns an error if no timer instance
    /// can be allocated.
    ///
    /// If a start time is given and the method returns without an error, then
    /// the timer is guaranteed to be started before the given instant. The
    /// start time must observe [`RadioTimerApi::GUARD_TIME`].
    ///
    /// If no start time is given, then the timer starts as fast as possible,
    /// i.e.  at the next available sleep timer tick. This MAY be faster than
    /// [`RadioTimerApi::GUARD_TIME`] but SHALL NOT be slower.
    ///
    /// The returned object serves as a token representing the running timer.
    /// Dropping it will cancel, stop and de-allocate the timer.
    ///
    /// Note: The sleep timer MAY be used concurrently with high-precision
    ///       timers.
    fn start_high_precision_timer(
        &self,
        at: OptionalNsInstant,
    ) -> Result<Self::HighPrecisionTimer, RadioTimerError>;
}

/// Represents a started high-precision timer. MAY be dropped to stop and
/// de-allocate the timer.
pub trait HighPrecisionTimer {
    const TICK_PERIOD: NsDuration;

    /// Programs a hardware signal to be sent over the event bus at a precise
    /// instant.
    ///
    /// This method provides access to deterministically timed signals at
    /// hardware level without CPU intervention. Exact timing specifications are
    /// implementation dependent.
    ///
    /// Returns an error if the timed signal could not be scheduled, e.g. due to
    /// lack of resources ([`RadioTimerError::Busy`]) or late scheduling
    /// ([`RadioTimerError::Overdue`]).
    fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> Result<&Self, RadioTimerError>;

    /// Programs a hardware signal to be sent over the event bus at a precise
    /// instant unless the given event happens before.
    ///
    /// Returns an error if the timed signal could not be scheduled, e.g. due to
    /// lack of resources ([`RadioTimerError::Busy`]) or late scheduling
    /// ([`RadioTimerError::Overdue`]).
    ///
    /// Returns [`RadioTimerError::Already`] if the event was already pending
    /// when calling the method.
    fn schedule_timed_signal_unless(
        &self,
        timed_signal: TimedSignal,
        event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError>;

    /// Waits until a scheduled signal has been executed.
    ///
    /// The resulting future SHALL be cancellable. Dropping the future SHALL NOT
    /// cancel the scheduled event.
    ///
    /// # Panics
    ///
    /// Panics if [`HighPrecisionTimer::schedule_timed_signal()`] had not been
    /// called for this signal.
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s).
    /// - The resulting future SHALL always be polled with the same waker, i.e.
    ///   it SHALL NOT be migrated to a different task. Wakers MAY change on
    ///   subsequent invocations of the method, though.
    unsafe fn wait_for(&mut self, signal: HardwareSignal) -> impl Future<Output = ()>;

    /// Prepares the timer to listen for a hardware event and capture the
    /// high-precision timestamp of the event if it occurs.
    ///
    /// Returns [`RadioTimerError::Busy`] if the timer cannot observe the event
    /// due to lack of resources.
    ///
    /// Returns [`RadioTimerError::Already`] if the event was already pending
    /// when calling the method.
    ///
    /// This method SHALL be idempotent, i.e. if the event is already being
    /// observed, then calling this method SHALL be a no-op and return
    /// successfully.
    ///
    /// See [`HighPrecisionTimer::poll_event`].
    fn observe_event(&self, event: HardwareEvent) -> Result<&Self, RadioTimerError>;

    /// Returns the timestamp observed by [`HighPrecisionTimer::observe_event`].
    /// Returns [`None`] if the corresponding event was not observed.
    ///
    /// In case an event was observed, calling this method will release
    /// corresponding timer resources so that they can be re-used by future
    /// calls to [`HighPrecisionTimer::observe_event()`].
    ///
    /// # Panics
    ///
    /// Panics if [`HighPrecisionTimer::observe_event()`] had not been called
    /// for this event or if the event had already been collected by a prior
    /// call to this method.
    fn poll_event(&self, event: HardwareEvent) -> OptionalNsInstant;

    /// Removes all scheduled signals and observed events but leaves the
    /// high-precision timer running.
    fn reset(&self);
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    use crate::timer::{HardwareEvent, HardwareSignal, NsInstant};

    #[cfg(feature = "timer-trace")]
    mod internal {
        pub use dot15d4_util::trace::{
            systemview_record_u32, systemview_record_u32x2, systemview_record_u32x3,
            systemview_register_module, SystemviewModule,
        };

        use crate::timer::NsInstant;

        // Events
        #[derive(Clone, Copy)]
        pub enum TraceEvents {
            RtcAlarm,
            StartHpTimer,
            ScheduleTimedSignal,
            ObserveEvent,
            NumEvents,
        }

        impl TraceEvents {
            pub fn event_id(&self) -> u32 {
                *self as u32 + unsafe { TIMER_MODULE }.event_offset()
            }
        }

        static TIMER_MODULE_DESC: &str = "M=timer, \
            0 Alarm µs=%u rt=%u, \
            1 Start µs=%u rt=%u, \
            2 Sig µs=%u tt=%u s=%u \
            3 Evt e=%u\0";
        const _: () = assert!(TIMER_MODULE_DESC.len() <= 128);
        pub static mut TIMER_MODULE: SystemviewModule =
            SystemviewModule::new(TIMER_MODULE_DESC, TraceEvents::NumEvents as u32);

        #[inline(always)]
        pub fn to_micros_remainder(instant: NsInstant) -> u32 {
            // The largest power of 10 that can be represented in a u32.
            const MAX_U32_POW_10: u64 = 1_000_000_000;
            (instant.duration_since_epoch().to_micros() % MAX_U32_POW_10) as u32
        }
    }

    #[cfg(feature = "timer-trace")]
    use internal::{TraceEvents::*, *};

    pub fn instrument() {
        #[cfg(feature = "timer-trace")]
        unsafe {
            systemview_register_module(&raw mut TIMER_MODULE)
        };
    }

    #[inline(always)]
    pub fn record_rtc_alarm(_instant: NsInstant, _rtc_ticks: u32) {
        #[cfg(feature = "timer-trace")]
        systemview_record_u32x2(
            RtcAlarm.event_id(),
            to_micros_remainder(_instant),
            _rtc_ticks,
        );
    }

    #[inline(always)]
    pub fn record_start_hp_timer(_instant: NsInstant, _rtc_ticks: u32) {
        #[cfg(feature = "timer-trace")]
        systemview_record_u32x2(
            StartHpTimer.event_id(),
            to_micros_remainder(_instant),
            _rtc_ticks,
        );
    }

    #[inline(always)]
    pub fn record_schedule_timed_signal(
        _instant: NsInstant,
        _timer_ticks: u32,
        _signal: HardwareSignal,
    ) {
        #[cfg(feature = "timer-trace")]
        systemview_record_u32x3(
            ScheduleTimedSignal.event_id(),
            to_micros_remainder(_instant),
            _timer_ticks,
            _signal as u32,
        );
    }

    #[inline(always)]
    pub fn record_observe_event(_event: HardwareEvent) {
        #[cfg(feature = "timer-trace")]
        systemview_record_u32(ObserveEvent.event_id(), _event as u32);
    }
}
