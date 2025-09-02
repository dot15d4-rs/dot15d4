//! Time structures.
//!
//! - [`Instant`] is used to represent a point in time.
//! - [`Duration`] is used to represent a duration of time.

use core::future::Future;

use fugit::NanosDurationU64;

pub mod export {
    pub use fugit::{Duration, ExtU64, Instant};
}

use export::*;

/// O-QPSK 250kB/s = 31.25kb/s = 62.5ksymbol/s (1 byte = 8 bit = 2 O-QPSK symbols)
pub const O_QPSK_FREQUENCY: u32 = 62_500;
pub type SymbolsOQpsk250Instant = Instant<u64, 1, 62_500>;
pub type SymbolsOQpsk250Duration = Duration<u64, 1, 62_500>;

pub type LocalClockInstant = Instant<u64, 1, 1_000_000_000>;
pub type LocalClockDuration = NanosDurationU64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioTimerResult {
    /// The alarm was successfully scheduled and fired with well-defined
    /// latency at the given instant.
    Ok,
    /// The alarm was already overdue or could not be safely scheduled due to
    /// guard time restrictions being violated. The method returned at an
    /// arbitrary time before or after the scheduled instant.
    Overdue,
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
    /// Toggle the outbound alarm pin.
    #[cfg(feature = "gpio-trace")]
    GpioToggle,

    /// Enable radio reception.
    RadioRxEnable,

    /// Enable radio transmission.
    RadioTxEnable,

    /// Disable radio reception/transmission.
    RadioDisable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum HardwareEvent {
    /// A toggle event on the inbound alarm pin.
    #[cfg(feature = "gpio-trace")]
    GpioToggle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimedSignal {
    pub instant: LocalClockInstant,
    pub signal: HardwareSignal,
}

impl TimedSignal {
    pub const fn new(instant: LocalClockInstant, signal: HardwareSignal) -> Self {
        Self { instant, signal }
    }
}

pub trait RadioTimerApi: Copy {
    /// Returns the current instant of the local radio clock's coarse sleep
    /// timer.
    ///
    /// Note: This method involves the CPU and therefore will always introduce
    ///       some latency. The timer might have ticked concurrently in the
    ///       meantime.
    fn now(&self) -> LocalClockInstant;

    /// Waits until the given instant, then wakes the current task.
    ///
    /// If an additional signal is provided, then that signal will be triggered
    /// precisely at the requested time. This option uses the high-precision
    /// timer.
    ///
    /// If no hardware signal is given then only the sleep timer will be used.
    /// The high-precision timer is kept off.
    ///
    /// Implementations SHALL be cancel-safe. Cancelling the future will cancel
    /// the alarm.
    ///
    /// Note: This wakes the current task with latency and jitter as there may
    ///       be an arbitrary delay between waking the task and the task
    ///       executing. Only the (optional) signal will be deterministically
    ///       timed. To reduce latency and (almost) eliminate jitter, use the
    ///       [`InterruptExecutor`].
    ///
    /// [`InterruptExecutor`]: crate::executor::InterruptExecutor
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s).
    /// - The resulting future SHALL always be polled with the same waker, i.e.
    ///   it SHALL NOT be migrated to a different task. Wakers MAY change on
    ///   subsequent invocations of the method, though.
    unsafe fn wait_until(
        &self,
        instant: LocalClockInstant,
        signal: Option<HardwareSignal>,
    ) -> impl Future<Output = RadioTimerResult>;

    /// Enables the high-precision timer at the given start time, then starts
    /// listening for a hardware event and captures the high-precision timestamp
    /// of the event.
    ///
    /// Implementations SHALL be cancel-safe. Cancelling the future will stop
    /// the high-precision timer and reset timer state.
    ///
    /// Note: This wakes the current task with latency and jitter as there may
    ///       be an arbitrary delay between waking the task and the task
    ///       executing. The event timestamp will be captured precisely, though.
    ///       To reduce latency and (almost) eliminate jitter, use the
    ///       [`InterruptExecutor`].
    ///
    /// [`InterruptExecutor`]: crate::executor::InterruptExecutor
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s) as well as any interrupt fired by the
    ///   captured hardware event.
    /// - The resulting future SHALL always be polled with the same waker, i.e.
    ///   it SHALL NOT be migrated to a different task. Wakers MAY change on
    ///   subsequent invocations of the method, though.
    unsafe fn wait_for_event(
        &self,
        start_at: LocalClockInstant,
        event: HardwareEvent,
    ) -> impl Future<Output = Result<LocalClockInstant, RadioTimerResult>>;

    /// Schedule a hardware event, i.e. programs a signal to be sent over the
    /// event bus at a precise instant.
    ///
    /// This method provides access to deterministically timed events at
    /// hardware level without CPU intervention. Uses the high-precision timer.
    /// Exact timing specifications are implementation dependent.
    ///
    /// The method does not block.
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s).
    ///
    unsafe fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> RadioTimerResult;
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    use dot15d4_util::trace::{
        systemview_record_u32x2, systemview_record_u32x3, systemview_register_module,
        SystemviewModule,
    };

    use crate::timer::LocalClockInstant;

    // Events
    #[derive(Clone, Copy)]
    enum TraceEvents {
        WaitUntil,
        WaitFor,
        ScheduleEvent,
        NumEvents,
    }

    impl TraceEvents {
        fn event_id(&self) -> u32 {
            *self as u32 + unsafe { TIMER_MODULE }.event_offset()
        }
    }

    use TraceEvents::*;

    static TIMER_MODULE_DESC: &str = "M=timer, \
        0 WaitUntil µs=%u rt=%u, \
        1 WaitFor µs=%u rt=%u, \
        2 SchedEvt µs=%u rt=%u tt=%u\0";
    static mut TIMER_MODULE: SystemviewModule =
        SystemviewModule::new(TIMER_MODULE_DESC, NumEvents as u32);

    pub fn instrument() {
        unsafe { systemview_register_module(&raw mut TIMER_MODULE) };
    }

    #[inline(always)]
    fn to_micros_remainder(instant: LocalClockInstant) -> u32 {
        // The largest power of 10 that can be represented in a u32.
        const MAX_U32_POW_10: u64 = 1_000_000_000;
        (instant.duration_since_epoch().to_micros() % MAX_U32_POW_10) as u32
    }

    #[inline(always)]
    pub fn record_wait_until(instant: LocalClockInstant, rtc_ticks: u32) {
        systemview_record_u32x2(
            WaitUntil.event_id(),
            to_micros_remainder(instant),
            rtc_ticks,
        );
    }

    #[inline(always)]
    pub fn record_wait_for(instant: LocalClockInstant, rtc_ticks: u32) {
        systemview_record_u32x2(WaitFor.event_id(), to_micros_remainder(instant), rtc_ticks);
    }

    #[inline(always)]
    pub fn record_schedule_event(
        instant: LocalClockInstant,
        rtc_ticks: u32,
        remaining_timer_ticks: u32,
    ) {
        systemview_record_u32x3(
            ScheduleEvent.event_id(),
            to_micros_remainder(instant),
            rtc_ticks,
            remaining_timer_ticks,
        );
    }
}
