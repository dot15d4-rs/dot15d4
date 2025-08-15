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

pub type SyntonizedInstant = Instant<u64, 1, 1_000_000_000>;
pub type SyntonizedDuration = NanosDurationU64;

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

/// Arbitrary output pins that can be targeted by hardware signals. The actual
/// mapping to physical pins is implementation specific.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Pin {
    Pin0,
    // ... add additional pins as needed
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
    /// A signal to toggle the given output pin.
    TogglePin(Pin),
}

pub trait RadioTimerApi: Copy {
    /// Returns the current syntonized instant.
    ///
    /// Note: This method involves the CPU and therefore will always introduce
    ///       some latency. The timer might have ticked concurrently in the
    ///       meantime and/or a syntonization event not reflected in the
    ///       returned value might have arrived.
    fn now(&self) -> SyntonizedInstant;

    /// Waits until the given instant, then wakes the current task. Only the
    /// sleep timer will be used, keeps the high-precision timer off.
    ///
    /// Implementations SHALL be cancel-safe. Cancelling the future will cancel
    /// the alarm.
    ///
    /// Note: This method may introduce latency and jitter as there may be an
    ///       arbitrary delay between waking the task and the task executing.
    ///       For precisely timed alarms, use one of the hardware-backed
    ///       methods. To reduce latency and (almost) eliminate jitter, use the
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
    #[allow(dead_code)]
    unsafe fn wait_until(
        &self,
        instant: SyntonizedInstant,
    ) -> impl Future<Output = RadioTimerResult>;

    /// Schedule a hardware event, i.e. programs a signal to be sent over the
    /// event bus at a precise instant.
    ///
    /// This method provides access to deterministically timed events at
    /// hardware level without CPU intervention. Uses the high-precision timer.
    /// Exact timing specifications are implementation dependent.
    ///
    /// The method blocks asynchronously until the event was executed.
    ///
    /// Implementations SHALL be cancel-safe. Cancelling the future will cancel
    /// the alarm.
    ///
    /// Note: Cancellation may race with the timer interrupt, so if the future
    ///       is cancelled very close to expiry it may be that the signal will
    ///       still be produced.
    ///
    /// # Safety
    ///
    /// - This method SHALL be called from a context that runs at lower priority
    ///   than the timer interrupt(s).
    /// - The resulting future SHALL always be polled with the same waker, i.e.
    ///   it SHALL NOT be migrated to a different task. Wakers MAY change on
    ///   subsequent invocations of the method, though.
    ///
    unsafe fn schedule_event(
        &self,
        instant: SyntonizedInstant,
        signal: HardwareSignal,
    ) -> impl Future<Output = RadioTimerResult>;
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    #![allow(non_camel_case_types, non_snake_case)]
    use core::ptr::null_mut;

    pub type SEGGER_SYSVIEW_MODULE = SEGGER_SYSVIEW_MODULE_STRUCT;

    #[repr(C)]
    #[derive(Debug, Copy, Clone)]
    pub struct SEGGER_SYSVIEW_MODULE_STRUCT {
        pub sModule: *const cty::c_char,
        pub NumEvents: cty::c_ulong,
        pub EventOffset: cty::c_ulong,
        pub pfSendModuleDesc: Option<unsafe extern "C" fn()>,
        pub pNext: *mut SEGGER_SYSVIEW_MODULE,
    }

    unsafe extern "C" {
        pub fn SEGGER_SYSVIEW_RegisterModule(pModule: *mut SEGGER_SYSVIEW_MODULE);

        pub fn SEGGER_SYSVIEW_RecordU32x2(
            EventId: cty::c_uint,
            Para0: cty::c_ulong,
            Para1: cty::c_ulong,
        );

        pub fn SEGGER_SYSVIEW_RecordU32x3(
            EventId: cty::c_uint,
            Para0: cty::c_ulong,
            Para1: cty::c_ulong,
            Para2: cty::c_ulong,
        );
    }

    static TIMER_MODULE_DESC: &str = "M=timer, \
        0 SchedAlarm ns=%u rt=%u, \
        1 SchedSignal ns=%u rt=%u tt=%u\0";
    static mut TIMER_MODULE: SEGGER_SYSVIEW_MODULE = SEGGER_SYSVIEW_MODULE {
        sModule: TIMER_MODULE_DESC.as_ptr(),
        NumEvents: 2,
        EventOffset: 0,
        pfSendModuleDesc: None,
        pNext: null_mut(),
    };

    // Events
    pub const EVENT_SCHEDULE_ALARM: u32 = 0;
    pub const EVENT_SCHEDULE_SIGNAL: u32 = 1;

    pub fn instrument() {
        unsafe {
            SEGGER_SYSVIEW_RegisterModule(&raw mut TIMER_MODULE);
        }
    }

    pub fn record_schedule_alarm(ns: u32, rtc_ticks: u32) {
        unsafe {
            SEGGER_SYSVIEW_RecordU32x2(
                EVENT_SCHEDULE_ALARM + TIMER_MODULE.EventOffset,
                ns,
                rtc_ticks,
            );
        }
    }

    pub fn record_schedule_signal(ns: u32, rtc_ticks: u32, remaining_timer_ticks: u32) {
        unsafe {
            SEGGER_SYSVIEW_RecordU32x3(
                EVENT_SCHEDULE_SIGNAL + TIMER_MODULE.EventOffset,
                ns,
                rtc_ticks,
                remaining_timer_ticks,
            );
        }
    }
}
