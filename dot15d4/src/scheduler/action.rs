//! Action types for scheduler service.
//!
//! These types define what the async runner should do (SchedulerAction)
//! and what inputs the sync logic receives (InputEvent).

use dot15d4_driver::timer::NsInstant;

use crate::driver::DrvSvcRequest;

/// Actions that the async runner should execute.
///
/// Returned by sync logic methods to direct the async runner.
pub enum SchedulerAction {
    /// Send driver request, then immediately wait for driver event.
    /// Optimization to avoid extra round-trip through logic.
    SendDriverRequestThenWait(DrvSvcRequest),
    /// Wait for an event from the driver service.
    WaitForDriverEvent,
    /// Wait for a request from the MAC layer (scheduler channel).
    WaitForSchedulerRequest,
    /// Select: wait for driver event OR scheduler request.
    /// Used in CSMA when waiting for frames but may receive TX request.
    SelectDriverEventOrRequest,
    /// Select: wait for timer expiry OR scheduler request.
    /// Used in TSCH to wake up for next slot or receive new requests.
    #[cfg(feature = "tsch")]
    WaitForTimeoutOrSchedulerRequest { deadline: NsInstant },
    /// Send driver request, then select: wait for driver event OR scheduler request.
    /// Used when starting RX - we want to receive frames but also handle TX requests.
    SendDriverRequestThenSelect(DrvSvcRequest),
}
