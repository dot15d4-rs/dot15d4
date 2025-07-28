use core::fmt::Debug;

use generic_array::ArrayLength;

use crate::timer::RadioTimerApi;

pub mod export {
    pub use generic_array::ArrayLength;
    pub use typenum::{Unsigned, U};
}

// TODO: Move this to an external per-driver config.
pub const MAX_DRIVER_OVERHEAD: usize = 2;

/// Type allowed for [`DriverConfig::Fcs`]
/// Drivers for LECIM, TVWS and SUN PHYs may be configured with a 4-byte FCS, all
pub type FcsFourBytes = u32;

/// Type allowed for [`DriverConfig::Fcs`]
/// Most drivers/PHYs use two bytes.
pub type FcsTwoBytes = u16;

/// Type allowed for [`DriverConfig::Fcs`]
/// Drivers that offload FCS (=CRC) checking to hardware will neither require
/// nor include an FCS in the frame.
pub type FcsNone = ();

// TODO: Convert into a runtime construct so that we can address multiple
//       radios and get rid of the generic. This can be done with minimal
//       overhead as higher-layer representations need to save headroom,
//       tailroom and FCS ranges anyway.
pub trait DriverConfig {
    /// Any buffer headroom required by the driver.
    type Headroom: ArrayLength;

    /// Any buffer tailroom required by the driver. If the driver takes care of
    /// FCS handling (see [`FcsNone`]), then the tailroom may have to include
    /// the required bytes to let the hardware add the FCS.
    type Tailroom: ArrayLength;

    /// aMaxPhyPacketSize if the FCS is handled by the MAC, otherwise
    /// aMaxPhyPacketSize minus the FCS size.
    type MaxSduLength: ArrayLength;

    /// FCS handling:
    ///  - [`FcsTwoBytes`]: No FCS handling inside the driver or hardware. The
    ///    driver expects the framework to calculate and inject a 2-byte FCS
    ///    into the frame.
    ///  - [`FcsFourBytes`]: No FCS handling inside the driver or hardware. The
    ///    driver expects the framework to calculate and inject a 4-byte FCS
    ///    into the frame.
    ///  - [`FcsNone`]: FCS handling is offloaded to the driver or hardware. The
    ///    driver expects the framework to end the MPDU after the frame payload
    ///    without any FCS. If the driver or hardware requires buffer space for
    ///    its own FCS handling, then it must be included in the tailroom.
    type Fcs: Copy + Debug;

    /// The radio timer implementation.
    type Timer: RadioTimerApi;
}

pub type Timer<RadioDriverImpl> = <RadioDriverImpl as DriverConfig>::Timer;

/// Basic features to be implemented by all radio drivers, independent of driver
/// state.
pub trait RadioDriverApi {
    fn ieee802154_address(&self) -> [u8; 8];
}

#[cfg(feature = "rtos-trace")]
pub(crate) mod trace {
    // Tasks
    pub const TASK_OFF_SCHEDULE: u32 = 200;
    pub const TASK_TRANSITION_TO_OFF: u32 = 201;
    pub const TASK_OFF_RUN: u32 = 202;

    pub const TASK_RX_SCHEDULE: u32 = 203;
    pub const TASK_TRANSITION_TO_RX: u32 = 204;
    pub const TASK_RX_RUN: u32 = 205;

    pub const TASK_TX_SCHEDULE: u32 = 206;
    pub const TASK_TRANSITION_TO_TX: u32 = 207;
    pub const TASK_TX_RUN: u32 = 208;

    pub const TASK_FALL_BACK: u32 = 209;

    // Markers
    pub const MISSED_ISR: u32 = 200;
    pub const TASK_RX_FRAME_STARTED: u32 = 201;
    pub const TASK_RX_FRAME_INFO: u32 = 202;

    /// Instruments the driver for task tracing.
    pub fn instrument() {
        rtos_trace::trace::task_new_stackless(TASK_OFF_SCHEDULE, "Schedule Off\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TRANSITION_TO_OFF, "Transition to Off\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_OFF_RUN, "Off\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_RX_SCHEDULE, "Schedule Rx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TRANSITION_TO_RX, "Transition to RX\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_RX_RUN, "Rx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TX_SCHEDULE, "Schedule Tx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TRANSITION_TO_TX, "Transition to TX\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_TX_RUN, "Tx\0", 0);
        rtos_trace::trace::task_new_stackless(TASK_FALL_BACK, "Off (fallback)\0", 0);
        rtos_trace::trace::name_marker(MISSED_ISR, "Missed ISR\0");
        rtos_trace::trace::name_marker(TASK_RX_FRAME_STARTED, "Frame Started\0");
        rtos_trace::trace::name_marker(TASK_RX_FRAME_INFO, "Preliminary Frame Info\0");
    }
}
