#[cfg(feature = "tsch")]
use crate::mac::mlme::tsch::{setlink::SetLinkRequest, setslotframe::SetSlotframeRequest};

pub enum SchedulerCommand {
    #[cfg(feature = "tsch")]
    UseTsch(
        /// Used to indicate if the TSCH mode is to be started or stopped
        bool,
        /// Used to indicate that CCA is to be used for transmission
        bool,
    ),
    #[cfg(feature = "tsch")]
    SetTschSlotframe(SetSlotframeRequest),
    #[cfg(feature = "tsch")]
    SetTschLink(SetLinkRequest),
    UseCsma,
}

pub enum SchedulerCommandResult {
    #[cfg(feature = "tsch")]
    UseTsch(UseTschCommandResult),
    #[cfg(feature = "tsch")]
    SetTschSlotframe(SetTschSlotframeResult),
    #[cfg(feature = "tsch")]
    SetTschLink(SetTschLinkResult),
}

#[cfg(feature = "tsch")]
pub enum UseTschCommandResult {
    StartedTsch,
    StoppedTsch,
}

#[cfg(feature = "tsch")]
pub enum SetTschSlotframeResult {
    Success,
    SlotframeNotFound,
    MaxSlotframesExceeded,
}

#[cfg(feature = "tsch")]
pub enum SetTschLinkResult {
    Success,
    UnknownLink,
    MaxLinksExceeded,
}
