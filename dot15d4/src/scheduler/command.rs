use crate::mac::mlme::tsch::{setlink::SetLinkRequest, setslotframe::SetSlotframeRequest};

pub enum SchedulerCommand {
    UseTsch(
        /// Used to indicate if the TSCH mode is to be started or stopped
        bool,
        /// Used to indicate that CCA is to be used for transmission
        bool,
    ),
    SetTschSlotframe(SetSlotframeRequest),
    SetTschLink(SetLinkRequest),
    UseCsma,
}

pub enum SchedulerCommandResult {
    UseTsch(UseTschCommandResult),
    SetTschSlotframe(SetTschSlotframeResult),
    SetTschLink(SetTschLinkResult),
}

pub enum UseTschCommandResult {
    StartedTsch,
    StoppedTsch,
}

pub enum SetTschSlotframeResult {
    Success,
    SlotframeNotFound,
    MaxSlotframesExceeded,
}

pub enum SetTschLinkResult {
    Success,
    UnknownLink,
    MaxLinksExceeded,
}
