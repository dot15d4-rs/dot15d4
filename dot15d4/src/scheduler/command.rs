#[cfg(feature = "tsch")]
use self::tsch::{TschCommand, TschCommandResult};

pub enum SchedulerCommand {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommand),
    UseCsma,
}

pub enum SchedulerCommandResult {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommandResult),
}

#[cfg(feature = "tsch")]
pub mod tsch {
    use crate::mac::mlme::tsch::{setlink::SetLinkRequest, setslotframe::SetSlotframeRequest};

    pub enum TschCommand {
        UseTsch(
            /// Used to indicate if the TSCH mode is to be started or stopped
            bool,
            /// Used to indicate that CCA is to be used for transmission
            bool,
        ),
        SetTschSlotframe(SetSlotframeRequest),
        SetTschLink(SetLinkRequest),
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
    pub enum TschCommandResult {
        UseTsch(UseTschCommandResult),
        SetTschSlotframe(SetTschSlotframeResult),
        SetTschLink(SetTschLinkResult),
    }
}
