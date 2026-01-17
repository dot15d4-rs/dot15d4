use dot15d4_driver::radio::config::Channel;

#[cfg(feature = "tsch")]
use self::tsch::{TschCommand, TschCommandResult};

pub enum SchedulerCommand {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommand),
    CsmaCommand(CsmaCommand),
}

pub enum SchedulerCommandResult {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommandResult),
    CsmaCommand(CsmaCommandResult),
}

pub enum CsmaCommand {
    UseCsma(Channel),
}

pub enum UseCsmaResult {
    Success,
}

pub enum CsmaCommandResult {
    UseCsma(UseCsmaResult),
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
