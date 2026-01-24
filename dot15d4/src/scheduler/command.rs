use dot15d4_driver::radio::config::Channel;

#[cfg(feature = "tsch")]
use self::tsch::{TschCommand, TschCommandResult};

pub use self::pib::{PibCommand, PibCommandResult};

pub enum SchedulerCommand {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommand),
    CsmaCommand(CsmaCommand),
    PibCommand(PibCommand),
}

pub enum SchedulerCommandResult {
    #[cfg(feature = "tsch")]
    TschCommand(TschCommandResult),
    CsmaCommand(CsmaCommandResult),
    PibCommand(PibCommandResult),
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

pub mod pib {
    use crate::mac::mlme::set::SetRequestAttribute;

    /// PIB-related commands for the scheduler
    pub enum PibCommand {
        /// Set a PIB attribute
        Set(SetRequestAttribute),
        /// Reset PIB to default values (preserving extended address)
        Reset,
    }

    /// Result of a PIB Set operation
    pub enum SetPibResult {
        Success,
        InvalidParameter,
    }

    /// Result of a PIB Reset operation
    pub enum ResetPibResult {
        Success,
    }

    /// Result of a PIB command
    pub enum PibCommandResult {
        Set(SetPibResult),
        Reset(ResetPibResult),
    }
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
