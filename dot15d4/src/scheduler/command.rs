//! Scheduler command definitions.
//!
//! This module defines commands that can be sent to the scheduler to configure
//! its operation, switch modes, or modify PIB attributes.

use dot15d4_driver::radio::config::Channel;

#[cfg(feature = "tsch")]
use self::scan::{ScanCommand, ScanCommandResult};
#[cfg(feature = "tsch")]
use self::tsch::{TschCommand, TschCommandResult};

pub use self::pib::{PibCommand, PibCommandResult};

/// Commands that can be sent to the scheduler service.
///
/// These commands allow the MAC layer to configure scheduler behavior,
/// switch between CSMA and TSCH modes, and modify PIB attributes.
pub enum SchedulerCommand {
    /// TSCH-specific command (requires `tsch` feature).
    #[cfg(feature = "tsch")]
    TschCommand(TschCommand),
    /// CSMA-specific command.
    CsmaCommand(CsmaCommand),
    /// PIB attribute command.
    PibCommand(PibCommand),
    /// Channel Scanning Command
    #[cfg(feature = "tsch")]
    ScanCommand(ScanCommand),
}

/// Results returned from scheduler commands.
pub enum SchedulerCommandResult {
    /// Result of a TSCH command.
    #[cfg(feature = "tsch")]
    TschCommand(TschCommandResult),
    /// Result of a CSMA command.
    CsmaCommand(CsmaCommandResult),
    /// Result of a PIB command.
    PibCommand(PibCommandResult),
    /// Result of a Scanning Command
    #[cfg(feature = "tsch")]
    ScanCommand(ScanCommandResult),
}

/// CSMA-specific commands.
pub enum CsmaCommand {
    /// Switch to CSMA mode on the specified channel.
    UseCsma(Channel),
}

/// Result of switching to CSMA mode.
pub enum UseCsmaResult {
    /// Successfully switched to CSMA mode.
    Success,
    //TODO: handle channel switch failure ?
}

/// Results from CSMA commands.
pub enum CsmaCommandResult {
    /// Result of UseCsma command.
    UseCsma(UseCsmaResult),
}

/// PIB (PAN Information Base) command module.
pub mod pib {
    use dot15d4_driver::radio::frame::Address;

    use crate::mac::mlme::{get::GetRequestAttribute, set::SetRequestAttribute};
    use crate::mac::neighbors::NeighborSnapshot;

    /// PIB-related commands for the scheduler.
    pub enum PibCommand {
        /// Set a PIB attribute to a new value.
        Set(SetRequestAttribute),
        /// Get a PIB attribute
        Get(GetRequestAttribute),
        /// Reset PIB to default values (preserving extended address).
        Reset,
    }

    /// Result of a PIB Set operation.
    pub enum SetPibResult {
        /// Attribute was successfully set.
        Success,
        /// The provided value was invalid for the attribute.
        InvalidParameter,
    }

    /// Result of a PIB Get operation.
    pub enum GetPibResult {
        MacExtendedAddress(Address<[u8; 8]>),
        MacCoordExtendedAddress(Address<[u8; 8]>),
        MacAssociationPermit(bool),
        MacPanId(u16),
        MacShortAddress(u16),
        /// Snapshot of the neighbor table entry at the requested index, or
        /// `None` when that slot is unused.
        MacNeighborStats(Option<NeighborSnapshot>),
        InvalidParameter,
    }

    /// Result of a PIB Reset operation.
    pub enum ResetPibResult {
        /// PIB was successfully reset to defaults.
        Success,
    }

    /// Result of a PIB command.
    pub enum PibCommandResult {
        /// Result of a Set command.
        Set(SetPibResult),
        /// Result of a Set command.
        Get(GetPibResult),
        /// Result of a Reset command.
        Reset(ResetPibResult),
    }
}

/// TSCH (Time Slotted Channel Hopping) command module.
#[cfg(feature = "tsch")]
pub mod tsch {
    use crate::mac::mlme::tsch::{setlink::SetLinkRequest, setslotframe::SetSlotframeRequest};

    /// Role configuration for a TSCH session, carried by the mode switch
    /// (MLME-TSCH-MODE.request).
    ///
    /// The two aspects are independent: in a multi-hop network every
    /// forwarding node sends Enhanced Beacons, but only one node (the time
    /// root) never adjusts its own clock.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct TschRoleConfig {
        /// The single node whose clock is the network time origin. It never
        /// applies frame- or acknowledgment-based corrections. Every other
        /// node synchronizes to its time source.
        pub is_time_root: bool,
        /// Enhanced Beacon transmission period in seconds; `None` disables
        /// beaconing.
        pub beacon_period_secs: Option<u32>,
        /// Random `+/-` jitter applied to each beacon period, in
        /// milliseconds, so nodes with the same period do not collide
        /// repeatedly in shared advertising cells.
        pub beacon_jitter_ms: u16,
    }

    impl TschRoleConfig {
        /// The network time origin, advertising with the given beacon period
        /// (the classic PAN coordinator role).
        pub fn time_root(beacon_period_secs: u32) -> Self {
            Self {
                is_time_root: true,
                beacon_period_secs: Some(beacon_period_secs),
                beacon_jitter_ms: 0,
            }
        }

        /// A joined device that keeps synchronizing to its time source and
        /// does not beacon. Enable beaconing with [`Self::with_beacon`] to
        /// make it a forwarding node.
        pub fn device() -> Self {
            Self {
                is_time_root: false,
                beacon_period_secs: None,
                beacon_jitter_ms: 0,
            }
        }

        /// Enable Enhanced Beacon transmission with the given period.
        pub fn with_beacon(mut self, period_secs: u32) -> Self {
            self.beacon_period_secs = Some(period_secs);
            self
        }

        /// Apply a random `+/-` jitter to each beacon period.
        pub fn with_beacon_jitter_ms(mut self, jitter_ms: u16) -> Self {
            self.beacon_jitter_ms = jitter_ms;
            self
        }
    }

    impl Default for TschRoleConfig {
        fn default() -> Self {
            Self::device()
        }
    }

    /// TSCH-specific commands.
    pub enum TschCommand {
        /// Enable or disable TSCH mode.
        UseTsch {
            /// `true` to start TSCH, `false` to stop.
            enabled: bool,
            /// `true` to use CCA before transmission.
            cca: bool,
            /// Role for the TSCH session being started.
            role: TschRoleConfig,
        },
        /// Configure a TSCH slotframe.
        SetTschSlotframe(SetSlotframeRequest),
        /// Configure a TSCH link within a slotframe.
        SetTschLink(SetLinkRequest),
    }

    /// Result of UseTsch command.
    pub enum UseTschCommandResult {
        /// TSCH mode was successfully started.
        StartedTsch,
        /// TSCH mode was successfully stopped.
        StoppedTsch,
    }

    /// Result of SetTschSlotframe command.
    pub enum SetTschSlotframeResult {
        /// Slotframe was successfully configured.
        Success,
        /// The specified slotframe was not found.
        SlotframeNotFound,
        /// Maximum number of slotframes exceeded.
        MaxSlotframesExceeded,
    }

    /// Result of SetTschLink command.
    pub enum SetTschLinkResult {
        /// Link was successfully configured.
        Success,
        /// The specified link was not found.
        UnknownLink,
        /// Maximum number of links exceeded.
        MaxLinksExceeded,
    }

    /// Results from TSCH commands.
    pub enum TschCommandResult {
        /// Result of UseTsch command.
        UseTsch(UseTschCommandResult),
        /// Result of SetTschSlotframe command.
        SetTschSlotframe(SetTschSlotframeResult),
        /// Result of SetTschLink command.
        SetTschLink(SetTschLinkResult),
    }
}

#[cfg(feature = "tsch")]
pub mod scan {

    use crate::scheduler::scan::ScanChannels;
    use crate::scheduler::SchedulerReceptionResult;

    /// TSCH-specific commands.
    pub enum ScanCommand {
        /// Start scanning channels
        StartScanning(ScanChannels, usize),
        StopScanning,
    }

    /// Result of UseTsch command.
    pub enum ScanCommandResult {
        /// TSCH mode was successfully started.
        StartedScanning,
        /// TSCH mode was successfully stopped.
        StoppedScanning(Option<SchedulerReceptionResult>),
    }
}
