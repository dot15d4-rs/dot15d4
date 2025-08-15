#![allow(dead_code)]
use core::ops::RangeInclusive;

use crate::{driver::radio::DriverConfig, mac::MacService};

pub enum ScanType {
    Ed,
    Active,
    Passive,
    Orphan,
    EnhancedActiveScan,
}

pub enum ScanChannels {
    All,
    Single(u8),
}

pub struct ScanConfirm {
    scan_type: ScanType,
    channel_page: u8,
}
pub enum ScanError {
    // TODO: not supported
    LimitReached,
    // TODO: not supported
    NoBeacon,
    // TODO: not supported
    ScanInProgress,
    // TODO: not supported
    CounterError,
    // TODO: not supported
    FrameTooLong,
    // TODO: not supported
    BadChannel,
    // TODO: not supported
    InvalidParameter,
}

impl<'svc, RadioDriverImpl: DriverConfig> MacService<'svc, RadioDriverImpl> {
    /// Initiates a channel scan over a given set of channels.
    pub(crate) async fn mlme_scan_request(
        &self,
        _scan_type: ScanType,
        _scan_channels: ScanChannels,
        _scan_duration: u8,
        _channel_page: u8,
    ) -> Result<ScanConfirm, ScanError> {
        Err(ScanError::InvalidParameter)
    }
}

// Implement IntoIterator for Channels so you can write: for x in scan_channels { ... }
impl IntoIterator for ScanChannels {
    type Item = u8;
    type IntoIter = RangeInclusive<u8>;

    fn into_iter(self) -> Self::IntoIter {
        match self {
            ScanChannels::All => 11_u8..=26,
            ScanChannels::Single(ch) => ch..=ch,
        }
    }
}
