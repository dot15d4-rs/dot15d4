//! MLME-SCAN primitive implementation for IEEE 802.15.4-2024.
//!
//! The MLME-SCAN primitive is used to discover devices in the radio range,
//! or to measure energy levels on channels.
//!
//! This implementation supports:
//! - Passive scan: Listen for beacons on each channel

#![allow(dead_code)]

use core::marker::PhantomData;

use dot15d4_driver::{
    radio::{config::Channel, frame::RadioFrameSized, DriverConfig},
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use heapless::Vec;

use crate::{
    driver::radio::frame::RadioFrame,
    mac::{
        frame::mpdu::MpduFrame,
        task::{MacTask, MacTaskEvent, MacTaskTransition},
    },
    scheduler::{
        command::scan::{ScanCommand, ScanCommandResult},
        scan::ScanChannels,
        ReceptionType, SchedulerCommand, SchedulerCommandResult, SchedulerReceptionResult,
        SchedulerRequest, SchedulerResponse,
    },
};

/// Type of scan to perform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanType {
    /// Active scan - send beacon request and listen for beacons.
    Active,
    /// Passive scan - listen for beacons without transmitting.
    Passive,
    /// Enhanced active scan with Information Elements.
    EnhancedActiveScan,
}

/// MLME-SCAN.request parameters.
#[derive(Debug, Clone)]
pub struct ScanRequest {
    /// Type of scan to perform.
    pub scan_type: ScanType,
    /// Channels to scan.
    pub scan_channels: ScanChannels,
    /// Scan duration exponent (0-14). Duration per channel = aBaseSuperframeDuration * (2^n + 1).
    /// For n=14, this is approximately 4 minutes per channel.
    /// A value of 5 gives approximately 1 second per channel.
    pub scan_duration: u8,
    pub max_pan_descriptors: usize,
}

impl ScanRequest {
    /// Create a new passive scan request for all channels.
    pub fn passive_all(scan_duration: u8, max_pan_descriptors: usize) -> Self {
        Self {
            scan_type: ScanType::Passive,
            scan_channels: ScanChannels::All,
            scan_duration,
            max_pan_descriptors,
        }
    }

    /// Create a new passive scan request for a single channel.
    pub fn passive_single(channel: Channel, scan_duration: u8, max_pan_descriptors: usize) -> Self {
        Self {
            scan_type: ScanType::Passive,
            scan_channels: ScanChannels::Single(channel),
            scan_duration,
            max_pan_descriptors,
        }
    }
}

/// Descriptor for a PAN discovered during scanning.
pub struct PanDescriptor {
    pub mpdu: MpduFrame,
    /// Timestamp when the beacon was received.
    pub timestamp: NsInstant,
    /// Link quality indicator.
    pub link_quality: u8,
}

/// MLME-SCAN.confirm parameters.
pub struct ScanConfirm<const MAX_RESULTS: usize = MAX_PAN_DESCRIPTORS> {
    /// Status of the scan operation.
    pub status: ScanStatus,
    /// Type of scan that was performed.
    pub scan_type: ScanType,
    /// PAN descriptors found (for active/passive scans).
    pub pan_descriptor_list: Vec<PanDescriptor, MAX_RESULTS>,
}

impl<const MAX_RESULTS: usize> ScanConfirm<MAX_RESULTS> {
    /// Create a new empty scan confirm with success status.
    pub fn new(scan_type: ScanType) -> Self {
        Self {
            status: ScanStatus::Success,
            scan_type,
            pan_descriptor_list: Vec::new(),
        }
    }

    /// Add a PAN descriptor to the results.
    pub fn add_pan_descriptor(&mut self, descriptor: PanDescriptor) -> bool {
        self.pan_descriptor_list.push(descriptor).is_ok()
    }
}

/// Status codes for MLME-SCAN.confirm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanStatus {
    /// Scan completed successfully.
    Success,
    /// Scan limit was reached (max PANs found).
    LimitReached,
    /// No beacon was found during scan.
    NoBeacon,
    /// A scan is already in progress.
    ScanInProgress,
    /// Counter error occurred.
    CounterError,
    /// Frame was too long.
    FrameTooLong,
    /// Invalid channel specified.
    BadChannel,
    /// Invalid parameter.
    InvalidParameter,
}

/// Legacy error type for backward compatibility.
pub type ScanError = ScanStatus;

// ============================================================================
// MLME-SCAN Task State Machine
// ============================================================================

/// Maximum number of PAN descriptors that can be stored.
pub const MAX_PAN_DESCRIPTORS: usize = 2;

/// State of the MLME-SCAN task.
pub(crate) enum ScanState {
    /// Initial state - Send Scan command
    Initial(ScanRequest),
    /// Waiting for scheduler confirmation of starting of scan procedure
    WaitingForScanStart,
    /// Waiting for reception result on current channel.
    ScanningChannel,
}

/// MLME-SCAN task for passive scanning.
pub(crate) struct ScanRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: ScanState,
    /// Duration per channel in nanoseconds.
    duration_per_channel: NsDuration,
    results: ScanConfirm<MAX_PAN_DESCRIPTORS>,
    max_pan_descriptors: usize,
    _task: PhantomData<&'task ()>,
    _radio: PhantomData<RadioDriverImpl>,
}

impl<'task, RadioDriverImpl: DriverConfig> ScanRequestTask<'task, RadioDriverImpl> {
    /// Create a new scan request task.
    pub fn new(request: ScanRequest) -> Self {
        //TODO: handle scan_duration exponent instead of arbitrary value
        let duration_per_channel = NsDuration::secs(4);
        let scan_type = request.scan_type;
        let max_pan_descriptors = request.max_pan_descriptors;
        Self {
            state: ScanState::Initial(request),
            duration_per_channel,
            results: ScanConfirm::new(scan_type),
            max_pan_descriptors,
            _task: PhantomData,
            _radio: PhantomData,
        }
    }
}

impl<RadioDriverImpl> MacTask for ScanRequestTask<'_, RadioDriverImpl>
where
    RadioDriverImpl: DriverConfig,
    RadioDriverImpl::Timer: RadioTimerApi,
{
    type Result = ScanConfirm<MAX_PAN_DESCRIPTORS>;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        match self.state {
            ScanState::Initial(request) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                assert!(request.max_pan_descriptors <= MAX_PAN_DESCRIPTORS);

                // TODO: support active and other types
                let _scan_type = request.scan_type;

                self.state = ScanState::WaitingForScanStart;

                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::ScanCommand(
                        ScanCommand::StartScanning(
                            request.scan_channels,
                            request.max_pan_descriptors,
                        ),
                    )),
                    None,
                )
            }
            ScanState::WaitingForScanStart => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::ScanCommand(ScanCommandResult::StartedScanning),
                )) => {
                    self.state = ScanState::ScanningChannel;
                    MacTaskTransition::SchedulerRequest(
                        self,
                        SchedulerRequest::Reception(ReceptionType::Beacon),
                        None,
                    )
                }
                _ => unreachable!(),
            },
            ScanState::ScanningChannel => {
                match event {
                    MacTaskEvent::SchedulerResponse(response) => match response {
                        SchedulerResponse::Reception(SchedulerReceptionResult::Beacon(
                            beacon_frame,
                            timestamp,
                        )) => {
                            // A frame was received - check if it's a beacon
                            self.process_received_frame(beacon_frame, timestamp)
                        }
                        SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                            ScanCommandResult::StoppedScanning,
                        )) => MacTaskTransition::Terminated(self.results),
                        _ => unreachable!(),
                    },
                    _ => unreachable!(),
                }
            }
        }
    }
}

impl<'task, RadioDriverImpl: DriverConfig> ScanRequestTask<'task, RadioDriverImpl> {
    /// Process a received frame and extract beacon information if applicable.
    fn process_received_frame(
        mut self,
        beacon_frame: RadioFrame<RadioFrameSized>,
        timestamp: NsInstant,
    ) -> MacTaskTransition<Self> {
        // Convert to MPDU and check frame type
        let mpdu = MpduFrame::from_radio_frame(beacon_frame);
        let descriptor = PanDescriptor {
            mpdu,
            timestamp,
            link_quality: 0, // TODO: Get from PHY if available
        };

        // Try to add to results (may fail if full)
        if !self.results.add_pan_descriptor(descriptor) {
            self.results.status = ScanStatus::LimitReached;
        }
        if self.results.pan_descriptor_list.len() == self.max_pan_descriptors {
            MacTaskTransition::Terminated(self.results)
        } else {
            MacTaskTransition::SchedulerRequest(
                self,
                SchedulerRequest::Reception(ReceptionType::Beacon),
                None,
            )
        }
    }
}
