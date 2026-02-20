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

#[cfg(test)]
mod scan_tests {
    use core::num::NonZero;

    use super::*;
    use crate::{
        mac::{
            mlme::scan::{ScanRequest, ScanRequestTask, ScanStatus, ScanType, MAX_PAN_DESCRIPTORS},
            tests::{beacon_reception_response, MacTaskTestRunner, RequestType},
        },
        scheduler::{
            command::scan::ScanCommandResult,
            scan::ScanChannels,
            tests::{create_test_allocator, FakeDriverConfig},
            SchedulerCommand, SchedulerCommandResult,
        },
    };
    use dot15d4_driver::radio::{config::Channel, frame::RadioFrameUnsized};

    fn setup_passive_all() -> MacTaskTestRunner<ScanRequestTask<'static, FakeDriverConfig>> {
        let allocator = create_test_allocator();
        let request = ScanRequest::passive_all(5, MAX_PAN_DESCRIPTORS);
        let task = ScanRequestTask::new(request);
        MacTaskTestRunner::new(task, allocator)
    }

    fn setup_passive_single(
        channel: Channel,
        max_descriptors: usize,
    ) -> MacTaskTestRunner<ScanRequestTask<'static, FakeDriverConfig>> {
        let allocator = create_test_allocator();
        let request = ScanRequest::passive_single(channel, 5, max_descriptors);
        let task = ScanRequestTask::new(request);
        MacTaskTestRunner::new(task, allocator)
    }

    fn started_scanning_response() -> SchedulerResponse {
        SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
            ScanCommandResult::StartedScanning,
        ))
    }

    fn stopped_scanning_response() -> SchedulerResponse {
        SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
            ScanCommandResult::StoppedScanning,
        ))
    }

    /// Create a minimal beacon-like radio frame for scan test responses.
    fn create_beacon_frame(
        runner: &MacTaskTestRunner<ScanRequestTask<'static, FakeDriverConfig>>,
    ) -> RadioFrame<RadioFrameSized> {
        let allocator = runner.allocator();
        let buffer = allocator
            .try_allocate_buffer(130)
            .expect("allocation failed");
        let frame = RadioFrame::<RadioFrameUnsized>::new::<FakeDriverConfig>(buffer);
        // Give it a minimal size (frame control + sequence number).
        frame.with_size(NonZero::new(10).unwrap())
    }

    // ========================================================================
    // Entry
    // ========================================================================

    #[test]
    fn scan_entry_sends_start_scanning_command() {
        let mut runner = setup_passive_all();

        let outcome = runner.step_entry();
        let request = outcome.unwrap_request();

        match request {
            SchedulerRequest::Command(SchedulerCommand::ScanCommand(
                crate::scheduler::command::scan::ScanCommand::StartScanning(channels, max),
            )) => {
                assert!(matches!(channels, ScanChannels::All));
                assert_eq!(max, MAX_PAN_DESCRIPTORS);
            }
            _ => panic!("expected ScanCommand::StartScanning"),
        }
    }

    #[test]
    fn scan_single_channel_entry() {
        let mut runner = setup_passive_single(Channel::_15, 1);

        let outcome = runner.step_entry();
        let request = outcome.unwrap_request();

        match request {
            SchedulerRequest::Command(SchedulerCommand::ScanCommand(
                crate::scheduler::command::scan::ScanCommand::StartScanning(channels, max),
            )) => {
                assert!(matches!(channels, ScanChannels::Single(Channel::_15)));
                assert_eq!(max, 1);
            }
            _ => panic!("expected ScanCommand::StartScanning"),
        }
    }

    // ========================================================================
    // Started Scanning -> Beacon Reception Request
    // ========================================================================

    #[test]
    fn scan_started_requests_beacon_reception() {
        let mut runner = setup_passive_all();
        runner.step_entry();

        let outcome = runner.step_response(started_scanning_response());
        let request = outcome.unwrap_request();

        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::Beacon)
        );
    }

    // ========================================================================
    // Beacon Reception
    // ========================================================================

    #[test]
    fn scan_terminates_on_max_descriptors() {
        let mut runner = setup_passive_single(Channel::_26, 1);
        runner.step_entry();
        runner.step_response(started_scanning_response());

        let beacon = create_beacon_frame(&runner);
        let outcome = runner.step_response(beacon_reception_response(
            beacon,
            NsInstant::from_ticks(1000),
        ));

        // max_pan_descriptors=1 and we received 1 beacon -> terminate.
        let confirm = outcome.unwrap_terminated();
        assert_eq!(confirm.scan_type, ScanType::Passive);
        assert_eq!(confirm.pan_descriptor_list.len(), 1);

        // Clean up the MpduFrame buffers inside the PanDescriptors.
        for descriptor in confirm.pan_descriptor_list {
            runner.track_mpdu(descriptor.mpdu);
        }
    }

    #[test]
    fn scan_two_beacons_fills_results() {
        let mut runner = setup_passive_all();
        runner.step_entry();
        runner.step_response(started_scanning_response());

        // First beacon -> continues.
        let beacon1 = create_beacon_frame(&runner);
        let outcome = runner.step_response(beacon_reception_response(
            beacon1,
            NsInstant::from_ticks(1000),
        ));
        assert!(outcome.is_pending());

        // Second beacon -> terminates (MAX_PAN_DESCRIPTORS = 2).
        let beacon2 = create_beacon_frame(&runner);
        let outcome = runner.step_response(beacon_reception_response(
            beacon2,
            NsInstant::from_ticks(2000),
        ));

        let confirm = outcome.unwrap_terminated();
        assert_eq!(confirm.pan_descriptor_list.len(), 2);
        assert_eq!(
            confirm.pan_descriptor_list[0].timestamp,
            NsInstant::from_ticks(1000)
        );
        assert_eq!(
            confirm.pan_descriptor_list[1].timestamp,
            NsInstant::from_ticks(2000)
        );

        for descriptor in confirm.pan_descriptor_list {
            runner.track_mpdu(descriptor.mpdu);
        }
    }

    // ========================================================================
    // Stopped Scanning
    // ========================================================================

    #[test]
    fn scan_terminates_on_stopped_scanning() {
        let mut runner = setup_passive_all();
        runner.step_entry();
        runner.step_response(started_scanning_response());

        let outcome = runner.step_response(stopped_scanning_response());

        let confirm = outcome.unwrap_terminated();
        assert_eq!(confirm.status, ScanStatus::Success);
        assert!(confirm.pan_descriptor_list.is_empty());
    }

    #[test]
    fn scan_stopped_after_one_beacon() {
        let mut runner = setup_passive_all();
        runner.step_entry();
        runner.step_response(started_scanning_response());

        // Receive one beacon, then stop.
        let beacon = create_beacon_frame(&runner);
        runner.step_response(beacon_reception_response(
            beacon,
            NsInstant::from_ticks(500),
        ));

        let outcome = runner.step_response(stopped_scanning_response());
        let confirm = outcome.unwrap_terminated();
        assert_eq!(confirm.pan_descriptor_list.len(), 1);

        for descriptor in confirm.pan_descriptor_list {
            runner.track_mpdu(descriptor.mpdu);
        }
    }

    // ========================================================================
    // Full Passive Scan Flow
    // ========================================================================

    #[test]
    fn scan_full_passive_flow() {
        let mut runner = setup_passive_all();

        // 1. Entry -> StartScanning command
        let outcome = runner.step_entry();
        assert!(outcome.is_pending());

        // 2. StartedScanning -> beacon reception request
        let outcome = runner.step_response(started_scanning_response());
        assert!(outcome.is_pending());

        // 3. Beacon received -> continues listening
        let beacon = create_beacon_frame(&runner);
        let outcome = runner.step_response(beacon_reception_response(
            beacon,
            NsInstant::from_ticks(100),
        ));
        assert!(outcome.is_pending());

        // 4. Second beacon -> terminates (max reached)
        let beacon = create_beacon_frame(&runner);
        let outcome = runner.step_response(beacon_reception_response(
            beacon,
            NsInstant::from_ticks(200),
        ));

        let confirm = outcome.unwrap_terminated();
        assert_eq!(confirm.scan_type, ScanType::Passive);
        assert_eq!(confirm.pan_descriptor_list.len(), 2);

        for descriptor in confirm.pan_descriptor_list {
            runner.track_mpdu(descriptor.mpdu);
        }
    }
}
