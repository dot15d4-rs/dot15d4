use core::{marker::PhantomData, mem};

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{self, FrameType, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        phy::PhyConfig,
        DriverConfig, PhyOf,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};

use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, Timestamp},
    scheduler::{
        command::scan::ScanCommandResult, ReceptionType, SchedulerCommand, SchedulerCommandResult,
        SchedulerReceptionResult, SchedulerRequest, SchedulerResponse, SchedulerTransmissionResult,
    },
};

use super::{SchedulerAction, SchedulerTaskCompletion};
use super::{SchedulerContext, SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition};

/// Default scan duration per channel
const SCAN_DURATION: u64 = 60;

/// Specification of which channels to scan.
#[derive(Debug, Clone, Copy)]
pub enum ScanChannels {
    /// Scan all channels (11-26 for 2.4GHz O-QPSK).
    All,
    /// Scan a single channel.
    Single(Channel),
    /// Scan channels specified by a bitmask (bit 11 = channel 11, etc.).
    Bitmask(u32),
}

impl ScanChannels {
    /// Returns an iterator over the channels to scan.
    pub fn iter(&self) -> ScanChannelsIter {
        match self {
            ScanChannels::All => ScanChannelsIter {
                current: 11,
                mask: 0xFFFF_FFFF,
            },
            ScanChannels::Single(ch) => {
                let ch_num: u8 = (*ch).into();
                ScanChannelsIter {
                    current: ch_num,
                    mask: 1 << ch_num,
                }
            }
            ScanChannels::Bitmask(mask) => ScanChannelsIter {
                current: 11,
                mask: *mask,
            },
        }
    }
}

/// Iterator over scan channels.
pub struct ScanChannelsIter {
    current: u8,
    mask: u32,
}

impl Iterator for ScanChannelsIter {
    type Item = Channel;

    fn next(&mut self) -> Option<Self::Item> {
        while self.current <= 26 {
            let ch = self.current;
            self.current += 1;
            if (self.mask & (1 << ch)) != 0 {
                return Channel::try_from(ch).ok();
            }
        }
        None
    }
}

/// Scan task state machine states.
pub enum ScanTaskState {
    Initial,
    ScanningChannel(Channel),
    ReceivingFrame(Channel),
    Placeholder,
}

/// Scheduler task for passive channel scanning.
///
/// Iterates over the requested channels, listens for beacons on each, and
/// forwards them to the MAC layer. Stops when the descriptor limit is
/// reached or all channels have been scanned
pub struct ScanTask<RadioDriverImpl: DriverConfig> {
    /// Current scheduler state
    pub state: ScanTaskState,
    pub channels: ScanChannelsIter,
    remaining_pan_descriptors: usize,
    /// End time of the current channel's scan window.
    channel_window_end: Option<NsInstant>,
    marker: PhantomData<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> SchedulerTask<RadioDriverImpl> for ScanTask<RadioDriverImpl> {
    /// Process an event by delegating to the active scheduler.
    ///
    /// If the active scheduler completes with a switch request, this method
    /// handles creating the new scheduler and transitioning to it.
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match mem::replace(&mut self.state, ScanTaskState::Placeholder) {
            ScanTaskState::Initial => self.execute_initial(event, context),
            ScanTaskState::ScanningChannel(channel) => {
                self.execute_scanning_channel(event, channel, context)
            }
            ScanTaskState::ReceivingFrame(channel) => {
                self.execute_receiving_frame(event, channel, context)
            }
            _ => unreachable!(),
        }
    }
}

impl<RadioDriverImpl: DriverConfig> ScanTask<RadioDriverImpl> {
    pub fn new(channels: ScanChannels, max_pan_descriptors: usize) -> Self {
        Self {
            state: ScanTaskState::Initial,
            channels: channels.iter(),
            marker: PhantomData,
            remaining_pan_descriptors: max_pan_descriptors,
            channel_window_end: None,
        }
    }

    fn execute_initial(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        debug_assert!(matches!(event, SchedulerTaskEvent::Entry));

        let rx_frame = context.allocate_frame();
        self.advance_to_next_channel(context, Err(rx_frame))
    }

    fn execute_scanning_channel(
        &mut self,
        event: SchedulerTaskEvent,
        channel: Channel,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(event) => match event {
                DrvSvcEvent::FrameStarted => {
                    self.state = ScanTaskState::ReceivingFrame(channel);
                    SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
                }
                DrvSvcEvent::RxWindowEnded(recovered_rx_frame) => {
                    self.continue_on_channel(channel, context, Err(recovered_rx_frame))
                }
                _ => {
                    dot15d4_util::warn!("scan: unexpected driver event while scanning; advancing channel.");
                    self.advance_to_next_channel(context, Err(context.allocate_frame()))
                }
            },
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.state = ScanTaskState::ScanningChannel(channel);
                self.on_scheduler_request(token, request, context)
            }
            _ => {
                dot15d4_util::warn!("scan: unexpected event while scanning a channel; advancing channel.");
                self.advance_to_next_channel(context, Err(context.allocate_frame()))
            }
        }
    }

    fn on_scheduler_request(
        &mut self,
        token: ResponseToken,
        request: SchedulerRequest,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let response = match request {
            SchedulerRequest::Command(SchedulerCommand::PibCommand(cmd)) => {
                context.process_pib_command(cmd)
            }
            SchedulerRequest::Transmission(mpdu) => SchedulerResponse::Transmission(
                SchedulerTransmissionResult::ChannelAccessFailure(
                    mpdu.into_radio_frame::<RadioDriverImpl>(),
                ),
            ),
            _ => {
                dot15d4_util::warn!("scan: unsupported request while scanning; ignoring.");
                return SchedulerTaskTransition::Execute(
                    SchedulerAction::SelectDriverEventOrRequest,
                    None,
                );
            }
        };

        SchedulerTaskTransition::Execute(
            SchedulerAction::SelectDriverEventOrRequest,
            Some((token, response)),
        )
    }

    fn execute_receiving_frame(
        &mut self,
        event: SchedulerTaskEvent,
        channel: Channel,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(event) => match event {
                DrvSvcEvent::Received(radio_frame, instant) => {
                    match self.validate_rx_frame(radio_frame) {
                        Ok(beacon_frame) => {
                            self.handle_valid_beacon(beacon_frame, instant, channel, context)
                        }
                        Err(recovered_rx_frame) => {
                            // Not a beacon, continue listening on same channel
                            self.continue_on_channel(channel, context, Err(recovered_rx_frame))
                        }
                    }
                }
                DrvSvcEvent::CrcError(recovered_rx_frame, _) => {
                    // CRC error, continue listening on same channel
                    self.continue_on_channel(channel, context, Err(recovered_rx_frame))
                }
                DrvSvcEvent::RxWindowEnded(recovered_rx_frame) => {
                    self.continue_on_channel(channel, context, Err(recovered_rx_frame))
                }
                _ => {
                    dot15d4_util::warn!(
                        "scan: unexpected driver event while receiving a frame; advancing channel."
                    );
                    self.advance_to_next_channel(context, Err(context.allocate_frame()))
                }
            },
            _ => {
                dot15d4_util::warn!("scan: unexpected event while receiving a frame; advancing channel.");
                self.advance_to_next_channel(context, Err(context.allocate_frame()))
            }
        }
    }

    fn validate_rx_frame(
        &mut self,
        radio_frame: RadioFrame<RadioFrameSized>,
    ) -> Result<RadioFrame<RadioFrameSized>, RadioFrame<RadioFrameUnsized>> {
        let mpdu = MpduFrame::from_radio_frame(radio_frame);
        let reader = mpdu.reader().parse_addressing();
        if let Ok(reader) = reader {
            let fc = reader.frame_control();
            // matching on Enhanced beacon only
            if matches!(fc.frame_type(), FrameType::Beacon)
                && matches!(fc.frame_version(), frame::FrameVersion::Ieee802154)
            {
                return Ok(mpdu.into_radio_frame::<RadioDriverImpl>());
            }
        }
        Err(mpdu
            .into_radio_frame::<RadioDriverImpl>()
            .forget_size::<RadioDriverImpl>())
    }

    /// Handle a valid beacon frame: forward to MAC if possible, then either
    /// continue listening or finish the scan.
    fn handle_valid_beacon(
        &mut self,
        beacon_frame: RadioFrame<RadioFrameSized>,
        instant: NsInstant,
        channel: Channel,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        self.remaining_pan_descriptors -= 1;
        // Forward the beacon to the MAC.
        let reception_result = SchedulerReceptionResult::Beacon(beacon_frame, instant);
        if self.remaining_pan_descriptors > 0 {
            self.continue_on_channel(channel, context, Ok(reception_result))
        } else {
            // Limit already reached , respond StoppedScanning.
            self.stopped_scanning_response(context, Ok(reception_result))
        }
    }

    fn rx_request_transition(
        &mut self,
        reception_result: Result<SchedulerReceptionResult, RadioFrame<RadioFrameUnsized>>,
        channel: Channel,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        let (radio_frame, response) = match reception_result {
            Ok(reception_result) => {
                match context.try_receive_rx_request(ReceptionType::Beacon) {
                    Some((response_token, _)) => (
                        context.allocate_frame(),
                        Some((
                            response_token,
                            SchedulerResponse::Reception(reception_result),
                        )),
                    ),
                    None => {
                        let recovered_rx_frame = match reception_result {
                            SchedulerReceptionResult::Beacon(radio_frame, _)
                            | SchedulerReceptionResult::Data(radio_frame, _)
                            | SchedulerReceptionResult::Command(radio_frame, _) => {
                                radio_frame.forget_size::<RadioDriverImpl>()
                            }
                        };
                        self.remaining_pan_descriptors += 1;
                        dot15d4_util::warn!(
                            "scan: beacon received before the MAC posted its request; \
                             not counted."
                        );
                        (recovered_rx_frame, None)
                    }
                }
            }
            Err(recovered_rx_frame) => (recovered_rx_frame, None),
        };

        let request = DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
            start: Timestamp::BestEffort,
            radio_frame,
            channel: Some(channel),
            rx_window_end: self.channel_window_end,
            expected_rx_framestart: None,
            ack_delay: <PhyOf<RadioDriverImpl> as PhyConfig>::AIFS,
        });

        SchedulerTaskTransition::Execute(
            SchedulerAction::SendDriverRequestThenSelect(request),
            response,
        )
    }

    /// Continue listening on the same channel for the remainder of the scan
    /// window. If the window has already expired, advance to the next channel.
    /// Before restarting RX, check whether the scan should stop.
    fn continue_on_channel(
        &mut self,
        // radio_frame: RadioFrame<RadioFrameUnsized>,
        channel: Channel,
        context: &mut SchedulerContext<RadioDriverImpl>,
        reception_result: Result<SchedulerReceptionResult, RadioFrame<RadioFrameUnsized>>,
    ) -> SchedulerTaskTransition {
        // If the limit has been reached and the MAC posted its follow-up
        // request, respond StoppedScanning immediately.
        if self.remaining_pan_descriptors == 0 {
            return self.stopped_scanning_response(context, reception_result);
        }

        if let Some(window_end) = self.channel_window_end {
            let now = context.timer.now();
            if now < window_end {
                self.state = ScanTaskState::ScanningChannel(channel);
                self.rx_request_transition(reception_result, channel, context)
            } else {
                // TODO: check
                self.advance_to_next_channel(context, reception_result)
            }
        } else {
            // Window expired , advance to next channel
            self.advance_to_next_channel(context, reception_result)
        }
    }

    /// Advance to the next channel in the scan list. If no more channels
    /// remain (or the descriptor limit is reached), finish the scan.
    fn advance_to_next_channel(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        reception_result: Result<SchedulerReceptionResult, RadioFrame<RadioFrameUnsized>>,
    ) -> SchedulerTaskTransition {
        // If the limit has been reached and the MAC posted its follow-up
        // request, respond StoppedScanning immediately.
        if self.remaining_pan_descriptors == 0 {
            self.stopped_scanning_response(context, reception_result)
        } else if let Some(next_channel) = self.channels.next() {
            self.state = ScanTaskState::ScanningChannel(next_channel);
            let current_time = context.timer.now();
            let window_end = current_time + NsDuration::secs(SCAN_DURATION);
            self.channel_window_end = Some(window_end);
            self.rx_request_transition(reception_result, next_channel, context)
        } else {
            self.finish_scan(context, reception_result)
        }
    }

    fn stopped_scanning_response(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        reception_result: Result<SchedulerReceptionResult, RadioFrame<RadioFrameUnsized>>,
    ) -> SchedulerTaskTransition {
        if let Some((response_token, _)) = context.try_receive_rx_request(ReceptionType::Beacon) {
            match reception_result {
                Ok(reception_result) => {
                    let response = SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                        ScanCommandResult::StoppedScanning(Some(reception_result)),
                    ));
                    return SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((response_token, response)),
                    );
                }
                Err(recovered_rx_frame) => unsafe {
                    context
                        .buffer_allocator
                        .deallocate_buffer(recovered_rx_frame.into_buffer());

                    let response = SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                        ScanCommandResult::StoppedScanning(None),
                    ));
                    return SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((response_token, response)),
                    );
                },
            }
        }
        unreachable!()
    }

    /// Complete the scan: deallocate any leftover frame buffer, check for a
    /// pending beacon Rx request from the MAC and respond with
    /// StoppedScanning, then switch back to CSMA.
    fn finish_scan(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        reception_result: Result<SchedulerReceptionResult, RadioFrame<RadioFrameUnsized>>,
    ) -> SchedulerTaskTransition {
        if let Some((response_token, _)) = context.try_receive_rx_request(ReceptionType::Beacon) {
            match reception_result {
                Ok(reception_result) => {
                    let response = SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                        ScanCommandResult::StoppedScanning(Some(reception_result)),
                    ));
                    return SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((response_token, response)),
                    );
                }
                Err(recovered_rx_frame) => unsafe {
                    context
                        .buffer_allocator
                        .deallocate_buffer(recovered_rx_frame.into_buffer());

                    let response = SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                        ScanCommandResult::StoppedScanning(None),
                    ));
                    return SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((response_token, response)),
                    );
                },
            }
        }
        unreachable!()
    }
}
