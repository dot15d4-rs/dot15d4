use core::{marker::PhantomData, mem};

use dot15d4_driver::radio::{
    config::Channel,
    frame::{self, FrameType, RadioFrame, RadioFrameSized, RadioFrameUnsized},
    DriverConfig,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};

use crate::{
    driver::{DrvSvcEvent, DrvSvcRequest, DrvSvcTaskRx, Timestamp},
    scheduler::{
        command::scan::ScanCommand, ReceptionType, SchedulerCommand, SchedulerCommandResult,
        SchedulerReceptionResult, SchedulerRequest, SchedulerResponse,
    },
};

use super::{SchedulerAction, SchedulerTaskCompletion};
use super::{SchedulerContext, SchedulerTask, SchedulerTaskEvent, SchedulerTaskTransition};

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

/// Root scheduler state machine states.
pub enum ScanTaskState {
    Initial,
    ScanningChannel(Channel),
    ReceivingFrame(Channel),
    Terminating(ResponseToken),
    /// Placeholder
    Placeholder,
}

/// Root scheduler task that manages scheduler switching.
///
/// This is the top-level task that wraps the actual scheduler implementations.
/// It handles mode switching requests and ensures proper initialization of
/// new schedulers when switching.
pub struct ScanTask<RadioDriverImpl: DriverConfig> {
    /// Current scheduler state
    pub state: ScanTaskState,
    pub channels: ScanChannelsIter,
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
        // Delegate to inner task
        match mem::replace(&mut self.state, ScanTaskState::Placeholder) {
            ScanTaskState::Initial => self.execute_initial(event, context),
            ScanTaskState::ScanningChannel(channel) => {
                self.execute_scanning_channel(event, channel)
            }
            ScanTaskState::ReceivingFrame(channel) => {
                self.execute_receiving_frame(event, channel, context)
            }
            ScanTaskState::Terminating(response_token) => {
                self.execute_terminating(event, response_token, context)
            }
            _ => unreachable!(),
        }
    }
}

impl<RadioDriverImpl: DriverConfig> ScanTask<RadioDriverImpl> {
    pub fn new(channels: ScanChannels) -> Self {
        Self {
            state: ScanTaskState::Initial,
            channels: channels.iter(),
            marker: PhantomData,
        }
    }

    fn execute_initial(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        debug_assert!(matches!(event, SchedulerTaskEvent::Entry));

        let radio_frame = context.allocate_frame();
        if let Some(next_channel) = self.channels.next() {
            let action = self.scan_action(radio_frame, next_channel);
            self.state = ScanTaskState::ScanningChannel(next_channel);
            SchedulerTaskTransition::Execute(action, None)
        } else {
            // Should not happend since we expect at least 1 channel to scan
            SchedulerTaskTransition::Completed(SchedulerTaskCompletion::SwitchToCsma, None)
        }
    }

    fn execute_scanning_channel(
        &mut self,
        event: SchedulerTaskEvent,
        channel: Channel,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(event) => match event {
                DrvSvcEvent::FrameStarted => {
                    self.state = ScanTaskState::ReceivingFrame(channel);
                    SchedulerTaskTransition::Execute(SchedulerAction::WaitForDriverEvent, None)
                }
                DrvSvcEvent::RxWindowEnded(_radio_frame) => todo!(),
                _ => unreachable!(),
            },
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.received_request(request, token)
            }
            _ => unreachable!(),
        }
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
                            if let Some((response_token, _)) =
                                context.try_receive_rx_request(ReceptionType::Beacon)
                            {
                                let response = SchedulerResponse::Reception(
                                    SchedulerReceptionResult::Beacon(beacon_frame, instant),
                                );
                                if let Some(next_channel) = self.channels.next() {
                                    self.state = ScanTaskState::ScanningChannel(next_channel);
                                    let radio_frame = context.allocate_frame();
                                    let action = self.scan_action(radio_frame, next_channel);
                                    SchedulerTaskTransition::Execute(
                                        action,
                                        Some((response_token, response)),
                                    )
                                } else {
                                    // TODO: should complete if no more channels.
                                    self.state = ScanTaskState::ScanningChannel(channel);
                                    let radio_frame = context.allocate_frame();
                                    let action = self.scan_action(radio_frame, channel);
                                    SchedulerTaskTransition::Execute(
                                        action,
                                        Some((response_token, response)),
                                    )
                                }
                            } else {
                                let action = self.scan_action(
                                    beacon_frame.forget_size::<RadioDriverImpl>(),
                                    channel,
                                );
                                SchedulerTaskTransition::Execute(action, None)
                            }
                        }
                        Err(recoved_rx_frame) => {
                            self.state = ScanTaskState::ScanningChannel(channel);
                            let action = self.scan_action(recoved_rx_frame, channel);
                            SchedulerTaskTransition::Execute(action, None)
                        }
                    }
                }
                // TODO: support rx window instead of waiting indefinitely
                DrvSvcEvent::RxWindowEnded(_radio_frame) => todo!(),
                _ => unreachable!(),
            },
            SchedulerTaskEvent::SchedulerRequest { token, request } => {
                self.received_request(request, token)
            }
            _ => unreachable!(),
        }
    }

    fn execute_terminating(
        &mut self,
        event: SchedulerTaskEvent,
        response_token: ResponseToken,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition {
        match event {
            SchedulerTaskEvent::DriverEvent(event) => match event {
                DrvSvcEvent::Received(radio_frame, instant) => todo!(),
                DrvSvcEvent::RxWindowEnded(radio_frame) => {
                    unsafe {
                        context
                            .buffer_allocator
                            .deallocate_buffer(radio_frame.into_buffer());
                    }
                    let resp = SchedulerResponse::Command(SchedulerCommandResult::ScanCommand(
                        crate::scheduler::command::scan::ScanCommandResult::StoppedScanning,
                    ));
                    SchedulerTaskTransition::Completed(
                        SchedulerTaskCompletion::SwitchToCsma,
                        Some((response_token, resp)),
                    )
                }
                _ => unreachable!(),
            },
            _ => unreachable!(),
        }
    }

    fn received_request(
        &mut self,
        request: SchedulerRequest,
        response_token: ResponseToken,
    ) -> SchedulerTaskTransition {
        match request {
            SchedulerRequest::Command(SchedulerCommand::ScanCommand(ScanCommand::StopScanning)) => {
                self.state = ScanTaskState::Terminating(response_token);
                SchedulerTaskTransition::Execute(
                    SchedulerAction::SendDriverRequestThenWait(DrvSvcRequest::CompleteThenGoIdle),
                    None,
                )
            }
            _ => unreachable!(),
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

    fn scan_action(
        &self,
        radio_frame: RadioFrame<RadioFrameUnsized>,
        channel: Channel,
    ) -> SchedulerAction {
        let request = DrvSvcRequest::CompleteThenStartRx(DrvSvcTaskRx {
            start: Timestamp::BestEffort,
            radio_frame,
            channel: Some(channel),
            rx_window: None,
        });
        SchedulerAction::SendDriverRequestThenSelect(request)
    }
}
