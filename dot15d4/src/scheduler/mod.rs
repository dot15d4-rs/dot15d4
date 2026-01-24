//! Scheduler Service Module
//!
//! This module provides the scheduler layer that coordinates access to the
//! radio medium.

#![allow(dead_code)]

pub mod action;
pub mod command;
pub mod csma;
mod runner;
pub mod state;
pub mod task;
#[cfg(feature = "tsch")]
pub mod tsch;

use dot15d4_driver::{
    radio::{
        config::Channel as PhyChannel,
        frame::{RadioFrame, RadioFrameRepr, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::NsInstant,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::{Channel, HasAddress, Receiver, Sender};
use rand_core::RngCore;

use crate::driver::{DriverEventReceiver, DriverRequestSender};
use crate::mac::MacBufferAllocator;
use crate::pib::Pib;

pub use self::command::{SchedulerCommand, SchedulerCommandResult};

use self::runner::run_task;
pub use self::state::{ActiveScheduler, RootSchedulerTask};

pub const SCHEDULER_CHANNEL_CAPACITY: usize = 5;
pub const SCHEDULER_CHANNEL_BACKLOG: usize = 5;

/// Message types for routing scheduler requests.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum MessageType {
    Tx,
    Rx,
    Command,
    TxOrCommand,
}

pub type SchedulerRequestChannel = Channel<
    MessageType,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;
pub type SchedulerRequestReceiver<'channel> = Receiver<
    'channel,
    MessageType,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;
pub type SchedulerRequestSender<'channel> = Sender<
    'channel,
    MessageType,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;

/// Request to the scheduler service.
pub enum SchedulerRequest {
    Transmission(MpduFrame),
    Reception,
    Command(SchedulerCommand),
}

pub enum SchedulerTransmissionResult {
    Sent(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameUnsized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
    ChannelAccessFailure(
        /// unsent radio frame
        RadioFrame<RadioFrameSized>,
    ),
    NoAck(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameSized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
}

pub enum SchedulerResponse {
    Transmission(SchedulerTransmissionResult),
    Reception(RadioFrame<RadioFrameSized>, NsInstant),
    Command(SchedulerCommandResult),
}

impl HasAddress<MessageType> for SchedulerRequest {
    fn matches(&self, address: &MessageType) -> bool {
        match self {
            SchedulerRequest::Transmission(_) => {
                matches!(*address, MessageType::TxOrCommand) || matches!(*address, MessageType::Tx)
            }
            SchedulerRequest::Reception => matches!(*address, MessageType::Rx),
            SchedulerRequest::Command(_) => {
                matches!(*address, MessageType::TxOrCommand)
                    || matches!(*address, MessageType::Command)
            }
        }
    }
}

pub struct SchedulerContext<'svc, RadioDriverImpl: DriverConfig> {
    /// PAN Information Base.
    pub pib: Pib,
    /// Buffer allocator reference.
    pub buffer_allocator: MacBufferAllocator,
    /// Random number generator for backoff calculation.
    pub rng: &'svc mut dyn RngCore,
    /// Timer
    timer: RadioDriverImpl::Timer,
    /// Scheduler Request Receiver
    request_receiver: SchedulerRequestReceiver<'svc>,
    /// Scheduler Request Sender
    driver_request_sender: DriverRequestSender<'svc>,
    /// Driver Event Receiver
    driver_event_receiver: DriverEventReceiver<'svc>,
}

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerContext<'svc, RadioDriverImpl> {
    pub fn new(
        buffer_allocator: MacBufferAllocator,
        rng: &'svc mut dyn RngCore,
        timer: RadioDriverImpl::Timer,
        address: &[u8; 8],
        request_receiver: SchedulerRequestReceiver<'svc>,
        driver_request_sender: DriverRequestSender<'svc>,
        driver_event_receiver: DriverEventReceiver<'svc>,
    ) -> Self {
        Self {
            pib: Pib::new(address),
            buffer_allocator,
            rng,
            timer,
            request_receiver,
            driver_event_receiver,
            driver_request_sender,
        }
    }

    /// Allocate a new radio frame.
    pub fn allocate_frame(&self) -> RadioFrame<RadioFrameUnsized> {
        let size = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new().max_buffer_length()
            as usize;
        RadioFrame::new::<RadioDriverImpl>(
            self.buffer_allocator
                .try_allocate_buffer(size)
                .expect("no capacity for frame buffer"),
        )
    }

    fn try_receive_tx_request(&self) -> Option<(dot15d4_util::sync::ResponseToken, MpduFrame)> {
        match self.request_receiver.try_receive_request(&MessageType::Tx) {
            Some((token, SchedulerRequest::Transmission(mpdu))) => Some((token, mpdu)),
            _ => None,
        }
    }

    fn try_receive_rx_request(
        &self,
    ) -> Option<(dot15d4_util::sync::ResponseToken, SchedulerRequest)> {
        self.request_receiver.try_receive_request(&MessageType::Rx)
    }
}

pub struct SchedulerService<'svc, RadioDriverImpl: DriverConfig> {
    context: SchedulerContext<'svc, RadioDriverImpl>,
}

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    /// Create a new scheduler service.
    pub fn new(
        timer: RadioDriverImpl::Timer,
        request_receiver: SchedulerRequestReceiver<'svc>,
        driver_request_sender: DriverRequestSender<'svc>,
        driver_event_receiver: DriverEventReceiver<'svc>,
        buffer_allocator: MacBufferAllocator,
        rng: &'svc mut dyn RngCore,
        address: &[u8; 8],
    ) -> Self {
        let context = SchedulerContext::new(
            buffer_allocator,
            rng,
            timer,
            address,
            request_receiver,
            driver_request_sender,
            driver_event_receiver,
        );
        Self { context }
    }

    /// Run the scheduler service.
    pub async fn run(&mut self) -> ! {
        let mut consumer_token = self
            .context
            .request_receiver
            .try_allocate_consumer_token()
            .expect("no capacity for consumer token");

        let mut task = RootSchedulerTask::new(PhyChannel::_12, &mut self.context);
        run_task(&mut task, &mut self.context, &mut consumer_token).await
    }
}
