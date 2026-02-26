//! Scheduler Service Module
//!
//! This module provides the scheduler layer that coordinates access to the
//! radio medium.

#![allow(dead_code)]

pub mod command;
pub mod csma;
mod runner;
#[cfg(feature = "tsch")]
pub mod scan;
pub mod task;
#[cfg(test)]
pub mod tests;
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
use dot15d4_util::sync::{Channel, HasAddress, Receiver, ResponseToken, Sender};
use rand_core::RngCore;

use crate::driver::{DriverEventReceiver, DriverRequestSender, DrvSvcEvent, DrvSvcRequest};
use crate::mac::MacBufferAllocator;
use crate::pib::Pib;

pub use self::command::{SchedulerCommand, SchedulerCommandResult};

#[cfg(feature = "tsch")]
use self::scan::ScanChannels;
use self::{runner::run_task, task::RootSchedulerTask};

pub const SCHEDULER_CHANNEL_CAPACITY: usize = 6;
pub const SCHEDULER_CHANNEL_BACKLOG: usize = 6;

/// Message types for routing scheduler requests.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum MessageType {
    Tx,
    Rx(ReceptionType),
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
    Reception(ReceptionType),
    Command(SchedulerCommand),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ReceptionType {
    Data,
    Beacon,
    MacCommand(MacCommandType),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum MacCommandType {
    AssociateRequest,
    AssociateResponse,
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

pub enum SchedulerReceptionResult {
    Data(
        RadioFrame<RadioFrameSized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
    Beacon(
        RadioFrame<RadioFrameSized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
    Command(
        RadioFrame<RadioFrameSized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
}

pub enum SchedulerResponse {
    Transmission(SchedulerTransmissionResult),
    Reception(SchedulerReceptionResult),
    Command(SchedulerCommandResult),
}

impl HasAddress<MessageType> for SchedulerRequest {
    fn matches(&self, address: &MessageType) -> bool {
        match self {
            SchedulerRequest::Transmission(_) => {
                matches!(*address, MessageType::TxOrCommand) || matches!(*address, MessageType::Tx)
            }
            SchedulerRequest::Reception(reception) => match reception {
                ReceptionType::Data => matches!(*address, MessageType::Rx(ReceptionType::Data)),
                ReceptionType::Beacon => matches!(*address, MessageType::Rx(ReceptionType::Beacon)),
                ReceptionType::MacCommand(_) => {
                    matches!(*address, MessageType::Rx(ReceptionType::MacCommand(..)))
                }
            },
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

    pub(crate) fn try_receive_rx_request(
        &self,
        reception_type: ReceptionType,
    ) -> Option<(dot15d4_util::sync::ResponseToken, SchedulerRequest)> {
        let address = match reception_type {
            ReceptionType::Data => MessageType::Rx(ReceptionType::Data),
            ReceptionType::Beacon => MessageType::Rx(ReceptionType::Beacon),
            ReceptionType::MacCommand(command) => {
                MessageType::Rx(ReceptionType::MacCommand(command))
            }
        };
        self.request_receiver.try_receive_request(&address)
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

        let mut task = RootSchedulerTask::new(PhyChannel::_26, &mut self.context);
        run_task(&mut task, &mut self.context, &mut consumer_token).await
    }
}

/// Trait for scheduler tasks.
///
/// Tasks are pure state machines that receive events and produce transitions.
pub trait SchedulerTask<RadioDriverImpl: DriverConfig> {
    /// Process an event and return the next transition.
    fn step(
        &mut self,
        event: SchedulerTaskEvent,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> SchedulerTaskTransition;
}

/// Events that can be delivered to a scheduler task.
pub enum SchedulerTaskEvent {
    /// Task is being entered (initial entry).
    Entry,
    /// A driver event was received.
    DriverEvent(DrvSvcEvent),
    /// A scheduler request was received from MAC layer.
    SchedulerRequest {
        token: ResponseToken,
        request: SchedulerRequest,
    },
    /// Timer expired (e.g. for TSCH slot timing).
    #[cfg(feature = "tsch")]
    TimerExpired,
}

/// Actions provided in task transitions to direct the execution of the
/// task's state machine.
pub enum SchedulerAction {
    /// Send driver request, then immediately wait for driver event.
    /// Optimization to avoid extra round-trip through logic.
    SendDriverRequestThenWait(DrvSvcRequest),
    /// Wait for an event from the driver service.
    WaitForDriverEvent,
    /// Wait for a request from the MAC layer (scheduler channel).
    WaitForSchedulerRequest,
    /// Select: wait for driver event OR scheduler request.
    /// Used in CSMA when waiting for frames but may receive TX request.
    SelectDriverEventOrRequest,
    /// Select: wait for timer expiry OR scheduler request.
    /// Used in TSCH to wake up for next slot or receive new requests.
    #[cfg(feature = "tsch")]
    WaitForTimeoutOrSchedulerRequest { deadline: NsInstant },
    /// Send driver request, then select: wait for driver event OR scheduler request.
    /// Used when starting RX - we want to receive frames but also handle TX requests.
    SendDriverRequestThenSelect(DrvSvcRequest),
}

/// Transitions returned by scheduler tasks.
pub enum SchedulerTaskTransition {
    /// Execute an action and optionally send a response.
    Execute(
        /// The action for the runner to execute.
        SchedulerAction,
        /// An optional response to send immediately.
        Option<(ResponseToken, SchedulerResponse)>,
    ),

    /// Task cycle completed
    Completed(
        SchedulerTaskCompletion,
        Option<(ResponseToken, SchedulerResponse)>,
    ),
}

/// Result from a scheduler task that completed
#[derive(Debug, Clone, Copy)]
pub enum SchedulerTaskCompletion {
    SwitchToCsma,
    #[cfg(feature = "tsch")]
    SwitchToTsch,
    #[cfg(feature = "tsch")]
    SwitchToScanning(ScanChannels, usize),
}
