#![allow(dead_code)]

pub mod command;
mod csma;
#[cfg(feature = "tsch")]
pub mod tsch;

use core::cell::Cell;

use dot15d4_driver::radio::frame::{
    RadioFrame, RadioFrameRepr, RadioFrameSized, RadioFrameUnsized,
};
use dot15d4_driver::radio::DriverConfig;
use dot15d4_driver::timer::NsInstant;
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::{Channel, HasAddress, Receiver, Sender};

use crate::driver::{DriverEventReceiver, DriverRequestSender};
use crate::mac::MacBufferAllocator;

pub use self::command::{SchedulerCommand, SchedulerCommandResult};
use self::tsch::beacon::EnhancedBeaconBuilder;

pub const SCHEDULER_CHANNEL_CAPACITY: usize = 5;
pub const SCHEDULER_CHANNEL_BACKLOG: usize = 5;

/// To ensure progress, we give precedence of outbound tasks over inbound tasks.
/// We therefore route these two classes of tasks into separate virtual
/// channels.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TaskDirection {
    Outbound,
    Inbound,
    Any,
}

pub type SchedulerRequestChannel = Channel<
    TaskDirection,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;
pub type SchedulerRequestReceiver<'channel> = Receiver<
    'channel,
    TaskDirection,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;
pub type SchedulerRequestSender<'channel> = Sender<
    'channel,
    TaskDirection,
    SchedulerRequest,
    SchedulerResponse,
    SCHEDULER_CHANNEL_CAPACITY,
    SCHEDULER_CHANNEL_BACKLOG,
    1,
>;

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

impl HasAddress<TaskDirection> for SchedulerRequest {
    fn matches(&self, address: &TaskDirection) -> bool {
        if matches!(*address, TaskDirection::Any) {
            return true;
        }

        match self {
            SchedulerRequest::Transmission(_) => matches!(*address, TaskDirection::Outbound),
            SchedulerRequest::Reception => matches!(*address, TaskDirection::Inbound),
            SchedulerRequest::Command(_) => matches!(*address, TaskDirection::Outbound),
        }
    }
}

pub(crate) enum SchedulerState {
    UsingCsmaCa,
    #[cfg(feature = "tsch")]
    UsingTsch,
}

pub struct SchedulerService<'svc, RadioDriverImpl: DriverConfig> {
    state: Cell<Option<SchedulerState>>,
    timer: RadioDriverImpl::Timer,
    request_receiver: SchedulerRequestReceiver<'svc>,
    driver_request_sender: DriverRequestSender<'svc>,
    driver_event_receiver: DriverEventReceiver<'svc>,
    // Pre-allocated frame for inbound frame
    rx_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,
    buffer_allocator: MacBufferAllocator,
    // TODO: feature tsch-coordinator
    beacon_frame: Cell<Option<RadioFrame<RadioFrameSized>>>,
    beacon_builder: EnhancedBeaconBuilder<'static, RadioDriverImpl>,
}

impl<'svc, RadioDriverImpl: DriverConfig> SchedulerService<'svc, RadioDriverImpl> {
    pub fn new(
        timer: RadioDriverImpl::Timer,
        request_receiver: SchedulerRequestReceiver<'svc>,
        driver_request_sender: DriverRequestSender<'svc>,
        driver_response_receiver: DriverEventReceiver<'svc>,
        buffer_allocator: MacBufferAllocator,
    ) -> Self {
        Self {
            state: Cell::new(Some(SchedulerState::UsingCsmaCa)),
            timer,
            request_receiver,
            driver_request_sender,
            driver_event_receiver: driver_response_receiver,
            rx_frame: Cell::new(Some(Self::allocate_frame(buffer_allocator))),
            beacon_frame: Cell::new(None),
            buffer_allocator,
            beacon_builder: EnhancedBeaconBuilder::new(),
        }
    }

    /// Pre-allocates a re-usable frame.
    fn allocate_frame(buffer_allocator: MacBufferAllocator) -> RadioFrame<RadioFrameUnsized> {
        let inbound_frame_buffer_size = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new()
            .max_buffer_length() as usize;
        RadioFrame::new::<RadioDriverImpl>(
            buffer_allocator
                .try_allocate_buffer(inbound_frame_buffer_size)
                .expect("no capacity"),
        )
    }

    pub async fn run(&mut self) -> ! {
        let mut mode = self.state.take().unwrap();

        let mut consumer_token = self
            .request_receiver
            .try_allocate_consumer_token()
            .expect("no capacity");

        loop {
            (mode, consumer_token) = match mode {
                SchedulerState::UsingCsmaCa => self.run_csma(consumer_token).await,
                SchedulerState::UsingTsch => self.run_tsch(consumer_token).await,
            }
        }
    }
}
