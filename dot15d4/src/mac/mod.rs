mod mcps;
mod mlme;
mod neighbors;
mod pib;
pub mod primitives;
mod task;
mod tsch;

pub use dot15d4_frame as frame;

use core::cell::RefCell;

use paste::paste;
use rand_core::RngCore;

use crate::{
    driver::{
        constants::PHY_MAX_PACKET_SIZE_127,
        frame::FrameType,
        radio::{DriverConfig, MAX_DRIVER_OVERHEAD},
        DriverRequestSender, DRIVER_CHANNEL_CAPACITY,
    },
    mac::mcps::data::DataRequestResult,
    util::{
        allocator::{BufferAllocator, IntoBuffer},
        sync::{
            channel::{Channel, Receiver, Sender},
            mutex::Mutex,
            select, Either, MatchingResponse, PollingResponseToken, ResponseToken,
        },
    },
};

use self::{
    frame::mpdu::MpduFrame,
    mcps::data::{DataIndication, DataIndicationTask, DataRequestTask},
    pib::Pib,
    primitives::{MacIndication, MacRequest},
    task::*,
};

// TODO: Make allocator and channel capacities and the number of upper layer
//       tasks configurable.

/// The max number of UL Rx tokens that may be handed out in parallel.
const UL_MAX_RX_TOKENS: usize = 1;

/// The max number of UL Tx tokens that may be handed out in parallel.
/// Note: Each Rx token requires an accompanying Tx token to be allocated.
const UL_MAX_TX_TOKENS: usize = 1 + UL_MAX_RX_TOKENS;

/// The number of additional messages that may be pending.
/// Note: 1 is currently the min number supported.
const UL_MSG_BACKLOG: usize = 1;

/// The number of upper layer tasks that receive indications.
///
/// Note: Currently we assume a single data channel terminated by a smoltcp
///       client. But this may change once we expose one or more independent
///       control channels towards applications directly.
const UL_NUM_CLIENTS: usize = 1;

pub type MacRequestChannel = Channel<(), MacRequest, (), UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;
pub type MacRequestReceiver<'channel> =
    Receiver<'channel, (), MacRequest, (), UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;
pub type MacRequestSender<'channel> =
    Sender<'channel, (), MacRequest, (), UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;

pub type MacIndicationChannel =
    Channel<(), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;
pub type MacIndicationReceiver<'channel> =
    Receiver<'channel, (), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;
pub type MacIndicationSender<'channel> =
    Sender<'channel, (), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;

/// The number of MAC indication tasks that must be executing in parallel making
/// use of the driver's pipelining capability.
const MAC_NUM_PARALLEL_INDICATION_TASKS: usize = UL_MAX_RX_TOKENS + 1;
const MAC_NUM_PARALLEL_REQUEST_TASKS: usize = UL_MAX_TX_TOKENS;
const _: () = {
    assert!(
        DRIVER_CHANNEL_CAPACITY
            == MAC_NUM_PARALLEL_INDICATION_TASKS + MAC_NUM_PARALLEL_REQUEST_TASKS,
        "driver channel capacity does not match number of MAC tasks"
    )
};

// TODO: Challenge the following capacity calculation.
/// Buffers are allocated by:
/// - tx token
/// - indication task
/// - driver service (2 pre-allocated buffers for RX/TX ACKs)
///
/// Required buffers:
/// - one buffer per max outstanding upper layer tx token (= max request tasks)
/// - one buffer per indication task
/// - one pre-allocated buffer for outgoing ACKs
/// - one pre-allocated buffer for incoming ACKs
pub const MAC_NUM_REQUIRED_BUFFERS: usize =
    UL_MAX_TX_TOKENS + MAC_NUM_PARALLEL_INDICATION_TASKS + 2;
pub const MAC_BUFFER_SIZE: usize = PHY_MAX_PACKET_SIZE_127 + MAX_DRIVER_OVERHEAD;

pub type MacBufferAllocator = BufferAllocator;

// Local macro: No need for strict macro hygiene.
macro_rules! mac_svc_tasks {
    ($($mac_task:ident),+)=> {
        paste!{
            enum MacSvcTask<'task, RadioDriverImpl: DriverConfig> {
                $($mac_task([<$mac_task Task>]<'task, RadioDriverImpl>)),*
            }

            enum MacSvcTaskResult<'task, RadioDriverImpl: DriverConfig> {
                $($mac_task(<[<$mac_task Task>]<'task, RadioDriverImpl> as MacTask>::Result)),*
            }

            $(mac_svc_tasks!(transition_converter: $mac_task);)*

            impl<'task, RadioDriverImpl: DriverConfig> MacTask for MacSvcTask<'task, RadioDriverImpl> {
                type Result = MacSvcTaskResult<'task, RadioDriverImpl>;

                fn step(self, event: MacTaskEvent) -> MacTaskTransition<Self> {
                    match self {
                        $(MacSvcTask::$mac_task(inner_task) => inner_task.step(event).into()),*
                    }
                }
            }

        }
    };

    (transition_converter: $mac_task:ident) => {
        paste!{
            impl<'task, RadioDriverImpl: DriverConfig> From<MacTaskTransition<[<$mac_task Task>]<'task, RadioDriverImpl>>> for MacTaskTransition<MacSvcTask<'task, RadioDriverImpl>> {
                fn from(value: MacTaskTransition<[<$mac_task Task>]<'task, RadioDriverImpl>>) -> Self {
                    match value {
                        MacTaskTransition::DrvSvcRequest(updated_task, driver_request, task_result) => {
                            let updated_task = MacSvcTask::$mac_task(updated_task);
                            let task_result = task_result.map(|task_result| MacSvcTaskResult::$mac_task(task_result)) ;
                            MacTaskTransition::DrvSvcRequest(updated_task, driver_request, task_result)
                        },
                        MacTaskTransition::Terminated(task_result) => {
                            let task_result = MacSvcTaskResult::$mac_task(task_result);
                            MacTaskTransition::Terminated(task_result.into())
                        },
                    }
                }
            }
        }
    }
}

mac_svc_tasks!(DataRequest, DataIndication);

#[allow(dead_code)]
/// A structure exposing MAC sublayer services such as MLME and MCPS. This runs
/// the main event loop that handles interactions between an upper layer and the
/// PHY sublayer. It uses channels to communicate with upper layer tasks and
/// with radio drivers.
pub struct MacService<'svc, Rng: RngCore, RadioDriverImpl: DriverConfig> {
    /// Timer instance to wait until driver requests become pending.
    // TODO: remove allow attribute once used in code
    #[allow(dead_code)]
    timer: RadioDriverImpl::Timer,
    /// Pseudo-random number generator
    rng: &'svc mut Mutex<Rng>,
    /// Message buffer allocator
    buffer_allocator: MacBufferAllocator,
    /// Upper layer channel from which MAC requests are received.
    request_receiver: MacRequestReceiver<'svc>,
    /// Upper layer channel to which MAC indications are sent.
    indication_sender: MacIndicationSender<'svc>,
    /// Channel to communicate with one or several radio drivers.
    driver_request_sender: DriverRequestSender<'svc>,
    /// PAN Information Base
    pib: RefCell<Pib>,
}

impl<'svc, Rng: RngCore, RadioDriverImpl: DriverConfig> MacService<'svc, Rng, RadioDriverImpl> {
    /// Creates a new [`MacService<Rng, U, Timer, R>`].
    pub fn new(
        timer: RadioDriverImpl::Timer,
        rng: &'svc mut Mutex<Rng>,
        buffer_allocator: MacBufferAllocator,
        request_receiver: MacRequestReceiver<'svc>,
        indication_sender: MacIndicationSender<'svc>,
        driver_request_sender: DriverRequestSender<'svc>,
    ) -> Self {
        Self {
            timer,
            rng,
            buffer_allocator,
            request_receiver,
            indication_sender,
            driver_request_sender,
            pib: RefCell::new(Pib::default()),
        }
    }

    /// Run the main event loop used by the MAC sublayer for its operation.
    ///
    /// The loop waits until receiving a MCPS-DATA request from the upper layer.
    /// It will then instantiate the corresponding state machine and start
    /// driving it. The state machine will produce driver service requests which
    /// will be passed on to the driver service. Whenever the driver service
    /// returns a response it will be used to drive the corresponding state
    /// machine.
    pub async fn run(&mut self) -> ! {
        // MAC request tasks are indexed by the message slots of the
        // corresponding MAC requests (0..UL_NUM_PARALLEL_REQUESTS).
        //
        // MAC indication tasks use the higher indices
        // (UL_NUM_PARALLEL_REQUESTS..UL_NUM_PARALLEL_REQUESTS +
        // MAC_NUM_PARALLEL_INDICATIONS).
        //
        // We need an additional indication background tasks so that we can
        // efficiently use the driver service's pipelining capability.
        let mut mac_svc_tasks: [Option<MacSvcTask<RadioDriverImpl>>;
            MAC_NUM_PARALLEL_REQUEST_TASKS + MAC_NUM_PARALLEL_INDICATION_TASKS] =
            [const { None }; MAC_NUM_PARALLEL_REQUEST_TASKS + MAC_NUM_PARALLEL_INDICATION_TASKS];

        // Outstanding driver requests will be pushed to this vector and polled
        // for responses.
        let mut outstanding_driver_requests: heapless::Vec<
            PollingResponseToken,
            DRIVER_CHANNEL_CAPACITY,
        > = heapless::Vec::new();

        // A driver-to-MAC message index: The index corresponds to the driver
        // message slot, the content to the corresponding MAC request slot.
        let mut driver_msg_slot_to_task_index: [usize; DRIVER_CHANNEL_CAPACITY] =
            [0; DRIVER_CHANNEL_CAPACITY];

        // Response tokens for outstanding MAC requests.
        let mut outstanding_mac_requests: [Option<ResponseToken>; MAC_NUM_PARALLEL_REQUEST_TASKS] =
            [const { None }; MAC_NUM_PARALLEL_REQUEST_TASKS];

        let first_mac_indication_task_index =
            mac_svc_tasks.len() - MAC_NUM_PARALLEL_INDICATION_TASKS;
        self.create_indication_tasks(
            first_mac_indication_task_index,
            &mut mac_svc_tasks,
            &mut driver_msg_slot_to_task_index,
            &mut outstanding_driver_requests,
        );

        let mut consumer_token = self
            .request_receiver
            .try_allocate_consumer_token()
            .expect("no capacity");

        loop {
            match select(
                self.request_receiver
                    .wait_for_request(&mut consumer_token, &()),
                self.driver_request_sender
                    .wait_for_response(&mut outstanding_driver_requests),
            )
            .await
            {
                // Upper layer: A MAC request was received. Create the corresponding task and kick it off.
                Either::First((mac_request_response_token, mac_request)) => {
                    let mac_request_task_index = mac_request_response_token.message_slot() as usize;
                    outstanding_mac_requests[mac_request_task_index] =
                        Some(mac_request_response_token);
                    let mac_request_task = self.create_request_task(mac_request);
                    self.step_task(
                        &mut mac_svc_tasks,
                        &mut driver_msg_slot_to_task_index,
                        &mut outstanding_driver_requests,
                        Some(&mut outstanding_mac_requests),
                        mac_request_task_index,
                        mac_request_task,
                        MacTaskEvent::Entry,
                    );
                }
                // Driver response
                Either::Second(MatchingResponse {
                    response: driver_response,
                    msg_slot: driver_msg_slot,
                }) => {
                    let mac_svc_task_index =
                        driver_msg_slot_to_task_index[driver_msg_slot as usize];
                    let mac_task_event = MacTaskEvent::DrvSvcResponse(driver_response);
                    let mac_svc_task = mac_svc_tasks[mac_svc_task_index].take().unwrap();

                    self.step_task(
                        &mut mac_svc_tasks,
                        &mut driver_msg_slot_to_task_index,
                        &mut outstanding_driver_requests,
                        Some(&mut outstanding_mac_requests),
                        mac_svc_task_index,
                        mac_svc_task,
                        mac_task_event,
                    );
                }
            };
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn step_task<'tasks>(
        &self,
        mac_svc_tasks: &mut [Option<MacSvcTask<'tasks, RadioDriverImpl>>],
        driver_msg_slot_to_task_index: &mut [usize],
        outstanding_driver_requests: &mut heapless::Vec<
            PollingResponseToken,
            DRIVER_CHANNEL_CAPACITY,
        >,
        outstanding_mac_requests: Option<&mut [Option<ResponseToken>]>,
        mac_svc_task_index: usize,
        mac_svc_task: MacSvcTask<'tasks, RadioDriverImpl>,
        event: MacTaskEvent,
    ) {
        let is_mac_request = mac_svc_task_index < MAC_NUM_PARALLEL_REQUEST_TASKS;
        let is_mac_indication = !is_mac_request;

        let task_result = match mac_svc_task.step(event) {
            MacTaskTransition::DrvSvcRequest(updated_task, driver_request, intermediate_result) => {
                // Safety: We reserved sufficient channel capacity.
                let driver_msg_token = self
                    .driver_request_sender
                    .try_allocate_request_token()
                    .unwrap();
                let driver_response_token = self
                    .driver_request_sender
                    .send_request_polling_response(driver_msg_token, driver_request);
                driver_msg_slot_to_task_index[driver_response_token.message_slot() as usize] =
                    mac_svc_task_index;
                outstanding_driver_requests
                    .push(driver_response_token)
                    .unwrap();
                mac_svc_tasks[mac_svc_task_index] = Some(updated_task);
                debug_assert!({
                    if intermediate_result.is_some() {
                        // Only indications may produce intermediate results.
                        is_mac_indication
                    } else {
                        true
                    }
                });
                intermediate_result
            }
            MacTaskTransition::Terminated(task_result) => {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end();

                // Only MAC requests may terminate.
                debug_assert!(is_mac_request);

                Some(task_result)
            }
        };

        if let Some(task_result) = task_result {
            if is_mac_request {
                self.handle_request_task_result(
                    task_result,
                    outstanding_mac_requests.unwrap()[mac_svc_task_index]
                        .take()
                        .unwrap(),
                );
            } else {
                self.handle_indication_task_result(task_result);
            }
        }
    }

    fn create_indication_tasks<'tasks>(
        &self,
        first_mac_indication_task_index: usize,
        mac_svc_tasks: &mut [Option<MacSvcTask<'tasks, RadioDriverImpl>>],
        driver_msg_slot_to_task_index: &mut [usize],
        outstanding_driver_requests: &mut heapless::Vec<
            PollingResponseToken,
            DRIVER_CHANNEL_CAPACITY,
        >,
    ) where
        'svc: 'tasks,
    {
        for mac_indication_task_index in first_mac_indication_task_index..mac_svc_tasks.len() {
            let mac_indication_task =
                MacSvcTask::DataIndication(DataIndicationTask::<'tasks, RadioDriverImpl>::new(
                    self.buffer_allocator,
                ));
            self.step_task(
                mac_svc_tasks,
                driver_msg_slot_to_task_index,
                outstanding_driver_requests,
                None,
                mac_indication_task_index,
                mac_indication_task,
                MacTaskEvent::Entry,
            );
        }
    }

    fn create_request_task(&self, mac_request: MacRequest) -> MacSvcTask<'_, RadioDriverImpl> {
        match mac_request {
            MacRequest::McpsDataRequest(data_request) => {
                MacSvcTask::DataRequest(DataRequestTask::new(data_request))
            }
            MacRequest::MlmeBeaconRequest(_) => todo!(),
            MacRequest::MlmeSetRequest(_) => todo!(),
        }
    }

    fn handle_request_task_result(
        &self,
        result: MacSvcTaskResult<RadioDriverImpl>,
        response_token: ResponseToken,
    ) {
        match result {
            MacSvcTaskResult::DataRequest(task_result) => {
                let recovered_radio_frame = match task_result {
                    DataRequestResult::Sent(recovered_radio_frame) => recovered_radio_frame,
                    DataRequestResult::CcaBusy(unsent_radio_frame)
                    | DataRequestResult::Nack(unsent_radio_frame) => {
                        // TODO: CSMA/CA or Retry.
                        unsent_radio_frame.forget_size::<RadioDriverImpl>()
                    }
                };

                // Safety: Clients must allocate buffers from the MAC's
                //         allocator.
                unsafe {
                    self.buffer_allocator
                        .deallocate_buffer(recovered_radio_frame.into_buffer());
                }

                // Safety: We signal reception _after_ de-allocating the buffer
                //         so that clients can use the reception signal to
                //         safely manage bounded buffer resources. We may even
                //         return the buffer at some time so that it doesn't
                //         have to be re-allocated. We just don't do that
                //         currently as the smoltcp driver is synchronous and
                //         cannot handle any response.
                self.request_receiver.received(response_token, ());
            }
            // The rest are indications
            _ => unreachable!(),
        }
    }

    fn handle_indication_task_result(&self, result: MacSvcTaskResult<RadioDriverImpl>) {
        match result {
            MacSvcTaskResult::DataIndication(DataIndication { mpdu, .. }) => {
                self.handle_incoming_mpdu(mpdu);
            }
            // The rest are requests
            _ => unreachable!(),
        }
    }

    fn handle_incoming_mpdu(&self, mpdu: MpduFrame) {
        // TODO: Implement proper handling of incoming frames.
        match mpdu.frame_control().frame_type() {
            FrameType::Data => {
                if let Some(request_token) = self.indication_sender.try_allocate_request_token() {
                    let indication = MacIndication::McpsData(DataIndication {
                        mpdu,
                        timestamp: None,
                    });

                    // TODO: Poll response, once we work with MAC response
                    //       primitives.
                    self.indication_sender
                        .send_request_no_response(request_token, indication);
                } else {
                    // To avoid DoS we drop incoming packets if the upper layer
                    // is not able to ingest them fast enough.

                    // Safety: Incoming frames are allocated by the
                    //         MAC service itself.
                    unsafe {
                        self.buffer_allocator.deallocate_buffer(mpdu.into_buffer());
                    }
                }

                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end();
            }
            _ => {
                // Safety: Incoming frames are allocated by the
                //         MAC service itself.
                unsafe {
                    self.buffer_allocator.deallocate_buffer(mpdu.into_buffer());
                }
            }
        }
    }
}
