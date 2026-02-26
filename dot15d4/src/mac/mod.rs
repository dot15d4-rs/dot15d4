mod mcps;
pub mod mlme;
pub mod neighbors;
pub mod primitives;
pub mod procedures;
mod task;
#[cfg(test)]
pub mod tests;

use dot15d4_driver::timer::NsInstant;
pub use dot15d4_frame as frame;

use paste::paste;

#[cfg(feature = "tsch")]
use self::mlme::associate::AssociateIndication;
#[cfg(feature = "tsch")]
use self::mlme::associate::{AssociateIndicationTask, AssociateRequestTask};
#[cfg(feature = "tsch")]
use self::mlme::scan::ScanRequestTask;
#[cfg(feature = "tsch")]
use self::mlme::tsch::{
    mode::TschModeRequestTask, setlink::SetLinkRequestTask, setslotframe::SetSlotframeRequestTask,
};
use self::{
    frame::mpdu::MpduFrame,
    mcps::data::{DataIndication, DataIndicationTask, DataRequestTask},
    mlme::{get::GetRequestTask, reset::ResetRequestTask, set::SetRequestTask},
    primitives::{MacConfirm, MacIndication, MacRequest},
    task::*,
};
use crate::{
    driver::radio::{
        frame::FrameType,
        phy::{OQpsk250KBit, Phy, PhyConfig},
        DriverConfig, MAX_DRIVER_OVERHEAD,
    },
    mac::mcps::data::DataConfirm,
    scheduler::{SchedulerRequestSender, SCHEDULER_CHANNEL_CAPACITY},
    util::{
        allocator::{BufferAllocator, IntoBuffer},
        sync::{
            channel::{Channel, Receiver, Sender},
            select, Either, MatchingResponse, PollingResponseToken, ResponseToken,
        },
    },
};

// TODO: Make allocator and channel capacities and the number of upper layer
//       tasks configurable.

/// The max number of UL Rx tokens that may be handed out in parallel.
const UL_MAX_RX_TOKENS: usize = 1;

/// The max number of UL Tx tokens that may be handed out in parallel.
/// Note: Each Rx token requires an accompanying Tx token to be allocated.
const UL_MAX_TX_TOKENS: usize = 4;

/// The number of additional messages that may be pending.
/// Note: 1 is currently the min number supported.
const UL_MSG_BACKLOG: usize = 1;

/// The number of upper layer tasks that receive indications.
///
/// Note: Currently we assume a single data channel terminated by a smoltcp
///       client. But this may change once we expose one or more independent
///       control channels towards applications directly.
const UL_NUM_CLIENTS: usize = 1;

pub type MacRequestChannel =
    Channel<(), MacRequest, MacConfirm, UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;
pub type MacRequestReceiver<'channel> =
    Receiver<'channel, (), MacRequest, MacConfirm, UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;
pub type MacRequestSender<'channel> =
    Sender<'channel, (), MacRequest, MacConfirm, UL_MAX_TX_TOKENS, UL_MSG_BACKLOG, 1>;

pub type MacIndicationChannel =
    Channel<(), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;
pub type MacIndicationReceiver<'channel> =
    Receiver<'channel, (), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;
pub type MacIndicationSender<'channel> =
    Sender<'channel, (), MacIndication, (), UL_MAX_RX_TOKENS, UL_MSG_BACKLOG, UL_NUM_CLIENTS>;

const MAC_NUM_PARALLEL_REQUEST_TASKS: usize = UL_MAX_TX_TOKENS;

/// Number of indication tasks: data + associate (tsch only).
#[cfg(feature = "tsch")]
const MAC_NUM_INDICATION_TASKS: usize = 2;
#[cfg(not(feature = "tsch"))]
const MAC_NUM_INDICATION_TASKS: usize = 1;

const _: () = {
    assert!(
        SCHEDULER_CHANNEL_CAPACITY >= 1 + MAC_NUM_PARALLEL_REQUEST_TASKS,
        "scheduler channel capacity does not match number of MAC tasks"
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
/// - one buffer for indication task
/// - one pre-allocated buffer for outgoing ACKs
/// - one pre-allocated buffer for incoming ACKs
pub const MAC_NUM_REQUIRED_BUFFERS: usize = UL_MAX_TX_TOKENS + MAC_NUM_INDICATION_TASKS + 3;
// TODO: Make this generic over the radio driver configuration.
pub const MAC_BUFFER_SIZE: usize =
    <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize + MAX_DRIVER_OVERHEAD;

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
                        MacTaskTransition::SchedulerRequest(updated_task, scheduler_request, task_result) => {
                            let updated_task = MacSvcTask::$mac_task(updated_task);
                            let task_result = task_result.map(|task_result| MacSvcTaskResult::$mac_task(task_result)) ;
                            MacTaskTransition::SchedulerRequest(updated_task, scheduler_request, task_result)
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

#[cfg(feature = "tsch")]
mac_svc_tasks!(
    DataRequest,
    DataIndication,
    TschModeRequest,
    ScanRequest,
    SetLinkRequest,
    SetSlotframeRequest,
    SetRequest,
    GetRequest,
    ResetRequest,
    AssociateRequest,
    AssociateIndication
);
#[cfg(not(feature = "tsch"))]
mac_svc_tasks!(
    DataRequest,
    DataIndication,
    GetRequest,
    SetRequest,
    ResetRequest
);

/// Number of mac service tasks which include requests tasks and indication tasks
const NUM_MAC_SVC_TASKS: usize = MAC_NUM_PARALLEL_REQUEST_TASKS + MAC_NUM_INDICATION_TASKS;

struct MacServiceState<'state, RadioDriverImpl: DriverConfig> {
    // MAC request tasks are indexed by the message slots of the corresponding
    // MAC requests (0..UL_NUM_PARALLEL_REQUESTS).
    //
    // MAC indication tasks use the higher indices
    // (UL_NUM_PARALLEL_REQUESTS..UL_NUM_PARALLEL_REQUESTS +
    // MAC_NUM_PARALLEL_INDICATIONS).
    //
    // We need an additional indication background task so that we can
    // efficiently use the driver service's pipelining capability.
    mac_svc_tasks: [Option<MacSvcTask<'state, RadioDriverImpl>>; NUM_MAC_SVC_TASKS],

    // Outstanding scheduler requests will be pushed to this vector and polled for
    // responses.
    outstanding_scheduler_requests: heapless::Vec<PollingResponseToken, SCHEDULER_CHANNEL_CAPACITY>,

    // A scheduler-to-MAC message index: The index corresponds to the scheduler
    // message slot, the content to the corresponding MAC request slot.
    scheduler_msg_slot_to_task_index: [usize; SCHEDULER_CHANNEL_CAPACITY],

    // Response tokens for outstanding MAC requests.
    outstanding_mac_requests: [Option<ResponseToken>; MAC_NUM_PARALLEL_REQUEST_TASKS],
}

impl<RadioDriverImpl: DriverConfig> MacServiceState<'_, RadioDriverImpl> {
    fn new() -> Self {
        Self {
            mac_svc_tasks: [const { None }; NUM_MAC_SVC_TASKS],
            outstanding_scheduler_requests: heapless::Vec::new(),
            scheduler_msg_slot_to_task_index: [0; SCHEDULER_CHANNEL_CAPACITY],
            outstanding_mac_requests: [const { None }; MAC_NUM_PARALLEL_REQUEST_TASKS],
        }
    }
}

/// A structure exposing MAC sublayer services such as MLME and MCPS. This runs
/// the main event loop that handles interactions between an upper layer and the
/// PHY sublayer. It uses channels to communicate with upper layer tasks and
/// with radio drivers.
pub struct MacService<'svc, RadioDriverImpl: DriverConfig> {
    /// Timer instance to wait until scheduler requests become pending.
    // TODO: remove allow attribute once used in code
    #[allow(dead_code)]
    timer: RadioDriverImpl::Timer,
    /// Message buffer allocator
    buffer_allocator: MacBufferAllocator,
    /// Upper layer channel from which MAC requests are received.
    request_receiver: MacRequestReceiver<'svc>,
    /// Upper layer channel to which MAC indications are sent.
    indication_sender: MacIndicationSender<'svc>,
    /// Channel to communicate with one or several radio drivers.
    scheduler_request_sender: SchedulerRequestSender<'svc>,
}

impl<'svc, RadioDriverImpl: DriverConfig> MacService<'svc, RadioDriverImpl> {
    /// Creates a new [`MacService<U, Timer, R>`].
    pub fn new(
        timer: RadioDriverImpl::Timer,
        buffer_allocator: MacBufferAllocator,
        request_receiver: MacRequestReceiver<'svc>,
        indication_sender: MacIndicationSender<'svc>,
        scheduler_request_sender: SchedulerRequestSender<'svc>,
    ) -> Self {
        Self {
            timer,
            buffer_allocator,
            request_receiver,
            indication_sender,
            scheduler_request_sender,
        }
    }

    /// Run the main event loop used by the MAC sublayer for its operation.
    ///
    /// The loop waits until receiving a MCPS-DATA request from the upper layer.
    /// It will then instantiate the corresponding state machine and start
    /// driving it. The state machine will produce scheduler service requests which
    /// will be passed on to the scheduler service. Whenever the scheduler service
    /// returns a response it will be used to drive the corresponding state
    /// machine.
    pub async fn run(&mut self) -> ! {
        let mut state = MacServiceState::new();

        self.create_indication_tasks(&mut state);

        let mut consumer_token = self
            .request_receiver
            .try_allocate_consumer_token()
            .expect("no capacity");

        loop {
            match select(
                self.request_receiver
                    .receive_request_async(&mut consumer_token, &()),
                self.scheduler_request_sender
                    .wait_for_response(&mut state.outstanding_scheduler_requests),
            )
            .await
            {
                // Upper layer: A MAC request was received. Create the corresponding task and kick it off.
                Either::First((mac_request_response_token, mac_request)) => {
                    let mac_request_task_index = mac_request_response_token.message_slot() as usize;
                    state.outstanding_mac_requests[mac_request_task_index] =
                        Some(mac_request_response_token);
                    let mac_request_task = self.create_request_task(mac_request);
                    self.step_task(
                        &mut state,
                        mac_request_task_index,
                        mac_request_task,
                        MacTaskEvent::Entry,
                    );
                }
                // Driver response
                Either::Second(MatchingResponse {
                    response: scheduler_response,
                    msg_slot: scheduler_msg_slot,
                }) => {
                    let mac_svc_task_index =
                        state.scheduler_msg_slot_to_task_index[scheduler_msg_slot as usize];
                    let mac_task_event = MacTaskEvent::SchedulerResponse(scheduler_response);
                    let mac_svc_task = state.mac_svc_tasks[mac_svc_task_index].take().unwrap();

                    self.step_task(&mut state, mac_svc_task_index, mac_svc_task, mac_task_event);
                }
            };
        }
    }

    fn step_task<'tasks>(
        &self,
        state: &mut MacServiceState<'tasks, RadioDriverImpl>,
        mac_svc_task_index: usize,
        mac_svc_task: MacSvcTask<'tasks, RadioDriverImpl>,
        event: MacTaskEvent,
    ) {
        let is_mac_request = mac_svc_task_index < MAC_NUM_PARALLEL_REQUEST_TASKS;
        let is_mac_indication = !is_mac_request;

        let task_result = match mac_svc_task.step(event) {
            MacTaskTransition::SchedulerRequest(
                updated_task,
                scheduler_request,
                intermediate_result,
            ) => {
                // Safety: We reserved sufficient channel capacity.
                let scheduler_msg_token = self
                    .scheduler_request_sender
                    .try_allocate_request_token()
                    .unwrap();
                let scheduler_response_token = self
                    .scheduler_request_sender
                    .send_request_polling_response(scheduler_msg_token, scheduler_request);
                state.scheduler_msg_slot_to_task_index
                    [scheduler_response_token.message_slot() as usize] = mac_svc_task_index;
                state
                    .outstanding_scheduler_requests
                    .push(scheduler_response_token)
                    .unwrap();
                state.mac_svc_tasks[mac_svc_task_index] = Some(updated_task);
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
                    state.outstanding_mac_requests[mac_svc_task_index]
                        .take()
                        .unwrap(),
                );
            } else {
                self.handle_indication_task_result(task_result);
            }
        }
    }

    fn create_request_task(&self, mac_request: MacRequest) -> MacSvcTask<'_, RadioDriverImpl> {
        match mac_request {
            MacRequest::McpsData(data_request) => {
                MacSvcTask::DataRequest(DataRequestTask::new(data_request))
            }
            MacRequest::MlmeBeacon(_) => todo!(),
            #[cfg(feature = "tsch")]
            MacRequest::MlmeScan(scan_request) => {
                MacSvcTask::ScanRequest(ScanRequestTask::new(scan_request))
            }
            MacRequest::MlmeSet(set_request) => {
                MacSvcTask::SetRequest(SetRequestTask::new(set_request))
            }
            MacRequest::MlmeGet(get_request) => {
                MacSvcTask::GetRequest(GetRequestTask::new(get_request))
            }
            MacRequest::MlmeReset(reset_request) => {
                MacSvcTask::ResetRequest(ResetRequestTask::new(reset_request))
            }
            #[cfg(feature = "tsch")]
            MacRequest::MlmeTschMode(mlme_request) => {
                MacSvcTask::TschModeRequest(TschModeRequestTask::new(mlme_request))
            }
            #[cfg(feature = "tsch")]
            MacRequest::MlmeSetSlotframe(set_slotframe_request) => {
                MacSvcTask::SetSlotframeRequest(SetSlotframeRequestTask::new(set_slotframe_request))
            }
            #[cfg(feature = "tsch")]
            MacRequest::MlmeSetLink(set_link_request) => {
                MacSvcTask::SetLinkRequest(SetLinkRequestTask::new(set_link_request))
            }
            #[cfg(feature = "tsch")]
            MacRequest::MlmeAssociate(associate_request) => MacSvcTask::AssociateRequest(
                AssociateRequestTask::new(associate_request, self.buffer_allocator),
            ),
        }
    }
    fn create_indication_tasks<'tasks>(&self, state: &mut MacServiceState<'tasks, RadioDriverImpl>)
    where
        'svc: 'tasks,
    {
        const DATA_INDICATION_TASK_INDEX: usize = MAC_NUM_PARALLEL_REQUEST_TASKS;

        let mac_indication_task =
            MacSvcTask::DataIndication(DataIndicationTask::<'tasks, RadioDriverImpl>::new());
        self.step_task(
            state,
            DATA_INDICATION_TASK_INDEX,
            mac_indication_task,
            MacTaskEvent::Entry,
        );

        #[cfg(feature = "tsch-coordinator")]
        {
            const ASSOCIATE_INDICATION_TASK_INDEX: usize = MAC_NUM_PARALLEL_REQUEST_TASKS + 1;

            let associate_indication_task = MacSvcTask::AssociateIndication(
                AssociateIndicationTask::<'tasks, RadioDriverImpl>::new(self.buffer_allocator),
            );
            self.step_task(
                state,
                ASSOCIATE_INDICATION_TASK_INDEX,
                associate_indication_task,
                MacTaskEvent::Entry,
            );
        }
    }

    fn handle_request_task_result(
        &self,
        result: MacSvcTaskResult<RadioDriverImpl>,
        response_token: ResponseToken,
    ) {
        match result {
            MacSvcTaskResult::DataRequest(data_confirm) => {
                let (recovered_radio_frame, instant) = match data_confirm {
                    DataConfirm::Sent(recovered_radio_frame, instant) => {
                        (recovered_radio_frame, Some(instant))
                    }
                    DataConfirm::Nack(unsent_radio_frame, instant) => (
                        unsent_radio_frame.forget_size::<RadioDriverImpl>(),
                        Some(instant),
                    ),
                    DataConfirm::ChannelAccessFailure(unsent_radio_frame) => {
                        (unsent_radio_frame.forget_size::<RadioDriverImpl>(), None)
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
                self.request_receiver
                    .received(response_token, MacConfirm::McpsData(instant));
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::ScanRequest(scan_confirm) => {
                // Scan completed - notify upper layer
                // TODO: Return scan results through a proper response type
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeScan(scan_confirm));
            }
            MacSvcTaskResult::SetRequest(set_confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeSet(set_confirm));
            }
            MacSvcTaskResult::GetRequest(get_confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeGet(get_confirm));
            }
            MacSvcTaskResult::ResetRequest(reset_confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeReset(reset_confirm));
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::TschModeRequest(mode_confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeTschMode(mode_confirm));
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::SetLinkRequest(confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeSetLink(confirm));
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::SetSlotframeRequest(confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeSetSlotframe(confirm));
            }
            MacSvcTaskResult::DataIndication(_) => unreachable!(),
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::AssociateRequest(associate_confirm) => {
                self.request_receiver
                    .received(response_token, MacConfirm::MlmeAssociate(associate_confirm));
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::AssociateIndication(_) => unreachable!(),
        }
    }

    fn handle_indication_task_result(&self, result: MacSvcTaskResult<RadioDriverImpl>) {
        match result {
            MacSvcTaskResult::DataIndication(DataIndication { mpdu, timestamp }) => {
                self.handle_incoming_mpdu(mpdu, timestamp);
            }
            #[cfg(feature = "tsch")]
            MacSvcTaskResult::AssociateIndication(indication) => {
                self.handle_associate_indication(indication);
            }
            // The rest are requests
            _ => unreachable!(),
        }
    }

    fn handle_incoming_mpdu(&self, mpdu: MpduFrame, timestamp: NsInstant) {
        // TODO: Implement proper handling of incoming frames.
        match mpdu.frame_control().frame_type() {
            // TODO: distinguish Data/Beacon
            FrameType::Data | FrameType::Beacon => {
                if let Some(request_token) = self.indication_sender.try_allocate_request_token() {
                    let indication = MacIndication::McpsData(DataIndication { mpdu, timestamp });

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

    #[cfg(feature = "tsch")]
    fn handle_associate_indication(&self, indication: AssociateIndication) {
        if let Some(request_token) = self.indication_sender.try_allocate_request_token() {
            self.indication_sender.send_request_no_response(
                request_token,
                MacIndication::MlmeAssociateIndication(indication),
            );
        }
        // If no token available, the indication is dropped (back-pressure).

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_end();
    }
}
