//! Access to IEEE 802.15.4 radio drivers.
//!
//! This module provides the upper half of the communication pipe towards IEEE
//! 802.15.4 radio drivers.

use core::cell::Cell;

use crate::{
    mac::{
        frame::mpdu::{imm_ack_frame, MpduFrame, ACK_MPDU_SIZE_WO_FCS},
        MacBufferAllocator,
    },
    util::{
        frame::Frame,
        sync::{
            select, Channel, ConsumerToken, Either, HasAddress, Receiver, ResponseToken, Sender,
        },
    },
};

use self::{
    radio::{
        frame::{
            is_frame_valid_and_for_us, RadioFrame, RadioFrameRepr, RadioFrameSized,
            RadioFrameUnsized,
        },
        phy::{Ifs, PhyConfig},
        tasks::{
            CompletedRadioTransition, CompletingRxState, ExternalRadioTransition, ListeningRxState,
            OffState, RadioTaskError, RadioTransitionResult, RxError, RxResult,
            SelfRadioTransition, TaskOff as RadioTaskOff, TaskRx as RadioTaskRx,
            TaskTx as RadioTaskTx, TxError, TxResult, TxState,
        },
        DriverConfig, PhyOf, RadioDriver, RadioDriverApi,
    },
    timer::{LocalClockDuration, LocalClockInstant},
};

use dot15d4_driver::radio::tasks::StopListeningResult;
pub use dot15d4_driver::*;

// Currently we make no distinction in the implementation of driver service
// tasks and radio tasks. But that only holds until we introduce more drivers
// with distinct capabilities.
//
// In the meantime, we use type definitions to de-couple driver service tasks
// from radio tasks.
pub trait DriverServiceTask {
    type Result;
    type Error;
}

/// Tasks can be scheduled as fast as possible ("best effort") or at a
/// well-defined tick of the local radio clock ("scheduled").
///
/// The timestamp is represented as a [`LocalClockInstant`] in terms of the
/// radio driver's local timer, i.e. the timestamp must already have been
/// compensated for clock drift.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Timestamp {
    /// A task with this timestamp will be executed back-to-back to the previous
    /// task with minimal standard-conforming inter-frame spacing.
    BestEffort,

    /// A task with this timestamp will be executed by the driver at a precisely
    /// defined time. The semantics of the timestamp depends on the task that's
    /// being scheduled (see the corresponding task's definition).
    ///
    /// Usually the timestamp will be related to the RMARKER of a frame. For all
    /// PHYs the RMARKER is defined to be the time when the beginning of the
    /// first symbol following the SFD of the frame is at the local antenna.
    Scheduled(LocalClockInstant),
}

impl From<Timestamp> for Option<LocalClockInstant> {
    fn from(value: Timestamp) -> Self {
        match value {
            Timestamp::BestEffort => None,
            Timestamp::Scheduled(instant) => Some(instant),
        }
    }
}

/// Task: receive a single frame
#[derive(Debug, PartialEq, Eq)]
pub struct DrvSvcTaskRx {
    /// The earliest time at which a frame with this RMARKER passing the local
    /// antenna SHALL be recognized. The receiver SHALL be switched on as late
    /// as possible to minimize energy consumption.
    ///
    /// Note: We do not define rx window duration at the radio driver level.
    ///       Schedule a subsequent timed [`RadioTaskOff`] or [`RadioTaskTx`]
    ///       instead to end an rx window. It's the responsibility of the driver
    ///       service to compensate for clock drift and insert guard times.
    pub start: Timestamp,

    /// radio frame allocated to receive incoming frames
    pub radio_frame: RadioFrame<RadioFrameUnsized>,
}
#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcResultRx {
    /// A valid frame was successfully received and acknowledged if requested.
    Frame(
        /// received radio frame
        RadioFrame<RadioFrameSized>,
        /// RMARKER timestamp of the received frame
        LocalClockInstant,
    ),
    /// A new task was scheduled before a frame was received.
    RxWindowEnded(
        /// recovered radio frame
        RadioFrame<RadioFrameUnsized>,
    ),
    /// A frame was received but the CRC didn't match.
    ///
    /// Note: This result is returned if the driver was programmed to switch to
    ///       the next radio task on CRC error, e.g. when scheduling a regular
    ///       off, rx or tx task back-to-back to an rx task.
    CrcError(
        /// recovered radio frame
        RadioFrame<RadioFrameUnsized>,
        /// RMARKER timestamp of the received frame
        LocalClockInstant,
    ),
    /// A frame with correct CRC was received but didn't match the filtering
    /// requirements, see IEEE 802.15.4-2024, section 6.6.2. This can be useful
    /// to implement promiscuous mode.
    FilteredFrame(
        /// received radio frame
        RadioFrame<RadioFrameSized>,
        /// RMARKER timestamp of the received frame
        LocalClockInstant,
    ),
}
pub type DrvSvcErrorRx = RxError;
impl DriverServiceTask for DrvSvcTaskRx {
    type Result = DrvSvcResultRx;
    type Error = DrvSvcErrorRx;
}

/// Task: send a single frame
#[derive(Debug, PartialEq, Eq)]
pub struct DrvSvcTaskTx {
    /// the time at which the RMARKER of the outbound frame SHALL pass the local
    /// antenna.
    pub at: Timestamp,

    /// radio frame to be sent
    pub radio_frame: RadioFrame<RadioFrameSized>,

    /// whether CCA is to be performed as a precondition to send out the frame
    pub cca: bool,
}
#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcResultTx {
    /// The frame was successfully sent and acknowledged if requested.
    /// Does not yet carry any data but MAY do so in the future.
    Sent(
        /// The transmitted frame.
        RadioFrame<RadioFrameSized>,
        /// RMARKER timestamp of the transmitted frame
        LocalClockInstant,
    ), // TODO: Support returning an optional Enh-Ack frame.
    /// The frame was sent but the ACK timeout expired or an Enh-ACK frame was
    /// received but its content indicates a NACK (used, e.g. in TSCH to signal
    /// NACK while still transporting time synchronization info).
    Nack(
        /// The radio frame that was not ack'ed.
        RadioFrame<RadioFrameSized>,
        /// RMARKER timestamp of the transmitted frame
        LocalClockInstant,
    ), // TODO: Support returning an optional Enh-Ack frame.
}
pub type DrvSvcErrorTx = TxError;
impl DriverServiceTask for DrvSvcTaskTx {
    type Result = DrvSvcResultTx;
    type Error = DrvSvcErrorTx;
}

/// driver service requests encapsulating driver service tasks
///
/// Driver service tasks, results and errors are the language between the MAC
/// scheduler and the driver service while radio tasks, results and errors are
/// the language between the driver service and a driver implementation.
///
/// Driver service tasks can be scheduled independently of driver-specific
/// capabilities, i.e. they must be available for all drivers.
///
/// Depending on driver capabilities, not all radio tasks or not all features of
/// a radio task may be available for all driver implementations. It is the
/// responsibility of the driver service to query driver capabilities and
/// polyfill missing capabilities in software while taking advantage of as many
/// of the available features of an individual driver implementation as possible
/// ("hardware offloading").
#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcRequest {
    /// Frames to be sent on air must be sized, i.e. their PDU length must be
    /// defined.
    Tx(DrvSvcTaskTx),
    /// Frames to be filled by the driver with a PDU received on air must be
    /// empty, i.e. their PDU length cannot yet be known.
    Rx(DrvSvcTaskRx),
}

impl From<DrvSvcTaskTx> for DrvSvcRequest {
    fn from(value: DrvSvcTaskTx) -> Self {
        DrvSvcRequest::Tx(value)
    }
}

impl From<DrvSvcTaskRx> for DrvSvcRequest {
    fn from(value: DrvSvcTaskRx) -> Self {
        DrvSvcRequest::Rx(value)
    }
}

/// Represents a driver service task error.
#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcTaskError<Task: DriverServiceTask> {
    /// The task could not be scheduled in time.
    ///
    /// Recovers the task that could not be scheduled.
    SchedulingError(Task),

    /// The driver service task itself failed.
    Task(Task::Error),
}

pub type DrvSvcTaskResult<Task> =
    Result<<Task as DriverServiceTask>::Result, DrvSvcTaskError<Task>>;

/// driver service response encapsulating driver service results
///
/// See [`DrvSvcRequest`] for more details.
#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcResponse {
    Tx(DrvSvcTaskResult<DrvSvcTaskTx>),
    Rx(DrvSvcTaskResult<DrvSvcTaskRx>),
}

impl From<TxResult> for DrvSvcResponse {
    fn from(value: TxResult) -> Self {
        let TxResult::Sent(radio_frame, timestamp) = value;
        DrvSvcResponse::Tx(Ok(DrvSvcResultTx::Sent(radio_frame, timestamp)))
    }
}

impl From<RadioTaskError<RadioTaskTx>> for DrvSvcResponse {
    fn from(value: RadioTaskError<RadioTaskTx>) -> Self {
        match value {
            RadioTaskError::Scheduling(tx_task, scheduling_error) => {
                DrvSvcResponse::Tx(Err(DrvSvcTaskError::SchedulingError(DrvSvcTaskTx {
                    at: scheduling_error
                        .instant
                        .map_or_else(|| Timestamp::BestEffort, Timestamp::Scheduled),
                    radio_frame: tx_task.radio_frame,
                    cca: tx_task.cca,
                })))
            }
            RadioTaskError::Task(tx_error) => {
                DrvSvcResponse::Tx(Err(DrvSvcTaskError::Task(tx_error)))
            }
        }
    }
}

impl From<RxResult> for DrvSvcResponse {
    fn from(value: RxResult) -> Self {
        let result = match value {
            RxResult::Frame(radio_frame, timestamp) => {
                DrvSvcResultRx::Frame(radio_frame, timestamp)
            }
            RxResult::RxWindowEnded(radio_frame) => DrvSvcResultRx::RxWindowEnded(radio_frame),
            RxResult::CrcError(radio_frame, timestamp) => {
                DrvSvcResultRx::CrcError(radio_frame, timestamp)
            }
        };
        DrvSvcResponse::Rx(Ok(result))
    }
}

impl From<RadioTaskError<RadioTaskRx>> for DrvSvcResponse {
    fn from(value: RadioTaskError<RadioTaskRx>) -> Self {
        match value {
            RadioTaskError::Scheduling(rx_task, scheduling_error) => {
                DrvSvcResponse::Rx(Err(DrvSvcTaskError::SchedulingError(DrvSvcTaskRx {
                    start: scheduling_error
                        .instant
                        .map_or_else(|| Timestamp::BestEffort, Timestamp::Scheduled),
                    radio_frame: rx_task.radio_frame,
                })))
            }
            RadioTaskError::Task(rx_error) => {
                DrvSvcResponse::Rx(Err(DrvSvcTaskError::Task(rx_error)))
            }
        }
    }
}

// TODO: Make channel capacities configurable.
pub const DRIVER_CHANNEL_CAPACITY: usize = 4;
const DRIVER_CHANNEL_BACKLOG: usize = 1;

/// To ensure progress, we give precedence of outbound tasks over inbound tasks.
/// We therefore route these two classes of tasks into separate virtual
/// channels.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TaskDirection {
    Outbound,
    Inbound,
    Any,
}

/// Currently we do not address different service instances wrapping
/// different drivers. This may change when managing several radios over a
/// single channel.
impl HasAddress<TaskDirection> for DrvSvcRequest {
    fn matches(&self, address: &TaskDirection) -> bool {
        if matches!(*address, TaskDirection::Any) {
            return true;
        }

        match self {
            DrvSvcRequest::Tx(_) => matches!(*address, TaskDirection::Outbound),
            DrvSvcRequest::Rx(_) => matches!(*address, TaskDirection::Inbound),
        }
    }
}

pub type DriverRequestChannel = Channel<
    TaskDirection,
    DrvSvcRequest,
    DrvSvcResponse,
    DRIVER_CHANNEL_CAPACITY,
    DRIVER_CHANNEL_BACKLOG,
    1,
>;
pub type DriverRequestReceiver<'channel> = Receiver<
    'channel,
    TaskDirection,
    DrvSvcRequest,
    DrvSvcResponse,
    DRIVER_CHANNEL_CAPACITY,
    DRIVER_CHANNEL_BACKLOG,
    1,
>;
pub type DriverRequestSender<'channel> = Sender<
    'channel,
    TaskDirection,
    DrvSvcRequest,
    DrvSvcResponse,
    DRIVER_CHANNEL_CAPACITY,
    DRIVER_CHANNEL_BACKLOG,
    1,
>;

/// We use this runtime state to prove that the radio can only be in three
/// different states when looping. This allows us to implement the scheduler as
/// an event loop while still reaping all benefits of a behaviorally typed radio
/// driver.
enum DriverState<RadioDriverImpl: DriverConfig> {
    /// We are currently sending a frame.
    Tx(
        RadioDriver<RadioDriverImpl, RadioTaskTx>,
        /// The sequence number of the frame if the outgoing frame requested ACK,
        /// otherwise None.
        Option<u8>,
        /// The IFS is determined by the length of the packet currently being
        /// sent.
        Ifs<PhyOf<RadioDriverImpl>>,
    ),
    /// There is no tx frame pending and we have rx capacity to receive an
    /// incoming frame.
    Rx(RadioDriver<RadioDriverImpl, RadioTaskRx>),
    /// We have no rx capacity and no tx frame is pending.
    Off(RadioDriver<RadioDriverImpl, RadioTaskOff>),
}

/// Structure managing a given driver implementation. Knows about and manages
/// individual driver capabilities and exposes a unified API to the MAC service.
pub struct DriverService<'svc, RadioDriverImpl: DriverConfig> {
    /// The current radio driver state.
    driver_state: Cell<Option<DriverState<RadioDriverImpl>>>,

    /// Receiver for driver service tasks.
    request_receiver: DriverRequestReceiver<'svc>,

    // Pre-allocated frame for outbound acknowledgements.
    outbound_ack_frame: Cell<Option<RadioFrame<RadioFrameSized>>>,

    // Pre-allocated frame for inbound acknowledgements and invalid frame
    // buffering.
    temp_inbound_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,
}

impl<'svc, RadioDriverImpl: DriverConfig> DriverService<'svc, RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, RadioTaskOff>: OffState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskRx>: ListeningRxState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskTx>: TxState<RadioDriverImpl> + RadioDriverApi,
{
    /// Creates a new [`DriverService`] instance wrapping the given driver
    /// implementation.
    pub fn new(
        driver: RadioDriver<RadioDriverImpl, RadioTaskOff>,
        driver_service_receiver: DriverRequestReceiver<'svc>,
        buffer_allocator: MacBufferAllocator,
    ) -> Self {
        Self {
            driver_state: Cell::new(Some(DriverState::Off(driver))),
            request_receiver: driver_service_receiver,
            outbound_ack_frame: Cell::new(Some(Self::allocate_outbound_ack_frame(
                buffer_allocator,
            ))),
            temp_inbound_frame: Cell::new(Some(Self::allocate_inbound_frame(buffer_allocator))),
        }
    }

    /// Pre-allocates and pre-populates a re-usable outbound ACK frame.
    ///
    /// Safety: We have separate incoming and outbound ACK buffers to ensure
    ///         that incoming ACKs cannot corrupt the pre-populated outbound ACK
    ///         buffer. This allows us to re-use the outbound ACK buffer w/o
    ///         validation.
    fn allocate_outbound_ack_frame(
        buffer_allocator: MacBufferAllocator,
    ) -> RadioFrame<RadioFrameSized> {
        let radio_frame_repr = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new();
        let outbound_ack_buffer_size = ACK_MPDU_SIZE_WO_FCS as usize
            + (radio_frame_repr.fcs_length() + radio_frame_repr.driver_overhead()) as usize;

        imm_ack_frame::<RadioDriverImpl>(
            0,
            buffer_allocator
                .try_allocate_buffer(outbound_ack_buffer_size)
                .expect("no capacity"),
        )
        .into_radio_frame::<RadioDriverImpl>()
    }

    /// Pre-allocates a re-usable rx frame for ACK or invalid frame buffering.
    fn allocate_inbound_frame(
        buffer_allocator: MacBufferAllocator,
    ) -> RadioFrame<RadioFrameUnsized> {
        let inbound_frame_buffer_size = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new()
            .max_buffer_length() as usize;
        RadioFrame::new::<RadioDriverImpl>(
            buffer_allocator
                .try_allocate_buffer(inbound_frame_buffer_size)
                .expect("no capacity"),
        )
    }

    /// Run the main driver service event loop.
    pub async fn run(&self) -> ! {
        let mut consumer_token = self
            .request_receiver
            .try_allocate_consumer_token()
            .expect("capacity");

        let mut driver_state = self.driver_state.take().expect("already running");
        let mut current_task_response_token = None;

        loop {
            (driver_state, current_task_response_token) = match driver_state {
                DriverState::Rx(listening_rx_driver) => {
                    debug_assert!(current_task_response_token.is_some());
                    self.try_receive_frame(
                        listening_rx_driver,
                        current_task_response_token.take().unwrap(),
                        &mut consumer_token,
                    )
                    .await
                }
                DriverState::Tx(tx_driver, ack_seq_num, next_task_ifs) => {
                    debug_assert!(current_task_response_token.is_some());
                    self.send_frame(
                        tx_driver,
                        current_task_response_token.take(),
                        ack_seq_num,
                        next_task_ifs,
                    )
                    .await
                }
                DriverState::Off(off_driver) => {
                    debug_assert!(current_task_response_token.is_none());
                    let (driver_state, current_task_response_token) = self
                        .schedule_next_request(off_driver, &mut consumer_token)
                        .await;
                    (driver_state, Some(current_task_response_token))
                }
            };
        }
    }

    /// Waits for an incoming frame and receive it or end the rx window when an
    /// outbound request is received - whatever happens first. Finally switch to
    /// the next requested driver state (if any) or turns the radio off.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    #[allow(clippy::await_holding_refcell_ref)]
    async fn try_receive_frame(
        &self,
        mut listening_rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        rx_task_response_token: ResponseToken,
        consumer_token: &mut ConsumerToken,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        loop {
            // Wait until a frame is being received or the next outbound request
            // ends the rx window.
            match select(
                listening_rx_driver.wait_for_frame_start(),
                self.request_receiver
                    .peek_request_async(consumer_token, &TaskDirection::Outbound),
            )
            .await
            {
                // The radio started receiving a frame.
                Either::First(_) => {
                    let hardware_address = listening_rx_driver.ieee802154_address();
                    let completing_rx_driver =
                        if let Ok(result) = listening_rx_driver.stop_listening(None).await {
                            result.1
                        } else {
                            // Without a timeout the stop_listening() method
                            // shouldn't fail.
                            unreachable!()
                        };
                    return self
                        .validate_and_receive_frame(
                            completing_rx_driver,
                            rx_task_response_token,
                            &hardware_address,
                        )
                        .await;
                }

                // An outbound request is pending.
                Either::Second(tx_request_ref) => {
                    let (at, cca) =
                        if let DrvSvcRequest::Tx(DrvSvcTaskTx { at, cca, .. }) = &*tx_request_ref {
                            (*at, *cca)
                        } else {
                            unreachable!()
                        };
                    drop(tx_request_ref);

                    let latest_frame_start = if let Timestamp::Scheduled(at) = at {
                        // rx_task_end
                        //   = at - (cca ? macUnitBackoffPeriod : 0) - LIFS
                        //     - rmarker_offset
                        //
                        // latest_frame_start
                        //   = rx_task_end - ppdu_rx_time(phyMaxPacketSize)
                        //     + rmarker_offset
                        //   = at - (cca ? macUnitBackoffPeriod : 0) - LIFS
                        //     - ppdu_rx_time(phyMaxPacketSize)
                        //
                        // Note:
                        //  - The RMARKER offsets cancel each other out.
                        //  - As we may still receive a max-sized frame, we need
                        //    to cater for LIFS in the worst case.
                        let ifs = <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_LIFS_PERIOD;
                        let max_ppdu_rx = listening_rx_driver.ppdu_rx_duration(
                            <PhyOf<RadioDriverImpl> as PhyConfig>::PHY_MAX_PACKET_SIZE,
                        );
                        let mut latest_frame_start = at - ifs - max_ppdu_rx;
                        if cca {
                            let back_off_period =
                                <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_UNIT_BACKOFF_PERIOD;
                            latest_frame_start -= back_off_period
                        }
                        Some(latest_frame_start)
                    } else {
                        None
                    };

                    let hardware_address = listening_rx_driver.ieee802154_address();
                    let (stop_listening_result, completing_rx_driver) =
                        match listening_rx_driver.stop_listening(latest_frame_start).await {
                            Ok(result) => result,
                            Err((_, recovered_rx_driver)) => {
                                listening_rx_driver = recovered_rx_driver;
                                continue;
                            }
                        };

                    if matches!(stop_listening_result, StopListeningResult::FrameStarted(_)) {
                        // A frame has started in the meantime, so we cannot
                        // serve the pending tx request, yet.
                        return self
                            .validate_and_receive_frame(
                                completing_rx_driver,
                                rx_task_response_token,
                                &hardware_address,
                            )
                            .await;
                    } else {
                        // No frame has started, so we can safely end the rx
                        // window and serve the pending tx request.

                        // Safety: We know that there is a pending tx request.
                        let tx_request = self
                            .request_receiver
                            .try_receive_request(&TaskDirection::Outbound)
                            .unwrap();

                        // End the rx window and handle the pending tx request.
                        return self
                            .end_rx_window(
                                completing_rx_driver,
                                rx_task_response_token,
                                None,
                                Some(tx_request),
                            )
                            .await;
                    }
                }
            }
        }
    }

    /// Once an incoming frame has been observed, it needs to be validated and
    /// possibly acknowledged.
    async fn validate_and_receive_frame(
        &self,
        mut completing_rx_driver: impl CompletingRxState<RadioDriverImpl>,
        rx_task_response_token: ResponseToken,
        hardware_address: &[u8; 8],
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        let preliminary_frame_info = completing_rx_driver.preliminary_frame_info().await.unwrap();
        let next_task_ifs = Ifs::from_mpdu_length(preliminary_frame_info.mpdu_length);
        let frame_is_valid = is_frame_valid_and_for_us(hardware_address, &preliminary_frame_info);

        // If the frame is valid and ACK is requested, then
        // schedule a tx ACK task. Otherwise finalize the rx
        // task and receive the next task (if any).
        if frame_is_valid {
            // Safety: Valid frames always have a frame control field.
            let ack_request = preliminary_frame_info.frame_control.unwrap().ack_request();
            let seq_nr = preliminary_frame_info.seq_nr;
            match seq_nr {
                Some(seq_nr) if ack_request => {
                    self.receive_frame_with_ack(
                        completing_rx_driver,
                        rx_task_response_token,
                        seq_nr,
                        next_task_ifs,
                    )
                    .await
                }
                _ => {
                    self.receive_frame(
                        completing_rx_driver,
                        None,
                        rx_task_response_token,
                        Some(next_task_ifs),
                    )
                    .await
                }
            }
        } else {
            self.drop_invalid_frame(completing_rx_driver, rx_task_response_token)
                .await
        }
    }

    /// Prepares an outgoing ACK frame, schedules it and sends it. Then switches
    /// to the next requested driver state (if any) or turns the radio off.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    async fn receive_frame_with_ack(
        &self,
        completing_rx_driver: impl CompletingRxState<RadioDriverImpl>,
        rx_task_response_token: ResponseToken,
        seq_nr: u8,
        next_task_ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: We use the tx ACK frame sequentially and exclusively from
        //         this method.
        let outbound_ack_frame = self.outbound_ack_frame.take().unwrap();

        let mut outbound_ack_mpdu = MpduFrame::from_radio_frame(outbound_ack_frame);
        let _ = outbound_ack_mpdu.try_set_sequence_number(seq_nr);
        let outbound_ack_frame = outbound_ack_mpdu.into_radio_frame::<RadioDriverImpl>();

        let outbound_ack_task = RadioTaskTx {
            radio_frame: outbound_ack_frame,
            cca: false,
        };

        match completing_rx_driver
            .schedule_tx(outbound_ack_task, None, Some(Ifs::ack()), true)
            .complete_and_transition()
            .await
        {
            // CRC ok: Send the received frame back to the client and update the
            //         driver state.
            CompletedRadioTransition::Entered(RadioTransitionResult {
                prev_task_result: rx_task_result,
                this_state: tx_driver,
                ..
            }) => {
                self.request_receiver
                    .received(rx_task_response_token, rx_task_result.into());

                return self.send_frame(tx_driver, None, None, next_task_ifs).await;
            }
            // CRC mismatch: Cancel ACK, recover the pre-allocated ACK frame and
            //               leave the driver in the rx state.
            CompletedRadioTransition::Rollback(
                listening_rx_driver,
                rx_task_error,
                rx_task_result,
                recovered_outbound_ack_task,
            ) => {
                debug_assert!(matches!(
                    rx_task_error,
                    RadioTaskError::Task(RxError::CrcError)
                ));
                debug_assert!(rx_task_result.is_none());

                self.outbound_ack_frame
                    .set(Some(recovered_outbound_ack_task.radio_frame));

                (
                    DriverState::Rx(listening_rx_driver),
                    Some(rx_task_response_token),
                )
            }
            // Safety: Scheduling ACK cannot fall back as it does no CCA.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        }
    }

    /// Finalizes ongoing frame reception. Then switches to the next requested
    /// driver state (if any) or turns the radio off.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    async fn receive_frame(
        &self,
        completing_rx_driver: impl CompletingRxState<RadioDriverImpl>,
        rx_ack_info: Option<(RadioFrame<RadioFrameSized>, LocalClockInstant, u8)>,
        prev_task_response_token: ResponseToken,
        next_task_ifs: Option<Ifs<PhyOf<RadioDriverImpl>>>,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        fn handle_rx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            prev_task_response_token: ResponseToken,
            rx_task_result: RxResult,
            rx_ack_info: Option<(RadioFrame<RadioFrameSized>, LocalClockInstant, u8)>,
        ) {
            if let Some((tx_radio_frame, tx_timestamp, seq_nr)) = rx_ack_info {
                // Expect rx ACK frame
                let (tx_result, recovered_rx_frame) = match rx_task_result {
                    RxResult::Frame(rx_ack_frame, _) => {
                        // TODO: Support enhanced ACK.
                        const ACK_FC_MASK: u16 = !0x1000; // Frame version 2003 or 2006
                        const ACK_FC: u16 = 0x0002; // Frame type ACK, other flags all zero
                        let ack = if rx_ack_frame.sdu_wo_fcs_length().get() == 3 {
                            let sdu = rx_ack_frame.sdu_ref();
                            let fc = u16::from_le_bytes([sdu[0], sdu[1]]) & ACK_FC_MASK;
                            fc == ACK_FC && sdu[2] == seq_nr
                        } else {
                            false
                        };
                        let tx_result = if ack {
                            DrvSvcResultTx::Sent(tx_radio_frame, tx_timestamp)
                        } else {
                            DrvSvcResultTx::Nack(tx_radio_frame, tx_timestamp)
                        };
                        (tx_result, rx_ack_frame.forget_size::<RadioDriverImpl>())
                    }
                    RxResult::CrcError(recovered_rx_frame, _) => (
                        DrvSvcResultTx::Nack(tx_radio_frame, tx_timestamp),
                        recovered_rx_frame,
                    ),
                    RxResult::RxWindowEnded(_) => unreachable!(),
                };
                this.request_receiver
                    .received(prev_task_response_token, DrvSvcResponse::Tx(Ok(tx_result)));
                this.temp_inbound_frame.set(Some(recovered_rx_frame));
            } else {
                // Expect regular frame reception.
                this.request_receiver
                    .received(prev_task_response_token, rx_task_result.into());
            }
        }

        let next_request = self
            .request_receiver
            .try_receive_request(&TaskDirection::Any);
        match next_request {
            Some((next_response_token, next_request)) => match next_request {
                DrvSvcRequest::Tx(tx_task) => {
                    let tx_task_ack_seq_nr = tx_task.radio_frame.ack_seq_num();
                    let tx_task_ifs = Ifs::from_mpdu_length(tx_task.radio_frame.sdu_length().get());
                    let DrvSvcTaskTx {
                        at,
                        radio_frame,
                        cca,
                    } = tx_task;
                    match completing_rx_driver
                        .schedule_tx(
                            RadioTaskTx { radio_frame, cca },
                            at.into(),
                            next_task_ifs,
                            false,
                        )
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            prev_task_result: rx_task_result,
                            this_state: tx_driver,
                            ..
                        }) => {
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            (
                                DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                                Some(next_response_token),
                            )
                        }
                        CompletedRadioTransition::Fallback(
                            RadioTransitionResult {
                                prev_task_result: rx_task_result,
                                this_state: off_driver,
                                ..
                            },
                            tx_task_error,
                        ) => {
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            self.request_receiver
                                .received(next_response_token, tx_task_error.into());

                            (DriverState::Off(off_driver), None)
                        }
                        // Safety: The transition was programmed to not roll
                        //         back on CRC error.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                    }
                }
                DrvSvcRequest::Rx(rx_task) => {
                    // We're already receiving another request and are
                    // therefore guaranteed to make progress. Therefore
                    // scheduling rx back-to-back is ok.
                    let DrvSvcTaskRx { start, radio_frame } = rx_task;

                    // Frames may only be received back-to-back with best-effort
                    // timing.
                    assert!(matches!(start, Timestamp::BestEffort));

                    match completing_rx_driver
                        .schedule_rx(RadioTaskRx { radio_frame }, next_task_ifs, false)
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            prev_task_result: rx_task_result,
                            this_state: listening_rx_driver,
                            ..
                        }) => {
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            (
                                DriverState::Rx(listening_rx_driver),
                                Some(next_response_token),
                            )
                        }
                        // Safety: The transition task was programmed to not
                        //         roll back on CRC error.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling rx cannot fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            },
            None => match completing_rx_driver
                .schedule_off(None, true)
                .complete_and_transition()
                .await
            {
                CompletedRadioTransition::Entered(RadioTransitionResult {
                    prev_task_result: rx_task_result,
                    this_state: off_driver,
                    ..
                }) => {
                    handle_rx_task_result(
                        self,
                        prev_task_response_token,
                        rx_task_result,
                        rx_ack_info,
                    );

                    (DriverState::Off(off_driver), None)
                }
                CompletedRadioTransition::Rollback(
                    listening_rx_driver,
                    rx_task_error,
                    rx_task_result,
                    .., // It is safe to drop the off task.
                ) => {
                    debug_assert!(matches!(
                        rx_task_error,
                        RadioTaskError::Task(RxError::CrcError)
                    ));
                    debug_assert!(rx_task_result.is_none());

                    // We rolled back to the previous rx task
                    (
                        DriverState::Rx(listening_rx_driver),
                        Some(prev_task_response_token),
                    )
                }
                // Safety: Switching the radio off is infallible.
                CompletedRadioTransition::Fallback(..) => unreachable!(),
            },
        }
    }

    /// Schedules rx into a temporary buffer back-to-back while finalizing
    /// reception of the invalid frame. Then drops the invalid frame. The
    /// recovered buffer from the dropped frame becomes the new temporary buffer.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    async fn drop_invalid_frame(
        &self,
        completing_rx_driver: impl CompletingRxState<RadioDriverImpl>,
        rx_task_response_token: ResponseToken,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: The temporary rx frame will be recovered by the end of the
        //         procedure.
        let temporary_rx_frame = self.temp_inbound_frame.take().unwrap();
        let rx_task = RadioTaskRx {
            radio_frame: temporary_rx_frame,
        };
        match completing_rx_driver
            .schedule_rx(rx_task, None, false)
            .complete_and_transition()
            .await
        {
            CompletedRadioTransition::Entered(RadioTransitionResult {
                prev_task_result: rx_task_result,
                this_state: listening_rx_driver,
                ..
            }) => {
                let recovered_rx_frame = match rx_task_result {
                    RxResult::Frame(invalid_frame, _) => {
                        invalid_frame.forget_size::<RadioDriverImpl>()
                    }
                    RxResult::RxWindowEnded(recovered_rx_frame)
                    | RxResult::CrcError(recovered_rx_frame, _) => recovered_rx_frame,
                };

                // Safety: Unsized frames (aka rx frames) for the same driver
                //         are always capable to accommodate the max SDU length,
                //         so they are interchangeable.
                self.temp_inbound_frame.set(Some(recovered_rx_frame));

                (
                    DriverState::Rx(listening_rx_driver),
                    Some(rx_task_response_token),
                )
            }
            // Safety: The transition task was programmed to not roll back on
            //         CRC error.
            CompletedRadioTransition::Rollback(..) => unreachable!(),
            // Safety: Scheduling rx tasks does not fall back.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        }
    }

    /// Ends the ongoing rx window by scheduling the next request and responding
    /// to the previous request.
    ///
    /// If the previous request was a tx request: We end up here because ACK
    /// reception timed out and the ACK reception window needs to be ended. The
    /// tx request will be nack'ed by this method and the next request
    /// scheduled.
    ///
    /// If the previous request was an rx request: We received a concurrent tx
    /// request that needs to make progress. The previous rx request will be
    /// ended without receiving a frame and the tx request scheduled.
    async fn end_rx_window(
        &self,
        completing_rx_driver: impl CompletingRxState<RadioDriverImpl>,
        prev_task_response_token: ResponseToken,
        rx_ack_info: Option<(RadioFrame<RadioFrameSized>, LocalClockInstant)>,
        next_request: Option<(ResponseToken, DrvSvcRequest)>,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        fn handle_rx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            rx_task_result: RxResult,
            rx_ack_info: Option<(RadioFrame<RadioFrameSized>, LocalClockInstant)>,
            prev_task_response_token: ResponseToken,
        ) {
            // It is improbable but possible that an inbound frame arrives just
            // as we try ending the rx window. We drop the incoming frame in
            // this case as if we had ended the rx window slightly earlier.
            //
            // Note: Well timed protocols should not experience this situation,
            //       also see the requirement in the method docs re timed
            //       follow-up tasks.
            let rx_radio_frame = match rx_task_result {
                RxResult::Frame(radio_frame, _) => radio_frame.forget_size::<RadioDriverImpl>(),
                RxResult::RxWindowEnded(radio_frame) | RxResult::CrcError(radio_frame, _) => {
                    radio_frame
                }
            };

            if let Some((tx_radio_frame, tx_timestamp)) = rx_ack_info {
                // End rx ACK window
                this.temp_inbound_frame.set(Some(rx_radio_frame));
                let tx_task_result = DrvSvcResultTx::Nack(tx_radio_frame, tx_timestamp);
                this.request_receiver.received(
                    prev_task_response_token,
                    DrvSvcResponse::Tx(Ok(tx_task_result)),
                );
            } else {
                // End regular rx window
                let rx_task_result = RxResult::RxWindowEnded(rx_radio_frame);
                this.request_receiver
                    .received(prev_task_response_token, rx_task_result.into());
            }
        }

        match next_request {
            Some((tx_task_response_token, DrvSvcRequest::Tx(tx_task))) => {
                let DrvSvcTaskTx {
                    at,
                    radio_frame,
                    cca,
                } = tx_task;
                let tx_task_ack_seq_nr = radio_frame.ack_seq_num();
                let tx_task_ifs = Ifs::from_mpdu_length(radio_frame.sdu_length().get());

                match completing_rx_driver
                    .schedule_tx(RadioTaskTx { radio_frame, cca }, at.into(), None, false)
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: rx_task_result,
                        this_state: tx_driver,
                        ..
                    }) => {
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        (
                            DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                            Some(tx_task_response_token),
                        )
                    }
                    // Fallback to "off" state due to CCA busy when trying to schedule
                    // the tx task.
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult {
                            prev_task_result: rx_task_result,
                            this_state: off_driver,
                            ..
                        },
                        tx_task_error,
                    ) => {
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        // Report CCA busy as result of the tx task.
                        self.request_receiver
                            .received(tx_task_response_token, tx_task_error.into());

                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: The transition was programmed not to roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                }
            }
            Some((rx_task_response_token, DrvSvcRequest::Rx(rx_task))) => {
                // We only ever end an rx window with another rx task if the
                // ACK reception window after a tx task times out. In this case:
                // - Schedule the waiting rx task back-to-back without IFS.
                // - Recover the ACK frame.
                // - Report NACK wrt the outstanding tx task.

                let DrvSvcTaskRx { start, radio_frame } = rx_task;

                // Only best-effort tasks may be scheduled after a tx task.
                // To schedule a timed task after tx, schedule a "Radio Off"
                // task in between first.
                assert!(matches!(start, Timestamp::BestEffort));

                match completing_rx_driver
                    .schedule_rx(RadioTaskRx { radio_frame }, None, false)
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: rx_ack_result,
                        this_state: listening_rx_driver,
                        ..
                    }) => {
                        match rx_ack_result {
                            RxResult::RxWindowEnded(radio_frame) => {
                                self.temp_inbound_frame.set(Some(radio_frame))
                            }
                            _ => unreachable!(),
                        }

                        let (tx_radio_frame, tx_timestamp) = rx_ack_info.unwrap();
                        let tx_task_result = DrvSvcResponse::Tx(Ok(DrvSvcResultTx::Nack(
                            tx_radio_frame,
                            tx_timestamp,
                        )));
                        self.request_receiver
                            .received(prev_task_response_token, tx_task_result);

                        (
                            DriverState::Rx(listening_rx_driver),
                            Some(rx_task_response_token),
                        )
                    }
                    // Safety: The transition task was programmed to not
                    //         roll back on CRC error.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                    // Safety: Scheduling rx cannot fall back.
                    CompletedRadioTransition::Fallback(..) => unreachable!(),
                }
            }
            None => {
                match completing_rx_driver
                    .schedule_off(None, false)
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: rx_task_result,
                        this_state: off_driver,
                        ..
                    }) => {
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: Switching the driver off from an rx state
                    //         w/o rollback should be infallible.
                    _ => unreachable!(),
                }
            }
        }
    }

    /// Sends the scheduled radio frame then switches to the next requested
    /// driver state (if any) or turns the radio off.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    async fn send_frame(
        &self,
        tx_driver: RadioDriver<RadioDriverImpl, RadioTaskTx>,
        tx_task_response_token: Option<ResponseToken>,
        tx_ack_seq_nr: Option<u8>,
        next_task_ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        async fn handle_tx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            tx_task_response_token: Option<ResponseToken>,
            tx_task_result: TxResult,
            ack_seq_nr: Option<u8>,
        ) {
            if let Some(tx_task_response_token) = tx_task_response_token {
                // External request: send back the result.
                this.request_receiver
                    .received(tx_task_response_token, tx_task_result.into());
            } else {
                // Tx ACK: recover the pre-allocated ACK frame.
                debug_assert!(ack_seq_nr.is_none());
                let TxResult::Sent(radio_frame, ..) = tx_task_result;
                this.outbound_ack_frame.set(Some(radio_frame));
            }
        }

        if let Some(tx_ack_seq_nr) = tx_ack_seq_nr {
            // Safety: Only regular tx tasks can request acknowledgement and
            //         therefore a response token is expected.
            return self
                .schedule_and_await_ack(tx_driver, tx_task_response_token.unwrap(), tx_ack_seq_nr)
                .await;
        }

        let next_request = self
            .request_receiver
            .try_receive_request(&TaskDirection::Any);
        match next_request {
            Some((next_response_token, next_request)) => match next_request {
                DrvSvcRequest::Tx(tx_task) => {
                    let tx_task_ack_seq_nr = tx_task.radio_frame.ack_seq_num();
                    let tx_task_ifs = Ifs::from_mpdu_length(tx_task.radio_frame.sdu_length().get());
                    let DrvSvcTaskTx {
                        at,
                        radio_frame,
                        cca,
                    } = tx_task;

                    // Only best-effort tasks may be scheduled after a tx task.
                    // To schedule a timed task after tx, schedule a "Radio Off"
                    // task in between first.
                    assert!(matches!(at, Timestamp::BestEffort));

                    match tx_driver
                        .schedule_tx(RadioTaskTx { radio_frame, cca }, next_task_ifs)
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            prev_task_result: tx_task_result,
                            this_state: tx_driver,
                            ..
                        }) => {
                            handle_tx_task_result(
                                self,
                                tx_task_response_token,
                                tx_task_result,
                                tx_ack_seq_nr,
                            )
                            .await;

                            (
                                DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                                Some(next_response_token),
                            )
                        }
                        CompletedRadioTransition::Fallback(
                            RadioTransitionResult {
                                prev_task_result: tx_task_result,
                                this_state: off_driver,
                                ..
                            },
                            tx_task_error,
                        ) => {
                            if let Some(tx_task_response_token) = tx_task_response_token {
                                // External request: send back the result.
                                self.request_receiver
                                    .received(tx_task_response_token, tx_task_result.into());
                            } else {
                                // Tx ACK: recover the pre-allocated ACK frame.
                                let TxResult::Sent(radio_frame, ..) = tx_task_result;
                                self.outbound_ack_frame.set(Some(radio_frame));
                            }

                            // Send back the result of the failed transition.
                            self.request_receiver
                                .received(next_response_token, tx_task_error.into());

                            (DriverState::Off(off_driver), None)
                        }
                        // Safety: The tx task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                    }
                }
                DrvSvcRequest::Rx(rx_task) => {
                    let DrvSvcTaskRx { start, radio_frame } = rx_task;

                    // Only best-effort tasks may be scheduled after a tx task.
                    // To schedule a timed task after tx, schedule a "Radio Off"
                    // task in between first.
                    assert!(matches!(start, Timestamp::BestEffort));

                    match tx_driver
                        .schedule_rx(RadioTaskRx { radio_frame }, next_task_ifs)
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            prev_task_result: tx_task_result,
                            this_state: listening_rx_driver,
                            ..
                        }) => {
                            handle_tx_task_result(
                                self,
                                tx_task_response_token,
                                tx_task_result,
                                tx_ack_seq_nr,
                            )
                            .await;

                            (
                                DriverState::Rx(listening_rx_driver),
                                Some(next_response_token),
                            )
                        }
                        // Safety: The tx task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling an rx task doesn't fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            },
            None => {
                match tx_driver.schedule_off().complete_and_transition().await {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: tx_task_result,
                        this_state: off_driver,
                        ..
                    }) => {
                        handle_tx_task_result(
                            self,
                            tx_task_response_token,
                            tx_task_result,
                            tx_ack_seq_nr,
                        )
                        .await;

                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: Switching the driver off from a tx state should
                    //         be infallible.
                    _ => unreachable!(),
                }
            }
        }
    }

    /// Waits for an incoming ACK frame matching the given sequence number and
    /// responds to the tx task accordingly.
    async fn schedule_and_await_ack(
        &self,
        tx_driver: RadioDriver<RadioDriverImpl, RadioTaskTx>,
        tx_task_response_token: ResponseToken,
        seq_nr: u8,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: The temporary frame is always recovered before being re-used.
        let inbound_ack_frame = self.temp_inbound_frame.take().unwrap();
        let inbound_ack_task = RadioTaskRx {
            radio_frame: inbound_ack_frame,
        };
        let (listening_rx_driver, tx_radio_frame, tx_timestamp, earliest_ack_start) =
            match tx_driver
                .schedule_rx(inbound_ack_task, Ifs::ack())
                .complete_and_transition()
                .await
            {
                CompletedRadioTransition::Entered(RadioTransitionResult {
                    prev_task_result: TxResult::Sent(tx_radio_frame, tx_timestamp),
                    this_state: listening_rx_driver,
                    measured_entry: earliest_ack_start,
                    ..
                }) => (
                    listening_rx_driver,
                    tx_radio_frame,
                    tx_timestamp,
                    earliest_ack_start,
                ),
                // Safety: The tx task doesn't roll back.
                CompletedRadioTransition::Rollback(..) => unreachable!(),
                // Safety: The rx task doesn't fall back.
                CompletedRadioTransition::Fallback(..) => unreachable!(),
            };

        let next_task_ifs = Ifs::from_mpdu_length(tx_radio_frame.sdu_length().get());
        // TODO: Currently we use an arbitrary reception window tolerance.
        //       This must be tuned based on actual performance measurements.
        const MAX_ACK_FRAME_START_DELAY: LocalClockDuration = LocalClockDuration::micros(10);
        let latest_ack_start = earliest_ack_start + MAX_ACK_FRAME_START_DELAY;
        let (stop_listening_result, completing_rx_driver) = match listening_rx_driver
            .stop_listening(Some(latest_ack_start))
            .await
        {
            Ok(result) => result,
            Err((_, listening_rx_driver)) => match listening_rx_driver.stop_listening(None).await {
                Ok(result) => result,
                Err(_) => unreachable!(),
            },
        };
        if matches!(stop_listening_result, StopListeningResult::FrameStarted(_)) {
            // Receive and validate the incoming frame.
            self.receive_frame(
                completing_rx_driver,
                Some((tx_radio_frame, tx_timestamp, seq_nr)),
                tx_task_response_token,
                Some(next_task_ifs),
            )
            .await
        } else {
            // Timeout
            let next_request = self
                .request_receiver
                .try_receive_request(&TaskDirection::Any);
            self.end_rx_window(
                completing_rx_driver,
                tx_task_response_token,
                Some((tx_radio_frame, tx_timestamp)),
                next_request,
            )
            .await
        }
    }

    /// Waits for the next request to arrive and then schedules it.
    ///
    /// Returns the driver in the requested driver state together with the
    /// corresponding response token.
    async fn schedule_next_request(
        &self,
        mut off_driver: RadioDriver<RadioDriverImpl, RadioTaskOff>,
        consumer_token: &mut ConsumerToken,
    ) -> (DriverState<RadioDriverImpl>, ResponseToken) {
        loop {
            let (next_response_token, next_request) = self
                .request_receiver
                .receive_request_async(consumer_token, &TaskDirection::Any)
                .await;
            match next_request {
                DrvSvcRequest::Tx(tx_task) => {
                    let tx_task_ack_seq_nr = tx_task.radio_frame.ack_seq_num();
                    let tx_task_ifs = Ifs::from_mpdu_length(tx_task.radio_frame.sdu_length().get());
                    let radio_tx_task = RadioTaskTx {
                        radio_frame: tx_task.radio_frame,
                        cca: tx_task.cca,
                    };
                    let at = if let Timestamp::Scheduled(at) = tx_task.at {
                        Some(at)
                    } else {
                        None
                    };
                    match off_driver
                        .schedule_tx(radio_tx_task, at)
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            this_state: tx_driver,
                            ..
                        }) => {
                            break (
                                DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                                next_response_token,
                            );
                        }
                        CompletedRadioTransition::Fallback(transition_result, tx_task_error) => {
                            // Send back the result of the failed transition.
                            self.request_receiver
                                .received(next_response_token, tx_task_error.into());

                            // Wait for the next request.
                            off_driver = transition_result.this_state;
                            continue;
                        }
                        // Safety: The off task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                    }
                }
                DrvSvcRequest::Rx(rx_task) => {
                    let radio_rx_task = RadioTaskRx {
                        radio_frame: rx_task.radio_frame,
                    };
                    let start = if let Timestamp::Scheduled(at) = rx_task.start {
                        Some(at)
                    } else {
                        None
                    };
                    match off_driver
                        .schedule_rx(radio_rx_task, start)
                        .complete_and_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(RadioTransitionResult {
                            this_state: listening_rx_driver,
                            ..
                        }) => {
                            break (DriverState::Rx(listening_rx_driver), next_response_token);
                        }
                        // Safety: The off task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling an rx task doesn't fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            }
        }
    }
}
