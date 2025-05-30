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
    constants::MAC_AIFS,
    frame::{
        is_frame_valid_and_for_us, RadioFrame, RadioFrameRepr, RadioFrameSized, RadioFrameUnsized,
    },
    tasks::{
        CompletedRadioTransition, ExternalRadioTransition, Ifs, OffResult, OffState, RadioDriver,
        RadioTask, RadioTaskError, RxError, RxResult, RxState, SelfRadioTransition,
        TaskOff as RadioTaskOff, TaskRx as RadioTaskRx, TaskTx as RadioTaskTx, Timestamp, TxResult,
        TxState,
    },
    time::{timer_frequency, Duration, SymbolsOQpsk250kB},
};

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

impl<Task: RadioTask> DriverServiceTask for Task {
    type Result = Task::Result;
    type Error = Task::Error;
}

pub type DrvSvcTaskOff = RadioTaskOff;
pub type DrvSvcTaskRx = RadioTaskRx;
pub type DrvSvcTaskTx = RadioTaskTx;

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
    /// Any interaction with the radio may fail and clients will have to deal
    /// with this.
    RadioError,

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
    Off(DrvSvcTaskResult<DrvSvcTaskOff>),
    Tx(DrvSvcTaskResult<DrvSvcTaskTx>),
    Rx(DrvSvcTaskResult<DrvSvcTaskRx>),
}

impl From<OffResult> for DrvSvcResponse {
    fn from(value: OffResult) -> Self {
        DrvSvcResponse::Off(Ok(value))
    }
}

impl From<RadioTaskError<RadioTaskOff>> for DrvSvcResponse {
    fn from(value: RadioTaskError<RadioTaskOff>) -> Self {
        match value {
            RadioTaskError::Scheduling(_) => DrvSvcResponse::Off(Err(DrvSvcTaskError::RadioError)),
            RadioTaskError::Task(off_error) => {
                DrvSvcResponse::Off(Err(DrvSvcTaskError::Task(off_error)))
            }
        }
    }
}

impl From<TxResult> for DrvSvcResponse {
    fn from(value: TxResult) -> Self {
        DrvSvcResponse::Tx(Ok(value))
    }
}

impl From<RadioTaskError<RadioTaskTx>> for DrvSvcResponse {
    fn from(value: RadioTaskError<RadioTaskTx>) -> Self {
        match value {
            RadioTaskError::Scheduling(_) => DrvSvcResponse::Tx(Err(DrvSvcTaskError::RadioError)),
            RadioTaskError::Task(tx_error) => {
                DrvSvcResponse::Tx(Err(DrvSvcTaskError::Task(tx_error)))
            }
        }
    }
}

impl From<RxResult> for DrvSvcResponse {
    fn from(value: RxResult) -> Self {
        DrvSvcResponse::Rx(Ok(value))
    }
}

impl From<RadioTaskError<RadioTaskRx>> for DrvSvcResponse {
    fn from(value: RadioTaskError<RadioTaskRx>) -> Self {
        match value {
            RadioTaskError::Scheduling(_) => DrvSvcResponse::Rx(Err(DrvSvcTaskError::RadioError)),
            RadioTaskError::Task(rx_error) => {
                DrvSvcResponse::Rx(Err(DrvSvcTaskError::Task(rx_error)))
            }
        }
    }
}

// TODO: Make channel capacities configurable.
pub const DRIVER_CHANNEL_CAPACITY: usize = 4;
const DRIVER_CHANNEL_BACKLOG: usize = 1;

/// We need distinct receivers for cancellable tasks (currently only Rx) and
/// cancelling tasks (currently only Tx).
const DRIVER_CHANNEL_CONSUMERS: usize = 2;

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
    DRIVER_CHANNEL_CONSUMERS,
>;
pub type DriverRequestReceiver<'channel> = Receiver<
    'channel,
    TaskDirection,
    DrvSvcRequest,
    DrvSvcResponse,
    DRIVER_CHANNEL_CAPACITY,
    DRIVER_CHANNEL_BACKLOG,
    DRIVER_CHANNEL_CONSUMERS,
>;
pub type DriverRequestSender<'channel> = Sender<
    'channel,
    TaskDirection,
    DrvSvcRequest,
    DrvSvcResponse,
    DRIVER_CHANNEL_CAPACITY,
    DRIVER_CHANNEL_BACKLOG,
    DRIVER_CHANNEL_CONSUMERS,
>;

/// We use this runtime state to prove that the radio can only be in three
/// different states when looping. This allows us to implement the scheduler as
/// an event loop while still reaping all benefits of a behaviorally typed radio
/// driver.
enum DriverState<RadioDriverImpl> {
    /// We are currently sending a frame.
    Tx(
        RadioDriver<RadioDriverImpl, RadioTaskTx>,
        /// The sequence number of the frame if the outgoing frame requested ACK,
        /// otherwise None.
        Option<u8>,
        /// The IFS is determined by the length of the packet currently being
        /// sent.
        Ifs,
    ),
    /// There is no Tx frame pending and we have Rx capacity to receive an
    /// incoming frame.
    Rx(RadioDriver<RadioDriverImpl, RadioTaskRx>),
    /// We have no Rx capacity and no Tx frame is pending.
    Off(RadioDriver<RadioDriverImpl, RadioTaskOff>),
}

/// Structure managing a given driver implementation. Knows about and manages
/// individual driver capabilities and exposes a unified API to the MAC service.
pub struct DriverService<'svc, RadioDriverImpl> {
    /// The current radio driver state.
    driver_state: Cell<Option<DriverState<RadioDriverImpl>>>,

    /// Receiver for driver service tasks.
    request_receiver: DriverRequestReceiver<'svc>,

    // Pre-allocated TX ACK frame.
    tx_ack_frame: Cell<Option<RadioFrame<RadioFrameSized>>>,

    // Pre-allocated frame for RX ACK and invalid frame buffering.
    temporary_rx_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,
}

impl<'svc, RadioDriverImpl: DriverConfig> DriverService<'svc, RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, RadioTaskOff>: OffState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskRx>: RxState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskTx>: TxState<RadioDriverImpl> + RadioDriverApi,
{
    /// IFS starts after the reception of the last symbol of the previous PPDU
    /// (END event) and ends with the first symbol of the next PPDU, i.e. the
    /// first symbol of the SHR's preamble in the case of the O-QPSK PHY.
    ///
    /// The radio driver polls for the FRAMESTART event which is emitted after
    /// receiving the SHR and PHY header (PHR) when the last symbol of the PHR
    /// has been received.
    ///
    /// The SHR consists of 8 symbols preamble and 1 byte SFD (2 symbols). The
    /// PHR is 1 byte (2 symbols).
    ///
    /// The ACK timeout counting from the END event until the FRAMESTART event
    /// therefore consists of:
    /// t_ACK = 12 symbols (MAC_AIFS) + 10 symbols (SHR) + 2 symbols (PHR)
    ///       = 24 symbols = 384µs.
    const DRIVER_RX_ACK_TIMEOUT: Duration<RadioDriverImpl::Timer> = {
        // Note: We cannot use addition of durations as they are non-const.
        assert!(MAC_AIFS.frequency() == timer_frequency::<SymbolsOQpsk250kB>());
        Duration::<SymbolsOQpsk250kB>::new(MAC_AIFS.ticks() + 10 + 2).convert_into_rounding_up()
    };

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
            tx_ack_frame: Cell::new(Some(Self::allocate_tx_ack_frame(buffer_allocator))),
            temporary_rx_frame: Cell::new(Some(Self::allocate_temporary_rx_frame(
                buffer_allocator,
            ))),
        }
    }

    /// Pre-allocates and pre-populates a re-usable outgoing ACK frame.
    ///
    /// Safety: We have separate incoming and outgoing ACK buffers to ensure
    ///         that incoming ACKs cannot corrupt the pre-populated outgoing ACK
    ///         buffer. This allows us to re-use the outgoing ACK buffer w/o
    ///         validation.
    fn allocate_tx_ack_frame(buffer_allocator: MacBufferAllocator) -> RadioFrame<RadioFrameSized> {
        let radio_frame_repr = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new();
        let tx_ack_buffer_size = ACK_MPDU_SIZE_WO_FCS as usize
            + (radio_frame_repr.fcs_length() + radio_frame_repr.driver_overhead()) as usize;

        imm_ack_frame::<RadioDriverImpl>(
            0,
            buffer_allocator
                .try_allocate_buffer(tx_ack_buffer_size)
                .expect("no capacity"),
        )
        .into_radio_frame::<RadioDriverImpl>()
    }

    /// Pre-allocates a re-usable RX frame for ACK or invalid frame buffering.
    fn allocate_temporary_rx_frame(
        buffer_allocator: MacBufferAllocator,
    ) -> RadioFrame<RadioFrameUnsized> {
        let rx_buffer_size = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new()
            .max_buffer_length() as usize;
        RadioFrame::new::<RadioDriverImpl>(
            buffer_allocator
                .try_allocate_buffer(rx_buffer_size)
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
                DriverState::Rx(rx_driver) => {
                    debug_assert!(current_task_response_token.is_some());
                    self.try_receive_frame(
                        rx_driver,
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

    /// Waits for an incoming frame and receive it or end the Rx window when an
    /// outbound request is received - whatever happens first. Finally switch to
    /// the next requested driver state (if any) or turns the radio off.
    ///
    /// If a request was scheduled: Returns the driver in the requested driver
    /// state together with the corresponding response token.
    ///
    /// If the radio was turned off: Returns the driver in the off state and no
    /// response token.
    async fn try_receive_frame(
        &self,
        mut rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        rx_task_response_token: ResponseToken,
        consumer_token: &mut ConsumerToken,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Wait until a frame is being received or the next outbound request
        // ends the Rx window.
        match select(
            rx_driver.frame_started(),
            self.request_receiver
                .wait_for_request(consumer_token, &TaskDirection::Outbound),
        )
        .await
        {
            // The radio started receiving a frame.
            Either::First(_) => {
                let hardware_address = rx_driver.ieee802154_address();
                let preliminary_frame_info = rx_driver.preliminary_frame_info().await;
                let ifs = Ifs::from_mpdu_length(preliminary_frame_info.mpdu_length);
                let frame_is_valid =
                    is_frame_valid_and_for_us(&hardware_address, &preliminary_frame_info);

                // If the frame is valid and ACK is requested, then
                // schedule a TX ACK task. Otherwise finalize the Rx
                // task and receive the next task (if any).
                if frame_is_valid {
                    // Safety: Valid frames always have a frame control field.
                    let ack_request = preliminary_frame_info.frame_control.unwrap().ack_request();
                    let seq_nr = preliminary_frame_info.seq_nr;
                    if ack_request && seq_nr.is_some() {
                        self.send_ack(rx_driver, rx_task_response_token, seq_nr.unwrap(), ifs)
                            .await
                    } else {
                        self.receive_frame(rx_driver, None, rx_task_response_token, ifs)
                            .await
                    }
                } else {
                    self.drop_invalid_frame(rx_driver, rx_task_response_token)
                        .await
                }
            }
            // We received an outbound request.
            Either::Second(tx_request) => {
                self.end_rx_window(rx_driver, rx_task_response_token, None, Some(tx_request))
                    .await
            }
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
    async fn send_ack(
        &self,
        rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        rx_task_response_token: ResponseToken,
        ack_seq_nr: u8,
        next_task_ifs: Ifs,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: We use the TX ACK frame sequentially and exclusively from
        //         this method.
        let tx_ack_frame = self.tx_ack_frame.take().unwrap();

        let mut tx_ack_mpdu = MpduFrame::from_radio_frame(tx_ack_frame);
        let _ = tx_ack_mpdu.set_sequence_number(ack_seq_nr);
        let tx_ack_frame = tx_ack_mpdu.into_radio_frame::<RadioDriverImpl>();

        let tx_ack_task = RadioTaskTx {
            at: Timestamp::BestEffort,
            radio_frame: tx_ack_frame,
            cca: false,
        };

        match rx_driver
            .schedule_tx(tx_ack_task, Ifs::Aifs, true)
            .execute_transition()
            .await
        {
            // CRC ok: Send the received frame back to the client and update the
            //         driver state.
            CompletedRadioTransition::Entered(transition_result) => {
                let rx_task_result = transition_result.prev_task_result;
                self.request_receiver
                    .received(rx_task_response_token, rx_task_result.into());

                let driver_tx = transition_result.this_state;
                return self.send_frame(driver_tx, None, None, next_task_ifs).await;
            }
            // CRC mismatch: Cancel ACK, recover the pre-allocated ACK frame and
            //               leave the driver in the RX state.
            CompletedRadioTransition::Rollback(
                rx_driver,
                rx_task_error,
                rx_task_result,
                recovered_tx_ack_task,
            ) => {
                debug_assert!(matches!(
                    rx_task_error,
                    RadioTaskError::Task(RxError::CrcError)
                ));
                debug_assert!(rx_task_result.is_none());

                self.tx_ack_frame
                    .set(Some(recovered_tx_ack_task.radio_frame));

                (DriverState::Rx(rx_driver), Some(rx_task_response_token))
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
        rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        rx_ack_info: Option<(RadioFrame<RadioFrameSized>, u8)>,
        prev_task_response_token: ResponseToken,
        next_task_ifs: Ifs,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        fn handle_rx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            prev_task_response_token: ResponseToken,
            rx_task_result: RxResult,
            rx_ack_info: Option<(RadioFrame<RadioFrameSized>, u8)>,
        ) {
            if let Some((tx_radio_frame, rx_task_ack_seq_nr)) = rx_ack_info {
                // Expect RX ACK frame
                let (tx_result, recovered_rx_frame) = match rx_task_result {
                    RxResult::Frame(rx_ack_frame) => {
                        // TODO: Support enhanced ACK.
                        const ACK_FC_MASK: u16 = !0x1000; // Frame version 2003 or 2006
                        const ACK_FC: u16 = 0x0002; // Frame type ACK, other flags all zero
                        let ack = if rx_ack_frame.sdu_wo_fcs_length().get() == 3 {
                            let sdu = rx_ack_frame.sdu_ref();
                            let fc = u16::from_le_bytes([sdu[0], sdu[1]]) & ACK_FC_MASK;
                            fc == ACK_FC && sdu[2] == rx_task_ack_seq_nr
                        } else {
                            false
                        };
                        let tx_result = if ack {
                            TxResult::Sent(tx_radio_frame)
                        } else {
                            TxResult::Nack(tx_radio_frame)
                        };
                        (tx_result, rx_ack_frame.forget_size::<RadioDriverImpl>())
                    }
                    RxResult::FilteredFrame(recovered_rx_frame) => {
                        let recovered_rx_frame =
                            recovered_rx_frame.forget_size::<RadioDriverImpl>();
                        (TxResult::Nack(tx_radio_frame), recovered_rx_frame)
                    }
                    RxResult::CrcError(recovered_rx_frame) => {
                        (TxResult::Nack(tx_radio_frame), recovered_rx_frame)
                    }
                    RxResult::RxWindowEnded(_) => unreachable!(),
                };
                this.request_receiver
                    .received(prev_task_response_token, tx_result.into());
                this.temporary_rx_frame.set(Some(recovered_rx_frame));
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
                    match rx_driver
                        .schedule_tx(tx_task, next_task_ifs, false)
                        .execute_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let rx_task_result = transition_result.prev_task_result;
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            let tx_driver = transition_result.this_state;
                            (
                                DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                                Some(next_response_token),
                            )
                        }
                        CompletedRadioTransition::Fallback(transition_result, tx_task_error) => {
                            let rx_task_result = transition_result.prev_task_result;
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            self.request_receiver
                                .received(next_response_token, tx_task_error.into());

                            let off_driver = transition_result.this_state;
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
                    // scheduling RX back-to-back is ok.
                    match rx_driver
                        .schedule_rx(rx_task, false)
                        .execute_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let rx_task_result = transition_result.prev_task_result;
                            handle_rx_task_result(
                                self,
                                prev_task_response_token,
                                rx_task_result,
                                rx_ack_info,
                            );

                            let rx_driver = transition_result.this_state;
                            (DriverState::Rx(rx_driver), Some(next_response_token))
                        }
                        // Safety: The transition task was programmed to not
                        //         roll back on CRC error.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling RX cannot fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            },
            None => match rx_driver
                .schedule_off(
                    RadioTaskOff {
                        at: Timestamp::BestEffort,
                    },
                    true,
                )
                .execute_transition()
                .await
            {
                CompletedRadioTransition::Entered(transition_result) => {
                    let rx_task_result = transition_result.prev_task_result;
                    handle_rx_task_result(
                        self,
                        prev_task_response_token,
                        rx_task_result,
                        rx_ack_info,
                    );

                    let off_driver = transition_result.this_state;
                    (DriverState::Off(off_driver), None)
                }
                CompletedRadioTransition::Rollback(
                    recovered_rx_driver,
                    rx_task_error,
                    rx_task_result,
                    .., // It is safe to drop the off task.
                ) => {
                    debug_assert!(matches!(
                        rx_task_error,
                        RadioTaskError::Task(RxError::CrcError)
                    ));
                    debug_assert!(rx_task_result.is_none());

                    // We rolled back to the previous Rx task
                    (
                        DriverState::Rx(recovered_rx_driver),
                        Some(prev_task_response_token),
                    )
                }
                // Safety: Switching the radio off is infallible.
                CompletedRadioTransition::Fallback(..) => unreachable!(),
            },
        }
    }

    /// Schedules RX into a temporary buffer back-to-back while finalizing
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
        rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        rx_task_response_token: ResponseToken,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: The temporary RX frame will be recovered by the end of the
        //         procedure.
        let temporary_rx_frame = self.temporary_rx_frame.take().unwrap();
        let rx_task = RadioTaskRx {
            start: Timestamp::BestEffort,
            radio_frame: temporary_rx_frame,
        };
        match rx_driver
            .schedule_rx(rx_task, false)
            .execute_transition()
            .await
        {
            CompletedRadioTransition::Entered(transition_result) => {
                let rx_task_result = transition_result.prev_task_result;
                let recovered_rx_frame = match rx_task_result {
                    RxResult::Frame(invalid_frame) | RxResult::FilteredFrame(invalid_frame) => {
                        invalid_frame.forget_size::<RadioDriverImpl>()
                    }
                    RxResult::RxWindowEnded(recovered_rx_frame)
                    | RxResult::CrcError(recovered_rx_frame) => recovered_rx_frame,
                };

                // Safety: Unsized frames (aka RX frames) for the same driver
                //         are always capable to accommodate the max SDU length,
                //         so they are interchangeable.
                self.temporary_rx_frame.set(Some(recovered_rx_frame));

                let rx_driver = transition_result.this_state;
                (DriverState::Rx(rx_driver), Some(rx_task_response_token))
            }
            // Safety: The transition task was programmed to not roll back on
            //         CRC error.
            CompletedRadioTransition::Rollback(..) => unreachable!(),
            // Safety: Scheduling RX tasks does not fall back.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        }
    }

    /// Ends the ongoing RX window by scheduling the next request and responding
    /// to the previous request.
    ///
    /// If the previous request was a TX request: We end up here because ACK
    /// reception timed out and the ACK reception window needs to be ended. The
    /// TX request will be nack'ed by this method and the next request
    /// scheduled.
    ///
    /// If the previous request was an RX request: We received a concurrent TX
    /// request that needs to make progress. The previous RX request will be
    /// ended without receiving a frame and the TX request scheduled.
    async fn end_rx_window(
        &self,
        rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
        prev_task_response_token: ResponseToken,
        rx_ack_info: Option<RadioFrame<RadioFrameSized>>,
        next_request: Option<(ResponseToken, DrvSvcRequest)>,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        fn handle_rx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            rx_task_result: RxResult,
            rx_ack_info: Option<RadioFrame<RadioFrameSized>>,
            prev_task_response_token: ResponseToken,
        ) {
            // It is improbable but possible that an inbound frame arrives just
            // as we try ending the RX window. We drop the incoming frame in
            // this case as if we had ended the RX window slightly earlier.
            //
            // Note: Well timed protocols should not experience this situation.
            let rx_radio_frame = match rx_task_result {
                RxResult::Frame(radio_frame) | RxResult::FilteredFrame(radio_frame) => {
                    radio_frame.forget_size::<RadioDriverImpl>()
                }
                RxResult::RxWindowEnded(radio_frame) | RxResult::CrcError(radio_frame) => {
                    radio_frame
                }
            };

            if let Some(tx_radio_frame) = rx_ack_info {
                // End RX ACK window
                this.temporary_rx_frame.set(Some(rx_radio_frame));
                let tx_task_result = TxResult::Nack(tx_radio_frame);
                this.request_receiver
                    .received(prev_task_response_token, tx_task_result.into());
            } else {
                // End regular RX window
                let rx_task_result = RxResult::RxWindowEnded(rx_radio_frame);
                this.request_receiver
                    .received(prev_task_response_token, rx_task_result.into());
            }
        }

        match next_request {
            Some((tx_task_response_token, DrvSvcRequest::Tx(tx_task))) => {
                let tx_task_ack_seq_nr = tx_task.radio_frame.ack_seq_num();
                let tx_task_ifs = Ifs::from_mpdu_length(tx_task.radio_frame.sdu_length().get());
                match rx_driver
                    .schedule_tx(tx_task, Ifs::None, false)
                    .execute_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(transition_result) => {
                        let rx_task_result = transition_result.prev_task_result;
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        let tx_driver = transition_result.this_state;
                        (
                            DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                            Some(tx_task_response_token),
                        )
                    }
                    // Fallback to "off" state due to CCA busy when trying to schedule
                    // the Tx task.
                    CompletedRadioTransition::Fallback(transition_result, tx_task_error) => {
                        let rx_task_result = transition_result.prev_task_result;
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        // Report CCA busy as result of the tx task.
                        self.request_receiver
                            .received(tx_task_response_token, tx_task_error.into());

                        let off_driver = transition_result.this_state;
                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: The transition was programmed not to roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                }
            }
            Some((rx_task_response_token, DrvSvcRequest::Rx(rx_task))) => {
                let tx_task_result = if let Some(tx_radio_frame) = rx_ack_info {
                    TxResult::Nack(tx_radio_frame)
                } else {
                    // Safety: We only ever end an RX window with another RX task
                    //         after an RX ACK window timed out.
                    unreachable!()
                };

                // Continue the ongoing reception and recover the temporary
                // frame from the incoming RX task instead.
                self.temporary_rx_frame.set(Some(rx_task.radio_frame));

                self.request_receiver
                    .received(prev_task_response_token, tx_task_result.into());
                (DriverState::Rx(rx_driver), Some(rx_task_response_token))
            }
            None => {
                let off_task = RadioTaskOff {
                    at: Timestamp::BestEffort,
                };
                match rx_driver
                    .schedule_off(off_task, false)
                    .execute_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(transition_result) => {
                        let rx_task_result = transition_result.prev_task_result;
                        handle_rx_task_result::<RadioDriverImpl>(
                            self,
                            rx_task_result,
                            rx_ack_info,
                            prev_task_response_token,
                        );

                        let off_driver = transition_result.this_state;
                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: Switching the driver off from an RX state
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
        ack_seq_nr: Option<u8>,
        next_task_ifs: Ifs,
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
                match tx_task_result {
                    TxResult::Sent(radio_frame) => {
                        this.tx_ack_frame.set(Some(radio_frame));
                    }
                    // Safety: Ack frames don't ask for ACK.
                    TxResult::Nack(_) => unreachable!(),
                }
            }
        }

        if let Some(ack_seq_nr) = ack_seq_nr {
            // Safety: Only regular TX tasks can request acknowledgement and
            //         therefore a response token is expected.
            return self
                .wait_for_ack(tx_driver, tx_task_response_token.unwrap(), ack_seq_nr)
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
                    match tx_driver
                        .schedule_tx(tx_task, next_task_ifs)
                        .execute_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let tx_task_result = transition_result.prev_task_result;
                            handle_tx_task_result(
                                self,
                                tx_task_response_token,
                                tx_task_result,
                                ack_seq_nr,
                            )
                            .await;

                            let tx_driver = transition_result.this_state;
                            (
                                DriverState::Tx(tx_driver, tx_task_ack_seq_nr, tx_task_ifs),
                                Some(next_response_token),
                            )
                        }
                        CompletedRadioTransition::Fallback(transition_result, tx_task_error) => {
                            let tx_task_result = transition_result.prev_task_result;

                            if let Some(tx_task_response_token) = tx_task_response_token {
                                // External request: send back the result.
                                self.request_receiver
                                    .received(tx_task_response_token, tx_task_result.into());
                            } else {
                                // Tx ACK: recover the pre-allocated ACK frame.
                                match tx_task_result {
                                    TxResult::Sent(radio_frame) => {
                                        self.tx_ack_frame.set(Some(radio_frame));
                                    }
                                    // Safety: Ack frames don't ask for ACK.
                                    TxResult::Nack(_) => unreachable!(),
                                }
                            }

                            // Send back the result of the failed transition.
                            self.request_receiver
                                .received(next_response_token, tx_task_error.into());

                            let off_driver = transition_result.this_state;
                            (DriverState::Off(off_driver), None)
                        }
                        // Safety: The TX task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                    }
                }
                DrvSvcRequest::Rx(rx_task) => {
                    match tx_driver
                        .schedule_rx(rx_task, next_task_ifs)
                        .execute_transition()
                        .await
                    {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let tx_task_result = transition_result.prev_task_result;
                            handle_tx_task_result(
                                self,
                                tx_task_response_token,
                                tx_task_result,
                                ack_seq_nr,
                            )
                            .await;

                            let rx_driver = transition_result.this_state;
                            (DriverState::Rx(rx_driver), Some(next_response_token))
                        }
                        // Safety: The TX task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling an RX task doesn't fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            },
            None => {
                match tx_driver
                    .schedule_off(RadioTaskOff {
                        at: Timestamp::BestEffort,
                    })
                    .execute_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(transition_result) => {
                        let tx_task_result = transition_result.prev_task_result;
                        handle_tx_task_result(
                            self,
                            tx_task_response_token,
                            tx_task_result,
                            ack_seq_nr,
                        )
                        .await;

                        let off_driver = transition_result.this_state;
                        (DriverState::Off(off_driver), None)
                    }
                    // Safety: Switching the driver off from a TX state should
                    //         be infallible.
                    _ => unreachable!(),
                }
            }
        }
    }

    /// Waits for an incoming ACK frame matching the given sequence number and
    /// responds to the TX task accordingly.
    async fn wait_for_ack(
        &self,
        tx_driver: RadioDriver<RadioDriverImpl, RadioTaskTx>,
        tx_task_response_token: ResponseToken,
        ack_seq_nr: u8,
    ) -> (DriverState<RadioDriverImpl>, Option<ResponseToken>) {
        // Safety: The temporary frame is always recovered before being re-used.
        let rx_ack_frame = self.temporary_rx_frame.take().unwrap();
        let rx_ack_task = RadioTaskRx {
            start: Timestamp::BestEffort,
            radio_frame: rx_ack_frame,
        };
        let (mut rx_driver, tx_radio_frame) = match tx_driver
            .schedule_rx(rx_ack_task, Ifs::Aifs)
            .execute_transition()
            .await
        {
            CompletedRadioTransition::Entered(transition_result) => {
                let tx_task_result = transition_result.prev_task_result;
                let tx_radio_frame = match tx_task_result {
                    TxResult::Sent(tx_radio_frame) => tx_radio_frame,
                    TxResult::Nack(_) => unreachable!(),
                };
                let rx_driver = transition_result.this_state;
                (rx_driver, tx_radio_frame)
            }
            // Safety: The TX task doesn't roll back.
            CompletedRadioTransition::Rollback(..) => unreachable!(),
            // Safety: The RX task doesn't fall back.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        };

        // Note: This is just a rough estimate with some safety margin for now.
        //       Precise timing requires timestamp and RX window support in the
        //       driver.
        let timeout = RadioDriverImpl::Timer::wait_for_alarm_at(
            RadioDriverImpl::Timer::now() + Self::DRIVER_RX_ACK_TIMEOUT,
        );

        let next_task_ifs = Ifs::from_mpdu_length(tx_radio_frame.sdu_length().get());
        match select(rx_driver.frame_started(), timeout).await {
            Either::First(_) => {
                // Receive and validate the incoming frame.
                self.receive_frame(
                    rx_driver,
                    Some((tx_radio_frame, ack_seq_nr)),
                    tx_task_response_token,
                    next_task_ifs,
                )
                .await
            }
            Either::Second(_) => {
                // Timeout
                let next_request = self
                    .request_receiver
                    .try_receive_request(&TaskDirection::Any);
                self.end_rx_window(
                    rx_driver,
                    tx_task_response_token,
                    Some(tx_radio_frame),
                    next_request,
                )
                .await
            }
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
                .wait_for_request(consumer_token, &TaskDirection::Any)
                .await;
            match next_request {
                DrvSvcRequest::Tx(tx_task) => {
                    let tx_task_ack_seq_nr = tx_task.radio_frame.ack_seq_num();
                    let tx_task_ifs = Ifs::from_mpdu_length(tx_task.radio_frame.sdu_length().get());
                    match off_driver.schedule_tx(tx_task).execute_transition().await {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let tx_driver = transition_result.this_state;
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
                        // Safety: The Off task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                    }
                }
                DrvSvcRequest::Rx(rx_task) => {
                    match off_driver.schedule_rx(rx_task).execute_transition().await {
                        CompletedRadioTransition::Entered(transition_result) => {
                            let rx_driver = transition_result.this_state;
                            break (DriverState::Rx(rx_driver), next_response_token);
                        }
                        // Safety: The Off task doesn't roll back.
                        CompletedRadioTransition::Rollback(..) => unreachable!(),
                        // Safety: Scheduling an RX task doesn't fall back.
                        CompletedRadioTransition::Fallback(..) => unreachable!(),
                    }
                }
            }
        }
    }
}
