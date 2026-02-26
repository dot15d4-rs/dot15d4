//! Access to IEEE 802.15.4 radio drivers.
//!
//! This module provides the upper half of the communication pipe towards IEEE
//! 802.15.4 radio drivers.

use core::cell::Cell;

#[cfg(not(feature = "tsch"))]
use crate::mac::frame::mpdu::ACK_MPDU_SIZE_WO_FCS;
use crate::{mac::MacBufferAllocator, utils::is_frame_valid_and_for_us};

#[cfg(feature = "tsch")]
use crate::mac::frame::mpdu::{enh_ack_frame, ENH_ACK_MPDU_SIZE_WO_FCS};

use self::{
    radio::{
        frame::{RadioFrame, RadioFrameRepr, RadioFrameSized, RadioFrameUnsized},
        phy::Ifs,
        tasks::{
            CompletedRadioTransition, ExternalRadioTransition, ListeningRxState, OffState,
            RadioTaskError, RadioTransitionResult, SelfRadioTransition, TaskOff as RadioTaskOff,
            TaskRx as RadioTaskRx, TaskTx as RadioTaskTx, TxError, TxResult, TxState,
        },
        DriverConfig, RadioDriver,
    },
    timer::NsInstant,
};

pub use dot15d4_driver::*;
use dot15d4_driver::{
    radio::{
        config::Channel as PhyChannel,
        frame::FrameVersion,
        tasks::{RadioDriverApi, ReceivingRxState, RxError, RxResult, StopListeningResult},
        PhyOf,
    },
    timer::{NsDuration, OptionalNsInstant},
};
use dot15d4_frame::mpdu::{imm_ack_frame, MpduFrame};
use dot15d4_util::{allocator::IntoBuffer, sync::select};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};

/// Tasks can be scheduled as fast as possible ("best effort") or at a
/// well-defined tick of the local radio clock ("scheduled").
///
/// The timestamp is represented as a [`NsInstant`] in terms of the radio
/// driver's local timer, i.e. the timestamp must already have been compensated
/// for clock drift.
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
    Scheduled(NsInstant),
}

impl From<Timestamp> for OptionalNsInstant {
    fn from(value: Timestamp) -> Self {
        match value {
            Timestamp::BestEffort => None.into(),
            Timestamp::Scheduled(instant) => instant.into(),
        }
    }
}

/// Task: send a single frame
#[derive(Debug, PartialEq, Eq)]
pub struct DrvSvcTaskTx {
    /// the time at which the RMARKER of the outbound frame SHALL pass the local
    /// antenna.
    pub at: Timestamp,

    /// MPDU frame to be sent
    pub mpdu: MpduFrame,

    /// whether CCA is to be performed as a precondition to send out the frame
    pub cca: bool,
    /// New PHY channel used for the transmission. `None` value has no effect
    /// on the channel.
    pub channel: Option<PhyChannel>,
    /// Whether the driver should fallback if request complete with NACK result
    pub fallback_on_nack: bool,
}

/// Task: receive a single frame
#[derive(Debug, PartialEq, Eq)]
pub struct DrvSvcTaskRx {
    /// The earliest time at which a frame with this RMARKER passing the local
    /// antenna SHALL be recognized. The receiver SHALL be switched on as late
    /// as possible to minimize energy consumption.
    pub start: Timestamp,

    /// radio frame allocated to receive incoming frames
    pub radio_frame: RadioFrame<RadioFrameUnsized>,

    /// channel
    pub channel: Option<PhyChannel>,

    /// Optional RX window instant. If set, the receiver will automatically
    /// stop listening after this instant.
    /// If not set, the receiver will listen indefinitely until a frame is
    /// received or a subsequent TX/idle request is sent.
    pub rx_window_end: Option<NsInstant>,
    /// Optional Expected RX framestart instant, i.e. RX RMARKER.
    #[cfg(feature = "tsch")]
    pub expected_rx_framestart: Option<NsInstant>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum DrvSvcRequest {
    CompleteThenStartTx(DrvSvcTaskTx),
    CompleteThenStartRx(DrvSvcTaskRx),
    CompleteThenGoIdle,
}

impl From<DrvSvcTaskTx> for DrvSvcRequest {
    fn from(value: DrvSvcTaskTx) -> Self {
        DrvSvcRequest::CompleteThenStartTx(value)
    }
}

enum TransmissionType {
    Ack(RxResult),
    Frame,
}

enum ReceptionType {
    Ack(
        /// TX frame awaiting ACK
        RadioFrame<RadioFrameSized>,
        // Expected Sequence Number
        u8,
        /// TX frame Timestamp
        NsInstant,
        /// Driver Fallback on NACK
        bool,
    ),
    Frame,
}

pub enum DrvSvcEvent {
    CcaBusy(RadioFrame<RadioFrameSized>, NsInstant),
    TxStarted(NsInstant),
    FrameStarted,
    Nack(
        RadioFrame<RadioFrameSized>,
        NsInstant,
        // On fallback, contains the next request
        Option<DrvSvcRequest>,
        // Time Sync in micro seconds
        Option<i16>,
    ),
    Sent(RadioFrame<RadioFrameSized>, NsInstant, Option<i16>),
    Received(RadioFrame<RadioFrameSized>, NsInstant),
    RxWindowEnded(RadioFrame<RadioFrameUnsized>),
    SchedulingFailed(RadioFrame<RadioFrameSized>),
    CrcError(
        RadioFrame<RadioFrameUnsized>,
        timer::export::Instant<u64, 1, 1000000000>,
    ),
}

pub type DriverRequestChannel = Channel<CriticalSectionRawMutex, DrvSvcRequest, 1>;
pub type DriverRequestSender<'channel> =
    Sender<'channel, CriticalSectionRawMutex, DrvSvcRequest, 1>;
pub type DriverRequestReceiver<'channel> =
    Receiver<'channel, CriticalSectionRawMutex, DrvSvcRequest, 1>;

pub type DriverEventChannel = Channel<CriticalSectionRawMutex, DrvSvcEvent, 1>;
pub type DriverEventSender<'channel> = Sender<'channel, CriticalSectionRawMutex, DrvSvcEvent, 1>;
pub type DriverEventReceiver<'channel> =
    Receiver<'channel, CriticalSectionRawMutex, DrvSvcEvent, 1>;

/// We use this runtime state to prove that the radio can only be in three
/// different states when looping. This allows us to implement the scheduler as
/// an event loop while still reaping all benefits of a behaviorally typed radio
/// driver.
enum DriverState<RadioDriverImpl: DriverConfig> {
    /// We are currently sending a frame.
    SendingFrame(
        RadioDriver<RadioDriverImpl, RadioTaskTx>,
        Ifs<PhyOf<RadioDriverImpl>>,
    ),
    SendingFrameWithAck(
        RadioDriver<RadioDriverImpl, RadioTaskTx>,
        Ifs<PhyOf<RadioDriverImpl>>,
        u8,
        /// Fallback on NACK result
        bool,
    ),
    /// There is no tx frame pending and we have rx capacity to receive an
    /// incoming frame.
    WaitingForFrame(RadioDriver<RadioDriverImpl, RadioTaskRx>),
    /// We have no rx capacity and no tx frame is pending.
    Idle(RadioDriver<RadioDriverImpl, RadioTaskOff>),
}

/// Structure managing a given driver implementation. Knows about and manages
/// individual driver capabilities and exposes a unified API to the MAC service.
pub struct DriverService<'svc, RadioDriverImpl: DriverConfig> {
    /// The current radio driver state.
    driver_state: Cell<Option<DriverState<RadioDriverImpl>>>,

    /// Receiver for driver service tasks.
    request_receiver: DriverRequestReceiver<'svc>,

    /// Sender for driver service response stream.
    event_sender: DriverEventSender<'svc>,

    // Pre-allocated frame for outbound acknowledgements.
    outbound_ack_frame: Cell<Option<RadioFrame<RadioFrameSized>>>,

    // Pre-allocated frame for inbound acknowledgements and invalid frame
    // buffering.
    temp_inbound_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,

    // Pending Request in case a Tx request handling gets delayed by RX frame
    // arriving just before transitioning from Rx to Tx.
    pending_request: Cell<Option<DrvSvcRequest>>,

    // RX window end time. When set, the receiver will stop listening at this
    // instant if no frame has started.
    rx_window_end: Cell<Option<NsInstant>>,

    // Expected RX frame start (i.e. RMARKER) of scheduled RX. When set, used
    // for Time Correction IE when sending Enh-Ack using TSCH.
    #[cfg(feature = "tsch")]
    expected_rx_framestart: Cell<Option<NsInstant>>,
}

impl<'svc, RadioDriverImpl: DriverConfig> DriverService<'svc, RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, RadioTaskOff>: OffState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, RadioTaskRx>: ListeningRxState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, RadioTaskTx>: TxState<RadioDriverImpl>,
{
    /// Creates a new [`DriverService`] instance wrapping the given driver
    /// implementation.
    pub fn new(
        driver: RadioDriver<RadioDriverImpl, RadioTaskOff>,
        driver_request_receiver: DriverRequestReceiver<'svc>,
        driver_response_sender: DriverEventSender<'svc>,
        buffer_allocator: MacBufferAllocator,
    ) -> Self {
        Self {
            driver_state: Cell::new(Some(DriverState::Idle(driver))),
            request_receiver: driver_request_receiver,
            event_sender: driver_response_sender,
            outbound_ack_frame: Cell::new(Some(Self::allocate_outbound_ack_frame(
                buffer_allocator,
            ))),
            temp_inbound_frame: Cell::new(Some(Self::allocate_inbound_frame(buffer_allocator))),
            pending_request: Cell::new(None),
            rx_window_end: Cell::new(None),
            expected_rx_framestart: Cell::new(None),
        }
    }

    /// Pre-allocates and pre-populates a re-usable outbound ACK frame.
    ///
    /// Safety: We have separate incoming and outbound ACK buffers to ensure
    ///         that incoming ACKs cannot corrupt the pre-populated outbound ACK
    ///         buffer. This allows us to re-use the outbound ACK buffer w/o
    ///         validation.
    ///
    /// When the `tsch` feature is enabled, this allocates an Enhanced ACK frame
    /// with Time Correction IE. Otherwise, it allocates a standard ImmAck frame.
    fn allocate_outbound_ack_frame(
        buffer_allocator: MacBufferAllocator,
    ) -> RadioFrame<RadioFrameSized> {
        let radio_frame_repr = RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new();

        #[cfg(feature = "tsch")]
        {
            // Enhanced ACK with Time Correction IE
            let outbound_ack_buffer_size = ENH_ACK_MPDU_SIZE_WO_FCS as usize
                + (radio_frame_repr.fcs_length() + radio_frame_repr.driver_overhead()) as usize;

            enh_ack_frame::<RadioDriverImpl>(
                0,
                buffer_allocator
                    .try_allocate_buffer(outbound_ack_buffer_size)
                    .expect("no capacity"),
            )
            .into_radio_frame::<RadioDriverImpl>()
        }

        #[cfg(not(feature = "tsch"))]
        {
            // Standard ImmAck
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
        let mut driver_state = self.driver_state.take().expect("already running");

        loop {
            driver_state = match driver_state {
                DriverState::SendingFrame(tx_driver, ifs) => {
                    self.complete_transmission(tx_driver, TransmissionType::Frame, ifs)
                        .await
                }
                DriverState::SendingFrameWithAck(tx_driver, ifs, seq_nb, fallback_on_nack) => {
                    self.complete_transmission_with_ack(tx_driver, seq_nb, ifs, fallback_on_nack)
                        .await
                }
                DriverState::WaitingForFrame(listening_rx_driver) => {
                    // TODO: safety
                    self.waiting_for_frame(listening_rx_driver).await
                }
                DriverState::Idle(off_driver) => self.idle(off_driver).await,
            }
        }
    }

    async fn idle(
        &self,
        mut off_driver: RadioDriver<RadioDriverImpl, RadioTaskOff>,
    ) -> DriverState<RadioDriverImpl> {
        loop {
            let request = match self.pending_request.take() {
                Some(request) => request,
                None => self.request_receiver.receive().await,
            };
            match self.schedule_next(off_driver, request).await {
                Ok(new_state) => break new_state,
                Err(fallback_driver) => {
                    off_driver = fallback_driver;
                    // Previous request has been handled, even if transition
                    // failed
                }
            }
        }
    }

    async fn complete_transmission(
        &self,
        tx_driver: RadioDriver<RadioDriverImpl, RadioTaskTx>,
        transmission_type: TransmissionType,
        ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> DriverState<RadioDriverImpl> {
        fn handle_tx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            tx_task_result: TxResult,
            transmission_type: TransmissionType,
        ) -> DrvSvcEvent {
            match transmission_type {
                TransmissionType::Frame => match tx_task_result {
                    TxResult::Sent(radio_frame, instant) => {
                        DrvSvcEvent::Sent(radio_frame, instant, None)
                    }
                },
                TransmissionType::Ack(rx_task_result) => {
                    // Tx ACK: recover the pre-allocated ACK frame.
                    let TxResult::Sent(radio_frame, ..) = tx_task_result;
                    this.outbound_ack_frame.set(Some(radio_frame));
                    match rx_task_result {
                        RxResult::Frame(radio_frame, instant) => {
                            DrvSvcEvent::Received(radio_frame, instant)
                        }
                        _ => unreachable!(),
                    }
                }
            }
        }
        let next_request = match self.pending_request.take() {
            Some(request) => request,
            None => match self.request_receiver.try_receive() {
                Ok(request) => request,
                Err(_) => DrvSvcRequest::CompleteThenGoIdle,
            },
        };

        match next_request {
            DrvSvcRequest::CompleteThenStartTx(task_tx) => {
                let next_ifs = Ifs::from_mpdu_length(task_tx.mpdu.pdu_length::<RadioDriverImpl>());
                let next_ack_seq_nr = task_tx.mpdu.sequence_number();
                let ack_request = task_tx.mpdu.frame_control().ack_request();
                match tx_driver
                    .schedule_tx(
                        RadioTaskTx {
                            radio_frame: task_tx.mpdu.into_radio_frame::<RadioDriverImpl>(),
                            cca: task_tx.cca,
                        },
                        ifs,
                    )
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result,
                        this_state,
                        measured_entry,
                        ..
                    }) => {
                        let previous_event =
                            handle_tx_task_result(self, prev_task_result, transmission_type);
                        self.event_sender.send(previous_event).await;
                        self.event_sender
                            .send(DrvSvcEvent::TxStarted(measured_entry))
                            .await;

                        match next_ack_seq_nr {
                            Some(seq_nb) if ack_request => DriverState::SendingFrameWithAck(
                                this_state,
                                next_ifs,
                                seq_nb,
                                task_tx.fallback_on_nack,
                            ),
                            _ => DriverState::SendingFrame(this_state, next_ifs),
                        }
                    }
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult {
                            this_state,
                            prev_task_result,
                            measured_entry,
                            ..
                        },
                        radio_task_error,
                    ) => {
                        let previous_event =
                            handle_tx_task_result(self, prev_task_result, transmission_type);
                        let event = match radio_task_error {
                            RadioTaskError::Scheduling(radio_task_tx, _) => {
                                DrvSvcEvent::SchedulingFailed(radio_task_tx.radio_frame)
                            }
                            RadioTaskError::Task(tx_error) => match tx_error {
                                TxError::CcaBusy(radio_frame) => {
                                    DrvSvcEvent::CcaBusy(radio_frame, measured_entry)
                                }
                            },
                        };
                        self.event_sender.send(previous_event).await;
                        self.event_sender.send(event).await;

                        DriverState::Idle(this_state)
                    }
                    // Safety: The tx task doesn't roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenStartRx(task_rx) => {
                // TODO: get from previous TX (store in state ?)
                let ifs = Ifs::long();
                let rx_window_end = task_rx.rx_window_end;
                let expected_rx_framestart = task_rx.expected_rx_framestart;
                match tx_driver
                    .schedule_rx(
                        RadioTaskRx {
                            radio_frame: task_rx.radio_frame,
                        },
                        ifs,
                    )
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result,
                        this_state,
                        ..
                    }) => {
                        let event =
                            handle_tx_task_result(self, prev_task_result, transmission_type);
                        self.event_sender.send(event).await;

                        // Set the RX window end time if specified
                        self.rx_window_end.set(rx_window_end);
                        self.expected_rx_framestart.set(expected_rx_framestart);

                        DriverState::WaitingForFrame(this_state)
                    }
                    // Safety: The tx task doesn't roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                    // Safety: Scheduling an rx task doesn't fall back.
                    CompletedRadioTransition::Fallback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenGoIdle => {
                match tx_driver.schedule_off().complete_and_transition().await {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result,
                        this_state: off_driver,
                        ..
                    }) => {
                        let previous_event =
                            handle_tx_task_result(self, prev_task_result, transmission_type);
                        // self.response_sender
                        //     .send(DrvSvcResponse::Entered(DrvSvcTransitionResult {
                        //         prev_task_result,
                        //         measured_entry,
                        //     }))
                        //     .await;
                        self.event_sender.send(previous_event).await;

                        DriverState::Idle(off_driver)
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
    async fn complete_transmission_with_ack(
        &self,
        tx_driver: RadioDriver<RadioDriverImpl, RadioTaskTx>,
        seq_nb: u8,
        ifs: Ifs<PhyOf<RadioDriverImpl>>,
        fallback_on_nack: bool,
    ) -> DriverState<RadioDriverImpl> {
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

        // TODO: Currently we use an arbitrary reception window tolerance.
        //       This must be tuned based on actual performance measurements.
        const MAX_ACK_FRAME_START_DELAY: NsDuration = NsDuration::micros(50);
        let latest_ack_start = earliest_ack_start + MAX_ACK_FRAME_START_DELAY;
        let stop_listening_result = match listening_rx_driver
            .stop_listening(latest_ack_start.into())
            .await
        {
            Ok(result) => result,
            Err((_, listening_rx_driver)) => {
                match listening_rx_driver.stop_listening(None.into()).await {
                    Ok(result) => result,
                    Err(_) => unreachable!(),
                }
            }
        };

        self.pending_request
            .set(self.request_receiver.try_receive().ok());
        match stop_listening_result {
            StopListeningResult::FrameStarted(_, receiving_rx_driver) => {
                // Receive and validate the incoming frame.
                self.complete_reception(
                    receiving_rx_driver,
                    ReceptionType::Ack(tx_radio_frame, seq_nb, tx_timestamp, fallback_on_nack),
                    ifs,
                )
                .await
            }
            StopListeningResult::RxWindowEnded(radio_transition_result) => {
                // Timeout
                self.end_rx_window(
                    radio_transition_result,
                    ReceptionType::Ack(tx_radio_frame, seq_nb, tx_timestamp, fallback_on_nack),
                )
                .await
            }
        }
    }

    async fn complete_reception(
        &self,
        receiving_rx_driver: impl ReceivingRxState<RadioDriverImpl>,
        reception_type: ReceptionType,
        next_ifs: Ifs<PhyOf<RadioDriverImpl>>,
    ) -> DriverState<RadioDriverImpl> {
        fn handle_rx_task_result<RadioDriverImpl: DriverConfig>(
            this: &DriverService<'_, RadioDriverImpl>,
            rx_task_result: RxResult,
            reception_type: ReceptionType,
        ) -> DrvSvcEvent {
            match reception_type {
                ReceptionType::Ack(tx_radio_frame, seq_nb, tx_timestamp, fallback_on_nack) => {
                    // Expect rx ACK frame
                    let (event, recovered_ack_frame) = match rx_task_result {
                        RxResult::Frame(rx_ack_frame, _) => {
                            match validate_ack_frame::<RadioDriverImpl>(rx_ack_frame, seq_nb) {
                                Ok((validated_ack_frame, time_sync_us)) => (
                                    DrvSvcEvent::Sent(tx_radio_frame, tx_timestamp, time_sync_us),
                                    validated_ack_frame,
                                ),
                                Err(invalid_ack_frame) => {
                                    let pending_request = if fallback_on_nack {
                                        this.pending_request.take()
                                    } else {
                                        None
                                    };
                                    (
                                        DrvSvcEvent::Nack(
                                            tx_radio_frame,
                                            tx_timestamp,
                                            pending_request,
                                            None,
                                        ),
                                        invalid_ack_frame,
                                    )
                                }
                            }
                        }
                        RxResult::CrcError(recovered_rx_frame, _) => {
                            let pending_request = if fallback_on_nack {
                                this.pending_request.take()
                            } else {
                                None
                            };
                            (
                                DrvSvcEvent::Nack(
                                    tx_radio_frame,
                                    tx_timestamp,
                                    pending_request,
                                    None,
                                ),
                                recovered_rx_frame,
                            )
                        }
                        RxResult::RxWindowEnded(_) => unreachable!(),
                    };
                    this.temp_inbound_frame.set(Some(recovered_ack_frame));
                    event
                }
                ReceptionType::Frame => match rx_task_result {
                    RxResult::Frame(radio_frame, instant) => {
                        DrvSvcEvent::Received(radio_frame, instant)
                    }
                    RxResult::RxWindowEnded(radio_frame) => DrvSvcEvent::RxWindowEnded(radio_frame),
                    RxResult::CrcError(radio_frame, instant) => {
                        DrvSvcEvent::CrcError(radio_frame, instant)
                    }
                },
            }
        }

        let next_request = match &reception_type {
            // If we need fallback on NACK, we first must schedule off in order to
            // check received ACK. Then, we schedule the next request (which is pending).
            ReceptionType::Ack(.., fallback_on_nack) if *fallback_on_nack => {
                DrvSvcRequest::CompleteThenGoIdle
            }
            // Matches Frame and Ack wo fallback
            _ => match self.pending_request.take() {
                Some(request) => request,
                None => match self.request_receiver.try_receive() {
                    Ok(request) => request,
                    Err(_) => DrvSvcRequest::CompleteThenGoIdle,
                },
            },
        };

        match next_request {
            DrvSvcRequest::CompleteThenStartTx(tx_task) => {
                let seq_nb = tx_task.mpdu.sequence_number();
                let ack_request = tx_task.mpdu.frame_control().ack_request();
                let tx_task_ifs =
                    Ifs::from_mpdu_length(tx_task.mpdu.pdu_length::<RadioDriverImpl>());
                let DrvSvcTaskTx {
                    at,
                    mpdu,
                    cca,
                    channel,
                    fallback_on_nack,
                } = tx_task;
                // Channel setting shall be performed only from off radio state
                assert!(channel.is_none());
                match receiving_rx_driver
                    .schedule_tx(
                        RadioTaskTx {
                            radio_frame: mpdu.into_radio_frame::<RadioDriverImpl>(),
                            cca,
                        },
                        at.into(),
                        Some(next_ifs),
                        false,
                    )
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: rx_task_result,
                        this_state: tx_driver,
                        measured_entry,
                        ..
                    }) => {
                        let previous_event =
                            handle_rx_task_result(self, rx_task_result, reception_type);
                        self.event_sender.send(previous_event).await;
                        self.event_sender
                            .send(DrvSvcEvent::TxStarted(measured_entry))
                            .await;

                        match seq_nb {
                            Some(seq_nb) if ack_request => DriverState::SendingFrameWithAck(
                                tx_driver,
                                tx_task_ifs,
                                seq_nb,
                                fallback_on_nack,
                            ),
                            _ => DriverState::SendingFrame(tx_driver, tx_task_ifs),
                        }
                    }
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult {
                            prev_task_result: rx_task_result,
                            this_state: off_driver,
                            measured_entry,
                            ..
                        },
                        tx_task_error,
                    ) => {
                        let previous_event =
                            handle_rx_task_result(self, rx_task_result, reception_type);

                        let event = match tx_task_error {
                            RadioTaskError::Scheduling(task_rx, _) => {
                                DrvSvcEvent::SchedulingFailed(task_rx.radio_frame)
                            }
                            RadioTaskError::Task(TxError::CcaBusy(radio_frame)) => {
                                DrvSvcEvent::CcaBusy(radio_frame, measured_entry)
                            }
                        };

                        self.event_sender.send(previous_event).await;
                        self.event_sender.send(event).await;

                        DriverState::Idle(off_driver)
                    }
                    // Safety: The transition was programmed to not roll
                    //         back on CRC error.
                    // TODO: support rollback
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenStartRx(rx_task) => {
                // We're already receiving another request and are
                // therefore guaranteed to make progress. Therefore
                // scheduling rx back-to-back is ok.
                let DrvSvcTaskRx {
                    start,
                    radio_frame,
                    channel,
                    rx_window_end,
                    #[cfg(feature = "tsch")]
                    expected_rx_framestart,
                } = rx_task;

                // Frames may only be received back-to-back with best-effort
                // timing.
                assert!(matches!(start, Timestamp::BestEffort));

                // Channel must be the same since the radio must go off to set channel
                assert!(channel.is_none());

                match receiving_rx_driver
                    .schedule_rx(RadioTaskRx { radio_frame }, Some(next_ifs), false)
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        prev_task_result: rx_task_result,
                        this_state: listening_rx_driver,
                        ..
                    }) => {
                        let event = handle_rx_task_result(self, rx_task_result, reception_type);

                        self.event_sender.send(event).await;

                        // Set the RX window end time if specified
                        self.rx_window_end.set(rx_window_end);
                        self.expected_rx_framestart.set(expected_rx_framestart);

                        DriverState::WaitingForFrame(listening_rx_driver)
                    }
                    // Safety: The transition task was programmed to not
                    //         roll back on CRC error.
                    // TODO: support rollback
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                    // Safety: Scheduling rx cannot fall back.
                    CompletedRadioTransition::Fallback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenGoIdle => match receiving_rx_driver
                .schedule_off(None.into(), false)
                .complete_and_transition()
                .await
            {
                CompletedRadioTransition::Entered(RadioTransitionResult {
                    prev_task_result: rx_task_result,
                    this_state: off_driver,
                    ..
                }) => {
                    let event = handle_rx_task_result(self, rx_task_result, reception_type);
                    self.event_sender.send(event).await;

                    // self.response_sender
                    //     .send(DrvSvcResponse::Entered(DrvSvcTransitionResult {
                    //         prev_task_result,
                    //         measured_entry,
                    //     }))
                    //     .await;

                    DriverState::Idle(off_driver)
                }
                // Safety: The transition task was programmed to not
                //         roll back on CRC error.
                CompletedRadioTransition::Rollback(..) => unreachable!(),
                // Safety: Switching the radio off is infallible.
                CompletedRadioTransition::Fallback(..) => unreachable!(),
            },
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
    ///
    /// When the `tsch` feature is enabled, this sends an Enhanced ACK with Time
    /// Correction IE. The Time Correction IE contains timing synchronization
    /// information for TSCH operations.
    async fn complete_reception_with_ack(
        &self,
        receiving_rx_driver: impl ReceivingRxState<RadioDriverImpl>,
        seq_nb: u8,
        frame_version: FrameVersion,
        ifs: Ifs<PhyOf<RadioDriverImpl>>,
        #[cfg(feature = "tsch")] rx_instant: NsInstant,
    ) -> DriverState<RadioDriverImpl> {
        // Safety: We use the tx ACK frame sequentially and exclusively from
        //         this method.
        let outbound_ack_frame = self.outbound_ack_frame.take().unwrap();

        let outbound_ack_buffer = outbound_ack_frame.into_buffer();

        let writer = match frame_version {
            FrameVersion::Ieee802154_2003 | FrameVersion::Ieee802154_2006 => {
                imm_ack_frame::<RadioDriverImpl>(seq_nb, outbound_ack_buffer)
            }
            #[cfg(feature = "tsch")]
            FrameVersion::Ieee802154 => {
                let mut writer = enh_ack_frame::<RadioDriverImpl>(seq_nb, outbound_ack_buffer);

                // For TSCH mode, update the Time Correction IE in the enhanced ACK
                if let Some(mut tc) = writer.ies_fields_mut().time_correction_mut() {
                    let delta = match self.expected_rx_framestart.get() {
                        Some(expected_rx_framestart) => {
                            if expected_rx_framestart > rx_instant {
                                -((expected_rx_framestart - rx_instant).to_micros() as i16)
                            } else {
                                (rx_instant - expected_rx_framestart).to_micros() as i16
                            }
                        }
                        None => 0i16,
                    };
                    tc.set_time_sync(delta);
                    tc.set_nack(false);
                }
                writer
            }
            _ => unreachable!(),
        };

        let outbound_ack_frame = writer.into_radio_frame::<RadioDriverImpl>();

        let outbound_ack_task = RadioTaskTx {
            radio_frame: outbound_ack_frame,
            cca: false,
        };

        match receiving_rx_driver
            .schedule_tx(outbound_ack_task, None.into(), Some(Ifs::ack()), true)
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
                return self
                    .complete_transmission(tx_driver, TransmissionType::Ack(rx_task_result), ifs)
                    .await;
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

                DriverState::WaitingForFrame(listening_rx_driver)
            }
            // Safety: Scheduling ACK cannot fall back as it does no CCA.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        }
    }

    async fn waiting_for_frame(
        &self,
        mut listening_rx_driver: RadioDriver<RadioDriverImpl, RadioTaskRx>,
    ) -> DriverState<RadioDriverImpl> {
        loop {
            // Check if there's an RX window timeout set
            if let Some(rx_window_end) = self.rx_window_end.get() {
                let hardware_address = listening_rx_driver.ieee802154_address();
                let stop_listening_result = match listening_rx_driver
                    .stop_listening(rx_window_end.into())
                    .await
                {
                    Ok(result) => result,
                    Err((_, recovered_rx_driver)) => {
                        // The stop_listening call failed (e.g., already past the deadline)
                        // Clear the window and try to stop immediately
                        self.rx_window_end.set(None);
                        self.expected_rx_framestart.set(None);
                        listening_rx_driver = recovered_rx_driver;
                        match listening_rx_driver.stop_listening(None.into()).await {
                            Ok(result) => result,
                            Err(_) => unreachable!(),
                        }
                    }
                };

                match stop_listening_result {
                    StopListeningResult::FrameStarted(rx_instant, receiving_rx_driver) => {
                        // Frame started within the window, process it
                        return self
                            .validate_and_receive_frame(
                                receiving_rx_driver,
                                &hardware_address,
                                #[cfg(feature = "tsch")]
                                rx_instant,
                            )
                            .await;
                    }
                    StopListeningResult::RxWindowEnded(result) => {
                        // Window expired without a frame arriving
                        self.rx_window_end.set(None);
                        self.expected_rx_framestart.set(None);
                        let rx_frame = match result.prev_task_result {
                            RxResult::RxWindowEnded(frame) => frame,
                            RxResult::Frame(frame, _) => frame.forget_size::<RadioDriverImpl>(),
                            RxResult::CrcError(frame, _) => frame,
                        };
                        self.event_sender
                            .send(DrvSvcEvent::RxWindowEnded(rx_frame))
                            .await;
                        return DriverState::Idle(result.this_state);
                    }
                }
            }

            // No RX window set, use the original select-based approach
            match select(
                listening_rx_driver.wait_for_frame_start(),
                self.request_receiver.receive(),
            )
            .await
            {
                select::Either::First(_) => {
                    let hardware_address = listening_rx_driver.ieee802154_address();
                    let (receiving_rx_driver, rx_instant) = if let Ok(
                        StopListeningResult::FrameStarted(rx_instant, receiving_rx_driver),
                    ) =
                        listening_rx_driver.stop_listening(None.into()).await
                    {
                        (receiving_rx_driver, rx_instant)
                    } else {
                        // Without a timeout the stop_listening() method
                        // shouldn't fail.
                        unreachable!()
                    };
                    return self
                        .validate_and_receive_frame(
                            receiving_rx_driver,
                            &hardware_address,
                            rx_instant,
                        )
                        .await;
                }
                select::Either::Second(request) => {
                    match &request {
                        DrvSvcRequest::CompleteThenStartTx(task_tx) => {
                            // Clear the RX window since we're transitioning to TX
                            self.rx_window_end.set(None);
                            self.expected_rx_framestart.set(None);

                            let latest_frame_start = if let Timestamp::Scheduled(tx_at) = task_tx.at
                            {
                                listening_rx_driver
                                    .latest_rx_frame_start_before_tx(tx_at, task_tx.cca, None)
                                    .into()
                            } else {
                                None.into()
                            };

                            let hardware_address = listening_rx_driver.ieee802154_address();
                            let stop_listening_result = match listening_rx_driver
                                .stop_listening(latest_frame_start)
                                .await
                            {
                                Ok(result) => result,
                                Err((_, recovered_rx_driver)) => {
                                    listening_rx_driver = recovered_rx_driver;
                                    continue;
                                }
                            };

                            let old_request = self.pending_request.replace(Some(request));
                            debug_assert!(old_request.is_none());

                            match stop_listening_result {
                                StopListeningResult::FrameStarted(
                                    rx_instant,
                                    receiving_rx_driver,
                                ) => {
                                    // A frame has started in the meantime, so we cannot
                                    // serve the pending tx request, yet.
                                    return self
                                        .validate_and_receive_frame(
                                            receiving_rx_driver,
                                            &hardware_address,
                                            rx_instant,
                                        )
                                        .await;
                                }
                                StopListeningResult::RxWindowEnded(radio_transition_result) => {
                                    // No frame has started, so we can safely end the rx
                                    // window and serve the pending tx request.
                                    return self
                                        .end_rx_window(
                                            radio_transition_result,
                                            ReceptionType::Frame,
                                        )
                                        .await;

                                    // give feedback with received
                                }
                            }
                        }
                        DrvSvcRequest::CompleteThenGoIdle => {
                            // Clear the RX window since we're going idle
                            self.rx_window_end.set(None);
                            self.expected_rx_framestart.set(None);

                            match listening_rx_driver.stop_listening(None.into()).await {
                                Ok(result) => match result {
                                    StopListeningResult::FrameStarted(_instant, _) => todo!(),
                                    StopListeningResult::RxWindowEnded(radio_transition_result) => {
                                        match radio_transition_result.prev_task_result {
                                            RxResult::RxWindowEnded(radio_frame) => {
                                                self.event_sender
                                                    .send(DrvSvcEvent::RxWindowEnded(radio_frame))
                                                    .await;
                                            }
                                            _ => unreachable!(),
                                        }
                                        return DriverState::Idle(
                                            radio_transition_result.this_state,
                                        );
                                    }
                                },
                                Err((_, _)) => unreachable!(),
                            }
                        }
                        _ => unreachable!(),
                    }
                }
            };
        }
    }

    /// Once an incoming frame has been observed, it needs to be validated and
    /// possibly acknowledged.
    async fn validate_and_receive_frame(
        &self,
        mut receiving_rx_driver: impl ReceivingRxState<RadioDriverImpl>,
        hardware_address: &[u8; 8],
        #[cfg(feature = "tsch")] rx_instant: NsInstant,
    ) -> DriverState<RadioDriverImpl> {
        let preliminary_frame_info = receiving_rx_driver.preliminary_frame_info().await.unwrap();
        let next_ifs =
            Ifs::<PhyOf<RadioDriverImpl>>::from_mpdu_length(preliminary_frame_info.mpdu_length);
        let frame_is_valid = is_frame_valid_and_for_us(hardware_address, &preliminary_frame_info);

        // If the frame is valid and ACK is requested, then
        // schedule a tx ACK task. Otherwise finalize the rx
        // task and receive the next task (if any).
        if frame_is_valid {
            self.event_sender.send(DrvSvcEvent::FrameStarted).await;
            // Safety: Valid frames always have a frame control field.
            let fc = preliminary_frame_info.frame_control.unwrap();
            let ack_request = fc.ack_request();
            let seq_nb = preliminary_frame_info.seq_nr;
            let frame_version = fc.frame_version();
            match seq_nb {
                Some(seq_nb) if ack_request => {
                    self.complete_reception_with_ack(
                        receiving_rx_driver,
                        seq_nb,
                        frame_version,
                        next_ifs,
                        #[cfg(feature = "tsch")]
                        rx_instant,
                    )
                    .await
                }
                _ => {
                    self.complete_reception(receiving_rx_driver, ReceptionType::Frame, next_ifs)
                        .await
                }
            }
        } else {
            self.drop_invalid_frame(receiving_rx_driver).await
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
        radio_transition_result: RadioTransitionResult<RadioDriverImpl, RadioTaskRx, RadioTaskOff>,
        reception_type: ReceptionType,
    ) -> DriverState<RadioDriverImpl> {
        let RadioTransitionResult {
            prev_task_result: rx_task_result,
            this_state: off_driver,
            ..
        } = radio_transition_result;

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

        let event = match reception_type {
            ReceptionType::Ack(tx_radio_frame, _, tx_timestamp, fallback_on_nack) => {
                self.temp_inbound_frame.set(Some(rx_radio_frame));
                let pending_request = if fallback_on_nack {
                    self.pending_request.take()
                } else {
                    None
                };
                DrvSvcEvent::Nack(tx_radio_frame, tx_timestamp, pending_request, None)
            }
            ReceptionType::Frame => DrvSvcEvent::RxWindowEnded(rx_radio_frame),
        };
        self.event_sender.send(event).await;

        let request = match self.pending_request.take() {
            Some(request) => request,
            None => self.request_receiver.receive().await,
        };
        match self.schedule_next(off_driver, request).await {
            Ok(new_state) => new_state,
            Err(off_driver) => DriverState::Idle(off_driver),
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
        receiving_rx_driver: impl ReceivingRxState<RadioDriverImpl>,
    ) -> DriverState<RadioDriverImpl> {
        // Safety: The temporary rx frame will be recovered by the end of the
        //         procedure.
        let temporary_rx_frame = self.temp_inbound_frame.take().unwrap();
        let rx_task = RadioTaskRx {
            radio_frame: temporary_rx_frame,
        };
        match receiving_rx_driver
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

                DriverState::WaitingForFrame(listening_rx_driver)
            }
            // Safety: The transition task was programmed to not roll back on
            //         CRC error.
            CompletedRadioTransition::Rollback(..) => unreachable!(),
            // Safety: Scheduling rx tasks does not fall back.
            CompletedRadioTransition::Fallback(..) => unreachable!(),
        }
    }

    async fn schedule_next(
        &self,
        off_driver: RadioDriver<RadioDriverImpl, RadioTaskOff>,
        request: DrvSvcRequest,
    ) -> Result<DriverState<RadioDriverImpl>, RadioDriver<RadioDriverImpl, RadioTaskOff>> {
        match request {
            DrvSvcRequest::CompleteThenStartTx(task_tx) => {
                let seq_nb = task_tx.mpdu.sequence_number();
                let ack_request = task_tx.mpdu.frame_control().ack_request();
                let tx_task_ifs = Ifs::<PhyOf<RadioDriverImpl>>::from_mpdu_length(
                    task_tx.mpdu.pdu_length::<RadioDriverImpl>(),
                );
                match off_driver
                    .schedule_tx(
                        RadioTaskTx {
                            radio_frame: task_tx.mpdu.into_radio_frame::<RadioDriverImpl>(),
                            cca: task_tx.cca,
                        },
                        OptionalNsInstant::from(task_tx.at),
                        task_tx.channel,
                    )
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        this_state: tx_driver,
                        measured_entry,
                        ..
                    }) => {
                        self.event_sender
                            .send(DrvSvcEvent::TxStarted(measured_entry))
                            .await;
                        Ok(match seq_nb {
                            Some(seq_nb) if ack_request => DriverState::SendingFrameWithAck(
                                tx_driver,
                                tx_task_ifs,
                                seq_nb,
                                task_tx.fallback_on_nack,
                            ),
                            _ => DriverState::SendingFrame(tx_driver, tx_task_ifs),
                        })
                    }
                    CompletedRadioTransition::Fallback(
                        RadioTransitionResult {
                            this_state,
                            measured_entry,
                            ..
                        },
                        radio_task_error,
                    ) => {
                        let event = match radio_task_error {
                            RadioTaskError::Scheduling(task_tx, _) => {
                                DrvSvcEvent::SchedulingFailed(task_tx.radio_frame)
                            }
                            RadioTaskError::Task(error) => match error {
                                TxError::CcaBusy(radio_frame) => {
                                    DrvSvcEvent::CcaBusy(radio_frame, measured_entry)
                                }
                            },
                        };
                        // Send back the result of the failed transition.
                        self.event_sender.send(event).await;

                        Err(this_state)
                    }
                    // Safety: The off task doesn't roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenStartRx(task_rx) => {
                // Only best-effort tasks may be scheduled after a tx task.
                // To schedule a timed task after tx, schedule a "Radio Off"
                // task in between first.
                // TODO: check if still valid ?
                // if prev_request_result.is_some() {
                //     assert!(matches!(task_rx.start, Timestamp::BestEffort));
                // }
                let rx_window_end = task_rx.rx_window_end;
                let expected_rx_framestart = task_rx.expected_rx_framestart;
                match off_driver
                    .schedule_rx(
                        RadioTaskRx {
                            radio_frame: task_rx.radio_frame,
                        },
                        OptionalNsInstant::from(task_rx.start),
                        task_rx.channel,
                    )
                    .complete_and_transition()
                    .await
                {
                    CompletedRadioTransition::Entered(RadioTransitionResult {
                        this_state, ..
                    }) => {
                        // Set the RX window end time if specified
                        self.rx_window_end.set(rx_window_end);
                        self.expected_rx_framestart.set(expected_rx_framestart);
                        // TODO: valid event ?
                        // self.event_sender
                        //     .send(DrvSvcEvent::RxStarted(measured_entry))
                        //     .await;
                        Ok(DriverState::WaitingForFrame(this_state))
                    }
                    // Safety: The off task doesn't roll back.
                    CompletedRadioTransition::Rollback(..) => unreachable!(),
                    // Safety: Scheduling an rx task doesn't fall back.
                    CompletedRadioTransition::Fallback(..) => unreachable!(),
                }
            }
            DrvSvcRequest::CompleteThenGoIdle => unreachable!(),
        }
    }
}

/// Validates an incoming ACK frame and checks if it matches the expected sequence number.
///
/// This function supports both:
/// - Immediate ACK
/// - Enhanced ACK
fn validate_ack_frame<RadioDriverImpl: DriverConfig>(
    rx_ack_frame: RadioFrame<RadioFrameSized>,
    expected_seq_nb: u8,
) -> Result<(RadioFrame<RadioFrameUnsized>, Option<i16>), RadioFrame<RadioFrameUnsized>> {
    use dot15d4_driver::radio::frame::{FrameType, FrameVersion};

    let frame = MpduFrame::from_radio_frame(rx_ack_frame);
    let reader = frame.reader();

    let fc = reader.frame_control();

    // Frame type must be ACK
    let (is_valid, time_sync_us) = if fc.frame_type() != FrameType::Ack
        || fc.sequence_number_suppression()
        || reader.sequence_number() != Some(expected_seq_nb)
    {
        (false, None)
    } else {
        // Check frame version and handle accordingly
        match fc.frame_version() {
            // IEEE 802.15.4-2003 or 2006: Immediate ACK
            FrameVersion::Ieee802154_2003 | FrameVersion::Ieee802154_2006 => (true, None),
            // IEEE 802.15.4-2015+: Could be Enhanced ACK
            FrameVersion::Ieee802154 => {
                #[cfg(feature = "tsch")]
                {
                    let reader = reader.parse_addressing();
                    if let Ok(reader) = reader {
                        let reader = reader.parse_security().parse_ies::<RadioDriverImpl>();
                        if let Ok(reader) = reader {
                            let ies = reader.ies_fields();
                            if let Some(tc) = ies.time_correction() {
                                // If NACK bit is set, this is a negative acknowledgement
                                (!tc.nack(), Some(tc.time_sync()))
                            } else {
                                (false, None)
                            }
                        } else {
                            (false, None)
                        }
                    } else {
                        (false, None)
                    }
                }
                #[cfg(not(feature = "tsch"))]
                {
                    // TODO: Is it too permissive to accept valid Enhanced
                    // Ack if we are not supposed to support TSCH ?
                    true
                }
            }
            // Unknown frame version
            _ => (false, None),
        }
    };
    if is_valid {
        Ok((
            frame
                .into_radio_frame::<RadioDriverImpl>()
                .forget_size::<RadioDriverImpl>(),
            time_sync_us,
        ))
    } else {
        Err(frame
            .into_radio_frame::<RadioDriverImpl>()
            .forget_size::<RadioDriverImpl>())
    }
}
