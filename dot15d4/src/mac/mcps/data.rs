#![allow(dead_code)]
use core::{marker::PhantomData, num::NonZero};

#[cfg(feature = "rtos-trace")]
use crate::trace::{
    MAC_INDICATION, MAC_REQUEST, RX_CRC_ERROR, RX_FRAME, RX_INVALID, RX_WINDOW_ENDED, TX_CCABUSY,
    TX_FRAME, TX_NACK,
};
use crate::{
    driver::{
        frame::{
            Address, AddressingMode, PanId, RadioFrame, RadioFrameRepr, RadioFrameSized,
            RadioFrameUnsized,
        },
        radio::DriverConfig,
        tasks::{RxError, RxResult, Timestamp, TxError, TxResult},
        DrvSvcRequest, DrvSvcResponse, DrvSvcTaskError, DrvSvcTaskRx, DrvSvcTaskTx,
    },
    mac::{frame::mpdu::MpduFrame, task::*, MacBufferAllocator},
    util::{Error, Result as SimplifiedResult},
};

pub enum DataError {
    // TODO: not supported
    TransactionOverflow,
    // TODO: not supported
    TransactionExpired,
    // TODO: not supported
    ChannelAccessFailure,
    // TODO: not supported
    InvalidAddress,
    // TODO: not supported
    NoAck,
    // TODO: not supported
    CounterError,
    // TODO: not supported
    FrameTooLong,
    // TODO: not supported
    InvalidParameter,
}

pub struct DataRequest {
    /// The frame to be sent.
    mpdu: MpduFrame,
}

/// Represents an MLME-DATA.request.
///
/// Note: Parameters that determine frame structure are currently read-only. We
///       may introduce structural writers if required. These should then safely
///       move existing data around.
impl DataRequest {
    pub fn new(mpdu: MpduFrame) -> Self {
        Self { mpdu }
    }

    pub fn src_addr_mode(&self) -> AddressingMode {
        self.mpdu.frame_control().src_addressing_mode()
    }

    pub fn dst_addr_mode(&self) -> AddressingMode {
        self.mpdu.frame_control().dst_addressing_mode()
    }

    pub fn dst_pan_id(&self) -> SimplifiedResult<PanId<&[u8]>> {
        self.mpdu
            .reader()
            .parse_addressing()?
            .into_addressing_fields()?
            .ok_or(Error)?
            .into_dst_pan_id()
            .ok_or(Error)
    }

    pub fn set_dst_pan_id<Bytes: AsRef<[u8]>>(
        &mut self,
        pan_id: PanId<Bytes>,
    ) -> SimplifiedResult<()> {
        self.mpdu
            .writer()
            .parse_addressing_mut()?
            .addressing_fields_mut()?
            .ok_or(Error)?
            .dst_pan_id_mut()
            .ok_or(Error)?
            .set_le_bytes(pan_id.as_ref());
        Ok(())
    }

    pub fn dst_addr(&self) -> SimplifiedResult<Address<&[u8]>> {
        self.mpdu
            .reader()
            .parse_addressing()?
            .into_addressing_fields()?
            .ok_or(Error)?
            .into_dst_address()
            .ok_or(Error)
    }

    pub fn set_dst_addr<Bytes: AsRef<[u8]>>(
        &mut self,
        dst_addr: &Address<Bytes>,
    ) -> SimplifiedResult<()> {
        let mut writer = self.mpdu.writer().parse_addressing_mut()?;
        let mut addr_fields = writer.addressing_fields_mut()?.ok_or(Error)?;
        addr_fields.dst_address_mut().ok_or(Error)?.set(dst_addr)
    }

    pub fn tx_options(&mut self) -> TxOptions<'_> {
        TxOptions {
            mpdu: &mut self.mpdu,
        }
    }
}
pub struct TxOptions<'mpdu> {
    mpdu: &'mpdu mut MpduFrame,
}

impl<'mpdu> TxOptions<'mpdu> {
    pub fn ack_tx(&self) -> bool {
        self.mpdu.frame_control().ack_request()
    }

    pub fn set_ack_tx(&mut self, ack_tx: bool) {
        self.mpdu.frame_control_mut().set_ack_request(ack_tx);
    }

    pub fn pan_id_suppressed(&self) -> bool {
        self.mpdu.frame_control().pan_id_compression()
    }

    pub fn set_pan_id_suppressed(&mut self, pan_id_suppressed: bool) {
        self.mpdu
            .frame_control_mut()
            .set_pan_id_compression(pan_id_suppressed);
    }

    pub fn seq_num_suppressed(&self) -> bool {
        self.mpdu.frame_control().sequence_number_suppression()
    }

    pub fn set_seq_num_suppressed(&mut self, seq_num_suppressed: bool) {
        self.mpdu
            .frame_control_mut()
            .set_sequence_number_suppression(seq_num_suppressed);
    }
}

pub struct DataConfirm {
    /// Timestamp of frame transmission
    pub timestamp: Option<NonZero<u32>>,
    /// Whether the frame has been acknowledged or not
    pub acked: bool,
}

pub struct DataIndication {
    /// The received frame.
    pub mpdu: MpduFrame,
    /// Timestamp of frame reception
    pub timestamp: Option<NonZero<u32>>,
}

pub(crate) struct DataRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: DataRequestState<'task, RadioDriverImpl>,
}

enum DataRequestState<'task, RadioDriverImpl: DriverConfig> {
    Initial(
        /// MPDU to be sent.
        MpduFrame,
        /// Placeholder for future references.
        PhantomData<&'task RadioDriverImpl>,
    ),
    SendingFrame,
}

impl<RadioDriverImpl: DriverConfig> DataRequestTask<'_, RadioDriverImpl> {
    pub fn new(data_request: DataRequest) -> Self {
        Self {
            state: DataRequestState::Initial(data_request.mpdu, PhantomData),
        }
    }

    fn handle_tx_driver_response(response: DrvSvcResponse) -> DataRequestResult {
        match response {
            DrvSvcResponse::Tx(tx_result) => match tx_result {
                Ok(TxResult::Sent(sent_tx_frame)) => {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::marker(TX_FRAME);

                    DataRequestResult::Sent(sent_tx_frame.forget_size::<RadioDriverImpl>())
                }
                // TODO: resend
                Ok(TxResult::Nack(unacknowledged_tx_frame)) => {
                    #[cfg(feature = "rtos-trace")]
                    rtos_trace::trace::marker(TX_NACK);

                    DataRequestResult::Nack(unacknowledged_tx_frame)
                }
                Err(tx_error) => match tx_error {
                    // TODO: CSMA/CA
                    DrvSvcTaskError::Task(TxError::CcaBusy(unsent_tx_frame)) => {
                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::marker(TX_CCABUSY);

                        DataRequestResult::CcaBusy(unsent_tx_frame)
                    }
                    // TODO: Implement if required by a driver implementation.
                    _ => unreachable!(),
                },
            },
            // Safety: We issued a Tx task and therefore expect a Tx result.
            _ => unreachable!(),
        }
    }

    fn tx_task(tx_mpdu: MpduFrame) -> DrvSvcRequest {
        DrvSvcTaskTx {
            at: Timestamp::BestEffort,
            radio_frame: tx_mpdu.into_radio_frame::<RadioDriverImpl>(),
            // TODO: CSMA/CA
            cca: false,
        }
        .into()
    }
}

/// Final result of a data request task.
pub(crate) enum DataRequestResult {
    /// The Tx frame was sent.
    ///
    /// If ACK was requested, this result will only be returned if the frame was
    /// successfully acknowledged. Otherwise this result merely indicates that
    /// the frame was accepted by the driver and transmitted over the air.
    Sent(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameUnsized>,
    ),
    CcaBusy(
        /// unsent radio frame
        RadioFrame<RadioFrameSized>,
    ),
    /// Not acknowledged: timeout or explicit NACK
    Nack(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameSized>,
    ),
}

impl<RadioDriverImpl: DriverConfig> MacTask for DataRequestTask<'_, RadioDriverImpl> {
    type Result = DataRequestResult;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            DataRequestState::Initial(tx_mpdu, _) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = DataRequestState::SendingFrame;
                MacTaskTransition::DrvSvcRequest(self, Self::tx_task(tx_mpdu), None)
            }
            DataRequestState::SendingFrame => {
                match event {
                    MacTaskEvent::DrvSvcResponse(driver_response) => {
                        let request_result = Self::handle_tx_driver_response(driver_response);
                        MacTaskTransition::Terminated(request_result)
                    }
                    // Safety: We issued a Tx task and therefore expect a Tx result.
                    _ => unreachable!(),
                }
            }
        }
    }
}

pub(crate) struct DataIndicationTask<'task, RadioDriverImpl: DriverConfig> {
    buffer_allocator: MacBufferAllocator,
    state: DataIndicationState<'task, RadioDriverImpl>,
}

enum DataIndicationState<'task, RadioDriverImpl: DriverConfig> {
    // Placeholder for future references.
    Initial(PhantomData<&'task RadioDriverImpl>),
    WaitingForFrame,
}

impl<'task, RadioDriverImpl: DriverConfig> DataIndicationTask<'task, RadioDriverImpl> {
    pub fn new(buffer_allocator: MacBufferAllocator) -> Self {
        Self {
            buffer_allocator,
            state: DataIndicationState::Initial(PhantomData),
        }
    }

    fn allocate_rx_radio_frame(
        buffer_allocator: &MacBufferAllocator,
    ) -> Option<RadioFrame<RadioFrameUnsized>> {
        let rx_buffer = buffer_allocator.try_allocate_buffer(
            RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new().max_buffer_length()
                as usize,
        );

        if let Ok(rx_buffer) = rx_buffer {
            Some(RadioFrame::<RadioFrameUnsized>::new::<RadioDriverImpl>(
                rx_buffer,
            ))
        } else {
            None
        }
    }

    fn handle_rx_driver_response(
        &self,
        response: DrvSvcResponse,
    ) -> Result<MpduFrame, RadioFrame<RadioFrameUnsized>> {
        match response {
            DrvSvcResponse::Rx(rx_result) => match rx_result {
                Ok(rx_result) => match rx_result {
                    RxResult::Frame(rx_frame) => {
                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::marker(RX_FRAME);

                        let mpdu = MpduFrame::from_radio_frame(rx_frame);
                        Ok(mpdu)
                    }
                    RxResult::FilteredFrame(recovered_radio_frame) => {
                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::marker(RX_INVALID);

                        Err(recovered_radio_frame.forget_size::<RadioDriverImpl>())
                    }
                    RxResult::RxWindowEnded(recovered_radio_frame) => {
                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::marker(RX_WINDOW_ENDED);

                        Err(recovered_radio_frame)
                    }
                    RxResult::CrcError(recovered_radio_frame) => {
                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::marker(RX_CRC_ERROR);

                        Err(recovered_radio_frame)
                    }
                },
                Err(rx_task_error) => match rx_task_error {
                    // Bailing CRC errors should be handled by the driver
                    // service.
                    DrvSvcTaskError::Task(RxError::CrcError) => unreachable!(),
                    // TODO: Implement if required by a driver implementation.
                    _ => unreachable!(),
                },
            },
            // Safety: We scheduled an Rx task and therefore expect an Rx task
            //         response.
            _ => unreachable!(),
        }
    }

    fn produce_indication_and_restart_rx(
        rx_mpdu: MpduFrame,
        buffer_allocator: MacBufferAllocator,
    ) -> MacTaskTransition<Self> {
        let data_indication = DataIndication {
            mpdu: rx_mpdu,
            timestamp: None,
        };
        let next_rx_radio_frame =
            Self::allocate_rx_radio_frame(&buffer_allocator).expect("no capacity");
        MacTaskTransition::DrvSvcRequest(
            Self {
                buffer_allocator,
                state: DataIndicationState::WaitingForFrame,
            },
            Self::rx_task(next_rx_radio_frame),
            Some(data_indication),
        )
    }

    fn rx_task(radio_frame: RadioFrame<RadioFrameUnsized>) -> DrvSvcRequest {
        DrvSvcTaskRx {
            start: Timestamp::BestEffort,
            radio_frame,
        }
        .into()
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for DataIndicationTask<'_, RadioDriverImpl> {
    type Result = DataIndication;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_INDICATION);

        match self.state {
            DataIndicationState::Initial(_) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));

                let rx_radio_frame =
                    Self::allocate_rx_radio_frame(&self.buffer_allocator).expect("no capacity");
                self.state = DataIndicationState::WaitingForFrame;
                MacTaskTransition::DrvSvcRequest(self, Self::rx_task(rx_radio_frame), None)
            }
            DataIndicationState::WaitingForFrame => match event {
                MacTaskEvent::DrvSvcResponse(driver_response) => {
                    match self.handle_rx_driver_response(driver_response) {
                        // We successfully received an MPDU.
                        Ok(rx_mpdu) => {
                            self.state = DataIndicationState::WaitingForFrame;
                            Self::produce_indication_and_restart_rx(rx_mpdu, self.buffer_allocator)
                        }
                        // The previous Rx task ended without receiving a valid
                        // frame. Start waiting for the next frame.
                        Err(recovered_rx_radio_frame) => {
                            // Wait for the next frame
                            self.state = DataIndicationState::WaitingForFrame;
                            MacTaskTransition::DrvSvcRequest(
                                self,
                                Self::rx_task(recovered_rx_radio_frame),
                                None,
                            )
                        }
                    }
                }
                // Safety: We issued an Rx task and therefore expect an Rx result.
                _ => unreachable!(),
            },
        }
    }
}
