#![allow(dead_code)]
use core::marker::PhantomData;

use dot15d4_driver::{
    radio::{
        frame::{Address, AddressingMode, PanId, RadioFrame, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    timer::NsInstant,
};

#[cfg(feature = "rtos-trace")]
use crate::trace::{MAC_INDICATION, MAC_REQUEST};
use crate::{
    mac::{
        frame::mpdu::MpduFrame,
        task::{MacTask, MacTaskEvent, MacTaskTransition},
    },
    scheduler::{
        ReceptionType, SchedulerReceptionResult, SchedulerRequest, SchedulerResponse,
        SchedulerTransmissionResult,
    },
    util::{Error, Result as SimplifiedResult},
};

pub struct DataRequest {
    /// The frame to be sent.
    pub mpdu: MpduFrame,
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
            .try_into_addressing_fields()?
            .try_into_dst_pan_id()
            .ok_or(Error)
    }

    pub fn set_dst_pan_id<Bytes: AsRef<[u8]>>(
        &mut self,
        pan_id: PanId<Bytes>,
    ) -> SimplifiedResult<()> {
        self.mpdu
            .writer()
            .parse_addressing_mut()?
            .try_addressing_fields_mut()?
            .try_dst_pan_id_mut()
            .ok_or(Error)?
            .set_le_bytes(pan_id.as_ref());
        Ok(())
    }

    pub fn dst_addr(&self) -> SimplifiedResult<Address<&[u8]>> {
        self.mpdu
            .reader()
            .parse_addressing()?
            .try_into_addressing_fields()?
            .try_into_dst_address()
            .ok_or(Error)
    }

    pub fn set_dst_addr<Bytes: AsRef<[u8]>>(
        &mut self,
        dst_addr: &Address<Bytes>,
    ) -> SimplifiedResult<()> {
        let mut writer = self.mpdu.writer().parse_addressing_mut()?;
        let mut addr_fields = writer.try_addressing_fields_mut()?;
        addr_fields
            .try_dst_address_mut()
            .ok_or(Error)?
            .try_set(dst_addr)
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

pub struct DataIndication {
    /// The received frame.
    pub mpdu: MpduFrame,
    /// Timestamp of frame reception
    pub timestamp: NsInstant,
}

pub(crate) struct DataRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: DataRequestState,
    task: PhantomData<&'task u8>,
    radio: PhantomData<RadioDriverImpl>,
}

pub(crate) enum DataRequestState {
    Initial(
        /// MPDU to be sent.
        MpduFrame,
    ),
    SendingRequest,
}

impl<'task, RadioDriverImpl: DriverConfig> DataRequestTask<'task, RadioDriverImpl> {
    pub fn new(data_request: DataRequest) -> Self {
        Self {
            state: DataRequestState::Initial(data_request.mpdu),
            task: PhantomData,
            radio: PhantomData,
        }
    }
}

/// Final result of a data request task.
pub(crate) enum DataConfirm {
    /// The Tx frame was sent.
    ///
    /// If ACK was requested, this result will only be returned if the frame was
    /// successfully acknowledged. Otherwise this result merely indicates that
    /// the frame was accepted by the driver and transmitted over the air.
    Sent(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameUnsized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
    /// Not acknowledged: timeout or explicit NACK
    Nack(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameSized>,
        /// RMARKER Timestamp
        NsInstant,
    ),
    /// CSMA-CA algorithm failed.
    ChannelAccessFailure(
        /// recovered Tx radio frame
        RadioFrame<RadioFrameSized>,
    ),
}

impl<RadioDriverImpl: DriverConfig> MacTask for DataRequestTask<'_, RadioDriverImpl> {
    type Result = DataConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(MAC_REQUEST);

        match self.state {
            DataRequestState::Initial(tx_mpdu) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = DataRequestState::SendingRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Transmission(tx_mpdu),
                    None,
                )
            }
            DataRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Transmission(result)) => {
                    match result {
                        SchedulerTransmissionResult::Sent(radio_frame, instant) => {
                            MacTaskTransition::Terminated(DataConfirm::Sent(radio_frame, instant))
                        }
                        SchedulerTransmissionResult::NoAck(radio_frame, instant) => {
                            MacTaskTransition::Terminated(DataConfirm::Nack(radio_frame, instant))
                        }
                        SchedulerTransmissionResult::ChannelAccessFailure(radio_frame) => {
                            MacTaskTransition::Terminated(DataConfirm::ChannelAccessFailure(
                                radio_frame,
                            ))
                        }
                    }
                }
                _ => unreachable!(),
            },
        }
    }
}

pub(crate) struct DataIndicationTask<'task, RadioDriverImpl: DriverConfig> {
    state: DataIndicationState<'task, RadioDriverImpl>,
}

enum DataIndicationState<'task, RadioDriverImpl: DriverConfig> {
    // Placeholder for future references.
    Initial(PhantomData<&'task RadioDriverImpl>),
    WaitingForFrame,
}

impl<'task, RadioDriverImpl: DriverConfig> DataIndicationTask<'task, RadioDriverImpl> {
    pub fn new() -> Self {
        Self {
            state: DataIndicationState::Initial(PhantomData),
        }
    }

    fn produce_indication_and_restart_rx(
        rx_mpdu: MpduFrame,
        rx_timestamp: NsInstant,
    ) -> MacTaskTransition<Self> {
        let data_indication = DataIndication {
            mpdu: rx_mpdu,
            timestamp: rx_timestamp,
        };
        MacTaskTransition::SchedulerRequest(
            Self {
                state: DataIndicationState::WaitingForFrame,
            },
            SchedulerRequest::Reception(ReceptionType::Data),
            Some(data_indication),
        )
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
                self.state = DataIndicationState::WaitingForFrame;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Reception(ReceptionType::Data),
                    None,
                )
            }
            DataIndicationState::WaitingForFrame => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Reception(
                    SchedulerReceptionResult::Data(radio_frame, rx_timestamp),
                )) => {
                    self.state = DataIndicationState::WaitingForFrame;
                    let mpdu = MpduFrame::from_radio_frame(radio_frame);
                    Self::produce_indication_and_restart_rx(mpdu, rx_timestamp)
                }
                // Safety: We issued an Rx task and therefore expect an Rx result.
                _ => unreachable!(),
            },
        }
    }
}
