#![allow(dead_code)]

use core::marker::PhantomData;

use dot15d4_driver::radio::{
    frame::{Address, ExtendedAddress, PanId, ShortAddress},
    DriverConfig,
};
use dot15d4_frame::{
    fields::MpduParser,
    mpdu::{
        associate_request_frame, associate_response_frame, AssociationStatus, CapabilityInformation,
    },
    MpduWithAllFields,
};

use crate::{
    mac::{
        frame::mpdu::MpduFrame,
        mlme::get::GetRequestAttribute,
        task::{MacTask, MacTaskEvent, MacTaskTransition},
        MacBufferAllocator,
    },
    scheduler::{
        command::{pib::GetPibResult, PibCommand, PibCommandResult},
        MacCommandType, ReceptionType, SchedulerCommand, SchedulerCommandResult,
        SchedulerReceptionResult, SchedulerRequest, SchedulerResponse, SchedulerTransmissionResult,
    },
    util::allocator::IntoBuffer,
};

/// Parameters for MLME-ASSOCIATE.request.
pub struct AssociateRequest {
    pub coord_address: [u8; 8],
    pub pan_id: u16,
    pub capability: CapabilityInformation,
}

impl AssociateRequest {
    pub fn new(coord_address: [u8; 8], pan_id: u16, capability: CapabilityInformation) -> Self {
        Self {
            coord_address,
            pan_id,
            capability,
        }
    }
}

/// MLME-ASSOCIATE.confirm delivered to the upper layer.
pub enum AssociateConfirm {
    /// Association completed: response received from coordinator.
    Completed {
        status: AssociationStatus,
        short_address: ShortAddress<[u8; 2]>,
    },
    /// No acknowledgement for the association request.
    NoAck,
    /// Channel Access Failure.
    ChannelAccessFailure,
}

/// MLME-ASSOCIATE.indication delivered to the upper layer when a device
/// sends an association request to the coordinator.
/// TODO: for now, indication is handled by indication task but should be
///       done by upper layer.
pub struct AssociateIndication {
    /// The extended address of the device requesting association
    pub device_address: [u8; 8],
    /// Capability information from the requesting device.
    pub capability_information: CapabilityInformation,
    /// Short address assigned by the coordinator, derived from the last two
    /// bytes of the device's extended address.
    pub assigned_short_address: [u8; 2],
}

pub(crate) enum AssociateRequestState {
    Initial(AssociateRequest),
    SendingRequest,
    WaitingForResponse,
    WaitingForPibResult(AssociateRequest),
}

pub(crate) struct AssociateRequestTask<'task, RadioDriverImpl: DriverConfig> {
    state: AssociateRequestState,
    buffer_allocator: MacBufferAllocator,
    _task: PhantomData<&'task ()>,
    _radio: PhantomData<RadioDriverImpl>,
}

impl<'task, RadioDriverImpl: DriverConfig> AssociateRequestTask<'task, RadioDriverImpl> {
    pub fn new(request: AssociateRequest, buffer_allocator: MacBufferAllocator) -> Self {
        Self {
            state: AssociateRequestState::Initial(request),
            buffer_allocator,
            _task: PhantomData,
            _radio: PhantomData,
        }
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for AssociateRequestTask<'_, RadioDriverImpl> {
    type Result = AssociateConfirm;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        match self.state {
            AssociateRequestState::Initial(request) => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = AssociateRequestState::WaitingForPibResult(request);
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Command(SchedulerCommand::PibCommand(PibCommand::Get(
                        GetRequestAttribute::MacExtendedAddress,
                    ))),
                    None,
                )
            }
            AssociateRequestState::WaitingForPibResult(request) => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Command(
                    SchedulerCommandResult::PibCommand(PibCommandResult::Get(
                        GetPibResult::MacExtendedAddress(extended_src_addr),
                    )),
                )) => {
                    self.state = AssociateRequestState::SendingRequest;

                    let mpdu_parser = associate_request_frame::<RadioDriverImpl>(
                        self.buffer_allocator,
                        request.capability,
                    );

                    match mpdu_parser {
                        Ok(mut mpdu_parser) => {
                            let pan_id = PanId::new_owned(request.pan_id.to_le_bytes());
                            let coord_address = Address::Extended(ExtendedAddress::new_owned(
                                request.coord_address,
                            ));
                            let mut addressing = mpdu_parser.addressing_fields_mut();
                            addressing.dst_pan_id_mut().set(&pan_id);
                            addressing.src_address_mut().set(&extended_src_addr);
                            addressing.dst_address_mut().set(&coord_address);
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Transmission(mpdu_parser.into_mpdu_frame()),
                                None,
                            )
                        }
                        Err(_) => todo!(),
                    }
                }
                _ => unreachable!(),
            },
            AssociateRequestState::SendingRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Transmission(result)) => {
                    match result {
                        SchedulerTransmissionResult::Sent(radio_frame, _instant) => {
                            unsafe {
                                self.buffer_allocator
                                    .deallocate_buffer(radio_frame.into_buffer())
                            };
                            self.state = AssociateRequestState::WaitingForResponse;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Reception(ReceptionType::MacCommand(
                                    MacCommandType::AssociateResponse,
                                )),
                                None,
                            )
                        }
                        SchedulerTransmissionResult::NoAck(radio_frame, _instant) => {
                            // Deallocate associate request MAC command.
                            unsafe {
                                self.buffer_allocator
                                    .deallocate_buffer(radio_frame.into_buffer())
                            };
                            MacTaskTransition::Terminated(AssociateConfirm::NoAck)
                        }
                        SchedulerTransmissionResult::ChannelAccessFailure(radio_frame) => {
                            // Deallocate associate request MAC command.
                            unsafe {
                                self.buffer_allocator
                                    .deallocate_buffer(radio_frame.into_buffer())
                            };
                            MacTaskTransition::Terminated(AssociateConfirm::ChannelAccessFailure)
                        }
                    }
                }
                _ => unreachable!(),
            },
            AssociateRequestState::WaitingForResponse => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Reception(
                    SchedulerReceptionResult::Command(radio_frame, _instant),
                )) => {
                    // Parse the association response command frame.
                    let rx_mpdu = MpduFrame::from_radio_frame(radio_frame);

                    // Extract payload data before consuming the frame.
                    let assoc_result = {
                        rx_mpdu
                            .reader()
                            .parse_addressing()
                            .ok()
                            .and_then(|r| r.parse_security().parse_ies::<RadioDriverImpl>().ok())
                            .and_then(|r| {
                                let payload = r.try_frame_payload()?;
                                // Association response payload:
                                //   [0] = command frame identifier (0x02)
                                //   [1..3] = short address (little-endian)
                                //   [3] = association status
                                if payload.len() >= 4 {
                                    Some((
                                        ShortAddress::new_owned([payload[1], payload[2]]),
                                        AssociationStatus::from(payload[3]),
                                    ))
                                } else {
                                    None
                                }
                            })
                    };

                    let (short_address, status) = assoc_result.unwrap_or((
                        ShortAddress::new_owned([0xff, 0xff]),
                        AssociationStatus::PanAccessDenied,
                    ));

                    unsafe {
                        self.buffer_allocator
                            .deallocate_buffer(rx_mpdu.into_buffer())
                    };

                    MacTaskTransition::Terminated(AssociateConfirm::Completed {
                        status,
                        short_address,
                    })
                }
                _ => unreachable!(),
            },
        }
    }
}

enum AssociateIndicationState {
    Initial,
    WaitingForRequest,
    SendingResponse,
}

pub(crate) struct AssociateIndicationTask<'task, RadioDriverImpl: DriverConfig> {
    state: AssociateIndicationState,
    buffer_allocator: MacBufferAllocator,
    _task: PhantomData<&'task ()>,
    _radio: PhantomData<RadioDriverImpl>,
}

impl<'task, RadioDriverImpl: DriverConfig> AssociateIndicationTask<'task, RadioDriverImpl> {
    pub fn new(buffer_allocator: MacBufferAllocator) -> Self {
        Self {
            state: AssociateIndicationState::Initial,
            buffer_allocator,
            _radio: PhantomData,
            _task: PhantomData,
        }
    }

    /// Extract association indication from a received command frame and build
    /// the association response frame.
    ///
    /// Parses the source extended address and capability information from
    /// the MPDU, then derives a short address from the last two bytes of
    /// the extended address.
    ///
    /// Returns the indication and the response MPDU to transmit, or `None`
    /// if parsing fails.
    ///
    /// This approach does not yet support short address conflict resolution.
    fn generate_associate_response(
        rx_mpdu: MpduFrame,
        buffer_allocator: MacBufferAllocator,
    ) -> Option<MpduParser<MpduFrame, MpduWithAllFields>> {
        let reader = rx_mpdu.reader();
        let addressing = reader.parse_addressing().ok()?;
        let fields = addressing.try_into_addressing_fields().ok()?;
        let src_addr = match fields.try_src_address()? {
            Address::Extended(ext) => ext,
            _ => return None,
        };
        let dst_addr = match fields.try_dst_address()? {
            Address::Extended(ext) => ext,
            _ => return None,
        };

        let mut device_address = [0u8; 8];
        device_address.copy_from_slice(src_addr.as_ref());
        let mut coord_address = [0u8; 8];
        coord_address.copy_from_slice(dst_addr.as_ref());
        let pan_id = fields.try_dst_pan_id().unwrap().into_u16();

        // Second pass: parse through to get frame payload.
        let reader = rx_mpdu
            .reader()
            .parse_addressing()
            .ok()?
            .parse_security()
            .parse_ies::<RadioDriverImpl>()
            .ok()?;

        let payload = reader.try_frame_payload()?;
        if payload.is_empty() {
            return None;
        }

        // TODO: use in associate response. For now, assume we request
        //       address allocation
        let capability_information = if payload.len() > 1 {
            CapabilityInformation(payload[1])
        } else {
            CapabilityInformation(0)
        };

        // Derive short address from last two bytes of extended address
        let assigned_short_address = [device_address[0], device_address[1]];

        // Deallocate received associate request.
        unsafe { buffer_allocator.deallocate_buffer(rx_mpdu.into_buffer()) };

        // Generate associate response frame with assigned short address.
        let mut response_parser =
            associate_response_frame::<RadioDriverImpl>(buffer_allocator, assigned_short_address)
                .ok()?;
        let mut addressing = response_parser.addressing_fields_mut();
        addressing
            .dst_pan_id_mut()
            .set(&PanId::new(pan_id.to_le_bytes()));
        addressing
            .src_address_mut()
            .set(&Address::Extended(ExtendedAddress::new(coord_address)));
        addressing
            .dst_address_mut()
            .set(&Address::Extended(ExtendedAddress::new(device_address)));

        Some(response_parser)
    }
}

impl<RadioDriverImpl: DriverConfig> MacTask for AssociateIndicationTask<'_, RadioDriverImpl> {
    type Result = AssociateIndication;

    fn step(mut self, event: MacTaskEvent) -> MacTaskTransition<Self> {
        match self.state {
            AssociateIndicationState::Initial => {
                debug_assert!(matches!(event, MacTaskEvent::Entry));
                self.state = AssociateIndicationState::WaitingForRequest;
                MacTaskTransition::SchedulerRequest(
                    self,
                    SchedulerRequest::Reception(ReceptionType::MacCommand(
                        MacCommandType::AssociateRequest,
                    )),
                    None,
                )
            }
            AssociateIndicationState::WaitingForRequest => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Reception(
                    SchedulerReceptionResult::Command(radio_frame, _rx_timestamp),
                )) => {
                    let mac_command_mpdu = MpduFrame::from_radio_frame(radio_frame);

                    match Self::generate_associate_response(mac_command_mpdu, self.buffer_allocator)
                    {
                        Some(mpdu_parser) => {
                            // Send the response and yield the indication.
                            self.state = AssociateIndicationState::SendingResponse;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Transmission(mpdu_parser.into_mpdu_frame()),
                                None,
                            )
                        }
                        None => {
                            // Not a valid association request; keep listening.
                            self.state = AssociateIndicationState::WaitingForRequest;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Reception(ReceptionType::MacCommand(
                                    MacCommandType::AssociateRequest,
                                )),
                                None,
                            )
                        }
                    }
                }
                _ => unreachable!(),
            },
            AssociateIndicationState::SendingResponse => match event {
                MacTaskEvent::SchedulerResponse(SchedulerResponse::Transmission(result)) => {
                    match result {
                        SchedulerTransmissionResult::Sent(radio_frame, _instant) => {
                            unsafe {
                                self.buffer_allocator
                                    .deallocate_buffer(radio_frame.into_buffer());
                            }
                            // Yield the indication and restart listening.
                            self.state = AssociateIndicationState::WaitingForRequest;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Reception(ReceptionType::MacCommand(
                                    MacCommandType::AssociateRequest,
                                )),
                                None,
                            )
                        }
                        SchedulerTransmissionResult::NoAck(radio_frame, _instant) => {
                            unsafe {
                                self.buffer_allocator.deallocate_buffer(
                                    radio_frame.forget_size::<RadioDriverImpl>().into_buffer(),
                                );
                            }
                            // Response not acknowledged; restart listening.
                            self.state = AssociateIndicationState::WaitingForRequest;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Reception(ReceptionType::MacCommand(
                                    MacCommandType::AssociateRequest,
                                )),
                                None,
                            )
                        }
                        SchedulerTransmissionResult::ChannelAccessFailure(radio_frame) => {
                            unsafe {
                                self.buffer_allocator.deallocate_buffer(
                                    radio_frame.forget_size::<RadioDriverImpl>().into_buffer(),
                                );
                            }
                            // Channel access failure; restart listening.
                            self.state = AssociateIndicationState::WaitingForRequest;
                            MacTaskTransition::SchedulerRequest(
                                self,
                                SchedulerRequest::Reception(ReceptionType::MacCommand(
                                    MacCommandType::AssociateRequest,
                                )),
                                None,
                            )
                        }
                    }
                }
                _ => unreachable!(),
            },
        }
    }
}
