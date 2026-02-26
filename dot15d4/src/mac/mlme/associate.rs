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
        let _capability_information = if payload.len() > 1 {
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

#[cfg(test)]
mod associate_request_tests {
    use super::*;
    use crate::{
        mac::{
            mlme::associate::{AssociateConfirm, AssociateRequest, AssociateRequestTask},
            tests::{
                channel_access_failure_response, extract_transmission_mpdu,
                mac_command_reception_response, noack_response, sent_response, MacTaskTestRunner,
                RequestType,
            },
        },
        scheduler::{
            command::pib::{GetPibResult, PibCommandResult},
            tests::{create_test_allocator, FakeDriverConfig},
            SchedulerCommand, SchedulerCommandResult,
        },
    };
    use dot15d4_driver::{
        radio::frame::{Address, ExtendedAddress, RadioFrame, RadioFrameSized},
        timer::NsInstant,
    };
    use dot15d4_frame::mpdu::{AssociationStatus, CapabilityInformation};

    const TEST_COORD_ADDR: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    const TEST_DEVICE_ADDR: [u8; 8] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22];
    const TEST_PAN_ID: u16 = 0xBEEF;
    const INSTANT_ZER0: NsInstant = NsInstant::from_ticks(0);

    fn setup() -> MacTaskTestRunner<AssociateRequestTask<'static, FakeDriverConfig>> {
        let allocator = create_test_allocator();
        let request = AssociateRequest::new(
            TEST_COORD_ADDR,
            TEST_PAN_ID,
            CapabilityInformation(0x80), // allocate address
        );
        let task = AssociateRequestTask::new(request, allocator);
        MacTaskTestRunner::new(task, allocator)
    }

    fn pib_extended_address_response(addr: [u8; 8]) -> SchedulerResponse {
        SchedulerResponse::Command(SchedulerCommandResult::PibCommand(PibCommandResult::Get(
            GetPibResult::MacExtendedAddress(Address::Extended(ExtendedAddress::new_owned(addr))),
        )))
    }

    /// Create a properly formatted association response radio frame.
    fn create_associate_response_frame(
        runner: &MacTaskTestRunner<AssociateRequestTask<'static, FakeDriverConfig>>,
        short_address: [u8; 2],
        status: AssociationStatus,
    ) -> RadioFrame<RadioFrameSized> {
        use dot15d4_driver::radio::frame::PanId;

        let allocator = runner.allocator();
        let mut parser = dot15d4_frame::mpdu::associate_response_frame::<FakeDriverConfig>(
            *allocator,
            short_address,
        )
        .expect("failed to build associate response frame");

        // Set addressing fields.
        let mut addressing = parser.addressing_fields_mut();
        addressing
            .dst_pan_id_mut()
            .set(&PanId::new(TEST_PAN_ID.to_le_bytes()));
        addressing
            .src_address_mut()
            .set(&Address::Extended(ExtendedAddress::new(TEST_COORD_ADDR)));
        addressing
            .dst_address_mut()
            .set(&Address::Extended(ExtendedAddress::new(TEST_DEVICE_ADDR)));

        // Override the status byte if not Successful.
        if status != AssociationStatus::Successful {
            let payload = parser.frame_payload_mut();
            payload[3] = status as u8;
        }

        parser
            .into_mpdu_frame()
            .into_radio_frame::<FakeDriverConfig>()
    }

    // ========================================================================
    // Entry -> PIB Get
    // ========================================================================

    #[test]
    fn associate_entry_requests_extended_address() {
        let mut runner = setup();

        let outcome = runner.step_entry();
        let request = outcome.unwrap_request();

        match request {
            SchedulerRequest::Command(SchedulerCommand::PibCommand(
                crate::scheduler::command::pib::PibCommand::Get(
                    crate::mac::mlme::get::GetRequestAttribute::MacExtendedAddress,
                ),
            )) => {} // expected
            _ => panic!("expected PibCommand::Get(MacExtendedAddress)"),
        }
    }

    // ========================================================================
    // PIB Response -> Transmission
    // ========================================================================

    #[test]
    fn associate_builds_request_frame_after_pib() {
        let mut runner = setup();
        runner.step_entry();

        let outcome = runner.step_response(pib_extended_address_response(TEST_DEVICE_ADDR));
        let request = outcome.unwrap_request();

        assert_eq!(RequestType::from(&request), RequestType::Transmission);

        // Clean up the transmitted MPDU.
        let mpdu = extract_transmission_mpdu(request);
        runner.track_mpdu(mpdu);
    }

    // ========================================================================
    // Sent -> Reception Request
    // ========================================================================

    #[test]
    fn associate_sent_requests_response_reception() {
        let mut runner = setup();
        runner.step_entry();

        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(sent_response(mpdu, INSTANT_ZER0));

        let request = outcome.unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateResponse))
        );
    }

    // ========================================================================
    // NoAck -> Terminated
    // ========================================================================

    #[test]
    fn associate_noack_terminates() {
        let mut runner = setup();
        runner.step_entry();

        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(noack_response(mpdu, INSTANT_ZER0));

        let confirm = outcome.unwrap_terminated();
        assert!(matches!(confirm, AssociateConfirm::NoAck));
    }

    // ========================================================================
    // ChannelAccessFailure -> Terminated
    // ========================================================================

    #[test]
    fn associate_channel_access_failure_terminates() {
        let mut runner = setup();
        runner.step_entry();

        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(channel_access_failure_response(mpdu));

        let confirm = outcome.unwrap_terminated();
        assert!(matches!(confirm, AssociateConfirm::ChannelAccessFailure));
    }

    // ========================================================================
    // Successful Association Response
    // ========================================================================

    #[test]
    fn associate_receives_successful_response() {
        let mut runner = setup();
        runner.step_entry();

        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);
        runner.step_response(sent_response(mpdu, INSTANT_ZER0));

        let response_frame =
            create_associate_response_frame(&runner, [0x42, 0x00], AssociationStatus::Successful);
        let outcome = runner.step_response(mac_command_reception_response(
            response_frame,
            NsInstant::from_ticks(5000),
        ));

        let confirm = outcome.unwrap_terminated();
        match confirm {
            AssociateConfirm::Completed {
                status,
                short_address,
            } => {
                assert_eq!(status, AssociationStatus::Successful);
                assert_eq!(short_address.as_ref(), &[0x42, 0x00]);
            }
            other => panic!(
                "expected Completed, got {:?}",
                core::mem::discriminant(&other)
            ),
        }
    }

    // ========================================================================
    // Denied Association Response
    // ========================================================================

    #[test]
    fn associate_receives_denied_response() {
        let mut runner = setup();
        runner.step_entry();

        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);
        runner.step_response(sent_response(mpdu, INSTANT_ZER0));

        let response_frame = create_associate_response_frame(
            &runner,
            [0xFF, 0xFF],
            AssociationStatus::PanAccessDenied,
        );
        let outcome = runner.step_response(mac_command_reception_response(
            response_frame,
            NsInstant::from_ticks(5000),
        ));

        let confirm = outcome.unwrap_terminated();
        match confirm {
            AssociateConfirm::Completed {
                status,
                short_address,
            } => {
                assert_eq!(status, AssociationStatus::PanAccessDenied);
                assert_eq!(short_address.as_ref(), &[0xFF, 0xFF]);
            }
            other => panic!(
                "expected Completed, got {:?}",
                core::mem::discriminant(&other)
            ),
        }
    }

    // ========================================================================
    // Full Association Flow
    // ========================================================================

    #[test]
    fn associate_full_success_flow() {
        let mut runner = setup();

        // 1. Entry -> requests extended address.
        let outcome = runner.step_entry();
        assert!(outcome.is_pending());

        // 2. PIB response -> builds and transmits request frame.
        let request = runner
            .step_response(pib_extended_address_response(TEST_DEVICE_ADDR))
            .unwrap_request();
        assert_eq!(RequestType::from(&request), RequestType::Transmission);
        let mpdu = extract_transmission_mpdu(request);

        // 3. Sent -> requests associate response reception.
        let request = runner
            .step_response(sent_response(mpdu, INSTANT_ZER0))
            .unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateResponse))
        );

        // 4. Response received -> terminates with Completed.
        let response_frame =
            create_associate_response_frame(&runner, [0x10, 0x00], AssociationStatus::Successful);
        let confirm = runner
            .step_response(mac_command_reception_response(
                response_frame,
                NsInstant::from_ticks(10000),
            ))
            .unwrap_terminated();

        match confirm {
            AssociateConfirm::Completed {
                status,
                short_address,
            } => {
                assert_eq!(status, AssociationStatus::Successful);
                assert_eq!(short_address.as_ref(), &[0x10, 0x00]);
            }
            _ => panic!("expected Completed"),
        }
    }
}

// ============================================================================
// Associate Indication Tests
// ============================================================================

#[cfg(test)]
mod associate_indication_tests {
    use super::*;
    use crate::{
        mac::tests::{
            channel_access_failure_response, extract_transmission_mpdu,
            mac_command_reception_response, noack_response, sent_response, MacTaskTestRunner,
            RequestType,
        },
        scheduler::tests::{create_test_allocator, FakeDriverConfig},
    };
    use dot15d4_driver::{
        radio::frame::{Address, ExtendedAddress, RadioFrame, RadioFrameSized},
        timer::NsInstant,
    };
    use dot15d4_frame::mpdu::CapabilityInformation;

    const TEST_COORD_ADDR: [u8; 8] = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7];
    const TEST_DEVICE_ADDR: [u8; 8] = [0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7];
    const TEST_PAN_ID: u16 = 0xBEEF;
    const INSTANT_ZERO: NsInstant = NsInstant::from_ticks(0);

    fn setup() -> MacTaskTestRunner<AssociateIndicationTask<'static, FakeDriverConfig>> {
        let allocator = create_test_allocator();
        let task = AssociateIndicationTask::new(allocator);
        MacTaskTestRunner::new(task, allocator)
    }

    /// Create a properly formatted association request radio frame.
    fn create_associate_request_frame(
        runner: &MacTaskTestRunner<AssociateIndicationTask<'static, FakeDriverConfig>>,
    ) -> RadioFrame<RadioFrameSized> {
        let allocator = runner.allocator();

        let mut mpdu_parser = dot15d4_frame::mpdu::associate_request_frame::<FakeDriverConfig>(
            *allocator,
            CapabilityInformation(0x80),
        )
        .expect("failed to build associate request frame");

        let pan_id = PanId::new_owned(TEST_PAN_ID.to_le_bytes());
        let coord_addr = Address::Extended(ExtendedAddress::new_owned(TEST_COORD_ADDR));
        let device_addr = Address::Extended(ExtendedAddress::new_owned(TEST_DEVICE_ADDR));

        let mut addressing = mpdu_parser.addressing_fields_mut();
        addressing.dst_pan_id_mut().set(&pan_id);
        addressing.dst_address_mut().set(&coord_addr);
        addressing.src_address_mut().set(&device_addr);

        mpdu_parser
            .into_mpdu_frame()
            .into_radio_frame::<FakeDriverConfig>()
    }

    // ========================================================================
    // Entry
    // ========================================================================

    #[test]
    fn indication_entry_requests_associate_command_reception() {
        let mut runner = setup();

        let outcome = runner.step_entry();
        let request = outcome.unwrap_request();

        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );
    }

    // ========================================================================
    // Receives Request -> Sends Response
    // ========================================================================

    #[test]
    fn indication_receives_request_sends_response() {
        let mut runner = setup();
        runner.step_entry();

        let req_frame = create_associate_request_frame(&runner);
        let outcome = runner.step_response(mac_command_reception_response(req_frame, INSTANT_ZERO));

        let request = outcome.unwrap_request();
        assert_eq!(RequestType::from(&request), RequestType::Transmission);

        // Clean up the response MPDU.
        let mpdu = extract_transmission_mpdu(request);
        runner.track_mpdu(mpdu);
    }

    // ========================================================================
    // Sent -> Restarts Listening
    // ========================================================================

    #[test]
    fn indication_sent_restarts_listening() {
        let mut runner = setup();
        runner.step_entry();

        let req_frame = create_associate_request_frame(&runner);
        let request = runner
            .step_response(mac_command_reception_response(req_frame, INSTANT_ZERO))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(sent_response(mpdu, INSTANT_ZERO));

        let request = outcome.unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );
        assert!(!runner.is_terminated());
    }

    // ========================================================================
    // NoAck -> Restarts Listening
    // ========================================================================

    #[test]
    fn indication_noack_restarts_listening() {
        let mut runner = setup();
        runner.step_entry();

        let req_frame = create_associate_request_frame(&runner);
        let request = runner
            .step_response(mac_command_reception_response(req_frame, INSTANT_ZERO))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(noack_response(mpdu, INSTANT_ZERO));

        let request = outcome.unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );
    }

    // ========================================================================
    // ChannelAccessFailure -> Restarts Listening
    // ========================================================================

    #[test]
    fn indication_channel_access_failure_restarts_listening() {
        let mut runner = setup();
        runner.step_entry();

        let req_frame = create_associate_request_frame(&runner);
        let request = runner
            .step_response(mac_command_reception_response(req_frame, INSTANT_ZERO))
            .unwrap_request();
        let mpdu = extract_transmission_mpdu(request);

        let outcome = runner.step_response(channel_access_failure_response(mpdu));

        let request = outcome.unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );
    }

    // ========================================================================
    // Full Indication Cycle
    // ========================================================================

    #[test]
    fn indication_full_cycle_then_relisten() {
        let mut runner = setup();

        // 1. Entry -> listen for AssociateRequest.
        let request = runner.step_entry().unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );

        // 2. Receive associate request -> send response.
        let req_frame = create_associate_request_frame(&runner);
        let request = runner
            .step_response(mac_command_reception_response(req_frame, INSTANT_ZERO))
            .unwrap_request();
        assert_eq!(RequestType::from(&request), RequestType::Transmission);
        let mpdu = extract_transmission_mpdu(request);

        // 3. Response sent -> restart listening.
        let request = runner
            .step_response(sent_response(mpdu, INSTANT_ZERO))
            .unwrap_request();
        assert_eq!(
            RequestType::from(&request),
            RequestType::Reception(ReceptionType::MacCommand(MacCommandType::AssociateRequest))
        );

        // 4. Second request -> send another response.
        let req_frame2 = create_associate_request_frame(&runner);
        let request = runner
            .step_response(mac_command_reception_response(
                req_frame2,
                NsInstant::from_ticks(5000),
            ))
            .unwrap_request();
        assert_eq!(RequestType::from(&request), RequestType::Transmission);
        let mpdu = extract_transmission_mpdu(request);
        runner.track_mpdu(mpdu);

        // Task is still alive (infinite listener).
        assert!(!runner.is_terminated());
    }
}
