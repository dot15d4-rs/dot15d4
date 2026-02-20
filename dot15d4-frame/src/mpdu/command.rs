use dot15d4_driver::radio::{
    frame::{AddressingMode, AddressingRepr, FrameType, FrameVersion, PanIdCompressionRepr},
    DriverConfig,
};
use dot15d4_util::allocator::BufferAllocator;
use dot15d4_util::{Error, Result};

use crate::{
    fields::MpduParser,
    mpdu::MpduFrame,
    repr::{mpdu_repr, MpduRepr, SeqNrRepr},
    MpduWithAllFields, MpduWithSecurity,
};

/// IEEE 802.15.4 MAC command frame identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommandFrameIdentifier {
    AssociationRequest = 0x01,
    AssociationResponse = 0x02,
    DisassociationNotification = 0x03,
    DataRequest = 0x04,
    PanIdConflictNotification = 0x05,
    OrphanNotification = 0x06,
    BeaconRequest = 0x07,
    CoordinatorRealignment = 0x08,
}

/// Capability information field for the association request.
#[derive(Debug, Clone, Copy)]
pub struct CapabilityInformation(pub u8);

impl CapabilityInformation {
    pub fn new() -> Self {
        Self(0)
    }

    pub fn set_device_type_ffd(&mut self, ffd: bool) {
        // NOTE: FFD concept removed from 802.15.4-2024.
        // Device type is always 1 in 802.15.4-2024.
        if ffd {
            self.0 |= 1 << 1;
        } else {
            self.0 &= !(1 << 1);
        }
    }

    pub fn set_rx_on_when_idle(&mut self, rx_on: bool) {
        if rx_on {
            self.0 |= 1 << 3;
        } else {
            self.0 &= !(1 << 3);
        }
    }

    pub fn set_allocate_address(&mut self, allocate: bool) {
        if allocate {
            self.0 |= 1 << 7;
        } else {
            self.0 &= !(1 << 7);
        }
    }

    pub fn set_fast_association(&mut self, fast: bool) {
        if fast {
            self.0 |= 1 << 4;
        } else {
            self.0 &= !(1 << 4);
        }
    }
}

impl Default for CapabilityInformation {
    fn default() -> Self {
        let mut information = Self::new();
        information.set_device_type_ffd(true);
        information.set_fast_association(true);
        information.set_allocate_address(true);

        information
    }
}

/// Association request command frame payload length:
/// 1 byte command ID + 1 byte capability information.
pub const ASSOCIATION_REQUEST_PAYLOAD_LENGTH: u16 = 2;

/// Association response command frame payload length:
/// 1 byte command ID + 2 bytes short address + 1 byte association status.
/// NOTE: assume allocate field is set
pub const ASSOCIATION_RESPONSE_PAYLOAD_LENGTH: u16 = 4;

/// Association status codes per IEEE 802.15.4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AssociationStatus {
    Successful = 0x00,
    PanAtCapacity = 0x01,
    PanAccessDenied = 0x02,
}

impl From<u8> for AssociationStatus {
    fn from(value: u8) -> Self {
        match value {
            0x00 => Self::Successful,
            0x01 => Self::PanAtCapacity,
            0x02 => Self::PanAccessDenied,
            _ => Self::PanAccessDenied,
        }
    }
}

/// Representation of an association request command MPDU.
///
/// The device sends this frame to the coordinator with:
/// - Source: Extended address of the device
/// - Destination: Short address of the coordinator
pub const ASSOCIATE_REQUEST_FRAME_REPR: MpduRepr<MpduWithSecurity> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new(
        AddressingMode::Extended,
        AddressingMode::Extended,
        true,
        PanIdCompressionRepr::Legacy,
    ))
    .without_security();

/// Representation of an association response command MPDU.
///
/// The coordinator sends this frame back to the device with:
/// - Source: Extended address of the coordinator
/// - Destination: Extended address of the device
pub const ASSOCIATE_RESPONSE_FRAME_REPR: MpduRepr<MpduWithSecurity> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new(
        AddressingMode::Extended,
        AddressingMode::Extended,
        true,
        PanIdCompressionRepr::Legacy,
    ))
    .without_security();

/// Allocates and initializes an association request command frame.
///
/// Payload will contain the command frame identifier byte (0x01)
/// followed by the capability information byte.
pub fn associate_request_frame<Config: DriverConfig>(
    buffer_allocator: BufferAllocator,
    capability: CapabilityInformation,
) -> Result<MpduParser<MpduFrame, MpduWithAllFields>> {
    let frame_repr = ASSOCIATE_REQUEST_FRAME_REPR.without_ies();
    let min_buffer_size =
        frame_repr.min_buffer_size::<Config>(ASSOCIATION_REQUEST_PAYLOAD_LENGTH)?;
    let buffer = buffer_allocator
        .try_allocate_buffer(min_buffer_size)
        .unwrap();
    match frame_repr.into_writer::<Config>(
        FrameVersion::Ieee802154_2006,
        FrameType::MacCommand,
        ASSOCIATION_REQUEST_PAYLOAD_LENGTH,
        buffer,
    ) {
        Ok(mut writer) => {
            // Write command frame identifier into the payload.
            let payload = writer.frame_payload_mut();
            payload[0] = CommandFrameIdentifier::AssociationRequest as u8;
            // TODO: handle that in dot15d4-frame with appropriate
            //       writer.
            payload[1] = capability.0;
            Ok(writer)
        }
        Err(buffer) => {
            unsafe { buffer_allocator.deallocate_buffer(buffer) };
            Err(Error)
        }
    }
}

/// Allocates and initializes an association response command frame.
pub fn associate_response_frame<Config: DriverConfig>(
    buffer_allocator: BufferAllocator,
    short_address: [u8; 2],
) -> Result<MpduParser<MpduFrame, MpduWithAllFields>> {
    let frame_repr = ASSOCIATE_RESPONSE_FRAME_REPR.without_ies();
    let min_buffer_size =
        frame_repr.min_buffer_size::<Config>(ASSOCIATION_RESPONSE_PAYLOAD_LENGTH)?;
    let buffer = buffer_allocator
        .try_allocate_buffer(min_buffer_size)
        .unwrap();
    match frame_repr.into_writer::<Config>(
        FrameVersion::Ieee802154_2006,
        FrameType::MacCommand,
        ASSOCIATION_RESPONSE_PAYLOAD_LENGTH,
        buffer,
    ) {
        Ok(mut writer) => {
            // Write command frame identifier into the payload.
            let payload = writer.frame_payload_mut();
            payload[0] = CommandFrameIdentifier::AssociationResponse as u8;
            // Short address assigned to the device (little-endian).
            payload[1] = short_address[0];
            payload[2] = short_address[1];
            // Association status: successful (0x00).
            payload[3] = AssociationStatus::Successful as u8;
            Ok(writer)
        }
        Err(buffer) => {
            unsafe { buffer_allocator.deallocate_buffer(buffer) };
            Err(Error)
        }
    }
}
