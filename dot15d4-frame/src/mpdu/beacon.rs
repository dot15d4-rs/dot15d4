use dot15d4_driver::{
    frame::{AddressingMode, AddressingRepr, FrameType, FrameVersion, PanIdCompressionRepr},
    DriverConfig,
};
use dot15d4_util::{
    allocator::{export::*, AsyncBufferAllocator},
    Error, Result,
};

#[cfg(feature = "ies")]
use crate::repr::IeListRepr;
use crate::{
    fields::MpduParser,
    mpdu::MpduFrame,
    repr::{mpdu_repr, IeRepr, IeReprList, MpduRepr, SeqNrRepr},
    MpduWithAllFields, MpduWithSecurity,
};

/// Re-usable part of the structural representation of a beacon MPDU.
///
/// Note: IEs have not yet been configured as they may be individual to each
///       beacon frame.
pub const BEACON_FRAME_REPR: MpduRepr<MpduWithSecurity> = mpdu_repr()
    .with_frame_control(SeqNrRepr::No)
    .with_addressing(AddressingRepr::new(
        AddressingMode::Short,
        AddressingMode::Absent,
        true,
        PanIdCompressionRepr::Legacy,
    ))
    .without_security();

/// Allocates an instantiates a reader/writer for a beacon frame with the given
/// IE list and payload representation.
///
/// Validates the given IE list (if any) and returns an error if inconsistencies
/// are found.
///
/// Note: We assume that this function is called when building beacons from
///       scratch. Therefore the given IE list representation must not contain
///       termination IEs. These will be added and initialized automatically.
///       Also note that actual IE content and payload must be written directly
///       into the returned buffer-backed MPDU. This zero-copy approach is more
///       efficient than instantiating an IE list and payload slice just to move
///       (copy) it into the function and copy it once again into the buffer
///       verbatim.
pub async fn beacon_frame<'ies, Config: DriverConfig, const ALLOCATOR_BACKLOG: usize>(
    ies: Option<IeReprList<'ies, IeRepr<'ies>>>,
    beacon_payload_length: u16,
    buffer_allocator: AsyncBufferAllocator<ALLOCATOR_BACKLOG>,
) -> Result<MpduParser<MpduFrame, MpduWithAllFields>>
where
    Const<ALLOCATOR_BACKLOG>: ToUInt,
    U<ALLOCATOR_BACKLOG>: ArrayLength,
{
    let beacon_frame_repr = match ies {
        Some(_ies) => {
            #[cfg(not(feature = "ies"))]
            panic!("not supported");
            #[cfg(feature = "ies")]
            BEACON_FRAME_REPR.with_ies(IeListRepr::WithoutTerminationIes(_ies))
        }
        None => BEACON_FRAME_REPR.without_ies(),
    };
    let min_buffer_size = beacon_frame_repr.min_buffer_size::<Config>(beacon_payload_length)?;
    let buffer = buffer_allocator.allocate_buffer(min_buffer_size).await;
    match beacon_frame_repr.into_parsed_mpdu::<Config>(
        FrameVersion::Ieee802154_2006,
        FrameType::Beacon,
        0,
        buffer,
    ) {
        Ok(result) => Ok(result),
        Err(buffer) => {
            // Safety: This is the buffer we just allocated.
            unsafe { buffer_allocator.deallocate_buffer(buffer) };
            Err(Error)
        }
    }
}
