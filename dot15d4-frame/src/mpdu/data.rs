#[cfg(feature = "ies")]
use dot15d4_driver::radio::frame::IeListRepr;
use dot15d4_driver::radio::{
    frame::{
        AddressingMode, AddressingRepr, FrameType, FrameVersion, IeRepr, IeReprList,
        PanIdCompressionRepr,
    },
    DriverConfig,
};
use dot15d4_util::{allocator::BufferAllocator, Error, Result};

#[cfg(feature = "ies")]
use crate::{
    fields::MpduParser,
    mpdu::MpduFrame,
    repr::{mpdu_repr, MpduRepr, SeqNrRepr},
    MpduWithAllFields, MpduWithSecurity,
};

/// Re-usable part of the structural representation of a data MPDU.
///
/// Uses extended addressing for both source and destination, with
/// PAN ID compression enabled (IEEE 802.15.4-2015+).
///
/// Note: IEs have not yet been configured as they may be individual to each
///       data frame.
pub const DATA_FRAME_REPR: MpduRepr<MpduWithSecurity> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new(
        AddressingMode::Extended,
        AddressingMode::Extended,
        true,
        PanIdCompressionRepr::Yes,
    ))
    .without_security();

/// Allocates and instantiates a reader/writer for a data frame with the given
/// IE list and payload representation.
///
/// Validates the given IE list (if any) and returns an error if inconsistencies
/// are found.
///
/// Note: We assume that this function is called when building data frames from
///       scratch. Therefore the given IE list representation must not contain
///       termination IEs. These will be added and initialized automatically.
///       Also note that actual IE content and payload must be written directly
///       into the returned buffer-backed MPDU. This zero-copy approach is more
///       efficient than instantiating an IE list and payload slice just to move
///       (copy) it into the function and copy it once again into the buffer
///       verbatim.
pub fn data_frame<'ies, Config: DriverConfig>(
    ies: Option<IeReprList<'ies, IeRepr<'ies>>>,
    payload_length: u16,
    buffer_allocator: BufferAllocator,
) -> Result<MpduParser<MpduFrame, MpduWithAllFields>> {
    let data_frame_repr = match ies {
        Some(_ies) => {
            #[cfg(not(feature = "ies"))]
            panic!("not supported");
            #[cfg(feature = "ies")]
            DATA_FRAME_REPR.with_ies(IeListRepr::WithoutTerminationIes(_ies))
        }
        None => DATA_FRAME_REPR.without_ies(),
    };
    let min_buffer_size = data_frame_repr.min_buffer_size::<Config>(payload_length)?;
    let buffer = buffer_allocator
        .try_allocate_buffer(min_buffer_size)
        .unwrap();
    match data_frame_repr.into_writer::<Config>(
        FrameVersion::Ieee802154,
        FrameType::Data,
        payload_length,
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
