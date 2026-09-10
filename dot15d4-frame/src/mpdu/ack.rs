use dot15d4_driver::radio::{
    frame::{FrameType, FrameVersion},
    DriverConfig,
};
use dot15d4_util::allocator::BufferToken;

#[cfg(feature = "ies")]
use dot15d4_driver::radio::frame::{IeListRepr, IeRepr, IeReprList};
#[cfg(all(feature = "ies", not(feature = "no-enh-ack-addressing")))]
use dot15d4_driver::radio::frame::{
    Address, AddressingMode, AddressingRepr, ExtendedAddress, PanId, PanIdCompressionRepr,
};

use crate::{
    fields::MpduParser,
    mpdu::MpduFrame,
    repr::{mpdu_repr, MpduRepr, SeqNrRepr},
    MpduWithAllFields, MpduWithIes,
};

/// Structural representation of an ImmAck MPDU.
pub const IMM_ACK_FRAME_REPR: MpduRepr<MpduWithIes> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .without_addressing()
    .without_security()
    .without_ies();

/// Size of an ImmAck MPDU without FCS.
pub const ACK_MPDU_SIZE_WO_FCS: u16 = {
    match IMM_ACK_FRAME_REPR.mpdu_length_wo_fcs(0) {
        Ok(len) => len.get(),
        _ => unreachable!(),
    }
};

const _: () = {
    assert!(ACK_MPDU_SIZE_WO_FCS == 3);
};

/// Instantiates a reader/writer for an ImmAck frame with the given buffer and
/// initializes it.
pub fn imm_ack_frame<Config: DriverConfig>(
    seq_num: u8,
    buffer: BufferToken,
) -> MpduParser<MpduFrame, MpduWithAllFields> {
    // Safety: We give a valid configuration and therefore expect the operation
    //         not to fail.
    let mut ack_frame = IMM_ACK_FRAME_REPR
        .into_writer::<Config>(FrameVersion::Ieee802154_2006, FrameType::Ack, 0, buffer)
        .unwrap();
    let _ = ack_frame.try_set_sequence_number(seq_num);
    ack_frame
}

/// Static IE list for Enhanced ACK containing only Time Correction IE.
#[cfg(feature = "ies")]
pub const ENH_ACK_IE_LIST: &[IeRepr<'static>] = &[IeRepr::TimeCorrectionHeaderIe];

#[cfg(all(feature = "ies", not(feature = "no-enh-ack-addressing")))]
pub const ENH_ACK_FRAME_REPR: MpduRepr<'static, MpduWithIes> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new(
        AddressingMode::Extended,
        AddressingMode::Absent,
        false,
        PanIdCompressionRepr::No,
    ))
    .without_security()
    .with_ies(IeListRepr::WithoutTerminationIes(IeReprList::new(
        ENH_ACK_IE_LIST,
    )));

/// Unaddressed Enhanced ACK, matched by sequence number alone.
///
/// See the `no-enh-ack-addressing` feature: this reproduces the acknowledgment
/// misattribution described above and is not for production use.
#[cfg(all(feature = "ies", feature = "no-enh-ack-addressing"))]
pub const ENH_ACK_FRAME_REPR: MpduRepr<'static, MpduWithIes> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .without_addressing()
    .without_security()
    .with_ies(IeListRepr::WithoutTerminationIes(IeReprList::new(
        ENH_ACK_IE_LIST,
    )));

/// Size of an Enhanced ACK MPDU without FCS.
///
/// Frame Control (2) + Seq Nr (1) + Dst Extended Address (8, unless
/// `no-enh-ack-addressing`) + Time Correction IE Header (2) + Time Correction
/// IE Content (2)
#[cfg(feature = "ies")]
pub const ENH_ACK_MPDU_SIZE_WO_FCS: u16 = {
    match ENH_ACK_FRAME_REPR.mpdu_length_wo_fcs(0) {
        Ok(len) => len.get(),
        _ => unreachable!(),
    }
};

#[cfg(all(feature = "ies", not(feature = "no-enh-ack-addressing")))]
const _: () = {
    // FC(2) + SeqNr(1) + DstExtAddr(8) + Time Correction Header(2) +
    // TimeCorrection Content(2) = 15
    assert!(ENH_ACK_MPDU_SIZE_WO_FCS == 17);
};

#[cfg(all(feature = "ies", feature = "no-enh-ack-addressing"))]
const _: () = {
    // FC(2) + SeqNr(1) + Time Correction Header(2) + TimeCorrection Content(2) = 7
    assert!(ENH_ACK_MPDU_SIZE_WO_FCS == 7);
};

/// Instantiates a reader/writer for an Enhanced ACK frame with Time Correction IE.
///
/// The Time Correction IE will be initialized with time_sync = 0 and nack = false.
/// The caller should use `ies_fields_mut().time_correction_mut()` to set the
/// actual values before transmission.
///
/// # Arguments
///
/// * `seq_num` - The sequence number to ACK
/// * `dst_le_bytes` - Extended address of the acknowledged frame's originator,
///   in little-endian (on-air) byte order
/// * `dst_pan_id_le_bytes` - PAN of the acknowledged frame, in little-endian
///   (on-air) byte order. The addressing representation always reserves this
///   field, so leaving it unwritten puts a zero PAN ID on air, which
///   conformant receivers reject.
/// * `buffer` - Buffer token to hold the frame
///
/// # Returns
///
/// An MPDU parser with access to all fields including the Time Correction IE.
#[cfg(feature = "ies")]
pub fn enh_ack_frame<Config: DriverConfig>(
    seq_num: u8,
    _dst_le_bytes: &[u8; 8],
    _dst_pan_id_le_bytes: &[u8; 2],
    buffer: BufferToken,
) -> MpduParser<MpduFrame, MpduWithAllFields> {
    // Safety: We give a valid configuration and therefore expect the operation
    //         not to fail.
    let mut ack_frame = ENH_ACK_FRAME_REPR
        .into_writer::<Config>(FrameVersion::Ieee802154, FrameType::Ack, 0, buffer)
        .unwrap();
    let _ = ack_frame.try_set_sequence_number(seq_num);

    // Address the acknowledgment to the originator of the acknowledged frame,
    // in the PAN that frame was sent in.
    #[cfg(not(feature = "no-enh-ack-addressing"))]
    {
        let dst = Address::Extended(ExtendedAddress::new_owned(*_dst_le_bytes));
        let mut addressing = ack_frame.addressing_fields_mut();
        addressing.dst_address_mut().set(&dst);
        addressing
            .dst_pan_id_mut()
            .set(&PanId::new_owned(*_dst_pan_id_le_bytes));
    }

    // Initialize Time Correction IE to zero values
    if let Some(mut tc) = ack_frame.ies_fields_mut().time_correction_mut() {
        tc.set_time_sync(0);
        tc.set_nack(false);
    }

    ack_frame
}
