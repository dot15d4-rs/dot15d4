use dot15d4_driver::radio::{
    frame::{FrameType, FrameVersion},
    DriverConfig,
};
use dot15d4_util::allocator::BufferToken;

#[cfg(feature = "ies")]
use dot15d4_driver::radio::frame::{IeListRepr, IeRepr, IeReprList};

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

/// Structural representation of an Enhanced ACK MPDU with Time Correction IE.
///
/// Enhanced ACK frame structure:
/// - Frame Control (2 bytes): IEEE 802.15.4-2015 version, ACK type, IE present
/// - Sequence Number (1 byte)
/// - Time Correction Header IE (4 bytes total):
///   - Header (2 bytes): Element ID = 0x1e, Length = 2
///   - Content (2 bytes): Time Sync Info + NACK bit
/// - HT2 Termination IE (2 bytes): Element ID = 0x7f, Length = 0
///
/// Total: 9 bytes (without FCS)
#[cfg(feature = "ies")]
pub const ENH_ACK_FRAME_REPR: MpduRepr<'static, MpduWithIes> = mpdu_repr()
    .with_frame_control(SeqNrRepr::Yes)
    .without_addressing()
    .without_security()
    .with_ies(IeListRepr::WithoutTerminationIes(IeReprList::new(
        ENH_ACK_IE_LIST,
    )));

/// Size of an Enhanced ACK MPDU without FCS.
///
/// Frame Control (2) + Seq Nr (1) + Time Correction IE Header (2) +
/// Time Correction IE Content (2) = 7 bytes
#[cfg(feature = "ies")]
pub const ENH_ACK_MPDU_SIZE_WO_FCS: u16 = {
    match ENH_ACK_FRAME_REPR.mpdu_length_wo_fcs(0) {
        Ok(len) => len.get(),
        _ => unreachable!(),
    }
};

#[cfg(feature = "ies")]
const _: () = {
    // FC(2) + SeqNr(1) + Time Correction Headr(2) + TimeCorrection Content(2) = 7
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
/// * `buffer` - Buffer token to hold the frame
///
/// # Returns
///
/// An MPDU parser with access to all fields including the Time Correction IE.
#[cfg(feature = "ies")]
pub fn enh_ack_frame<Config: DriverConfig>(
    seq_num: u8,
    buffer: BufferToken,
) -> MpduParser<MpduFrame, MpduWithAllFields> {
    // Safety: We give a valid configuration and therefore expect the operation
    //         not to fail.
    let mut ack_frame = ENH_ACK_FRAME_REPR
        .into_writer::<Config>(FrameVersion::Ieee802154, FrameType::Ack, 0, buffer)
        .unwrap();
    let _ = ack_frame.try_set_sequence_number(seq_num);

    // Initialize Time Correction IE to zero values
    if let Some(mut tc) = ack_frame.ies_fields_mut().time_correction_mut() {
        tc.set_time_sync(0);
        tc.set_nack(false);
    }

    ack_frame
}
