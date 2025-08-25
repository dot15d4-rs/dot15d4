use dot15d4_driver::{
    frame::{FrameType, FrameVersion},
    radio::DriverConfig,
};
use dot15d4_util::allocator::BufferToken;

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

/// Instantiates a reader/writer for an ImmAck frame with the given buffer and
/// initializes it.
pub fn imm_ack_frame<Config: DriverConfig>(
    seq_num: u8,
    buffer: BufferToken,
) -> MpduParser<MpduFrame, MpduWithAllFields> {
    // Safety: We give a valid configuration and therefore expect the operation
    //         not to fail.
    let mut ack_frame = IMM_ACK_FRAME_REPR
        .into_parsed_mpdu::<Config>(FrameVersion::Ieee802154_2006, FrameType::Ack, 0, buffer)
        .unwrap();
    let _ = ack_frame.try_set_sequence_number(seq_num);
    ack_frame
}
