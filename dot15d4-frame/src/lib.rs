#![no_std]
#![cfg_attr(feature = "strict", deny(warnings))]
#![allow(dead_code)]

pub mod fields;
pub mod mpdu;
pub mod repr;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduNoFields;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduWithFrameControl;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduWithAddressing;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduWithSecurity;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduWithIes;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct MpduWithAllFields;

/// A marker trait that subsumes all MPDU states that provide access to
/// addressing fields.
pub trait MpduParsedUpToAddressing {}
impl MpduParsedUpToAddressing for MpduWithAddressing {}
impl MpduParsedUpToAddressing for MpduWithSecurity {}
impl MpduParsedUpToAddressing for MpduWithAllFields {}

/// A marker trait that subsumes all MPDU states that provide access to
/// security-related fields.
pub trait MpduParsedUpToSecurity {}
impl MpduParsedUpToSecurity for MpduWithSecurity {}
impl MpduParsedUpToSecurity for MpduWithAllFields {}

#[cfg(test)]
mod test {
    use core::num::NonZeroU16;

    use dot15d4_driver::{
        constants::PHY_MAX_PACKET_SIZE_127,
        frame::{
            AddressingMode, AddressingRepr, FrameType, FrameVersion, PanIdCompressionRepr,
            RadioFrameRepr, RadioFrameSized, RadioFrameUnsized,
        },
        radio::{DriverConfig, FcsTwoBytes},
        timer::{HardwareSignal, RadioTimerApi, RadioTimerResult, SyntonizedInstant},
    };
    use dot15d4_util::allocator::{BufferToken, IntoBuffer};
    use static_cell::ConstStaticCell;
    use typenum::{U, U1, U2};

    #[cfg(feature = "ies")]
    use crate::repr::{IeListRepr, IeRepr, IeReprList};
    #[cfg(feature = "security")]
    use crate::repr::{KeyIdRepr, SecurityLevelRepr, SecurityRepr};
    use crate::{
        mpdu::imm_ack_frame,
        repr::{MpduRepr, SeqNrRepr},
        MpduWithIes,
    };
    #[derive(Clone, Copy)]
    struct FakeRadioTimer;
    impl RadioTimerApi for FakeRadioTimer {
        fn now(&self) -> SyntonizedInstant {
            todo!()
        }

        async unsafe fn wait_until(&self, _: SyntonizedInstant) -> RadioTimerResult {
            todo!()
        }

        async unsafe fn schedule_event(
            &self,
            _: SyntonizedInstant,
            _: HardwareSignal,
        ) -> RadioTimerResult {
            todo!()
        }
    }

    struct FakeDriverConfig;
    impl DriverConfig for FakeDriverConfig {
        type Headroom = U1;
        type Tailroom = U2;
        type MaxSduLength = U<PHY_MAX_PACKET_SIZE_127>;
        type Fcs = FcsTwoBytes;
        type Timer = FakeRadioTimer;
    }

    #[test]
    fn test_mpdu_repr_api_and_size() {
        const MPDU_REPR: MpduRepr<'static, MpduWithIes> = const {
            let mpdu_repr = MpduRepr::new();

            let mpdu_repr = mpdu_repr
                .with_frame_control(SeqNrRepr::Yes)
                .with_addressing(AddressingRepr::new(
                    AddressingMode::Short,
                    AddressingMode::Short,
                    true,
                    PanIdCompressionRepr::Yes,
                ));

            #[cfg(feature = "security")]
            let mpdu_repr = mpdu_repr.with_security(SecurityRepr::new(
                false,
                SecurityLevelRepr::EncMic32,
                KeyIdRepr::Source4Byte,
            ));

            #[cfg(not(feature = "security"))]
            let mpdu_repr = mpdu_repr.without_security();

            #[cfg(feature = "ies")]
            let mpdu_repr = {
                static SLOTFRAMES: [u8; 3] = [2, 3, 4];
                static IES: [IeRepr; 3] = [
                    IeRepr::TimeCorrectionHeaderIe,
                    IeRepr::FullTschTimeslotNestedIe,
                    IeRepr::TschSlotframeAndLinkNestedIe(&SLOTFRAMES),
                ];
                static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
                static IE_LIST: IeListRepr<'static> =
                    IeListRepr::WithoutTerminationIes(IE_REPR_LIST);
                mpdu_repr.with_ies(IE_LIST)
            };

            #[cfg(not(feature = "ies"))]
            let mpdu_repr = mpdu_repr.without_ies();

            mpdu_repr
        };

        #[cfg(all(not(feature = "ies"), not(feature = "security")))]
        assert_eq!(size_of_val(&MPDU_REPR), 5);

        #[cfg(all(feature = "security", not(feature = "ies")))]
        assert_eq!(size_of_val(&MPDU_REPR), 8);

        #[cfg(all(feature = "ies", not(feature = "security")))]
        assert_eq!(size_of_val(&MPDU_REPR), 32);

        #[cfg(all(feature = "security", feature = "ies"))]
        assert_eq!(size_of_val(&MPDU_REPR), 32);

        const FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameUnsized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new();
        const MAX_BUFFER_LENGTH: usize = FRAME_REPR.max_buffer_length() as usize;

        static BUFFER: ConstStaticCell<[u8; MAX_BUFFER_LENGTH]> =
            ConstStaticCell::new([0; MAX_BUFFER_LENGTH]);
        let buffer = BufferToken::new(BUFFER.take());

        const PAYLOAD_LENGTH: u16 = 5;
        let parsed_mpdu = MPDU_REPR
            .into_parsed_mpdu::<FakeDriverConfig>(
                FrameVersion::Ieee802154,
                FrameType::Data,
                PAYLOAD_LENGTH,
                buffer,
            )
            .unwrap();

        #[cfg(not(any(feature = "security", feature = "ies")))]
        assert_eq!(size_of_val(&parsed_mpdu), 32);

        #[cfg(any(feature = "security", feature = "ies"))]
        assert_eq!(size_of_val(&parsed_mpdu), 40);

        unsafe {
            parsed_mpdu.into_buffer().consume();
        }
    }

    #[test]
    fn test_imm_ack_frame() {
        const IMM_ACK_LEN: u8 = 3;

        const IMM_ACK_FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameSized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new()
                .with_sdu(NonZeroU16::new(IMM_ACK_LEN as u16).unwrap());
        const IMM_ACK_BUF_LEN: usize = IMM_ACK_FRAME_REPR.pdu_length() as usize;

        static mut BUFFER: [u8; IMM_ACK_BUF_LEN] = [0; IMM_ACK_BUF_LEN];
        #[allow(static_mut_refs)]
        let buffer = BufferToken::new(unsafe { &mut BUFFER });

        const TEST_SEQ_NUM: u8 = 55;
        let frame = imm_ack_frame::<FakeDriverConfig>(TEST_SEQ_NUM, buffer);

        assert_eq!(
            IMM_ACK_BUF_LEN as u8,
            IMM_ACK_FRAME_REPR.driver_overhead() + IMM_ACK_LEN + IMM_ACK_FRAME_REPR.fcs_length()
        );

        let expected_buffer = [
            0,
            FrameType::Ack as u8,
            (FrameVersion::Ieee802154_2006 as u8) << 4,
            TEST_SEQ_NUM,
            0,
            0,
            0,
            0,
        ];
        let frame_buffer = frame.into_buffer();
        assert_eq!(frame_buffer.as_ref(), &expected_buffer);

        unsafe {
            frame_buffer.consume();
        }
    }

    fn round_to_alignment(size: usize, alignment: usize) -> usize {
        assert!(alignment > 0 && ((alignment & (alignment - 1)) == 0));

        let size = size as isize;
        let alignment = alignment as isize;

        ((size + alignment - 1) & -alignment) as usize
    }
}
