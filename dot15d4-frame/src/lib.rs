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

/// A marker trait that subsumes all MPDU states that provide access to
/// Information Elements fields.
pub trait MpduParsedUpToIes {}
impl MpduParsedUpToIes for MpduWithAllFields {}

#[cfg(test)]
mod test {
    use core::{marker::PhantomData, num::NonZeroU16};

    use dot15d4_driver::{
        radio::{
            frame::{
                Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
                IeListRepr, IeRepr, IeReprList, PanId, PanIdCompressionRepr, RadioFrame,
                RadioFrameRepr, RadioFrameSized, RadioFrameUnsized,
            },
            phy::{OQpsk250KBit, Phy},
            DriverConfig, FcsTwoBytes,
        },
        timer::{
            HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
            OptionalNsInstant, RadioTimerApi, RadioTimerError, TimedSignal,
        },
    };
    use dot15d4_util::allocator::{BufferToken, IntoBuffer};
    use static_cell::ConstStaticCell;

    #[cfg(feature = "security")]
    use crate::repr::{KeyIdRepr, SecurityLevelRepr, SecurityRepr};
    use crate::{
        mpdu::{enh_ack_frame, imm_ack_frame, MpduFrame},
        repr::{MpduRepr, SeqNrRepr},
        MpduWithIes,
    };
    #[derive(Clone, Copy)]
    struct FakeRadioTimer<State: Clone> {
        state: PhantomData<State>,
    }
    #[derive(Clone, Copy)]
    struct Sleeping;
    #[derive(Clone, Copy)]
    struct Running;

    impl RadioTimerApi for FakeRadioTimer<Sleeping> {
        const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);
        const GUARD_TIME: NsDuration = NsDuration::from_ticks(0);

        type HighPrecisionTimer = FakeRadioTimer<Running>;

        fn now(&self) -> NsInstant {
            todo!()
        }

        async unsafe fn wait_until(&mut self, _instant: NsInstant) -> Result<(), RadioTimerError> {
            todo!()
        }

        fn start_high_precision_timer(
            &self,
            _start_at: OptionalNsInstant,
        ) -> Result<Self::HighPrecisionTimer, RadioTimerError> {
            todo!()
        }
    }

    impl HighPrecisionTimer for FakeRadioTimer<Running> {
        const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);

        fn schedule_timed_signal(
            &self,
            _timed_signal: TimedSignal,
        ) -> Result<&Self, RadioTimerError> {
            todo!()
        }

        fn schedule_timed_signal_unless(
            &self,
            _timed_signal: TimedSignal,
            _event: HardwareEvent,
        ) -> Result<&Self, RadioTimerError> {
            todo!()
        }

        async unsafe fn wait_for(&mut self, _signal: HardwareSignal) {
            todo!()
        }

        fn observe_event(&self, _event: HardwareEvent) -> Result<&Self, RadioTimerError> {
            todo!()
        }

        fn poll_event(&self, _event: HardwareEvent) -> OptionalNsInstant {
            todo!()
        }

        fn reset(&self) {
            todo!()
        }
    }

    struct FakeDriverConfig;
    impl DriverConfig for FakeDriverConfig {
        const HEADROOM: u8 = 1;
        const TAILROOM: u8 = 2;
        const MAX_SDU_LENGTH: u16 = 127;
        type Fcs = FcsTwoBytes;
        type Timer = FakeRadioTimer<Sleeping>;
        type Phy = Phy<OQpsk250KBit>;
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
            .into_writer::<FakeDriverConfig>(
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
    fn test_tsch() {
        const MPDU_REPR: MpduRepr<'_, MpduWithIes> = const {
            let mpdu_repr = MpduRepr::new();

            let mpdu_repr =
                mpdu_repr
                    .with_frame_control(SeqNrRepr::No)
                    .with_addressing(AddressingRepr::new(
                        AddressingMode::Absent,
                        AddressingMode::Extended,
                        false,
                        PanIdCompressionRepr::No,
                    ));

            let mpdu_repr = mpdu_repr.without_security();

            {
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
            }

            // mpdu_repr.without_ies()
        };

        const FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameUnsized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new();
        const MAX_BUFFER_LENGTH: usize = FRAME_REPR.max_buffer_length() as usize;

        static BUFFER: ConstStaticCell<[u8; MAX_BUFFER_LENGTH]> =
            ConstStaticCell::new([0; MAX_BUFFER_LENGTH]);
        let buffer = BufferToken::new(BUFFER.take());

        let mut mpdu_writer = MPDU_REPR
            .into_writer::<FakeDriverConfig>(FrameVersion::Ieee802154, FrameType::Beacon, 0, buffer)
            .unwrap();

        // DEVICE_ID: FC36 5A7D AF1F D6FE (SN: ...7064)
        const SERVER_MAC_ADDR: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
            0xfe, 0xd6, 0x1f, 0xaf, 0x7d, 0x5a, 0x36, 0xfc,
        ]));
        // DEVICE_ID: 74EB 0174 27E3 04D2 (SN: ...2182)
        const CLIENT_MAC_ADDR: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
            0xD2, 0x04, 0xE3, 0x27, 0x74, 0x01, 0xEB, 0x74,
        ]));
        pub const MAC_PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0xBE, 0xEF]); // PAN Id

        let mut addressing = mpdu_writer.addressing_fields_mut();
        addressing.src_address_mut().set(&SERVER_MAC_ADDR);
        addressing.src_pan_id_mut().set(&MAC_PAN_ID);

        let _ies = mpdu_writer.ies_fields_mut();

        unsafe {
            mpdu_writer.into_buffer().consume();
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

    #[test]
    fn test_enh_ack_frame() {
        const ENH_ACK_LEN: u8 = 7;

        const ENH_ACK_FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameSized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new()
                .with_sdu(NonZeroU16::new(ENH_ACK_LEN as u16).unwrap());
        const ENH_ACK_BUF_LEN: usize = ENH_ACK_FRAME_REPR.pdu_length() as usize;

        static mut BUFFER: [u8; ENH_ACK_BUF_LEN] = [0; ENH_ACK_BUF_LEN];
        #[allow(static_mut_refs)]
        let buffer = BufferToken::new(unsafe { &mut BUFFER });

        const TEST_SEQ_NUM: u8 = 55;
        let mut frame = enh_ack_frame::<FakeDriverConfig>(TEST_SEQ_NUM, buffer);
        let mut ies = frame.ies_fields_mut();
        let mut time_correction_ie = ies.time_correction_mut().unwrap();
        time_correction_ie.set_time_sync(42);
        time_correction_ie.set_nack(true);

        assert_eq!(
            ENH_ACK_BUF_LEN as u8,
            ENH_ACK_FRAME_REPR.driver_overhead() + ENH_ACK_LEN + ENH_ACK_FRAME_REPR.fcs_length()
        );

        let expected_buffer = [
            0,
            FrameType::Ack as u8,
            2 | (FrameVersion::Ieee802154 as u8) << 4, // IE Present + Enhanced
            TEST_SEQ_NUM,
            2,      // Header IE Time sync
            15,     // Header IE Time sync
            42,     // time sync value
            1 << 7, // Header IE Time sync + Nack
            0,
            0,
            0,
            0,
        ];
        let frame_buffer = frame.into_buffer();
        assert_eq!(frame_buffer.as_ref(), &expected_buffer);

        // Now we try to parse the buffer

        let radio_frame = RadioFrame::new::<FakeDriverConfig>(frame_buffer)
            .with_size(NonZeroU16::new(ENH_ACK_LEN as u16).unwrap());
        let mpdu = MpduFrame::from_radio_frame(radio_frame);

        let reader = mpdu
            .into_parser()
            .parse_addressing()
            .unwrap()
            .parse_security()
            .parse_ies::<FakeDriverConfig>()
            .unwrap();

        let ies = reader.ies_fields();
        let time_correction_ie = ies.time_correction().unwrap();
        assert!(time_correction_ie.time_sync() == 42);
        assert!(time_correction_ie.nack());

        let frame_buffer = reader.into_buffer();

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
