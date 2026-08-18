#![no_std]
#![cfg_attr(feature = "strict", deny(warnings))]
#![allow(dead_code)]

pub mod fields;
pub mod mpdu;
pub mod repr;

#[cfg(feature = "fuzz")]
pub mod fuzz;

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

        fn schedule_timed_signal_if(
            &self,
            _timed_signal: TimedSignal,
            _event: HardwareEvent,
        ) -> Result<&Self, RadioTimerError> {
            todo!()
        }

        fn commit_timed_signal(&self) {
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
    #[cfg(not(feature = "no-enh-ack-addressing"))]
    fn test_enh_ack_frame() {
        // FC(2) + SeqNr(1) + DstPanId(2) + DstExtAddr(8) + Time Correction
        // Header(2) + Time Correction Content(2), see ENH_ACK_MPDU_SIZE_WO_FCS.
        const ENH_ACK_LEN: u8 = 17;
        const TEST_DST: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        // Little-endian (on-air) byte order, i.e. PAN 0xBEEF.
        const TEST_DST_PAN_ID: [u8; 2] = [0xEF, 0xBE];

        const ENH_ACK_FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameSized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new()
                .with_sdu(NonZeroU16::new(ENH_ACK_LEN as u16).unwrap());
        const ENH_ACK_BUF_LEN: usize = ENH_ACK_FRAME_REPR.pdu_length() as usize;

        static mut BUFFER: [u8; ENH_ACK_BUF_LEN] = [0; ENH_ACK_BUF_LEN];
        #[allow(static_mut_refs)]
        let buffer = BufferToken::new(unsafe { &mut BUFFER });

        const TEST_SEQ_NUM: u8 = 55;
        let mut frame =
            enh_ack_frame::<FakeDriverConfig>(TEST_SEQ_NUM, &TEST_DST, &TEST_DST_PAN_ID, buffer);
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
            // PAN ID Compression cleared: the Destination PAN ID is carried.
            FrameType::Ack as u8,
            // IE Present + Enhanced + dst addressing mode extended
            2 | (FrameVersion::Ieee802154 as u8) << 4 | 0b11 << 2,
            TEST_SEQ_NUM,
            // Destination PAN ID, little endian (on-air order).
            TEST_DST_PAN_ID[0],
            TEST_DST_PAN_ID[1],
            // Destination extended address, little endian (on-air order).
            TEST_DST[0],
            TEST_DST[1],
            TEST_DST[2],
            TEST_DST[3],
            TEST_DST[4],
            TEST_DST[5],
            TEST_DST[6],
            TEST_DST[7],
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

        let addressing = reader.try_addressing_fields().unwrap();
        assert_eq!(addressing.try_dst_pan_id().unwrap().as_ref(), &TEST_DST_PAN_ID);
        assert!(addressing.try_src_pan_id().is_none());
        assert_eq!(addressing.try_dst_address().unwrap().as_le_bytes(), &TEST_DST);

        let ies = reader.ies_fields();
        let time_correction_ie = ies.time_correction().unwrap();
        assert!(time_correction_ie.time_sync() == 42);
        assert!(time_correction_ie.nack());

        let frame_buffer = reader.into_buffer();

        unsafe {
            frame_buffer.consume();
        }
    }

    #[test]
    fn test_mac_command_frame_version_0b10_addressing() {
        use crate::mpdu::{
            ASSOCIATE_REQUEST_FRAME_REPR, ASSOCIATE_RESPONSE_FRAME_REPR,
            ASSOCIATION_REQUEST_PAYLOAD_LENGTH,
        };

        const FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameUnsized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new();
        const MAX_BUFFER_LENGTH: usize = FRAME_REPR.max_buffer_length() as usize;

        static BUFFER: ConstStaticCell<[u8; MAX_BUFFER_LENGTH]> =
            ConstStaticCell::new([0; MAX_BUFFER_LENGTH]);
        let buffer = BufferToken::new(BUFFER.take());

        // Both associate command representations must be structurally valid
        // and consistent with frame version 0b10.
        for repr in [ASSOCIATE_REQUEST_FRAME_REPR, ASSOCIATE_RESPONSE_FRAME_REPR] {
            let addressing = repr.addressing.unwrap();
            assert!(addressing.is_compatible_with_frame_version(FrameVersion::Ieee802154));
            // Destination PAN ID present, source PAN ID omitted, extended
            // addresses (IEEE 802.15.4-2024, sections 10.21.5.1/10.21.5.2 and
            // Table 7-2).
            assert_eq!(addressing.try_addressing_fields_lengths(), Ok([2, 8, 0, 8]));
        }

        let mut writer = ASSOCIATE_REQUEST_FRAME_REPR
            .without_ies()
            .into_writer::<FakeDriverConfig>(
                FrameVersion::Ieee802154,
                FrameType::MacCommand,
                ASSOCIATION_REQUEST_PAYLOAD_LENGTH,
                buffer,
            )
            .unwrap();

        const DEVICE_ADDR: Address<[u8; 8]> =
            Address::Extended(ExtendedAddress::new_owned([1, 2, 3, 4, 5, 6, 7, 8]));
        const COORD_ADDR: Address<[u8; 8]> =
            Address::Extended(ExtendedAddress::new_owned([9, 10, 11, 12, 13, 14, 15, 16]));
        const PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0xBE, 0xEF]);

        // This is the access pattern that previously panicked.
        {
            let mut addressing = writer.addressing_fields_mut();
            addressing.dst_pan_id_mut().set(&PAN_ID);
            addressing.dst_address_mut().set(&COORD_ADDR);
            addressing.src_address_mut().set(&DEVICE_ADDR);
        }
        writer.set_sequence_number(7);

        // Verify the frame control field on the wire.
        let fc = writer.frame_control();
        assert_eq!(fc.frame_type(), FrameType::MacCommand);
        assert_eq!(fc.frame_version(), FrameVersion::Ieee802154);
        assert!(!fc.pan_id_compression());
        assert_eq!(fc.dst_addressing_mode(), AddressingMode::Extended);
        assert_eq!(fc.src_addressing_mode(), AddressingMode::Extended);

        // Re-parse the finished frame the way the receive path does and check
        // that all addressing fields read back consistently.
        let mpdu = writer.into_mpdu_frame();
        {
            let reader = mpdu
                .reader()
                .parse_addressing()
                .unwrap()
                .parse_security()
                .parse_ies::<FakeDriverConfig>()
                .unwrap();
            let fields = reader.try_addressing_fields().unwrap();
            assert_eq!(fields.try_dst_pan_id().unwrap().as_ref(), PAN_ID.as_ref());
            assert!(fields.try_src_pan_id().is_none());
            assert_eq!(
                fields.try_dst_address().unwrap().as_le_bytes(),
                COORD_ADDR.as_le_bytes()
            );
            assert_eq!(
                fields.try_src_address().unwrap().as_le_bytes(),
                DEVICE_ADDR.as_le_bytes()
            );
        }

        assert_eq!(mpdu.sequence_number(), Some(7));

        unsafe {
            mpdu.into_buffer().consume();
        }
    }

    /// Frame versions that contradict the PAN ID compression rule set of the
    /// addressing representation (or other version-dependent frame control
    /// invariants) must be rejected when building a frame.
    #[test]
    fn test_into_writer_rejects_inconsistent_frame_version() {
        const FRAME_REPR: RadioFrameRepr<FakeDriverConfig, RadioFrameUnsized> =
            RadioFrameRepr::<_, RadioFrameUnsized>::new();
        const MAX_BUFFER_LENGTH: usize = FRAME_REPR.max_buffer_length() as usize;

        static BUFFER: ConstStaticCell<[u8; MAX_BUFFER_LENGTH]> =
            ConstStaticCell::new([0; MAX_BUFFER_LENGTH]);
        let mut buffer = BufferToken::new(BUFFER.take());

        // Legacy PAN ID compression rules combined with frame version 0b10:
        // this is the combination that previously produced frames whose
        // buffer layout disagreed with their frame control field.
        const LEGACY_ADDRESSING: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
            .with_frame_control(SeqNrRepr::Yes)
            .with_addressing(AddressingRepr::new_legacy_addressing(
                AddressingMode::Extended,
                AddressingMode::Extended,
                true,
            ))
            .without_security()
            .without_ies();
        buffer = match LEGACY_ADDRESSING.into_writer::<FakeDriverConfig>(
            FrameVersion::Ieee802154,
            FrameType::MacCommand,
            2,
            buffer,
        ) {
            Err(buffer) => buffer,
            Ok(_) => panic!("legacy addressing with frame version 0b10 must be rejected"),
        };

        // IEEE 802.15.4-2015+ PAN ID compression rules combined with a legacy
        // frame version.
        const MODERN_ADDRESSING: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
            .with_frame_control(SeqNrRepr::Yes)
            .with_addressing(AddressingRepr::new(
                AddressingMode::Extended,
                AddressingMode::Extended,
                false,
                PanIdCompressionRepr::No,
            ))
            .without_security()
            .without_ies();
        buffer = match MODERN_ADDRESSING.into_writer::<FakeDriverConfig>(
            FrameVersion::Ieee802154_2006,
            FrameType::MacCommand,
            2,
            buffer,
        ) {
            Err(buffer) => buffer,
            Ok(_) => panic!("2015+ addressing with a legacy frame version must be rejected"),
        };

        // Sequence number suppression is only allowed for frame version 0b10
        // (IEEE 802.15.4-2024, section 7.2.2.7).
        const SUPPRESSED_SEQ_NR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
            .with_frame_control(SeqNrRepr::No)
            .without_addressing()
            .without_security()
            .without_ies();
        buffer = match SUPPRESSED_SEQ_NR.into_writer::<FakeDriverConfig>(
            FrameVersion::Ieee802154_2006,
            FrameType::Data,
            2,
            buffer,
        ) {
            Err(buffer) => buffer,
            Ok(_) => panic!("sequence number suppression with a legacy frame version must be rejected"),
        };

        // Reserved frame version.
        buffer = match MODERN_ADDRESSING.into_writer::<FakeDriverConfig>(
            FrameVersion::Unknown,
            FrameType::Data,
            2,
            buffer,
        ) {
            Err(buffer) => buffer,
            Ok(_) => panic!("a reserved frame version must be rejected"),
        };

        // The compatible combination succeeds with the same inputs.
        let writer = match MODERN_ADDRESSING.into_writer::<FakeDriverConfig>(
            FrameVersion::Ieee802154,
            FrameType::MacCommand,
            2,
            buffer,
        ) {
            Ok(writer) => writer,
            Err(_) => panic!("the compatible combination must succeed"),
        };

        unsafe {
            writer.into_buffer().consume();
        }
    }

    fn round_to_alignment(size: usize, alignment: usize) -> usize {
        assert!(alignment > 0 && ((alignment & (alignment - 1)) == 0));

        let size = size as isize;
        let alignment = alignment as isize;

        ((size + alignment - 1) & -alignment) as usize
    }
}
