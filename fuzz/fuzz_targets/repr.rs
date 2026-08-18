#![no_main]

use arbitrary::Arbitrary;

use dot15d4_driver::radio::frame::{
    AddressingMode, AddressingRepr, FrameType, FrameVersion, IeListRepr, IeRepr, IeReprList,
    PanIdCompressionRepr,
};
use dot15d4_frame::fuzz::{FuzzDriverConfig, BUFFER_LENGTH};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_frame::repr::{MpduRepr, SeqNrRepr};
use dot15d4_util::allocator::{BufferToken, IntoBuffer};

use libfuzzer_sys::fuzz_target;

static mut BUF: [u8; BUFFER_LENGTH] = [0u8; BUFFER_LENGTH];

#[derive(Arbitrary, Debug)]
enum Mode {
    Absent,
    Short,
    Extended,
}

impl Mode {
    fn to_addressing_mode(&self) -> AddressingMode {
        match self {
            Mode::Absent => AddressingMode::Absent,
            Mode::Short => AddressingMode::Short,
            Mode::Extended => AddressingMode::Extended,
        }
    }
}

#[derive(Arbitrary, Debug)]
struct Spec {
    seq_nr: bool,
    dst: Mode,
    src: Mode,
    pan_ids_equal: bool,
    pan_id_compression: bool,
    beacon: bool,
    payload_len: u8,
    ie_time_correction: bool,
    ie_tsch_sync: bool,
    ie_tsch_timeslot: bool,
    ie_channel_hopping_channels: u8,
}

fuzz_target!(|spec: Spec| {
    let payload_len = (spec.payload_len % 32) as u16;

    let frame_type = if spec.beacon {
        FrameType::Beacon
    } else {
        FrameType::Data
    };
    let frame_version = FrameVersion::Ieee802154;

    let addressing = AddressingRepr::new(
        spec.dst.to_addressing_mode(),
        spec.src.to_addressing_mode(),
        spec.pan_ids_equal,
        if spec.pan_id_compression {
            PanIdCompressionRepr::Yes
        } else {
            PanIdCompressionRepr::No
        },
    );

    let mut ies = heapless::Vec::<_, 4>::new();
    if spec.ie_time_correction {
        let _ = ies.push(IeRepr::TimeCorrectionHeaderIe);
    }
    if spec.ie_tsch_sync {
        let _ = ies.push(IeRepr::TschSynchronizationNestedIe);
    }
    if spec.ie_tsch_timeslot {
        let _ = ies.push(IeRepr::FullTschTimeslotNestedIe);
    }
    let hop_channels = spec.ie_channel_hopping_channels % 8;
    if hop_channels > 0 {
        let _ = ies.push(IeRepr::FullChannelHoppingNestedIe(hop_channels, false));
    }
    let has_ies = !ies.is_empty();

    let repr = MpduRepr::new()
        .with_frame_control(if spec.seq_nr {
            SeqNrRepr::Yes
        } else {
            SeqNrRepr::No
        })
        .with_addressing(addressing)
        .without_security();

    // Safety: single-threaded fuzzer; the static is used by one run at a time.
    let backing: &'static mut [u8] = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
    backing.fill(0);
    let buffer = BufferToken::new(backing);

    let built = if has_ies {
        repr.with_ies(IeListRepr::WithoutTerminationIes(IeReprList::new(&ies[..])))
            .into_writer::<FuzzDriverConfig>(frame_version, frame_type, payload_len, buffer)
    } else {
        repr.without_ies().into_writer::<FuzzDriverConfig>(
            frame_version,
            frame_type,
            payload_len,
            buffer,
        )
    };

    let parsed = match built {
        Ok(parsed) => parsed,
        // The builder rejected the combination. Reclaim the buffer and stop.
        // Safety: the returned token owns the static buffer created above.
        Err(buffer) => {
            unsafe {
                let _ = buffer.consume();
            }
            return;
        }
    };

    let radio_frame = parsed.into_radio_frame::<FuzzDriverConfig>();
    let mpdu = MpduFrame::from_radio_frame(radio_frame);

    check_roundtrip(&mpdu, &spec, has_ies);

    // Safety: the token came from the static buffer created above.
    unsafe {
        let _ = mpdu.into_buffer().consume();
    }
});

fn check_roundtrip(mpdu: &MpduFrame, spec: &Spec, has_ies: bool) {
    let reader = mpdu.reader();

    assert_eq!(
        reader.frame_control().sequence_number_suppression(),
        !spec.seq_nr,
        "sequence number presence did not round-trip"
    );

    let reader = reader
        .parse_addressing()
        .expect("addressing built by the writer must parse");

    if !has_ies {
        return;
    }

    let reader = reader
        .parse_security()
        .parse_ies::<FuzzDriverConfig>()
        .expect("IEs built by the writer must parse");
    let ies = reader
        .try_ies_fields()
        .expect("IEs were built but the parser reports none present");

    if spec.ie_time_correction {
        assert!(
            ies.time_correction().is_some(),
            "Time Correction IE was built but not parsed"
        );
    }
    if spec.ie_tsch_sync {
        assert!(
            ies.tsch_sync().is_some(),
            "TSCH Synchronization IE was built but not parsed"
        );
    }
    if spec.ie_tsch_timeslot {
        assert!(
            ies.tsch_timeslot().is_some(),
            "TSCH Timeslot IE was built but not parsed"
        );
    }
    if (spec.ie_channel_hopping_channels % 8) > 0 {
        assert!(
            ies.channel_hopping().is_some(),
            "Channel Hopping IE was built but not parsed"
        );
    }
}
