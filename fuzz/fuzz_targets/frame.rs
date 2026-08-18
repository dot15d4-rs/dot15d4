#![no_main]

use core::num::NonZero;

use dot15d4_frame::fuzz::{FuzzDriverConfig, BUFFER_LENGTH, MAX_MPDU_LENGTH_WO_FCS};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::allocator::{BufferToken, IntoBuffer};

use libfuzzer_sys::{fuzz_target, Corpus};

const HEADROOM: usize = 1;

static mut BUF: [u8; BUFFER_LENGTH] = [0u8; BUFFER_LENGTH];

fuzz_target!(|data: &[u8]| -> Corpus {
    let n = data.len();
    if n == 0 || n > MAX_MPDU_LENGTH_WO_FCS {
        return Corpus::Reject;
    }

    // Copy the bytes into the MPDU region of a zeroed, max-length buffer.
    // Safety: the fuzzer is single-threaded, so this static is used by one
    // iteration at a time. The slice is `'static` and never moves.
    let backing: &'static mut [u8] = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
    backing.fill(0);
    backing[HEADROOM..HEADROOM + n].copy_from_slice(data);

    let token = BufferToken::new(backing);
    // Safety: the buffer is exactly the max buffer length, as an incoming frame
    // requires.
    let mpdu = unsafe { MpduFrame::new(token, HEADROOM as u8, NonZero::new(n as u16).unwrap()) };

    read_all(&mpdu);

    // Hand the buffer back to its owner (this target). This consumes the
    // linear token without going through a pool, and reclaims the static.
    // Safety: the token was created here from this static buffer.
    unsafe {
        let _ = mpdu.into_buffer().consume();
    }

    Corpus::Keep
});

fn read_all(mpdu: &MpduFrame) {
    let fc = mpdu.frame_control();
    let _ = fc.frame_type();
    let _ = fc.frame_version();
    let _ = fc.security_enabled();
    let _ = fc.ack_request();
    let _ = fc.pan_id_compression();
    let _ = fc.sequence_number_suppression();
    let _ = fc.information_elements_present();
    let _ = fc.is_valid();
    let _ = mpdu.sequence_number();

    let reader = mpdu.reader();
    let reader = match reader.parse_addressing() {
        Ok(reader) => reader,
        Err(_) => return,
    };

    if let Ok(addressing) = reader.try_addressing_fields() {
        let _ = addressing.try_dst_pan_id();
        let _ = addressing.try_dst_address();
        let _ = addressing.try_src_address();
    }

    if reader.frame_control().security_enabled() {
        return;
    }

    let reader = reader.parse_security();
    let reader = match reader.parse_ies::<FuzzDriverConfig>() {
        Ok(reader) => reader,
        Err(_) => return,
    };

    if let Some(ies) = reader.try_ies_fields() {
        if let Some(tc) = ies.time_correction() {
            let _ = tc.time_sync();
            let _ = tc.nack();
        }
        if let Some(sync) = ies.tsch_sync() {
            let _ = sync.asn();
            let _ = sync.join_metric();
        }
        if let Some(hopping) = ies.channel_hopping() {
            let _ = hopping.is_reduced();
            if let Some(list) = hopping.hopping_sequence_list() {
                let _ = list.iter().fold(0u8, |acc, &b| acc ^ b);
            }
        }
        if let Some(timeslot) = ies.tsch_timeslot() {
            if !timeslot.is_reduced() {
                assert!(
                    timeslot.tx_offset().is_some()
                        && timeslot.rx_offset().is_some()
                        && timeslot.rx_wait().is_some()
                        && timeslot.timeslot_length().is_some(),
                    "a non-reduced Timeslot IE must expose all fields"
                );
            }
        }
    }

    let _ = reader.try_frame_payload();
}
