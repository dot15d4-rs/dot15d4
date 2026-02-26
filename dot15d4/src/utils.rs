use dot15d4_driver::{
    radio::{
        frame::{
            Address, FrameType, PanId, RadioFrame, RadioFrameSized, RadioFrameUnsized,
            ShortAddress, BROADCAST_PAN_ID,
        },
        tasks::PreliminaryFrameInfo,
        DriverConfig,
    },
    timer::NsInstant,
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;

use crate::scheduler::{
    ReceptionType, SchedulerContext, SchedulerReceptionResult, SchedulerResponse,
};
use crate::{
    constants::{MAC_IMPLICIT_BROADCAST, MAC_PAN_ID},
    scheduler::MacCommandType,
};

/// Checks if the given MPDU is valid and intended for us. For the hardware
/// address, the full big-endian 64-bit address should be provided.
///
/// TODO: Implement the full incoming frame procedure here.
pub fn is_frame_valid_and_for_us(
    hardware_addr: &[u8; 8],
    preliminary_frame_info: &PreliminaryFrameInfo,
) -> bool {
    let PreliminaryFrameInfo {
        frame_control,
        addressing_fields,
        ..
    } = preliminary_frame_info;

    if frame_control.is_none() || addressing_fields.is_none() {
        return false;
    }

    let frame_control = frame_control.as_ref().unwrap();
    if !frame_control.is_valid() {
        return false;
    }

    let addressing_fields = addressing_fields.as_ref().unwrap();

    // Check destination PAN id.
    let dst_pan_id = addressing_fields
        .try_dst_pan_id()
        .unwrap_or(BROADCAST_PAN_ID);
    // TODO: get current PAN ID from PIB instead of default value
    let pan_id = PanId::new_owned(MAC_PAN_ID.to_le_bytes());
    if *dst_pan_id.as_ref() != *pan_id.as_ref() && dst_pan_id != BROADCAST_PAN_ID {
        return false;
    }

    // Check destination address.
    let dst_addr = addressing_fields.try_dst_address();
    match dst_addr {
        Some(dst_addr) => {
            if dst_addr == Address::<&[u8]>::BROADCAST_ADDR {
                return true;
            }

            match dst_addr {
                Address::Absent if MAC_IMPLICIT_BROADCAST => true,
                Address::Short(addr) => {
                    // Convert a little-endian short address from the big-endian
                    // hardware address.
                    // TODO: This is not a valid method to generate short addresses.
                    let mut derived_short_address =
                        <[u8; 2]>::try_from(&hardware_addr[6..]).unwrap();
                    derived_short_address.reverse();
                    let derived_short_addr = ShortAddress::new_owned(derived_short_address);
                    *derived_short_addr.as_ref() == *addr.as_ref()
                }
                Address::Extended(addr) => hardware_addr == addr.as_ref(),
                _ => false,
            }
        }
        _ => false,
    }
}

pub fn process_rx_frame<RadioDriverImpl: DriverConfig>(
    rx_frame: RadioFrame<RadioFrameSized>,
    instant: NsInstant,
    context: &mut SchedulerContext<RadioDriverImpl>,
) -> Result<(SchedulerResponse, ResponseToken), RadioFrame<RadioFrameUnsized>> {
    let mpdu_frame = MpduFrame::from_radio_frame(rx_frame);
    let reader = mpdu_frame.reader();

    match reader.frame_control().frame_type() {
        FrameType::Beacon => {
            if let Some((token, _)) = context.try_receive_rx_request(ReceptionType::Beacon) {
                let response = SchedulerResponse::Reception(SchedulerReceptionResult::Beacon(
                    mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                    instant,
                ));
                Ok((response, token))
            } else {
                // No receiver, reuse frame
                Err(mpdu_frame
                    .into_radio_frame::<RadioDriverImpl>()
                    .forget_size::<RadioDriverImpl>())
            }
        }
        FrameType::Data => {
            if let Some((token, _)) = context.try_receive_rx_request(ReceptionType::Data) {
                let response = SchedulerResponse::Reception(SchedulerReceptionResult::Data(
                    mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                    instant,
                ));
                Ok((response, token))
            } else {
                // No receiver, reuse frame
                Err(mpdu_frame
                    .into_radio_frame::<RadioDriverImpl>()
                    .forget_size::<RadioDriverImpl>())
            }
        }
        FrameType::MacCommand => {
            // TODO: handle unwrap()
            let reader = reader
                .parse_addressing()
                .unwrap()
                .parse_security()
                .parse_ies::<RadioDriverImpl>()
                .unwrap();
            let payload = reader.try_frame_payload().unwrap();
            // TODO: use enum
            let mac_command_type = match payload[0] {
                0x01 => MacCommandType::AssociateRequest,
                0x02 => MacCommandType::AssociateResponse,
                _ => todo!(),
            };
            // Association Request
            if let Some((token, _)) =
                context.try_receive_rx_request(ReceptionType::MacCommand(mac_command_type))
            {
                let response = SchedulerResponse::Reception(SchedulerReceptionResult::Command(
                    mpdu_frame.into_radio_frame::<RadioDriverImpl>(),
                    instant,
                ));
                Ok((response, token))
            } else {
                // No receiver, reuse frame
                Err(mpdu_frame
                    .into_radio_frame::<RadioDriverImpl>()
                    .forget_size::<RadioDriverImpl>())
            }
        }
        _ => todo!(),
    }
}
