use crate::{
    const_config::{MAC_IMPLICIT_BROADCAST, MAC_PAN_ID},
    tasks::PreliminaryFrameInfo,
};

use super::{Address, ShortAddress, BROADCAST_PAN_ID};

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
    if *dst_pan_id.as_ref() != *MAC_PAN_ID.as_ref() && dst_pan_id != BROADCAST_PAN_ID {
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
                Address::Extended(addr) => *hardware_addr == addr.into_be_bytes(),
                _ => false,
            }
        }
        _ => false,
    }
}
