use core::sync::atomic::{AtomicU8, Ordering};

use dot15d4::{
    driver::radio::{
        frame::{
            Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
            PanId, RadioFrame,
        },
        phy::PhyConfig,
        tasks::{TaskRx, TaskTx},
        DriverConfig, PhyOf,
    },
    mac::frame::{
        repr::{MpduRepr, SeqNrRepr},
        MpduWithIes,
    },
    util::allocator::BufferAllocator,
};

// PAN ID: 7B:3C
const PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0x3C, 0x7B]);

// Address 1: 02:1A:7D:00:00:8F:12:34
const ADDR1: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0x34, 0x12, 0x8F, 0x00, 0x00, 0x7D, 0x1A, 0x02,
]));

// Address 2: 02:1A:7D:00:00:8F:56:78
const ADDR2: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0x78, 0x56, 0x8F, 0x00, 0x00, 0x7D, 0x1A, 0x02,
]));

const PAYLOAD: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];

const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new_legacy_addressing(
        AddressingMode::Extended,
        AddressingMode::Extended,
        true,
    ))
    .without_security()
    .without_ies();

static SEQ_NR: AtomicU8 = AtomicU8::new(0);

#[allow(dead_code)]
pub fn rx_task<Config: DriverConfig>(buffer_allocator: BufferAllocator) -> TaskRx {
    let radio_frame = RadioFrame::new::<Config>(
        buffer_allocator
            .try_allocate_buffer(<PhyOf<Config> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize)
            .unwrap(),
    );

    TaskRx { radio_frame }
}

#[allow(dead_code)]
pub fn tx_task<Config: DriverConfig>(cca: bool, buffer_allocator: BufferAllocator) -> TaskTx {
    let mut mpdu = MPDU_REPR
        .into_writer::<Config>(
            FrameVersion::Ieee802154_2006,
            FrameType::Data,
            PAYLOAD.len() as u16,
            buffer_allocator
                .try_allocate_buffer(<PhyOf<Config> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize)
                .unwrap(),
        )
        .unwrap();

    mpdu.set_sequence_number(SEQ_NR.fetch_add(1, Ordering::Relaxed));
    let mut addressing = mpdu.addressing_fields_mut();
    addressing.src_address_mut().set(&ADDR1);
    addressing.dst_address_mut().set(&ADDR2);
    addressing.dst_pan_id_mut().set(&PAN_ID);
    mpdu.frame_payload_mut().copy_from_slice(&PAYLOAD);
    let radio_frame = mpdu.into_radio_frame::<Config>();

    TaskTx { radio_frame, cca }
}
