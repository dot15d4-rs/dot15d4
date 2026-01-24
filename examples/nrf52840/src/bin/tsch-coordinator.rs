#![no_std]
#![no_main]

use dot15d4::{
    driver::{
        radio::{
            frame::{
                Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
                PanId,
            },
            tasks::{RadioDriverApi, TaskOff},
            RadioDriver,
        },
        socs::nrf::{NrfRadioDriver, NrfRadioSleepTimer},
        timer::{NsDuration, RadioTimerApi},
        DriverEventChannel, DriverEventReceiver, DriverEventSender, DriverRequestChannel,
        DriverRequestReceiver, DriverRequestSender, DriverService,
    },
    mac::{
        frame::{
            repr::{MpduRepr, SeqNrRepr},
            MpduWithIes,
        },
        primitives::{DataRequest, MacRequest},
        MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender,
        MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, MAC_BUFFER_SIZE,
    },
    scheduler::{
        SchedulerRequestChannel, SchedulerRequestReceiver, SchedulerRequestSender, SchedulerService,
    },
    util::{
        allocator::{BufferToken, IntoBuffer},
        buffer_allocator, info,
    },
    RngCore, RngError,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, AvailableResources};
use embassy_executor::Spawner;
use static_cell::StaticCell;

// DEVICE_ID: FC36 5A7D AF1F D6FE (SN: ...7064)
const SERVER_MAC_ADDR: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0xfe, 0xd6, 0x1f, 0xaf, 0x7d, 0x5a, 0x36, 0xfc,
]));
// DEVICE_ID: 74EB 0174 27E3 04D2 (SN: ...2182)
const CLIENT_MAC_ADDR: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0xD2, 0x04, 0xE3, 0x27, 0x74, 0x01, 0xEB, 0x74,
]));
pub const MAC_PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0xEF, 0xBE]); // PAN Id

//
static MAC_REQUEST_CHANNEL: StaticCell<MacRequestChannel> = StaticCell::new();
static SCHEDULER_REQUEST_CHANNEL: StaticCell<SchedulerRequestChannel> = StaticCell::new();
static DRIVER_REQUEST_CHANNEL: StaticCell<DriverRequestChannel> = StaticCell::new();
static DRIVER_EVENT_CHANNEL: StaticCell<DriverEventChannel> = StaticCell::new();
static MAC_INDICATION_CHANNEL: StaticCell<MacIndicationChannel> = StaticCell::new();

// TODO: use PNRG from device
#[derive(Debug, Clone, Copy, Default)]
pub struct FakeRng;

impl RngCore for FakeRng {
    fn next_u32(&mut self) -> u32 {
        3
    }
    fn next_u64(&mut self) -> u64 {
        3
    }
    fn fill_bytes(&mut self, d: &mut [u8]) {
        d.fill(0);
    }
    fn try_fill_bytes(&mut self, d: &mut [u8]) -> Result<(), RngError> {
        d.fill(0);
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);
    let AvailableResources { radio, timer, .. } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let radio = RadioDriver::new(
        radio,
        timer,
        #[cfg(feature = "executor-trace")]
        executor_trace_channel,
        #[cfg(feature = "radio-trace")]
        radio_tracing_config(),
    );

    let buffer_allocator = buffer_allocator!(MAC_BUFFER_SIZE, 10);
    let mac_request_channel = MAC_REQUEST_CHANNEL.init(MacRequestChannel::new());
    let scheduler_request_channel = SCHEDULER_REQUEST_CHANNEL.init(SchedulerRequestChannel::new());
    let driver_request_channel = DRIVER_REQUEST_CHANNEL.init(DriverRequestChannel::new());
    let driver_response_channel = DRIVER_EVENT_CHANNEL.init(DriverEventChannel::new());
    let mac_indication_channel = MAC_INDICATION_CHANNEL.init(MacIndicationChannel::new());

    let ieee802154_address = radio.ieee802154_address();

    spawner.spawn(
        driver_service_task(
            timer,
            radio,
            buffer_allocator,
            driver_request_channel.receiver(),
            driver_response_channel.sender(),
        )
        .unwrap(),
    );
    spawner.spawn(
        scheduler_service_task(
            timer,
            scheduler_request_channel.receiver(),
            driver_request_channel.sender(),
            driver_response_channel.receiver(),
            buffer_allocator,
            ieee802154_address,
        )
        .unwrap(),
    );
    spawner.spawn(
        mac_service_task(
            timer,
            buffer_allocator,
            mac_request_channel.receiver(),
            scheduler_request_channel.sender(),
            mac_indication_channel.sender(),
        )
        .unwrap(),
    );

    upper_layer_task(
        timer,
        buffer_allocator,
        mac_request_channel.sender(),
        mac_indication_channel.receiver(),
    )
    .await;
}

#[embassy_executor::task]
async fn mac_service_task(
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    mac_request_receiver: MacRequestReceiver<'static>,
    scheduler_request_sender: SchedulerRequestSender<'static>,
    mac_indication_sender: MacIndicationSender<'static>,
) {
    let mut mac_service = MacService::<NrfRadioDriver>::new(
        timer,
        buffer_allocator,
        mac_request_receiver,
        mac_indication_sender,
        scheduler_request_sender,
    );
    mac_service.run().await
}

#[embassy_executor::task]
async fn scheduler_service_task(
    timer: NrfRadioSleepTimer,
    scheduler_request_receiver: SchedulerRequestReceiver<'static>,
    driver_request_sender: DriverRequestSender<'static>,
    driver_response_receiver: DriverEventReceiver<'static>,
    buffer_allocator: MacBufferAllocator,
    address: [u8; 8],
) -> ! {
    let mut rng = FakeRng;
    let mut scheduler_service = SchedulerService::<NrfRadioDriver>::new(
        timer,
        scheduler_request_receiver,
        driver_request_sender,
        driver_response_receiver,
        buffer_allocator,
        &mut rng,
        &address,
    );

    scheduler_service.run().await
}

#[embassy_executor::task]
async fn driver_service_task(
    timer: NrfRadioSleepTimer,
    radio: RadioDriver<NrfRadioDriver, TaskOff>,
    buffer_allocator: MacBufferAllocator,
    driver_request_receiver: DriverRequestReceiver<'static>,
    driver_response_sender: DriverEventSender<'static>,
) -> ! {
    #[cfg(feature = "executor-trace")]
    let executor_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;

    let driver_service = DriverService::new(
        radio,
        driver_request_receiver,
        driver_response_sender,
        buffer_allocator,
    );

    driver_service.run().await
}

#[cfg(feature = "tsch")]
async fn start_tsch(request_sender: &MacRequestSender<'static>) {
    use dot15d4::mac::{
        frame::fields::TschLinkOption,
        mlme::tsch::{
            setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
        },
        primitives::{MacRequest, TschModeRequest},
    };
    use dot15d4::scheduler::tsch::TschLinkType;

    // We create a single slotframe with handle 0 and size 100
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetSlotframe(SetSlotframeRequest {
        handle: 0,                             // Slotframe Identifier
        operation: TschScheduleOperation::Add, // we want to add a slotframe
        size: 100,                             // Size of the sloframe in timeslots
        advertise: true, // The slotframe will be advertised in Enhanced Beacons
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Then, we add a link (for advertising) to that new slotframe
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSetLink(SetLinkRequest {
        slotframe_handle: 0,                  // handle of the associated slotframe
        channel_offset: 0,                    // Channel offset used for the link
        timeslot: 0,                          // Timeslot to use in the slotframe
        link_options: TschLinkOption::Shared, // Link used only for transmissions
        link_type: TschLinkType::Advertising, // Used for data transmission and not for advertising
        neighbor: None,
        advertise: true, // The link will be advertised in Enhanced Beacons
    });

    // We submit the request to the MAC service and wait for the operation to be completed
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Finally, we switch to TSCH mode
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeTschMode(TschModeRequest {
        tsch_mode: true,
        tsch_cca: false,
    });
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
    info!("TSCH started");
}

async fn upper_layer_task(
    mut timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    info!("Start as client");

    let is_sender = !option_env!("SENDER").unwrap_or("").is_empty();

    #[cfg(feature = "tsch")]
    if is_sender {
        start_tsch(&request_sender).await;
    }

    #[cfg(not(feature = "tsch"))]
    if is_sender {
        use dot15d4::driver::radio::phy::{OQpsk250KBit, Phy, PhyConfig};

        const BUFFER_SIZE: usize = <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize;
        let payload = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut seq_nr = 42;
        let instant = timer.now();
        let mut nb_sent = 0;
        loop {
            let buffer = buffer_allocator.try_allocate_buffer(BUFFER_SIZE).unwrap();
            let data_request = data_request(buffer, seq_nr, &payload);

            let request_token = request_sender.allocate_request_token().await;

            let mac_confirm = request_sender
                .send_request_awaiting_response(request_token, data_request)
                .await;
            nb_sent += 1;

            let timestamp = mac_confirm.timestamp.unwrap().ticks();
            info!("Tx Timestamp: {}", timestamp);
            unsafe {
                timer
                    .wait_until(instant + nb_sent * NsDuration::millis(10))
                    .await
                    .unwrap()
            };
            seq_nr += 1;
        }
    }
    let mut consumer_token = mac_indication_receiver
        .try_allocate_consumer_token()
        .unwrap();
    loop {
        let (response_token, mac_indication) = mac_indication_receiver
            .receive_request_async(&mut consumer_token, &())
            .await;
        match mac_indication {
            dot15d4::mac::primitives::MacIndication::McpsData(data_indication) => {
                let received_mpdu = data_indication.mpdu;
                let timestamp = data_indication.timestamp.ticks();
                info!("Rx Timestamp : {}", timestamp);
                unsafe { buffer_allocator.deallocate_buffer(received_mpdu.into_buffer()) };
                mac_indication_receiver.received(response_token, ());
            }
            _ => unreachable!(),
        }
    }
}

fn data_request(buffer: BufferToken, seq_nr: u8, payload: &[u8]) -> MacRequest {
    const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
        .with_frame_control(SeqNrRepr::Yes)
        .with_addressing(AddressingRepr::new_legacy_addressing(
            AddressingMode::Extended,
            AddressingMode::Extended,
            true,
        ))
        .without_security()
        .without_ies();

    let mut mpdu_writer = MPDU_REPR
        .into_writer::<NrfRadioDriver>(
            FrameVersion::Ieee802154,
            FrameType::Data,
            payload.len() as u16,
            buffer,
        )
        .unwrap();

    mpdu_writer.set_sequence_number(seq_nr);
    mpdu_writer.set_ack_request(true);

    let mut addressing = mpdu_writer.addressing_fields_mut();
    // addressing.dst_pan_id_mut().set(&MAC_PAN_ID);
    addressing.src_address_mut().set(&SERVER_MAC_ADDR);
    addressing.dst_address_mut().set(&CLIENT_MAC_ADDR);

    mpdu_writer.frame_payload_mut().copy_from_slice(payload);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_writer.into_mpdu_frame(),
    })
}
