#![no_std]
#![no_main]

#[cfg(feature = "tsch")]
use dot15d4::{
    driver::timer::NsInstant,
    mac::{
        frame::{fields::MpduParser, mpdu::MpduFrame, MpduWithAllFields},
        primitives::ScanConfirm,
    },
};
use dot15d4::{
    driver::{
        radio::{
            frame::{
                Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
                IeListRepr, IeRepr, IeReprList, PanId, PanIdCompressionRepr,
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
        mlme::get::{GetRequest, GetRequestAttribute},
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
async fn start_tsch(request_sender: &MacRequestSender<'static>, mut timer: NrfRadioSleepTimer) {
    use dot15d4::mac::{
        frame::fields::TschLinkOption,
        mlme::{
            set::{SetRequest, SetRequestAttribute},
            tsch::{
                setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
            },
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
        slotframe_handle: 0, // handle of the associated slotframe
        channel_offset: 0,   // Channel offset used for the link
        timeslot: 0,         // Timeslot to use in the slotframe
        link_options: TschLinkOption::Tx
            | TschLinkOption::Rx
            | TschLinkOption::Shared
            | TschLinkOption::TimeKeeping, // Link used only for transmissions
        link_type: TschLinkType::Advertising, // Used for data transmission and not for advertising
        neighbor: None,
        advertise: true, // The link will be advertised in Enhanced Beacons
    });

    // We submit the request to the MAC service and wait for the operation to be completed
    request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    // Then, we configure absolute timing (i.e. t0 for ASN 0)
    let start_timestamp = timer.now();
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSet(SetRequest::new(SetRequestAttribute::MacAsn(
        0,
        start_timestamp,
    )));
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
    info!("TSCH PAN Started");
}

#[cfg(feature = "tsch")]
async fn join_network_from_scan(
    request_sender: &MacRequestSender<'static>,
    scan_confirm: ScanConfirm,
    buffer_allocator: MacBufferAllocator,
) -> Option<(u16, [u8; 8])> {
    let mut best_candidate = None;
    for descriptor in scan_confirm.pan_descriptor_list {
        let timestamp = descriptor.timestamp;
        let parser = descriptor.mpdu.into_parser().parse_addressing().unwrap();
        let parser = parser
            .parse_security()
            .parse_ies::<NrfRadioDriver>()
            .unwrap();
        let ies = parser.ies_fields();
        let join_metric = ies.tsch_sync().map(|ts| ts.join_metric());

        if let Some(join_metric) = join_metric {
            if let Some((best_join_metric, best_mpdu, best_timestamp)) = best_candidate {
                if join_metric < best_join_metric {
                    best_candidate = Some((join_metric, parser, timestamp));
                    unsafe { buffer_allocator.deallocate_buffer(best_mpdu.into_buffer()) };
                } else {
                    best_candidate = Some((best_join_metric, best_mpdu, best_timestamp));
                    unsafe { buffer_allocator.deallocate_buffer(parser.into_buffer()) };
                }
            } else {
                best_candidate = Some((join_metric, parser, timestamp));
            }
        } else {
            unsafe { buffer_allocator.deallocate_buffer(parser.into_buffer()) };
        }
    }

    if let Some((_, mpdu_parser, timestamp)) = best_candidate {
        join_network_from_beacon(request_sender, mpdu_parser, timestamp, buffer_allocator).await
    } else {
        None
    }
}

#[cfg(feature = "tsch")]
async fn join_network_from_beacon(
    request_sender: &MacRequestSender<'static>,
    mpdu_parser: MpduParser<MpduFrame, MpduWithAllFields>,
    rx_timestamp: NsInstant,
    buffer_allocator: MacBufferAllocator,
) -> Option<(u16, [u8; 8])> {
    use dot15d4::{
        mac::{
            frame::fields::TschLinkOption,
            mlme::{
                set::{SetRequest, SetRequestAttribute},
                tsch::{
                    setlink::SetLinkRequest, setslotframe::SetSlotframeRequest,
                    TschScheduleOperation,
                },
            },
        },
        scheduler::tsch::TschLinkType,
    };
    let ies = mpdu_parser.ies_fields();

    let sf_links_ie = ies.tsch_slotframe_link();
    if let Some(sf_links_ie) = sf_links_ie {
        use dot15d4::mac::primitives::TschModeRequest;

        for slotframe in sf_links_ie.slotframes() {
            // We add the slotframe to our schedule
            let request_token = request_sender.allocate_request_token().await;
            let mac_request = MacRequest::MlmeSetSlotframe(SetSlotframeRequest {
                handle: slotframe.handle(),            // Slotframe Identifier
                operation: TschScheduleOperation::Add, // we want to add a slotframe
                size: slotframe.size(),                // Size of the sloframe in timeslots
                advertise: true, // The slotframe will be advertised in Enhanced Beacons
            });
            info!(
                "Add Slotframe #{} with size {} :",
                (slotframe.handle()),
                (slotframe.size())
            );
            request_sender
                .send_request_awaiting_response(request_token, mac_request)
                .await;
            for link in slotframe.links() {
                let request_token = request_sender.allocate_request_token().await;
                let mac_request = MacRequest::MlmeSetLink(SetLinkRequest {
                    slotframe_handle: slotframe.handle(),
                    channel_offset: link.channel_offset(),
                    timeslot: link.timeslot(),
                    link_options: TschLinkOption::from_bits(link.options()).unwrap(),
                    link_type: TschLinkType::Advertising,
                    neighbor: None,
                    advertise: true,
                });
                info!(
                    " * Add Link at timeslot={}, channel_offset={} with options {:b}",
                    (link.timeslot()),
                    (link.channel_offset()),
                    (link.options())
                );
                request_sender
                    .send_request_awaiting_response(request_token, mac_request)
                    .await;
            }
        }

        let request_token = request_sender.allocate_request_token().await;
        let asn = ies.tsch_sync().unwrap().asn();
        let mac_request = MacRequest::MlmeSet(SetRequest::new(SetRequestAttribute::MacAsn(
            asn,
            rx_timestamp,
        )));
        request_sender
            .send_request_awaiting_response(request_token, mac_request)
            .await;
        info!("ASN:{} rx_timestamp:{}", asn, (rx_timestamp.ticks()));

        // TODO: join metric

        // Finally, we switch to TSCH mode
        let request_token = request_sender.allocate_request_token().await;
        let mac_request = MacRequest::MlmeTschMode(TschModeRequest {
            tsch_mode: true,
            tsch_cca: false,
        });
        request_sender
            .send_request_awaiting_response(request_token, mac_request)
            .await;

        let addressing = mpdu_parser.try_addressing_fields().unwrap();
        let pan_id = addressing.try_dst_pan_id().unwrap().into_u16();
        let src_address = addressing.try_src_address().unwrap();
        let src_address = match src_address {
            Address::Extended(extended_address) => extended_address,
            _ => unreachable!(),
        };
        let mut coord_address = [0u8; 8];
        coord_address.copy_from_slice(src_address.as_ref());

        unsafe { buffer_allocator.deallocate_buffer(mpdu_parser.into_buffer()) };
        info!("Synchronized to TSCH PAN");

        Some((pan_id, coord_address))
    } else {
        None
    }
}

#[cfg(feature = "tsch")]
async fn scan(request_sender: &MacRequestSender<'static>) -> ScanConfirm {
    use dot15d4::driver::radio::config::Channel;
    use dot15d4::mac::primitives::{
        MacConfirm, MacRequest, ScanRequest, ScanType, TschModeRequest,
    };
    use dot15d4::scheduler::scan::ScanChannels;

    // Finally, we switch to TSCH mode
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeScan(ScanRequest {
        scan_type: ScanType::Passive,
        scan_channels: ScanChannels::Single(Channel::_26),
        scan_duration: 12,
        max_pan_descriptors: 1,
    });
    let response = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;
    info!("Scanned Channels");
    match response {
        MacConfirm::MlmeScan(scan_confirm) => scan_confirm,
        _ => unreachable!(),
    }
}

#[cfg(feature = "tsch")]
async fn associate(
    request_sender: &MacRequestSender<'static>,
    pan_id: u16,
    coord_address: [u8; 8],
) {
    use dot15d4::mac::frame::mpdu::{AssociationStatus, CapabilityInformation};
    use dot15d4::mac::{
        mlme::set::{SetRequest, SetRequestAttribute},
        primitives::{AssociateConfirm, AssociateRequest, MacConfirm, MacRequest},
    };
    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeSet(SetRequest::new(
        SetRequestAttribute::MacCoordExtendedAddress(coord_address),
    ));

    let confirm = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    let request_token = request_sender.allocate_request_token().await;
    let mac_request = MacRequest::MlmeAssociate(AssociateRequest::new(
        coord_address,
        pan_id,
        CapabilityInformation::default(),
    ));
    let response = request_sender
        .send_request_awaiting_response(request_token, mac_request)
        .await;

    match response {
        MacConfirm::MlmeAssociate(AssociateConfirm::Completed {
            status,
            short_address,
        }) => {
            if status == AssociationStatus::Successful {
                let addr_bytes = short_address.as_ref();
                let short_addr_u16 = u16::from_le_bytes([addr_bytes[0], addr_bytes[1]]);
                info!("Associated with short address: {:04X}", short_addr_u16);

                // MLME-SET short address from associate confirm.
                let request_token = request_sender.allocate_request_token().await;
                let mac_request = MacRequest::MlmeSet(SetRequest::new(
                    SetRequestAttribute::MacShortAddress(short_addr_u16),
                ));
                request_sender
                    .send_request_awaiting_response(request_token, mac_request)
                    .await;
                info!("Short address configured");
            } else {
                info!("Association failed");
            }
        }
        MacConfirm::MlmeAssociate(AssociateConfirm::NoAck) => {
            info!("Association request not acknowledged");
        }
        MacConfirm::MlmeAssociate(AssociateConfirm::ChannelAccessFailure) => {
            info!("Association failed: channel access failure");
        }
        _ => unreachable!(),
    }
}

async fn upper_layer_task(
    mut timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    info!("Start as client");

    let is_sender = !option_env!("SENDER").unwrap_or("").is_empty();
    let is_coord = !option_env!("COORDINATOR").unwrap_or("").is_empty();

    #[cfg(feature = "tsch")]
    if is_coord {
        start_tsch(&request_sender, timer).await;
    } else {
        let scan_confirm = scan(&request_sender).await;

        if let Some((pan_id, coord_address)) =
            join_network_from_scan(&request_sender, scan_confirm, buffer_allocator).await
        {
            associate(&request_sender, pan_id, coord_address).await;
        }
    }

    if is_sender {
        use dot15d4::driver::radio::phy::{OQpsk250KBit, Phy, PhyConfig};

        let dst_addr = {
            let request_token = request_sender.allocate_request_token().await;
            let mac_request = MacRequest::MlmeGet(GetRequest::new(
                GetRequestAttribute::MacCoordExtendedAddress,
            ));
            let confirm = request_sender
                .send_request_awaiting_response(request_token, mac_request)
                .await;
            match confirm {
                dot15d4::mac::primitives::MacConfirm::MlmeGet(
                    dot15d4::mac::mlme::get::GetConfirm::MacCoordExtendedAddress(addr),
                ) => addr,
                _ => unreachable!(),
            }
        };
        let src_addr = {
            let request_token = request_sender.allocate_request_token().await;
            let mac_request =
                MacRequest::MlmeGet(GetRequest::new(GetRequestAttribute::MacExtendedAddress));

            let confirm = request_sender
                .send_request_awaiting_response(request_token, mac_request)
                .await;

            match confirm {
                dot15d4::mac::primitives::MacConfirm::MlmeGet(
                    dot15d4::mac::mlme::get::GetConfirm::MacExtendedAddress(addr),
                ) => addr,
                _ => unreachable!(),
            }
        };

        const BUFFER_SIZE: usize = <Phy<OQpsk250KBit> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize;
        let payload = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut seq_nr = 42;
        let instant = timer.now();
        let mut nb_sent = 0;
        loop {
            let buffer = buffer_allocator.try_allocate_buffer(BUFFER_SIZE).unwrap();
            let data_request = data_request(buffer, seq_nr, &src_addr, &dst_addr, &payload);

            let request_token = request_sender.allocate_request_token().await;

            let mac_confirm = request_sender
                .send_request_awaiting_response(request_token, data_request)
                .await;
            nb_sent += 1;

            match mac_confirm {
                dot15d4::mac::primitives::MacConfirm::McpsData(timestamp) => {
                    let timestamp = timestamp.unwrap().ticks();
                    info!("Tx Timestamp: {}", timestamp);
                    unsafe {
                        timer
                            .wait_until(instant + nb_sent * NsDuration::millis(2000))
                            .await
                            .unwrap()
                    };
                    seq_nr += 1;
                }
                _ => unreachable!(),
            }
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

fn data_request(
    buffer: BufferToken,
    seq_nr: u8,
    src_addr: &Address<[u8; 8]>,
    dst_addr: &Address<[u8; 8]>,
    payload: &[u8],
) -> MacRequest {
    static IES: [IeRepr; 1] = [IeRepr::TschSynchronizationNestedIe];
    static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
    static IE_LIST: IeListRepr<'static> = IeListRepr::WithoutTerminationIes(IE_REPR_LIST);
    const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
        .with_frame_control(SeqNrRepr::Yes)
        .with_addressing(AddressingRepr::new(
            AddressingMode::Extended,
            AddressingMode::Extended,
            true,
            PanIdCompressionRepr::Yes,
        ))
        .without_security()
        .with_ies(IE_LIST);

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
    addressing.src_address_mut().set(src_addr);
    addressing.dst_address_mut().set(dst_addr);

    mpdu_writer.frame_payload_mut().copy_from_slice(payload);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_writer.into_mpdu_frame(),
    })
}
