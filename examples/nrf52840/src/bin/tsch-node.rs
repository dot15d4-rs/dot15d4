#![no_std]
#![no_main]

#[cfg(feature = "tsch")]
use dot15d4::mac::procedures::{join_network_from_scan, scan, tsch_start_pan};
use dot15d4::{
    driver::{
        radio::{
            frame::{Address, IeRepr, IeReprList},
            tasks::{RadioDriverApi, TaskOff},
            RadioDriver,
        },
        socs::nrf::{NrfRadioDriver, NrfRadioSleepTimer},
        timer::{NsDuration, RadioTimerApi},
        DriverEventChannel, DriverEventReceiver, DriverEventSender, DriverRequestChannel,
        DriverRequestReceiver, DriverRequestSender, DriverService,
    },
    mac::{
        frame::mpdu::data_frame,
        primitives::{DataRequest, MacRequest},
        procedures::{get_coordinator_extended_address, get_device_extended_address},
        MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender,
        MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, MAC_BUFFER_SIZE,
    },
    scheduler::{
        SchedulerRequestChannel, SchedulerRequestReceiver, SchedulerRequestSender, SchedulerService,
    },
    util::{allocator::IntoBuffer, buffer_allocator, info},
    RngCore, RngError,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, AvailableResources};
use embassy_executor::Spawner;
use static_cell::StaticCell;

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
        tsch_start_pan(&request_sender, timer).await;
    } else {
        let scan_confirm = scan(&request_sender).await;

        if let Some((pan_id, coord_address)) = join_network_from_scan::<NrfRadioDriver>(
            &request_sender,
            scan_confirm,
            buffer_allocator,
        )
        .await
        {
            use dot15d4::mac::procedures::tsch_associate;

            tsch_associate(&request_sender, pan_id, coord_address).await;
        }
    }

    if is_sender {
        let dst_addr = get_coordinator_extended_address(&request_sender).await;
        let src_addr = get_device_extended_address(&request_sender).await;

        let payload = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut seq_nr = 42;
        let instant = timer.now();
        let mut nb_sent = 0;
        loop {
            let data_request =
                data_request(buffer_allocator, seq_nr, &src_addr, &dst_addr, &payload);

            let request_token = request_sender.allocate_request_token().await;

            let mac_confirm = request_sender
                .send_request_awaiting_response(request_token, data_request)
                .await;
            nb_sent += 1;

            match mac_confirm {
                dot15d4::mac::primitives::MacConfirm::McpsData(timestamp) => {
                    #[cfg(any(feature = "log", feature = "defmt"))]
                    {
                        let timestamp = timestamp.unwrap().ticks();
                        info!("Tx Timestamp: {}", timestamp);
                    }
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
                #[cfg(any(feature = "log", feature = "defmt"))]
                {
                    let timestamp = data_indication.timestamp.ticks();
                    info!("Rx Timestamp : {}", timestamp);
                }
                unsafe { buffer_allocator.deallocate_buffer(received_mpdu.into_buffer()) };
                mac_indication_receiver.received(response_token, ());
            }
            _ => unreachable!(),
        }
    }
}

fn data_request(
    buffer_allocator: MacBufferAllocator,
    seq_nr: u8,
    src_addr: &Address<[u8; 8]>,
    dst_addr: &Address<[u8; 8]>,
    payload: &[u8],
) -> MacRequest {
    static IES: [IeRepr; 1] = [IeRepr::TschSynchronizationNestedIe];
    static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
    let mut mpdu_writer =
        data_frame::<NrfRadioDriver>(Some(IE_REPR_LIST), payload.len() as u16, buffer_allocator)
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
