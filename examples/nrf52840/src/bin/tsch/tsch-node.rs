#![no_std]
#![no_main]

#[cfg(feature = "tsch-ramp-trace")]
mod ramp_trace;
mod schedule;

use dot15d4::{
    driver::{
        radio::{
            frame::{Address, ExtendedAddress, IeRepr, IeReprList},
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
        procedures::get_device_extended_address,
        MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender,
        MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, MAC_BUFFER_SIZE,
    },
    scheduler::{
        SchedulerRequestChannel, SchedulerRequestReceiver, SchedulerRequestSender, SchedulerService,
    },
    util::{allocator::IntoBuffer, buffer_allocator, info},
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, rng::FicrRng, AvailableResources};
use embassy_executor::Spawner;
use static_cell::StaticCell;

static MAC_REQUEST_CHANNEL: StaticCell<MacRequestChannel> = StaticCell::new();
static SCHEDULER_REQUEST_CHANNEL: StaticCell<SchedulerRequestChannel> = StaticCell::new();
static DRIVER_REQUEST_CHANNEL: StaticCell<DriverRequestChannel> = StaticCell::new();
static DRIVER_EVENT_CHANNEL: StaticCell<DriverEventChannel> = StaticCell::new();
static MAC_INDICATION_CHANNEL: StaticCell<MacIndicationChannel> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);
    let AvailableResources { radio, timer, .. } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    #[cfg(feature = "executor-trace")]
    let executor_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        radio,
        timer,
        #[cfg(feature = "executor-trace")]
        executor_trace_channel,
        #[cfg(feature = "radio-trace")]
        radio_tracing_config(),
    );

    #[cfg(feature = "tsch-ramp-trace")]
    ramp_trace::setup();

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

    #[cfg(any(feature = "log", feature = "defmt"))]
    {
        let stats_period_ms: u32 = option_env!("STATS_PERIOD_MS")
            .map(|s| s.parse().expect("STATS_PERIOD_MS must be an integer"))
            .unwrap_or(0);
        if stats_period_ms > 0 {
            spawner.spawn(
                neighbor_stats_task(timer, mac_request_channel.sender(), stats_period_ms).unwrap(),
            );
        }
    }

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
    let mut rng = FicrRng::new(address);
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
    let driver_service = DriverService::new(
        radio,
        driver_request_receiver,
        driver_response_sender,
        buffer_allocator,
    );

    driver_service.run().await
}

#[cfg(any(feature = "log", feature = "defmt"))]
#[embassy_executor::task]
async fn neighbor_stats_task(
    mut timer: NrfRadioSleepTimer,
    request_sender: MacRequestSender<'static>,
    period_ms: u32,
) -> ! {
    use dot15d4::mac::mlme::get::{GetConfirm, GetRequest, GetRequestAttribute};
    use dot15d4::mac::primitives::MacConfirm;

    let period = NsDuration::millis(period_ms as u64);
    loop {
        let wake_at = timer.now() + period;
        // Safety: this task drives one timer future at a time to completion
        // and is never migrated to another executor task.
        unsafe {
            let _ = timer.wait_until(wake_at).await;
        }

        let mut index: u8 = 0;
        while index < 16 {
            let request_token = request_sender.allocate_request_token().await;
            let mac_request = MacRequest::MlmeGet(GetRequest::new(
                GetRequestAttribute::MacNeighborStats(index),
            ));
            let confirm = request_sender
                .send_request_awaiting_response(request_token, mac_request)
                .await;
            let snapshot = match confirm {
                MacConfirm::MlmeGet(GetConfirm::MacNeighborStats(snapshot)) => snapshot,
                _ => None,
            };
            let Some(s) = snapshot else { break };
            info!(
                "neighbor {}: addr={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} short={:04x} join_metric={} rx={} tx_ok={}/{} etx_q7={} lqi={} last_heard={} last_ok={}",
                index,
                s.ext_addr[0], s.ext_addr[1], s.ext_addr[2], s.ext_addr[3],
                s.ext_addr[4], s.ext_addr[5], s.ext_addr[6], s.ext_addr[7],
                s.short_addr, s.advertised_join_metric, s.rx_count,
                s.tx_acked, s.tx_attempts, s.etx_q7, s.lqi,
                s.last_heard.ticks(), s.last_sent_ok.ticks(),
            );
            index += 1;
        }
    }
}

async fn upper_layer_task(
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    let config = Config::from_env();

    #[cfg(any(feature = "log", feature = "defmt"))]
    {
        let own = match get_device_extended_address(&request_sender).await {
            Address::Extended(ext) => {
                let mut bytes = [0u8; 8];
                bytes.copy_from_slice(ext.as_ref());
                bytes
            }
            _ => [0u8; 8],
        };
        info!(
            "node {}: PARENT={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            config.node_id, own[0], own[1], own[2], own[3], own[4], own[5], own[6], own[7]
        );
    }

    schedule::install(&request_sender, config.node_id, config.parent, config.beacon).await;
    schedule::start_or_join(
        &request_sender,
        timer,
        buffer_allocator,
        config.node_id,
        config.parent,
        config.beacon,
    )
    .await;

    if config.sender {
        // `wait_until` needs `&mut`; the timer is `Copy`.
        let mut timer = timer;
        let dst = config
            .dest
            .expect("SENDER is set but there is no DEST/PARENT to send to");
        let src_addr = get_device_extended_address(&request_sender).await;
        let dst_addr = Address::Extended(ExtendedAddress::new_owned(dst));
        let payload = [1u8, 2, 3, 4, 5, 6, 7, 8];

        let interval = NsDuration::millis(config.send_period_ms as u64);
        let mut next_tx = timer.now() + interval;
        loop {
            let request = data_request(buffer_allocator, &src_addr, &dst_addr, &payload);
            let request_token = request_sender.allocate_request_token().await;
            let _mac_confirm = request_sender
                .send_request_awaiting_response(request_token, request)
                .await;

            #[cfg(any(feature = "log", feature = "defmt"))]
            if let dot15d4::mac::primitives::MacConfirm::McpsData(Some(timestamp)) = &_mac_confirm {
                info!("Tx Timestamp: {}", timestamp.ticks());
            }

            let now = timer.now();
            if next_tx <= now {
                next_tx = now;
            }
            // Safety: this task drives one timer future at a time to completion
            // and is never migrated to another executor task.
            unsafe {
                let _ = timer.wait_until(next_tx).await;
            }
            next_tx += interval;
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

struct Config {
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
    sender: bool,
    dest: Option<[u8; 8]>,
    send_period_ms: u32,
}

impl Config {
    fn from_env() -> Self {
        let parent = option_env!("PARENT")
            .map(|s| parse_hex_addr(s).expect("PARENT must be 16 hex characters"));
        Self {
            node_id: option_env!("NODE_ID")
                .map(|s| s.parse().expect("NODE_ID must be a small integer"))
                .unwrap_or(0),
            parent,
            beacon: option_env!("BEACON").map(|s| s != "0").unwrap_or(true),
            sender: !option_env!("SENDER").unwrap_or("").is_empty(),
            dest: option_env!("DEST")
                .map(|s| parse_hex_addr(s).expect("DEST must be 16 hex characters"))
                .or(parent),
            send_period_ms: option_env!("SEND_PERIOD_MS")
                .map(|s| s.parse().expect("SEND_PERIOD_MS must be an integer"))
                .unwrap_or(10000),
        }
    }
}

fn parse_hex_addr(s: &str) -> Option<[u8; 8]> {
    let s = s.as_bytes();
    if s.len() != 16 {
        return None;
    }
    let mut out = [0u8; 8];
    for (i, byte) in out.iter_mut().enumerate() {
        let hi = (s[2 * i] as char).to_digit(16)?;
        let lo = (s[2 * i + 1] as char).to_digit(16)?;
        *byte = ((hi << 4) | lo) as u8;
    }
    Some(out)
}

fn data_request(
    buffer_allocator: MacBufferAllocator,
    src_addr: &Address<[u8; 8]>,
    dst_addr: &Address<[u8; 8]>,
    payload: &[u8],
) -> MacRequest {
    static IES: [IeRepr; 1] = [IeRepr::TschSynchronizationNestedIe];
    static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
    let mut mpdu_writer =
        data_frame::<NrfRadioDriver>(Some(IE_REPR_LIST), payload.len() as u16, buffer_allocator)
            .unwrap();

    mpdu_writer.set_ack_request(true);

    let mut addressing = mpdu_writer.addressing_fields_mut();
    addressing.src_address_mut().set(src_addr);
    addressing.dst_address_mut().set(dst_addr);

    mpdu_writer.frame_payload_mut().copy_from_slice(payload);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_writer.into_mpdu_frame(),
    })
}
