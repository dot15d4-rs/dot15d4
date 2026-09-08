#![no_std]
#![no_main]

#[cfg(feature = "tsch-ramp-trace")]
mod ramp_trace;
mod schedule;
mod val;

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
        frame::mpdu::{data_frame, MpduFrame},
        primitives::{DataRequest, MacConfirm, MacIndication, MacRequest},
        procedures::{
            get_coordinator_extended_address, get_device_extended_address, is_synchronized,
        },
        MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender,
        MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, MAC_BUFFER_SIZE,
    },
    scheduler::{
        SchedulerRequestChannel, SchedulerRequestReceiver, SchedulerRequestSender, SchedulerService,
    },
    util::{allocator::IntoBuffer, buffer_allocator},
    RngCore,
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, rng::NodeRng, AvailableResources};
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
    let AvailableResources {
        radio, rng, timer, ..
    } = config_peripherals(
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

    let node_rng = NodeRng::from_trng(rng);

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
            node_rng,
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

    upper_layer(
        spawner,
        timer,
        buffer_allocator,
        mac_request_channel.sender(),
        mac_request_channel.sender(),
        mac_request_channel.sender(),
        mac_indication_channel.receiver(),
    )
    .await
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
    mut rng: NodeRng,
) -> ! {
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
            dot15d4::util::info!(
                "val neighbor index={} addr={:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x} short={:04x} join_metric={} rx={} tx_acked={} tx_attempts={} etx_q7={} lqi={}",
                index,
                s.ext_addr[0], s.ext_addr[1], s.ext_addr[2], s.ext_addr[3],
                s.ext_addr[4], s.ext_addr[5], s.ext_addr[6], s.ext_addr[7],
                s.short_addr, s.advertised_join_metric, s.rx_count,
                s.tx_acked, s.tx_attempts, s.etx_q7, s.lqi,
            );
            index += 1;
        }
    }
}

#[embassy_executor::task]
async fn resync_task(
    mut timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    poll_period_ms: u32,
) -> ! {
    let config = Config::from_env();
    let period = NsDuration::millis(poll_period_ms as u64);

    loop {
        let wake_at = timer.now() + period;
        // Safety: this task drives one timer future at a time to completion
        // and is never migrated to another executor task.
        unsafe {
            let _ = timer.wait_until(wake_at).await;
        }

        if is_synchronized(&request_sender).await {
            continue;
        }

        val::desync(val::uptime_ms(&timer));

        schedule::rejoin(&request_sender, timer, buffer_allocator, &config).await;
    }
}

async fn upper_layer(
    spawner: Spawner,
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    data_request_sender: MacRequestSender<'static>,
    resync_request_sender: MacRequestSender<'static>,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    let config = Config::from_env();

    let own_address = extended_address(&request_sender).await;
    #[cfg(any(feature = "log", feature = "defmt"))]
    {
        let role = if schedule::is_time_root(&config) {
            "root"
        } else if config.beacon {
            "relay"
        } else {
            "leaf"
        };
        val::boot(config.node_id, role, &own_address, val::uptime_ms(&timer));
    }

    schedule::start(&request_sender, timer, buffer_allocator, &config).await;

    if !schedule::is_time_root(&config) && config.resync_poll_ms > 0 {
        spawner
            .spawn(resync_task(
                timer,
                buffer_allocator,
                resync_request_sender,
                config.resync_poll_ms,
            ).unwrap());
    }

    if config.sender {
        let destination = data_destination(&request_sender, &config).await;
        spawner
            .spawn(data_tx_task(
                timer,
                buffer_allocator,
                data_request_sender,
                own_address,
                destination,
                config.send_period_ms,
                config.send_jitter_ms,
                NodeRng::new(u64::from_be_bytes(own_address) ^ config.send_jitter_seed),
            ).unwrap());
    }

    indication_loop(timer, buffer_allocator, mac_indication_receiver).await
}

#[embassy_executor::task]
async fn data_tx_task(
    mut timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    source: [u8; 8],
    destination: [u8; 8],
    period_ms: u32,
    jitter_ms: u32,
    mut jitter_rng: NodeRng,
) -> ! {
    let src_addr = Address::Extended(ExtendedAddress::new_owned(source));
    let dst_addr = Address::Extended(ExtendedAddress::new_owned(destination));

    let interval = NsDuration::millis(period_ms as u64);
    let mut next_tx = timer.now() + interval;
    let mut seq: u32 = 0;

    loop {
        let request = data_request(buffer_allocator, &src_addr, &dst_addr, seq);
        let request_token = request_sender.allocate_request_token().await;
        let _confirm = request_sender
            .send_request_awaiting_response(request_token, request)
            .await;

        #[cfg(any(feature = "log", feature = "defmt"))]
        val::tx(
            seq,
            matches!(_confirm, MacConfirm::McpsData(Some(_))),
            val::uptime_ms(&timer),
        );

        seq = seq.wrapping_add(1);

        let now = timer.now();
        if next_tx <= now {
            next_tx = now;
        }

        let wake_at = if jitter_ms == 0 {
            next_tx
        } else {
            let span = 2 * jitter_ms as u64 + 1;
            let offset = (jitter_rng.next_u64() % span) as i64 - jitter_ms as i64;
            if offset >= 0 {
                next_tx + NsDuration::millis(offset as u64)
            } else {
                let back = NsDuration::millis(offset.unsigned_abs());
                if next_tx > now + back {
                    next_tx - back
                } else {
                    now
                }
            }
        };

        // Safety: this task drives one timer future at a time to completion
        // and is never migrated to another executor task.
        unsafe {
            let _ = timer.wait_until(wake_at).await;
        }
        next_tx += interval;
    }
}

async fn indication_loop(
    #[allow(unused_variables)] timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    let mut consumer_token = mac_indication_receiver
        .try_allocate_consumer_token()
        .unwrap();
    loop {
        let (response_token, mac_indication) = mac_indication_receiver
            .receive_request_async(&mut consumer_token, &())
            .await;
        match mac_indication {
            MacIndication::McpsData(data_indication) => {
                let mpdu = data_indication.mpdu;
                #[cfg(any(feature = "log", feature = "defmt"))]
                {
                    let (seq, source) = data_frame_origin(&mpdu);
                    val::rx(seq, &source, val::uptime_ms(&timer));
                }
                // Safety: incoming frames are allocated by the MAC service
                // from this allocator.
                unsafe { buffer_allocator.deallocate_buffer(mpdu.into_buffer()) };
            }
            #[cfg(feature = "tsch")]
            MacIndication::MlmeAssociateIndication(indication) => {
                #[cfg(any(feature = "log", feature = "defmt"))]
                val::assoc_peer(
                    &indication.device_address,
                    u16::from_le_bytes(indication.assigned_short_address),
                    val::uptime_ms(&timer),
                );
            }
            MacIndication::MlmeBeaconNotify(notification) => {
                // Safety: as above.
                unsafe { buffer_allocator.deallocate_buffer(notification.mpdu.into_buffer()) };
            }
        }
        mac_indication_receiver.received(response_token, ());
    }
}

async fn extended_address(request_sender: &MacRequestSender<'static>) -> [u8; 8] {
    match get_device_extended_address(request_sender).await {
        Address::Extended(extended) => {
            let mut bytes = [0u8; 8];
            bytes.copy_from_slice(extended.as_ref());
            bytes
        }
        _ => [0u8; 8],
    }
}

async fn data_destination(
    request_sender: &MacRequestSender<'static>,
    config: &Config,
) -> [u8; 8] {
    if let Some(destination) = config.dest {
        return destination;
    }
    match get_coordinator_extended_address(request_sender).await {
        Address::Extended(coord) if coord.as_ref() != [0xffu8; 8] => {
            let mut bytes = [0u8; 8];
            bytes.copy_from_slice(coord.as_ref());
            bytes
        }
        _ => panic!(
            "SENDER is set but there is no DEST/PARENT and the PIB holds \
             no coordinator address to send to"
        ),
    }
}

#[allow(dead_code)]
fn data_frame_origin(mpdu: &MpduFrame) -> (u32, [u8; 8]) {
    let mut sequence = u32::MAX;
    let mut source = [0u8; 8];

    let Ok(parser) = mpdu.reader().parse_addressing() else {
        return (sequence, source);
    };
    let Ok(parser) = parser.parse_security().parse_ies::<NrfRadioDriver>() else {
        return (sequence, source);
    };
    if let Ok(addressing) = parser.try_addressing_fields() {
        if let Some(Address::Extended(extended)) = addressing.try_src_address() {
            source.copy_from_slice(extended.as_ref());
        }
    }
    if let Some(payload) = parser.try_frame_payload() {
        if payload.len() >= 4 {
            sequence = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        }
    }
    (sequence, source)
}

struct Config {
    node_id: u16,
    parent: Option<[u8; 8]>,
    beacon: bool,
    sender: bool,
    dest: Option<[u8; 8]>,
    send_period_ms: u32,
    send_jitter_ms: u32,
    send_jitter_seed: u64,
    resync_poll_ms: u32,
}

impl Config {
    fn from_env() -> Self {
        let parent = option_env!("PARENT")
            .map(|s| parse_hex_addr(s).expect("PARENT must be 16 hex characters"));
        let send_period_ms: u32 = option_env!("SEND_PERIOD_MS")
            .map(|s| s.parse().expect("SEND_PERIOD_MS must be an integer"))
            .unwrap_or(10000);
        let send_jitter_ms: u32 = option_env!("SEND_JITTER_MS")
            .map(|s| s.parse().expect("SEND_JITTER_MS must be an integer"))
            .unwrap_or(0);
        Self {
            node_id: option_env!("NODE_ID")
                .map(|s| s.parse().expect("NODE_ID must be a small integer"))
                .unwrap_or(0),
            parent,
            beacon: option_env!("BEACON").map(|s| s != "0").unwrap_or(true),
            sender: option_env!("SENDER").unwrap_or("0").eq("1"),
            dest: option_env!("DEST")
                .map(|s| parse_hex_addr(s).expect("DEST must be 16 hex characters"))
                .or(parent),
            send_period_ms,
            send_jitter_ms: send_jitter_ms.min(send_period_ms),
            send_jitter_seed: option_env!("SEND_JITTER_SEED")
                .map(|s| s.parse().expect("SEND_JITTER_SEED must be an integer"))
                .unwrap_or(0),
            resync_poll_ms: option_env!("RESYNC_POLL_MS")
                .map(|s| s.parse().expect("RESYNC_POLL_MS must be an integer"))
                .unwrap_or(5000),
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
    sequence: u32,
) -> MacRequest {
    static IES: [IeRepr; 1] = [IeRepr::TschSynchronizationNestedIe];
    static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);

    let payload = sequence.to_le_bytes();
    let mut mpdu_writer =
        data_frame::<NrfRadioDriver>(Some(IE_REPR_LIST), payload.len() as u16, buffer_allocator)
            .unwrap();

    mpdu_writer.set_ack_request(true);

    let mut addressing = mpdu_writer.addressing_fields_mut();
    addressing.src_address_mut().set(src_addr);
    addressing.dst_address_mut().set(dst_addr);

    mpdu_writer.frame_payload_mut().copy_from_slice(&payload);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_writer.into_mpdu_frame(),
    })
}
