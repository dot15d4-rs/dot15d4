#![no_std]
#![no_main]

use core::future::pending;

use dot15d4::{
    driver::{
        radio::RadioDriver,
        socs::nrf::{export::pac::RADIO, NrfRadioDriver, NrfRadioSleepTimer},
        DriverEventChannel, DriverEventReceiver, DriverEventSender, DriverRequestChannel,
        DriverRequestReceiver, DriverRequestSender, DriverService,
    },
    mac::{
        frame::fields::TschLinkOption,
        mlme::tsch::{
            setlink::SetLinkRequest, setslotframe::SetSlotframeRequest, TschScheduleOperation,
        },
        primitives::{MacRequest, TschModeRequest},
        MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender,
        MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, MAC_BUFFER_SIZE,
    },
    scheduler::{
        tsch::schedule::TschLinkType, SchedulerRequestChannel, SchedulerRequestReceiver,
        SchedulerRequestSender, SchedulerService,
    },
    util::{buffer_allocator, info},
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, AvailableResources};
use embassy_executor::Spawner;
use static_cell::StaticCell;

//
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

    let buffer_allocator = buffer_allocator!(MAC_BUFFER_SIZE, 10);
    let mac_request_channel = MAC_REQUEST_CHANNEL.init(MacRequestChannel::new());
    let scheduler_request_channel = SCHEDULER_REQUEST_CHANNEL.init(SchedulerRequestChannel::new());
    let driver_request_channel = DRIVER_REQUEST_CHANNEL.init(DriverRequestChannel::new());
    let driver_response_channel = DRIVER_EVENT_CHANNEL.init(DriverEventChannel::new());
    let mac_indication_channel = MAC_INDICATION_CHANNEL.init(MacIndicationChannel::new());

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
) -> ! {
    let mut scheduler_service = SchedulerService::<NrfRadioDriver>::new(
        timer,
        scheduler_request_receiver,
        driver_request_sender,
        driver_response_receiver,
        buffer_allocator,
    );

    scheduler_service.run().await
}

#[embassy_executor::task]
async fn driver_service_task(
    timer: NrfRadioSleepTimer,
    radio: RADIO,
    buffer_allocator: MacBufferAllocator,
    driver_request_receiver: DriverRequestReceiver<'static>,
    driver_response_sender: DriverEventSender<'static>,
) -> ! {
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

    let driver_service = DriverService::new(
        radio,
        driver_request_receiver,
        driver_response_sender,
        buffer_allocator,
    );

    driver_service.run().await
}

async fn start_tsch(request_sender: &MacRequestSender<'static>) {
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
    _timer: NrfRadioSleepTimer,
    _buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    _mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    info!("Start as client");
    start_tsch(&request_sender).await;
    pending().await
}
