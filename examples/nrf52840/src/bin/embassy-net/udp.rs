#![no_std]
#![no_main]

use panic_probe as _;

use dot15d4::driver::{
    radio::RadioDriver,
    socs::nrf::NrfRadioDriver,
    timer::{LocalClockDuration, RadioTimerApi, RadioTimerResult},
};
use dot15d4_embassy::{
    driver::Ieee802154Driver, export::*, mac_buffer_allocator, stack::Ieee802154Stack,
};
#[cfg(feature = "gpio-trace")]
use dot15d4_examples_nrf52840::PIN_EXECUTOR;
use embassy_executor::Spawner;
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    IpAddress, IpEndpoint, Ipv6Address, Ipv6Cidr, Runner,
};
use heapless::Vec;
use static_cell::StaticCell;

const FRAME_PERIOD: LocalClockDuration = LocalClockDuration::millis(10);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);

    let (peripherals, clocks, timer) = dot15d4_examples_nrf52840::config_peripherals();
    #[cfg(feature = "gpio-trace")]
    let gpiote_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;
    let radio = RadioDriver::new(
        peripherals.radio,
        clocks,
        timer,
        #[cfg(feature = "gpio-trace")]
        &peripherals.gpiote,
        #[cfg(feature = "gpio-trace")]
        gpiote_trace_channel,
    );
    let buffer_allocator = mac_buffer_allocator!();

    static RADIO_STACK: StaticCell<Ieee802154Stack<NrfRadioDriver>> = StaticCell::new();
    let radio_stack = RADIO_STACK.init(Ieee802154Stack::new(radio, buffer_allocator));

    let driver = radio_stack.driver();

    // We spawn the task that will control the CSMA task
    let ieee802154_task = ieee802154_task(radio_stack).unwrap();
    #[cfg(feature = "rtos-trace")]
    ieee802154_task.metadata().set_name("dot15d4\0");
    spawner.spawn(ieee802154_task);

    let addr = option_env!("ADDRESS").unwrap_or("1").parse().unwrap();
    let config = embassy_net::Config::ipv6_static(embassy_net::StaticConfigV6 {
        address: Ipv6Cidr::new(Ipv6Address::new(0xfd0e, 0, 0, 0, 0, 0, 0, addr), 64),
        dns_servers: Vec::new(),
        gateway: None,
    });

    // Init network stack
    let seed: u64 = 10; // XXX this should be random
    static NET_STACK_RESOURCES: StaticCell<embassy_net::StackResources<2>> = StaticCell::new();
    let (net_stack, net_runner) = embassy_net::new(
        driver,
        config,
        NET_STACK_RESOURCES.init(embassy_net::StackResources::<2>::new()),
        seed,
    );

    // Launch network task
    let net_task = net_task(net_runner).unwrap();
    #[cfg(feature = "rtos-trace")]
    net_task.metadata().set_name("embassy-net\0");
    spawner.spawn(net_task);

    // Then we can use it!
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    let mut socket = UdpSocket::new(
        net_stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket.bind(9400).unwrap();

    let mut tx_count = 0;
    let anchor_time = timer.now();

    loop {
        // If we are 1 -> echo the result back.
        if addr == 1 {
            let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
            socket.send_to(&buf[..n], ep).await.unwrap();
        } else {
            // If we are not 1 -> send a UDP packet to 1.
            let ep = IpEndpoint::new(IpAddress::v6(0xfd0e, 0, 0, 0, 0, 0, 0, 1), 9400);
            socket.send_to(b"Hello, World !", ep).await.unwrap();
            let (_, _ep) = socket.recv_from(&mut buf).await.unwrap();

            tx_count += 1;
            // Safety: The main task runs at lowest priority and won't be migrated.
            let res = unsafe {
                timer
                    .wait_until(anchor_time + ((tx_count + 1) * FRAME_PERIOD), None)
                    .await
            };
            debug_assert_eq!(res, RadioTimerResult::Ok);
        }
    }
}

/// Run Radio stack in the background
#[embassy_executor::task]
async fn ieee802154_task(radio_stack: &'static Ieee802154Stack<NrfRadioDriver>) -> ! {
    radio_stack.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Ieee802154Driver<'static, NrfRadioDriver>>) -> ! {
    runner.run().await
}
