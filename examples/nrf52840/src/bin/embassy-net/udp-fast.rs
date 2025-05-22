#![no_std]
#![no_main]

use panic_probe as _;

use dot15d4_driver::{
    socs::nrf::{export::*, NrfRadioDriver},
    tasks::RadioDriver,
    time::{now, wait_for_alarm_at, Duration, Milliseconds, Timer},
};
use dot15d4_embassy::{
    driver::Ieee802154Driver, export::*, mac_buffer_allocator, stack::Ieee802154Stack,
};
use dot15d4_util::info;
use embassy_executor::{Spawner, SpawnerTraceExt};
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    IpAddress, IpEndpoint, Ipv6Address, Ipv6Cidr, Runner,
};
use heapless::Vec;
use static_cell::StaticCell;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    dot15d4_util::trace::instrument();

    let peripherals = pac::Peripherals::take().unwrap();

    // Enable the DC/DC converter
    peripherals.POWER.dcdcen.write(|w| w.dcdcen().enabled());

    // Enable external oscillators.
    let clocks = Clocks::new(peripherals.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    type NrfTimer = Timer<NrfRadioDriver>;
    NrfTimer::init(peripherals.RTC0);

    let radio = RadioDriver::new(peripherals.RADIO, clocks);
    let buffer_allocator = mac_buffer_allocator!();

    static RADIO_STACK: StaticCell<Ieee802154Stack<NrfRadioDriver>> = StaticCell::new();
    let radio_stack = RADIO_STACK.init(Ieee802154Stack::new(radio, buffer_allocator));

    let driver = radio_stack.driver();

    // We spawn the task that will control the CSMA task
    spawner
        .spawn_named("dot15d4\0", ieee802154_task(radio_stack, peripherals.RNG))
        .unwrap();

    let addr = option_env!("ADDRESS").unwrap_or("1").parse().unwrap();
    let config = embassy_net::Config::ipv6_static(embassy_net::StaticConfigV6 {
        address: Ipv6Cidr::new(Ipv6Address::new(0xfd0e, 0, 0, 0, 0, 0, 0, addr), 64),
        dns_servers: Vec::new(),
        gateway: None,
    });

    // Init network stack
    let seed: u64 = 10; // TODO: This should be random.
    static NET_STACK_RESOURCES: StaticCell<embassy_net::StackResources<2>> = StaticCell::new();
    let (net_stack, net_runner) = embassy_net::new(
        driver,
        config,
        NET_STACK_RESOURCES.init(embassy_net::StackResources::<2>::new()),
        seed,
    );

    // Launch network task
    spawner
        .spawn_named("embassy-net\0", net_task(net_runner))
        .unwrap();

    info!("Network task initialized");

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
    let mut rx_count = 0;

    let anchor_time = now::<NrfTimer>();

    loop {
        // If we are 1 -> Wait for datagram
        if addr == 1 {
            let (_n, _ep) = socket.recv_from(&mut buf).await.unwrap();
            rx_count += 1;
            if rx_count % 100 == 0 {
                info!("Received {} packets", rx_count);
            }
        } else {
            // If we are not 1 -> Send datagram every 10 ms
            let ep = IpEndpoint::new(IpAddress::v6(0xfd0e, 0, 0, 0, 0, 0, 0, 1), 9400);
            // info!("Sending message");
            socket.send_to(b"Hello, World !", ep).await.unwrap();
            tx_count += 1;
            if tx_count % 100 == 0 {
                info!("Sent {} packets", tx_count);
            }

            // Schedule the next transmission in 10ms (synchronized with the
            // time at which the first packet was transmitted)
            const TIMEOUT: Duration<NrfTimer> =
                Duration::<Milliseconds>::new(10).convert_into_rounding_up();
            wait_for_alarm_at::<NrfTimer>(anchor_time + ((tx_count + 1) * TIMEOUT)).await;
        }
    }
}

/// Run Radio stack in the background
#[embassy_executor::task]
async fn ieee802154_task(
    radio_stack: &'static Ieee802154Stack<NrfRadioDriver>,
    p_rng: pac::RNG,
) -> ! {
    let rng = Rng::new(p_rng);
    radio_stack.run(rng).await
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Ieee802154Driver<'static, NrfRadioDriver>>) -> ! {
    runner.run().await
}
