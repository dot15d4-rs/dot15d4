#![no_std]
#![no_main]

use panic_probe as _;

use dot15d4_driver::{
    radio::Timer,
    socs::nrf::{export::*, NrfRadioDriver},
    timer::{now, wait_until, SyntonizedDuration},
};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    type NrfTimer = Timer<NrfRadioDriver>;
    const TIMEOUT: SyntonizedDuration = SyntonizedDuration::millis(10);

    let peripherals = pac::Peripherals::take().unwrap();

    // Enable the DC/DC converter
    peripherals.POWER.dcdcen.write(|w| w.dcdcen().enabled());

    // Enable external oscillators.
    let _ = Clocks::new(peripherals.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    NrfTimer::init(peripherals.RTC0);
    let anchor_time = now::<NrfTimer>();

    let mut count = 0;
    loop {
        count += 1;
        wait_until::<NrfTimer>(anchor_time + count * TIMEOUT).await;
    }
}
