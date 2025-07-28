#![no_std]
#![no_main]

use panic_probe as _;

use dot15d4_driver::{
    socs::nrf::{export::*, NrfRadioDriver},
    time::{now, wait_for_alarm_at, Duration, Milliseconds, Timer},
};
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    type NrfTimer = Timer<NrfRadioDriver>;
    const TIMEOUT: Duration<NrfTimer> = Duration::<Milliseconds>::new(10).convert_into_rounding_up();

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
        wait_for_alarm_at::<NrfTimer>(anchor_time + count * TIMEOUT).await;
    }
}
