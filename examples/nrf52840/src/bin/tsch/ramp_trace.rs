
pub fn setup() {
    use dot15d4::driver::socs::nrf::export::pac::Peripherals;
    use dot15d4::driver::socs::nrf::HFXO_PREWARM_PPI_CHANNEL;

    // Safety: spare pins / spare GPIOTE+PPI channels / read-only TIMER0+RADIO+CLOCK
    //         register addresses. Runs once, after RadioDriver::new.
    let p = unsafe { Peripherals::steal() };

    let fork = |event_ptr: u32, pin: u8, gpiote_ch: usize, ppi_ch: usize| {
        p.GPIOTE.config[gpiote_ch].write(|w| {
            w.mode().task();
            w.port().bit(true);
            w.psel().variant(pin);
            w.polarity().toggle()
        });
        let task = p.GPIOTE.tasks_out[gpiote_ch].as_ptr() as u32;
        p.PPI.ch[ppi_ch].eep.write(|w| w.eep().variant(event_ptr));
        p.PPI.ch[ppi_ch].tep.write(|w| w.tep().variant(task));
        p.PPI.fork[ppi_ch].tep.reset();
        // Safety: ppi_ch is in range and otherwise unused.
        p.PPI.chenset.write(|w| unsafe { w.bits(1 << ppi_ch) });
    };

    fork(p.TIMER0.events_compare[0].as_ptr() as u32, 7, 7, 8);
    fork(p.RADIO.events_rxready.as_ptr() as u32, 6, 6, 9);
    fork(p.RADIO.events_txready.as_ptr() as u32, 5, 5, 10);
    fork(p.CLOCK.events_hfclkstarted.as_ptr() as u32, 4, 4, 11);

    p.GPIOTE.config[3].write(|w| {
        w.mode().task();
        w.port().bit(true);
        w.psel().variant(3);
        w.polarity().toggle()
    });
    p.PPI.fork[HFXO_PREWARM_PPI_CHANNEL]
        .tep
        .write(|w| w.tep().variant(p.GPIOTE.tasks_out[3].as_ptr() as u32));
}
