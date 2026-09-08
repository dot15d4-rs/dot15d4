#[cfg(feature = "timer-trace")]
compile_error!("tsch-ramp-trace collides with timer-trace (GPIOTE 3 / PPI 9..12); enable only one");
#[cfg(feature = "radio-trace")]
compile_error!("tsch-ramp-trace collides with radio-trace (both drive P1.06); the FRAME pin below replaces the radio-trace frame pin");

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

    fork(p.TIMER0.events_compare[0].as_ptr() as u32, 7, 7, 9);
    fork(p.RADIO.events_rxready.as_ptr() as u32, 6, 6, 10);
    fork(p.RADIO.events_txready.as_ptr() as u32, 5, 5, 11);
    fork(p.CLOCK.events_hfclkstarted.as_ptr() as u32, 4, 4, 12);
    fork(p.RADIO.events_phyend.as_ptr() as u32, 1, 1, 15);

    p.GPIOTE.config[3].write(|w| {
        w.mode().task();
        w.port().bit(true);
        w.psel().variant(3);
        w.polarity().toggle()
    });
    p.PPI.fork[HFXO_PREWARM_PPI_CHANNEL]
        .tep
        .write(|w| w.tep().variant(p.GPIOTE.tasks_out[3].as_ptr() as u32));

    p.GPIOTE.config[2].write(|w| {
        w.mode().task();
        w.port().bit(true);
        w.psel().variant(2);
        w.polarity().toggle()
    });
    let frame_set = p.GPIOTE.tasks_set[2].as_ptr() as u32;
    let frame_clr = p.GPIOTE.tasks_clr[2].as_ptr() as u32;
    let ch = &p.PPI.ch[13];
    ch.eep
        .write(|w| w.eep().variant(p.RADIO.events_framestart.as_ptr() as u32));
    ch.tep.write(|w| w.tep().variant(frame_set));
    p.PPI.fork[13].tep.reset();
    let ch = &p.PPI.ch[14];
    ch.eep
        .write(|w| w.eep().variant(p.RADIO.events_end.as_ptr() as u32));
    ch.tep.write(|w| w.tep().variant(frame_clr));
    p.PPI.fork[14].tep.reset();
    // Safety: channels 13 and 14 are in range and otherwise unused.
    p.PPI.chenset.write(|w| unsafe { w.bits(1 << 13 | 1 << 14) });
}
