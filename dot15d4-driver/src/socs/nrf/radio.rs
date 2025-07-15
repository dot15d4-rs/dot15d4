//! nRF IEEE 802.15.4 radio driver

//TODO: In all poll functions: Check whether enabling the interrupt will pend
//      the interrupt if the event is pending. Otherwise we have a race
//      condition if an event occurs between us reading the event flag and
//      enabling the interrupt.

use core::{
    cell::{RefCell, RefMut},
    num::NonZero,
    ops::Deref,
    sync::atomic::{compiler_fence, Ordering},
    task::{Context, Poll, Waker},
};

use critical_section::{with as with_cs, CriticalSection, Mutex};
use dot15d4_util::{debug, frame::FramePdu, sync::CancellationGuard};
use nrf52840_hal::{
    clocks::{ExternalOscillator, LfOscStarted},
    pac::{
        self,
        generic::{Writable, W},
        interrupt,
        radio::{intenset::INTENSET_SPEC, state::STATE_A},
    },
    Clocks,
};
use typenum::U;

#[cfg(feature = "rtos-trace")]
use crate::trace::{
    MISSED_ISR, TASK_FALL_BACK, TASK_OFF_RUN, TASK_OFF_SCHEDULE, TASK_RX_FRAME_INFO,
    TASK_RX_FRAME_STARTED, TASK_RX_RUN, TASK_RX_SCHEDULE, TASK_TRANSITION_TO_OFF,
    TASK_TRANSITION_TO_RX, TASK_TRANSITION_TO_TX, TASK_TX_RUN, TASK_TX_SCHEDULE,
};
use crate::{
    config::{CcaMode, Channel},
    constants::{
        DEFAULT_SFD, FCS_LEN, MAC_AIFS, MAC_LIFS, MAC_SIFS, PHY_HDR_LEN, PHY_MAX_PACKET_SIZE_127,
    },
    frame::{AddressingFields, RadioFrame, RadioFrameSized},
    tasks::{
        ExternalRadioTransition, Ifs, OffResult, OffState, PreliminaryFrameInfo, RadioDriver,
        RadioState, RadioTaskError, RadioTransition, RxError, RxResult, RxState, SchedulingError,
        SelfRadioTransition, TaskOff, TaskRx, TaskTx, Timestamp, TxError, TxResult, TxState,
    },
    time::{Duration, Microseconds},
    DriverConfig, FcsNone, RadioDriverApi,
};

use super::NrfRadioTimer;

pub mod export {
    pub use nrf52840_hal::{
        clocks::{Clocks, ExternalOscillator, LfOscConfiguration, LfOscStarted},
        pac,
        rng::Rng,
    };
}

struct RadioInterruptHandler;

// TODO: Replace with a fast pseudo-executor that is able to poll all purely
//       interrupt-driven futures directly: run(), transition(),
//       frame_started(), preliminary_frame_info(), etc.
impl RadioInterruptHandler {
    /// Private function not to be called from outside this struct.
    ///
    /// Safety: Not to be called recursively.
    fn waker(cs: CriticalSection) -> RefMut<Option<Waker>> {
        static WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
        WAKER.borrow_ref_mut(cs)
    }

    /// Checks or sets the waker in the given context to be woken when the radio
    /// interrupt fires.
    fn arm<F: FnOnce(&mut <INTENSET_SPEC as Writable>::Writer) -> &mut W<INTENSET_SPEC>>(
        cx: &mut Context,
        f: F,
    ) {
        with_cs(|cs| {
            let mut waker = Self::waker(cs);
            if let Some(waker) = waker.deref() {
                debug_assert!(waker.will_wake(cx.waker()))
            } else {
                *waker = Some(cx.waker().clone());
            }

            NrfRadioDriver::radio().intenset.write(f);
        });
    }

    /// To be called from a radio interrupt.
    fn radio_interrupt() {
        with_cs(|cs| {
            let waker = Self::waker(cs).take();
            if let Some(waker) = waker {
                waker.wake();
            } else {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::marker(MISSED_ISR);
            }

            NrfRadioDriver::radio()
                .intenclr
                .write(|w| unsafe { w.bits(0xffffffff) });
        });
    }
}

#[interrupt]
fn RADIO() {
    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_enter();

    RadioInterruptHandler::radio_interrupt();

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::isr_exit_to_scheduler();
}

/// This struct serves multiple purposes:
/// 1. It provides access to private radio driver state across typestates of the
///    surrounding [`RadioDriver`].
/// 2. It serves as a unique marker for the nRF-specific implementation of the
///    [`RadioDriver`].
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct NrfRadioDriver;

impl DriverConfig for NrfRadioDriver {
    type Headroom = U<PHY_HDR_LEN>; // Headroom for the PHY header (packet length).
    type Tailroom = U<FCS_LEN>; // Tailroom for driver-level FCS handling.
    type MaxSduLength = U<{ PHY_MAX_PACKET_SIZE_127 - FCS_LEN }>; // The FCS is handled by the driver and must not be part of the MAC's MPDU.
    type Fcs = FcsNone; // Assuming automatic FCS handling.
    type Timer = NrfRadioTimer;
}

impl NrfRadioDriver {
    fn radio() -> pac::RADIO {
        // Safety: We let clients prove unique ownership of the peripheral by
        //         requiring an instance when instantiating the driver.
        // TODO: Check whether this results in efficient assembly.
        unsafe { pac::Peripherals::steal() }.RADIO
    }
}

impl<Task> RadioDriver<NrfRadioDriver, Task> {
    /// Convenience shortcut to access the radio registers.
    ///
    /// Safety: see [`NrfRadioDriver::radio()`]
    fn radio() -> pac::RADIO {
        NrfRadioDriver::radio()
    }

    fn set_ifs(ifs: Ifs) {
        const DRIVER_AIFS: Duration<Microseconds> = MAC_AIFS.convert_into_rounding_up();
        const DRIVER_SIFS: Duration<Microseconds> = MAC_SIFS.convert_into_rounding_up();
        const DRIVER_LIFS: Duration<Microseconds> = MAC_LIFS.convert_into_rounding_up();

        let tifs_us = match ifs {
            Ifs::Aifs => DRIVER_AIFS,
            Ifs::Sifs => DRIVER_SIFS,
            Ifs::Lifs => DRIVER_LIFS,
            Ifs::None => Duration::ZERO,
        }
        .ticks() as u16;

        Self::radio().tifs.write(|w| w.tifs().variant(tifs_us));
    }
}

/// RX bit counter event triggered after the frame control field (2 bytes) has
/// been received.
const BCC_FC_BITS: u32 = 2 * 8;

/// Radio Off state.
///
/// Entry: DISABLED event
/// Exit: READY event
///
/// State Invariants:
/// - The radio is in the DISABLED state.
/// - The READY event has been cleared.
/// - Only the READY interrupt is enabled.
///
/// The disabled state remains stable unless some task is actively
/// triggered.
impl RadioDriver<NrfRadioDriver, TaskOff> {
    /// Create a new IEEE 802.15.4 radio driver.
    ///
    /// Safety:
    /// - The constructor ensures that clients transfer exclusive ownership of
    ///   the radio peripheral. This also ensures that only a single instance of
    ///   the driver can be created in safe code.
    /// - The constructor lets clients prove proper configuration of the clocks
    ///   peripheral.
    pub fn new(
        radio: pac::RADIO,
        _clocks: Clocks<ExternalOscillator, ExternalOscillator, LfOscStarted>,
    ) -> Self {
        #[cfg(feature = "rtos-trace")]
        crate::trace::instrument_radio();

        // Disable and enable to reset peripheral
        radio.power.write(|w| w.power().disabled());
        radio.power.write(|w| w.power().enabled());

        // Enable 802.15.4 mode
        radio.mode.write(|w| w.mode().ieee802154_250kbit());
        // Configure CRC skip address
        radio
            .crccnf
            .write(|w| w.len().two().skipaddr().ieee802154());
        // Configure CRC polynomial and init
        radio.crcpoly.write(|w| w.crcpoly().variant(0x0001_1021));
        radio.crcinit.write(|w| w.crcinit().variant(0));
        radio.pcnf0.write(|w| {
            // 8-bit on air length
            w.lflen().variant(8);
            // Zero bytes S0 field length
            w.s0len().clear_bit();
            // Zero bytes S1 field length
            w.s1len().variant(0);
            // Do not include S1 field in RAM if S1 length > 0
            w.s1incl().automatic();
            // Zero code Indicator length
            w.cilen().variant(0);
            // 32-bit zero preamble
            w.plen()._32bit_zero();
            // Include CRC in length
            w.crcinc().include()
        });
        radio.pcnf1.write(|w| {
            // Maximum packet length
            w.maxlen().variant(PHY_MAX_PACKET_SIZE_127 as u8);
            // Zero static length
            w.statlen().variant(0);
            // Zero base address length
            w.balen().variant(0);
            // Little-endian
            w.endian().little();
            // Disable packet whitening
            w.whiteen().clear_bit()
        });
        // Default ramp-up mode for TIFS support.
        radio.modecnf0.write(|w| w.ru().default());

        // Configure the RX bit counter to trigger once the frame control field
        // has been received.
        radio.bcc.write(|w| w.bcc().variant(BCC_FC_BITS));

        // Clear and enable the radio interrupt
        pac::NVIC::unpend(pac::Interrupt::RADIO);
        // Safety: We're in early initialization, so there should be no
        //         concurrent critical sections.
        unsafe { pac::NVIC::unmask(pac::Interrupt::RADIO) };

        let mut driver = Self {
            inner: NrfRadioDriver,
            task: Some(TaskOff {
                at: Timestamp::BestEffort,
            }),
        };

        driver.set_sfd(DEFAULT_SFD);
        driver.set_tx_power(0);
        driver.set_channel(Channel::_11);
        driver.set_cca(CcaMode::CarrierSense);

        driver
    }

    /// Changes the Clear Channel Assessment method
    pub fn set_cca(&mut self, cca: CcaMode) {
        let r = Self::radio();
        match cca {
            CcaMode::CarrierSense => r.ccactrl.write(|w| w.ccamode().carrier_mode()),
            CcaMode::EnergyDetection { ed_threshold } => {
                // "[ED] is enabled by first configuring the field CCAMODE=EdMode in CCACTRL
                // and writing the CCAEDTHRES field to a chosen value."
                r.ccactrl.write(|w| {
                    w.ccamode().ed_mode();
                    w.ccaedthres().variant(ed_threshold)
                });
            }
        }
    }

    /// Changes the Start of Frame Delimiter (SFD)
    pub fn set_sfd(&mut self, sfd: u8) {
        Self::radio().sfd.write(|w| w.sfd().variant(sfd));
    }

    /// Changes the radio transmission power
    pub fn set_tx_power(&mut self, power: i8) {
        Self::radio().txpower.write(|w| match power {
            #[cfg(not(any(feature = "nrf52811", feature = "nrf5340-net")))]
            8 => w.txpower().pos8d_bm(),
            #[cfg(not(any(feature = "nrf52811", feature = "nrf5340-net")))]
            7 => w.txpower().pos7d_bm(),
            #[cfg(not(any(feature = "nrf52811", feature = "nrf5340-net")))]
            6 => w.txpower().pos6d_bm(),
            #[cfg(not(any(feature = "nrf52811", feature = "nrf5340-net")))]
            5 => w.txpower().pos5d_bm(),
            #[cfg(not(feature = "nrf5340-net"))]
            4 => w.txpower().pos4d_bm(),
            #[cfg(not(feature = "nrf5340-net"))]
            3 => w.txpower().pos3d_bm(),
            #[cfg(not(any(feature = "nrf52811", feature = "nrf5340-net")))]
            2 => w.txpower().pos2d_bm(),
            0 => w.txpower()._0d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -1 => w.txpower().neg1d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -2 => w.txpower().neg2d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -3 => w.txpower().neg3d_bm(),
            -4 => w.txpower().neg4d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -5 => w.txpower().neg5d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -6 => w.txpower().neg6d_bm(),
            #[cfg(feature = "nrf5340-net")]
            -7 => w.txpower().neg7d_bm(),
            -8 => w.txpower().neg8d_bm(),
            -12 => w.txpower().neg12d_bm(),
            -16 => w.txpower().neg16d_bm(),
            -20 => w.txpower().neg20d_bm(),
            -40 => w.txpower().neg40d_bm(),
            _ => panic!("Invalid transmission power value"),
        });
    }
}

impl<State> RadioDriverApi for RadioDriver<NrfRadioDriver, State> {
    fn ieee802154_address(&self) -> [u8; 8] {
        // Safety: Read-only access to a read-only register.
        let ficr: nrf52840_hal::pac::FICR = unsafe { pac::Peripherals::steal() }.FICR;
        let id1 = ficr.deviceid[0].read().bits(); // TODO: Should this be modified to use DEVICEADDR (only 48bit)?
        let id2 = ficr.deviceid[1].read().bits();
        [
            ((id1 & 0xff000000u32) >> 24u32) as u8,
            ((id1 & 0x00ff0000u32) >> 16u32) as u8,
            ((id1 & 0x0000ff00u32) >> 8u32) as u8,
            (id1 & 0x000000ffu32) as u8,
            ((id2 & 0xff000000u32) >> 24u32) as u8,
            ((id2 & 0x00ff0000u32) >> 16u32) as u8,
            ((id2 & 0x0000ff00u32) >> 8u32) as u8,
            (id2 & 0x000000ffu32) as u8,
        ]
    }
}

impl RadioState<TaskOff> for RadioDriver<NrfRadioDriver, TaskOff> {
    async fn transition(&mut self) -> Result<(), RadioTaskError<TaskOff>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TRANSITION_TO_OFF);

        // Wait until the state enters.
        core::future::poll_fn(|cx| {
            let r = Self::radio();
            if r.events_disabled.read().events_disabled().bit_is_set() {
                r.events_disabled.reset();
                Poll::Ready(())
            } else {
                RadioInterruptHandler::arm(cx, |w| w.disabled().set_bit());
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    async fn run(&mut self, _: bool) -> Result<OffResult, RadioTaskError<TaskOff>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_RUN);

        Ok(OffResult::Off)
    }

    fn exit(&mut self) -> Result<(), SchedulingError> {
        Ok(())
    }
}

impl OffState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskOff> {
    /// Changes the default radio channel
    fn set_channel(&mut self, channel: Channel) {
        let channel: u8 = channel.into();
        let frequency_offset = (channel - 10) * 5;
        Self::radio()
            .frequency
            .write(|w| w.frequency().variant(frequency_offset).map().default());
    }

    fn schedule_rx(
        self,
        rx_task: TaskRx,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskOff, TaskRx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_SCHEDULE);

        if let Timestamp::Scheduled(_) = rx_task.start {
            // TODO: Implement timed RX.
            todo!("not implemented")
        }

        let packetptr = rx_task.radio_frame.as_ptr() as u32;
        RadioTransition::new(
            self,
            rx_task,
            move || {
                let r = Self::radio();

                // Ramp up the receiver and start packet reception immediately.
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                dma_start_fence();

                r.shorts.write(|w| {
                    w.rxready_start().enabled();
                    w.framestart_bcstart().enabled()
                });
                r.tasks_rxen.write(|w| w.tasks_rxen().set_bit());

                Ok(())
            },
            || Ok(()),
            || {
                Self::radio()
                    .shorts
                    .modify(|_, w| w.rxready_start().disabled());
                Ok(())
            },
            false,
        )
    }

    fn schedule_tx(
        self,
        mut tx_task: TaskTx,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskOff, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        if let Timestamp::Scheduled(_) = tx_task.at {
            // TODO: Implement timed TX.
            todo!("not implemented")
        }

        let cca = tx_task.cca;
        let packetptr = prepare_tx_frame(&mut tx_task.radio_frame);
        RadioTransition::new(
            self,
            tx_task,
            move || {
                let r = Self::radio();

                r.packetptr.write(|w| w.packetptr().variant(packetptr));
                dma_start_fence();

                if cca {
                    r.shorts.write(|w| {
                        // Start CCA immediately after the receiver ramped up.
                        w.rxready_ccastart().enabled();
                        // If the channel is idle, then ramp up the transmitter and start TX
                        // immediately.
                        w.ccaidle_txen().enabled();
                        w.txready_start().enabled();
                        // If the channel is busy, then disable the receiver.
                        w.ccabusy_disable().enabled()
                    });

                    r.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
                } else {
                    r.shorts.write(|w| w.txready_start().enabled());
                    r.tasks_txen.write(|w| w.tasks_txen().set_bit());
                }
                Ok(())
            },
            || Ok(()),
            || {
                // Cleanup shorts.
                Self::radio().shorts.reset();
                Ok(())
            },
            false,
        )
    }

    async fn switch_off() -> Self {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_FALL_BACK);

        let off_task = Some(TaskOff {
            at: Timestamp::BestEffort,
        });
        let mut off_state = Self {
            inner: NrfRadioDriver,
            task: off_task,
        };

        let r = Self::radio();
        match r.state.read().state().variant().unwrap() {
            STATE_A::DISABLED => return off_state,
            STATE_A::RX_RU
            | STATE_A::RX_IDLE
            | STATE_A::RX
            | STATE_A::TX_RU
            | STATE_A::TX_IDLE
            | STATE_A::TX => {
                r.tasks_disable.write(|w| w.tasks_disable().set_bit());
            }
            STATE_A::TX_DISABLE | STATE_A::RX_DISABLE => {}
        }

        let is_off = off_state.transition().await;
        debug_assert!(is_off.is_ok());

        off_state
    }
}

/// Radio reception state.
///
/// Entry: READY event (coming from a non-RX state), otherwise RX state
/// Exit: END event
///
/// State Invariants:
/// - The radio is in the RX or RXIDLE state.
/// - The radio's DMA pointer points to an empty, writable packet in RAM.
/// - The "END" and "CRCERROR" events have been cleared before starting reception.
/// - Only the "END" interrupt is enabled.
impl RadioState<TaskRx> for RadioDriver<NrfRadioDriver, TaskRx> {
    async fn transition(&mut self) -> Result<(), RadioTaskError<TaskRx>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TRANSITION_TO_RX);

        // Wait until the state enters.
        core::future::poll_fn(|cx| {
            let r = Self::radio();
            if r.events_rxready.read().events_rxready().bit_is_set() {
                r.events_rxready.reset();
                Poll::Ready(())
            } else {
                // Double check state in case we're coming from another RX state
                // (CCA).
                match r.state.read().state().variant() {
                    Some(STATE_A::RX | STATE_A::RX_IDLE) => Poll::Ready(()),
                    _ => {
                        RadioInterruptHandler::arm(cx, |w| w.rxready().set_bit());
                        Poll::Pending
                    }
                }
            }
        })
        .await;

        Ok(())
    }

    async fn run(
        &mut self,
        rollback_on_crcerror: bool,
    ) -> Result<RxResult, RadioTaskError<TaskRx>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_RUN);

        let r = Self::radio();

        let is_back_to_back_rx = r.shorts.read().end_start().is_enabled();
        // Read the framestart event at the last possible moment to keep the
        // risk of missing a frame minimal.
        let reception_may_be_ongoing = r.events_framestart.read().events_framestart().bit_is_set();
        let result = if reception_may_be_ongoing || is_back_to_back_rx {
            // Ongoing reception or RX back-to-back.

            // Wait until the remainder of the packet has been received and the
            // receiver becomes idle.
            core::future::poll_fn(|cx| {
                if r.events_end.read().events_end().bit_is_set() {
                    r.events_end.reset();
                    // We reset the BCMATCH event here just in case we didn't
                    // retrieve the preliminary frame info for the last RX
                    // packet (e.g. if it was an ACK packet) and therefore also
                    // didn't reset the event.
                    r.events_bcmatch.reset();
                    Poll::Ready(())
                } else {
                    RadioInterruptHandler::arm(cx, |w| w.end().set_bit());
                    Poll::Pending
                }
            })
            .await;

            if r.events_crcerror.read().events_crcerror().bit_is_set() {
                r.events_crcerror.reset();

                if rollback_on_crcerror {
                    // When rolling back we're expected to place the radio in RX
                    // mode again and receive the next packet into the same
                    // buffer. Therefore restart the receiver unless it was
                    // already started.
                    if !is_back_to_back_rx {
                        r.tasks_start.write(|w| w.tasks_start().set_bit());
                    }
                    Err(RadioTaskError::Task(RxError::CrcError))
                } else {
                    let rx_task = self.task.take().unwrap();
                    Ok(RxResult::CrcError(rx_task.radio_frame))
                }
            } else {
                r.events_crcok.reset();
                dma_end_fence();

                // The CRC has been checked so the frame must have a non-zero
                // size saved in the headroom of the nRF packet (PHY header).
                let rx_task = self.task.take().unwrap();
                let sdu_length_wo_fcs =
                    NonZero::new(rx_task.radio_frame.pdu_ref()[0] as u16 - FCS_LEN as u16)
                        .expect("invalid length");

                Ok(RxResult::Frame(
                    rx_task.radio_frame.with_size(sdu_length_wo_fcs),
                ))
            }
        } else {
            // Otherwise: Cancel the ongoing task and leave the receiver in idle
            // state.
            r.tasks_stop.write(|w| w.tasks_stop().set_bit());

            let rx_task = self.task.take().unwrap();
            Ok(RxResult::RxWindowEnded(rx_task.radio_frame))
        };

        // Clear the framestart flag _after_ the receiver became idle to avoid
        // race conditions.
        r.events_framestart.reset();

        result
    }

    fn exit(&mut self) -> Result<(), SchedulingError> {
        Self::radio()
            .shorts
            .modify(|_, w| w.framestart_bcstart().disabled());
        Ok(())
    }
}

impl RxState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskRx> {
    async fn frame_started(&mut self) {
        let r = Self::radio();

        let cleanup_on_drop = CancellationGuard::new(|| {
            r.intenclr.write(|w| w.framestart().set_bit());
        });

        core::future::poll_fn(|cx| {
            if r.events_framestart.read().events_framestart().bit_is_set() {
                // Do not clear the framestart event as it is used in the RX run()
                // method.
                Poll::Ready(())
            } else {
                RadioInterruptHandler::arm(cx, |w| w.framestart().set_bit());
                Poll::Pending
            }
        })
        .await;

        drop(cleanup_on_drop);

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker(TASK_RX_FRAME_STARTED);
    }

    async fn preliminary_frame_info(&mut self) -> PreliminaryFrameInfo<'_> {
        let radio_frame = &self.task.as_ref().unwrap().radio_frame;

        let r = Self::radio();

        let cleanup_on_drop = CancellationGuard::new(|| {
            r.intenclr.write(|w| {
                w.bcmatch().set_bit();
                w.end().set_bit()
            });
            // Do not clear the end event as it is used in the RX run() method.
            r.tasks_bcstop.write(|w| w.tasks_bcstop().set_bit());
            r.bcc.write(|w| w.bcc().variant(BCC_FC_BITS));
        });

        // Wait until the frame control field has been received.
        const FC_LEN: u8 = 2;
        const SEQ_NR_LEN: u8 = 1;
        let frame_control = core::future::poll_fn(|cx| {
            if r.events_bcmatch.read().events_bcmatch().bit_is_set() {
                r.events_bcmatch.reset();

                // Safety: The bit counter match guarantees that the frame
                //         control field has been received.
                let (fc, addressing_repr) = unsafe { radio_frame.fc_and_addressing_repr() };
                let (seq_nr_len, seq_nr_offset) = if fc.sequence_number_suppression() {
                    (0, None)
                } else {
                    (SEQ_NR_LEN, Some(FC_LEN as usize))
                };

                // Note: BCMATCH counts are calculated relative to the MPDU
                //       (i.e. w/o headroom).
                let (fc, addressing_info, bcc_bytes) = if let Some(addressing_repr) =
                    addressing_repr
                {
                    let addressing_fields_lengths = addressing_repr.addressing_fields_lengths();
                    if let Ok(addressing_fields_lengths) = addressing_fields_lengths {
                        let [dst_pan_id_len, dst_addr_len, ..] = addressing_fields_lengths;
                        (
                            Some(fc),
                            Some((addressing_repr, (FC_LEN + seq_nr_len) as usize)),
                            Some((FC_LEN + seq_nr_len + dst_pan_id_len + dst_addr_len) as usize),
                        )
                    } else {
                        (
                            Some(fc),
                            None,
                            seq_nr_offset.map(|seq_nr_offset| seq_nr_offset + SEQ_NR_LEN as usize),
                        )
                    }
                } else {
                    (
                        None,
                        None,
                        seq_nr_offset.map(|seq_nr_offset| seq_nr_offset + SEQ_NR_LEN as usize),
                    )
                };

                if let Some(bcc) = bcc_bytes {
                    r.bcc.write(|w| w.bcc().variant((bcc as u32) << 3));
                }

                return Poll::Ready(Ok((
                    fc,
                    seq_nr_offset,
                    addressing_info,
                    bcc_bytes.is_some(),
                )));
            }

            if r.events_end.read().events_end().bit_is_set() {
                // Do not clear the end event as it is used in the RX run()
                // method.
                return Poll::Ready(Err(()));
            }

            RadioInterruptHandler::arm(cx, |w| {
                w.bcmatch().set_bit();
                w.end().set_bit()
            });
            Poll::Pending
        })
        .await;

        if frame_control.is_err() {
            return PreliminaryFrameInfo {
                mpdu_length: 0,
                frame_control: None,
                seq_nr: None,
                addressing_fields: None,
            };
        }

        let (frame_control, seq_nr_offset, addressing_info, bcc_set) = frame_control.unwrap();

        // Wait until the sequence number and/or destination address fields have
        // been received.
        if bcc_set {
            let bcmatch = core::future::poll_fn(|cx| {
                if r.events_bcmatch.read().events_bcmatch().bit_is_set() {
                    r.events_bcmatch.reset();
                    return Poll::Ready(Ok(()));
                }

                if r.events_end.read().events_end().bit_is_set() {
                    // Do not clear the end event as it is used in the RX run()
                    // method.
                    return Poll::Ready(Err(()));
                }

                RadioInterruptHandler::arm(cx, |w| {
                    w.bcmatch().set_bit();
                    w.end().set_bit()
                });
                Poll::Pending
            })
            .await;

            if bcmatch.is_err() {
                return PreliminaryFrameInfo {
                    mpdu_length: 0,
                    frame_control: None,
                    seq_nr: None,
                    addressing_fields: None,
                };
            }
        }

        drop(cleanup_on_drop);

        let pdu_ref = radio_frame.pdu_ref();
        let mpdu_length = pdu_ref[0] as u16;

        // Safety: The bit counter guarantees that all bytes up to the
        //         addressing fields have been received.
        const HEADROOM: usize = 1;
        let seq_nr = seq_nr_offset.map(|seq_nr_offset| pdu_ref[HEADROOM + seq_nr_offset]);
        let addressing_fields = addressing_info
            .map(|(addressing_repr, addressing_offset)| {
                let addressing_offset = HEADROOM + addressing_offset;
                let addressing_bytes = &pdu_ref[addressing_offset..];
                unsafe { AddressingFields::new_unchecked(addressing_bytes, addressing_repr).ok() }
            })
            .unwrap_or_default();

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker(TASK_RX_FRAME_INFO);

        PreliminaryFrameInfo {
            mpdu_length,
            frame_control,
            seq_nr,
            addressing_fields,
        }
    }

    fn schedule_rx(
        self,
        rx_task: TaskRx,
        rollback_on_crcerror: bool,
    ) -> impl SelfRadioTransition<NrfRadioDriver, TaskRx, TaskRx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_SCHEDULE);

        if let Timestamp::Scheduled(_) = rx_task.start {
            // See the note re back-to-back scheduling in the API.
            panic!("not supported")
        }

        let packetptr = rx_task.radio_frame.as_ptr() as u32;
        RadioTransition::new(
            self,
            rx_task,
            move || {
                let r = Self::radio();

                r.packetptr.write(|w| w.packetptr().variant(packetptr));
                dma_start_fence();

                // Enable back-to-back packet reception.
                //
                // NOTE: We need to set up the short before checking radio state to
                //       avoid race conditions, see RX_IDLE case below.
                r.shorts.write(|w| {
                    w.end_start().enabled();
                    w.framestart_bcstart().enabled()
                });

                // Check whether the task has already completed.
                //
                // NOTE: Read the state _after_ having set the short.
                match r.state.read().state().variant() {
                    Some(STATE_A::RX_IDLE) => {
                        // We're idle, although we have a short in place: This means
                        // that the previous packet was fully received before we were
                        // able to set the short, i.e. reception of the new packet was
                        // not started by hardware, we need to start it manually, see
                        // conditions 1. and 2. in the method documentation.
                        r.tasks_start.write(|w| w.tasks_start().set_bit());

                        debug!("late scheduling");
                    }
                    Some(STATE_A::RX) => {
                        // We're still receiving the previous packet (i.e., END pending,
                        // condition 3.).
                    }
                    _ => unreachable!(),
                };

                Ok(())
            },
            || Ok(()),
            || {
                Self::radio().shorts.modify(|_, w| w.end_start().disabled());
                Ok(())
            },
            rollback_on_crcerror,
        )
    }

    fn schedule_tx(
        self,
        mut tx_task: TaskTx,
        ifs: Ifs,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskRx, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        if let Timestamp::Scheduled(_) = tx_task.at {
            // TODO: Implement timed TX.
            todo!("not implemented")
        }

        let cca = tx_task.cca;
        // PACKETPTR is double buffered so we don't cause a race by setting it
        // while reception might still be ongoing.
        let packetptr = prepare_tx_frame(&mut tx_task.radio_frame);
        RadioTransition::new(
            self,
            tx_task,
            move || {
                let r = Self::radio();

                r.packetptr.write(|w| w.packetptr().variant(packetptr));
                dma_start_fence();

                Self::set_ifs(ifs);

                // NOTE: We need to set up shorts before completing the task to
                //       avoid race conditions, see RX_IDLE case below.
                if cca {
                    r.shorts.write(|w| {
                        // Ramp down and up again for proper IFS and CCA timing.
                        w.end_disable().enabled();
                        w.disabled_rxen().enabled();
                        w.rxready_ccastart().enabled();
                        // If the channel is idle, then ramp up and start TX immediately.
                        w.ccaidle_txen().enabled();
                        w.txready_start().enabled();
                        // If the channel is busy, then disable the receiver so that
                        // we reach the fallback state.
                        w.ccabusy_disable().enabled()
                    });
                } else {
                    r.shorts.write(|w| {
                        // Directly switch to TX mode w/o CCA.
                        w.end_disable().enabled();
                        w.disabled_txen().enabled();
                        w.txready_start().enabled()
                    });
                }

                Ok(())
            },
            || {
                // Check whether the task has already completed.
                //
                // NOTE: Read the state _after_ having set the shorts.
                let r = Self::radio();
                if let Some(STATE_A::RX_IDLE) = r.state.read().state().variant() {
                    // We're idle, although we have a short in place: This
                    // means that the previous packet was either fully
                    // received before we were able to set the short or the
                    // RX window ended w/o receiving a packet. In any case,
                    // disabling the radio was not started by hardware, we
                    // need to disable it manually.
                    r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                }

                Ok(())
            },
            || {
                Self::radio().shorts.reset();
                Ok(())
            },
            rollback_on_crcerror,
        )
    }

    fn schedule_off(
        self,
        off_task: TaskOff,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskRx, TaskOff> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_SCHEDULE);

        if let Timestamp::Scheduled(_) = off_task.at {
            // TODO: Implement timed RX.
            todo!("not implemented")
        }

        RadioTransition::new(
            self,
            off_task,
            || {
                // Ramp down the receiver.
                //
                // NOTE: We need to set up the short before completing the task
                //       to avoid race conditions, see RX_IDLE case below.
                Self::radio().shorts.write(|w| w.end_disable().enabled());
                Ok(())
            },
            || {
                // Check whether the task has already completed.
                //
                // NOTE: Read the state _after_ having set the short.
                let r = Self::radio();
                match r.state.read().state().variant() {
                    Some(STATE_A::RX_IDLE) => {
                        // We're idle, although we have a short in place: This
                        // means that the previous packet was either fully
                        // received before we were able to set the short or the
                        // RX window ended w/o receiving a packet. In any case,
                        // disabling the radio was not started by hardware, we
                        // need to disable it manually.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                    }
                    Some(STATE_A::RX_DISABLE | STATE_A::DISABLED) => {}
                    _ => unreachable!(),
                };

                Ok(())
            },
            || {
                // Cleanup shorts.
                Self::radio().shorts.reset();
                Ok(())
            },
            rollback_on_crcerror,
        )
    }
}

/// Radio transmission state.
///
/// Entry: READY event
/// Exit: PHYEND event
///
/// State Invariants:
/// - The radio is in the TX or TXIDLE state.
/// - The radio's DMA pointer points to an empty packet in RAM.
/// - The "PHYEND" event has been cleared before starting reception.
/// - Only the "PHYEND" interrupt is enabled.
impl RadioState<TaskTx> for RadioDriver<NrfRadioDriver, TaskTx> {
    async fn transition(&mut self) -> Result<(), RadioTaskError<TaskTx>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TRANSITION_TO_TX);

        let r = Self::radio();

        if let Some(tx_task) = &self.task {
            if tx_task.cca {
                if r.events_ccabusy.read().events_ccabusy().bit_is_set() {
                    r.events_ccabusy.reset();
                    let recovered_task = self.task.take().unwrap();
                    return Err(RadioTaskError::Task(TxError::CcaBusy(
                        recovered_task.radio_frame,
                    )));
                }

                r.events_ccaidle.reset();
            }
        }

        // Wait until the state enters.
        core::future::poll_fn(|cx| {
            if r.events_txready.read().events_txready().bit_is_set() {
                r.events_txready.reset();
                Poll::Ready(())
            } else {
                RadioInterruptHandler::arm(cx, |w| w.txready().set_bit());
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    async fn run(&mut self, _: bool) -> Result<TxResult, RadioTaskError<TaskTx>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_RUN);

        let r = Self::radio();

        // Wait until the task completed.
        core::future::poll_fn(|cx| {
            if r.events_end.read().events_end().bit_is_set() {
                r.events_end.reset();
                r.events_framestart.reset();
                let tx_task = self.task.take().unwrap();
                Poll::Ready(Ok(TxResult::Sent(tx_task.radio_frame)))
            } else {
                RadioInterruptHandler::arm(cx, |w| w.end().set_bit());
                Poll::Pending
            }
        })
        .await
    }

    fn exit(&mut self) -> Result<(), SchedulingError> {
        Ok(())
    }
}

impl TxState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskTx> {
    fn schedule_rx(
        self,
        rx_task: TaskRx,
        ifs: Ifs,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskTx, TaskRx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_SCHEDULE);

        if let Timestamp::Scheduled(_) = rx_task.start {
            // TODO: Implement timed RX.
            todo!("not implemented")
        }

        let packetptr = rx_task.radio_frame.as_ptr() as u32;
        RadioTransition::new(
            self,
            rx_task,
            move || {
                let r = Self::radio();

                r.packetptr.write(|w| w.packetptr().variant(packetptr));
                dma_start_fence();

                Self::set_ifs(ifs);

                // NOTE: To cater for errata 204 (see rev1 v1.4) a TX-to-RX
                //       switch must pass through the disabled state, which is
                //       what the shorts imply anyway.

                // NOTE: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see the TX_IDLE case below.
                r.shorts.write(|w| {
                    // Ramp down the receiver, ramp it up again in RX mode and
                    // then start packet reception immediately.
                    w.end_disable().enabled();
                    w.disabled_rxen().enabled();
                    w.rxready_start().enabled()
                });

                // Check whether the transition has already triggered. If not, then wait
                // until it triggers.
                //
                // NOTE: Only read the state _after_ having set the short.
                match r.state.read().state().variant() {
                    Some(STATE_A::TX_IDLE) => {
                        // We're idle, although we have a short in place: This means
                        // that the previous packet ended before we were able to set the
                        // short, i.e., hardware did not start transitioning to RX,
                        // we need to start it manually, see conditions 1. and 2. in the
                        // method documentation.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());

                        debug!("late scheduling");
                    }
                    Some(STATE_A::TX | STATE_A::TX_DISABLE | STATE_A::RX_RU) => {
                        // We're still sending the previous packet (i.e., PHYEND pending,
                        // condition 3.), or we're already ramping down and back up again
                        // (i.e., PHYEND triggered, condition 2.).
                    }
                    _ => unreachable!(),
                };

                Ok(())
            },
            || {
                // We need to schedule the BCSTART short _after_ the FRAMESTART
                // event of the TX packet, otherwise the TX packet will trigger
                // the BCMATCH event already.
                Self::radio()
                    .shorts
                    .modify(|_, w| w.framestart_bcstart().enabled());

                Ok(())
            },
            || {
                // Cleanup shorts
                Self::radio().shorts.modify(|_, w| {
                    w.end_disable().disabled();
                    w.disabled_rxen().disabled();
                    w.rxready_start().disabled()
                });
                Ok(())
            },
            false,
        )
    }

    fn schedule_tx(
        self,
        mut tx_task: TaskTx,
        ifs: Ifs,
    ) -> impl SelfRadioTransition<NrfRadioDriver, TaskTx, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        if let Timestamp::Scheduled(_) = tx_task.at {
            // TODO: Implement timed TX.
            todo!("not implemented")
        }

        let cca = tx_task.cca;
        let packetptr = prepare_tx_frame(&mut tx_task.radio_frame);
        RadioTransition::new(
            self,
            tx_task,
            move || {
                // NOTE: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see the TX_IDLE case below.
                let r = Self::radio();

                r.packetptr.write(|w| w.packetptr().variant(packetptr));
                dma_start_fence();

                Self::set_ifs(ifs);

                if cca {
                    r.shorts.write(|w| {
                        // Ramp down the transceiver, ramp it up again in RX mode and
                        // then start CCA immediately.
                        w.end_disable().enabled();
                        w.disabled_rxen().enabled();
                        w.rxready_ccastart().enabled();

                        // If the channel is idle, then ramp up and start TX immediately.
                        w.ccaidle_txen().enabled();
                        w.txready_start().enabled();

                        // If the channel is busy, then disable the receiver.
                        w.ccabusy_disable().enabled()
                    });
                } else {
                    r.shorts.write(|w| {
                        // Ramp down and up again for proper IFS timing.
                        w.end_disable().enabled();
                        w.disabled_txen().enabled();
                        w.txready_start().enabled()
                    })
                }

                Ok(())
            },
            || {
                // Check whether the transition has already triggered. If not then wait
                // until it triggers.
                //
                // NOTE: Read the state _after_ having set the short.
                let r = Self::radio();
                if let Some(STATE_A::TX_IDLE) = r.state.read().state().variant() {
                    // We check whether a second packet was already sent
                    // just in the unlikely case that we got here so late
                    // that the next task was already executed.
                    if r.events_end.read().events_end().bit_is_clear() {
                        // We're idle, although we have a short in place:
                        // This means that the previous packet ended before
                        // we were able to set the short, i.e. transitioning
                        // to RX was not started by hardware, we need to
                        // start it manually, see conditions 1. and 2. in
                        // the method documentation.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                        debug!("late scheduling");
                    } else {
                        debug!("slow completion");
                    }
                }

                Ok(())
            },
            || {
                // Clean up shorts
                Self::radio().shorts.reset();
                Ok(())
            },
            false,
        )
    }

    fn schedule_off(
        self,
        off_task: TaskOff,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskTx, TaskOff> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_SCHEDULE);

        if let Timestamp::Scheduled(_) = off_task.at {
            // TODO: Implement timed TX.
            todo!("not implemented")
        }

        RadioTransition::new(
            self,
            off_task,
            move || {
                // Ramp down the receiver.
                //
                // NOTE: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see TX_IDLE case below.
                Self::radio().shorts.write(|w| w.end_disable().enabled());

                Ok(())
            },
            move || {
                // Check whether the transition has already triggered. If not then wait
                // until it triggers.
                //
                // NOTE: Read the state _after_ having set the short.
                let r = Self::radio();
                match r.state.read().state().variant() {
                    Some(STATE_A::TX_IDLE) => {
                        // We're idle, although we have a short in place: This
                        // means that the previous packet was either fully
                        // received before we were able to set the short or the
                        // RX window ended w/o receiving a packet. In any case
                        // disabling the radio was not started by hardware, we
                        // need to disable it manually.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                    }
                    Some(STATE_A::TX | STATE_A::TX_DISABLE | STATE_A::DISABLED) => {}
                    _ => unreachable!(),
                };

                Ok(())
            },
            || {
                // Clean up shorts.
                Self::radio().shorts.reset();
                Ok(())
            },
            false,
        )
    }
}

fn prepare_tx_frame(radio_frame: &mut RadioFrame<RadioFrameSized>) -> u32 {
    let sdu_length = radio_frame.sdu_wo_fcs_length().get() as u8 + FCS_LEN as u8;
    // Set PHY HDR.
    radio_frame.pdu_mut()[0] = sdu_length;
    // Return PACKETPTR.
    radio_frame.as_ptr() as u32
}

/// NOTE: Must be preceded by a volatile write operation to the DMA pointer.
///
/// TODO: Is this actually correct?
/// Also see <https://github.com/rust-lang/unsafe-code-guidelines/issues/260>.
fn dma_start_fence() {
    compiler_fence(Ordering::Release);
}

/// NOTE: Must be followed by a volatile read operation to the DMA pointer.
///
/// TODO: Is this actually correct?
/// Also see <https://github.com/rust-lang/unsafe-code-guidelines/issues/260>.
fn dma_end_fence() {
    compiler_fence(Ordering::Acquire);
}
