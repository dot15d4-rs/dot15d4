//! nRF IEEE 802.15.4 radio driver

use core::{
    future::{poll_fn, Future},
    num::NonZero,
    sync::atomic::{compiler_fence, Ordering},
    task::Poll,
};

use dot15d4_util::{debug, frame::FramePdu, sync::CancellationGuard};
use nrf52840_pac::{self as pac, radio::state::STATE_A};

#[cfg(feature = "rtos-trace")]
use crate::radio::trace::{
    MARKER_RX_FRAME_INFO, MARKER_RX_FRAME_STARTED, MARKER_RX_WINDOW_ENDED, TASK_FALL_BACK,
    TASK_OFF_COMPLETE, TASK_OFF_ENTRY, TASK_OFF_SCHEDULE, TASK_RX_COMPLETE, TASK_RX_ENTRY,
    TASK_RX_SCHEDULE, TASK_TX_COMPLETE, TASK_TX_ENTRY, TASK_TX_SCHEDULE,
};
use crate::{
    executor::InterruptExecutor,
    radio::{
        config::{CcaMode, Channel},
        frame::{AddressingFields, RadioFrame, RadioFrameSized},
        phy::{Ifs as PhyIfs, OQpsk250KBit, Phy, PhyConfig},
        tasks::{
            ExternalRadioTransition, ListeningRxState, OffResult, OffState, PreliminaryFrameInfo,
            RadioDriverApi, RadioState, RadioTask, RadioTaskError, RadioTransition,
            ReceivingRxState, RxError, RxResult, SchedulingError, SelfRadioTransition,
            StopListeningResult, TaskOff, TaskRx, TaskTx, TxError, TxResult, TxState,
        },
        DriverConfig, FcsNone, PhyOf, RadioDriver, RadioDriverState,
    },
    socs::nrf::NrfRadioHighPrecisionTimer,
    timer::{
        HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
        OptionalNsInstant, RadioTimerError, TimedSignal,
    },
};

use super::{
    executor::{nrf_interrupt_executor, NrfInterruptPriority},
    timer::NrfRadioSleepTimer,
};

use self::executor::NrfInterruptExecutor;

// The nRF hardware only supports default CCA duration.
const _: () = {
    let cca_duration: <PhyOf<NrfRadioDriver> as PhyConfig>::SymbolPeriods =
        <PhyOf<NrfRadioDriver> as PhyConfig>::PHY_CCA_DURATION.convert();
    assert!(cca_duration.ticks() == 8)
};

// On a real device, we measure considerable deviations of timestamps from the
// timings documented in Nordic's product specification.
//
// The following facts have been established so far:
// - Offsets differ when executing via Ozone vs debug-embed.
// - It seems that the rx and tx enabled events are triggered with the same
//   offset (within the error margin of the high-precision timer).
// - There is an inconsistency between the rx/tx enabled event and the
//   framestart event. These events should be spaced by exactly T_SHR in the tx
//   case and T_SHR+T_PHR in the rx case. We measure an additional delay,
//   though. Which of the two events should we trust?
//
// Open questions:
// - Do offsets also differ per device?
// - Are offsets needed at all when running w/o a probe attached?
//
// How could we make further progress?
// - Check via GPIOs w/o a debug probe attached?
// - Start tx + rx tests on separate devices with precisely synchronized timers
//   and correlate the send timestamp and receive timestamp.
// - Observe the radio channel itself with a trusted high-precision sniffer.
//
// TODO: Find a way to identify the "true" offsets (as observable on the
//       physical radio channel).

// Ozone/JRun/SystemView: 7
// debug-embed: 10
const T_MEASURED_RAMP_UP_ERROR: NsDuration =
    NsDuration::from_ticks(10 * NrfRadioHighPrecisionTimer::TICK_PERIOD.ticks());
// all: 5
const T_MEASURED_RAMP_DOWN_ERROR: NsDuration =
    NsDuration::from_ticks(5 * NrfRadioHighPrecisionTimer::TICK_PERIOD.ticks());
// Ozone/JRun/SystemView: 4
// debug-embed: 8
const T_MEASURED_TURNAROUND_ERROR: NsDuration =
    NsDuration::from_ticks(8 * NrfRadioHighPrecisionTimer::TICK_PERIOD.ticks());
// all: 195
const T_MEASURED_TX_FRAMESTART_ERROR: NsDuration =
    NsDuration::from_ticks(195 * NrfRadioHighPrecisionTimer::TICK_PERIOD.ticks());

// Disabled to tx idle duration
const T_TXEN: NsDuration = NsDuration::micros(130)
    .checked_sub(T_MEASURED_RAMP_UP_ERROR)
    .unwrap();
// Disabled to rx idle duration
const T_RXEN: NsDuration = NsDuration::micros(130)
    .checked_sub(T_MEASURED_RAMP_UP_ERROR)
    .unwrap();
// Rx idle to disabled duration
const T_RXDIS: NsDuration = NsDuration::nanos(500);
// CCA duration
const T_CCA: NsDuration = <PhyOf<NrfRadioDriver> as PhyConfig>::PHY_CCA_DURATION;
// Rx-to-tx and tx-to-rx duration
const T_TURNAROUND: NsDuration = NsDuration::micros(130)
    .checked_sub(T_MEASURED_TURNAROUND_ERROR)
    .unwrap();
// Shortcut for CCA + Turnaround
const T_BACKOFF_PERIOD: NsDuration = T_CCA.checked_add(T_TURNAROUND).unwrap();

/// This struct serves multiple purposes:
/// 1. It provides access to private radio driver state across typestates of the
///    surrounding [`RadioDriver`].
/// 2. It serves as a unique marker for the nRF-specific implementation of the
///    [`RadioDriver`].
#[derive(Clone, Copy, Debug)]
pub struct NrfRadioDriver {
    executor: NrfInterruptExecutor,
}

const FCS_LEN: u8 = Phy::<OQpsk250KBit>::fcs_length();
impl DriverConfig for NrfRadioDriver {
    type Phy = Phy<OQpsk250KBit>;
    const HEADROOM: u8 = OQpsk250KBit::PHY_HDR_LEN as u8; // Headroom for the PHY header (frame length).
    const TAILROOM: u8 = FCS_LEN; // Tailroom for driver-level FCS handling.
    const MAX_SDU_LENGTH: u16 = (<Self::Phy as PhyConfig>::PHY_MAX_PACKET_SIZE - FCS_LEN as u16); // The FCS is handled by the driver and must not be part of the MAC's MPDU.
    type Fcs = FcsNone; // Assuming automatic FCS handling.
    type Timer = NrfRadioSleepTimer;
}

type Ifs = PhyIfs<Phy<OQpsk250KBit>>;

/// Convenience shortcut to access the radio registers.
fn radio() -> pac::RADIO {
    // Safety: We let clients prove unique ownership of the peripheral by
    //         requiring an instance when instantiating the driver.
    unsafe { pac::Peripherals::steal() }.RADIO
}

fn set_ifs(ifs: Option<Ifs>, with_guard_time: bool) {
    const AIFS_US: u16 = Ifs::ack().into_local_clock_duration().to_micros() as u16;
    const SIFS_US: u16 = Ifs::short().into_local_clock_duration().to_micros() as u16;
    const LIFS_US: u16 = Ifs::long().into_local_clock_duration().to_micros() as u16;

    use PhyIfs::*;

    let tifs_us = match ifs {
        Some(ifs) => {
            let mut tifs_us = match ifs {
                Aifs(_) => AIFS_US,
                Sifs(_) => SIFS_US,
                Lifs(_) => LIFS_US,
            };

            if with_guard_time {
                // Worst case clock drift counted from the end of a frame to the
                // beginning of the next frame (IFS):
                //
                //         IFS * (clock_drift_ppm / 1_000_000).
                //
                // This will always be less than a single microsecond even assuming
                // a clock drift of up to 1500 ppm applied to a LIFS duration.
                tifs_us -= 1;
            }

            tifs_us
        }
        None => 0,
    };

    radio().tifs.write(|w| w.tifs().variant(tifs_us));
}

/// Rx bit counter event triggered after the frame control field (2 bytes) has
/// been received.
const BCC_FC_BITS: u32 = 2 * 8;

const fn timed_rx_enable(start: NsInstant) -> TimedSignal {
    // RMARKER offset: disabled -> rx -> SHR
    const OFFSET: NsDuration = T_RXEN.checked_add(OQpsk250KBit::T_SHR).unwrap();
    TimedSignal::new(
        start.checked_sub_duration(OFFSET).unwrap(),
        HardwareSignal::RadioRxEnable,
    )
}

const fn timed_tx_enable(at: NsInstant, cca: bool) -> TimedSignal {
    if cca {
        // RMARKER offset with CCA: disabled -> rx -> CCA -> turnaround -> SHR
        const OFFSET_DIS_TO_TX_W_CCA: NsDuration = T_RXEN
            .checked_add(T_BACKOFF_PERIOD)
            .unwrap()
            .checked_add(OQpsk250KBit::T_SHR)
            .unwrap();
        TimedSignal::new(
            at.checked_sub_duration(OFFSET_DIS_TO_TX_W_CCA).unwrap(),
            HardwareSignal::RadioRxEnable,
        )
    } else {
        // RMARKER offset without CCA: disabled -> tx -> SHR
        const OFFSET_DIS_TO_TX_NO_CCA: NsDuration =
            T_TXEN.checked_add(OQpsk250KBit::T_SHR).unwrap();
        TimedSignal::new(
            at.checked_sub_duration(OFFSET_DIS_TO_TX_NO_CCA).unwrap(),
            HardwareSignal::RadioTxEnable,
        )
    }
}

const fn timed_off(at: NsInstant) -> TimedSignal {
    TimedSignal::new(at, HardwareSignal::RadioDisable)
}

#[cfg(feature = "radio-trace")]
pub struct NrfRadioTracingConfig {
    pub gpiote_frame_channel: usize,
    pub gpiote_trx_channel: usize,
    pub ppi_framestart_channel: usize,
    pub ppi_frameend_channel: usize,
    pub ppi_trxstart_channel: usize,
    pub ppi_trxend_channel: usize,
}

static mut STATE: RadioDriverState<NrfRadioDriver> = RadioDriverState::new();

impl<Task> RadioDriver<NrfRadioDriver, Task> {
    fn state(&mut self) -> &'static mut RadioDriverState<NrfRadioDriver> {
        // Safety:
        // - We ensure that there only ever is a single instance of the radio
        //   driver: Initial state is derived from unique instances of the
        //   peripherals owned by this driver. Whenever a new state is created
        //   the old state must be handed in. This can be checked by verifying
        //   all occurrences of RadioDriver::new_internal().
        // - Based on that, requiring &mut self ensures that only a single
        //   mutable reference can be created at any time by piggybacking on the
        //   exclusivity guarantee of a mutable reference to a singleton.
        // - Also see safety remarks for of the Sync-implementation of driver state.
        #[allow(static_mut_refs)]
        unsafe {
            &mut STATE
        }
    }

    fn state_ref(&self) -> &'static RadioDriverState<NrfRadioDriver> {
        // Safety: Same as above, only that this time we piggyback on the
        //         exclusivity guarantee of shared access to the radio driver
        //         singleton.
        #[allow(static_mut_refs)]
        unsafe {
            &STATE
        }
    }
}

/// "Radio Off" state.
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
        // Note: The nRF radio timer implicitly enforces clock policy for the
        //       radio.
        sleep_timer: NrfRadioSleepTimer,
        #[cfg(feature = "executor-trace")] executor_trace_channel: usize,
        #[cfg(feature = "radio-trace")] tracing_config: NrfRadioTracingConfig,
    ) -> Self {
        #[cfg(feature = "rtos-trace")]
        crate::radio::trace::instrument();

        let mut driver = Self::new_internal();

        let inner = NrfRadioDriver {
            executor: *self::executor(
                NrfInterruptPriority::HIGHEST,
                #[cfg(feature = "executor-trace")]
                executor_trace_channel,
            ),
        };
        driver.state().init(inner, sleep_timer);

        #[cfg(feature = "radio-trace")]
        {
            // Safety: We only use explicitly allocated PPI and GPIOTE
            //         resources.
            let ppi = unsafe { pac::Peripherals::steal().PPI };
            let gpiote = unsafe { pac::Peripherals::steal().GPIOTE };

            // Route framestart and end events to the frame pin.
            let framestart_event = radio.events_framestart.as_ptr() as u32;
            let framestart_task =
                gpiote.tasks_set[tracing_config.gpiote_frame_channel].as_ptr() as u32;
            let ch = &ppi.ch[tracing_config.ppi_framestart_channel];
            ch.eep.write(|w| w.eep().variant(framestart_event));
            ch.tep.write(|w| w.tep().variant(framestart_task));
            ppi.fork[tracing_config.ppi_framestart_channel].tep.reset();

            let frameend_event = radio.events_end.as_ptr() as u32;
            let frameend_task =
                gpiote.tasks_clr[tracing_config.gpiote_frame_channel].as_ptr() as u32;
            let ch = &ppi.ch[tracing_config.ppi_frameend_channel];
            ch.eep.write(|w| w.eep().variant(frameend_event));
            ch.tep.write(|w| w.tep().variant(frameend_task));
            ppi.fork[tracing_config.ppi_frameend_channel].tep.reset();

            // Route ready and disabled events to the trx pin.
            let trxstart_event = radio.events_ready.as_ptr() as u32;
            let trxstart_task = gpiote.tasks_set[tracing_config.gpiote_trx_channel].as_ptr() as u32;
            let ch = &ppi.ch[tracing_config.ppi_trxstart_channel];
            ch.eep.write(|w| w.eep().variant(trxstart_event));
            ch.tep.write(|w| w.tep().variant(trxstart_task));
            ppi.fork[tracing_config.ppi_trxstart_channel].tep.reset();

            let trxend_event = radio.events_disabled.as_ptr() as u32;
            let trxend_task = gpiote.tasks_clr[tracing_config.gpiote_trx_channel].as_ptr() as u32;
            let ch = &ppi.ch[tracing_config.ppi_trxend_channel];
            ch.eep.write(|w| w.eep().variant(trxend_event));
            ch.tep.write(|w| w.tep().variant(trxend_task));
            ppi.fork[tracing_config.ppi_trxend_channel].tep.reset();

            let ppi_channel_mask = (1 << tracing_config.ppi_framestart_channel)
                | (1 << tracing_config.ppi_frameend_channel)
                | (1 << tracing_config.ppi_trxstart_channel)
                | (1 << tracing_config.ppi_trxend_channel);
            // Safety: We operate on allocated channels only.
            ppi.chenset.write(|w| unsafe { w.bits(ppi_channel_mask) });
        }

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
            // Maximum frame length
            w.maxlen()
                .variant(<PhyOf<NrfRadioDriver> as PhyConfig>::PHY_MAX_PACKET_SIZE as u8);
            // Zero static length
            w.statlen().variant(0);
            // Zero base address length
            w.balen().variant(0);
            // Little-endian
            w.endian().little();
            // Disable whitening
            w.whiteen().clear_bit()
        });
        // Default ramp-up mode for TIFS support.
        radio.modecnf0.write(|w| w.ru().default());

        // Configure the rx bit counter to trigger once the frame control field
        // has been received.
        radio.bcc.write(|w| w.bcc().variant(BCC_FC_BITS));

        driver.set_sfd(OQpsk250KBit::DEFAULT_SFD);
        driver.set_tx_power(0);
        driver.set_channel(Channel::_12);
        driver.set_cca(CcaMode::CarrierSense);

        driver
    }

    /// Changes the Clear Channel Assessment method
    pub fn set_cca(&mut self, cca: CcaMode) {
        let r = radio();
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
        radio().sfd.write(|w| w.sfd().variant(sfd));
    }

    /// Changes the radio transmission power
    pub fn set_tx_power(&mut self, power: i8) {
        radio().txpower.write(|w| match power {
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

impl<Task: RadioTask> RadioDriverApi<NrfRadioDriver, Task> for RadioDriver<NrfRadioDriver, Task> {
    fn ieee802154_address(&self) -> [u8; 8] {
        // Safety: Read-only access to a read-only register.
        let ficr: pac::FICR = unsafe { pac::Peripherals::steal() }.FICR;
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

    fn enter_next_task<NextTask: RadioTask>(
        mut self,
        next_task: NextTask,
    ) -> RadioDriver<NrfRadioDriver, NextTask>
    where
        RadioDriver<NrfRadioDriver, NextTask>: RadioState<NrfRadioDriver, NextTask>,
    {
        self.state().enter_next_task(next_task);
        RadioDriver::new_internal()
    }

    fn switch_off(
        mut self,
    ) -> impl Future<Output = (RadioDriver<NrfRadioDriver, TaskOff>, NsInstant)> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_FALL_BACK);

        let r = radio();
        match r.state.read().state().variant().unwrap() {
            STATE_A::TX_DISABLE | STATE_A::RX_DISABLE | STATE_A::DISABLED => {}
            _ => {
                let state = self.state();
                state.reset_timer();
                state
                    .timer()
                    .observe_event(HardwareEvent::RadioDisabled)
                    .unwrap();
                r.tasks_disable.write(|w| w.tasks_disable().set_bit());
            }
        }
        self.enter_next_task(TaskOff).wait_until_off()
    }

    fn sleep_timer(&self) -> <NrfRadioDriver as DriverConfig>::Timer {
        self.state_ref().sleep_timer()
    }

    fn scheduled_entry(&self) -> OptionalNsInstant {
        self.state_ref().scheduled_entry
    }

    fn set_measured_entry(&mut self, measured_entry: NsInstant) {
        self.state().measured_entry.set(measured_entry)
    }

    fn take_task(&mut self) -> Task {
        self.state().take_task()
    }
}

impl RadioState<NrfRadioDriver, TaskOff> for RadioDriver<NrfRadioDriver, TaskOff> {
    async fn transition(&mut self) -> Result<NsInstant, RadioTaskError<TaskOff>> {
        let state = self.state();
        unsafe {
            state
                .inner()
                .executor
                .spawn(poll_fn(|_| {
                    let r = radio();
                    if r.events_disabled.read().events_disabled().bit_is_set() {
                        r.intenclr.write(|w| w.disabled().set_bit());
                        r.events_disabled.reset();

                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::task_exec_begin(TASK_OFF_ENTRY);

                        Poll::Ready(())
                    } else {
                        r.intenset.write(|w| w.disabled().set_bit());
                        Poll::Pending
                    }
                }))
                .await;
        }

        let entry = state
            .timer()
            .poll_event(HardwareEvent::RadioDisabled)
            .ok_or_else(|| self.scheduling_error());
        state.stop_timer();
        entry
    }

    fn entry(&mut self) -> Result<(), RadioTaskError<TaskOff>> {
        Ok(())
    }

    async fn completion(&mut self, _: bool) -> Result<OffResult, RadioTaskError<TaskOff>> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_COMPLETE);

        Ok(OffResult::Off)
    }

    fn exit(&mut self) -> Result<(), SchedulingError> {
        Ok(())
    }
}

impl OffState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskOff> {
    fn set_channel(&mut self, channel: Channel) {
        let channel: u8 = channel.into();
        let frequency_offset = (channel - 10) * 5;
        radio()
            .frequency
            .write(|w| w.frequency().variant(frequency_offset).map().default());
    }

    fn schedule_rx(
        self,
        rx_task: TaskRx,
        start: OptionalNsInstant,
        channel: Option<Channel>,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskOff, TaskRx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskOff>,
            rx_task: TaskRx,
            start: OptionalNsInstant,
            channel: Option<Channel>,
        }

        impl RadioTransition<NrfRadioDriver, TaskOff, TaskRx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskOff> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                let timed_rx_enable = self.start.map(timed_rx_enable);
                let timer = self
                    .driver
                    .state()
                    .start_timer(timed_rx_enable.map(|ts| ts.instant).into())?;
                if let Some(timed_rx_enable) = timed_rx_enable {
                    timer.schedule_timed_signal(timed_rx_enable)?;
                }
                timer
                    .observe_event(HardwareEvent::RadioRxEnabled)?
                    .observe_event(HardwareEvent::RadioFrameStarted)?;

                let r = radio();

                if let Some(channel) = self.channel {
                    let channel: u8 = channel.into();
                    let frequency_offset = (channel - 10) * 5;
                    r.frequency
                        .write(|w| w.frequency().variant(frequency_offset).map().default());
                }

                // Ramp up the receiver and start frame reception immediately.
                let packetptr = self.rx_task.radio_frame.as_ptr() as u32;
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                dma_start_fence();
                r.shorts.write(|w| {
                    w.rxready_start().enabled();
                    w.framestart_bcstart().enabled()
                });

                if self.start.is_none() {
                    r.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
                }

                Ok(self.start)
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskRx>> {
                radio().shorts.modify(|_, w| w.rxready_start().disabled());
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                false
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskOff>, TaskRx) {
                (self.driver, self.rx_task)
            }
        }

        Transition {
            driver: self,
            rx_task,
            start,
            channel,
        }
    }

    fn schedule_tx(
        self,
        tx_task: TaskTx,
        at: OptionalNsInstant,
        channel: Option<Channel>,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskOff, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskOff>,
            tx_task: TaskTx,
            at: OptionalNsInstant,
            channel: Option<Channel>,
        }

        impl RadioTransition<NrfRadioDriver, TaskOff, TaskTx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskOff> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                let cca = self.tx_task.cca;
                let timed_tx_enable = self.at.map(|instant| timed_tx_enable(instant, cca));
                let timer = self
                    .driver
                    .state()
                    .start_timer(timed_tx_enable.map(|ts| ts.instant).into())?;
                if let Some(timed_tx_enable) = timed_tx_enable {
                    timer.schedule_timed_signal(timed_tx_enable)?;
                }
                timer.observe_event(HardwareEvent::RadioFrameStarted)?;

                let r = radio();

                if let Some(channel) = self.channel {
                    let channel: u8 = channel.into();
                    let frequency_offset = (channel - 10) * 5;
                    r.frequency
                        .write(|w| w.frequency().variant(frequency_offset).map().default());
                }

                let packetptr = prepare_tx_frame(&mut self.tx_task.radio_frame);
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                let is_best_effort = self.at.is_none();
                dma_start_fence();
                if cca {
                    r.shorts.write(|w| {
                        // Start CCA immediately after the receiver ramped up.
                        w.rxready_ccastart().enabled();
                        // If the channel is idle, then ramp up the transmitter and start tx
                        // immediately.
                        w.ccaidle_txen().enabled();
                        w.txready_start().enabled();
                        // If the channel is busy, then disable the receiver.
                        w.ccabusy_disable().enabled()
                    });

                    if is_best_effort {
                        r.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
                    }
                } else {
                    r.shorts.write(|w| w.txready_start().enabled());
                    if is_best_effort {
                        r.tasks_txen.write(|w| w.tasks_txen().set_bit());
                    }
                }

                Ok(self.at)
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskTx>> {
                // Cleanup shorts.
                radio().shorts.reset();
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                false
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskOff>, TaskTx) {
                (self.driver, self.tx_task)
            }
        }

        Transition {
            driver: self,
            tx_task,
            at,
            channel,
        }
    }
}

/// Radio reception states.
///
/// See sub-state specific information below.
impl RadioDriver<NrfRadioDriver, TaskRx> {
    #[inline(always)]
    fn is_back_to_back_rx() -> bool {
        radio().shorts.read().end_start().is_enabled()
    }
}

impl RadioState<NrfRadioDriver, TaskRx> for RadioDriver<NrfRadioDriver, TaskRx> {
    async fn transition(&mut self) -> Result<NsInstant, RadioTaskError<TaskRx>> {
        let state = self.state();
        let is_back_to_back_rx = Self::is_back_to_back_rx();

        // Wait until the state enters.
        unsafe {
            state
                .inner()
                .executor
                .spawn(async {
                    let r = radio();
                    if is_back_to_back_rx {
                        poll_fn(|_| {
                            if r.events_framestart.read().events_framestart().bit_is_set() {
                                r.intenclr.write(|w| w.framestart().set_bit());
                                r.events_framestart.reset();

                                #[cfg(feature = "rtos-trace")]
                                rtos_trace::trace::marker(MARKER_RX_FRAME_STARTED);

                                Poll::Ready(())
                            } else {
                                r.intenset.write(|w| w.framestart().set_bit());
                                Poll::Pending
                            }
                        })
                        .await;
                    } else {
                        poll_fn(|_| {
                            if r.events_rxready.read().events_rxready().bit_is_set() {
                                r.intenclr.write(|w| w.rxready().set_bit());
                                r.events_disabled.reset();
                                r.events_rxready.reset();

                                #[cfg(feature = "rtos-trace")]
                                rtos_trace::trace::task_exec_begin(TASK_RX_ENTRY);

                                Poll::Ready(())
                            } else {
                                r.intenset.write(|w| w.rxready().set_bit());

                                Poll::Pending
                            }
                        })
                        .await;
                    }
                })
                .await;
        }

        let entry_event = if is_back_to_back_rx {
            HardwareEvent::RadioFrameStarted
        } else {
            HardwareEvent::RadioRxEnabled
        };

        let entry = state
            .timer()
            .poll_event(entry_event)
            .map(|ts| match entry_event {
                HardwareEvent::RadioRxEnabled => ts + OQpsk250KBit::T_SHR,
                HardwareEvent::RadioFrameStarted => ts - OQpsk250KBit::T_PHR,
                _ => unreachable!(),
            })
            .ok_or_else(|| self.scheduling_error());

        // Transitioning to the RX state must not stop the timer as it is
        // required to observe the framestart event and (optionally) await the
        // latest frame start.

        if is_back_to_back_rx {
            if let Ok(entry) = entry {
                state.set_rx_frame_started(entry);
            }
        }

        entry
    }

    fn entry(&mut self) -> Result<(), RadioTaskError<TaskRx>> {
        Ok(())
    }

    async fn completion(
        &mut self,
        rollback_on_crcerror: bool,
    ) -> Result<RxResult, RadioTaskError<TaskRx>> {
        let state = self.state();
        let r = radio();

        debug_assert!(state.rx_frame_started().is_some());

        // Wait until (the remainder of) the frame has been received or ongoing
        // reception is cut off at the end of the rx window.
        unsafe {
            state
                .inner()
                .executor
                .spawn(poll_fn(|_| {
                    if r.events_end.read().events_end().bit_is_set() {
                        r.intenclr.write(|w| w.end().set_bit());
                        r.events_end.reset();

                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::task_exec_begin(TASK_RX_COMPLETE);

                        Poll::Ready(())
                    } else {
                        r.intenset.write(|w| w.end().set_bit());
                        Poll::Pending
                    }
                }))
                .await;
        }

        dma_end_fence();

        // We're now in RXIDLE state and received a frame.

        // We reset the BCMATCH event here just in case we didn't retrieve the
        // preliminary frame info for the last rx frame (e.g. if it was an ACK
        // frame) and therefore also didn't reset the event.
        r.events_bcmatch.reset();

        let frame_started_at = state.rx_frame_started().unwrap();
        if r.events_crcok.read().events_crcok().bit_is_set() {
            r.events_crcok.reset();

            // The CRC has been checked so the frame must have a non-zero
            // size saved in the headroom of the nRF buffer (PHY header).
            let rx_task = self.take_task();
            let sdu_length_wo_fcs =
                NonZero::new(rx_task.radio_frame.pdu_ref()[0] as u16 - FCS_LEN as u16)
                    .expect("invalid length");

            Ok(RxResult::Frame(
                rx_task.radio_frame.with_size(sdu_length_wo_fcs),
                frame_started_at,
            ))
        } else {
            r.events_crcerror.reset();

            if rollback_on_crcerror {
                // When rolling back we're expected to place the radio in RX
                // state again and receive the next frame into the same
                // buffer. Therefore restart the receiver unless it was
                // already started. Not required for back-to-back rx as the
                // radio will be re-started by a short in that case.
                if !Self::is_back_to_back_rx() {
                    r.tasks_start.write(|w| w.tasks_start().set_bit());
                }
                Err(RadioTaskError::Task(RxError::CrcError))
            } else {
                Ok(RxResult::CrcError(
                    self.take_task().radio_frame,
                    frame_started_at,
                ))
            }
        }
    }

    fn exit(&mut self) -> Result<(), SchedulingError> {
        radio()
            .shorts
            .modify(|_, w| w.framestart_bcstart().disabled());
        Ok(())
    }
}

/// Listening state.
///
/// Entry: RXREADY event (coming from a non-RX state), FRAMESTART event or RX
///        state respectively (back-to-back reception)
/// Exit: FRAMESTART event or DISABLED (RX window ended)
///
/// State Invariants:
/// - The radio is in the RX or RXIDLE state.
/// - The radio's DMA pointer points to an empty, writable buffer in RAM.
/// - The "FRAMESTART" and "DISABLED" events have been cleared before starting
///   reception.
/// - Only the "FRAMESTART" and "DISABLED" interrupts may be enabled. The latter
///   is only enabled when waiting for the end of the RX window.
impl ListeningRxState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskRx> {
    fn ppdu_rx_duration(&self, psdu_size: u16) -> NsDuration {
        const IMM_ACK_PSDU_OCTETS: u16 = 5;
        const IMM_ACK_PSDU_SYMBOLS: u64 = IMM_ACK_PSDU_OCTETS as u64 * 2;
        const IMM_ACK_PSDU_RX_TIME: NsDuration =
            <PhyOf<NrfRadioDriver> as PhyConfig>::SymbolPeriods::from_ticks(IMM_ACK_PSDU_SYMBOLS)
                .convert();
        const IMM_ACK_PPDU_RX_TIME: NsDuration = OQpsk250KBit::T_SHR
            .checked_add(OQpsk250KBit::T_PHR)
            .unwrap()
            .checked_add(IMM_ACK_PSDU_RX_TIME)
            .unwrap();

        const FULL_PSDU_OCTETS: u16 = <PhyOf<NrfRadioDriver> as PhyConfig>::PHY_MAX_PACKET_SIZE;
        const FULL_PSDU_SYMBOLS: u64 = FULL_PSDU_OCTETS as u64 * 2;
        const FULL_PSDU_RX_TIME: NsDuration =
            <PhyOf<NrfRadioDriver> as PhyConfig>::SymbolPeriods::from_ticks(FULL_PSDU_SYMBOLS)
                .convert();
        const FULL_PPDU_RX_TIME: NsDuration = OQpsk250KBit::T_SHR
            .checked_add(OQpsk250KBit::T_PHR)
            .unwrap()
            .checked_add(FULL_PSDU_RX_TIME)
            .unwrap();

        match psdu_size {
            IMM_ACK_PSDU_OCTETS => IMM_ACK_PPDU_RX_TIME,
            FULL_PSDU_OCTETS => FULL_PPDU_RX_TIME,
            not_precomputed => {
                OQpsk250KBit::T_SHR
                    + OQpsk250KBit::T_PHR
                    + <PhyOf<NrfRadioDriver> as PhyConfig>::SymbolPeriods::from_ticks(
                        not_precomputed as u64 * 2,
                    )
                    .convert()
            }
        }
    }

    async fn wait_for_frame_start(&mut self) -> NsInstant {
        let state = self.state();
        // Shortcut in case we had already observed the framestart event before.
        if let Some(rx_rmarker) = state.rx_frame_started().into() {
            return rx_rmarker;
        }

        unsafe {
            state
                .inner()
                .executor
                .spawn(async {
                    let r = radio();

                    let cleanup_on_drop = CancellationGuard::new(|| {
                        r.intenclr.write(|w| w.framestart().set_bit());
                    });

                    poll_fn(|_| {
                        if r.events_framestart.read().events_framestart().bit_is_set() {
                            r.intenclr.write(|w| w.framestart().set_bit());
                            r.events_framestart.reset();

                            #[cfg(feature = "rtos-trace")]
                            rtos_trace::trace::marker(MARKER_RX_FRAME_STARTED);

                            Poll::Ready(())
                        } else {
                            r.intenset.write(|w| w.framestart().set_bit());
                            Poll::Pending
                        }
                    })
                    .await;

                    cleanup_on_drop.inactivate();
                })
                .await;
        }

        let rx_rmarker = state
            .timer()
            .poll_event(HardwareEvent::RadioFrameStarted)
            .unwrap();
        state.set_rx_frame_started(rx_rmarker);
        rx_rmarker
    }

    async fn stop_listening(
        mut self,
        latest_frame_start: OptionalNsInstant,
    ) -> Result<
        StopListeningResult<NrfRadioDriver, impl ReceivingRxState<NrfRadioDriver>>,
        (SchedulingError, Self),
    > {
        let state = self.state();

        // Shortcut in case we had already observed the framestart event before.
        if let Some(rx_rmarker) = state.rx_frame_started().into() {
            state.stop_timer();
            return Ok(StopListeningResult::FrameStarted(rx_rmarker, self));
        }

        let timer = state.timer();
        if let Err(err) = timer.observe_event(HardwareEvent::RadioDisabled) {
            return Err((err.into(), self));
        }

        let r = radio();

        let disable_at = latest_frame_start.map(|ts| ts + OQpsk250KBit::T_PHR);
        let mut disabled = false;
        let mut should_disable = false;
        if let Some(disable_at) = disable_at {
            // Window widening is the responsibility of the client. Therefore,
            // the latest frame start designates an exact RMARKER in terms of
            // the local radio clock. The nRF driver's framestart event fires
            // after the PHY header, i.e. one byte later.
            match timer.schedule_timed_signal_unless(
                timed_off(disable_at),
                HardwareEvent::RadioFrameStarted,
            ) {
                Err(RadioTimerError::Already) => {
                    // The frame already started.
                    r.events_framestart.reset();
                }
                Err(RadioTimerError::Overdue(_)) => {
                    if r.events_framestart
                        .read()
                        .events_framestart()
                        .bit_is_clear()
                    {
                        should_disable = true;
                    }
                }
                Err(err) => {
                    // Let the timer running as we're not leaving the listening state.
                    return Err((err.into(), self));
                }
                Ok(_) => unsafe {
                    state
                        .inner()
                        .executor
                        .spawn(poll_fn(|_| {
                            disabled = r.events_disabled.read().events_disabled().bit_is_set();
                            if disabled
                                || r.events_framestart.read().events_framestart().bit_is_set()
                            {
                                r.intenclr.write(|w| {
                                    w.framestart().set_bit();
                                    w.disabled().set_bit()
                                });
                                r.events_framestart.reset();
                                // Do not reset the disabled event which is needed
                                // to transition to the off state.

                                if disabled {
                                    #[cfg(feature = "rtos-trace")]
                                    rtos_trace::trace::marker(MARKER_RX_WINDOW_ENDED);
                                } else {
                                    #[cfg(feature = "rtos-trace")]
                                    rtos_trace::trace::marker(MARKER_RX_FRAME_STARTED);
                                }

                                Poll::Ready(())
                            } else {
                                r.intenset.write(|w| {
                                    w.framestart().set_bit();
                                    w.disabled().set_bit()
                                });
                                Poll::Pending
                            }
                        }))
                        .await;
                },
            }
        } else {
            should_disable = true;
        }

        if should_disable {
            r.tasks_disable.write(|w| w.tasks_disable().set_bit());
            // Disabling RX is so fast that scheduling an interrupt doesn't make
            // sense.
            while r.events_disabled.read().events_disabled().bit_is_clear() {}
            // Do not reset the disabled event which is needed to transition to
            // the off state.

            #[cfg(feature = "rtos-trace")]
            rtos_trace::trace::marker(MARKER_RX_WINDOW_ENDED);

            disabled = true;
        }

        let result = if disabled {
            dma_end_fence();
            self.rx_window_ended(
                disable_at
                    .map(|ts| ts + T_RXDIS - T_MEASURED_RAMP_DOWN_ERROR)
                    .into(),
            )
            .await
        } else {
            debug_assert!(r.events_disabled.read().events_disabled().bit_is_clear());

            let rx_rmarker = state
                .timer()
                .poll_event(HardwareEvent::RadioFrameStarted)
                .unwrap();
            state.stop_timer();
            state.set_rx_frame_started(rx_rmarker);
            StopListeningResult::FrameStarted(rx_rmarker, self)
        };

        Ok(result)
    }
}

/// Receiving state.
///
/// Entry: FRAMESTART event
/// Exit: END event when a frame started, otherwise immediate.
///
/// State Invariants:
/// - The radio is in the RX or RXIDLE state.
/// - The "END" and "CRCOK" events have been cleared before starting reception.
/// - Only the "END" interrupt may be enabled.
impl ReceivingRxState<NrfRadioDriver> for RadioDriver<NrfRadioDriver, TaskRx> {
    async fn preliminary_frame_info(&mut self) -> Option<PreliminaryFrameInfo<'_>> {
        // Wait until the frame control field has been received.
        const FC_LEN: usize = 2;
        const SEQ_NR_LEN: usize = 1;

        let mut preliminary_frame_info: Option<PreliminaryFrameInfo> = None;

        unsafe {
            self.state()
                .inner()
                .executor
                .spawn(async {
                    let r = radio();

                    let cleanup_on_drop = CancellationGuard::new(|| {
                        r.intenclr.write(|w| {
                            w.bcmatch().set_bit();
                            w.end().set_bit()
                        });

                        dma_end_fence();

                        // Do not clear the end event as it is used in the rx
                        // completion() method.
                        r.tasks_bcstop.write(|w| w.tasks_bcstop().set_bit());
                        r.bcc.write(|w| w.bcc().variant(BCC_FC_BITS));
                        r.events_bcmatch.reset();
                    });

                    let ended = poll_fn(|_| {
                        let bcmatch = r.events_bcmatch.read().events_bcmatch().bit_is_set();
                        if bcmatch || r.events_end.read().events_end().bit_is_set() {
                            // Do not clear the end event as it is used below and in the
                            // rx completion() method.
                            r.intenclr.write(|w| {
                                w.bcmatch().set_bit();
                                w.end().set_bit()
                            });
                            r.events_bcmatch.reset();
                            return Poll::Ready(!bcmatch);
                        }

                        r.intenset.write(|w| {
                            w.bcmatch().set_bit();
                            w.end().set_bit()
                        });
                        Poll::Pending
                    })
                    .await;

                    dma_end_fence();

                    if ended && r.events_crcerror.read().events_crcerror().bit_is_set() {
                        return;
                    }

                    // Cannot use self::ref_task() as we need to borrow self.task and
                    // self.inner at the same time.
                    let radio_frame = &self.state_ref().ref_task::<TaskRx>().radio_frame;

                    // Safety: The bit counter match guarantees that the frame
                    //         control field has been received.
                    let fc_and_addressing_repr = radio_frame.fc_and_addressing_repr();

                    if fc_and_addressing_repr.is_err() {
                        return;
                    }

                    let (frame_control, addressing_repr) = fc_and_addressing_repr.unwrap();
                    let addressing_fields_lengths = addressing_repr.try_addressing_fields_lengths();

                    if addressing_fields_lengths.is_err() {
                        return;
                    }

                    let seq_nr_len = if frame_control.sequence_number_suppression() {
                        0
                    } else {
                        SEQ_NR_LEN
                    };

                    let [dst_pan_id_len, dst_addr_len, ..] = addressing_fields_lengths.unwrap();
                    let dst_len = (dst_pan_id_len + dst_addr_len) as usize;

                    let pdu_ref = radio_frame.pdu_ref();
                    let mpdu_length = pdu_ref[0] as u16;

                    if seq_nr_len == 0 && dst_len == 0 {
                        preliminary_frame_info = Some(PreliminaryFrameInfo {
                            mpdu_length,
                            frame_control: Some(frame_control),
                            seq_nr: None,
                            addressing_fields: None,
                        });
                        return;
                    }

                    let ended = if !ended {
                        // Note: BCMATCH counts are calculated relative to the MPDU
                        //       (i.e. w/o headroom).
                        let bcc = ((FC_LEN + seq_nr_len + dst_len) as u32) << 3;

                        // Wait until the sequence number and/or destination address
                        // fields have been received.
                        r.bcc.write(|w| w.bcc().variant(bcc));

                        poll_fn(|_| {
                            let bcmatch = r.events_bcmatch.read().events_bcmatch().bit_is_set();
                            if bcmatch || r.events_end.read().events_end().bit_is_set() {
                                // Do not clear the end event as it is used in the rx
                                // completion() method. The remaining cleanup will be
                                // done by the cancel guard.
                                return Poll::Ready(!bcmatch);
                            }

                            r.intenset.write(|w| {
                                w.bcmatch().set_bit();
                                w.end().set_bit()
                            });
                            Poll::Pending
                        })
                        .await
                    } else {
                        true
                    };

                    drop(cleanup_on_drop);

                    dma_end_fence();

                    if ended {
                        if r.events_crcerror.read().events_crcerror().bit_is_set() {
                            return;
                        } else {
                            debug_assert!(r.events_crcok.read().events_crcok().bit_is_set());
                        }
                    }

                    const HEADROOM: usize = 1;

                    let seq_nr_offset = HEADROOM + FC_LEN;
                    let seq_nr = if frame_control.sequence_number_suppression() {
                        None
                    } else {
                        Some(pdu_ref[seq_nr_offset])
                    };

                    let addressing_offset = seq_nr_offset + seq_nr_len;
                    let addressing_bytes = &pdu_ref[addressing_offset..];

                    // Safety: We checked that we do actually have addressing fields.
                    //         The bit counter guarantees that all bytes up to the
                    //         addressing fields have been received.
                    let addressing_fields =
                        AddressingFields::new_unchecked(addressing_bytes, addressing_repr);

                    preliminary_frame_info = Some(PreliminaryFrameInfo {
                        mpdu_length,
                        frame_control: Some(frame_control),
                        seq_nr,
                        addressing_fields: Some(addressing_fields),
                    });
                })
                .await
        };

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker(MARKER_RX_FRAME_INFO);

        Some(preliminary_frame_info.unwrap_or(PreliminaryFrameInfo {
            mpdu_length: 0,
            frame_control: None,
            seq_nr: None,
            addressing_fields: None,
        }))
    }

    fn schedule_rx(
        self,
        rx_task: TaskRx,
        ifs: Option<Ifs>,
        rollback_on_crcerror: bool,
    ) -> impl SelfRadioTransition<NrfRadioDriver, TaskRx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_RX_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskRx>,
            rx_task: TaskRx,
            ifs: Option<Ifs>,
            rollback_on_crcerror: bool,
        }

        impl RadioTransition<NrfRadioDriver, TaskRx, TaskRx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskRx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                let timer = self.driver.state().start_timer(None.into())?;

                let is_back_to_back_rx = self.ifs.is_none();
                if is_back_to_back_rx {
                    // Back-to-back is only allowed if the rx window has ended
                    // with frame reception, i.e. the radio is still enabled.
                    debug_assert!(
                        (radio().state.read().state().bits()
                            & (STATE_A::RX as u8 | STATE_A::RX_IDLE as u8))
                            != 0
                    );
                } else {
                    timer.observe_event(HardwareEvent::RadioRxEnabled)?;
                }
                timer.observe_event(HardwareEvent::RadioFrameStarted)?;

                let r = radio();

                let packetptr = self.rx_task.radio_frame.as_ptr() as u32;
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                // Enable back-to-back frame reception.
                //
                // Note: We need to set up the short before checking radio state to
                //       avoid race conditions, see RX_IDLE case below.
                dma_start_fence();
                r.shorts.write(|w| {
                    if let Some(ifs) = self.ifs {
                        set_ifs(Some(ifs), true);
                        w.end_disable().enabled();
                        w.disabled_rxen().enabled();
                        w.rxready_start().enabled();
                    } else {
                        w.end_start().enabled();
                    }

                    w.framestart_bcstart().enabled()
                });

                Ok(None.into())
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                // Check whether the task completed before we were able to
                // automate the transition.
                //
                // Note: Read the state _after_ having set the short.
                let r = radio();
                if r.state.read().state().is_rx_idle()
                    && r.events_framestart
                        .read()
                        .events_framestart()
                        .bit_is_clear()
                {
                    // We're idle, although we have a short in place: This means
                    // that the previous frame was fully received before we were
                    // able to set the short, i.e. reception of the new frame
                    // was not started by hardware, we need to start it
                    // manually, see conditions 1. and 2. in the method
                    // documentation.
                    //
                    // TODO: We currently have no way to enforce IFS in this
                    //       case.
                    let is_back_to_back_rx = self.ifs.is_none();
                    if is_back_to_back_rx {
                        r.tasks_start.write(|w| w.tasks_start().set_bit());
                    } else {
                        // If end->disable short was set, radio must be disabled
                        // first to enforce TIFS. disabled->rxen short will then
                        // start reception.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                    }

                    debug!("late scheduling");
                };

                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskRx>> {
                // Clean up shorts: The framestart-bcstart short must
                // remain enabled.
                radio().shorts.write(|w| w.framestart_bcstart().enabled());
                set_ifs(None, false);
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                self.rollback_on_crcerror
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskRx>, TaskRx) {
                (self.driver, self.rx_task)
            }
        }

        Transition {
            driver: self,
            rx_task,
            ifs,
            rollback_on_crcerror,
        }
    }

    fn schedule_tx(
        self,
        tx_task: TaskTx,
        at: OptionalNsInstant,
        ifs: Option<Ifs>,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskRx, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskRx>,
            tx_task: TaskTx,
            at: OptionalNsInstant,
            ifs: Option<Ifs>,
            rollback_on_crcerror: bool,
        }

        impl RadioTransition<NrfRadioDriver, TaskRx, TaskTx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskRx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                let cca = self.tx_task.cca;
                let is_best_effort = self.at.is_none();

                let timed_signals = self.at.map(|at| {
                    let tx_enable = timed_tx_enable(at, cca);
                    // No need for window-widening as it is the client's
                    // responsibility to cater for clock drift.
                    let rx_off_at = if let Some(ifs) = self.ifs {
                        tx_enable.instant + if cca { T_RXEN } else { T_TXEN }
                            - NsDuration::from(ifs)
                    } else {
                        tx_enable.instant - T_RXDIS
                    };
                    (timed_off(rx_off_at), tx_enable)
                });
                let timer = self
                    .driver
                    .state()
                    .start_timer(timed_signals.map(|ts| ts.0.instant).into())?;
                if let Some((rx_off, tx_enable)) = timed_signals {
                    debug_assert!(rx_off.instant < tx_enable.instant);

                    timer
                        .schedule_timed_signal(rx_off)?
                        .schedule_timed_signal(tx_enable)?;
                }
                timer.observe_event(HardwareEvent::RadioFrameStarted)?;

                let r = radio();

                // PACKETPTR is double buffered so we don't cause a race by setting it
                // while reception might still be ongoing.
                let packetptr = prepare_tx_frame(&mut self.tx_task.radio_frame);
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                set_ifs(self.ifs, false);

                // Note: We need to set up shorts before completing the task to
                //       avoid race conditions, see RX_IDLE case below.
                dma_start_fence();
                if cca {
                    // TODO: In case of no IFS, we could use timed rx stop and
                    //       cca start events rather than disabling for faster
                    //       turnaround.
                    r.shorts.write(|w| {
                        // Ramp down and up again for proper IFS and CCA timing.
                        w.end_disable().enabled();
                        if is_best_effort {
                            w.disabled_rxen().enabled();
                        }
                        w.rxready_ccastart().enabled();
                        // If the channel is idle, then ramp up and start tx
                        // immediately.
                        w.ccaidle_txen().enabled();
                        w.txready_start().enabled();
                        // If the channel is busy, then disable the receiver so
                        // that we reach the fallback state.
                        w.ccabusy_disable().enabled()
                    });
                } else {
                    r.shorts.write(|w| {
                        // Ramp down and directly switch to TX state w/o CCA
                        // including IFS timing.
                        w.end_disable().enabled();
                        if is_best_effort {
                            w.disabled_txen().enabled();
                        }
                        w.txready_start().enabled()
                    });
                }

                Ok(self.at)
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                let r = radio();

                // Check whether the task completed before we were able to
                // automate the transition.
                //
                // Note: Read the state _after_ having set the shorts.
                if r.state.read().state().is_rx_idle() {
                    // We're idle, although we have a short in place: This means
                    // that the previous frame was fully received before we were
                    // able to set the short, i.e. transmission of the new frame
                    // was not started by hardware, we need to start it
                    // manually, see conditions 1. and 2. in the method
                    // documentation.
                    //
                    // TODO: We currently have no way to enforce IFS in this
                    //       case.
                    r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                }

                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskTx>> {
                // Clean up shorts.
                radio().shorts.reset();
                set_ifs(None, false);
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                self.rollback_on_crcerror
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskRx>, TaskTx) {
                (self.driver, self.tx_task)
            }
        }

        Transition {
            driver: self,
            tx_task,
            at,
            ifs,
            rollback_on_crcerror,
        }
    }

    fn schedule_off(
        self,
        at: OptionalNsInstant,
        rollback_on_crcerror: bool,
    ) -> impl ExternalRadioTransition<NrfRadioDriver, TaskRx, TaskOff> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskRx>,
            at: OptionalNsInstant,
            rollback_on_crcerror: bool,
        }

        impl RadioTransition<NrfRadioDriver, TaskRx, TaskOff> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskRx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                // We're in RX or RXIDLE state and need to ramp down the
                // receiver.
                let timer = self.driver.state().start_timer(self.at)?;
                if let Some(at) = self.at.into() {
                    timer.schedule_timed_signal(timed_off(at))?;
                }
                timer.observe_event(HardwareEvent::RadioDisabled)?;

                // Ramp down the receiver.
                //
                // Note: We need to set up the short before completing the task
                //       to avoid race conditions, see RX_IDLE case below.
                //
                // Note: It's ok to leave the short on even in the timed case,
                //       as we don't have to schedule anything after the disable
                //       task.
                radio().shorts.write(|w| w.end_disable().enabled());
                Ok(self.at)
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                // Check whether the task completed before we were able to
                // automate the transition.
                //
                // Note: Read the state _after_ having set the short.
                let r = radio();
                match r.state.read().state().variant() {
                    Some(STATE_A::RX_IDLE) => {
                        // We're idle, although we have a short in place: This
                        // means that the previous frame was fully received
                        // before we were able to set the short.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                    }
                    Some(STATE_A::RX_DISABLE | STATE_A::DISABLED) => {}
                    _ => unreachable!(),
                };

                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskOff>> {
                // Cleanup shorts.
                radio().shorts.reset();
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                self.rollback_on_crcerror
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskRx>, TaskOff) {
                (self.driver, TaskOff)
            }
        }

        Transition {
            driver: self,
            at,
            rollback_on_crcerror,
        }
    }
}

/// Radio transmission state.
///
/// Entry: FRAMESTART event
/// Exit: END event
///
/// State Invariants:
/// - The radio is in the TX or TXIDLE state.
/// - The radio's DMA pointer points to an empty buffer in RAM.
/// - The "END" event has been cleared before starting transmission.
/// - Only the "END" interrupt is enabled.
///
/// Note: On entry, we await FRAMESTART rather than the TXREADY so that we can
///       reliably reset this event in case a subsequent rx task is scheduled
///       which needs to observe that event.
impl RadioState<NrfRadioDriver, TaskTx> for RadioDriver<NrfRadioDriver, TaskTx> {
    async fn transition(&mut self) -> Result<NsInstant, RadioTaskError<TaskTx>> {
        let state = self.state();
        let r = radio();

        let tx_task = state.ref_task::<TaskTx>();
        if tx_task.cca {
            unsafe {
                state
                    .inner()
                    .executor
                    .spawn(poll_fn(|_| {
                        if r.events_ccaidle.read().events_ccaidle().bit_is_set()
                            || r.events_ccabusy.read().events_ccabusy().bit_is_set()
                        {
                            r.intenclr.write(|w| {
                                w.ccaidle().set_bit();
                                w.ccabusy().set_bit()
                            });
                            Poll::Ready(())
                        } else {
                            r.intenset.write(|w| {
                                w.ccaidle().set_bit();
                                w.ccabusy().set_bit()
                            });
                            Poll::Pending
                        }
                    }))
                    .await;
            }

            r.events_rxready.reset();

            if r.events_ccabusy.read().events_ccabusy().bit_is_set() {
                r.events_ccabusy.reset();
                return Err(RadioTaskError::Task(TxError::CcaBusy(
                    self.take_task().radio_frame,
                )));
            }

            r.events_ccaidle.reset();
        }

        // Wait until the state enters.
        unsafe {
            state
                .inner()
                .executor
                .spawn(poll_fn(|_| {
                    if r.events_framestart.read().events_framestart().bit_is_set() {
                        r.intenclr.write(|w| w.framestart().set_bit());
                        r.events_disabled.reset();
                        // Reliably reset the framestart event as a
                        // pre-condition to being able to schedule (and observe)
                        // a subsequent rx task.
                        r.events_framestart.reset();

                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::task_exec_begin(TASK_TX_ENTRY);

                        Poll::Ready(())
                    } else {
                        r.intenset.write(|w| w.framestart().set_bit());
                        Poll::Pending
                    }
                }))
                .await;
        }

        let entry = state
            .timer()
            .poll_event(HardwareEvent::RadioFrameStarted)
            .map(|ts| ts - T_MEASURED_TX_FRAMESTART_ERROR)
            .ok_or_else(|| self.scheduling_error());
        state.stop_timer();
        entry
    }

    fn entry(&mut self) -> Result<(), RadioTaskError<TaskTx>> {
        Ok(())
    }

    async fn completion(&mut self, _: bool) -> Result<TxResult, RadioTaskError<TaskTx>> {
        let state = self.state();
        let r = radio();

        unsafe {
            state
                .inner()
                .executor
                .spawn(poll_fn(|_| {
                    if r.events_end.read().events_end().bit_is_set() {
                        r.intenclr.write(|w| w.end().set_bit());
                        r.events_end.reset();

                        #[cfg(feature = "rtos-trace")]
                        rtos_trace::trace::task_exec_begin(TASK_TX_COMPLETE);

                        Poll::Ready(())
                    } else {
                        r.intenset.write(|w| w.end().set_bit());
                        Poll::Pending
                    }
                }))
                .await;
        }

        dma_end_fence();

        let radio_frame = state.take_task::<TaskTx>().radio_frame;
        Ok(TxResult::Sent(radio_frame, state.measured_entry.unwrap()))
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

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskTx>,
            rx_task: TaskRx,
            ifs: Ifs,
        }

        impl RadioTransition<NrfRadioDriver, TaskTx, TaskRx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskTx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                self.driver
                    .state()
                    .start_timer(None.into())?
                    .observe_event(HardwareEvent::RadioRxEnabled)?
                    .observe_event(HardwareEvent::RadioFrameStarted)?;

                let r = radio();

                let packetptr = self.rx_task.radio_frame.as_ptr() as u32;
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                set_ifs(Some(self.ifs), true);

                // Note: To cater for errata 204 (see rev1 v1.4) a tx-to-rx
                //       switch must pass through the disabled state, which is
                //       what the shorts imply anyway.

                // Note: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see the TX_IDLE case below.
                dma_start_fence();
                r.shorts.write(|w| {
                    // Ramp down the receiver, ramp it up again in RX state and
                    // then start frame reception immediately.
                    w.end_disable().enabled();
                    w.disabled_rxen().enabled();
                    w.rxready_start().enabled()
                });

                Ok(None.into())
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                let r = radio();

                // We need to schedule the BCSTART short _after_ the FRAMESTART
                // event of the tx frame, otherwise the tx frame will trigger
                // the BCMATCH event already.
                r.shorts.modify(|_, w| w.framestart_bcstart().enabled());

                // Check whether the task completed before we were able to
                // automate the transition.
                //
                // Note: Only read the state _after_ having set the short.
                if r.state.read().state().is_tx_idle() {
                    // We're idle, although we have a short in place: This means
                    // that the previous frame was sent before we were able to
                    // set the short, i.e., hardware did not start transitioning
                    // to RX, we need to transition manually, see conditions 1.
                    // and 2. in the method documentation.
                    //
                    // TODO: We currently have no way to enforce IFS in this
                    //       case.
                    r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                    debug!("late scheduling");
                };
                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskRx>> {
                // Clean up shorts: The framestart-bcstart short must
                // remain enabled.
                radio().shorts.write(|w| w.framestart_bcstart().enabled());
                set_ifs(None, false);
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                false
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskTx>, TaskRx) {
                (self.driver, self.rx_task)
            }
        }

        Transition {
            driver: self,
            rx_task,
            ifs,
        }
    }

    fn schedule_tx(
        self,
        tx_task: TaskTx,
        ifs: Ifs,
    ) -> impl SelfRadioTransition<NrfRadioDriver, TaskTx> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_TX_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskTx>,
            tx_task: TaskTx,
            ifs: Ifs,
        }

        impl RadioTransition<NrfRadioDriver, TaskTx, TaskTx> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskTx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                self.driver
                    .state()
                    .start_timer(None.into())?
                    .observe_event(HardwareEvent::RadioFrameStarted)?;

                // Note: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see the TX_IDLE case below.
                let r = radio();

                let packetptr = prepare_tx_frame(&mut self.tx_task.radio_frame);
                r.packetptr.write(|w| w.packetptr().variant(packetptr));

                set_ifs(Some(self.ifs), false);

                dma_start_fence();
                if self.tx_task.cca {
                    r.shorts.write(|w| {
                        // Ramp down the transceiver, ramp it up again in RX state and
                        // then start CCA immediately.
                        w.end_disable().enabled();
                        w.disabled_rxen().enabled();
                        w.rxready_ccastart().enabled();

                        // If the channel is idle, then ramp up and start tx immediately.
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

                Ok(None.into())
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                // Check whether the task completed before we were able to
                // automate the transition.
                //
                // Note: Read the state _after_ having set the short.
                let r = radio();
                if r.state.read().state().is_tx_idle() {
                    // We check whether a second frame was already sent
                    // just in the unlikely case that we got here so late
                    // that the next task was already executed.
                    if r.events_end.read().events_end().bit_is_clear() {
                        // We're idle, although we have a short in place: This
                        // means that the previous frame was sent before we were
                        // able to set the short, i.e., hardware did not start
                        // transitioning to TX, we need to transition manually,
                        // see conditions 1. and 2. in the method documentation.
                        //
                        // TODO: We currently have no way to enforce IFS in this
                        //       case.
                        r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                        debug!("late scheduling");
                    } else {
                        // The radio has already self-transitioned via shorts
                        // and fully executed the next tx task. It should stay
                        // in tx idle to uphold tx state invariants for the next
                        // schedule call.
                        //
                        // We won't be able to capture a framestart event in
                        // that case but we'll discover that when trying to
                        // retrieve a tx timestamp.
                        debug!("slow completion");
                    }
                }

                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskTx>> {
                // Clean up shorts.
                radio().shorts.reset();
                set_ifs(None, false);
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                false
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskTx>, TaskTx) {
                (self.driver, self.tx_task)
            }
        }

        Transition {
            driver: self,
            tx_task,
            ifs,
        }
    }

    fn schedule_off(self) -> impl ExternalRadioTransition<NrfRadioDriver, TaskTx, TaskOff> {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_exec_begin(TASK_OFF_SCHEDULE);

        struct Transition {
            driver: RadioDriver<NrfRadioDriver, TaskTx>,
        }

        impl RadioTransition<NrfRadioDriver, TaskTx, TaskOff> for Transition {
            fn driver(&mut self) -> &mut RadioDriver<NrfRadioDriver, TaskTx> {
                &mut self.driver
            }

            fn on_scheduled(&mut self) -> Result<OptionalNsInstant, SchedulingError> {
                self.driver
                    .state()
                    .start_timer(None.into())?
                    .observe_event(HardwareEvent::RadioDisabled)?;

                // Ramp down the receiver.
                //
                // Note: We need to set up the shorts before checking radio state to
                //       avoid race conditions, see TX_IDLE case below.
                //
                // Note: It's ok to leave the short on even in the timed case,
                //       as we don't have to schedule anything after the disable
                //       task.
                radio().shorts.write(|w| w.end_disable().enabled());

                Ok(None.into())
            }

            fn on_completed(&mut self) -> Result<(), SchedulingError> {
                // Check whether the transition has already triggered. If not then wait
                // until it triggers.
                //
                // Note: Read the state _after_ having set the short.
                let r = radio();
                if r.state.read().state().is_tx_idle() {
                    // We're idle, although we have a short in place: This means
                    // that the previous frame was sent before we were able to
                    // set the short, i.e., hardware did not disable the
                    // receiver, we need to disable it manually, see conditions
                    // 1.  and 2. in the method documentation.
                    r.tasks_disable.write(|w| w.tasks_disable().set_bit());
                };

                Ok(())
            }

            fn cleanup() -> Result<(), RadioTaskError<TaskOff>> {
                // Clean up shorts.
                radio().shorts.reset();
                Ok(())
            }

            fn alt_outcome_is_error(&self) -> bool {
                false
            }

            fn consume(self) -> (RadioDriver<NrfRadioDriver, TaskTx>, TaskOff) {
                (self.driver, TaskOff)
            }
        }

        Transition { driver: self }
    }
}

fn prepare_tx_frame(radio_frame: &mut RadioFrame<RadioFrameSized>) -> u32 {
    let sdu_length = radio_frame.sdu_wo_fcs_length().get() as u8 + FCS_LEN;
    // Set PHY HDR.
    radio_frame.pdu_mut()[0] = sdu_length;
    // Return PACKETPTR.
    radio_frame.as_ptr() as u32
}

/// This method must be called after all normal memory write accesses to the
/// buffer and before the volatile write operation passing the buffer pointer to
/// DMA hardware.
fn dma_start_fence() {
    // Note the explanation re using compiler fences with volatile accesses
    // rather than atomics in
    // <https://docs.rust-embedded.org/embedonomicon/dma.html>. The example
    // there is basically correct except that the fence should be placed before
    // passing the pointer to the hardware, not after.
    //
    // Other relevant sources:
    // - Interaction between volatile and fence:
    //   <https://github.com/rust-lang/unsafe-code-guidelines/issues/260>
    // - RFC re volatile access - including DMA discussions:
    //   <https://github.com/rust-lang/unsafe-code-guidelines/issues/321#issuecomment-2894697770>
    // - Compiler fence and DMA:
    //   <https://users.rust-lang.org/t/compiler-fence-dma/132027/39>
    // - asm! as memory barrier:
    //   <https://users.rust-lang.org/t/how-to-correctly-use-asm-as-memory-barrier-llvm-question/132105>
    // - Why asm! cannot (yet) be used as a barrier:
    //   <https://github.com/rust-lang/rust/issues/144351>
    compiler_fence(Ordering::Release);
}

/// This method must be called after any volatile read operation confirming that
/// the DMA has finished and before normal memory read accesses to the buffer.
fn dma_end_fence() {
    compiler_fence(Ordering::Acquire);
}

nrf_interrupt_executor!(executor, RADIO);
