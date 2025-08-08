#![cfg_attr(not(feature = "std"), no_std)]
pub mod driver;
pub mod mac;

pub use dot15d4_util as util;

pub mod export {
    pub use rand_core::RngCore;
}

use rand_core::RngCore;

use self::{
    driver::{
        radio::{DriverConfig, RadioDriverApi},
        tasks::{
            OffState, RadioDriver, RxState, TaskOff as RadioTaskOff, TaskRx as RadioTaskRx,
            TaskTx as RadioTaskTx, TxState,
        },
        DriverRequestChannel, DriverService,
    },
    mac::{MacBufferAllocator, MacIndicationSender, MacRequestReceiver, MacService},
    util::sync::{mutex::Mutex, select, Either},
};

pub struct Device<RadioDriverImpl: DriverConfig, Rng> {
    radio: RadioDriver<RadioDriverImpl, RadioTaskOff>,
    rng: Mutex<Rng>,
}

impl<RadioDriverImpl: DriverConfig, Rng: RngCore> Device<RadioDriverImpl, Rng> {
    pub fn new(radio: RadioDriver<RadioDriverImpl, RadioTaskOff>, rng: Rng) -> Self {
        Self {
            radio,
            rng: Mutex::new(rng),
        }
    }
}

impl<RadioDriverImpl: DriverConfig, Rng: RngCore> Device<RadioDriverImpl, Rng>
where
    RadioDriver<RadioDriverImpl, RadioTaskOff>: OffState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskRx>: RxState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, RadioTaskTx>: TxState<RadioDriverImpl> + RadioDriverApi,
{
    pub async fn run<'upper_layer>(
        mut self,
        buffer_allocator: MacBufferAllocator,
        request_receiver: MacRequestReceiver<'upper_layer>,
        indication_sender: MacIndicationSender<'upper_layer>,
    ) -> ! {
        #[cfg(feature = "rtos-trace")]
        self::trace::instrument();

        let driver_service_channel = DriverRequestChannel::new();
        let driver_service = DriverService::new(
            self.radio,
            driver_service_channel.receiver(),
            buffer_allocator,
        );
        let mut mac_service = MacService::<'_, Rng, RadioDriverImpl>::new(
            &mut self.rng,
            buffer_allocator,
            request_receiver,
            indication_sender,
            driver_service_channel.sender(),
        );

        match select::select(mac_service.run(), driver_service.run()).await {
            Either::First(_) => panic!("MAC service terminated"),
            Either::Second(_) => panic!("Driver service terminated"),
        }
    }

    // pub async fn start_as_coordinator(&mut self) {
    //     self.scan_energy().await;
    // }

    // pub async fn start(&mut self) {
    //     //
    //     self.scan_channels().await;
    // }

    // async fn receive_beacon_request<'a>(
    //     &self,
    //     buffer: &mut [u8; 128],
    //     radio_guard: &mut Option<MutexGuard<'a, R>>,
    // ) {
    //     receive(
    //         &mut **radio_guard.as_mut().unwrap(),
    //         buffer,
    //         RxConfig {
    //             channel: crate::phy::config::Channel::_26,
    //         },
    //     )
    //     .await;
    // }
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    use dot15d4_util::trace::TraceOffset;

    #[cfg(feature = "defmt")]
    compile_error!(
        "Tracing cannot be enabled at the same time as defmt. Logs will be visible in the SystemView application if the 'log' feature is enabled."
    );

    const OFFSET: TraceOffset = TraceOffset::Dot15d4;

    // Tasks
    pub const MAC_INDICATION: u32 = OFFSET.wrap(0);
    pub const MAC_REQUEST: u32 = OFFSET.wrap(1);

    // Markers
    pub const TX_FRAME: u32 = OFFSET.wrap(0);
    pub const TX_NACK: u32 = OFFSET.wrap(1);
    pub const TX_CCABUSY: u32 = OFFSET.wrap(2);
    pub const RX_FRAME: u32 = OFFSET.wrap(3);
    pub const RX_INVALID: u32 = OFFSET.wrap(4);
    pub const RX_CRC_ERROR: u32 = OFFSET.wrap(5);
    pub const RX_WINDOW_ENDED: u32 = OFFSET.wrap(6);

    /// Instrument the library for tracing.
    pub(crate) fn instrument() {
        rtos_trace::trace::task_new_stackless(MAC_INDICATION, "MAC indication\0", 0);
        rtos_trace::trace::task_new_stackless(MAC_REQUEST, "MAC request\0", 0);
        rtos_trace::trace::name_marker(TX_FRAME, "TX frame\0");
        rtos_trace::trace::name_marker(TX_NACK, "TX NACK\0");
        rtos_trace::trace::name_marker(TX_CCABUSY, "TX CCA Busy\0");
        rtos_trace::trace::name_marker(RX_FRAME, "RX frame\0");
        rtos_trace::trace::name_marker(RX_INVALID, "RX invalid frame\0");
        rtos_trace::trace::name_marker(RX_CRC_ERROR, "RX CRC error\0");
        rtos_trace::trace::name_marker(RX_WINDOW_ENDED, "RX window ended\0");
    }
}
