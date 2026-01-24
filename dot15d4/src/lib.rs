#![cfg_attr(not(feature = "std"), no_std)]
pub mod constants;
pub mod driver;
pub mod mac;
pub mod pib;
pub mod scheduler;
mod utils;

use dot15d4_driver::radio::tasks::RadioDriverApi;
pub use dot15d4_util as util;
pub use rand_core::{Error as RngError, RngCore};

use self::{
    driver::{
        radio::{
            tasks::{
                ListeningRxState, OffState, TaskOff as RadioTaskOff, TaskRx as RadioTaskRx,
                TaskTx as RadioTaskTx, TxState,
            },
            DriverConfig, RadioDriver,
        },
        DriverEventChannel, DriverRequestChannel, DriverService,
    },
    mac::{MacBufferAllocator, MacIndicationSender, MacRequestReceiver, MacService},
    scheduler::{SchedulerRequestChannel, SchedulerService},
    util::sync::{select, Either},
};

pub struct Device<RadioDriverImpl: DriverConfig> {
    radio: RadioDriver<RadioDriverImpl, RadioTaskOff>,
}

impl<RadioDriverImpl: DriverConfig> Device<RadioDriverImpl> {
    pub fn new(radio: RadioDriver<RadioDriverImpl, RadioTaskOff>) -> Self {
        Self { radio }
    }
}

impl<RadioDriverImpl: DriverConfig> Device<RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, RadioTaskOff>: OffState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, RadioTaskRx>: ListeningRxState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, RadioTaskTx>: TxState<RadioDriverImpl>,
{
    pub async fn run<'upper_layer>(
        self,
        buffer_allocator: MacBufferAllocator,
        request_receiver: MacRequestReceiver<'upper_layer>,
        indication_sender: MacIndicationSender<'upper_layer>,
        timer: RadioDriverImpl::Timer,
        mut rng: &'upper_layer mut dyn RngCore,
    ) -> ! {
        #[cfg(feature = "rtos-trace")]
        self::trace::instrument();

        let driver_request_channel = DriverRequestChannel::new();
        let driver_response_channel = DriverEventChannel::new();
        let scheduler_service_channel = SchedulerRequestChannel::new();

        let ieee802154_address = self.radio.ieee802154_address();

        let driver_service = DriverService::new(
            self.radio,
            driver_request_channel.receiver(),
            driver_response_channel.sender(),
            buffer_allocator,
        );

        let mut scheduler_service = SchedulerService::<'_, RadioDriverImpl>::new(
            timer,
            scheduler_service_channel.receiver(),
            driver_request_channel.sender(),
            driver_response_channel.receiver(),
            buffer_allocator,
            &mut rng,
            &ieee802154_address,
        );

        let mut mac_service = MacService::<'_, RadioDriverImpl>::new(
            timer,
            buffer_allocator,
            request_receiver,
            indication_sender,
            scheduler_service_channel.sender(),
        );

        match select::select(
            select::select(mac_service.run(), driver_service.run()),
            scheduler_service.run(),
        )
        .await
        {
            Either::First(either) => match either {
                Either::First(_) => panic!("MAC service terminated"),
                Either::Second(_) => panic!("Driver service terminated"),
            },
            Either::Second(_) => panic!("Scheduler service terminated"),
        }
    }
}

#[cfg(feature = "rtos-trace")]
pub mod trace {
    use crate::util::trace::TraceOffset;

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
