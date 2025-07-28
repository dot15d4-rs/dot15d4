use core::{cell::Cell, marker::PhantomData};

use dot15d4::{
    driver::{
        radio::{DriverConfig, RadioDriverApi},
        tasks::{OffState, RadioDriver, RxState, TaskOff, TaskRx, TaskTx, TxState},
    },
    export::*,
    mac::{MacBufferAllocator, MacIndicationChannel, MacRequestChannel},
    Device,
};
use embassy_net_driver::HardwareAddress;

use crate::driver::Ieee802154Driver;

pub mod export {
    pub use dot15d4::mac::{MAC_BUFFER_SIZE, MAC_NUM_REQUIRED_BUFFERS};
    pub use dot15d4::util::buffer_allocator;
}

#[macro_export]
macro_rules! mac_buffer_allocator {
    () => {{
        use $crate::export::{buffer_allocator, MAC_BUFFER_SIZE, MAC_NUM_REQUIRED_BUFFERS};

        buffer_allocator!(MAC_BUFFER_SIZE, MAC_NUM_REQUIRED_BUFFERS)
    }};
}

pub struct Ieee802154Stack<RadioDriverImpl: DriverConfig> {
    buffer_allocator: MacBufferAllocator,
    request_channel: MacRequestChannel,
    indication_channel: MacIndicationChannel,
    radio: Cell<Option<RadioDriver<RadioDriverImpl, TaskOff>>>,
    hardware_addr: HardwareAddress,
    driver: PhantomData<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> Ieee802154Stack<RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, TaskOff>: RadioDriverApi,
{
    pub fn new(
        radio: RadioDriver<RadioDriverImpl, TaskOff>,
        buffer_allocator: MacBufferAllocator,
    ) -> Self {
        let hardware_addr = HardwareAddress::Ieee802154(radio.ieee802154_address());
        Self {
            buffer_allocator,
            request_channel: MacRequestChannel::new(),
            indication_channel: MacIndicationChannel::new(),
            radio: Cell::new(Some(radio)),
            hardware_addr,
            driver: PhantomData,
        }
    }

    pub fn driver(&self) -> Ieee802154Driver<'_, RadioDriverImpl> {
        Ieee802154Driver::new(
            self.buffer_allocator,
            self.request_channel.sender(),
            self.indication_channel.receiver(),
            self.hardware_addr,
        )
    }
}

impl<RadioDriverImpl: DriverConfig> Ieee802154Stack<RadioDriverImpl>
where
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, TaskRx>: RxState<RadioDriverImpl> + RadioDriverApi,
    RadioDriver<RadioDriverImpl, TaskTx>: TxState<RadioDriverImpl> + RadioDriverApi,
{
    pub async fn run<Rng: RngCore>(&self, rng: Rng) -> ! {
        let radio = self.radio.take().expect("already running");
        let device = Device::new(radio, rng);
        device
            .run(
                self.buffer_allocator,
                self.request_channel.receiver(),
                self.indication_channel.sender(),
            )
            .await
    }
}
