use core::{cell::Cell, marker::PhantomData};

use dot15d4::{
    driver::radio::{
        tasks::{ListeningRxState, OffState, RadioDriverApi, TaskOff, TaskRx, TaskTx, TxState},
        DriverConfig, RadioDriver,
    },
    mac::{MacBufferAllocator, MacIndicationChannel, MacRequestChannel},
    Device, RngCore,
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

pub struct Ieee802154Stack<RadioDriverImpl: DriverConfig, Rng: RngCore> {
    buffer_allocator: MacBufferAllocator,
    request_channel: MacRequestChannel,
    indication_channel: MacIndicationChannel,
    radio: Cell<Option<RadioDriver<RadioDriverImpl, TaskOff>>>,
    hardware_addr: HardwareAddress,
    driver: PhantomData<RadioDriverImpl>,
    rng: Rng,
}

impl<RadioDriverImpl: DriverConfig, Rng: RngCore> Ieee802154Stack<RadioDriverImpl, Rng>
where
    RadioDriver<RadioDriverImpl, TaskOff>: RadioDriverApi<RadioDriverImpl, TaskOff>,
{
    pub fn new(
        radio: RadioDriver<RadioDriverImpl, TaskOff>,
        buffer_allocator: MacBufferAllocator,
        rng: Rng,
    ) -> Self {
        let hardware_addr = HardwareAddress::Ieee802154(radio.ieee802154_address());
        Self {
            buffer_allocator,
            request_channel: MacRequestChannel::new(),
            indication_channel: MacIndicationChannel::new(),
            radio: Cell::new(Some(radio)),
            hardware_addr,
            driver: PhantomData,
            rng,
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

impl<RadioDriverImpl: DriverConfig, Rng: RngCore> Ieee802154Stack<RadioDriverImpl, Rng>
where
    RadioDriver<RadioDriverImpl, TaskOff>: OffState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, TaskRx>: ListeningRxState<RadioDriverImpl>,
    RadioDriver<RadioDriverImpl, TaskTx>: TxState<RadioDriverImpl>,
{
    pub async fn run(&mut self) -> ! {
        let radio = self.radio.take().expect("already running");
        let timer = radio.sleep_timer();
        let device = Device::new(radio);
        device
            .run(
                self.buffer_allocator,
                self.request_channel.receiver(),
                self.indication_channel.sender(),
                timer,
                &mut self.rng,
            )
            .await
    }
}
