#[cfg(feature = "executor")]
pub mod executor;
#[cfg(feature = "radio")]
mod radio;
#[cfg(feature = "timer")]
mod timer;

pub mod export {
    #[cfg(feature = "radio")]
    pub use super::radio::export::*;
    pub use nrf52840_pac as pac;
}

#[cfg(feature = "radio")]
pub use radio::*;
#[cfg(feature = "timer")]
pub use timer::*;
