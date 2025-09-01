pub mod executor;
mod radio;
mod timer;

pub mod export {
    pub use super::radio::export::*;
}

pub use radio::*;
pub use timer::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmChannel {
    Timer = 0,
    Rtc1,
    Rtc2,
    NumAlarmChannels,
}
