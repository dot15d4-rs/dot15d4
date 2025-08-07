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
    Event = 0,
    Cpu,
    NumAlarmChannels,
}
