use dot15d4_driver::radio::{config::Channel, frame::FrameType};

#[cfg(feature = "tsch-log")]
macro_rules! tsch_info {
    ($fmt:literal $(, $arg:expr)*) => { ::dot15d4_util::info!($fmt $(, $arg)*) };
}

#[cfg(not(feature = "tsch-log"))]
macro_rules! tsch_info {
    ($fmt:literal $(, $arg:expr)*) => {{ $( let _ = &$arg; )* }};
}

#[cfg(feature = "tsch-log")]
macro_rules! tsch_debug {
    ($fmt:literal $(, $arg:expr)*) => { ::dot15d4_util::debug!($fmt $(, $arg)*) };
}

#[cfg(not(feature = "tsch-log"))]
macro_rules! tsch_debug {
    ($fmt:literal $(, $arg:expr)*) => {{ $( let _ = &$arg; )* }};
}

#[cfg(feature = "tsch-log")]
macro_rules! tsch_trace {
    ($fmt:literal $(, $arg:expr)*) => { ::dot15d4_util::trace!($fmt $(, $arg)*) };
}

#[cfg(not(feature = "tsch-log"))]
macro_rules! tsch_trace {
    ($fmt:literal $(, $arg:expr)*) => {{ $( let _ = &$arg; )* }};
}

#[cfg(feature = "tsch-log")]
macro_rules! tsch_warn {
    ($fmt:literal $(, $arg:expr)*) => { ::dot15d4_util::warn!($fmt $(, $arg)*) };
}

#[cfg(not(feature = "tsch-log"))]
macro_rules! tsch_warn {
    ($fmt:literal $(, $arg:expr)*) => {{ $( let _ = &$arg; )* }};
}

pub(crate) use tsch_debug;
pub(crate) use tsch_info;
pub(crate) use tsch_trace;
pub(crate) use tsch_warn;

pub(crate) fn ch(channel: Channel) -> u8 {
    u8::from(channel)
}

pub(crate) fn ft(frame_type: FrameType) -> &'static str {
    match frame_type {
        FrameType::Beacon => "beacon",
        FrameType::Data => "data",
        FrameType::Ack => "ack",
        FrameType::MacCommand => "cmd",
        FrameType::Multipurpose => "multipurpose",
        FrameType::FragmentOrFrak => "frag",
        FrameType::Extended => "ext",
        FrameType::Unknown => "unknown",
    }
}
