pub mod export {
    #[cfg(feature = "defmt")]
    pub use rtt_target::set_defmt_channel;
    pub use rtt_target::{rtt_init, DownChannel, UpChannel};
}

pub const RTT_SYNC_BUF_LEN: usize = 16;

#[macro_export]
macro_rules! rtt_channels {
    // Macro Entry
    ($( $channel:literal:$name:tt ),+ ) => {
        $crate::rtt::rtt_channels!{ _channels { up: {} down: {} } tail { $( $channel:$name )+ } }
    };

    // Add Defmt/Log/Terminal channel.
    (
        _channels { up: { $( $up:tt )* } down: { $( $down:tt )* } }
        tail { $channel:literal:terminal $( $tail:tt )* }
    ) => {
        $crate::rtt::rtt_channels!{
            _channels {
                up: { $( $up )* $channel: { size: 1024, name: "Terminal" } }
                down: { $( $down )* $channel: { size: $crate::rtt::RTT_SYNC_BUF_LEN, name: "Terminal" } }
            }
            tail { $( $tail )* }
        }
    };

    // Add SystemView channel.
    (
        _channels { up: { $( $up:tt )* } down: { $( $down:tt )* } }
        tail { $channel:literal:systemview $( $tail:tt )* }
    ) => {
        $crate::rtt::rtt_channels!{
            _channels {
                up: { $( $up )* $channel: { } }
                down: { $( $down )* $channel: { } }
            }
            tail { $( $tail )* }
        }
    };

    // Macro Exit
    (
        _channels { up: { $( $up:tt )* } down: { $( $down:tt )* } }
        tail { }
    ) => {
        $crate::rtt::export::rtt_init!{
            up: { $( $up )+ }
            down: { $( $down )+ }
        };
    }
}

pub use rtt_channels;

#[cfg(feature = "rtos-trace")]
#[macro_export]
macro_rules! init_rtt_channels { () => { $crate::rtt::rtt_channels!(0:terminal, 1:systemview) }; }

#[cfg(not(feature = "rtos-trace"))]
#[macro_export]
macro_rules! init_rtt_channels { () => { $crate::rtt::rtt_channels!(0:terminal) }; }

pub use init_rtt_channels;
