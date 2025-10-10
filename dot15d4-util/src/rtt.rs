pub mod export {
    #[cfg(feature = "defmt")]
    pub use rtt_target::set_defmt_channel;
    pub use rtt_target::{rtt_init, DownChannel, UpChannel};
}

pub const RTT_SYNC_BUF_LEN: usize = 20;

#[macro_export]
macro_rules! rtt_channels {
    // Macro Entry
    ($( $channel:literal:$name:tt ),+ ) => {
        $crate::rtt::rtt_channels!{ _channels { up: {} down: {} } tail { $( $channel:$name )+ } }
    };

    // Add Defmt/Log channel.
    (
        _channels { up: { $( $up:tt )* } down: { $( $down:tt )* } }
        tail { $channel:literal:terminal $( $tail:tt )* }
    ) => {
        $crate::rtt::rtt_channels!{
            _channels {
                up: { $( $up )* $channel: { size: 1024, name: "Terminal" } }
                down: { $( $down )* $channel: { size: 16, name: "Terminal" } }
            }
            tail { $( $tail )* }
        }
    };

    // Add Sync channel.
    (
        _channels { up: { $( $up:tt )* } down: { $( $down:tt )* } }
        tail { $channel:literal:sync $( $tail:tt )* }
    ) => {
        $crate::rtt::rtt_channels!{
            _channels {
                up: { $( $up )* $channel: { size: $crate::rtt::RTT_SYNC_BUF_LEN, name: "Sync" } }
                down: { $( $down )* $channel: { size: $crate::rtt::RTT_SYNC_BUF_LEN, name: "Sync" } }
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

// If rtos-trace is enabled then we always need to allocate three channels
// although the sync channel might not be needed. This is due to the
// systemview-target crate requiring a fixed number of external RTT channels to
// be allocated. As this is not a production feature the overhead is tolerable.
#[cfg(feature = "rtos-trace")]
#[macro_export]
macro_rules! init_rtt_channels { () => { $crate::rtt::rtt_channels!(0:terminal, 1:sync, 2:systemview) }; }

#[cfg(all(feature = "sync", not(feature = "rtos-trace")))]
#[macro_export]
macro_rules! init_rtt_channels { () => { $crate::rtt::rtt_channels!(0:terminal, 1:sync) }; }

#[cfg(all(not(feature = "sync"), not(feature = "rtos-trace")))]
#[macro_export]
macro_rules! init_rtt_channels { () => { $crate::rtt::rtt_channels!(0:terminal) }; }

pub use init_rtt_channels;
