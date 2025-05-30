//! Logger backend agnostic logging

#[cfg(all(feature = "defmt", feature = "log"))]
compile_error!("Cannot select log and defmt features together.");

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(feature = "log")]
pub use log::{debug, error, info, trace, warn};

#[cfg(not(any(feature = "defmt", feature = "log")))]
#[allow(unused_macros)]
#[macro_export]
macro_rules! error {
    ($($arg:tt),*) => {{ // no-op
    }};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
#[allow(unused_macros)]
#[macro_export]
macro_rules! warn {
    ($($arg:tt),*) => {{ // no-op
    }};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
#[allow(unused_macros)]
#[macro_export]
macro_rules! info {
    ($($arg:tt),*) => {{ // no-op
    }};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
#[allow(unused_macros)]
#[macro_export]
macro_rules! debug {
    ($($arg:tt),*) => {{ // no-op
    }};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
#[allow(unused_macros)]
#[macro_export]
macro_rules! trace {
    ($($arg:tt),*) => {{ // no-op
    }};
}
