//! A handful of executor independent synchronization primitives.
//!
//! The goal is to provide synchronization or communication across or within
//! tasks
pub mod cancellation;
pub mod channel;
pub mod mutex;
pub mod select;

pub use cancellation::*;
pub use channel::*;
pub use mutex::*;
pub use select::*;
