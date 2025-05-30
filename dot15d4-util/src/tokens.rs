//! Basic infrastructure for linear types or what we call "tokens".

use core::mem;

/// A utility communicating the intent that tokens _should_ be used as linear
/// types i.e. they must not be dropped unless explicitly consumed.
///
/// A token can still be leaked in several ways which would neutralize the drop
/// guard. That's ok. This pattern is not meant to be literally foolproof, just
/// to keep most users from accidentally doing the wrong thing in practice.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct TokenGuard;

impl TokenGuard {
    /// Consumes the token.
    pub(crate) const fn consume(self) {
        mem::forget(self);
    }
}

impl Drop for TokenGuard {
    fn drop(&mut self) {
        panic!("Tokens must not be dropped. Always return them to the originator.")
    }
}
