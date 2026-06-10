//! Basic infrastructure for linear types or what we call "tokens".

use core::mem;

/// A utility communicating the intent that tokens _should_ be used as linear
/// types i.e. they must not be dropped unless explicitly consumed.
///
/// A token can still be leaked in several ways which would neutralize the drop
/// guard. That's ok. This pattern is not meant to be literally foolproof, just
/// to keep most users from accidentally doing the wrong thing in practice.
#[derive(Debug, PartialEq, Eq)]
#[must_use = "Tokens are linear and must be returned to their originator, not dropped."]
pub(crate) struct TokenGuard;

impl TokenGuard {
    /// Consumes the token.
    pub(crate) const fn consume(self) {
        mem::forget(self);
    }
}

impl Drop for TokenGuard {
    fn drop(&mut self) {
        record_token_leak();
        crate::warn!("Linear token dropped instead of returned to its originator; leaking it.");
    }
}

#[cfg(debug_assertions)]
static LEAKED_TOKENS: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

#[cfg(debug_assertions)]
#[inline]
fn record_token_leak() {
    LEAKED_TOKENS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
}

#[cfg(not(debug_assertions))]
#[inline(always)]
fn record_token_leak() {}

pub fn leaked_tokens() -> usize {
    #[cfg(debug_assertions)]
    {
        LEAKED_TOKENS.load(core::sync::atomic::Ordering::Relaxed)
    }
    #[cfg(not(debug_assertions))]
    {
        0
    }
}

pub fn reset_leaked_tokens() {
    #[cfg(debug_assertions)]
    LEAKED_TOKENS.store(0, core::sync::atomic::Ordering::Relaxed);
}
