use core::mem;

/// Futures cancellation guard.
#[must_use = "Must be inactivated when the future is not cancelled."]
pub struct CancellationGuard<F: FnMut()> {
    on_cancellation: F,
}

impl<F: FnMut()> CancellationGuard<F> {
    /// The given closure will be run when the guard is dropped before it was
    /// inactivated. This can be used to clean-up on cancellation of a future.
    pub fn new(on_cancellation: F) -> Self {
        Self { on_cancellation }
    }

    /// Prevent drop handler from running.
    pub fn inactivate(self) {
        mem::forget(self)
    }
}

impl<F: FnMut()> Drop for CancellationGuard<F> {
    fn drop(&mut self) {
        (self.on_cancellation)()
    }
}
