use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::future::Future;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::pin::Pin;
use core::task::Waker;
use core::task::{Context, Poll};

struct MutexState {
    locked: bool,
    waker: Option<Waker>,
}

/// A generic mutex that is independent on the underlying async runtime.
/// The idea is that this is used to synchronize different parts inside 1 single
/// task that may run concurrently through `select`.
pub struct Mutex<T> {
    value: UnsafeCell<T>,
    state: RefCell<MutexState>,
    _no_send_sync: PhantomData<*mut T>, // Probably not needed as we have `UnsafeCell`
}

impl<T> Mutex<T> {
    pub fn new(value: T) -> Self {
        Mutex {
            value: UnsafeCell::new(value),
            state: RefCell::new(MutexState {
                locked: false,
                waker: None,
            }),
            _no_send_sync: PhantomData,
        }
    }

    pub async fn lock(&self) -> MutexGuard<'_, T> {
        // Wait until we can acquire the lock
        LockFuture { mutex: self }.await;

        // Now that we have acquired the lock, we can return the mutex
        MutexGuard { mutex: self }
    }

    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        let mut state = self.state.borrow_mut();
        if !state.locked {
            // The lock is currently not yet locked -> acquire lock
            state.locked = true;
            Some(MutexGuard { mutex: self })
        } else {
            // The current lock is locked, return None
            None
        }
    }

    /// Get access to the protected value inside the mutex. This is similar to
    /// the Mutex::get_mut in std.
    pub fn get_mut(&mut self) -> &mut T {
        // Safety: &mut gives us exclusive access to T
        self.value.get_mut()
    }

    /// # Safety
    /// Only use this method if you are sure there are no locks currently taken.
    /// If you have a mutable reference, prefer to use the `get_mut` method
    /// instead.
    pub unsafe fn read(&self) -> &T {
        &*self.value.get()
    }
}

/// Represents current exclusive access to the resource protected by a mutex
pub struct MutexGuard<'a, T> {
    mutex: &'a Mutex<T>,
}

impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // Safety: Only one mutex can exist at a time
        unsafe { &*self.mutex.value.get() }
    }
}

impl<T> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // Safety: Only one mutex can exist at a time
        unsafe { &mut *self.mutex.value.get() }
    }
}

impl<T> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        let mut mutex_state = self.mutex.state.borrow_mut();

        // Release the lock
        mutex_state.locked = false;

        // Call the waker if needed
        if let Some(waker) = mutex_state.waker.take() {
            waker.wake()
        }
    }
}

struct LockFuture<'a, T> {
    mutex: &'a Mutex<T>,
}

impl<T> Future for LockFuture<'_, T> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut mutex_state = self.mutex.state.borrow_mut();
        if mutex_state.locked {
            // Mutex is locked here, wake the previous task, so it can make progress, and we
            // do not have to remember it
            let new_waker = cx.waker();
            match &mut mutex_state.waker {
                // We already have the same waker stored, do not wake
                Some(waker) if waker.will_wake(new_waker) => {
                    waker.clone_from(new_waker);
                }
                // New waker, wake the previous and store current
                waker @ Some(_) => {
                    waker.take().unwrap().wake();
                    *waker = Some(new_waker.clone());
                }
                // No waker yet, store the new one
                waker @ None => *waker = Some(new_waker.clone()),
            };

            // Mutex is locked, keep waiting
            Poll::Pending
        } else {
            // Mutex is unlocked, lock it
            mutex_state.locked = true;

            Poll::Ready(())
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use pollster::FutureExt as _;

    use crate::sync::select::select;

    use super::Mutex;

    use core::{
        future::{poll_fn, Future},
        pin::pin,
        task::Poll,
    };
    use std::boxed::Box;

    #[test]
    pub fn test_mutex_no_concurrency() {
        async {
            let mut mutex = Mutex::new(0usize);
            {
                let mut guard = mutex.lock().await;
                *guard += 1;
                assert_eq!(*guard, 1, "The guard should be readable");
            }

            assert_eq!(
                *mutex.get_mut(),
                1,
                "The internal mutex should have been updated"
            )
        }
        .block_on()
    }

    #[test]
    pub fn test_mutex_select_concurrency() {
        async {
            let mut mutex = Mutex::new(0usize);
            for _ in 0..100 {
                select(
                    async {
                        let mut guard = mutex.lock().await;
                        *guard += 1;
                    },
                    async {
                        let mut guard = mutex.lock().await;
                        *guard += 1;
                    },
                )
                .await;
            }

            assert_eq!(*mutex.get_mut(), 100);
        }
        .block_on()
    }

    #[test]
    pub fn test_try_lock() {
        async {
            let mutex = Mutex::new(0usize);
            let mut fut1 = pin!(async {
                let mut counter = mutex.lock().await;

                poll_fn(|_| {
                    if *counter < 10 {
                        *counter += 1;
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                })
                .await
            });
            let mut fut2 = pin!(async {
                let mut failed_to_unlock = 0;

                poll_fn(|_| {
                    if let Some(mut counter) = mutex.try_lock() {
                        *counter += 1;
                        Poll::Ready(())
                    } else {
                        failed_to_unlock += 1;
                        Poll::Pending
                    }
                })
                .await;

                assert_eq!(failed_to_unlock, 10, "Try lock takes to long!");
            });

            poll_fn(move |cx| {
                // Ensure liveness.
                cx.waker().wake_by_ref();

                let mut fut1_pending = true;
                let mut fut2_pending = true;

                if fut1_pending {
                    fut1_pending = matches!(fut1.as_mut().poll(cx), Poll::Pending);
                }
                if fut2_pending {
                    fut2_pending = matches!(fut2.as_mut().poll(cx), Poll::Pending);
                }

                if fut1_pending || fut2_pending {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            assert_eq!(*mutex.try_lock().unwrap(), 11);
        }
        .block_on()
    }

    #[test]
    /// Check with Miri whether or not drop is called correctly. If true, then
    /// all heap allocation should be deallocated correctly
    pub fn test_drop_by_leaking() {
        async {
            let mutex = Mutex::new(Box::new(0));
            let _guard = mutex.lock().await;
        }
        .block_on()
    }
}
