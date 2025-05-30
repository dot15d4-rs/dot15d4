use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

/// Type representing 2 possible outcomes/states
#[derive(Debug, PartialEq)]
pub enum Either<T, S> {
    First(T),
    Second(S),
}

impl<T, S> Either<T, S> {
    pub fn is_first(&self) -> bool {
        matches!(self, Either::First(_))
    }

    pub fn is_second(&self) -> bool {
        matches!(self, Either::Second(_))
    }
}

/// Combines 2 futures and returns the result of the first future to terminate.
/// The other one gets canceled/dropped
pub fn select<F1: Future, F2: Future>(
    f1: F1,
    f2: F2,
) -> impl Future<Output = Either<F1::Output, F2::Output>> {
    SelectFuture { f1, f2 }
}

struct SelectFuture<F1, F2> {
    f1: F1,
    f2: F2,
}

impl<F1, F2> Future for SelectFuture<F1, F2>
where
    F1: Future,
    F2: Future,
{
    type Output = Either<F1::Output, F2::Output>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        if let Poll::Ready(res) = unsafe { Pin::new_unchecked(&mut this.f1) }.poll(cx) {
            return Poll::Ready(Either::First(res));
        }
        if let Poll::Ready(res) = unsafe { Pin::new_unchecked(&mut this.f2) }.poll(cx) {
            return Poll::Ready(Either::Second(res));
        }

        Poll::Pending
    }
}

#[cfg(test)]
mod tests {
    use core::future::poll_fn;
    use core::task::Poll;

    use pollster::FutureExt as _;

    use crate::sync::Either;

    use super::select;

    #[test]
    pub fn test_select_immediate_ready_first() {
        async {
            let f1 = poll_fn(|_| Poll::Ready(1));
            let f2 = poll_fn(|_| Poll::Ready(2));

            assert_eq!(select(f1, f2).await, Either::First(1));
        }
        .block_on();
    }

    #[test]
    pub fn test_select_immediate_ready_second() {
        async {
            let f1 = poll_fn(|_| Poll::<()>::Pending);
            let f2 = poll_fn(|_| Poll::Ready(2));

            assert_eq!(select(f1, f2).await, Either::Second(2));
        }
        .block_on();
    }

    #[test]
    pub fn test_select_wait_until_one_finished() {
        async {
            let mut counter = 10;
            let f1 = poll_fn(move |cx| {
                if counter == 0 {
                    Poll::Ready(())
                } else {
                    counter -= 1;
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            });
            let f2 = poll_fn(|_| Poll::<()>::Pending);

            assert_eq!(select(f1, f2).await, Either::First(()));
        }
        .block_on();
    }

    #[test]
    pub fn test_select_wait_until_second_finished() {
        async {
            let mut counter = 10;
            let f1 = poll_fn(|_| Poll::<()>::Pending);
            let f2 = poll_fn(move |cx| {
                if counter == 0 {
                    Poll::Ready(())
                } else {
                    counter -= 1;
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            });

            assert_eq!(select(f1, f2).await, Either::Second(()));
        }
        .block_on();
    }
}
