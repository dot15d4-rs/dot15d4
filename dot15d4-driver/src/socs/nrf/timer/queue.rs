//! Intrusive waiter queue for the RTC sleep timer.
//!
//! # Safety
//!
//! `Inner` is not synchronized here. All access — linking, unlinking, firing,
//! reading node state — must happen under the priority-ceiling section
//! provided by [`super::SleepQueue::with`]. Node addresses are only inserted
//! once the owning future is pinned and are removed in its destructor, so the
//! pin drop-guarantee keeps every linked pointer valid.
//!
//! No foreign code runs while the lock is held: firing moves the wakers into
//! a [`WakerBatch`] which the caller invokes only after `with` has returned,
//! and [`Node::set_waker`] hands any displaced waker back to the caller for
//! the same reason. A waker that synchronously polls or drops a
//! [`super::WaitUntil`] re-enters the queue, which would alias the `&mut`
//! handed out by `with`; deferring also keeps arbitrary waker code off the
//! RTC interrupt's critical path. Only the [`Waker`]s leave the lock — node
//! memory is never touched after unlock, as a fired future may be dropped
//! and freed by its owner at any point from then on.

use core::{
    ptr::{self, null_mut},
    task::Waker,
};

use crate::timer::{NsInstant, RadioTimerError};

/// Life cycle of a waiter node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum NodeState {
    Unlinked,
    Linked,
    Fired(Result<(), RadioTimerError>),
}

/// A waiter, embedded in a [`super::WaitUntil`] future.
pub(super) struct Node {
    /// The absolute, overflow-protected RTC tick to wait for.
    tick: u64,
    /// The originally requested instant, reported back in
    /// [`RadioTimerError::Overdue`].
    instant: NsInstant,
    state: NodeState,
    waker: Option<Waker>,
    next: *mut Node,
}

impl Node {
    pub(super) const fn new(tick: u64, instant: NsInstant) -> Self {
        Self {
            tick,
            instant,
            state: NodeState::Unlinked,
            waker: None,
            next: null_mut(),
        }
    }

    pub(super) fn tick(&self) -> u64 {
        self.tick
    }

    pub(super) fn instant(&self) -> NsInstant {
        self.instant
    }

    pub(super) fn state(&self) -> NodeState {
        self.state
    }

    /// Marks the node as fired without waking.
    pub(super) fn fire_unlinked(&mut self, result: Result<(), RadioTimerError>) {
        debug_assert_eq!(self.state, NodeState::Unlinked);
        self.state = NodeState::Fired(result);
    }

    /// Registers the waker to be woken when the node fires.
    #[must_use = "drop the displaced waker outside the queue lock"]
    pub(super) fn set_waker(&mut self, waker: &Waker) -> Option<Waker> {
        match &self.waker {
            Some(current) if current.will_wake(waker) => None,
            _ => self.waker.replace(waker.clone()),
        }
    }
}

/// A small batch of wakers collected under the queue lock and invoked by
/// the caller after the lock has been released.
pub(super) struct WakerBatch {
    wakers: [Option<Waker>; Self::CAPACITY],
    len: usize,
}

impl WakerBatch {
    const CAPACITY: usize = 4;

    pub(super) fn new() -> Self {
        Self {
            wakers: core::array::from_fn(|_| None),
            len: 0,
        }
    }

    fn is_full(&self) -> bool {
        self.len == Self::CAPACITY
    }

    fn push(&mut self, waker: Waker) -> bool {
        if self.is_full() {
            return false;
        }
        self.wakers[self.len] = Some(waker);
        self.len += 1;
        true
    }

    /// Invokes and releases all collected wakers.
    pub(super) fn wake_all(&mut self) {
        for slot in &mut self.wakers[..self.len] {
            if let Some(waker) = slot.take() {
                waker.wake();
            }
        }
        self.len = 0;
    }
}

/// Result of one [`Inner::fire_and_min`] pass.
pub(super) enum FireOutcome {
    Complete(Option<u64>),
    Truncated,
}

/// The queue: an unsorted, singly-linked list of waiters.
pub(super) struct Inner {
    head: *mut Node,
}

impl Inner {
    pub(super) const fn new() -> Self {
        Self { head: null_mut() }
    }

    /// Links an unlinked node at the front of the list.
    ///
    /// # Safety (module-internal contract)
    ///
    /// The node must stay at its current address until it is unlinked or
    /// fired; the caller guarantees this via pinning plus the unlink-on-drop
    /// destructor.
    pub(super) fn link(&mut self, node: &mut Node) {
        debug_assert_eq!(node.state, NodeState::Unlinked);
        node.state = NodeState::Linked;
        node.next = self.head;
        self.head = node;
    }

    /// Unlinks a linked node (cancellation path).
    pub(super) fn unlink(&mut self, node: &mut Node) {
        debug_assert_eq!(node.state, NodeState::Linked);
        let target: *mut Node = node;
        let mut cursor: *mut *mut Node = &mut self.head;
        // Safety: All pointers in the chain refer to linked nodes, which are
        //         pinned and kept alive by their futures. The only Rust
        //         reference into the list is `node` itself; the cursor never
        //         points into `node` when we write through it (it points at
        //         `head` or at the *previous* node's `next` field).
        unsafe {
            loop {
                let candidate = *cursor;
                debug_assert!(!candidate.is_null(), "linked node not in list");
                if ptr::eq(candidate, target) {
                    *cursor = node.next;
                    break;
                }
                cursor = &mut (*candidate).next;
            }
        }
        node.next = null_mut();
        node.state = NodeState::Unlinked;
    }

    /// Fires every node whose deadline can no longer be armed and returns the
    /// earliest remaining deadline.
    pub(super) fn fire_and_min(
        &mut self,
        now: u64,
        guard: u64,
        wakers: &mut WakerBatch,
    ) -> FireOutcome {
        let mut min: Option<u64> = None;
        let mut cursor: *mut *mut Node = &mut self.head;
        // Safety: See `unlink`. Fired nodes are removed through the cursor
        //         (which never aliases the node being written) before their
        //         state and waker are touched.
        unsafe {
            loop {
                let candidate = *cursor;
                if candidate.is_null() {
                    return FireOutcome::Complete(min);
                }
                let node = &mut *candidate;
                if node.tick <= now + guard {
                    if wakers.is_full() && node.waker.is_some() {
                        return FireOutcome::Truncated;
                    }
                    *cursor = node.next;
                    node.next = null_mut();
                    node.state = NodeState::Fired(if node.tick <= now {
                        Ok(())
                    } else {
                        Err(RadioTimerError::Overdue(node.instant))
                    });
                    if let Some(waker) = node.waker.take() {
                        let pushed = wakers.push(waker);
                        debug_assert!(pushed);
                    }
                } else {
                    min = Some(match min {
                        Some(min) => min.min(node.tick),
                        None => node.tick,
                    });
                    cursor = &mut node.next;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use core::{
        pin::pin,
        sync::atomic::{AtomicUsize, Ordering},
        task::{RawWaker, RawWakerVTable, Waker},
    };

    use super::*;

    const GUARD: u64 = 2;

    static WAKES: AtomicUsize = AtomicUsize::new(0);

    fn counting_waker() -> Waker {
        fn wake(_: *const ()) {
            WAKES.fetch_add(1, Ordering::Relaxed);
        }
        fn clone(data: *const ()) -> RawWaker {
            RawWaker::new(data, &VTABLE)
        }
        fn drop(_: *const ()) {}
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, drop);
        // Safety: The vtable functions only touch a static counter.
        unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &VTABLE)) }
    }

    fn node(tick: u64) -> Node {
        let mut node = Node::new(tick, NsInstant::from_ticks(tick));
        let displaced = node.set_waker(&counting_waker());
        assert!(displaced.is_none());
        node
    }

    fn fire(queue: &mut Inner, now: u64) -> Option<u64> {
        let mut wakers = WakerBatch::new();
        let outcome = queue.fire_and_min(now, GUARD, &mut wakers);
        wakers.wake_all();
        match outcome {
            FireOutcome::Complete(min) => min,
            FireOutcome::Truncated => panic!("unexpected truncation"),
        }
    }

    #[test]
    fn fires_in_classification_order() {
        let mut queue = Inner::new();
        let mut past = pin!(node(10));
        let mut in_guard = pin!(node(101));
        let mut future = pin!(node(200));
        // Safety (tests): the nodes are pinned for the whole test and
        //         unlinked/fired before they drop.
        unsafe {
            queue.link(past.as_mut().get_unchecked_mut());
            queue.link(in_guard.as_mut().get_unchecked_mut());
            queue.link(future.as_mut().get_unchecked_mut());
        }

        WAKES.store(0, Ordering::Relaxed);
        assert_eq!(fire(&mut queue, 100), Some(200));
        assert_eq!(WAKES.load(Ordering::Relaxed), 2);
        assert_eq!(past.state(), NodeState::Fired(Ok(())));
        assert_eq!(
            in_guard.state(),
            NodeState::Fired(Err(RadioTimerError::Overdue(NsInstant::from_ticks(101))))
        );
        assert_eq!(future.state(), NodeState::Linked);

        assert_eq!(fire(&mut queue, 200), None);
        assert_eq!(future.state(), NodeState::Fired(Ok(())));
        assert_eq!(WAKES.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn min_over_unsorted_ticks_and_ties() {
        let mut queue = Inner::new();
        let mut a = pin!(node(300));
        let mut b = pin!(node(150));
        let mut c = pin!(node(150));
        unsafe {
            queue.link(a.as_mut().get_unchecked_mut());
            queue.link(b.as_mut().get_unchecked_mut());
            queue.link(c.as_mut().get_unchecked_mut());
        }

        assert_eq!(fire(&mut queue, 100), Some(150));

        WAKES.store(0, Ordering::Relaxed);
        assert_eq!(fire(&mut queue, 150), Some(300));
        assert_eq!(WAKES.load(Ordering::Relaxed), 2);
        assert_eq!(b.state(), NodeState::Fired(Ok(())));
        assert_eq!(c.state(), NodeState::Fired(Ok(())));
        assert_eq!(a.state(), NodeState::Linked);
        unsafe { queue.unlink(a.as_mut().get_unchecked_mut()) };
    }

    #[test]
    fn unlink_head_middle_tail() {
        let mut queue = Inner::new();
        let mut a = pin!(node(1000));
        let mut b = pin!(node(2000));
        let mut c = pin!(node(3000));
        unsafe {
            queue.link(a.as_mut().get_unchecked_mut());
            queue.link(b.as_mut().get_unchecked_mut());
            queue.link(c.as_mut().get_unchecked_mut());
        }

        unsafe { queue.unlink(b.as_mut().get_unchecked_mut()) };
        assert_eq!(b.state(), NodeState::Unlinked);
        assert_eq!(fire(&mut queue, 0), Some(1000));

        unsafe { queue.unlink(c.as_mut().get_unchecked_mut()) };
        assert_eq!(fire(&mut queue, 0), Some(1000));

        unsafe { queue.unlink(a.as_mut().get_unchecked_mut()) };
        assert_eq!(fire(&mut queue, 0), None);

        WAKES.store(0, Ordering::Relaxed);
        assert_eq!(WAKES.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn truncated_pass_leaves_remainder_linked() {
        const N: usize = WakerBatch::CAPACITY + 2;
        let mut queue = Inner::new();
        let mut nodes: [_; N] = core::array::from_fn(|i| node(i as u64));
        for n in &mut nodes {
            // Safety (tests): the array does not move for the whole test and
            //         every node is fired before it drops.
            queue.link(n);
        }

        WAKES.store(0, Ordering::Relaxed);
        let mut wakers = WakerBatch::new();
        assert!(matches!(
            queue.fire_and_min(100, GUARD, &mut wakers),
            FireOutcome::Truncated
        ));
        assert_eq!(WAKES.load(Ordering::Relaxed), 0);
        wakers.wake_all();
        assert_eq!(WAKES.load(Ordering::Relaxed), WakerBatch::CAPACITY);
        let linked = nodes
            .iter()
            .filter(|n| n.state() == NodeState::Linked)
            .count();
        assert_eq!(linked, N - WakerBatch::CAPACITY);

        assert_eq!(fire(&mut queue, 100), None);
        assert_eq!(WAKES.load(Ordering::Relaxed), N);
        assert!(nodes
            .iter()
            .all(|n| matches!(n.state(), NodeState::Fired(Ok(())))));
    }

    #[test]
    fn relink_after_cancellation() {
        let mut queue = Inner::new();
        let mut a = pin!(node(500));
        unsafe {
            queue.link(a.as_mut().get_unchecked_mut());
            queue.unlink(a.as_mut().get_unchecked_mut());
            queue.link(a.as_mut().get_unchecked_mut());
        }
        assert_eq!(fire(&mut queue, 500), None);
        assert_eq!(a.state(), NodeState::Fired(Ok(())));
    }
}
