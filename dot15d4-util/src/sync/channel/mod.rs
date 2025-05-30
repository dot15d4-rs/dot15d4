//! A bounded channel for sending requests from multiple asynchronous tasks to
//! one or many receiving task(s) with backpressure.
//!
//! The channel is not synchronized across threads. It can be used concurrently
//! by multiple producer (sender) and consumer (receiver) tasks as long as all
//! are managed by a single executor (thread).
//!
//! The channel has a limit on the number of messages that it can store. If this
//! limit is reached, trying to send another message will exert backpressure on
//! the sender.
//!
//! The channel exposes a sender and a receiver API.
//!
//! 1. Sender API:
//!
//!   - [`Sender::allocate_request_token()`]
//!     Waits until a request token becomes available and returns it.
//!
//!   - [`Sender::poll_allocate_request_token()`]
//!     Poll function that can be used to build futures waiting for request
//!     token availability.
//!
//!   - [`Sender::try_allocate_request_token()`]
//!     Try to allocate a request token without blocking.
//!
//!   - [`Sender::release_request_token()`]
//!     Releases an allocated request token without sending a request.
//!
//!   - [`Sender::send_request_no_response()`]
//!     Consumes a request token to send the given request. Returns
//!     synchronously without waiting for delivery. Any response produced by the
//!     receiver will be dropped.
//!
//!   - [`Sender::send_request_polling_response()`]
//!     Sends the given request without consuming the request token. Returns
//!     synchronously without waiting for delivery. The response produced by the
//!     receiver must actively awaited or polled, see
//!     [`Sender::wait_for_response()`], [`Sender::poll_wait_for_response()`]
//!     and [`Sender::try_receive_response()`].
//!
//!   - [`Sender::wait_for_response()`]
//!     Asynchronously waits until a response matching any of the given request
//!     tokens becomes available. Removes the matching request token from the
//!     vector, consumes it and produces the corresponding response together
//!     with its message slot id (see [`RequestToken::message_slot()`]). The
//!     latter can be used to correlate the response to a pending request.
//!
//!   - [`Sender::try_receive_response()`]
//!     Synchronously checks whether a response matching any of the given
//!     request tokens is pending. If so, returns the pending response together
//!     with the corresponding message slot id.
//!
//!   - [`Sender::send_request_awaiting_response()`]
//!     Consumes a request token to send the given request and waits until it
//!     was delivered to the receiver. Returns the response produced by the
//!     receiver.
//!
//!   - [`Sender::send_request()`]
//!     A convenience method over [`Sender::allocate_request_token()`] and
//!     [`Sender::send_request_awaiting_response()`]
//!
//!   This API targets asynchronous producers, provides feedback about delivery,
//!   supports a request/response model as required by the IEEE 802.15.4
//!   standard and facilitates safe resource management.
//!
//! 2. Receiver API:
//!   - [`Receiver::try_allocate_consumer_token()`]
//!     Tries to allocate a consumer token that allows a single consumer to
//!     listen for requests.
//!
//!   - [`Receiver::release_consumer_token()`]
//!     Releases a previously
//!
//!   - [`Receiver::wait_for_request()`]
//!     Asynchronously waits until a matching request becomes pending and
//!     returns it. Also returns the token required to signal delivery to the
//!     sender.
//!
//!   - [`Receiver::poll_wait_for_request()`]
//!     Poll function that can be used to build futures waiting for pending
//!     requests. The poll result will return the request and a response token.
//!
//!   - [`Receiver::try_receive_request()`]
//!     Synchronously checks whether a matching request is pending. If so,
//!     returns the pending request together with the token required to signal
//!     delivery to the sender.
//!
//!   - [`Receiver::received()`]
//!     Releases the message slot and signals delivery to the sender.
//!
//!   - [`Receiver::receive()`]
//!     This is convenience method over [`Receiver::wait_for_request()`] and
//!     [`Receiver::received()`]. It handles message reception in a closure.

// TODO: Make a prioritized version of the channel for timestamped radio tasks
//       that allows awaiting changes to the queue front.

use bitmaps::{Bits, BitsImpl};
pub use receiver::Receiver;
pub use sender::*;
pub use tokens::*;

mod receiver;
mod sender;
mod state;
mod tokens;
mod util;

use core::cell::{RefCell, RefMut};

use self::state::State;

/// A trait to be implemented by requests. This trait enables routing of
/// requests to appropriate selectable receivers.
///
/// Receivers will be registered with a receiver address. A request will be
/// directed to the first receiver whose address matches the request address.
pub trait HasAddress<Address> {
    /// Checks whether the given address matches the request.
    fn matches(&self, address: &Address) -> bool;
}

/// Opaque type representing a message slot.
///
/// Note: We use a plain number rather than a newtype so that the slot id can be
///       used to index resources.
pub type MsgSlot = u8;

/// Opaque type representing a consumer slot.
pub type ConsSlot = u8;

/// An asynchronous bounded MPMC queue for sending requests from multiple
/// asynchronous producer tasks to selectable receiving tasks with backpressure.
///
/// The channel will buffer requests up to the guaranteed capacity and will then
/// be able to backlog a limited number of further producer tasks while they are
/// waiting for a message slot to become available. Trying to schedule waiting
/// producer tasks beyond the capacity of the backlog will cause the queue to
/// panic.
///
/// More specifically: Given `PRODUCERS` as the number of independent tasks that
/// are accessing the queue in parallel and `MESSAGES` as the number of messages
/// that can be handled concurrently, the `BACKLOG` parameter needs to be set to
/// `PRODUCERS - MESSAGES` for panic-free queue operation.
///
/// Requests will be delivered to the receiver in the same order as they were
/// sent.
pub struct Channel<
    Address: Clone,
    Request: HasAddress<Address>,
    Response,
    const MESSAGES: usize,
    const BACKLOG: usize,
    const CONSUMERS: usize,
> where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    state: RefCell<State<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>>,
}

impl<
        Address: Clone,
        Request: HasAddress<Address>,
        Response,
        const MESSAGES: usize,
        const BACKLOG: usize,
        const CONSUMERS: usize,
    > Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>
where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    /// Initialize a new [`Channel`].
    pub fn new() -> Self {
        Self {
            state: RefCell::new(State::new()),
        }
    }

    /// Returns an additional [`Sender`] attached to the channel.
    pub fn sender(&self) -> Sender<'_, Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS> {
        Sender::new(self)
    }

    /// Returns an additional [`Receiver`] attached to the channel.
    pub fn receiver(
        &self,
    ) -> Receiver<'_, Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS> {
        Receiver::new(self)
    }

    fn state(&self) -> RefMut<'_, State<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>> {
        self.state.borrow_mut()
    }
}

impl<
        Address: Clone,
        Request: HasAddress<Address>,
        Response,
        const MESSAGES: usize,
        const BACKLOG: usize,
        const CONSUMERS: usize,
    > Default for Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>
where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    fn default() -> Self {
        Self::new()
    }
}
