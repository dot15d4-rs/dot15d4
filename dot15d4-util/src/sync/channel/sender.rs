use core::{
    future::{pending, poll_fn, Future},
    task::{Context, Poll},
};

use bitmaps::{Bits, BitsImpl};

use crate::sync::{CancellationGuard, MsgSlot, PollingResponseToken};

use super::{state::SlotState, Channel, HasAddress, RequestToken};

pub struct MatchingResponse<Response> {
    pub msg_slot: MsgSlot,
    pub response: Response,
}

/// Asynchronous send-only access to a [`Channel`].
pub struct Sender<
    'a,
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
    channel: &'a Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>,
}

impl<
        'a,
        Address: Clone,
        Request: HasAddress<Address>,
        Response,
        const MESSAGES: usize,
        const BACKLOG: usize,
        const CONSUMERS: usize,
    > Sender<'a, Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>
where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    /// Instantiate a new sender for the given channel.
    pub fn new(
        channel: &'a Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>,
    ) -> Self {
        Self { channel }
    }

    /// Waits until a message slot becomes available and blocks it's capacity
    /// for later use by the sender.
    ///
    /// Changes the state of the allocated slot from available to allocated.
    ///
    /// Note: Using this method introduces a risk of deadlock unless you ensure
    ///       that at least one task owning and willing to release a request
    ///       token is life when blocking on this method. See
    ///       [`crate::allocator::AsyncBufferAllocator::allocate_buffer()`] for
    ///       more information and an example.
    pub fn allocate_request_token(
        &self,
    ) -> impl Future<Output = RequestToken>
           + use<'_, 'a, Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS> {
        poll_fn(|cx| self.poll_allocate_request_token(cx))
    }

    /// Poll function that can be used to build futures waiting for request
    /// token availability.
    pub fn poll_allocate_request_token(&self, cx: &mut Context) -> Poll<RequestToken> {
        match self.channel.state().allocate_msg_slot(Some(cx)) {
            Some(msg_slot) => Poll::Ready(RequestToken::new(msg_slot)),
            None => Poll::Pending,
        }
    }

    /// Try to allocate a request token without blocking.
    pub fn try_allocate_request_token(&self) -> Option<RequestToken> {
        self.channel
            .state()
            .allocate_msg_slot(None)
            .map(RequestToken::new)
    }

    /// Release an allocated message slot without sending a message.
    ///
    /// Changes the state of any allocated slot from allocated to available.
    pub fn release_request_token(&self, request_token: RequestToken) {
        let msg_slot = request_token.consume();
        self.channel.state().release_msg_slot(msg_slot);
    }

    /// Synchronously sends the given request over a previously allocated slot
    /// (i.e. makes it "pending" on the receiver side).
    ///
    /// The method returns immediately ("fire and forget"). The request has not
    /// been delivered yet when this method returns. Does not support responses.
    ///
    /// Changes the state of the allocated slot from available to pending.
    ///
    /// Note: This operation cannot fail.
    pub fn send_request_no_response(&self, request_token: RequestToken, request: Request) {
        let msg_slot = request_token.consume();
        let mut state = self.channel.state();
        state.send(msg_slot, request);
        state.slot_state[msg_slot as usize] = SlotState::RequestNoResponse;
    }

    /// Synchronously sends the given request over a previously allocated slot
    /// (i.e. makes it "pending" on the receiver side).
    ///
    /// The method returns immediately but lets the client retain the request
    /// token so that it can await or poll the response. The request has not
    /// been delivered yet when this method returns.
    ///
    /// Changes the state of the allocated slot from available to pending.
    ///
    /// Note: This operation cannot fail.
    pub fn send_request_polling_response(
        &self,
        request_token: RequestToken,
        request: Request,
    ) -> PollingResponseToken {
        let msg_slot = request_token.consume();
        let mut state = self.channel.state();
        state.send(msg_slot, request);
        state.slot_state[msg_slot as usize] = SlotState::RequestPollingResponse(None);
        PollingResponseToken::new(msg_slot)
    }

    /// Asynchronously waits until a response matching any of the given request
    /// tokens becomes available. Removes the matching request token from the
    /// vector, consumes it and produces the corresponding response together
    /// with its message slot id (see [`PollingResponseToken::message_slot()`]).
    /// The latter can be used to correlate the response to a pending request.
    pub async fn wait_for_response<const N: usize>(
        &self,
        response_tokens: &mut heapless::Vec<PollingResponseToken, N>,
    ) -> MatchingResponse<Response> {
        // As a convenience, we poll a cancellable pending-forever future when
        // the response token list is empty.
        if response_tokens.is_empty() {
            pending().await
        }

        // We need a copy of the message slots as the response tokens are &mut
        // and !Copy and therefore cannot be shared between the cancellation
        // guard and poll fn.
        let msg_slots: heapless::Vec<u8, N> = response_tokens
            .iter()
            .map(|response_token| response_token.message_slot())
            .collect();

        let deregister_wakers = || {
            let mut state = self.channel.state();
            for msg_slot in msg_slots.iter() {
                let slot_state = &mut state.slot_state[*msg_slot as usize];
                if let SlotState::RequestPollingResponse(maybe_waker) = slot_state {
                    // Remove any pending wakers.
                    maybe_waker.take();
                }
            }
        };

        // The cancellation guard can only be dropped at an await point at which
        // time we're guaranteed that the wakers have been registered.
        let cancellation_guard = CancellationGuard::new(deregister_wakers);

        let matching_response = poll_fn(|cx| {
            let mut state = self.channel.state();

            let maybe_matching_response = state.try_poll_response(&msg_slots);

            let maybe_registered_waker =
                if let SlotState::RequestPollingResponse(registered_waker) =
                    &state.slot_state[msg_slots[0] as usize]
                {
                    registered_waker.as_ref()
                } else {
                    None
                };

            if let Some((matching_response_token_idx, response)) = maybe_matching_response {
                // We only register wakers if a response isn't pending
                // initially.
                if maybe_registered_waker.is_some() {
                    drop(state);
                    deregister_wakers();
                }

                let matching_response_token =
                    // Safety: The message slot vector has the same number of
                    //         entries as the request tokens vector.
                    unsafe { response_tokens.swap_remove_unchecked(matching_response_token_idx) };

                Poll::Ready(MatchingResponse {
                    msg_slot: matching_response_token.consume(),
                    response,
                })
            } else {
                if let Some(registered_waker) = maybe_registered_waker {
                    debug_assert!({ cx.waker().will_wake(registered_waker) });
                } else {
                    for msg_slot in msg_slots.iter() {
                        let slot_state = &mut state.slot_state[*msg_slot as usize];
                        if let SlotState::RequestPollingResponse(maybe_waker) = slot_state {
                            // Register wakers into pending slots.
                            maybe_waker.get_or_insert(cx.waker().clone());
                        }
                    }
                }

                Poll::Pending
            }
        })
        .await;

        cancellation_guard.inactivate();

        matching_response
    }

    /// Synchronously checks whether a response matching any of the given
    /// request tokens is pending. If so, returns the pending response together
    /// with the corresponding message slot id.
    pub fn try_receive_response<const N: usize>(
        &self,
        response_tokens: &mut heapless::Vec<PollingResponseToken, N>,
    ) -> Option<MatchingResponse<Response>> {
        let msg_slots: heapless::Vec<u8, N> = response_tokens
            .iter()
            .map(|response_token| response_token.message_slot())
            .collect();
        let mut state = self.channel.state();
        let maybe_matching_response = state.try_poll_response(&msg_slots);
        if let Some((matching_msg_slot_idx, response)) = maybe_matching_response {
            let matching_request_token =
                    // Safety: The message slot vector has the same number of
                    //         entries as the request tokens vector.
                    unsafe { response_tokens.swap_remove_unchecked(matching_msg_slot_idx) };
            Some(MatchingResponse {
                msg_slot: matching_request_token.consume(),
                response,
            })
        } else {
            None
        }
    }

    /// Sends the given request and then waits until it has been delivered and
    /// the message slot was released.
    ///
    /// Changes the state of the allocated message slot from available to
    /// pending and waits until it is being released (i.e. becomes available
    /// again).
    ///
    /// Panics when cancelled. If you need cancel-safety on the sender side
    /// consider polling the response instead.
    ///
    /// Note: As we hand over ownership of requests on delivery, their content
    ///       (including any references to buffers or other dependent resources)
    ///       may leak beyond the time of return of this method. You may not
    ///       assume that resources assigned to the request can be re-used
    ///       unless you implement drop handlers that allow you to prove that
    ///       dependent resources blocked by the message have actually been
    ///       released.
    pub async fn send_request_awaiting_response(
        &self,
        request_token: RequestToken,
        request: Request,
    ) -> Response {
        let msg_slot = request_token.consume();
        self.channel.state().send(msg_slot, request);

        let cancellation_guard = CancellationGuard::new(|| panic!("cancelled"));

        let response = poll_fn(move |cx| {
            let mut state = self.channel.state();
            if let Some(response) = state.try_get_response(msg_slot) {
                Poll::Ready(response)
            } else {
                state.slot_state[msg_slot as usize] =
                    SlotState::RequestAwaitingResponse(cx.waker().clone());
                Poll::Pending
            }
        })
        .await;

        cancellation_guard.inactivate();

        response
    }

    /// Convenience method that allocates a message slot, sends the given
    /// message as soon as a slot becomes available and then waits until the
    /// message has been delivered.
    pub async fn send_request(&self, request: Request) -> Response {
        let request_token = self.allocate_request_token().await;
        self.send_request_awaiting_response(request_token, request)
            .await
    }
}
