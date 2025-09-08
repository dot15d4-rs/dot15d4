use core::{
    cell::Ref,
    future::{poll_fn, Future},
    marker::PhantomData,
    mem,
    task::{Context, Poll},
};

use bitmaps::{Bits, BitsImpl};

use crate::sync::{channel::State, CancellationGuard, MsgSlot};

use super::{
    state::{Message, SlotState},
    Channel, ConsumerToken, HasAddress, ResponseToken,
};

/// Consumer-side access to a channel.
pub struct Receiver<
    'channel,
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
    channel: &'channel Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>,
    address: PhantomData<Address>,
    request: PhantomData<Request>,
    response: PhantomData<Response>,
}

impl<
        'a,
        Address: Clone,
        Request: HasAddress<Address>,
        Response,
        const MESSAGES: usize,
        const BACKLOG: usize,
        const CONSUMERS: usize,
    > Receiver<'a, Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>
where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    /// Instantiate a new receiver for the given channel.
    pub fn new(
        channel: &'a Channel<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>,
    ) -> Self {
        Self {
            channel,
            address: PhantomData,
            request: PhantomData,
            response: PhantomData,
        }
    }

    /// Try to allocate a consumer token.
    ///
    /// Consumers typically acquire a consumer token once when the consumer task
    /// starts and then present it on any reception.
    ///
    /// A consumer may allocate more than one consumer token to be able to
    /// receive requests in parallel.
    pub fn try_allocate_consumer_token(&self) -> Option<ConsumerToken> {
        self.channel
            .state_mut()
            .allocate_cons_slot()
            .map(ConsumerToken::new)
    }

    /// Release a consumer token.
    ///
    /// Consumers typically call this method just before the consumer task ends.
    ///
    /// Calling this method will release a previously allocated consumer slot
    /// back to the pool.
    pub fn release_consumer_token(&self, consumer_token: ConsumerToken) {
        self.channel
            .state_mut()
            .release_cons_slot(consumer_token.consume());
    }

    /// Wait for a request matching the given address or address wildcard and
    /// return it together with a token representing the message slot allocated
    /// for the response.
    ///
    /// Reception occurs in the order that requests became pending.
    ///
    /// If several receivers match a pending request, only a single, arbitrary
    /// one will receive the request.
    ///
    /// Note: Other than on the producer side (which may have to wait for a
    ///       radio slot to become available) we do not allow for backpressure
    ///       on the consumer side as we currently assume that local consumers
    ///       should be fast enough to handle reception synchronously. Therefore
    ///       it is required to present an allocated consumer token to get
    ///       access to the channel.
    ///
    /// On return, changes the state of the matching message slot from pending
    /// to receiving.
    ///
    /// Canceling reception will free the consumer slot.
    ///
    /// You SHOULD NOT try to receive and peek at the same time as this will
    /// result in non-deterministic behavior.
    pub async fn receive_request_async(
        &self,
        consumer_token: &mut ConsumerToken,
        address: &Address,
    ) -> (ResponseToken, Request) {
        let consumer_slot = consumer_token.consumer_slot();
        let cancellation_guard = CancellationGuard::new(|| {
            // Clean up the consumer slot.
            self.channel.state_mut().consumers[consumer_slot as usize] = None;
        });

        let result = poll_fn(|cx| self.poll_receive_request(cx, consumer_token, address)).await;
        cancellation_guard.inactivate();

        result
    }

    /// Poll function that can be used to build futures waiting for pending
    /// requests. The poll result will return the request and a response token.
    ///
    /// You SHOULD NOT try to receive and peek at the same time as this will
    /// result in non-deterministic behavior.
    pub fn poll_receive_request(
        &self,
        cx: &mut Context,
        consumer_token: &mut ConsumerToken,
        address: &Address,
    ) -> Poll<(ResponseToken, Request)> {
        let consumer_slot = consumer_token.consumer_slot();

        let mut state = self.channel.state_mut();
        match state.try_receive(address) {
            Some((msg_slot, request)) => Poll::Ready((ResponseToken::new(msg_slot), request)),
            None => {
                // None of the pending messages fits the given address, so let's
                // wait for one that fits.
                let consumer = &mut state.consumers[consumer_slot as usize];
                match consumer {
                    Some((ex_address, ex_waker)) => {
                        ex_waker.clone_from(cx.waker());
                        *ex_address = address.clone();
                    }
                    None => {
                        *consumer = Some((address.clone(), cx.waker().clone()));
                    }
                }
                Poll::Pending
            }
        }
    }

    /// Try to receive a pending request matching the given address or address
    /// wildcard and return it together with a token representing the message
    /// slot allocated for the response.
    ///
    /// The same ordering, filtering and backpressure rules apply as for
    /// [`Self::receive_request_async()`], see there.
    ///
    /// Changes the state of any matching message slot from pending to
    /// receiving.
    ///
    /// Note: This call does not require a consumer token as it does not block
    ///       any consumer-related resources.
    pub fn try_receive_request(&self, address: &Address) -> Option<(ResponseToken, Request)> {
        self.channel
            .state_mut()
            .try_receive(address)
            .map(|(msg_slot, request)| (ResponseToken::new(msg_slot), request))
    }

    /// Releases the message slot for re-use and signals delivery to
    /// asynchronous senders returning a response.
    ///
    /// Defining the notion of "delivery" is up to the implementor. It can
    /// signal delivery of a message over the air, a promise that certain
    /// resources have been released, reception of a response by a peer, etc.
    ///
    /// Changes the state of the allocated message slot from receiving to
    /// released (available).
    ///
    /// Note: Not calling this method will block the slot forever.
    pub fn received(&self, response_token: ResponseToken, response: Response) {
        let msg_slot = response_token.consume();

        // Signal to the sending task, that a the message in this slot has been
        // fully received.
        let mut state = self.channel.state_mut();
        debug_assert!(state.messages[msg_slot as usize].is_none());

        let current_slot_state = &mut state.slot_state[msg_slot as usize];

        let next_slot_state = match current_slot_state {
            SlotState::RequestAwaitingResponse(_) | SlotState::RequestPollingResponse(_) => {
                // A sender is waiting or will be polling for the response. The
                // response needs to be delivered.
                SlotState::ResponseAvailable
            }
            SlotState::RequestNoResponse => {
                // This is the "fire-and-forget" case. The response will be
                // dropped and the message slot released right away.
                SlotState::Unused
            }
            // Safety: The sent state must have been reached.
            _ => unreachable!(),
        };

        let previous_slot_state = mem::replace(current_slot_state, next_slot_state);

        let pend_response = match previous_slot_state {
            SlotState::RequestAwaitingResponse(waker) => {
                // A sender is asynchronously waiting for the response. The sending
                // task can now be woken up.
                waker.wake();
                // As the waker correlates the response to the request in the
                // same task, there's no need to pend the response for polling.
                false
            }
            // The response must be pended, so that it can be found by polling
            // senders
            SlotState::RequestPollingResponse(maybe_waker) => {
                // If a sender is already polling for the response it can now be
                // woken up.
                if let Some(waker) = maybe_waker {
                    waker.wake();
                }
                // A waker may not yet be available and if it is then it does
                // not necessarily uniquely correlate the response to a request.
                // Therefore the response must be made pending.
                true
            }
            _ => false,
        };

        if matches!(current_slot_state, SlotState::ResponseAvailable) {
            state.messages[msg_slot as usize] = Some(Message::Response(response));
            if pend_response {
                // Safety: The queue has dedicated capacity for all slots.
                state.pending_responses.push_back(msg_slot).unwrap();
            }
        } else {
            // This is the "fire-and-forget" case. The response will be
            // dropped and the message slot released right away.
            state.release_msg_slot(msg_slot);
        }
    }

    /// Convenience method over `receive_request_async()` and `received()`.
    pub async fn receive<
        Result,
        Fut: Future<Output = (Response, Result)>,
        F: FnOnce(Request) -> Fut,
    >(
        &self,
        consumer_token: &mut ConsumerToken,
        address: &Address,
        f: F,
    ) -> Result {
        let (response_token, request) = self.receive_request_async(consumer_token, address).await;
        let (response, result) = f(request).await;
        self.received(response_token, response);
        result
    }

    /// Wait for a request matching the given address or address wildcard and
    /// return a copy of it.
    ///
    /// Peeking accesses the matching request that has been pending for the
    /// longest time already.
    ///
    /// Does not change the state of the channel.
    ///
    /// Canceling will free the consumer slot.
    ///
    /// You SHOULD NOT try to receive and peek at the same time as this will
    /// result in non-deterministic behavior.
    pub async fn peek_request_async(
        &self,
        consumer_token: &mut ConsumerToken,
        address: &Address,
    ) -> Ref<'_, Request> {
        let consumer_slot = consumer_token.consumer_slot();
        let cancellation_guard = CancellationGuard::new(|| {
            // Clean up the consumer slot.
            self.channel.state_mut().consumers[consumer_slot as usize] = None;
        });

        let result = poll_fn(move |cx| self.poll_peek_request(cx, consumer_token, address)).await;
        cancellation_guard.inactivate();

        result
    }

    /// Poll function that can be used to build futures waiting for pending
    /// requests. The poll result will return a reference to the request without
    /// dequeuing it from the list of pending requests.
    ///
    /// You SHOULD NOT try to receive and peek at the same time as this will
    /// result in non-deterministic behavior.
    pub fn poll_peek_request(
        &self,
        cx: &mut Context,
        consumer_token: &mut ConsumerToken,
        address: &Address,
    ) -> Poll<Ref<'_, Request>> {
        let consumer_slot = consumer_token.consumer_slot();

        match Self::try_peek(self.channel.state(), address) {
            Some((_, request)) => Poll::Ready(request),
            None => {
                // None of the pending messages fits the given address, so let's
                // wait for one that fits.
                let consumer = &mut self.channel.state_mut().consumers[consumer_slot as usize];
                match consumer {
                    Some((ex_address, ex_waker)) => {
                        ex_waker.clone_from(cx.waker());
                        *ex_address = address.clone();
                    }
                    None => {
                        *consumer = Some((address.clone(), cx.waker().clone()));
                    }
                }

                Poll::Pending
            }
        }
    }

    /// Try to find a pending request that matches the given address. If found,
    /// it will be cloned and returned.
    fn try_peek<'request>(
        state: Ref<'request, State<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>>,
        address: &Address,
    ) -> Option<(MsgSlot, Ref<'request, Request>)> {
        for &msg_slot in state.pending_requests.iter() {
            if let Some(Message::Request(request)) = state.messages[msg_slot as usize].as_ref() {
                // Check whether this consumer listens for the pending request.
                if request.matches(address) {
                    let borrowed_request = Ref::map(state, |state| {
                        // Safety: A slot can never be allocated and pending at the
                        //         same time. We're guaranteed exclusive access to
                        //         the slot right now and may read it. A pending
                        //         slot must have been allocated and set.
                        let request = state.messages[msg_slot as usize].as_ref().unwrap();
                        if let Message::Request(request) = request {
                            request
                        } else {
                            unreachable!()
                        }
                    });

                    return Some((msg_slot, borrowed_request));
                }
            } else {
                // Safety: We know that the request is pending, so a
                //         corresponding message must be available.
                unreachable!()
            }
        }

        None
    }
}
