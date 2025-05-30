use core::{
    array::from_fn,
    task::{Context, Waker},
};

use bitmaps::{Bitmap, Bits, BitsImpl};
use heapless::Deque;

use crate::sync::ConsSlot;

use super::{util::DequeWrapper, HasAddress, MsgSlot};

/// Wrapper to store both, requests and responses, within the same message slot.
pub(super) enum Message<Request, Response> {
    /// A message representing a request.
    Request(Request),

    /// A message representing a response.
    Response(Response),
}

// TODO: Consider removing support for async senders to reduce the enum's size.
pub(super) enum SlotState {
    Unused,
    // The waker wakes the sending task, that asynchronously awaits the
    // response.
    RequestAwaitingResponse(Waker),
    // The waker is inserted while a sender is awaiting the response.
    RequestPollingResponse(Option<Waker>),
    RequestNoResponse,
    ResponseAvailable,
}

/// MESSAGES defines the number of available slots for messages that may be sent
/// concurrently over the channel.
///
/// BACKLOG is the number of producers that may wait for a slot to become
/// available.
///
/// CONSUMERS defines the number of available slots for consumers that may be
/// concurrently listening for requests on the channel.
///
/// The lifecycle of an individual message slot:
/// - available
/// - allocated
/// - pending
/// - receiving
/// - released (i.e. available again)
pub(super) struct State<
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
    /// Pre-allocated message slots. Messages are expected to be small.
    ///
    /// Large resources linked to messages SHALL be kept in separately allocated
    /// buffers.
    pub(super) messages: [Option<Message<Request, Response>>; MESSAGES],

    /// Contains the list of slots with pending requests in the order they
    /// became pending.
    ///
    /// Safety: This is a non-synchronized queue.
    ///
    /// TODO: The synchronized [`heapless::spsc::Queue`] cannot be used. It is
    ///       only implemented for two archs plus we need MPMC or at least MPSC.
    ///       But more importantly its capacity is N-1 which doesn't match the
    ///       required const generic arguments.
    pub(super) pending_requests: Deque<MsgSlot, MESSAGES>,

    /// Contains the list of slots with pending responses in the order they
    /// became pending.
    ///
    /// Safety: This is a non-synchronized queue.
    ///
    /// TODO: This queue is only required when polling for responses, so we may
    ///       make it an optional feature.
    pub(super) pending_responses: Deque<MsgSlot, MESSAGES>,

    /// A bitmap that manages message slots: 0 - in use, 1 - available
    ///
    /// Note: The same information could be retrieved from the slot_state field
    ///       below. But using a bitmap is faster and weighs little in terms of
    ///       memory footprint.
    msg_available: Bitmap<MESSAGES>,

    /// A bitmap that manages consumer slots: 0 - in use, 1 - available
    ///
    /// Note: The same information could be retrieved from the consumers field
    ///       below.
    ///
    /// TODO: Consider removing this field if it turns out that consumer
    ///       allocation is slow and rare (e.g. just on program initialization).
    cons_available: Bitmap<CONSUMERS>,

    /// Represents the state of the corresponding message slot. Also stores
    /// wakers in case the response is being awaited asynchronously.
    pub(super) slot_state: [SlotState; MESSAGES],

    /// When a request becomes pending, the first consumer selectable by the
    /// requests's address will be woken and receives the request.
    ///
    /// The consumer address may be a wildcard address matching all requests or
    /// a well-defined selection. The messages [`HasAddress`] implementation
    /// must be able to handle the address space.
    ///
    /// Note: Currently we assume that the consumer list is short. Therefore
    ///       iteratively O(n)-searching for matching consumers is less resource
    ///       intensive than using a hash map, binary search or similar.
    ///
    /// Note: Requests are not cloneable in general. As we hand out ownership of
    ///       requests to consumers, only a single consumer should currently be
    ///       matching per request. This might change in the future: We may
    ///       introduce an order to consumers and re-define the consumer list as
    ///       a "filter onion" such that each consumer may individually decide
    ///       whether they consume a request (OK + Response), drop it (DROP) or
    ///       pass it on to consumers further down the chain without consuming
    ///       it themselves (CONTINUE).
    pub(super) consumers: [Option<(Address, Waker)>; CONSUMERS],

    /// Contains the list of sender tasks waiting for a message slot in the
    /// order they started waiting.
    ///
    /// The tasks are woken in the order they started waiting as soon as a slot
    /// becomes available.
    ///
    /// TODO: Consider using a deque implementation that supports zero size or
    ///       make this field optional via feature flag.
    backlog: Deque<Waker, BACKLOG>,
}

/// Safety: None of the methods are idempotent. They must not be called from
///         call-sites prone to spurious wake-ups (e.g. the pending branch of a
///         poll function).
impl<
        Address: Clone,
        Request: HasAddress<Address>,
        Response,
        const MESSAGES: usize,
        const BACKLOG: usize,
        const CONSUMERS: usize,
    > State<Address, Request, Response, MESSAGES, BACKLOG, CONSUMERS>
where
    BitsImpl<MESSAGES>: Bits,
    BitsImpl<CONSUMERS>: Bits,
{
    pub(super) fn new() -> Self {
        assert!(MESSAGES <= MsgSlot::MAX as _, "messages > 256");
        Self {
            messages: from_fn(|_| None),
            pending_requests: Deque::new(),
            pending_responses: Deque::new(),
            msg_available: Bitmap::mask(MESSAGES),
            cons_available: Bitmap::mask(CONSUMERS),
            slot_state: from_fn(|_| SlotState::Unused),
            consumers: from_fn(|_| None),
            backlog: Deque::new(),
        }
    }

    /// Guarantees exclusive access to the returned message slot in the channel.
    pub(super) fn allocate_msg_slot(&mut self, cx: Option<&mut Context>) -> Option<MsgSlot> {
        match self.msg_available.first_index() {
            None => {
                if let Some(cx) = cx {
                    self.backlog
                        .push_front(cx.waker().clone())
                        .expect("backlog full");
                }
                None
            }
            Some(msg_slot) => {
                debug_assert!(matches!(self.slot_state[msg_slot], SlotState::Unused));
                self.msg_available.set(msg_slot, false);
                Some(msg_slot as MsgSlot)
            }
        }
    }

    /// Return a message slot to the list of available slots.
    ///
    /// Safety: The slot must still be allocated (= not available) but no longer
    ///         pending.
    pub(super) fn release_msg_slot(&mut self, msg_slot: MsgSlot) {
        let was_available = self.msg_available.set(msg_slot as _, true);
        debug_assert!(!was_available);
        debug_assert!(matches!(
            self.slot_state[msg_slot as usize],
            SlotState::Unused
        ));
        // Signal to the next task waiting for slots (if any), that a slot is
        // now available.
        if let Some(waker) = self.backlog.pop_back() {
            waker.wake()
        }
    }

    /// Guarantees exclusive access to the returned consumer slot in the
    /// channel.
    pub(super) fn allocate_cons_slot(&mut self) -> Option<ConsSlot> {
        match self.cons_available.first_index() {
            None => None,
            Some(const_slot) => {
                self.cons_available.set(const_slot, false);
                Some(const_slot as ConsSlot)
            }
        }
    }

    /// Return a consumer slot to the list of available slots.
    ///
    /// Safety: The slot must still be allocated.
    pub(super) fn release_cons_slot(&mut self, cons_slot: ConsSlot) {
        let was_available = self.cons_available.set(cons_slot as _, true);
        debug_assert!(!was_available);
    }

    /// Store the request, mark the slot as pending and notify the first
    /// matching consumer (if any) that a request is pending.
    pub(super) fn send(&mut self, msg_slot: MsgSlot, request: Request) {
        // Safety: Slot must be reserved (= not available) but not yet pending.
        debug_assert!(!self.is_msg_slot_available(msg_slot));
        debug_assert!(self.messages[msg_slot as usize].is_none());

        // Wake the first matching consumer (if any).
        for consumer in &mut self.consumers {
            if let Some((consumer_address, _)) = consumer {
                if request.matches(consumer_address) {
                    if let Some((_, consumer_waker)) = consumer.take() {
                        consumer_waker.wake()
                    }
                    break;
                }
            }
        }

        // Safety: A slot can never be allocated and pending at the same time.
        //         We're guaranteed exclusive access to the slot right now and
        //         may write to it. An available slot must be empty.
        self.messages[msg_slot as usize] = Some(Message::Request(request));

        // Safety: The queue has dedicated capacity for all slots.
        self.pending_requests.push_back(msg_slot).unwrap();
    }

    /// Try to find a pending request that matches the given address. If found,
    /// it will be removed from the list of pending requests and returned.
    pub(super) fn try_receive(&mut self, address: &Address) -> Option<(MsgSlot, Request)> {
        for (pending_request_index, &msg_slot) in self.pending_requests.iter().enumerate() {
            if let Some(Message::Request(request)) = &self.messages[msg_slot as usize] {
                // Check whether this consumer listens for the pending request.
                if request.matches(address) {
                    // Safety: A slot can never be allocated and pending at the
                    //         same time. We're guaranteed exclusive access to
                    //         the slot right now and may write to it. A pending
                    //         slot must have been allocated and set.
                    if let Message::Request(request) =
                        self.messages[msg_slot as usize].take().unwrap()
                    {
                        // Remove the pending request from the list.
                        DequeWrapper::new(&mut self.pending_requests).remove(pending_request_index);

                        return Some((msg_slot, request));
                    } else {
                        unreachable!()
                    }
                }
            } else {
                // Safety: We know that the request is pending, so a
                //         corresponding message must be available.
                unreachable!()
            }
        }

        None
    }

    /// Check whether a pending response matches any of the given message slots.
    /// If that is the case, it will be removed from the list of pending
    /// responses, the corresponding message slot released and the index of the
    /// matching message slot and the response returned.
    ///
    /// Note: This SHALL only be called by polling clients.
    pub(super) fn try_poll_response<const N: usize>(
        &mut self,
        msg_slots: &heapless::Vec<u8, N>,
    ) -> Option<(usize, Response)> {
        for (pending_response_index, &response_msg_slot) in
            self.pending_responses.iter().enumerate()
        {
            let (matching_msg_slot_idx, matching_msg_slot) =
                if let Some((matching_msg_slot_idx, matching_msg_slot)) = msg_slots
                    .iter()
                    .copied()
                    .enumerate()
                    .find(|(_, request_msg_slot)| response_msg_slot == *request_msg_slot)
                {
                    (matching_msg_slot_idx, matching_msg_slot)
                } else {
                    continue;
                };

            let slot_state = &mut self.slot_state[matching_msg_slot as usize];
            if let SlotState::ResponseAvailable = slot_state {
                *slot_state = SlotState::Unused;
                self.release_msg_slot(matching_msg_slot);

                let response = if let Message::Response(response) =
                    // Safety: The slot state indicates that a response is
                    //         present in the allocated message slot.
                    self.messages[matching_msg_slot as usize].take().unwrap()
                {
                    // Remove the pending response from the list.
                    DequeWrapper::new(&mut self.pending_responses).remove(pending_response_index);

                    response
                } else {
                    unreachable!()
                };

                return Some((matching_msg_slot_idx, response));
            } else {
                // Safety: The message slot is pending, so it must contain a
                //         response.
                unreachable!()
            }
        }

        None
    }

    /// Try to get a response from a specific message slot. This bypasses the
    /// list of pending responses which is not used for clients awaiting a
    /// response on a specific message slot.
    ///
    /// Note: This SHALL only be called by waiting clients.
    pub(super) fn try_get_response(&mut self, msg_slot: u8) -> Option<Response> {
        let slot_state = &mut self.slot_state[msg_slot as usize];

        if let SlotState::ResponseAvailable = slot_state {
            *slot_state = SlotState::Unused;
            self.release_msg_slot(msg_slot);

            if let Message::Response(response) =
                // Safety: The slot state indicates that a response is
                //         present in the allocated message slot.
                self.messages[msg_slot as usize].take().unwrap()
            {
                return Some(response);
            } else {
                unreachable!()
            }
        }

        None
    }

    /// Checks whether the message slot with the given slot id is available.
    fn is_msg_slot_available(&mut self, msg_slot: MsgSlot) -> bool {
        self.msg_available.get(msg_slot as _)
    }
}
