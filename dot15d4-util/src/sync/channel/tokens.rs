use crate::tokens::TokenGuard;

use super::{ConsSlot, MsgSlot};

/// A non-cloneable token representing a message slot allocated for sending a
/// request: Produced by allocating a slot and consumed by sending a request
/// across the channel.  Guarantees bandwidth on the channel for a single
/// request.
#[must_use = "Must be returned to the channel to send a request."]
#[derive(Debug)]
pub struct RequestToken(MsgSlot, TokenGuard);

impl RequestToken {
    /// Creates a new request token.
    pub(super) fn new(msg_slot: MsgSlot) -> Self {
        Self(msg_slot, TokenGuard)
    }

    /// Consume the token.
    pub(super) fn consume(self) -> MsgSlot {
        self.1.consume();
        self.0
    }

    /// Returns an opaque number representing the message slot allocated for the
    /// request.
    ///
    /// This can be used to manage resources based on message slot allocation.
    pub fn message_slot(&self) -> MsgSlot {
        self.0
    }
}

/// A non-cloneable token representing a message slot allocated for receiving a
/// response: Produced by sending a polling request and consumed by receiving
/// the corresponding response across the channel. Guarantees bandwidth on the
/// channel for a single response.
#[must_use = "Must be returned to the channel to send a request."]
#[derive(Debug)]
pub struct PollingResponseToken(MsgSlot, TokenGuard);

impl PollingResponseToken {
    /// Creates a new polling response token.
    pub(super) fn new(msg_slot: MsgSlot) -> Self {
        Self(msg_slot, TokenGuard)
    }

    /// Consume the token.
    pub(super) fn consume(self) -> MsgSlot {
        self.1.consume();
        self.0
    }

    /// Returns an opaque number representing the message slot allocated for the
    /// response.
    ///
    /// This can be used to manage resources based on message slot allocation.
    pub fn message_slot(&self) -> MsgSlot {
        self.0
    }
}

/// A non-cloneable token representing a message slot allocated for receiving a
/// request and returning a response: Produced by receiving a request and consumed
/// by returning a response across the channel. Guarantees bandwidth on the
/// channel for a single response.
#[must_use = "Must be returned to the channel to send a response."]
#[derive(Debug)]
pub struct ResponseToken(MsgSlot, TokenGuard);

impl ResponseToken {
    /// Creates a new response token.
    pub(super) fn new(msg_slot: MsgSlot) -> Self {
        Self(msg_slot, TokenGuard)
    }

    /// Consume the token.
    pub(super) fn consume(self) -> MsgSlot {
        self.1.consume();
        self.0
    }

    /// Returns an opaque number representing the message slot allocated for the
    /// response.
    ///
    /// This can be used to manage resources based on message slot allocation.
    pub fn message_slot(&self) -> MsgSlot {
        self.0
    }
}

/// A non-cloneable token representing an allocated consumer slot: Produced by
/// allocating a consumer slot. Must be presented to receive a request from the
/// channel. Guarantees bandwidth on the channel for a single consumer.
#[must_use = "Must be presented to the channel to access the consumer slot."]
#[derive(Debug)]
pub struct ConsumerToken(ConsSlot, TokenGuard);

impl ConsumerToken {
    /// Creates a new consumer token.
    pub(super) fn new(cons_slot: ConsSlot) -> Self {
        Self(cons_slot, TokenGuard)
    }

    /// Consume the token.
    pub(super) fn consume(self) -> ConsSlot {
        self.1.consume();
        self.0
    }

    /// Returns the consumer slot represented by this token.
    pub(super) fn consumer_slot(&self) -> ConsSlot {
        self.0
    }
}
