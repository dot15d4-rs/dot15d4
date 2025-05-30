use dot15d4_util::{
    allocator::{export::*, BufferAllocator, BufferToken},
    buffer_allocator,
    sync::{Channel, HasAddress, Receiver, Sender},
};
use embassy_executor::Spawner;
use embassy_time::Timer;
use log::*;

const NUM_PRODUCERS: usize = 10;
const CHANNEL_CAPACITY: usize = 5;
const BACKLOG: usize = if NUM_PRODUCERS > CHANNEL_CAPACITY {
    NUM_PRODUCERS - CHANNEL_CAPACITY
} else {
    1
};
const NUM_CONSUMERS: usize = NUM_PRODUCERS / 2;

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct Address(u8);

impl Address {
    fn subnet(&self) -> Self {
        Self(self.0 & !1)
    }

    fn as_number(&self) -> u8 {
        self.0
    }
}

const MESSAGE_BUFFER_SIZE: usize = 2;
const NUM_MESSAGE_BUFFERS: usize = NUM_PRODUCERS;

struct Request {
    // A buffer token behaves just like a &'static mut [u8] without the hassle
    // of having to manage lifetimes or generics.
    buffer: BufferToken,
}

// A message may implement structure on top of the backing buffer.
impl Request {
    fn address(&self) -> Address {
        // A buffer token can be used just like a 1-aligned slice of bytes.
        Address(self.buffer[0])
    }

    fn counter(&self) -> u8 {
        self.buffer[1]
    }

    fn increment_counter(&mut self) {
        self.buffer[1] = self.buffer[1].wrapping_add(1)
    }

    // Consume the message to re-use the buffer.
    fn consume(self) -> BufferToken {
        self.buffer
    }
}

// A request needs to expose an address so that it can be routed to the
// corresponding receiver.
impl HasAddress<Address> for Request {
    fn matches(&self, address: &Address) -> bool {
        self.address().subnet() == address.subnet()
    }
}

struct Response {
    buffer: BufferToken,
}

impl Response {
    // The response's buffer can be taken from another message (e.g. the
    // request) without copying and will immediately expose the new message's
    // representation of the message. Possibly at a higher or lower level of
    // abstraction.
    fn from_request(request: Request) -> Response {
        Response {
            buffer: request.consume(),
        }
    }

    fn address(&self) -> Address {
        Address(self.buffer[0])
    }

    fn counter(&self) -> u8 {
        self.buffer[1]
    }

    fn increment_counter(&mut self) {
        self.buffer[1] = self.buffer[1].wrapping_add(1)
    }

    // Consume the message to re-use the buffer.
    fn consume(self) -> BufferToken {
        self.buffer
    }
}

type MpmcChannel = Channel<Address, Request, Response, CHANNEL_CAPACITY, BACKLOG, NUM_CONSUMERS>;
type MpmcSender =
    Sender<'static, Address, Request, Response, CHANNEL_CAPACITY, BACKLOG, NUM_CONSUMERS>;
type MpmcReceiver =
    Receiver<'static, Address, Request, Response, CHANNEL_CAPACITY, BACKLOG, NUM_CONSUMERS>;

#[embassy_executor::task(pool_size = NUM_PRODUCERS)]
async fn producer(
    producer_address: Address,
    buffer_allocator: BufferAllocator,
    sender: MpmcSender,
) {
    // We have a dedicated buffer per producer. The receiver's delivery
    // semantics ensures that it will be safely returned to us for re-use. This
    // allows us to work with a single, zero-copy, zero-allocation buffer.
    let mut buffer = buffer_allocator
        .try_allocate_buffer(MESSAGE_BUFFER_SIZE)
        .expect("out of memory");

    // We're guaranteed to receive a buffer with the exact size requested, so we
    // can initialize it from a slice.
    buffer.copy_from_slice(&[producer_address.0, 0]);

    loop {
        let mut request = Request { buffer };
        request.increment_counter();

        info!(
            "request: address {} value {}",
            producer_address.as_number(),
            request.counter()
        );

        // Transfer buffer ownership to the receiver.
        let response = sender.send_request(request).await;

        // Assert that the response was correctly routed back to the sender it
        // belongs to.
        assert_eq!(response.address(), producer_address);

        info!(
            "response: address {} value {}",
            producer_address.as_number(),
            response.counter()
        );

        // Recover buffer ownership from the response.
        buffer = response.consume();

        Timer::after_millis(100 - producer_address.as_number() as u64).await;
    }
}

#[embassy_executor::task(pool_size = NUM_CONSUMERS)]
async fn consumer(consumer_subnet: Address, receiver: MpmcReceiver) {
    let mut consumer_token = receiver.try_allocate_consumer_token().unwrap();
    loop {
        receiver
            .receive::<Result<(), ()>, _, _>(
                &mut consumer_token,
                &consumer_subnet,
                |request| async {
                    // Long delivery delay to demonstrate backpressure.
                    Timer::after_millis(1000).await;

                    // Check that the request has been correctly routed.
                    assert_eq!(request.address().subnet(), consumer_subnet);

                    // Zero-copy, zero-allocation re-use of buffers to convert
                    // between messages or message representations.
                    let mut response = Response::from_request(request);
                    response.increment_counter();

                    (response, Ok(()))
                },
            )
            .await
            .unwrap();
    }
}

fn mpdu_channel() -> &'static MpmcChannel {
    static CHANNEL: StaticCell<MpmcChannel> = StaticCell::new();
    CHANNEL.init(MpmcChannel::new())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .format_timestamp_nanos()
        .init();

    // The allocator macro hides the implementation details of allocator backend
    // instantiation.
    let buffer_allocator = buffer_allocator!(MESSAGE_BUFFER_SIZE, NUM_MESSAGE_BUFFERS);
    let channel = mpdu_channel();

    // Allocators, senders and receivers can all be safely and efficiently cloned.

    for id in 0..NUM_PRODUCERS {
        spawner
            .spawn(producer(
                Address(id as u8),
                buffer_allocator,
                channel.sender(),
            ))
            .unwrap();
    }
    for id in 0..NUM_CONSUMERS {
        spawner
            .spawn(consumer(Address((id << 1) as u8), channel.receiver()))
            .unwrap();
    }
}
