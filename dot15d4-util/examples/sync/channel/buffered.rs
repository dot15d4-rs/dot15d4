//! Demonstrates greedily buffering messages into a channel that will be handled
//! in bulk by the receiver while the sender is polling for the list of
//! outstanding responses represented by their corresponding response tokens.

use dot15d4_util::{
    allocator::export::*,
    sync::{
        Channel, HasAddress, MatchingResponse, PollingResponseToken, Receiver, ResponseToken,
        Sender,
    },
};
use embassy_executor::Spawner;
use embassy_time::Timer;
use log::*;

const CHANNEL_CAPACITY: usize = 5;

#[derive(Debug)]
struct Request {
    seq_nr: u8,
}

impl HasAddress<()> for Request {
    fn matches(&self, _: &()) -> bool {
        true
    }
}

struct Response {
    seq_nr: u8,
}

type BufferedChannel = Channel<(), Request, Response, CHANNEL_CAPACITY, 1, 1>;
type BufferedSender = Sender<'static, (), Request, Response, CHANNEL_CAPACITY, 1, 1>;
type BufferedReceiver = Receiver<'static, (), Request, Response, CHANNEL_CAPACITY, 1, 1>;

#[embassy_executor::task]
async fn producer(sender: BufferedSender) {
    let mut seq_nr = 0;
    let mut outstanding_responses = heapless::Vec::<PollingResponseToken, CHANNEL_CAPACITY>::new();

    loop {
        // We use the full capacity of the buffered channel.
        while let Some(request_token) = sender.try_allocate_request_token() {
            let request = Request { seq_nr };
            info!("request: seq_nr {seq_nr}");

            let response_token = sender.send_request_polling_response(request_token, request);
            outstanding_responses.push(response_token).unwrap();

            seq_nr = seq_nr.wrapping_add(1);
        }

        // Poll for the next response.
        let MatchingResponse { response, .. } =
            sender.wait_for_response(&mut outstanding_responses).await;
        info!("response: seq_nr {}", response.seq_nr);
    }
}

#[embassy_executor::task]
async fn consumer(receiver: BufferedReceiver) {
    let mut outstanding_requests =
        heapless::Vec::<(ResponseToken, Request), CHANNEL_CAPACITY>::new();

    loop {
        // Greedily consume requests.
        while let Some(request) = receiver.try_receive_request(&()) {
            outstanding_requests.push(request).unwrap();
        }

        let num_outstanding_requests = outstanding_requests.len();
        if num_outstanding_requests > 0 {
            // Echo an arbitrary request out of order.
            let random_index = rand::random::<u8>() as usize % num_outstanding_requests;
            let (response_token, request) = outstanding_requests.swap_remove(random_index);
            let response = Response {
                seq_nr: request.seq_nr,
            };
            receiver.received(response_token, response);
        }

        // Throttle responses to simulate a slow receiver.
        Timer::after_millis(1000).await;
    }
}

static CHANNEL: StaticCell<BufferedChannel> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .format_timestamp_nanos()
        .init();

    let channel = CHANNEL.init(BufferedChannel::new());
    spawner.spawn(producer(channel.sender())).unwrap();
    spawner.spawn(consumer(channel.receiver())).unwrap();
}
