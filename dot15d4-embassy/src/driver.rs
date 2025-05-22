use core::{cell::RefCell, marker::PhantomData, mem, num::NonZero, task::Context, task::Poll};

use dot15d4::{
    driver::{
        const_config::MAC_PAN_ID,
        frame::{RadioFrame, RadioFrameRepr, RadioFrameSized, RadioFrameUnsized},
        DriverConfig,
    },
    mac::{
        frame::mpdu::MpduFrame,
        primitives::{DataRequest, MacIndication, MacRequest},
        MacBufferAllocator, MacIndicationReceiver, MacRequestSender,
    },
    util::{
        allocator::IntoBuffer,
        frame::Frame,
        sync::{ConsumerToken, RequestToken, ResponseToken},
    },
};
use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};

/// Driver for the `dot15d4` side
pub struct Ieee802154Driver<'driver, RadioDriverImpl: DriverConfig> {
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'driver>,
    indication_receiver: MacIndicationReceiver<'driver>,
    consumer_token: RefCell<ConsumerToken>,
    hardware_addr: HardwareAddress,
    driver: PhantomData<RadioDriverImpl>,
}

impl<'driver, RadioDriverImpl: DriverConfig> Ieee802154Driver<'driver, RadioDriverImpl> {
    const RADIO_FRAME_REPR: RadioFrameRepr<RadioDriverImpl, RadioFrameUnsized> =
        RadioFrameRepr::<RadioDriverImpl, RadioFrameUnsized>::new();

    // Note: This API forces us to allocate the max buffer size. The actual
    //       payload length will only be known when consuming the token. The Tx
    //       token API is sync and must be infallible, so we can't wait for a
    //       buffer to become available or bail during token consumption.
    const BUFFER_LENGTH: usize = Self::RADIO_FRAME_REPR.max_buffer_length() as usize;

    pub fn new(
        buffer_allocator: MacBufferAllocator,
        request_sender: MacRequestSender<'driver>,
        indication_receiver: MacIndicationReceiver<'driver>,
        hardware_addr: HardwareAddress,
    ) -> Self {
        #[cfg(feature = "rtos-trace")]
        crate::trace::instrument();

        let consumer_token = indication_receiver
            .try_allocate_consumer_token()
            .expect("consumer slot");
        Self {
            buffer_allocator,
            request_sender,
            indication_receiver,
            consumer_token: RefCell::new(consumer_token),
            hardware_addr,
            driver: PhantomData,
        }
    }

    fn rx_token(&self, cx: &mut Context) -> Option<RxToken<'_>> {
        let (response_token, indication) = match self.indication_receiver.poll_wait_for_request(
            cx,
            &mut self.consumer_token.borrow_mut(),
            &(),
        ) {
            Poll::Ready(request) => request,
            Poll::Pending => return None,
        };

        let mpdu = match indication {
            MacIndication::McpsData(data_indication) => data_indication.mpdu,
            _ => unreachable!(),
        };
        Some(RxToken {
            indication_receiver: &self.indication_receiver,
            radio_frame: mpdu.into_radio_frame::<RadioDriverImpl>(),
            response_token,
            buffer_allocator: &self.buffer_allocator,
        })
    }

    fn tx_token(&self, cx: &mut Context) -> Option<TxToken<'_>> {
        // Safety: Always allocate the buffer before trying to allocate a
        //         request token to avoid deadlock (or livelock in this case).
        let buffer = self
            .buffer_allocator
            .try_allocate_buffer(Self::BUFFER_LENGTH)
            .expect("no capacity");

        let request_token = match self.request_sender.poll_allocate_request_token(cx) {
            Poll::Ready(request_token) => request_token,
            Poll::Pending => {
                // Safety: The buffer was allocated from the same allocator it
                //         is now de-allocated from.
                unsafe {
                    self.buffer_allocator.deallocate_buffer(buffer);
                }
                return None;
            }
        };

        let radio_frame = RadioFrame::<RadioFrameUnsized>::new::<RadioDriverImpl>(buffer);

        Some(TxToken {
            request_sender: &self.request_sender,
            radio_frame: Some(radio_frame),
            request_token: Some(request_token),
            buffer_allocator: &self.buffer_allocator,
        })
    }
}

impl<'driver, RadioDriverImpl: DriverConfig> embassy_net_driver::Driver
    for Ieee802154Driver<'driver, RadioDriverImpl>
{
    type RxToken<'token>
        = RxToken<'token>
    where
        Self: 'token;
    type TxToken<'token>
        = TxToken<'token>
    where
        Self: 'token;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let tx_token = self.tx_token(cx)?;
        let rx_token = self.rx_token(cx)?;
        Some((rx_token, tx_token))
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        self.tx_token(cx)
    }

    fn link_state(&mut self, _cx: &mut Context) -> LinkState {
        LinkState::Up
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = 125;
        caps.max_burst_size = Some(1);

        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.hardware_addr
    }
}

pub struct TxToken<'token> {
    request_sender: &'token MacRequestSender<'token>,
    radio_frame: Option<RadioFrame<RadioFrameUnsized>>,
    request_token: Option<RequestToken>,
    buffer_allocator: &'token MacBufferAllocator,
}

impl<'token> TxToken<'token> {
    /// Check if the given data request is to be acknowledged, based on its
    /// addressing. If so, the acknowledgment request flag is set in the frame
    /// and the sequence number of the frame returned.
    fn set_ack_requested(data_request: &mut DataRequest) {
        let dst_address = data_request.dst_addr();
        let ack_tx = matches!(dst_address, Ok(dst_address) if dst_address.is_unicast());
        data_request.tx_options().set_ack_tx(ack_tx);
    }
}

impl<'token> Drop for TxToken<'token> {
    fn drop(&mut self) {
        // Safety: Release the buffer last to avoid deadlock.
        if let Some(request_token) = self.request_token.take() {
            self.request_sender.release_request_token(request_token);
        }
        if let Some(radio_frame) = self.radio_frame.take() {
            // Safety: We allocated the buffer ourselves from the same allocator.
            unsafe {
                self.buffer_allocator
                    .deallocate_buffer(radio_frame.into_buffer());
            }
        }
    }
}

impl<'token> embassy_net_driver::TxToken for TxToken<'token> {
    fn consume<R, F>(mut self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut radio_frame = self
            .radio_frame
            .take()
            .unwrap()
            .with_size(NonZero::new(len as u16).expect("invalid len"));

        let result = f(radio_frame.sdu_mut());

        let mut data_request = DataRequest::new(MpduFrame::from_radio_frame(radio_frame));
        // TODO: This may conflict with a PAN ID configured into smoltcp.
        //       Embassy, however, doesn't allow to configure a PAN ID anyway.
        let _ = data_request.set_dst_pan_id(MAC_PAN_ID);
        Self::set_ack_requested(&mut data_request);

        let request = MacRequest::McpsDataRequest(data_request);
        self.request_sender
            .send_request_no_response(self.request_token.take().unwrap(), request);

        // No need to drop a consumed token.
        mem::forget(self);

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker(crate::trace::TX_TOKEN_CONSUMED);

        result
    }
}

pub struct RxToken<'token> {
    indication_receiver: &'token MacIndicationReceiver<'token>,
    radio_frame: RadioFrame<RadioFrameSized>,
    response_token: ResponseToken,
    buffer_allocator: &'token MacBufferAllocator,
}

impl<'token> embassy_net_driver::RxToken for RxToken<'token> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(self.radio_frame.sdu_mut());
        self.indication_receiver.received(self.response_token, ());
        // Safety: We use the MAC service's allocator to release a buffer
        //         allocated by the MAC service.
        unsafe {
            self.buffer_allocator
                .deallocate_buffer(self.radio_frame.into_buffer());
        }

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker(crate::trace::RX_TOKEN_CONSUMED);

        result
    }
}
