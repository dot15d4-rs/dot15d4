use crate::phy::radio::{Radio, RadioFrameMut};
use crate::{mac::MacService, phy::FrameBuffer, upper::UpperLayer};
use embedded_hal_async::delay::DelayNs;
use rand_core::RngCore;

pub enum DataError {
    // TODO: not supported
    TransactionOverflow,
    // TODO: not supported
    TransactionExpired,
    // TODO: not supported
    ChannelAccessFailure,
    // TODO: not supported
    InvalidAddress,
    // TODO: not supported
    NoAck,
    // TODO: not supported
    CounterError,
    // TODO: not supported
    FrameTooLong,
    // TODO: not supported
    InvalidParameter,
}

pub struct DataRequest {
    pub buffer: FrameBuffer,
}

pub struct DataConfirm {
    /// Timestamp of frame transmission
    pub timestamp: u32,
    /// Whether the frame has been acknowledged or not
    pub acked: bool,
}

#[derive(Default)]
pub struct DataIndication {
    /// buffer containing the received frame
    pub buffer: FrameBuffer,
    /// Timestamp of frame reception
    pub timestamp: u32,
}

impl<Rng, U, TIMER, R> MacService<'_, Rng, U, TIMER, R>
where
    Rng: RngCore,
    U: UpperLayer,
    TIMER: DelayNs + Clone,
    R: Radio,
    for<'a> R::RadioFrame<&'a mut [u8]>: RadioFrameMut<&'a mut [u8]>,
    for<'a> R::TxToken<'a>: From<&'a mut [u8]>,
{
    /// Requests the transfer of data to another device
    pub async fn mcps_data_request(
        &self,
        frame: &mut FrameBuffer,
    ) -> Result<DataConfirm, DataError> {
        let sequence_number = Self::set_ack(frame);

        self.phy_send(core::mem::take(frame)).await;
        let acked = match sequence_number {
            Some(sequence_number) => self.wait_for_ack(sequence_number).await,
            _ => true,
        };
        Ok(DataConfirm {
            // TODO: support timestamp
            timestamp: 0,
            acked,
        })
    }

    pub async fn mcps_data_indication(&self, indication: &mut DataIndication) {
        self.upper_layer
            .process_mac_indication(crate::mac::primitives::MacIndication::McpsData(
                core::mem::take(indication),
            ))
            .await;
    }
}
