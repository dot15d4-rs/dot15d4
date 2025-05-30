use rand_core::RngCore;

use crate::{driver::DriverConfig, mac::MacService};

#[allow(dead_code)]
pub struct PurgeConfirm {
    msdu_handle: u8,
}

pub enum PurgeError {
    InvalidHandle,
}

#[allow(dead_code)]
impl<'svc, Rng: RngCore, RadioDriverImpl: DriverConfig> MacService<'svc, Rng, RadioDriverImpl> {
    /// Allows a higher layer to purge an MSDU from the transaction
    /// queue.
    async fn purge_request(&mut self) -> Result<PurgeConfirm, PurgeError> {
        // TODO: not supported
        Err(PurgeError::InvalidHandle)
    }
}
