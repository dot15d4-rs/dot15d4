use rand_core::RngCore;

use crate::{driver::DriverConfig, mac::MacService};

pub struct AssociateConfirm;

#[allow(dead_code)]
impl<'svc, Rng: RngCore, RadioDriverImpl: DriverConfig> MacService<'svc, Rng, RadioDriverImpl> {
    /// Requests the association with a coordinator.
    async fn mlme_associate_request(&self) -> Result<AssociateConfirm, ()> {
        // TODO: support association
        Err(())
    }
}
