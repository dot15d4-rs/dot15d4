use crate::{driver::radio::DriverConfig, mac::MacService};

pub struct AssociateConfirm;

#[allow(dead_code)]
impl<'svc, RadioDriverImpl: DriverConfig> MacService<'svc, RadioDriverImpl> {
    /// Requests the association with a coordinator.
    async fn mlme_associate_request(&self) -> Result<AssociateConfirm, ()> {
        // TODO: support association
        Err(())
    }
}
