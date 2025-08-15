use crate::{driver::radio::DriverConfig, mac::MacService};

struct StartConfirm {}

#[allow(dead_code)]
impl<'svc, RadioDriverImpl: DriverConfig> MacService<'svc, RadioDriverImpl> {
    /// Used by PAN coordinator to initiate a new PAN or to begin using a new
    /// configuration. Also used by a device already associated with an
    /// existing PAN to begin using a new configuration.
    async fn mlme_start_request(&mut self) -> Result<StartConfirm, ()> {
        Err(())
    }
}
