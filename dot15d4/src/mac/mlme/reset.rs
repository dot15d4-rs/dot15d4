use core::cell::RefCell;

use crate::{
    driver::radio::DriverConfig,
    mac::{pib, MacService},
};

#[allow(dead_code)]
pub struct ResetConfirm {
    status: bool,
}

#[allow(dead_code)]
impl<'svc, RadioDriverImpl: DriverConfig> MacService<'svc, RadioDriverImpl> {
    /// Used by the next higher layer to request a reset operation that
    /// involves resetting the PAN Information Base
    async fn mlme_reset_request(&mut self, set_default_pib: bool) -> Result<ResetConfirm, ()> {
        if set_default_pib {
            self.pib = RefCell::new(pib::Pib::default());
        }
        Ok(ResetConfirm { status: true })
    }
}
