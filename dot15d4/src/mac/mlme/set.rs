#![allow(dead_code)]
use rand_core::RngCore;

use crate::{driver::radio::DriverConfig, mac::MacService};

pub enum SetError {
    InvalidParameter,
}

/// Attributes that may be written by an upper layer
pub enum SetRequestAttribute {
    // IEEE 802.15.4-2020, section 8.4.3.1, table 8-94
    MacExtendedAddress([u8; 8]),
    MacAssociationPermit(bool),
    MacPanId(u16),
    MacShortAddress(u16),
}

#[allow(dead_code)]
impl<'svc, Rng: RngCore, RadioDriverImpl: DriverConfig> MacService<'svc, Rng, RadioDriverImpl> {
    /// Used by the next higher layer to attempt to write the given value to
    /// the indicated MAC PIB attribute.
    ///
    /// * `attribute` - Attribute to write
    pub(crate) async fn mlme_set_request(
        &self,
        attribute: &SetRequestAttribute,
    ) -> Result<(), SetError> {
        let mut pib = self.pib.borrow_mut();
        match attribute {
            SetRequestAttribute::MacPanId(pan_id) => pib.pan_id.set_u16(*pan_id),
            SetRequestAttribute::MacShortAddress(short_address) => {
                pib.short_address = *short_address
            }
            SetRequestAttribute::MacExtendedAddress(extended_address) => {
                pib.extended_address = Some(*extended_address)
            }
            SetRequestAttribute::MacAssociationPermit(association_permit) => {
                pib.association_permit = *association_permit
            }
        }
        Ok(())
    }
}
