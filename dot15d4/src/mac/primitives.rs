use crate::util::sync::HasAddress;

pub use super::{
    mcps::data::{DataIndication, DataRequest},
    mlme::{
        beacon::{BeaconNotifyIndication, BeaconRequest},
        set::SetRequestAttribute,
    },
};

/// Enum representing all (currently) supported MAC services request primitives
pub enum MacRequest {
    /// IEEE 802.15.4-2020, section 8.2.6.4
    MlmeSetRequest(SetRequestAttribute),
    /// IEEE 802.15.4-2020, section 8.2.18.1
    MlmeBeaconRequest(BeaconRequest),
    /// IEEE 802.15.4-2020, section 8.3.2
    McpsDataRequest(DataRequest),
}

/// Fake implementation to satisfy the generic channel.
///
/// May have to change if we want to direct MLME messages to a different
/// receiver than MCPS messages, for example.
impl HasAddress<()> for MacRequest {
    fn matches(&self, _: &()) -> bool {
        true
    }
}

pub enum MacIndication {
    McpsData(DataIndication),
    MlmeBeaconNotify(BeaconNotifyIndication),
}

/// Fake implementation to satisfy the generic channel.
///
/// Will change once we allow several tasks to listen for indications in
/// parallel.
impl HasAddress<()> for MacIndication {
    fn matches(&self, _: &()) -> bool {
        true
    }
}
