use dot15d4_driver::timer::NsInstant;

use crate::util::sync::HasAddress;

use super::mlme::reset::ResetRequest;
use super::mlme::set::SetRequest;
#[cfg(feature = "tsch")]
pub use super::mlme::tsch::mode::TschModeRequest;
#[cfg(feature = "tsch")]
use super::mlme::tsch::{setlink::SetLinkRequest, setslotframe::SetSlotframeRequest};
pub use super::{
    mcps::data::{DataIndication, DataRequest},
    mlme::beacon::{BeaconNotifyIndication, BeaconRequest},
};

// TODO: update doc, protocol version and sections numbering
/// Enum representing all (currently) supported MAC services request primitives
pub enum MacRequest {
    /// IEEE 802.15.4-2024, section 10.3.10.7
    #[cfg(feature = "tsch")]
    MlmeTschMode(TschModeRequest),
    /// IEEE 802.15.4-2024, section 10.3.10.2
    #[cfg(feature = "tsch")]
    MlmeSetSlotframe(SetSlotframeRequest),
    /// IEEE 802.15.4-2024, section 10.3.10.4
    #[cfg(feature = "tsch")]
    MlmeSetLink(SetLinkRequest),
    /// IEEE 802.15.4-2024, section 8.2.5.4
    MlmeSet(SetRequest),
    /// IEEE 802.15.4-2024, section 8.2.6.2
    MlmeReset(ResetRequest),
    /// IEEE 802.15.4-2020, section 8.2.18.1
    MlmeBeacon(BeaconRequest),
    /// IEEE 802.15.4-2020, section 8.3.2
    McpsData(DataRequest),
}

// FIXME: temporary solution. Should mirror MacRequest
pub struct MacConfirm {
    pub timestamp: Option<NsInstant>,
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
