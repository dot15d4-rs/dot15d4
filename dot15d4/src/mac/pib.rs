use crate::driver::frame::PanId;

/// PAN Information Base (PIB) specified by MAC sublayer
#[allow(dead_code)]
pub struct Pib {
    /// The extended address assigned to the device.
    pub(crate) extended_address: Option<[u8; 8]>,
    /// Indication of whether the device is associated to the PAN through the
    /// PAN coordinator. A value of `true` indicates the device has associated
    /// through the PAN coordinator. Otherwise, the value is set to `false`.
    pub(crate) associated_pan_coord: bool,
    /// Indication of whether a coordinator is currently allowing association.
    /// If `true`, association is permitted.
    pub(crate) association_permit: bool,
    /// The address of the coordinator through which the device is associated.
    pub(crate) coord_extended_address: Option<[u8; 8]>,
    /// The short address assigned to the coordinator through which the device
    /// is associated. A value of 0xfffe indicates that the coordinator is
    /// only using its extended address. A value of 0xffff indicates that this
    /// value is unknown.
    pub(crate) coord_short_address: u16,
    /// The maximum value of the backoff exponent, BE, in the CSMA-CA
    /// algorithm.
    pub(crate) max_be: u8,
    /// The minimum value of the backoff exponent (BE) in the CSMA-CA
    /// algorithm.
    pub(crate) min_be: u8,
    /// The maximum number of retries allowed after a transmission failure.
    pub(crate) max_frame_retries: u8,
    /// The maximum number of backoffs the CSMA-CA algorithm will attempt
    /// before declaring a channel access failure.
    pub(crate) max_csma_backoffs: u8,
    /// The identifier of the PAN on which the device is operating. If this
    /// value is 0xffff, the device is not associated.
    pub(crate) pan_id: PanId<[u8; 2]>,
    /// Indication of whether the MAC sublayer is in a promiscuous (receive
    /// all) mode. A value of `true` indicates that the MAC sublayer accepts
    /// all frames received from the PHY.
    pub(crate) promiscuous_mode: bool,
    /// Indication of whether the MAC sublayer is to enable its receiver
    /// during idle periods. For a beacon-enabled PAN, this attribute is
    /// relevant only during the CAP of the incoming superframe. For a
    /// non-beacon-enabled PAN, this attribute is relevant at all times.
    pub(crate) rx_on_when_idle: bool,
    /// Indication of whether the MAC sublayer has security enabled. A value
    /// of `true` indicates that security is enabled, while a value of `false`
    /// indicates that security is disabled.
    pub(crate) security_enabled: bool,
    /// The address that the device uses to communicate in the PAN. If the
    /// device is the PAN coordinator, this value shall be chosen before a PAN
    /// is started. Otherwise, the short address is allocated by a coordinator
    /// during association.
    pub(crate) short_address: u16,
    /// Specification of how often the coordinator transmits an Enhanced
    /// Beacon frame. Value ranges from 0 to 15. If value is 15, no periodic
    /// Enhanced Beacon frame will be transmitted.
    pub(crate) enhanced_beacon_order: u8,
}

impl Default for Pib {
    fn default() -> Self {
        use dot15d4_driver::const_config::*;

        Self {
            extended_address: None,
            associated_pan_coord: false,
            association_permit: false,
            coord_extended_address: None,
            coord_short_address: 0xffff,
            max_be: MAC_MAX_BE,
            min_be: MAC_MIN_BE,
            max_frame_retries: MAC_MAX_FRAME_RETRIES,
            max_csma_backoffs: MAC_MAX_CSMA_BACKOFFS,
            pan_id: MAC_PAN_ID,
            promiscuous_mode: false,
            rx_on_when_idle: false,
            security_enabled: false,
            short_address: 0xffff,
            enhanced_beacon_order: 0,
        }
    }
}
