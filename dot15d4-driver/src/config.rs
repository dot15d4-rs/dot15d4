/// IEEE 802.15.4 channels
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Channel {
    /// 2_405 MHz
    _11,
    /// 2_410 MHz
    _12,
    /// 2_415 MHz
    _13,
    /// 2_420 MHz
    _14,
    /// 2_425 MHz
    _15,
    /// 2_430 MHz
    _16,
    /// 2_435 MHz
    _17,
    /// 2_440 MHz
    _18,
    /// 2_445 MHz
    _19,
    /// 2_450 MHz
    _20,
    /// 2_455 MHz
    _21,
    /// 2_460 MHz
    _22,
    /// 2_465 MHz
    _23,
    /// 2_470 MHz
    _24,
    /// 2_475 MHz
    _25,
    /// 2_480 MHz
    _26,
}

impl TryFrom<u8> for Channel {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            11 => Ok(Channel::_11),
            12 => Ok(Channel::_12),
            13 => Ok(Channel::_13),
            14 => Ok(Channel::_14),
            15 => Ok(Channel::_15),
            16 => Ok(Channel::_16),
            17 => Ok(Channel::_17),
            18 => Ok(Channel::_18),
            19 => Ok(Channel::_19),
            20 => Ok(Channel::_20),
            21 => Ok(Channel::_21),
            22 => Ok(Channel::_22),
            23 => Ok(Channel::_23),
            24 => Ok(Channel::_24),
            25 => Ok(Channel::_25),
            26 => Ok(Channel::_26),
            _ => Err(()),
        }
    }
}

impl TryFrom<i32> for Channel {
    type Error = ();

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            11 => Ok(Channel::_11),
            12 => Ok(Channel::_12),
            13 => Ok(Channel::_13),
            14 => Ok(Channel::_14),
            15 => Ok(Channel::_15),
            16 => Ok(Channel::_16),
            17 => Ok(Channel::_17),
            18 => Ok(Channel::_18),
            19 => Ok(Channel::_19),
            20 => Ok(Channel::_20),
            21 => Ok(Channel::_21),
            22 => Ok(Channel::_22),
            23 => Ok(Channel::_23),
            24 => Ok(Channel::_24),
            25 => Ok(Channel::_25),
            26 => Ok(Channel::_26),
            _ => Err(()),
        }
    }
}

impl From<Channel> for u8 {
    fn from(ch: Channel) -> u8 {
        match ch {
            Channel::_11 => 11,
            Channel::_12 => 12,
            Channel::_13 => 13,
            Channel::_14 => 14,
            Channel::_15 => 15,
            Channel::_16 => 16,
            Channel::_17 => 17,
            Channel::_18 => 18,
            Channel::_19 => 19,
            Channel::_20 => 20,
            Channel::_21 => 21,
            Channel::_22 => 22,
            Channel::_23 => 23,
            Channel::_24 => 24,
            Channel::_25 => 25,
            Channel::_26 => 26,
        }
    }
}

impl Default for Channel {
    fn default() -> Self {
        Self::_26
    }
}

// TODO: expose the other variants in `pac::CCAMODE_A`
/// Clear Channel Assessment method
pub enum CcaMode {
    /// Carrier sense
    CarrierSense,
    /// Energy Detection / Energy Above Threshold
    EnergyDetection {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in d_bm(),
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Default, Clone, PartialEq)]
pub struct RxConfig {
    pub channel: Channel,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Default, Clone, PartialEq)]
pub struct TxConfig {
    pub channel: Channel,
    pub cca: bool,
}

impl TxConfig {
    pub fn default_with_cca() -> Self {
        Self {
            cca: true,
            ..Default::default()
        }
    }
}
