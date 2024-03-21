#![allow(dead_code)]

use crate::time::Duration;

use super::constants::{SYMBOL_RATE_INV_US, UNIT_BACKOFF_PERIOD};

// XXX These are just random numbers I picked by fair dice roll; what should
// they be?
pub const MAC_MIN_BE: u16 = 0;
pub const MAC_MAX_BE: u16 = 8;
pub const MAC_MAX_CSMA_BACKOFFS: u16 = 16;
pub const MAC_UNIT_BACKOFF_DURATION: Duration =
    Duration::from_us((UNIT_BACKOFF_PERIOD * SYMBOL_RATE_INV_US) as i64);
pub const MAC_MAX_FRAME_RETIES: u16 = 3; // 0-7
pub const _MAC_INTER_FRAME_TIME: Duration = Duration::from_us(1000); // TODO: XXX
/// AIFS=1ms, for SUN PHY, LECIM PHY, TVWS PHY
pub const ACKNOWLEDGEMENT_INTERFRAME_SPACING: Duration = Duration::from_us(1000);
pub const MAC_SIFT_PERIOD: Duration = Duration::from_us(1000); // TODO: SIFS=XXX
pub const MAC_LIFS_PERIOD: Duration = Duration::from_us(10_000); // TODO: LIFS=XXX
