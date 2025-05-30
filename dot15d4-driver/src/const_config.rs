#![allow(dead_code)]
pub use customizable::*;

#[cfg(test)]
mod customizable {
    #![allow(dead_code)]
    use crate::frame::PanId;

    // XXX These are just random numbers I picked by fair dice roll; what should
    // they be?
    pub const MAC_MIN_BE: u8 = 0;
    pub const MAC_MAX_BE: u8 = 8;
    pub const MAC_MAX_CSMA_BACKOFFS: u8 = 16;
    pub const MAC_MAX_FRAME_RETRIES: u8 = 3; // 0-7
    pub const MAC_PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0xff, 0xff]); // PAN Id
    pub const MAC_IMPLICIT_BROADCAST: bool = false;
}

#[cfg(not(test))]
mod customizable {
    #![allow(unused)]
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
