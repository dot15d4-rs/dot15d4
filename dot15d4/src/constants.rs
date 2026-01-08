#![allow(dead_code)]
pub use customizable::*;

#[cfg(test)]
mod customizable {
    #![allow(dead_code)]
    pub const MAC_MIN_BE: u8 = 0;
    pub const MAC_MAX_BE: u8 = 8;
    pub const MAC_MAX_CSMA_BACKOFFS: u8 = 16;
    pub const MAC_MAX_FRAME_RETRIES: u8 = 3; // 0-7
    pub const MAC_PAN_ID: u16 = 0xbeef; // PAN Id
    pub const MAC_IMPLICIT_BROADCAST: bool = false;

    pub const MAC_HOPPING_SEQUENCE_MAX_LENGTH: usize = 16;

    pub const MAC_TSCH_MIN_BE: u8 = 1;
    pub const MAC_TSCH_MAX_BE: u8 = 7;
    pub const MAC_JOIN_METRIC: u16 = 1;
    pub const MAC_DISCONNECT_TIME: u16 = 0x00ff;

    pub const MAC_TSCH_MAX_LINKS: usize = 5;
    pub const MAC_TSCH_MAX_SLOTFRAMES: usize = 1;
    pub const MAC_TSCH_MAX_PENDING_OPERATIONS: usize = 5;
}

#[cfg(not(test))]
mod customizable {
    #![allow(unused)]
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
