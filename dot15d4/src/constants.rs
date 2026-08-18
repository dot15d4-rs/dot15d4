#![allow(dead_code)]
pub use customizable::*;

#[cfg(test)]
mod customizable {
    #![allow(dead_code)]
    // Test-only config, intentionally outside the standard ranges: MIN_BE=0
    // forces a zero first backoff and the large MAX_CSMA_BACKOFFS lets the
    // retry tests step through many attempts. The range checks in build.rs
    // apply to the generated (non-test) config only.
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
    pub const MAC_NEIGHBOR_TABLE_SIZE: usize = 8;
    pub const MAC_ETX_NOACK_PENALTY: u16 = 12;
    pub const MAC_DISCONNECT_TIME: u16 = 0x00ff;

    // Default macKeepAlivePeriod in timeslots (10.3.10.8); 0 disables the
    // keep-alive. 1500 slots = 15 s with the default 10 ms timeslot.
    pub const MAC_TSCH_KEEP_ALIVE_PERIOD: u16 = 1500;

    // Test-only desynchronization thresholds. Kept small so a test can step
    // through the whole staircase (relax the guard, then disconnect) in a
    // handful of wake-ups.
    pub const MAC_TSCH_MAX_KEEP_ALIVE_MISSES: u8 = 3;
    pub const MAC_TSCH_GUARD_RELAX_MISSES: u8 = 1;

    pub const MAC_TSCH_MAX_LINKS: usize = 5;
    pub const MAC_TSCH_MAX_SLOTFRAMES: usize = 1;
    pub const MAC_TSCH_MAX_PENDING_OPERATIONS: usize = 5;
    pub const MAC_TSCH_DRIFT_BUFFER_LEN: usize = 8;
    pub const MAC_TSCH_MAX_DRIFT_PPM: u32 = 200;
    pub const MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS: u32 = 100;

    pub const MAC_TSCH_GUARD_FLOOR_US: u16 = 100;
    pub const MAC_TSCH_GUARD_SAFE_US: u16 = 2200;
    pub const MAC_TSCH_GUARD_COMMIT_SYNCS: u8 = 8;
    pub const MAC_TSCH_GUARD_COMMIT_MARGIN: u8 = 4;
}

#[cfg(not(test))]
mod customizable {
    #![allow(unused)]
    include!(concat!(env!("OUT_DIR"), "/config.rs"));
}
