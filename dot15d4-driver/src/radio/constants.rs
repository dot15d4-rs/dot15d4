#![allow(dead_code)]

/// The number of consecutive lost beacons that will cause the MAC sublayer of a
/// receiving device to declare a loss of synchronization.
pub const A_MAX_LOST_BEACONS: u8 = 4;
/// The maximum size of an MPDU, in octets, that can be followed by a SIFS
/// period.
pub const A_MAX_SIFS_FRAME_SIZE: u16 = 18;
/// The number of slots contained in any superframe.
pub const A_NUM_SUPERFRAME_SLOTS: u8 = 16;

// Constants from section 10.25.11, Table 10-121
/// The number of superframes in which a GTS descriptor exists in the beacon
/// frame of the PAN coordinator.
pub const A_GTS_DESC_PERSISTENCE_TIME: u8 = 4;
