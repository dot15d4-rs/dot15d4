#![allow(dead_code)]

use crate::timer::SymbolsOQpsk250Duration;

// TODO: Several of the following values are PHY-specific. Rename accordingly.

// Constants of IEEE 802.15.4-2024, section 8.4.2, Table 8-35, MAC constants
/// The number of symbols forming a superframe slot when the superframe order is
/// equal to zero, as described in 6.2.1.
pub const A_BASE_SLOT_DURATION: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(60);
/// The number of symbols forming a superframe when the superframe order is
/// equal to zero.
pub const A_BASE_SUPERFRAME_DURATION: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(
    A_BASE_SLOT_DURATION.ticks() * A_NUM_SUPERFRAME_SLOTS as u64,
);
/// The number of consecutive lost beacons that will cause the MAC sublayer of a
/// receiving device to declare a loss of synchronization.
pub const A_MAX_LOST_BEACONS: u8 = 4;
/// The maximum size of an MPDU, in octets, that can be followed by a SIFS
/// period.
pub const A_MAX_SIFS_FRAME_SIZE: u16 = 18;
/// The minimum number of symbols forming the CAP. This ensures that MAC
/// commands can still be transferred to devices when GTSs are being used.
///
/// An exception to this minimum shall be allowed for the accommodation of the
/// temporary increase in the beacon frame length needed to perform GTS
/// maintenance, as described in 7.3.1.5. Additional restrictions apply when PCA
/// is enabled, as described in 6.2.5.4.
pub const A_MIN_CAP_LENGTH: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(440);
/// The number of slots contained in any superframe.
pub const A_NUM_SUPERFRAME_SLOTS: u8 = 16;

// Constants from section 10.25.11, Table 10-121
/// The number of superframes in which a GTS descriptor exists in the beacon
/// frame of the PAN coordinator.
pub const A_GTS_DESC_PERSISTENCE_TIME: u8 = 4;

// Constants from section 11.3, Table 11-1, PHY constants
/// The maximum PSDU size (in octets) the PHY shall be able to receive.
pub const PHY_MAX_PACKET_SIZE_2047: usize = 2048; // SUN, TVWS, RCC, LECIM FSK, and MSK with a 2000 kb/s data rate
pub const PHY_MAX_PACKET_SIZE_127: usize = 127; // all other PHYs

/// RX-to-TX or TX-to-RX turnaround time (in symbol periods), as defined in
/// 10.2.2 and 10.2.3.
pub const A_TURNAROUND_TIME: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(12);
/// The time required to perform CCA detection in symbol periods.
pub const PHY_CCA_DURATION: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(8);
/// The number of symbols forming the basic time period used by the CSMA-CA
/// algorithm.
pub const MAC_UNIT_BACKOFF_PERIOD: SymbolsOQpsk250Duration =
    SymbolsOQpsk250Duration::from_ticks(A_TURNAROUND_TIME.ticks() + PHY_CCA_DURATION.ticks());
/// O-QPSK symbol rate for 2.4G is 62.5kS/s, i.e. a symbol period of 16µs.
/// SIFS: 12 symbols = 192µs
pub const MAC_SIFS: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(12);
/// LIFS: 40 symbols = 480µs
pub const MAC_LIFS: SymbolsOQpsk250Duration = SymbolsOQpsk250Duration::from_ticks(40);
/// AIFS=1ms, for SUN PHY, LECIM PHY, TVWS PHY, SIFS otherwise
pub const MAC_AIFS: SymbolsOQpsk250Duration = MAC_SIFS;

/// O-QPSK Start of Frame Delimiter
pub const DEFAULT_SFD: u8 = 0xA7;
pub const PHY_HDR_LEN: usize = 1;
pub const FCS_LEN: usize = 2;

#[cfg(test)]
mod tests {
    use crate::timer::SymbolsOQpsk250Duration;

    #[test]
    fn inv_symbol_rate() {
        let symbol_period: u64 = SymbolsOQpsk250Duration::from_ticks(1).to_micros();
        assert_eq!(symbol_period, 16);
    }
}
