//! MAC neighbor table.
//!
//! One fixed-size table per device that stores the identity of the MAC
//! neighbors (extended address, and the short address once known) together
//! with per-neighbor link statistics: ETX, when the neighbor was last heard,
//! when a frame to it was last acknowledged, TX/RX counters, and the join
//! metric from its last Enhanced Beacon.
//!
//! Design rules:
//! - The table is the only owner of neighbor addresses. Everything else (TSCH
//!   links, the time source, in-flight frames) refers to a neighbor with a
//!   small [`NeighborId`] handle (1 byte), never the address bytes.
//! - Lookups take borrowed [`Address<&[u8]>`] views straight from a frame
//!   buffer and compare little-endian bytes, with no byte reversal or copies on
//!   the hot path. The only copy in a neighbor's life is the 8-byte address
//!   copy done once by [`NeighborTable::intern`].
//! - Behavior lives on the table (`note_*` methods), not behind field setters,
//!   so an update is one call from the scheduler slot path.

use core::num::NonZeroU8;

use dot15d4_driver::radio::frame::Address;
use dot15d4_driver::timer::NsInstant;

use crate::constants::{MAC_ETX_NOACK_PENALTY, MAC_NEIGHBOR_TABLE_SIZE};

/// ETX fixed-point scale: `128` means an ETX of `1.0`.
pub const ETX_DIVISOR: u16 = 128;

/// EWMA weight for the ETX estimate:
/// `new = old + (sample - old) / 2^ETX_ALPHA_SHIFT`.
const ETX_ALPHA_SHIFT: u32 = 3;

/// Marker for "short address not known".
const SHORT_ADDR_UNKNOWN: u16 = 0xffff;

/// The neighbor table type with the configured capacity.
pub type MacNeighborTable = NeighborTable<MAC_NEIGHBOR_TABLE_SIZE>;

/// Handle into the neighbor table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NeighborId(NonZeroU8);

impl NeighborId {
    fn from_index(index: usize) -> Self {
        // The table capacity is far below 255, so `index + 1` never wraps
        // and is never zero.
        Self(NonZeroU8::new((index as u8) + 1).unwrap())
    }

    fn index(self) -> usize {
        (self.0.get() - 1) as usize
    }
}

mod flags {
    /// The slot holds a neighbor.
    pub const IN_USE: u8 = 1 << 0;
    /// At least one Enhanced Beacon from this neighbor was seen.
    pub const EB_HEARD: u8 = 1 << 1;
    /// The entry is referenced (time source or link neighbor); never evict.
    pub const PINNED: u8 = 1 << 2;
}

/// One neighbor entry.
///
/// All fields are private; reads go through the accessors and writes go
/// through the `note_*` methods on [`NeighborTable`].
#[derive(Debug, Clone, Copy)]
pub struct Neighbor {
    /// Extended address, little-endian (the on-air byte order).
    ext_addr: [u8; 8],
    /// Short address, or [`SHORT_ADDR_UNKNOWN`].
    short_addr: u16,
    /// Last time a valid frame from this neighbor was accepted.
    last_heard: NsInstant,
    /// Last time a frame to this neighbor was acknowledged.
    last_sent_ok: NsInstant,
    /// ETX estimate in 1/128 units (EWMA); `0` means "no estimate yet".
    etx_q7: u16,
    /// Unicast transmission attempts toward this neighbor (saturating).
    tx_attempts: u16,
    /// Acknowledged unicast transmissions toward this neighbor (saturating).
    tx_acked: u16,
    /// Frames heard from this neighbor (saturating).
    rx_count: u16,
    /// Join metric from the neighbor's last Enhanced Beacon.
    advertised_join_metric: u8,
    /// Link quality EWMA. Stays `0` until the radio driver reports LQI.
    lqi_ewma: u8,
    flags: u8,
}

impl Neighbor {
    const EMPTY: Neighbor = Neighbor {
        ext_addr: [0; 8],
        short_addr: SHORT_ADDR_UNKNOWN,
        last_heard: NsInstant::from_ticks(0),
        last_sent_ok: NsInstant::from_ticks(0),
        etx_q7: 0,
        tx_attempts: 0,
        tx_acked: 0,
        rx_count: 0,
        advertised_join_metric: 0,
        lqi_ewma: 0,
        flags: 0,
    };

    fn in_use(&self) -> bool {
        self.flags & flags::IN_USE != 0
    }

    /// Extended address, little-endian bytes.
    pub fn ext_addr(&self) -> &[u8; 8] {
        &self.ext_addr
    }

    /// Short address if known.
    pub fn short_addr(&self) -> Option<u16> {
        (self.short_addr != SHORT_ADDR_UNKNOWN).then_some(self.short_addr)
    }

    /// Last time a valid frame from this neighbor was accepted.
    pub fn last_heard(&self) -> NsInstant {
        self.last_heard
    }

    /// Last time a frame to this neighbor was acknowledged.
    pub fn last_sent_ok(&self) -> NsInstant {
        self.last_sent_ok
    }

    /// ETX estimate in 1/128 units ([`ETX_DIVISOR`]); `0` = no estimate yet.
    pub fn etx_q7(&self) -> u16 {
        self.etx_q7
    }

    /// Unicast transmission attempts toward this neighbor.
    pub fn tx_attempts(&self) -> u16 {
        self.tx_attempts
    }

    /// Acknowledged unicast transmissions toward this neighbor.
    pub fn tx_acked(&self) -> u16 {
        self.tx_acked
    }

    /// Frames heard from this neighbor.
    pub fn rx_count(&self) -> u16 {
        self.rx_count
    }

    /// Join metric from the neighbor's last Enhanced Beacon.
    pub fn advertised_join_metric(&self) -> u8 {
        self.advertised_join_metric
    }

    /// Whether at least one Enhanced Beacon from this neighbor was seen.
    pub fn eb_heard(&self) -> bool {
        self.flags & flags::EB_HEARD != 0
    }

    /// Whether this entry is protected from eviction.
    pub fn is_pinned(&self) -> bool {
        self.flags & flags::PINNED != 0
    }

    /// Link quality EWMA (`0` until the driver reports LQI).
    pub fn lqi(&self) -> u8 {
        self.lqi_ewma
    }
}

/// A copyable, self-contained view of one neighbor entry.
///
/// This is what leaves the MAC (MLME-GET, logging): a plain value, so no
/// borrow of the table escapes the scheduler task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NeighborSnapshot {
    /// Extended address, little-endian.
    pub ext_addr: [u8; 8],
    /// Short address, `0xffff` when unknown.
    pub short_addr: u16,
    pub last_heard: NsInstant,
    pub last_sent_ok: NsInstant,
    /// ETX in 1/128 units, `0` = no estimate yet.
    pub etx_q7: u16,
    pub tx_attempts: u16,
    pub tx_acked: u16,
    pub rx_count: u16,
    pub advertised_join_metric: u8,
    pub lqi: u8,
}

impl From<&Neighbor> for NeighborSnapshot {
    fn from(n: &Neighbor) -> Self {
        Self {
            ext_addr: n.ext_addr,
            short_addr: n.short_addr,
            last_heard: n.last_heard,
            last_sent_ok: n.last_sent_ok,
            etx_q7: n.etx_q7,
            tx_attempts: n.tx_attempts,
            tx_acked: n.tx_acked,
            rx_count: n.rx_count,
            advertised_join_metric: n.advertised_join_metric,
            lqi: n.lqi_ewma,
        }
    }
}

/// Fixed-capacity MAC neighbor table.
pub struct NeighborTable<const N: usize> {
    slots: [Neighbor; N],
}

impl<const N: usize> NeighborTable<N> {
    pub const fn new() -> Self {
        Self {
            slots: [Neighbor::EMPTY; N],
        }
    }

    /// Find a neighbor by a borrowed address view (extended or short).
    ///
    /// The broadcast address never matches. A short address only matches once
    /// its mapping to an extended address is known.
    pub fn find(&self, address: &Address<&[u8]>) -> Option<NeighborId> {
        if address.is_broadcast() {
            return None;
        }
        match address {
            Address::Extended(ext) => self.find_ext(ext.as_ref()),
            Address::Short(short) => {
                let short = short.into_u16();
                self.position(|n| n.short_addr == short)
            }
            _ => None,
        }
    }

    /// Find a neighbor by its little-endian extended address bytes.
    pub fn find_ext(&self, ext_addr_le: &[u8]) -> Option<NeighborId> {
        if ext_addr_le.len() != 8 {
            return None;
        }
        self.position(|n| n.ext_addr[..] == *ext_addr_le)
    }

    fn position(&self, pred: impl Fn(&Neighbor) -> bool) -> Option<NeighborId> {
        self.slots
            .iter()
            .position(|n| n.in_use() && pred(n))
            .map(NeighborId::from_index)
    }

    /// Find a neighbor, or create it if the address is a (non-broadcast)
    /// extended address.
    ///
    /// Returns `None` when the address cannot identify a neighbor (absent,
    /// broadcast, or a short address with no known mapping) or when the table
    /// is full of pinned entries. This is the only place an address is copied.
    pub fn intern(&mut self, address: &Address<&[u8]>, now: NsInstant) -> Option<NeighborId> {
        if let Some(id) = self.find(address) {
            return Some(id);
        }
        if address.is_broadcast() {
            return None;
        }
        let Address::Extended(ext) = address else {
            return None;
        };
        let index = self.free_or_evict()?;
        let neighbor = &mut self.slots[index];
        *neighbor = Neighbor::EMPTY;
        neighbor.ext_addr.copy_from_slice(ext.as_ref());
        neighbor.flags = flags::IN_USE;
        neighbor.last_heard = now;
        Some(NeighborId::from_index(index))
    }

    /// A free slot, or the least recently heard entry that is not pinned.
    fn free_or_evict(&mut self) -> Option<usize> {
        if let Some(index) = self.slots.iter().position(|n| !n.in_use()) {
            return Some(index);
        }
        self.slots
            .iter()
            .enumerate()
            .filter(|(_, n)| !n.is_pinned())
            .min_by_key(|(_, n)| n.last_heard)
            .map(|(index, _)| index)
    }

    /// Read access to an entry; `None` when the handle points at a slot that
    /// was evicted (or never used).
    pub fn get(&self, id: NeighborId) -> Option<&Neighbor> {
        self.slots.get(id.index()).filter(|n| n.in_use())
    }

    fn get_mut(&mut self, id: NeighborId) -> Option<&mut Neighbor> {
        self.slots.get_mut(id.index()).filter(|n| n.in_use())
    }

    /// A copyable view of an entry.
    pub fn snapshot(&self, id: NeighborId) -> Option<NeighborSnapshot> {
        self.get(id).map(NeighborSnapshot::from)
    }

    /// A copyable view of the entry stored at `index` (for walking the table
    /// from outside, e.g. MLME-GET).
    pub fn snapshot_at(&self, index: usize) -> Option<NeighborSnapshot> {
        self.slots
            .get(index)
            .filter(|n| n.in_use())
            .map(NeighborSnapshot::from)
    }

    /// Protect an entry from eviction. Used when the entry becomes the time
    /// source or is referenced by a TSCH link.
    pub fn pin(&mut self, id: NeighborId) {
        if let Some(n) = self.get_mut(id) {
            n.flags |= flags::PINNED;
        }
    }

    /// Make an entry evictable again, once nothing references it any more (the
    /// node stopped using it as its time source, a TSCH link was removed).
    ///
    /// Without this, a device that joins several different parents over its
    /// lifetime accumulates one un-evictable entry per former parent, and
    /// [`Self::intern`] eventually refuses new neighbors on a table that is
    /// full of them.
    pub fn unpin(&mut self, id: NeighborId) {
        if let Some(n) = self.get_mut(id) {
            n.flags &= !flags::PINNED;
        }
    }

    /// Record a valid frame heard from the neighbor.
    pub fn note_rx(&mut self, id: NeighborId, now: NsInstant) {
        if let Some(n) = self.get_mut(id) {
            n.last_heard = now;
            n.rx_count = n.rx_count.saturating_add(1);
        }
    }

    /// Record an Enhanced Beacon heard from the neighbor.
    pub fn note_beacon(&mut self, id: NeighborId, join_metric: u8, now: NsInstant) {
        if let Some(n) = self.get_mut(id) {
            n.last_heard = now;
            n.advertised_join_metric = join_metric;
            n.flags |= flags::EB_HEARD;
        }
    }

    /// Record the neighbor's short address (learned from association or from
    /// frames carrying both addresses).
    pub fn note_short_address(&mut self, id: NeighborId, short_addr: u16) {
        if let Some(n) = self.get_mut(id) {
            n.short_addr = short_addr;
        }
    }

    /// Record the outcome of a completed unicast exchange toward the
    /// neighbor and update the ETX estimate.
    ///
    /// `attempts_used` is the total number of transmission attempts the
    /// exchange took (1 for a first-try ACK). A failed exchange (retry budget
    /// exhausted) is scored with [`MAC_ETX_NOACK_PENALTY`] attempts.
    pub fn note_tx_result(
        &mut self,
        id: NeighborId,
        acked: bool,
        attempts_used: u8,
        now: NsInstant,
    ) {
        let Some(n) = self.get_mut(id) else {
            return;
        };
        n.tx_attempts = n.tx_attempts.saturating_add(attempts_used as u16);
        let cost = if acked {
            n.tx_acked = n.tx_acked.saturating_add(1);
            n.last_sent_ok = now;
            (attempts_used.max(1)) as u16
        } else {
            MAC_ETX_NOACK_PENALTY
        };
        let sample_q7 = cost.saturating_mul(ETX_DIVISOR);
        n.etx_q7 = if n.etx_q7 == 0 {
            sample_q7
        } else {
            let old = n.etx_q7 as i32;
            (old + ((sample_q7 as i32 - old) >> ETX_ALPHA_SHIFT)) as u16
        };
    }

    /// Iterate over the used entries.
    pub fn iter(&self) -> impl Iterator<Item = (NeighborId, &Neighbor)> {
        self.slots
            .iter()
            .enumerate()
            .filter(|(_, n)| n.in_use())
            .map(|(index, n)| (NeighborId::from_index(index), n))
    }

    /// Number of used entries.
    pub fn len(&self) -> usize {
        self.slots.iter().filter(|n| n.in_use()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Table capacity.
    pub const fn capacity(&self) -> usize {
        N
    }
}

impl<const N: usize> Default for NeighborTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use dot15d4_driver::radio::frame::{Address, ExtendedAddress, ShortAddress};

    use super::*;

    fn t(ticks: u64) -> NsInstant {
        NsInstant::from_ticks(ticks)
    }

    fn ext(last: u8) -> [u8; 8] {
        [1, 2, 3, 4, 5, 6, 7, last]
    }

    fn ext_view(bytes: &[u8; 8]) -> Address<&[u8]> {
        Address::Extended(ExtendedAddress::new(&bytes[..]))
    }

    #[test]
    fn intern_then_find_by_extended_and_short() {
        let mut table = NeighborTable::<4>::new();
        let a = ext(1);
        let id = table.intern(&ext_view(&a), t(10)).unwrap();
        assert_eq!(table.find(&ext_view(&a)), Some(id));
        assert_eq!(table.len(), 1);
        // Interning the same address again returns the same handle.
        assert_eq!(table.intern(&ext_view(&a), t(20)), Some(id));
        assert_eq!(table.len(), 1);

        // Short address resolves only once the mapping is known.
        let short_bytes = 0x1234u16.to_le_bytes();
        let short = Address::Short(ShortAddress::new(&short_bytes[..]));
        assert_eq!(table.find(&short), None);
        table.note_short_address(id, 0x1234);
        assert_eq!(table.find(&short), Some(id));
    }

    #[test]
    fn broadcast_and_short_do_not_create_entries() {
        let mut table = NeighborTable::<4>::new();
        let bcast = [0xff; 8];
        assert_eq!(table.intern(&ext_view(&bcast), t(0)), None);
        let short_bytes = 0x0001u16.to_le_bytes();
        let short = Address::Short(ShortAddress::new(&short_bytes[..]));
        assert_eq!(table.intern(&short, t(0)), None);
        assert!(table.is_empty());
    }

    #[test]
    fn eviction_takes_the_oldest_unpinned_entry() {
        let mut table = NeighborTable::<2>::new();
        let a = ext(1);
        let b = ext(2);
        let c = ext(3);
        let id_a = table.intern(&ext_view(&a), t(10)).unwrap();
        let id_b = table.intern(&ext_view(&b), t(20)).unwrap();
        table.pin(id_a); // `a` is older but pinned, so `b` must go.
        let id_c = table.intern(&ext_view(&c), t(30)).unwrap();
        assert_eq!(id_c.index(), id_b.index());
        assert_eq!(table.find(&ext_view(&b)), None);
        assert_eq!(table.find(&ext_view(&a)), Some(id_a));
        // Stale handle to the evicted entry does not resolve to `b` anymore.
        assert_ne!(table.get(id_b).unwrap().ext_addr(), &b);
    }

    #[test]
    fn full_table_of_pinned_entries_rejects_intern() {
        let mut table = NeighborTable::<1>::new();
        let a = ext(1);
        let id = table.intern(&ext_view(&a), t(10)).unwrap();
        table.pin(id);
        assert_eq!(table.intern(&ext_view(&ext(2)), t(20)), None);
    }

    #[test]
    fn etx_first_sample_and_convergence() {
        let mut table = NeighborTable::<2>::new();
        let a = ext(1);
        let id = table.intern(&ext_view(&a), t(0)).unwrap();

        // First sample sets the estimate directly: 2 attempts -> ETX 2.0.
        table.note_tx_result(id, true, 2, t(1));
        assert_eq!(table.get(id).unwrap().etx_q7(), 2 * ETX_DIVISOR);

        // A long run of alternating 1- and 3-attempt exchanges converges to
        // an ETX of about 2.0.
        for i in 0..200 {
            let attempts = if i % 2 == 0 { 1 } else { 3 };
            table.note_tx_result(id, true, attempts, t(2 + i));
        }
        let etx = table.get(id).unwrap().etx_q7();
        let target = 2 * ETX_DIVISOR;
        assert!(
            etx.abs_diff(target) <= ETX_DIVISOR / 4,
            "etx {etx} not near {target}"
        );

        // Failures push the estimate up via the penalty.
        for i in 0..50 {
            table.note_tx_result(id, false, 4, t(300 + i));
        }
        assert!(table.get(id).unwrap().etx_q7() > 4 * ETX_DIVISOR);
        // Counters moved as expected.
        let n = table.get(id).unwrap();
        assert_eq!(n.tx_acked(), 201);
        assert!(n.tx_attempts() > 200);
    }

    #[test]
    fn rx_and_beacon_notes_update_freshness_and_metric() {
        let mut table = NeighborTable::<2>::new();
        let a = ext(1);
        let id = table.intern(&ext_view(&a), t(0)).unwrap();
        table.note_rx(id, t(5));
        table.note_beacon(id, 3, t(9));
        let n = table.get(id).unwrap();
        assert_eq!(n.last_heard(), t(9));
        assert_eq!(n.rx_count(), 1);
        assert_eq!(n.advertised_join_metric(), 3);
        assert!(n.eb_heard());
    }
}
