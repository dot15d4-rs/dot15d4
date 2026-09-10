//! TSCH PIB (PAN Information Base) attributes.
//!
//! This module contains TSCH-specific PIB attributes as defined in
//! IEEE 802.15.4-2024 Section 10.3.11.

#![allow(dead_code)]

use dot15d4_driver::radio::config::Channel;
use dot15d4_driver::timer::{NsDuration, NsInstant};
use heapless::Vec;

use crate::constants::{
    MAC_DISCONNECT_TIME, MAC_JOIN_METRIC, MAC_TSCH_DRIFT_BUFFER_LEN,
    MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS, MAC_TSCH_GUARD_COMMIT_MARGIN, MAC_TSCH_GUARD_COMMIT_SYNCS,
    MAC_TSCH_GUARD_FLOOR_US, MAC_TSCH_GUARD_SAFE_US, MAC_TSCH_KEEP_ALIVE_PERIOD, MAC_TSCH_MAX_BE,
    MAC_TSCH_MAX_DRIFT_PPM, MAC_TSCH_MAX_LINKS, MAC_TSCH_MAX_SLOTFRAMES, MAC_TSCH_MIN_BE,
};
use crate::driver::AckWindow;
use crate::mac::frame::fields::{TschLinkOption, TschTimeslotTimings};
use crate::mac::neighbors::NeighborId;
use crate::scheduler::command::tsch::TschRoleConfig;

use super::log::{tsch_info, tsch_trace};

/// Type alias for Absolute Slot Number.
pub type TschAsn = u64;

#[derive(Clone, Copy)]
struct DriftSample {
    dev_ns: i64,
    delta_asn: u32,
}

struct DriftEstimator<const N: usize> {
    buf: [Option<DriftSample>; N],
    head: usize,
}

impl<const N: usize> DriftEstimator<N> {
    const fn new() -> Self {
        Self {
            buf: [None; N],
            head: 0,
        }
    }

    fn clear(&mut self) {
        self.buf = [None; N];
        self.head = 0;
    }

    fn push(&mut self, dev_ns: i64, delta_asn: u32, timeslot_length_us: u64) {
        if delta_asn == 0 {
            return;
        }
        // max ns per slot = slot_ns * ppm / 1e6 = slot_us * ppm / 1e3
        let max_per_slot_ns =
            (timeslot_length_us.saturating_mul(MAC_TSCH_MAX_DRIFT_PPM as u64) / 1_000) as i64;
        let max = (delta_asn as i64).saturating_mul(max_per_slot_ns.max(1));
        let dev_ns = dev_ns.clamp(-max, max);
        self.buf[self.head] = Some(DriftSample { dev_ns, delta_asn });
        self.head = (self.head + 1) % N;
    }

    /// Whether the window holds `N` samples, i.e. the estimate has converged.
    fn is_full(&self) -> bool {
        self.buf.iter().all(Option::is_some)
    }

    /// Returns `(sum(dev_ns), sum(delta_asn))` over the filled part of the window.
    fn sums(&self) -> (i64, u64) {
        let mut num = 0i64;
        let mut den = 0u64;
        for sample in self.buf.iter().flatten() {
            num += sample.dev_ns;
            den += sample.delta_asn as u64;
        }
        (num, den)
    }

    /// Mean per-slot drift in nanoseconds (0 with no samples). Used for logging;
    /// the slot-start math uses [`Self::compensation_q32`] so it does not multiply
    /// a rounded value again.
    fn per_slot_ns(&self) -> i32 {
        let (num, den) = self.sums();
        if den == 0 {
            0
        } else {
            (num / den as i64) as i32
        }
    }

    fn compensation_q32(&self, slots_diff: u32) -> i128 {
        let (num, den) = self.sums();
        if den == 0 {
            0
        } else {
            ((slots_diff as i128 * num as i128) << 32) / den as i128
        }
    }

    /// Reset the window to a single sample that means `ns` per slot. Used to set a
    /// known drift (tests, or a direct override).
    fn set_per_slot(&mut self, ns: i32, timeslot_length_us: u64) {
        self.clear();
        self.push(ns as i64, 1, timeslot_length_us);
    }
}

/// `a - b` in nanoseconds, signed (fugit durations are unsigned).
fn signed_delta_ns(a: NsInstant, b: NsInstant) -> i64 {
    if a >= b {
        (a - b).to_nanos() as i64
    } else {
        -((b - a).to_nanos() as i64)
    }
}

/// Schedule operation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    InvalidSlotframe,
    InvalidTimeslot,
    InvalidChannelOffset,
    CapacityExceeded,
    HandleDuplicate,
}

/// Type of link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TschLinkType {
    /// Advertisement/beacon link
    Advertising,
    /// Normal data link
    Normal,
}

/// A TSCH link is a pairwise assignment of a directed communication between
/// devices for a given slotframe, in a given timeslot on a given channel offset.
///
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.3
#[derive(Debug, Clone)]
pub struct TschLink {
    /// Slotframe identifier of the slotframe to which the link is associated.
    pub slotframe_handle: u8,
    /// Associated timeslot in the slotframe.
    pub timeslot: u16,
    /// Associated channel offset for the given timeslot for the link.
    pub channel_offset: u16,
    /// Link communication option (TX, RX, Shared, etc.).
    pub link_options: TschLinkOption,
    /// Type of link (normal or advertising).
    pub link_type: TschLinkType,
    /// Handle into the MAC neighbor table for the neighbor assigned to the
    /// link. `None` if the link is not dedicated to one neighbor.
    pub neighbor: Option<NeighborId>,
    /// Whether this link shall be advertised in Enhanced beacon frames
    /// using the TSCH Slotframe and Link IE.
    pub link_advertise: bool,
}

impl Default for TschLink {
    fn default() -> Self {
        Self {
            slotframe_handle: 0,
            timeslot: 0,
            channel_offset: 0,
            link_options: TschLinkOption::Shared,
            link_type: TschLinkType::Normal,
            neighbor: None,
            link_advertise: true,
        }
    }
}

/// A TSCH slotframe - a collection of timeslots repeating in time.
///
/// This representation follows specification described in
/// IEEE802.15.4-2024, Section 10.3.11.2
#[derive(Debug)]
pub struct TschSlotframe {
    /// Slotframe Identifier.
    pub handle: u8,
    /// The number of timeslots in a given slotframe.
    pub size: u16,
}

impl TschSlotframe {
    /// Create a new slotframe.
    pub fn new(handle: u8, size: u16) -> Self {
        Self { handle, size }
    }

    /// Return the timeslot within the slotframe for a given ASN.
    pub fn timeslot(&self, asn: TschAsn) -> u16 {
        (asn % self.size as u64) as u16
    }

    /// First ASN at or after `from_asn` at which this link is active.
    ///
    /// Inclusive in `from_asn`: when `from_asn` already falls on the link's
    /// timeslot it is returned unchanged. This is the primitive a
    /// deadline-driven caller needs ("the first opportunity from this ASN
    /// on"); callers that want the occurrence strictly after a slot - the one
    /// they are currently executing, typically - ask for `from_asn + 1`.
    pub fn link_asn_at_or_after(&self, link: &TschLink, from_asn: TschAsn) -> TschAsn {
        // In u64 throughout: `size + timeslot` overflows u16 for large
        // slotframes.
        let size = self.size as u64;
        let current = self.timeslot(from_asn) as u64;
        let target = link.timeslot as u64;
        let ahead = if current <= target {
            // Later in this slotframe cycle, or right here.
            target - current
        } else {
            // Already past it: next slotframe cycle.
            size - current + target
        };
        from_asn + ahead
    }

    /// Get the slotframe size.
    pub fn size(&self) -> u16 {
        self.size
    }

    /// Get the slotframe handle.
    pub fn handle(&self) -> u8 {
        self.handle
    }
}

/// TSCH-specific PIB attributes.
///
/// IEEE802.15.4-2024 Section 10.3.11
pub struct TschPib {
    /// The minimum value of the backoff exponent (BE) in the TSCH-CA algorithm.
    pub tsch_min_be: u8,
    /// The maximum value of the BE in the TSCH-CA algorithm.
    pub tsch_max_be: u8,
    /// Whether CCA is performed before a TSCH transmission (macTschCca, set via
    /// MLME-TSCH-MODE.request). In TSCH, CCA only guards against non-802.15.4
    /// interferers; intra-network contenders are slot-aligned and cannot be
    /// detected by it. Defaults to off.
    pub tsch_cca: bool,
    /// Time (in timeslots) to send Disassociate frames before disconnecting.
    pub disconnect_time: u16,
    /// Metric used when selecting and joining a TSCH network.
    pub join_metric: u16,
    /// Role of the current (or upcoming) TSCH session: time root or
    /// synchronizing device, plus beacon period and jitter. Written by the
    /// UseTsch handler before the mode switch, read at TSCH entry.
    pub role: TschRoleConfig,
    /// The neighbor this node synchronizes to (its parent in the time
    /// distribution tree). `None` on the time root and before joining.
    /// Frame- and acknowledgment-based corrections only apply to exchanges
    /// with this neighbor.
    pub time_source: Option<NeighborId>,
    /// Keep-alive period in timeslots (macKeepAlivePeriod, IEEE
    /// 802.15.4-2024, 10.3.10.8). Ignored while no time source is set.
    pub keep_alive_period: u16,
    /// Slotframe table (macSlotframeTable).
    pub slotframes: Vec<TschSlotframe, MAC_TSCH_MAX_SLOTFRAMES>,
    /// Link table (macLinkTable).
    pub links: Vec<TschLink, MAC_TSCH_MAX_LINKS>,
    /// Timeslot timing template (macTimeslotTemplate).
    pub timeslot_timings: TschTimeslotTimings,
    /// Current Absolute Slot Number.
    pub asn: TschAsn,
    /// Timestamp of last known timeslot start.
    pub last_base_time: NsInstant,
    /// RMARKER and ASN of last received frame
    pub last_rx: Option<(NsInstant, TschAsn)>,
    /// Whether a timing observation from the time source was made since the
    /// last time-base reset. Kept separate from `last_rx` (which only tracks
    /// applied corrections) so the sync-error log still works when the
    /// corrections are compiled out.
    #[cfg(feature = "sync-error-log")]
    had_sync_obs: bool,
    /// Sliding-window estimate of the per-slot clock drift against the time
    /// source.
    drift: DriftEstimator<MAC_TSCH_DRIFT_BUFFER_LEN>,
    /// Sub-nanosecond drift compensation not yet applied to `last_base_time`,
    /// as a Q32 fraction of a nanosecond (invariant: `< 2^32`). Carried across
    /// [`Self::update_timing`] re-anchors so that chained whole-nanosecond
    /// floors stay exactly equal to one flooring over the whole span; without
    /// the carry, each re-anchor discards up to 1 ns of compensation in a
    /// drift-sign-fixed direction, accumulating tens of nanoseconds of phase
    /// bias between resyncs. Reset whenever the anchor is replaced by an
    /// actual measurement.
    comp_rem_q32: u64,
    /// Whether the advertised guard time has been adopted. Until then, an
    /// advertised guard below [`MAC_TSCH_GUARD_FLOOR_US`] is replaced by
    /// [`MAC_TSCH_GUARD_SAFE_US`] when listening. One-way latch, cleared only
    /// by [`Self::reset_time_base`].
    guard_committed: bool,
    /// Resyncs observed with a converged drift estimate since the last
    /// time-base reset.
    guard_syncs: u8,
    /// Largest absolute synchronization error observed over those resyncs.
    guard_max_err_ns: u32,
}

impl TschPib {
    /// Create a new TSCH PIB with default values.
    pub fn new() -> Self {
        Default::default()
    }

    /// Get the timeslot length in microseconds.
    pub fn timeslot_length_us(&self) -> u64 {
        self.timeslot_timings.timeslot_length() as u64
    }

    /// Acknowledgment window of the current template (`macTsRxAckDelay`,
    /// `macTsAckWait`).
    pub fn ack_window(&self) -> AckWindow {
        AckWindow {
            delay: NsDuration::micros(self.timeslot_timings.rx_ack_delay() as u64),
            width: NsDuration::micros(self.timeslot_timings.ack_wait() as u64),
        }
    }

    /// Acknowledgment delay of the current template (`macTsTxAckDelay`).
    pub fn ack_delay(&self) -> NsDuration {
        NsDuration::micros(self.timeslot_timings.tx_ack_delay() as u64)
    }

    /// Create a slotframe and add it to the schedule.
    pub fn create_slotframe(&mut self, handle: u8, size: u16) -> Result<u16, ScheduleError> {
        // Check for duplicate handle
        if self.slotframes.iter().any(|sf| sf.handle == handle) {
            return Err(ScheduleError::HandleDuplicate);
        }

        if self
            .slotframes
            .push(TschSlotframe::new(handle, size))
            .is_err()
        {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.slotframes.len() as u16 - 1)
        }
    }

    /// Get a slotframe by its handle.
    pub fn get_slotframe(&self, handle: u8) -> Option<&TschSlotframe> {
        self.slotframes.iter().find(|sf| sf.handle == handle)
    }

    /// Get a slotframe by index.
    pub fn get_slotframe_by_index(&self, index: usize) -> Option<&TschSlotframe> {
        self.slotframes.get(index)
    }

    /// Add a link to the schedule.
    pub fn add_link(&mut self, link: TschLink) -> Result<u16, ScheduleError> {
        // Validate slotframe exists
        let slotframe = self
            .get_slotframe(link.slotframe_handle)
            .ok_or(ScheduleError::InvalidSlotframe)?;

        // Validate timeslot
        if link.timeslot >= slotframe.size {
            return Err(ScheduleError::InvalidTimeslot);
        }

        // TODO: Validate channel offset from parent PIB

        // Re-adding a cell updates it in place; appending would fill the table on rejoin.
        if let Some(index) = self.links.iter().position(|existing| {
            existing.slotframe_handle == link.slotframe_handle
                && existing.timeslot == link.timeslot
                && existing.channel_offset == link.channel_offset
        }) {
            self.links[index] = link;
            return Ok(index as u16);
        }

        // Add link
        if self.links.push(link).is_err() {
            Err(ScheduleError::CapacityExceeded)
        } else {
            Ok(self.links.len() as u16 - 1)
        }
    }

    /// Sort links by (slotframe_handle, timeslot) for efficient iteration
    /// during link selection. Lower slotframe handles are processed first,
    /// which naturally respects IEEE 802.15.4-2024 priority rules.
    pub fn sort_links(&mut self) {
        let len = self.links.len();
        for i in 1..len {
            let mut j = i;
            while j > 0 {
                let should_swap = {
                    let a = &self.links[j - 1];
                    let b = &self.links[j];
                    (a.slotframe_handle, a.timeslot) > (b.slotframe_handle, b.timeslot)
                };
                if should_swap {
                    self.links.swap(j - 1, j);
                    j -= 1;
                } else {
                    break;
                }
            }
        }
    }

    /// Select the matching link across all slotframes with the smallest ASN
    /// **at or after** `from_asn`.
    ///
    /// Inclusive in `from_asn`, so a caller holding a deadline ("the beacon is
    /// due at this ASN") gets the cell falling exactly on it rather than the
    /// following occurrence a whole slotframe later. Callers that mean "after
    /// the slot I am executing" use [`Self::select_next_link`].
    ///
    /// When multiple links resolve to the same ASN (from different slotframes),
    /// the IEEE 802.15.4-2024 priority rules apply:
    /// - Lower macSlotframeHandle takes precedence over higher handle.
    ///
    /// TX vs RX precedence is handled at the task level since this method
    /// operates on a single filter (TX-only or RX-only).
    pub fn select_link_from(
        &self,
        from_asn: TschAsn,
        filter: impl Fn(&TschLink) -> bool,
    ) -> Option<(&TschLink, TschAsn)> {
        let mut best: Option<(&TschLink, TschAsn)> = None;

        for link in self.links.iter() {
            if !filter(link) {
                continue;
            }

            let Some(slotframe) = self.get_slotframe(link.slotframe_handle) else {
                continue;
            };

            let asn = slotframe.link_asn_at_or_after(link, from_asn);

            // Earliest ASN wins; on a tie the lower macSlotframeHandle does
            // (IEEE 802.15.4-2024).
            let is_better = match best {
                None => true,
                Some((best_link, best_asn)) => {
                    (asn, link.slotframe_handle) < (best_asn, best_link.slotframe_handle)
                }
            };

            if is_better {
                best = Some((link, asn));
            }
        }

        best
    }

    /// Select the matching link with the smallest ASN **strictly after**
    /// `current_asn`, i.e. the next opportunity once the slot at `current_asn`
    /// is spoken for.
    pub fn select_next_link(
        &self,
        current_asn: TschAsn,
        filter: impl Fn(&TschLink) -> bool,
    ) -> Option<(&TschLink, TschAsn)> {
        self.select_link_from(current_asn.saturating_add(1), filter)
    }

    /// Select the Nth valid link (skipping `skip` occurrences) across all
    /// slotframes that matches the given criteria.
    ///
    /// When `skip == 0`, this behaves identically to [`select_next_link`].
    /// When `skip > 0`, the first `skip` matching link occurrences are
    /// skipped and the (skip+1)th is returned. This is used by the TSCH
    /// CSMA-CA retransmission backoff algorithm to skip a random number of
    /// shared link slots before retransmitting.
    pub fn select_nth_link(
        &self,
        current_asn: TschAsn,
        skip: u16,
        filter: impl Fn(&TschLink) -> bool,
    ) -> Option<(&TschLink, TschAsn)> {
        let mut search_asn = current_asn;
        let mut result = None;
        for _ in 0..=skip {
            result = self.select_next_link(search_asn, &filter);
            match result {
                Some((_, asn)) => search_asn = asn,
                None => return None,
            }
        }
        result
    }

    /// Calculate the channel for a given ASN and link.
    pub fn channel_for_link(
        &self,
        asn: TschAsn,
        link: &TschLink,
        hopping_sequence: &[Channel],
    ) -> Channel {
        if hopping_sequence.is_empty() {
            dot15d4_util::warn!("TSCH hopping sequence is empty; falling back to the default channel.");
            return Channel::default();
        }
        let channel_index =
            ((asn + link.channel_offset as u64) % hopping_sequence.len() as u64) as usize;
        hopping_sequence[channel_index]
    }

    pub fn update_timing(&mut self, asn: TschAsn) {
        let timeslot_length_us = self.timeslot_length_us();
        let (base, rem) = self.slot_start_and_rem(asn, timeslot_length_us);
        self.last_base_time = base;
        self.comp_rem_q32 = rem;
        self.asn = asn;
    }

    pub fn rx_window_us(&self, min_rx_offset_us: u64) -> (u64, u64) {
        let advertised = self.timeslot_timings.rx_wait() as u64;
        if self.guard_committed || advertised >= MAC_TSCH_GUARD_FLOOR_US as u64 {
            return (self.timeslot_timings.rx_offset() as u64, advertised);
        }
        let tx_offset = self.timeslot_timings.tx_offset() as u64;
        let guard = (MAC_TSCH_GUARD_SAFE_US as u64)
            .min(2 * tx_offset.saturating_sub(min_rx_offset_us))
            .max(advertised);
        (tx_offset.saturating_sub(guard / 2), guard)
    }

    fn observe_sync_error(&mut self, err_ns: i64) {
        if self.guard_committed || !self.drift.is_full() {
            return;
        }
        self.guard_syncs = self.guard_syncs.saturating_add(1);
        self.guard_max_err_ns = self
            .guard_max_err_ns
            .max(err_ns.unsigned_abs().min(u32::MAX as u64) as u32);

        tsch_trace!("tsch observe sync error");

        let half_guard_ns = (self.timeslot_timings.rx_wait() as u64) * 1000 / 2;
        if self.guard_syncs >= MAC_TSCH_GUARD_COMMIT_SYNCS
            && (self.guard_max_err_ns as u64) * MAC_TSCH_GUARD_COMMIT_MARGIN as u64 <= half_guard_ns
        {
            self.guard_committed = true;
            tsch_info!(
                "tsch guard commit rx_wait_us={} max_err_ns={}",
                self.timeslot_timings.rx_wait(),
                self.guard_max_err_ns
            );
        }
    }

    pub fn relax_guard(&mut self) {
        if self.guard_committed {
            tsch_info!(
                "tsch guard relax rx_wait_us={} max_err_ns={}",
                self.timeslot_timings.rx_wait(),
                self.guard_max_err_ns
            );
        }
        self.guard_committed = false;
        self.guard_syncs = 0;
        self.guard_max_err_ns = 0;
    }

    /// Whether the advertised guard time is currently adopted.
    #[cfg(test)]
    pub(crate) fn is_guard_committed(&self) -> bool {
        self.guard_committed
    }

    /// Adopt the advertised guard time directly, without going through the
    /// resync evidence [`Self::observe_sync_error`] normally requires. Test
    /// setup for the relaxation path.
    #[cfg(test)]
    pub(crate) fn force_guard_committed(&mut self) {
        self.guard_committed = true;
    }

    /// Current per-slot drift estimate in nanoseconds (mean of the window). A
    /// positive value means the local clock runs a bit slow against the time
    /// source.
    pub fn drift_ns(&self) -> i32 {
        self.drift.per_slot_ns()
    }

    /// Set the drift estimate to a known per-slot value and drop the current
    /// window. Same as recording one `ns`-per-slot sample.
    pub fn set_drift_ns(&mut self, ns: i32) {
        let timeslot_length_us = self.timeslot_length_us();
        self.drift.set_per_slot(ns, timeslot_length_us);
    }

    /// Calculate expected slot start time for given ASN.
    ///
    /// The drift correction is applied as the exact fraction over the whole
    /// `asn - self.asn` span instead of a rounded per-slot duration multiplied
    /// out, so the error does not build up with distance from the anchor.
    pub fn expected_slot_start(&self, asn: TschAsn, timeslot_length_us: u64) -> NsInstant {
        self.slot_start_and_rem(asn, timeslot_length_us).0
    }

    /// Drift-compensated start of `asn`'s timeslot, plus the subnanosecond
    /// compensation remainder (Q32, `< 2^32`) left after flooring the applied
    /// correction to whole nanoseconds.
    fn slot_start_and_rem(&self, asn: TschAsn, timeslot_length_us: u64) -> (NsInstant, u64) {
        let slots_diff = asn.saturating_sub(self.asn) as u32;
        let nominal_ns = (timeslot_length_us.saturating_mul(1000) as i64).saturating_mul(slots_diff as i64);
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        let (comp_ns, rem) = {
            // Exact compensation over the span period in Q32 ns, including the
            // fraction carried over from earlier re-anchors. `>> 32` on the
            // signed value floors, so the new remainder is always in
            // `[0, 2^32)`.
            let comp_q32 = self.drift.compensation_q32(slots_diff) + self.comp_rem_q32 as i128;
            let comp_ns = (comp_q32 >> 32) as i64;
            let rem = (comp_q32 - ((comp_ns as i128) << 32)) as u64;
            (comp_ns, rem)
        };
        #[cfg(feature = "no-tsch-drift-adjustment")]
        let (comp_ns, rem) = (0i64, 0u64);
        let total_ns = nominal_ns - comp_ns;
        (
            self.last_base_time + NsDuration::nanos(total_ns.max(0) as u64),
            rem,
        )
    }

    /// Calculate current ASN from timestamp.
    pub fn asn_at(&self, instant: NsInstant) -> TschAsn {
        let last_asn = self.asn;
        let timeslot_length_us = self.timeslot_length_us();
        if instant <= self.last_base_time {
            return last_asn;
        }
        let elapsed = instant - self.last_base_time;
        last_asn + elapsed.to_micros() / timeslot_length_us
    }

    /// Synchronize an ASN to an observed RMARKER.
    pub fn sync_asn(&mut self, asn: TschAsn, instant: NsInstant) {
        let tx_offset_us = self.timeslot_timings.tx_offset() as u64;
        let timeslot_start = instant - NsDuration::micros(tx_offset_us);

        #[cfg(feature = "sync-error-log")]
        {
            if self.last_rx.is_some() || self.had_sync_obs {
                let predicted = self.expected_slot_start(asn, self.timeslot_length_us());
                let err_ns = signed_delta_ns(predicted, timeslot_start);
                dot15d4_util::info!("tsch sync asn={=u64} src=frame err_ns={=i64}", asn, err_ns);
            }
            self.had_sync_obs = true;
        }

        #[cfg(not(feature = "no-tsch-frame-sync"))]
        {
            if self.last_rx.is_some() {
                let predicted = self.expected_slot_start(asn, self.timeslot_length_us());
                self.observe_sync_error(signed_delta_ns(predicted, timeslot_start));
            }

            self.asn = asn;
            self.last_base_time = timeslot_start;
            // The anchor is now a fresh measurement; any sub-ns compensation
            // left over from the previous model chain no longer applies.
            self.comp_rem_q32 = 0;
            #[cfg(not(feature = "no-tsch-drift-adjustment"))]
            self.update_drift(asn, timeslot_start);
            self.last_rx = Some((timeslot_start, asn));

            tsch_trace!(
                "tsch resync src=frame asn={} drift_ns_per_slot={}",
                asn,
                self.drift_ns()
            );
        }
        #[cfg(all(feature = "no-tsch-frame-sync", not(feature = "sync-error-log")))]
        let _ = (asn, timeslot_start);
    }

    /// Log the synchronization error observed on a frame from a neighbor that
    /// is *not* the time source, applying nothing.
    #[cfg(feature = "sync-error-log")]
    pub fn log_peer_sync_error(&self, instant: NsInstant, peer: &[u8; 8]) {
        let tx_offset_us = self.timeslot_timings.tx_offset() as u64;
        let timeslot_start = instant - NsDuration::micros(tx_offset_us);
        let predicted = self.expected_slot_start(self.asn, self.timeslot_length_us());
        let err_ns = signed_delta_ns(predicted, timeslot_start);
        // The peer goes out big-endian so the 16 hex digits read exactly like
        // the address the node prints at boot.
        dot15d4_util::info!(
            "tsch sync asn={=u64} src=peer err_ns={=i64} peer={=u64:016x}",
            self.asn,
            err_ns,
            u64::from_be_bytes(*peer)
        );
    }

    pub fn reset_time_base(&mut self, asn: TschAsn, instant: NsInstant) {
        let tx_offset_us = self.timeslot_timings.tx_offset() as u64;
        self.asn = asn;
        self.last_base_time = instant - NsDuration::micros(tx_offset_us);
        self.comp_rem_q32 = 0;
        self.last_rx = None;
        self.relax_guard();
        #[cfg(feature = "sync-error-log")]
        {
            self.had_sync_obs = false;
        }
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.drift.clear();

        tsch_trace!("tsch time-base reset asn={}", asn);
    }

    /// Update clock drift relative to timesource from a measured RMARKER of a
    /// frame received from the time source. 
    #[cfg(not(feature = "no-tsch-drift-adjustment"))]
    #[cfg_attr(feature = "no-tsch-frame-sync", allow(dead_code))]
    fn update_drift(&mut self, asn: TschAsn, timeslot_start: NsInstant) {
        if let Some((last_slot_start, last_rx_asn)) = self.last_rx {
            let delta_asn = asn.saturating_sub(last_rx_asn);

            // Below the minimum baseline the sample is dominated by per-event
            // phase noise, not frequency.
            if delta_asn < MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS as u64 {
                return;
            }

            let elapsed_ns = signed_delta_ns(timeslot_start, last_slot_start);
            let expected_ns =
                (delta_asn.saturating_mul(self.timeslot_length_us()).saturating_mul(1000)) as i64;
            let dev_ns = expected_ns - elapsed_ns;
            let timeslot_length_us = self.timeslot_length_us();
            self.drift.push(dev_ns, delta_asn as u32, timeslot_length_us);
        }
    }

    pub fn sync_ack(&mut self, timesync_us: i16) {
        #[cfg(feature = "sync-error-log")]
        {
            dot15d4_util::info!(
                "tsch sync asn={=u64} src=ack err_ns={=i64}",
                self.asn,
                (timesync_us as i64) * 1000
            );
            self.had_sync_obs = true;
        }
        #[cfg(not(feature = "no-tsch-ack-sync"))]
        {
            self.observe_sync_error((timesync_us as i64) * 1000);

            // The Time Correction IE is quantized to whole microseconds, so the
            // smallest correction it can express, +-1 us, stands for any true
            // error in +-[0.5, 1.5) us. We decide to ignore 0us quantized error 
            // to limit the noise it will inject in our drift estimator since 
            // frame-based correction quantization is much finer.
            if timesync_us == 0 {
                tsch_trace!(
                    "tsch resync src=ack asn={} correction_us=0 ignored=deadband",
                    self.asn
                );
                return;
            }

            let asn = self.asn;
            if timesync_us < 0 {
                self.last_base_time -= NsDuration::micros(timesync_us.unsigned_abs() as u64);
            } else {
                self.last_base_time += NsDuration::micros(timesync_us.unsigned_abs() as u64);
            }

            tsch_trace!(
                "tsch resync src=ack asn={} correction_us={} drift_ns_per_slot={}",
                asn,
                timesync_us,
                self.drift_ns()
            );
        }
        #[cfg(all(feature = "no-tsch-ack-sync", not(feature = "sync-error-log")))]
        let _ = timesync_us;
    }

    /// Get iterator over slotframes.
    pub fn slotframes(&self) -> impl Iterator<Item = &TschSlotframe> {
        self.slotframes.iter()
    }

    /// Get iterator over links.
    pub fn links(&self) -> impl Iterator<Item = &TschLink> {
        self.links.iter()
    }

    /// Get slotframe info for beacon IE generation.
    /// Returns iterator of (handle, size) tuples.
    pub fn slotframe_info(&self) -> impl Iterator<Item = (u8, u16)> + '_ {
        self.slotframes.iter().map(|sf| (sf.handle, sf.size))
    }

    /// Get number of slotframes.
    pub fn num_slotframes(&self) -> usize {
        self.slotframes.len()
    }

    /// Get number of links.
    pub fn num_links(&self) -> usize {
        self.links.len()
    }
}

impl Default for TschPib {
    fn default() -> Self {
        Self {
            tsch_min_be: MAC_TSCH_MIN_BE,
            tsch_max_be: MAC_TSCH_MAX_BE,
            tsch_cca: false,
            disconnect_time: MAC_DISCONNECT_TIME,
            join_metric: MAC_JOIN_METRIC,
            role: TschRoleConfig::default(),
            time_source: None,
            keep_alive_period: MAC_TSCH_KEEP_ALIVE_PERIOD,
            slotframes: Vec::new(),
            links: Vec::new(),
            timeslot_timings: TschTimeslotTimings::default(),
            asn: 0,
            last_base_time: NsInstant::from_ticks(0),
            last_rx: None,
            #[cfg(feature = "sync-error-log")]
            had_sync_obs: false,
            guard_committed: false,
            guard_syncs: 0,
            guard_max_err_ns: 0,
            drift: DriftEstimator::new(),
            comp_rem_q32: 0,
        }
    }
}
