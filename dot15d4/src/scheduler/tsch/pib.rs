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
    MAC_TSCH_DRIFT_MIN_INTERVAL_SLOTS, MAC_TSCH_MAX_BE, MAC_TSCH_MAX_DRIFT_PPM,
    MAC_TSCH_MAX_LINKS, MAC_TSCH_MAX_SLOTFRAMES, MAC_TSCH_MIN_BE,
};
use crate::mac::frame::fields::{TschLinkOption, TschTimeslotTimings};
use crate::mac::neighbors::NeighborId;
use crate::scheduler::command::tsch::TschRoleConfig;

use super::log::tsch_trace;

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
    /// the slot-start math uses [`Self::compensation_ns`] so it does not multiply
    /// a rounded value again.
    fn per_slot_ns(&self) -> i32 {
        let (num, den) = self.sums();
        if den == 0 {
            0
        } else {
            (num / den as i64) as i32
        }
    }

    fn compensation_ns(&self, slots_diff: u32) -> i64 {
        let (num, den) = self.sums();
        if den == 0 {
            0
        } else {
            ((slots_diff as i128 * num as i128) / den as i128) as i64
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

    /// Calculate the next ASN when this link will be active.
    pub fn next_asn_for_link(
        &self,
        link: &TschLink,
        current_asn: TschAsn,
    ) -> TschAsn {
        let current_timeslot = self.timeslot(current_asn);
        if current_timeslot < link.timeslot {
            // Link is later in this slotframe cycle
            current_asn + (link.timeslot - current_timeslot) as u64
        } else {
            // Link is at current timeslot or earlier - go to next slotframe cycle
            current_asn + (self.size - current_timeslot + link.timeslot) as u64
        }
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
    /// Sliding-window estimate of the per-slot clock drift against the time
    /// source.
    drift: DriftEstimator<MAC_TSCH_DRIFT_BUFFER_LEN>,
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

    /// Select the next valid link across all slotframes that matches the given
    /// criteria, returning the link with the smallest next ASN strictly after
    /// `current_asn`.
    ///
    /// When multiple links resolve to the same ASN (from different slotframes),
    /// the IEEE 802.15.4-2024 priority rules apply:
    /// - Lower macSlotframeHandle takes precedence over higher handle.
    ///
    /// TX vs RX precedence is handled at the task level since this method
    /// operates on a single filter (TX-only or RX-only).
    pub fn select_next_link(
        &self,
        current_asn: TschAsn,
        filter: impl Fn(&TschLink) -> bool,
    ) -> Option<(&TschLink, TschAsn)> {
        let mut best: Option<(&TschLink, u64)> = None;

        for link in self.links.iter() {
            if !filter(link) {
                continue;
            }

            let slotframe = match self.get_slotframe(link.slotframe_handle) {
                Some(sf) => sf,
                None => continue,
            };

            let next_asn = slotframe.next_asn_for_link(link, current_asn);

            // Skip if the computed ASN is not strictly in the future.
            if next_asn <= current_asn {
                continue;
            }

            let is_better = match best {
                None => true,
                Some((best_link, best_asn)) => {
                    if next_asn < best_asn {
                        true
                    } else if next_asn == best_asn {
                        // Tie-break per IEEE 802.15.4-2024:
                        // lower slotframe handle takes precedence.
                        let best_handle = best_link.slotframe_handle;
                        link.slotframe_handle < best_handle
                    } else {
                        false
                    }
                }
            };

            if is_better {
                best = Some((link, next_asn));
            }
        }

        best
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

    /// Update base time and ASN after executing an operation.
    pub fn update_timing(&mut self, asn: TschAsn) {
        let timeslot_length_us = self.timeslot_length_us();
        self.last_base_time = self.expected_slot_start(asn, timeslot_length_us);
        self.asn = asn;
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
        let slots_diff = asn.saturating_sub(self.asn) as u32;
        let nominal_ns = (timeslot_length_us.saturating_mul(1000) as i64).saturating_mul(slots_diff as i64);
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        let total_ns = nominal_ns - self.drift.compensation_ns(slots_diff);
        #[cfg(feature = "no-tsch-drift-adjustment")]
        let total_ns = nominal_ns;
        self.last_base_time + NsDuration::nanos(total_ns.max(0) as u64)
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
        if self.last_rx.is_some() {
            let predicted = self.expected_slot_start(asn, self.timeslot_length_us());
            let err_ns = signed_delta_ns(predicted, timeslot_start);
            dot15d4_util::info!("tsch sync asn={=u64} src=frame err_ns={=i64}", asn, err_ns);
        }

        self.asn = asn;
        self.last_base_time = timeslot_start;
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.update_drift(asn, timeslot_start);
        self.last_rx = Some((timeslot_start, asn));

        tsch_trace!(
            "tsch resync src=frame asn={} drift_ns_per_slot={}",
            asn,
            self.drift_ns()
        );
    }

    pub fn reset_time_base(&mut self, asn: TschAsn, instant: NsInstant) {
        let tx_offset_us = self.timeslot_timings.tx_offset() as u64;
        self.asn = asn;
        self.last_base_time = instant - NsDuration::micros(tx_offset_us);
        self.last_rx = None;
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.drift.clear();

        tsch_trace!("tsch time-base reset asn={}", asn);
    }

    /// Update clock drift relative to timesource with an instant derived from
    /// communication with timesource neighbor (i.e. when receiving a frame
    /// or using Time Sync information in Time Correction IE).
    #[cfg(not(feature = "no-tsch-drift-adjustment"))]
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

    /// Acknowledgment-based synchronization.
    #[cfg(not(feature = "no-tsch-ack-sync"))]
    pub fn sync_ack(&mut self, timesync_us: i16) {
        let asn = self.asn;
        #[cfg(feature = "sync-error-log")]
        dot15d4_util::info!(
            "tsch sync asn={=u64} src=ack err_ns={=i64}",
            asn,
            (timesync_us as i64) * 1000
        );
        if timesync_us < 0 {
            self.last_base_time -= NsDuration::micros(timesync_us.unsigned_abs() as u64);
        } else {
            self.last_base_time += NsDuration::micros(timesync_us.unsigned_abs() as u64);
        }
        #[cfg(not(feature = "no-tsch-drift-adjustment"))]
        self.update_drift(asn, self.last_base_time);
        self.last_rx = Some((self.last_base_time, asn));

        tsch_trace!(
            "tsch resync src=ack asn={} correction_us={} drift_ns_per_slot={}",
            asn,
            timesync_us,
            self.drift_ns()
        );
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
            slotframes: Vec::new(),
            links: Vec::new(),
            timeslot_timings: TschTimeslotTimings::default(),
            asn: 0,
            last_base_time: NsInstant::from_ticks(0),
            last_rx: None,
            drift: DriftEstimator::new(),
        }
    }
}
