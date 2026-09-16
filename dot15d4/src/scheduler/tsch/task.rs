//! TSCH scheduler state types.
//!
//! Defines the state machine for TSCH (Time Slotted Channel Hopping) operation.

use core::{cell::Cell, marker::PhantomData};

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{
            Address, ExtendedAddress, FrameType, FrameVersion, RadioFrame, RadioFrameUnsized,
        },
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::{
    fields::TschLinkOption,
    mpdu::{MpduFrame, DATA_FRAME_REPR},
};
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};
use heapless::Vec;

use crate::{
    constants::{MAC_TSCH_GUARD_RELAX_MISSES, MAC_TSCH_MAX_PENDING_OPERATIONS},
    mac::neighbors::NeighborId,
    scheduler::{
        tsch::TschLinkType, SchedulerContext, SchedulerResponse, SchedulerTransmissionResult,
    },
};

use super::beacon::EnhancedBeaconBuilder;
use super::log::{ch, tsch_debug, tsch_info, tsch_trace, tsch_warn};
use super::pib::{TschAsn, TschLink};

pub const TIMESLOT_GUARD_TIME_US: u64 = 1000;

/// Infinite deadline - used when no pending operations.
pub const INFINITE_DEADLINE: NsInstant = NsInstant::from_ticks(u64::MAX);

/// TSCH CSMA-CA retransmission backoff state.
///
/// Tracks the backoff exponent (BE), number of backoffs (NB), the number of
/// shared links left to skip before the next attempt, and whether the frame
/// has entered the retransmission procedure. NB also counts the channel-access
/// failures that precede the first missing acknowledgment.
#[derive(Debug, Clone, Copy)]
pub struct TschCsmaBackoff {
    /// Backoff exponent (BE).
    pub be: u8,
    /// Number of backoffs (NB).
    pub nb: u8,
    /// Number of shared links remaining to skip before retransmission.
    pub backoff_remaining: u16,
    /// Whether at least one transmission attempt went unacknowledged, i.e.
    /// the frame has entered the retransmission procedure. Channel-access
    /// (CCA) failures before that point retry without any backoff.
    pub retransmitting: bool,
}

impl TschCsmaBackoff {
    /// Create initial backoff state from PIB parameters.
    pub fn new(min_be: u8) -> Self {
        Self {
            be: min_be,
            nb: 0,
            backoff_remaining: 0,
            retransmitting: false,
        }
    }
}

/// Who a transmission belongs to, and therefore what happens with its
/// outcome.
///
/// A MAC-originated frame carries the response token that returns the frame
/// and the result to the upper layer. Beacons and keep-alives are generated
/// inside the scheduler: nobody upstream waits for them, so their outcome is
/// consumed internally (the beacon buffer is recycled, the keep-alive buffer
/// is released).
///
/// [`TschOperation::TxSlot`] only ever carries `Mac` or `KeepAlive`; beacons
/// travel as [`TschOperation::AdvertisementSlot`] and take the `Beacon`
/// origin when they enter the TX states.
#[derive(Debug)]
pub enum TxOrigin {
    /// Frame submitted by the MAC layer on behalf of an upper layer.
    Mac(ResponseToken),
    /// Scheduler-internal Enhanced Beacon.
    Beacon,
    /// Scheduler-internal keep-alive frame toward the time source
    /// (IEEE 802.15.4-2024, 10.3.10.8).
    KeepAlive,
}

/// TSCH operation to be executed during a timeslot.
#[derive(Debug)]
pub enum TschOperation {
    /// Transmission slot (data).
    TxSlot {
        mpdu: MpduFrame,
        asn: TschAsn,
        slotframe_handle: u8,
        channel: Channel,
        cca: bool,
        /// Whether this TX is on a shared link.
        shared: bool,
        /// TSCH CSMA-CA backoff state for retransmissions on shared links.
        backoff: TschCsmaBackoff,
        origin: TxOrigin,
    },
    /// Reception slot.
    RxSlot {
        asn: TschAsn,
        slotframe_handle: u8,
        channel: Channel,
    },
    /// Advertisement (beacon) slot - internally managed.
    AdvertisementSlot {
        asn: TschAsn,
        slotframe_handle: u8,
        channel: Channel,
    },
    /// No operation.
    Idle,
}

impl TschOperation {
    /// Get the ASN for this operation.
    pub fn asn(&self) -> Option<TschAsn> {
        match self {
            TschOperation::TxSlot { asn, .. } => Some(*asn),
            TschOperation::RxSlot { asn, .. } => Some(*asn),
            TschOperation::AdvertisementSlot { asn, .. } => Some(*asn),
            TschOperation::Idle => None,
        }
    }

    /// Get the slotframe handle for this operation.
    pub fn slotframe_handle(&self) -> Option<u8> {
        match self {
            TschOperation::TxSlot {
                slotframe_handle, ..
            } => Some(*slotframe_handle),
            TschOperation::RxSlot {
                slotframe_handle, ..
            } => Some(*slotframe_handle),
            TschOperation::AdvertisementSlot {
                slotframe_handle, ..
            } => Some(*slotframe_handle),
            TschOperation::Idle => None,
        }
    }

    /// Check if this is a TX-type operation (TX or Advertisement).
    pub fn is_tx(&self) -> bool {
        matches!(
            self,
            TschOperation::TxSlot { .. } | TschOperation::AdvertisementSlot { .. }
        )
    }

    /// Check if this is an advertisement operation.
    pub fn is_advertisement(&self) -> bool {
        matches!(self, TschOperation::AdvertisementSlot { .. })
    }

    /// Check if this is a scheduler-internal keep-alive transmission.
    pub fn is_keep_alive(&self) -> bool {
        matches!(
            self,
            TschOperation::TxSlot {
                origin: TxOrigin::KeepAlive,
                ..
            }
        )
    }
}

fn operation_wins_over(new_op: &TschOperation, existing: &TschOperation) -> bool {
    let new_is_tx = new_op.is_tx();
    let existing_is_tx = existing.is_tx();

    if new_is_tx && !existing_is_tx {
        return true; // TX > RX
    }
    if !new_is_tx && existing_is_tx {
        return false; // RX < TX
    }
    // Same direction: lower slotframe handle wins.
    match (new_op.slotframe_handle(), existing.slotframe_handle()) {
        (Some(new_h), Some(ex_h)) => new_h < ex_h,
        _ => false,
    }
}

/// TSCH scheduler state.
#[derive(Debug)]
pub enum TschState {
    /// Idle - waiting for next deadline or scheduler request.
    Idle,
    /// TX slot - driver request sent, waiting for TxStarted.
    WaitingForTxStart {
        origin: TxOrigin,
        shared: bool,
        backoff: TschCsmaBackoff,
    },
    /// TX slot - TxStarted received, waiting for Sent/Nack.
    Transmitting {
        origin: TxOrigin,
        shared: bool,
        backoff: TschCsmaBackoff,
    },
    /// RX slot - driver request sent, waiting for FrameStarted/RxWindowEnded.
    Listening,
    /// RX slot - FrameStarted received, waiting for Received/CrcError.
    Receiving,
    /// Placeholder
    Placeholder,
}

/// Configuration for periodic beacon advertisement.
#[derive(Debug, Clone, Copy)]
pub struct BeaconConfig {
    /// Period between beacon transmissions in seconds.
    /// The scheduler will find the next advertising link opportunity
    /// that is at least this many seconds from the previous beacon.
    pub period_secs: u32,
    /// Random `+/-` jitter applied to each period, in milliseconds.
    pub jitter_ms: u16,
    /// Whether beacon transmission is enabled.
    pub enabled: bool,
}

impl Default for BeaconConfig {
    fn default() -> Self {
        Self {
            period_secs: 10,
            jitter_ms: 0,
            enabled: false,
        }
    }
}

impl BeaconConfig {
    pub fn new(period_secs: u32) -> Self {
        Self {
            period_secs,
            jitter_ms: 0,
            enabled: true,
        }
    }

    /// Apply a random `+/-` jitter to each period.
    pub fn with_jitter_ms(mut self, jitter_ms: u16) -> Self {
        self.jitter_ms = jitter_ms;
        self
    }

    /// Create a disabled beacon configuration.
    pub fn disabled() -> Self {
        Self {
            period_secs: 0,
            jitter_ms: 0,
            enabled: false,
        }
    }

    /// The period in timeslots, which is the unit the schedule is expressed
    /// in. Never zero, so a period shorter than one timeslot still advances
    /// instead of targeting the same ASN forever.
    pub fn period_slots(&self, timeslot_length_us: u64) -> u64 {
        (self.period_secs as u64 * 1_000_000 / timeslot_length_us.max(1)).max(1)
    }

    /// The `+/-` jitter half-width in timeslots.
    pub fn jitter_slots(&self, timeslot_length_us: u64) -> u64 {
        self.jitter_ms as u64 * 1_000 / timeslot_length_us.max(1)
    }
}

/// Update the TSCH Synchronization IE in an MpduFrame with the given ASN
/// and join metric.
///
/// If the frame does not contain IEs or does not contain a TSCH
/// Synchronization IE, the frame is returned unchanged.
pub fn update_tsch_sync_ie<RadioDriverImpl: DriverConfig>(
    mpdu: MpduFrame,
    asn: TschAsn,
    join_metric: u8,
) -> MpduFrame {
    if !mpdu.frame_control().information_elements_present() {
        return mpdu;
    }

    let mut parser = mpdu
        .into_parser()
        .parse_addressing()
        .expect("well-formed MPDU: addressing parse failed")
        .parse_security()
        .and_then(|r| r.parse_ies::<RadioDriverImpl>())
        .expect("well-formed MPDU: IE parse failed");

    if let Some(mut ies) = parser.try_ies_fields_mut() {
        if let Some(mut tsch_sync) = ies.tsch_sync_mut() {
            tsch_sync.set_asn(asn);
            tsch_sync.set_join_metric(join_metric);
        }
    }

    parser.into_mpdu_frame()
}

/// Complete TSCH scheduler state.
pub struct TschTask<RadioDriverImpl: DriverConfig> {
    /// Current operating state.
    pub state: TschState,
    /// Queue of pending operations (sorted by ASN, earliest last for pop).
    pub pending_operations: Vec<TschOperation, MAC_TSCH_MAX_PENDING_OPERATIONS>,
    /// Pre-allocated frame for inbound frames.
    pub rx_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,
    /// Whether this node is the network time root: the one clock that never
    /// adjusts itself. Everyone else applies frame- and acknowledgment-based
    /// corrections toward its time source. Set from the session role at
    /// entry.
    pub is_time_root: bool,
    /// Beacon frame (present when beaconing is enabled).
    pub beacon_mpdu: Cell<Option<MpduFrame>>,
    /// Beacon builder.
    pub beacon_builder: EnhancedBeaconBuilder<'static, RadioDriverImpl>,
    /// Beacon configuration (period and enabled state).
    pub beacon_config: BeaconConfig,
    /// ASN the next Enhanced Beacon is due at: a point of the absolute beacon
    /// grid (multiples of the period from ASN 0). `None` until the first
    /// beacon of the session, which goes out at the first advertising cell
    /// that comes up rather than waiting for the grid, so that a node
    /// starting a PAN does not stay silent for a whole period.
    pub next_beacon_asn: Option<TschAsn>,
    /// Instant at which the next keep-alive toward the time source is due.
    pub next_keep_alive: NsInstant,
    /// Keep-alive periods that have elapsed without any synchronizing exchange
    /// with the time source.
    pub missed_keep_alives: u8,
    /// Marker for radio driver implementation
    _marker: PhantomData<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Create new TSCH state.
    ///
    /// Takes context as parameter to allocate RX frame, but does NOT store it.
    pub fn new(context: &mut SchedulerContext<RadioDriverImpl>) -> Self {
        let rx_frame = context.allocate_frame();
        Self {
            state: TschState::Idle,
            pending_operations: Vec::new(),
            rx_frame: Cell::new(Some(rx_frame)),
            is_time_root: false,
            beacon_mpdu: Cell::new(None),
            beacon_builder: EnhancedBeaconBuilder::new(),
            beacon_config: BeaconConfig::disabled(),
            next_beacon_asn: None,
            next_keep_alive: NsInstant::from_ticks(0),
            missed_keep_alives: 0,
            _marker: PhantomData,
        }
    }

    /// Initialize the TSCH session from the role stored in the PIB
    /// (written by the UseTsch handler before the mode switch).
    ///
    /// Time root or not, and beaconing or not, are independent: a joined
    /// forwarding node beacons while still synchronizing to its time source.
    pub fn init(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        let role = context.pib.tsch.role;
        self.is_time_root = role.is_time_root;
        self.beacon_config = match role.beacon_period_secs {
            Some(period_secs) => {
                BeaconConfig::new(period_secs).with_jitter_ms(role.beacon_jitter_ms)
            }
            None => BeaconConfig::disabled(),
        };
        self.next_beacon_asn = None;
        self.state = TschState::Idle;

        if self.beacon_config.enabled {
            self.init_beacon_frame(context);
            self.queue_next_beacon(context);
        }

        let current_time = context.timer.now();

        // Arm the keep-alive one period from entry: the node either just
        // synchronized to a beacon (join) or is the time root (in which case
        // the deadline is never consulted). Either way the session starts
        // synchronized, so the miss counter starts at zero.
        self.missed_keep_alives = 0;
        self.touch_keep_alive(current_time, context);

        let current_asn = context.pib.tsch.asn_at(current_time);

        if !self.queue_next_rx(current_asn, context) {
            // TODO: no rx slot available, fallback to CSMA ?
            tsch_warn!("tsch: no RX link in the schedule; node will never listen");
        }
    }

    /// Pop the next operation from the queue (earliest ASN).
    pub fn pop_operation(&mut self) -> TschOperation {
        self.pending_operations.pop().unwrap_or(TschOperation::Idle)
    }

    /// Push an operation to the queue, maintaining ASN order.
    /// Operations are stored with earliest ASN at the end (for efficient pop).
    pub fn push_operation(
        &mut self,
        op: TschOperation,
    ) -> Result<Option<TschOperation>, TschOperation> {
        if let Some(op_asn) = op.asn() {
            // Check if there is already an operation at the same ASN.
            if let Some(conflict_idx) = self
                .pending_operations
                .iter()
                .position(|existing| existing.asn() == Some(op_asn))
            {
                if operation_wins_over(&op, &self.pending_operations[conflict_idx]) {
                    // Replace the lower-priority operation and return it.
                    let displaced =
                        core::mem::replace(&mut self.pending_operations[conflict_idx], op);
                    self.sort_pending();

                    return Ok(Some(displaced));
                } else {
                    // Existing operation has equal or higher priority; reject.
                    return Err(op);
                }
            }

            // No conflict, sorted insert maintaining descending ASN order
            // (earliest ASN at the end for efficient pop).
            if self.pending_operations.is_full() {
                return Err(op);
            }

            // Push to end then re-sort.
            let _res = self.pending_operations.push(op);
            self.sort_pending();
            Ok(None)
        } else {
            Err(op)
        }
    }

    /// Sort pending operations in descending ASN order (earliest last for pop).
    fn sort_pending(&mut self) {
        // Simple insertion sort, the list is small (MAC_TSCH_MAX_PENDING_OPERATIONS).
        let len = self.pending_operations.len();
        for i in 1..len {
            let mut j = i;
            while j > 0 {
                let a_asn = self.pending_operations[j - 1].asn().unwrap_or(0);
                let b_asn = self.pending_operations[j].asn().unwrap_or(0);
                if b_asn > a_asn {
                    self.pending_operations.swap(j - 1, j);
                    j -= 1;
                } else {
                    break;
                }
            }
        }
    }

    /// Get the next instant the scheduler has to wake up at: the deadline of
    /// the next pending operation, advanced to the keep-alive deadline when
    /// that comes first. Waking at the keep-alive deadline lets the idle
    /// handler queue the keep-alive transmission even when no operation is
    /// due (or pending) for a long time.
    pub fn peek_deadline(&self, context: &SchedulerContext<RadioDriverImpl>) -> NsInstant {
        let op_deadline = self.op_deadline(context);
        match self.keep_alive_deadline(context) {
            Some(keep_alive) => op_deadline.min(keep_alive),
            None => op_deadline,
        }
    }

    /// Get the deadline of the next pending operation.
    pub fn op_deadline(&self, context: &SchedulerContext<RadioDriverImpl>) -> NsInstant {
        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        match self.pending_operations.last() {
            Some(op) => {
                if let Some(asn) = op.asn() {
                    let instant = context
                        .pib
                        .tsch
                        .expected_slot_start(asn, timeslot_length_us);
                    instant - NsDuration::micros(TIMESLOT_GUARD_TIME_US)
                } else {
                    INFINITE_DEADLINE
                }
            }
            None => INFINITE_DEADLINE,
        }
    }

    /// Check if in idle state.
    pub fn is_idle(&self) -> bool {
        matches!(self.state, TschState::Idle)
    }

    /// Check if there are pending operations.
    pub fn has_pending_operations(&self) -> bool {
        !self.pending_operations.is_empty()
    }

    /// Get count of pending operations.
    pub fn pending_operation_count(&self) -> usize {
        self.pending_operations.len()
    }

    /// Take the rx_frame, leaving None.
    pub fn take_rx_frame(&self) -> Option<RadioFrame<RadioFrameUnsized>> {
        self.rx_frame.take()
    }

    /// Put back an rx_frame.
    pub fn put_rx_frame(&self, frame: RadioFrame<RadioFrameUnsized>) {
        self.rx_frame.set(Some(frame));
    }

    /// Get mutable reference to rx_frame option.
    pub fn rx_frame_mut(&mut self) -> &mut Option<RadioFrame<RadioFrameUnsized>> {
        self.rx_frame.get_mut()
    }

    /// Select the next RX link across all slotframes.
    pub fn next_rx_operation(
        &self,
        current_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Option<TschOperation> {
        match context
            .pib
            .tsch
            .select_next_link(current_asn, |l| l.link_options.contains(TschLinkOption::Rx))
        {
            Some((link, asn)) => {
                // Calculate channel
                let channel =
                    context
                        .pib
                        .tsch
                        .channel_for_link(asn, link, &context.pib.hopping_sequence);
                let slotframe_handle = link.slotframe_handle;
                Some(TschOperation::RxSlot {
                    asn,
                    channel,
                    slotframe_handle,
                })
            }
            None => None,
        }
    }

    /// Select the TX link for a frame, returning plain link data
    /// `(asn, slotframe_handle, channel, shared)`.
    fn select_tx_link(
        &self,
        current_asn: TschAsn,
        backoff: &TschCsmaBackoff,
        dst: Option<NeighborId>,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> Option<(TschAsn, u8, Channel, bool)> {
        let to_destination = |l: &TschLink| l.neighbor.is_none() || l.neighbor == dst;
        let selected = if backoff.nb > 0 {
            let shared = context.pib.tsch.select_nth_link(
                current_asn,
                backoff.backoff_remaining,
                |l| {
                    l.link_options.contains(TschLinkOption::Tx)
                        && l.link_options.contains(TschLinkOption::Shared)
                        && to_destination(l)
                },
            );
            let dedicated = context.pib.tsch.select_next_link(current_asn, |l| {
                l.link_options.contains(TschLinkOption::Tx)
                    && !l.link_options.contains(TschLinkOption::Shared)
                    && to_destination(l)
            });
            match (shared, dedicated) {
                // Dedicated link preempts the remaining shared-link wait on a
                // tie or when it occurs no later than the skipped shared slot.
                (Some((sl, sa)), Some((dl, da))) => {
                    if da <= sa {
                        Some((dl, da))
                    } else {
                        Some((sl, sa))
                    }
                }
                (Some(link), None) => Some(link),
                (None, Some(link)) => Some(link),
                (None, None) => None,
            }
        } else {
            context.pib.tsch.select_next_link(current_asn, |l| {
                l.link_options.contains(TschLinkOption::Tx) && to_destination(l)
            })
        };

        selected.map(|(link, asn)| {
            let channel =
                context
                    .pib
                    .tsch
                    .channel_for_link(asn, link, &context.pib.hopping_sequence);
            let shared = link.link_options.contains(TschLinkOption::Shared);
            (asn, link.slotframe_handle, channel, shared)
        })
    }

    /// Resolve the destination address of a frame to a neighbor table entry.
    ///
    /// Returns `None` when the destination does not identify a single
    /// neighbor: broadcast, absent address, a short address with no known
    /// mapping, or a device not (yet) in the neighbor table.
    fn frame_dst_neighbor(
        mpdu: &MpduFrame,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> Option<NeighborId> {
        let reader = mpdu.reader();
        let parsed = reader.parse_addressing().ok()?;
        let addressing = parsed.try_addressing_fields().ok()?;
        addressing
            .try_dst_address()
            .and_then(|dst| context.pib.neighbors.find(&dst))
    }

    /// Build the TX operation for a frame at the next suitable link to the
    /// given destination neighbor.
    ///
    /// Returns the frame and origin unconsumed when no suitable link exists
    /// so that the caller can fail the request properly (a MAC origin
    /// carries a `ResponseToken`, which must not be dropped).
    pub fn next_tx_operation(
        &self,
        current_asn: TschAsn,
        mpdu: MpduFrame,
        origin: TxOrigin,
        backoff: TschCsmaBackoff,
        dst: Option<NeighborId>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Result<TschOperation, (MpduFrame, TxOrigin)> {
        match self.select_tx_link(current_asn, &backoff, dst, context) {
            Some((asn, slotframe_handle, channel, shared)) => Ok(TschOperation::TxSlot {
                mpdu,
                asn,
                slotframe_handle,
                channel,
                cca: context.pib.tsch.tsch_cca,
                shared,
                backoff,
                origin,
            }),
            None => Err((mpdu, origin)),
        }
    }

    /// The first advertising cell usable at or after `from_asn`.
    ///
    /// The search is inclusive: `from_asn` is the ASN a beacon is due at, not
    /// a slot already spoken for, so a cell landing exactly on it is the right
    /// one. The caller ([`Self::queue_next_beacon`]) is what keeps `from_asn`
    /// out of the past.
    pub fn next_advertising_operation(
        &self,
        from_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Option<TschOperation> {
        match context.pib.tsch.select_link_from(from_asn, |l| {
            matches!(l.link_type, TschLinkType::Advertising)
                && l.link_options.contains(TschLinkOption::Shared)
        }) {
            Some((link, asn)) => {
                // Calculate channel
                let channel =
                    context
                        .pib
                        .tsch
                        .channel_for_link(asn, link, &context.pib.hopping_sequence);
                let slotframe_handle = link.slotframe_handle;
                Some(TschOperation::AdvertisementSlot {
                    asn,
                    channel,
                    slotframe_handle,
                })
            }
            None => None,
        }
    }

    fn reschedule_operation(
        &mut self,
        operation: TschOperation,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        tsch_debug!(
            "tsch reschedule asn={} tx={} adv={}",
            operation.asn().unwrap_or(0),
            operation.is_tx(),
            operation.is_advertisement()
        );

        if self.pending_operations.is_full() {
            tsch_warn!(
                "tsch queue full: dropping op asn={} tx={}",
                operation.asn().unwrap_or(0),
                operation.is_tx()
            );
            if let TschOperation::TxSlot { mpdu, origin, .. } = operation {
                self.fail_tx(mpdu, origin, context);
            }
            return false;
        }
        match operation {
            TschOperation::TxSlot {
                mpdu,
                asn,
                backoff,
                origin,
                ..
            } => self.queue_tx_with_backoff(origin, mpdu, asn, backoff, context),
            TschOperation::RxSlot { asn, .. } => self.queue_next_rx(asn, context),
            TschOperation::AdvertisementSlot { .. } => self.queue_next_beacon(context),
            _ => unreachable!(),
        }
    }

    pub fn queue_next_rx(
        &mut self,
        from_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        match self.next_rx_operation(from_asn, context) {
            Some(op) => {
                if let TschOperation::RxSlot { asn, channel, .. } = &op {
                    tsch_trace!("tsch rx-link asn={} ch={}", *asn, ch(*channel));
                }
                match self.push_operation(op) {
                    Ok(None) => true,
                    Ok(Some(op)) => self.reschedule_operation(op, context),
                    Err(op) => self.reschedule_operation(op, context),
                }
            }
            None => {
                tsch_trace!("tsch rx-link none from-asn={}", from_asn);
                false
            }
        }
    }

    /// Queue a fresh MAC-originated transmission, starting the TSCH CSMA-CA
    /// state from the PIB defaults.
    ///
    /// Returns `true` if the operation was queued. When no suitable link exists
    /// the request is failed immediately and `false` is returned.
    pub fn queue_tx(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        from_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        // Stamp macDsn here, on the fresh request only: retransmissions go
        // through `queue_tx_with_backoff` and must keep the sequence number
        // of the frame that is already on the air, or the acknowledgment
        // would no longer match.
        let mut mpdu = mpdu;
        let _ = mpdu.try_set_sequence_number(context.pib.next_dsn());

        // macPanId is MAC state, not something the upper layer passes down, so
        // it is stamped here as well. The call is a no-op for frames whose
        // addressing configuration elides the Destination PAN ID.
        let _ = mpdu.try_set_dst_pan_id(&context.pib.pan_id);

        let backoff = TschCsmaBackoff::new(context.pib.tsch.tsch_min_be);
        self.queue_tx_with_backoff(TxOrigin::Mac(token), mpdu, from_asn, backoff, context)
    }

    /// Queue a transmission carrying an explicit TSCH CSMA-CA backoff state
    /// (fresh request, retransmission, or displaced operation).
    ///
    /// When no suitable link exists, the transmission is failed through its
    /// origin: a MAC frame is returned to the upper layer as
    /// `ChannelAccessFailure`, a keep-alive frame is released.
    pub fn queue_tx_with_backoff(
        &mut self,
        origin: TxOrigin,
        mpdu: MpduFrame,
        from_asn: TschAsn,
        backoff: TschCsmaBackoff,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        let dst = Self::frame_dst_neighbor(&mpdu, context);
        match self.next_tx_operation(from_asn, mpdu, origin, backoff, dst, context) {
            Ok(op) => {
                if let TschOperation::TxSlot {
                    asn,
                    slotframe_handle,
                    channel,
                    shared,
                    ..
                } = &op
                {
                    tsch_debug!(
                        "tsch tx-link asn={} sf={} ch={} shared={} nb={} skip-shared={}",
                        *asn,
                        *slotframe_handle,
                        ch(*channel),
                        *shared,
                        backoff.nb,
                        backoff.backoff_remaining
                    );
                }
                match self.push_operation(op) {
                    Ok(None) => true,
                    Ok(Some(op)) => self.reschedule_operation(op, context),
                    Err(op) => self.reschedule_operation(op, context),
                }
            }
            Err((mpdu, origin)) => {
                tsch_info!(
                    "tsch tx drop from-asn={} nb={} result=channel-access-failure cause=no-tx-link",
                    from_asn,
                    backoff.nb
                );
                self.fail_tx(mpdu, origin, context);
                false
            }
        }
    }

    /// Fail a transmission outside of a state machine transition, according
    /// to its origin: a MAC frame consumes its response token with a
    /// `ChannelAccessFailure` carrying the frame back to the upper layer; a
    /// keep-alive frame is simply released (the keep-alive deadline was
    /// already re-armed when it was queued, so it is retried one period
    /// later). Beacons never travel as `TxSlot` operations.
    fn fail_tx(
        &self,
        mpdu: MpduFrame,
        origin: TxOrigin,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) {
        match origin {
            TxOrigin::Mac(token) => {
                let frame = mpdu.into_radio_frame::<RadioDriverImpl>();
                context.request_receiver.received(
                    token,
                    SchedulerResponse::Transmission(
                        SchedulerTransmissionResult::ChannelAccessFailure(frame),
                    ),
                );
            }
            TxOrigin::KeepAlive => {
                // Safety: the keep-alive frame was allocated from this
                // allocator by `build_keep_alive_frame`.
                unsafe {
                    context
                        .buffer_allocator
                        .deallocate_buffer(mpdu.into_buffer())
                };
            }
            TxOrigin::Beacon => unreachable!(),
        }
    }

    /// Terminate TSCH scheduler by releasing resources owned by the scheduler,
    /// i.e. buffers for beacons, keep-alives and RX frames, and by failing
    /// pending MAC TX operations so that their response tokens are consumed
    /// and the frames are returned to the upper layer.
    pub fn terminate(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        while let Some(op) = self.pending_operations.pop() {
            if let TschOperation::TxSlot { mpdu, origin, .. } = op {
                self.fail_tx(mpdu, origin, context);
            }
        }
        if let Some(radio_frame) = self.take_rx_frame() {
            unsafe {
                context
                    .buffer_allocator
                    .deallocate_buffer(radio_frame.into_buffer())
            };
        }
        if let Some(mpdu) = self.beacon_mpdu.take() {
            unsafe {
                context
                    .buffer_allocator
                    .deallocate_buffer(mpdu.into_buffer())
            };
        }
    }
}

// ========================================================================
// Beacon / advertisement features
// ========================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    fn init_beacon_frame(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        let radio_frame = context.allocate_frame();
        let beacon_mpdu = self
            .beacon_builder
            .build_enhanced_beacon(&context.pib, radio_frame);
        if let Some(beacon_frame) = beacon_mpdu {
            self.beacon_mpdu.set(Some(beacon_frame));
        } else {
            panic!("Enhanced beacon could not be initialized");
        }
    }
    /// Set beacon period.
    pub fn set_beacon_period(&mut self, period_secs: u32) {
        self.beacon_config.period_secs = period_secs;
        self.beacon_config.enabled = period_secs > 0;
    }

    /// Enable or disable beacon transmission.
    pub fn set_beacon_enabled(&mut self, enabled: bool) {
        self.beacon_config.enabled = enabled;
    }
    /// Schedule the next Enhanced Beacon.
    ///
    /// The whole cadence lives in the ASN domain. [`Self::next_beacon_asn`] is
    /// the ASN the beacon is *due* at, and the advertising cell is looked up
    /// at or afte it.
    pub fn queue_next_beacon(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) -> bool {
        use dot15d4_driver::timer::RadioTimerApi;

        if !self.beacon_config.enabled {
            return false;
        }

        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        let current_asn = context.pib.tsch.asn_at(context.timer.now());
        // No due ASN yet means the first beacon of the session: send at the
        // first advertising cell that comes up.
        let earliest_asn = current_asn.saturating_add(1);
        let due_asn = self
            .next_beacon_asn
            .unwrap_or(earliest_asn)
            .max(earliest_asn);

        // Random +/- jitter around the due ASN, so nodes sharing an
        // advertising cell do not collide period after period.
        let jitter = self.beacon_config.jitter_slots(timeslot_length_us);
        let search_from_asn = if jitter > 0 {
            let offset = context.rng.next_u32() as u64 % (2 * jitter + 1);
            due_asn
                .saturating_add(offset)
                .saturating_sub(jitter)
                .max(earliest_asn)
        } else {
            due_asn
        };

        match self.next_advertising_operation(search_from_asn, context) {
            Some(op) => {
                if let TschOperation::AdvertisementSlot { asn, channel, .. } = &op {
                    tsch_debug!("tsch adv-link asn={} ch={}", *asn, ch(*channel));
                }
                match self.push_operation(op) {
                    Ok(None) => true,
                    Ok(Some(op)) => self.reschedule_operation(op, context),
                    Err(op) => self.reschedule_operation(op, context),
                }
            }
            None => {
                tsch_warn!("tsch adv-link none from-asn={}", search_from_asn);
                false
            }
        }
    }

    /// Record that a beacon went out at `asn` and set the ASN the next one is
    /// due at: the first point of the beacon grid past it.
    pub fn on_beacon_sent(&mut self, asn: TschAsn, timeslot_length_us: u64) {
        let period = self.beacon_config.period_slots(timeslot_length_us);
        // Jitter can place a beacon before its due ASN, which still belongs to
        // the grid point being served.
        let served = self.next_beacon_asn.unwrap_or(asn).max(asn);
        self.next_beacon_asn = Some((served / period + 1) * period);
    }
}

// ========================================================================
// Time-source keep-alive
// ========================================================================
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// The keep-alive deadline, or `None` while the mechanism is disarmed
    /// (no time source designated - time root or not joined - or
    /// `macKeepAlivePeriod` set to zero).
    pub fn keep_alive_deadline(
        &self,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> Option<NsInstant> {
        (context.pib.tsch.time_source.is_some() && context.pib.tsch.keep_alive_period > 0)
            .then_some(self.next_keep_alive)
    }

    /// Re-arm the keep-alive deadline one period after `now`.
    ///
    /// Called on TSCH entry, on every exchange with the time source (frame
    /// heard from it or ACK received from it, whether or not the matching
    /// clock correction is compiled in), and whenever a keep-alive is queued
    /// (so a keep-alive whose
    /// every retransmission failed is retried one period later, not every
    /// wake-up).
    pub fn touch_keep_alive(
        &mut self,
        now: NsInstant,
        context: &SchedulerContext<RadioDriverImpl>,
    ) {
        let period_us =
            context.pib.tsch.keep_alive_period as u64 * context.pib.tsch.timeslot_length_us();
        self.next_keep_alive = now + NsDuration::micros(period_us);
    }

    /// Record a synchronizing exchange with the time source at `instant`: an
    /// acknowledgment it sent for one of our frames (data or keep-alive), or a
    /// frame heard from it.
    pub fn note_sync(&mut self, instant: NsInstant, context: &SchedulerContext<RadioDriverImpl>) {
        self.missed_keep_alives = 0;
        self.touch_keep_alive(instant, context);
    }

    /// Queue a keep-alive transmission toward the time source when it is
    /// due. Called from the idle handler on every timer wake-up.
    pub fn maybe_queue_keep_alive(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        let now = context.timer.now();
        match self.keep_alive_deadline(context) {
            Some(deadline) if deadline <= now => {}
            _ => return,
        }
        if self.pending_operations.iter().any(|op| op.is_keep_alive()) {
            return;
        }

        // Pace the next keep-alive from now on, whatever becomes of this
        // one; a synchronizing exchange in the meantime pushes it further.
        self.touch_keep_alive(now, context);

        self.missed_keep_alives = self.missed_keep_alives.saturating_add(1);

        if self.missed_keep_alives >= MAC_TSCH_GUARD_RELAX_MISSES {
            context.pib.tsch.relax_guard();
        }

        let Some((mpdu, seq_nr)) = self.build_keep_alive_frame(context) else {
            // No buffer or no (evicted) time source entry: skip this round
            // and retry a period later. With the allocator sized per
            // `MAC_NUM_REQUIRED_BUFFERS` and a pinned time source, this does
            // not happen.
            tsch_warn!("tsch keep-alive skipped: no buffer or time source entry");
            return;
        };

        tsch_info!(
            "tsch keep-alive queue asn={} seq={}",
            context.pib.tsch.asn,
            seq_nr
        );
        let backoff = TschCsmaBackoff::new(context.pib.tsch.tsch_min_be);
        let from_asn = context.pib.tsch.asn_at(now);
        self.queue_tx_with_backoff(TxOrigin::KeepAlive, mpdu, from_asn, backoff, context);
    }

    /// Build the keep-alive frame: an empty Data frame to the time source
    /// with the AR field set (IEEE 802.15.4-2024, 10.3.10.8). 
    /// Returns `None` when no buffer is available or the time source has 
    /// no neighbor table entry.
    ///
    /// The sequence number stamped on the frame is returned along with it, so
    /// callers can report it without a second copy being kept anywhere.
    fn build_keep_alive_frame(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Option<(MpduFrame, u8)> {
        let time_source = context.pib.tsch.time_source?;
        let dst = *context.pib.neighbors.get(time_source)?.ext_addr();

        let repr = DATA_FRAME_REPR.without_ies();
        let size = repr.min_buffer_size::<RadioDriverImpl>(0).ok()?;
        let buffer = context.buffer_allocator.try_allocate_buffer(size).ok()?;
        let mut writer = match repr.into_writer::<RadioDriverImpl>(
            FrameVersion::Ieee802154,
            FrameType::Data,
            0,
            buffer,
        ) {
            Ok(writer) => writer,
            Err(buffer) => {
                // Safety: this is the buffer we just allocated.
                unsafe { context.buffer_allocator.deallocate_buffer(buffer) };
                return None;
            }
        };

        // Keep-alives are Data frames and draw from macDsn like every other
        // transmission. A counter of their own would let one device emit two
        // different frames carrying the same sequence number.
        let seq_nr = context.pib.next_dsn();
        writer.set_sequence_number(seq_nr);
        writer.set_ack_request(true);

        let dst_addr = Address::Extended(ExtendedAddress::new_owned(dst));
        let mut addressing = writer.addressing_fields_mut();
        addressing
            .src_address_mut()
            .set(&context.pib.extended_address);
        addressing.dst_address_mut().set(&dst_addr);
        // Keep-alives are built here rather than queued through `queue_tx()`,
        // so the Destination PAN ID carried by `DATA_FRAME_REPR` is written on
        // this path directly.
        addressing.dst_pan_id_mut().set(&context.pib.pan_id);

        Some((writer.into_mpdu_frame(), seq_nr))
    }
}
