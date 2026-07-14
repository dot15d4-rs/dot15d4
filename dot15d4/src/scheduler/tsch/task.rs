//! TSCH scheduler state types.
//!
//! Defines the state machine for TSCH (Time Slotted Channel Hopping) operation.

use core::{cell::Cell, marker::PhantomData};

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{RadioFrame, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::{fields::TschLinkOption, mpdu::MpduFrame};
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};
use heapless::Vec;

use crate::{
    constants::MAC_TSCH_MAX_PENDING_OPERATIONS,
    mac::neighbors::NeighborId,
    scheduler::{
        tsch::TschLinkType, SchedulerContext, SchedulerResponse, SchedulerTransmissionResult,
    },
};

use super::beacon::EnhancedBeaconBuilder;
use super::log::{ch, tsch_debug, tsch_info, tsch_trace, tsch_warn};
use super::pib::{TschAsn, TschLink};

/// Guard time before a timeslot starts (microseconds).
pub const TIMESLOT_GUARD_TIME_US: u64 = 2000;

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
        response_token: ResponseToken,
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
        response_token: Option<ResponseToken>,
        shared: bool,
        backoff: TschCsmaBackoff,
    },
    /// TX slot - TxStarted received, waiting for Sent/Nack.
    Transmitting {
        response_token: Option<ResponseToken>,
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

    /// Convert period to nanoseconds.
    pub fn period_ns(&self) -> u64 {
        self.period_secs as u64 * 1_000_000_000
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
        .parse_ies::<RadioDriverImpl>()
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
    /// Timestamp of last beacon transmission.
    pub last_beacon_time: Option<NsInstant>,
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
            last_beacon_time: None,
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
        self.last_beacon_time = None;
        self.state = TschState::Idle;

        if self.beacon_config.enabled {
            self.init_beacon_frame(context);
            self.queue_next_beacon(context);
        }

        let current_time = context.timer.now();
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

    /// Get the deadline of the next pending operation.
    pub fn peek_deadline(&self, context: &SchedulerContext<RadioDriverImpl>) -> NsInstant {
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
    /// Returns the frame and response token unconsumed when no suitable link
    /// exists so that the caller can fail the request properly (dropping a
    /// `ResponseToken` is not allowed).
    pub fn next_tx_operation(
        &self,
        current_asn: TschAsn,
        mpdu: MpduFrame,
        response_token: ResponseToken,
        backoff: TschCsmaBackoff,
        dst: Option<NeighborId>,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Result<TschOperation, (MpduFrame, ResponseToken)> {
        match self.select_tx_link(current_asn, &backoff, dst, context) {
            Some((asn, slotframe_handle, channel, shared)) => Ok(TschOperation::TxSlot {
                mpdu,
                asn,
                slotframe_handle,
                channel,
                cca: context.pib.tsch.tsch_cca,
                shared,
                backoff,
                response_token,
            }),
            None => Err((mpdu, response_token)),
        }
    }

    pub fn next_advertising_operation(
        &self,
        current_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> Option<TschOperation> {
        match context.pib.tsch.select_next_link(current_asn, |l| {
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
            if let TschOperation::TxSlot {
                mpdu,
                response_token,
                ..
            } = operation
            {
                self.fail_tx(mpdu, response_token, context);
            }
            return false;
        }
        match operation {
            TschOperation::TxSlot {
                mpdu,
                asn,
                backoff,
                response_token,
                ..
            } => self.queue_tx_with_backoff(response_token, mpdu, asn, backoff, context),
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

    /// Queue a fresh transmission, starting the TSCH CSMA-CA state from the
    /// PIB defaults.
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
        let backoff = TschCsmaBackoff::new(context.pib.tsch.tsch_min_be);
        self.queue_tx_with_backoff(token, mpdu, from_asn, backoff, context)
    }

    /// Queue a transmission carrying an explicit TSCH CSMA-CA backoff state
    /// (fresh request, retransmission, or displaced operation).
    ///
    /// When no suitable link exists, the request is failed with
    /// `ChannelAccessFailure` so that the response token is consumed and the
    /// frame is returned to the upper layer.
    pub fn queue_tx_with_backoff(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        from_asn: TschAsn,
        backoff: TschCsmaBackoff,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        let dst = Self::frame_dst_neighbor(&mpdu, context);
        match self.next_tx_operation(from_asn, mpdu, token, backoff, dst, context) {
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
            Err((mpdu, token)) => {
                tsch_info!(
                    "tsch tx drop from-asn={} nb={} result=channel-access-failure cause=no-tx-link",
                    from_asn,
                    backoff.nb
                );
                self.fail_tx(mpdu, token, context);
                false
            }
        }
    }

    /// Fail a transmission request outside of a state machine transition:
    /// consume the response token with a `ChannelAccessFailure` carrying the
    /// frame back to the upper layer.
    fn fail_tx(
        &self,
        mpdu: MpduFrame,
        token: ResponseToken,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) {
        let frame = mpdu.into_radio_frame::<RadioDriverImpl>();
        context.request_receiver.received(
            token,
            SchedulerResponse::Transmission(SchedulerTransmissionResult::ChannelAccessFailure(
                frame,
            )),
        );
    }

    /// Terminate TSCH scheduler by releasing resources owned by the scheduler,
    /// i.e. buffers for beacons and RX frames, and by failing pending TX
    /// operations so that their response tokens are consumed and the frames
    /// are returned to the upper layer.
    pub fn terminate(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        while let Some(op) = self.pending_operations.pop() {
            if let TschOperation::TxSlot {
                mpdu,
                response_token,
                ..
            } = op
            {
                self.fail_tx(mpdu, response_token, context);
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
    /// Check if beacon should be scheduled based on time elapsed since last beacon.
    pub fn should_schedule_beacon(&self, current_time: NsInstant) -> bool {
        if !self.beacon_config.enabled {
            return false;
        }

        // Check if there's already a pending advertisement
        if self
            .pending_operations
            .iter()
            .any(|op| op.is_advertisement())
        {
            return false;
        }

        match self.last_beacon_time {
            Some(last_time) => {
                let elapsed = current_time - last_time;
                elapsed.to_nanos() >= self.beacon_config.period_ns()
            }
            None => true, // No beacon sent yet, schedule one
        }
    }

    /// Schedule the next beacon advertisement.
    pub fn queue_next_beacon(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) -> bool {
        use dot15d4_driver::timer::RadioTimerApi;

        if !self.beacon_config.enabled {
            return false;
        }

        let current_time = context.timer.now();

        // Calculate the minimum ASN for the next beacon
        let min_beacon_time = match self.last_beacon_time {
            Some(last_time) => {
                let mut next =
                    last_time + NsDuration::from_ticks(self.beacon_config.period_ns());
                // Random +/- jitter around the period, so nodes with the same
                // period do not collide repeatedly in shared advertising
                // cells.
                let jitter_ns = self.beacon_config.jitter_ms as u64 * 1_000_000;
                if jitter_ns > 0 && self.beacon_config.period_ns() > jitter_ns {
                    let span_ms = 2 * self.beacon_config.jitter_ms as u64 + 1;
                    let offset_ms = context.rng.next_u32() as u64 % span_ms;
                    next = next - NsDuration::from_ticks(jitter_ns)
                        + NsDuration::from_ticks(offset_ms * 1_000_000);
                }
                next
            }
            None => current_time, // First beacon can be sent immediately
        };

        // Convert min_beacon_time to ASN
        let min_asn = context.pib.tsch.asn_at(min_beacon_time);
        let current_asn = context.pib.tsch.asn_at(current_time);

        let search_from_asn = core::cmp::max(current_asn, min_asn);

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

    /// Mark beacon as sent and record the time.
    pub fn on_beacon_sent(&mut self, instant: NsInstant) {
        self.last_beacon_time = Some(instant);
    }
}
