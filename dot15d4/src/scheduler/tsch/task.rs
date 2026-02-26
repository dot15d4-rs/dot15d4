//! TSCH scheduler state types.
//!
//! Defines the state machine for TSCH (Time Slotted Channel Hopping) operation.

use core::{cell::Cell, marker::PhantomData};

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{FrameType, RadioFrame, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant, RadioTimerApi},
};
use dot15d4_frame::{fields::TschLinkOption, mpdu::MpduFrame};
use dot15d4_util::{allocator::IntoBuffer, sync::ResponseToken};
use heapless::Vec;

use crate::{
    constants::MAC_TSCH_MAX_PENDING_OPERATIONS,
    scheduler::{tsch::TschLinkType, SchedulerContext},
};

#[cfg(feature = "tsch-coordinator")]
use super::beacon::EnhancedBeaconBuilder;
use super::pib::TschAsn;

/// Guard time before a timeslot starts (microseconds).
pub const TIMESLOT_GUARD_TIME_US: u64 = 2000;

/// Infinite deadline - used when no pending operations.
pub const INFINITE_DEADLINE: NsInstant = NsInstant::from_ticks(u64::MAX);

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
    },
    /// TX slot - TxStarted received, waiting for Sent/Nack.
    Transmitting {
        response_token: Option<ResponseToken>,
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
    /// Whether beacon transmission is enabled.
    pub enabled: bool,
}

impl Default for BeaconConfig {
    fn default() -> Self {
        Self {
            period_secs: 10,
            enabled: false,
        }
    }
}

impl BeaconConfig {
    pub fn new(period_secs: u32) -> Self {
        Self {
            period_secs,
            enabled: true,
        }
    }

    /// Create a disabled beacon configuration.
    pub fn disabled() -> Self {
        Self {
            period_secs: 0,
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
    /// Whether this device is coordinator.
    pub is_coordinator: bool,
    /// Beacon frame (for coordinator).
    #[cfg(feature = "tsch-coordinator")]
    pub beacon_mpdu: Cell<Option<MpduFrame>>,
    /// Beacon builder.
    #[cfg(feature = "tsch-coordinator")]
    pub beacon_builder: EnhancedBeaconBuilder<'static, RadioDriverImpl>,
    /// Beacon configuration (period and enabled state).
    pub beacon_config: BeaconConfig,
    /// Timestamp of last beacon transmission.
    #[cfg(feature = "tsch-coordinator")]
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
            is_coordinator: false,
            #[cfg(feature = "tsch-coordinator")]
            beacon_mpdu: Cell::new(None),
            #[cfg(feature = "tsch-coordinator")]
            beacon_builder: EnhancedBeaconBuilder::new(),
            beacon_config: BeaconConfig::disabled(),
            #[cfg(feature = "tsch-coordinator")]
            last_beacon_time: None,
            _marker: PhantomData,
        }
    }

    /// Initialize as device with observed ASN and timestamp.
    pub fn init_device(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        self.is_coordinator = false;
        self.beacon_config = BeaconConfig::disabled();
        #[cfg(feature = "tsch-coordinator")]
        {
            self.last_beacon_time = None;
        }
        self.state = TschState::Idle;

        let current_time = context.timer.now();
        let current_asn = context.pib.tsch.asn_at(current_time);

        if !self.queue_next_rx(current_asn, context) {
            // TODO: no rx slot available, fallback to CSMA ?
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

    pub fn next_tx_operation(
        &self,
        current_asn: TschAsn,
        mpdu: MpduFrame,
        response_token: ResponseToken,
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
                Some(TschOperation::TxSlot {
                    mpdu,
                    asn,
                    slotframe_handle,
                    channel,
                    cca: false, // TODO
                    response_token,
                })
            }
            None => None,
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
        match operation {
            TschOperation::TxSlot {
                mpdu,
                asn,
                response_token,
                ..
            } => self.queue_tx(response_token, mpdu, asn, context),
            TschOperation::RxSlot { asn, .. } => self.queue_next_rx(asn, context),
            #[cfg(feature = "tsch-coordinator")]
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
            Some(op) => match self.push_operation(op) {
                Ok(None) => true,
                Ok(Some(op)) => self.reschedule_operation(op, context),
                Err(op) => self.reschedule_operation(op, context),
            },
            None => false,
        }
    }

    pub fn queue_tx(
        &mut self,
        token: ResponseToken,
        mpdu: MpduFrame,
        from_asn: TschAsn,
        context: &mut SchedulerContext<RadioDriverImpl>,
    ) -> bool {
        // Find appropriate link based on frame type
        let operation = match mpdu.frame_control().frame_type() {
            FrameType::Beacon => self.next_advertising_operation(from_asn, context),
            _ => self.next_tx_operation(from_asn, mpdu, token, context),
        };

        match operation {
            Some(op) => match self.push_operation(op) {
                Ok(None) => true,
                Ok(Some(op)) => self.reschedule_operation(op, context),
                Err(op) => self.reschedule_operation(op, context),
            },
            None => false,
        }
    }

    /// Terminate TSCH scheduler by releasing resources owned by the scheduler,
    /// i.e. buffers for beacons and RX frames.
    pub fn terminate(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) {
        if let Some(radio_frame) = self.take_rx_frame() {
            unsafe {
                context
                    .buffer_allocator
                    .deallocate_buffer(radio_frame.into_buffer())
            };
        }
        #[cfg(feature = "tsch-coordinator")]
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
// Coordinator features
// ========================================================================
#[cfg(feature = "tsch-coordinator")]
impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Initialize as coordinator with network start time and beacon period.
    pub fn init_coordinator(
        &mut self,
        context: &mut SchedulerContext<RadioDriverImpl>,
        beacon_period_secs: Option<u32>,
    ) {
        self.is_coordinator = true;
        self.beacon_config = if let Some(beacon_period_secs) = beacon_period_secs {
            BeaconConfig::new(beacon_period_secs)
        } else {
            BeaconConfig::disabled()
        };
        self.last_beacon_time = None;
        self.state = TschState::Idle;
        self.init_beacon_frame(context);
        self.queue_next_beacon(context);

        let current_time = context.timer.now();
        let current_asn = context.pib.tsch.asn_at(current_time);
        self.queue_next_rx(current_asn, context);
    }

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
    /// Set beacon period (coordinator only).
    pub fn set_beacon_period(&mut self, period_secs: u32) {
        if self.is_coordinator {
            self.beacon_config.period_secs = period_secs;
            self.beacon_config.enabled = period_secs > 0;
        }
    }

    /// Enable or disable beacon transmission.
    pub fn set_beacon_enabled(&mut self, enabled: bool) {
        if self.is_coordinator {
            self.beacon_config.enabled = enabled;
        }
    }
    /// Check if beacon should be scheduled based on time elapsed since last beacon.
    pub fn should_schedule_beacon(&self, current_time: NsInstant) -> bool {
        if !self.is_coordinator || !self.beacon_config.enabled {
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
    #[cfg(feature = "tsch-coordinator")]
    pub fn queue_next_beacon(&mut self, context: &mut SchedulerContext<RadioDriverImpl>) -> bool {
        use dot15d4_driver::timer::RadioTimerApi;

        if !self.is_coordinator || !self.beacon_config.enabled {
            return false;
        }

        let current_time = context.timer.now();

        // Calculate the minimum ASN for the next beacon
        let min_beacon_time = match self.last_beacon_time {
            Some(last_time) => last_time + NsDuration::from_ticks(self.beacon_config.period_ns()),
            None => current_time, // First beacon can be sent immediately
        };

        // Convert min_beacon_time to ASN
        let min_asn = context.pib.tsch.asn_at(min_beacon_time);
        let current_asn = context.pib.tsch.asn_at(current_time);

        let search_from_asn = core::cmp::max(current_asn, min_asn);

        match self.next_advertising_operation(search_from_asn, context) {
            Some(op) => match self.push_operation(op) {
                Ok(None) => true,
                Ok(Some(op)) => self.reschedule_operation(op, context),
                Err(op) => self.reschedule_operation(op, context),
            },
            None => false,
        }
    }

    /// Mark beacon as sent and record the time.
    pub fn on_beacon_sent(&mut self, instant: NsInstant) {
        self.last_beacon_time = Some(instant);
    }
}
