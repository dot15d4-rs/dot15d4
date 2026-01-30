//! TSCH scheduler state types.
//!
//! Defines the state machine for TSCH (Time Slotted Channel Hopping) operation.

use core::cell::Cell;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{RadioFrame, RadioFrameUnsized},
        DriverConfig,
    },
    timer::{NsDuration, NsInstant},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;
use heapless::Vec;

use crate::{constants::MAC_TSCH_MAX_PENDING_OPERATIONS, scheduler::SchedulerContext};

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
        channel: Channel,
        cca: bool,
        response_token: ResponseToken,
    },
    /// Reception slot.
    RxSlot {
        asn: TschAsn,
        channel: Channel,
        response_token: Option<ResponseToken>,
    },
    /// Advertisement (beacon) slot - internally managed.
    AdvertisementSlot { asn: TschAsn, channel: Channel },
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

    /// Check if this is an advertisement operation.
    pub fn is_advertisement(&self) -> bool {
        matches!(self, TschOperation::AdvertisementSlot { .. })
    }
}

/// TSCH scheduler state.
#[derive(Debug)]
pub enum TschState {
    /// Idle - waiting for next deadline or scheduler request.
    Idle { next_deadline: NsInstant },
    /// TX slot - driver request sent, waiting for TxStarted.
    WaitingForTxStart {
        response_token: Option<ResponseToken>,
    },
    /// TX slot - TxStarted received, waiting for Sent/Nack.
    Transmitting {
        response_token: Option<ResponseToken>,
    },
    /// RX slot - driver request sent, waiting for FrameStarted/RxWindowEnded.
    Listening {
        response_token: Option<ResponseToken>,
    },
    /// RX slot - FrameStarted received, waiting for Received/CrcError.
    Receiving {
        response_token: Option<ResponseToken>,
    },
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

/// Complete TSCH scheduler state.
pub struct TschTask<RadioDriverImpl: DriverConfig> {
    /// Current operating state.
    pub state: TschState,
    /// Queue of pending operations (sorted by ASN, earliest last for pop).
    pub pending_operations: Vec<TschOperation, MAC_TSCH_MAX_PENDING_OPERATIONS>,
    /// Timestamp of last known timeslot start.
    pub last_base_time: NsInstant,
    /// Last known Absolute Slot Number.
    pub last_asn: TschAsn,
    /// Pre-allocated frame for inbound frames.
    pub rx_frame: Cell<Option<RadioFrame<RadioFrameUnsized>>>,
    /// Whether this device is coordinator.
    pub is_coordinator: bool,
    /// Beacon frame (for coordinator).
    pub beacon_mpdu: Cell<Option<MpduFrame>>,
    /// Beacon builder.
    pub beacon_builder: EnhancedBeaconBuilder<'static, RadioDriverImpl>,
    /// Beacon configuration (period and enabled state).
    pub beacon_config: BeaconConfig,
    /// Timestamp of last beacon transmission.
    pub last_beacon_time: Option<NsInstant>,
}

impl<RadioDriverImpl: DriverConfig> TschTask<RadioDriverImpl> {
    /// Create new TSCH state.
    ///
    /// Takes context as parameter to allocate RX frame, but does NOT store it.
    pub fn new(context: &mut SchedulerContext<RadioDriverImpl>) -> Self {
        let rx_frame = context.allocate_frame();
        Self {
            state: TschState::Idle {
                next_deadline: INFINITE_DEADLINE,
            },
            pending_operations: Vec::new(),
            last_base_time: NsInstant::from_ticks(0),
            last_asn: 0,
            rx_frame: Cell::new(Some(rx_frame)),
            is_coordinator: false,
            beacon_mpdu: Cell::new(None),
            beacon_builder: EnhancedBeaconBuilder::new(),
            beacon_config: BeaconConfig::disabled(),
            last_beacon_time: None,
        }
    }

    /// Initialize as device with observed ASN and timestamp.
    pub fn init_device(&mut self, asn: TschAsn, timestamp: NsInstant) {
        self.last_base_time = timestamp;
        self.last_asn = asn;
        self.is_coordinator = false;
        self.beacon_config = BeaconConfig::disabled();
        self.last_beacon_time = None;
        self.state = TschState::Idle {
            next_deadline: INFINITE_DEADLINE,
        };
    }

    /// Pop the next operation from the queue (earliest ASN).
    pub fn pop_operation(&mut self) -> TschOperation {
        self.pending_operations.pop().unwrap_or(TschOperation::Idle)
    }

    /// Push an operation to the queue, maintaining ASN order.
    /// Operations are stored with earliest ASN at the end (for efficient pop).
    pub fn push_operation(&mut self, op: TschOperation) -> Result<(), TschOperation> {
        if let Some(op_asn) = op.asn() {
            // Find insertion point to maintain descending order (earliest last)
            let insert_pos = self
                .pending_operations
                .iter()
                .position(|existing| existing.asn().map(|a| a <= op_asn).unwrap_or(true))
                .unwrap_or(self.pending_operations.len());

            // TODO: implement proper sorted insert
            self.pending_operations.push(op)
        } else {
            Err(op)
        }
    }

    /// Get the deadline of the next pending operation.
    pub fn peek_deadline(&self, context: &SchedulerContext<RadioDriverImpl>) -> NsInstant {
        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        match self.pending_operations.last() {
            Some(op) => {
                if let Some(asn) = op.asn() {
                    let instant = self.expected_slot_start(asn, timeslot_length_us);
                    instant - NsDuration::micros(TIMESLOT_GUARD_TIME_US)
                } else {
                    INFINITE_DEADLINE
                }
            }
            None => INFINITE_DEADLINE,
        }
    }

    /// Calculate expected slot start time for given ASN.
    pub fn expected_slot_start(&self, asn: TschAsn, timeslot_length_us: u64) -> NsInstant {
        let slot_duration = NsDuration::micros(timeslot_length_us);
        let slots_diff = asn.saturating_sub(self.last_asn) as u32;
        self.last_base_time + slots_diff * slot_duration
    }

    /// Calculate current ASN from timestamp.
    pub fn asn_at(
        &self,
        instant: NsInstant,
        context: &SchedulerContext<RadioDriverImpl>,
    ) -> TschAsn {
        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        if instant <= self.last_base_time {
            return self.last_asn;
        }
        let elapsed = instant - self.last_base_time;
        self.last_asn + elapsed.to_micros() / timeslot_length_us
    }

    /// Update base time and ASN after executing an operation.
    pub fn update_timing(&mut self, asn: TschAsn, context: &SchedulerContext<RadioDriverImpl>) {
        let timeslot_length_us = context.pib.tsch.timeslot_length_us();
        self.last_base_time = self.expected_slot_start(asn, timeslot_length_us);
        self.last_asn = asn;
    }

    /// Check if in idle state.
    pub fn is_idle(&self) -> bool {
        matches!(self.state, TschState::Idle { .. })
    }

    /// Get the current deadline if idle.
    pub fn idle_deadline(&self) -> Option<NsInstant> {
        match self.state {
            TschState::Idle { next_deadline } => Some(next_deadline),
            _ => None,
        }
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
        start_time: NsInstant,
        beacon_period_secs: Option<u32>,
    ) {
        self.last_base_time = start_time;
        self.last_asn = 0;
        self.is_coordinator = true;
        self.beacon_config = if let Some(beacon_period_secs) = beacon_period_secs {
            BeaconConfig::new(beacon_period_secs)
        } else {
            BeaconConfig::disabled()
        };
        self.last_beacon_time = None;
        self.state = TschState::Idle {
            next_deadline: INFINITE_DEADLINE,
        };
        self.init_beacon_frame(context);
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
    pub fn schedule_beacon(&mut self, context: &SchedulerContext<RadioDriverImpl>) -> bool {
        use dot15d4_driver::timer::RadioTimerApi;

        use crate::scheduler::tsch::TschLinkType;

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
        let min_asn = self.asn_at(min_beacon_time, context);
        let current_asn = self.asn_at(current_time, context);

        let search_from_asn = core::cmp::max(current_asn, min_asn);

        // Find an advertising link
        let link = match context
            .pib
            .tsch
            .links()
            .find(|l| matches!(l.link_type, TschLinkType::Advertising))
        {
            Some(l) => l,
            None => return false,
        };

        // Calculate next ASN for this link
        let next_asn = match context
            .pib
            .tsch
            .next_asn_for_link_strict(link, search_from_asn)
        {
            Some(asn) => asn,
            None => return false,
        };

        // Calculate channel
        let channel =
            context
                .pib
                .tsch
                .channel_for_link(next_asn, link, &context.pib.hopping_sequence);

        // Create and queue the operation
        let op = TschOperation::AdvertisementSlot {
            asn: next_asn,
            channel,
        };

        self.push_operation(op).is_ok()
    }

    /// Mark beacon as sent and record the time.
    pub fn on_beacon_sent(&mut self, instant: NsInstant) {
        self.last_beacon_time = Some(instant);
    }
}
