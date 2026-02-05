//! CSMA-CA scheduler task state and structures.
//!
//! This module defines the state machine structures for the CSMA-CA scheduler,
//! including the task state, backoff algorithm implementation, and pipelining.

use core::marker::PhantomData;

use dot15d4_driver::{
    radio::{
        config::Channel,
        frame::{RadioFrame, RadioFrameUnsized},
        phy::PhyConfig,
        DriverConfig, PhyOf,
    },
    timer::{NsDuration, NsInstant},
};
use dot15d4_frame::mpdu::MpduFrame;
use dot15d4_util::sync::ResponseToken;
use rand_core::RngCore;

use crate::scheduler::SchedulerContext;
#[cfg(feature = "tsch")]
use crate::scheduler::SchedulerTaskCompletion;

/// CSMA-CA scheduler state machine states.
///
/// Represents the current state of the CSMA scheduler.
#[derive(Debug)]
#[repr(u8)]
pub enum CsmaState {
    /// No operation pending, scheduler is inactive.
    Idle,
    /// TX request submitted to driver, waiting for CCA and TX start.
    WaitingForTxStart,
    /// Frame transmission in progress, waiting for completion (Sent/Nack).
    Transmitting,
    /// Receiver enabled, waiting for incoming frames.
    Listening,
    /// Frame reception in progress (SFD detected).
    Receiving,
    /// Terminating CSMA operation to switch to another mode (e.g., TSCH).
    #[cfg(feature = "tsch")]
    Terminating(SchedulerTaskCompletion),
}

/// Information about a pipelined operation.
///
/// Pipelining allows the scheduler to prepare the next operation while
/// the current one is still completing, reducing latency.
#[derive(Debug)]
pub enum Pipelined {
    /// Next operation is RX (frame buffer stored in `rx_frame`).
    Rx,
    /// Next operation is TX with the given response token.
    Tx(ResponseToken),
}

impl Pipelined {
    /// Check if the pipelined operation is a transmission.
    #[inline]
    pub fn is_tx(&self) -> bool {
        matches!(self, Pipelined::Tx(_))
    }
}

/// Pending TX request from NACK recovery.
///
/// Contains the response token and MPDU for retransmission.
pub type TxRequest = (ResponseToken, MpduFrame);

/// CSMA-CA backoff algorithm state.
#[derive(Debug, Clone, Copy)]
pub struct Backoff {
    /// Number of backoffs attempted (NB).
    pub nb: u8,
    /// Current backoff exponent (BE).
    pub be: u8,
}

impl Backoff {
    /// Create new backoff state initialized with the given minimum BE.
    ///
    /// # Arguments
    ///
    /// * `min_be` - Initial backoff exponent (typically macMinBE from PIB)
    #[inline]
    pub const fn new(min_be: u8) -> Self {
        Self { nb: 0, be: min_be }
    }

    /// Reset backoff state for a new transmission attempt.
    ///
    /// Called when starting a new transmission or after successful TX.
    ///
    /// # Arguments
    ///
    /// * `min_be` - Minimum backoff exponent to reset to
    #[inline]
    pub fn reset(&mut self, min_be: u8) {
        self.nb = 0;
        self.be = min_be;
    }

    /// Handle CCA failure and update backoff parameters.
    ///
    /// Increments NB and increases BE (up to max_be).
    ///
    /// # Arguments
    ///
    /// * `max_backoffs` - Maximum number of backoff allowed (usually macMaxCSMABackoffs)
    /// * `max_be` - Maximum backoff exponent (usually macMaxBE)
    ///
    /// # Returns
    ///
    /// `true` if another retry is allowed, `false` if limit exceeded.
    #[inline]
    pub fn on_failure(&mut self, max_backoffs: u8, max_be: u8) -> bool {
        self.nb += 1;
        self.be = self.be.saturating_add(1).min(max_be);
        self.nb <= max_backoffs
    }

    /// Calculate the random backoff delay.
    ///
    /// Computes: random(0, 2^BE - 1) * aUnitBackoffPeriod + overhead(guards)
    ///
    /// # Arguments
    ///
    /// * `rng` - Random number generator
    ///
    /// # Returns
    ///
    /// The backoff delay duration.
    #[inline]
    pub fn delay<RadioDriverImpl: DriverConfig>(&self, rng: &mut dyn RngCore) -> NsDuration {
        let max = (1u32 << self.be).saturating_sub(1);
        let backoff_periods = if max == 0 {
            0
        } else {
            rng.next_u32() % (max + 1)
        };

        // Calculate the standard backoff delay based on the backoff exponent
        let base_delay = <PhyOf<RadioDriverImpl> as PhyConfig>::MAC_UNIT_BACKOFF_PERIOD;

        // The backoff delay in the standard
        let standard_delay = base_delay * backoff_periods;

        // FIXME: only for nRF52840
        // Add fixed overhead for nRF52840 to compensate for RMARKER:
        // - RXEN ramp-up: +- 130us
        // - Turnaround time: +- 130us
        // - PHY SHR duration for frame detection : 160us
        // - CCA+Turnaround : 320us
        // Total overhead +- 580us minimum
        const FIXED_RMARKER_OVERHEAD: NsDuration = NsDuration::micros(580);

        // Minimum delay to ensure timer synchronization completes
        const MIN_DELAY: NsDuration =
            match FIXED_RMARKER_OVERHEAD.checked_add(NsDuration::micros(150)) {
                Some(d) => d,
                None => NsDuration::micros(730), // fallback
            };

        // Total delay is max of minimum and (standard + overhead)
        let total_delay = match standard_delay.checked_add(FIXED_RMARKER_OVERHEAD) {
            Some(d) => d,
            None => standard_delay,
        };

        if total_delay < MIN_DELAY {
            MIN_DELAY
        } else {
            total_delay
        }
    }
}

/// CSMA-CA scheduler task.
///
/// Manages the state machine for CSMA-CA medium access control.
/// Handles transmission requests, backoff timing, retransmissions,
/// and reception.
pub struct CsmaTask<RadioDriverImpl: DriverConfig> {
    /// Current state machine state.
    pub state: CsmaState,
    /// Backoff algorithm state.
    pub backoff: Backoff,
    /// Number of transmission retries for current frame.
    pub tx_retries: u8,

    /// PHY channel for transmissions.
    pub channel: Channel,
    /// Base timestamp for scheduling (updated after each operation).
    pub base_time: NsInstant,

    /// Response token for current TX operation.
    pub tx_token: Option<ResponseToken>,
    /// Information about pipelined next operation.
    pub pipelined: Option<Pipelined>,

    /// Pending TX request from NACK recovery for retransmission.
    pub pending_tx: Option<TxRequest>,
    /// Pre-allocated buffer for receiving frames.
    pub rx_frame: Option<RadioFrame<RadioFrameUnsized>>,

    /// Phantom data for radio driver type.
    _marker: PhantomData<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Create a new CSMA task.
    ///
    /// Initializes the task in Idle state with default backoff parameters
    /// from the PIB and allocates an RX frame buffer.
    ///
    /// # Arguments
    ///
    /// * `channel` - PHY channel to use
    /// * `context` - Scheduler context for accessing PIB and allocator
    pub fn new(channel: Channel, context: &mut SchedulerContext<RadioDriverImpl>) -> Self {
        Self {
            state: CsmaState::Idle,
            backoff: Backoff::new(context.pib.min_be),
            tx_retries: 0,
            channel,
            base_time: NsInstant::from_ticks(0),
            tx_token: None,
            pipelined: None,
            pending_tx: None,
            rx_frame: Some(context.allocate_frame()),
            _marker: PhantomData,
        }
    }

    /// Take the current TX response token.
    #[inline]
    pub fn take_tx_token(&mut self) -> ResponseToken {
        self.tx_token.take().expect("no tx token")
    }

    /// Calculate the next backoff time based on current base_time.
    #[inline]
    pub fn backoff_time(&mut self, rng: &mut dyn RngCore) -> NsInstant {
        self.base_time + self.backoff.delay::<RadioDriverImpl>(rng)
    }
}
