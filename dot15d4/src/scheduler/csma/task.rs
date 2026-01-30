//! CSMA scheduler state

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

/// CSMA scheduler state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CsmaState {
    /// No operation pending.
    Idle,
    /// Performing CCA before TX.
    WaitingForTxStart,
    /// Transmitting, waiting for TX completion (Sent/Nack).
    Transmitting,
    /// Listening for incoming frames.
    Listening,
    /// Actively receiving a frame.
    Receiving,
    /// Terminating CSMA (e.g. to switch to TSCH).
    #[cfg(feature = "tsch")]
    Terminating,
}

/// Information about what operation has been pipelined.
#[derive(Debug)]
pub enum Pipelined {
    /// Next is RX (frame stored in rx_frame).
    Rx,
    /// Next is TX with this token (frame already sent to driver).
    Tx(ResponseToken),
}

impl Pipelined {
    #[inline]
    pub fn is_tx(&self) -> bool {
        matches!(self, Pipelined::Tx(_))
    }
}

/// Pending TX from NACK recovery.
pub type TxRequest = (ResponseToken, MpduFrame);

/// CSMA-CA backoff state.
#[derive(Debug, Clone, Copy)]
pub struct Backoff {
    /// Number of backoffs (NB).
    pub nb: u8,
    /// Backoff exponent (BE).
    pub be: u8,
}

impl Backoff {
    /// Create new backoff state with given minimum BE.
    #[inline]
    pub const fn new(min_be: u8) -> Self {
        Self { nb: 0, be: min_be }
    }

    /// Reset backoff state for a new transmission attempt.
    #[inline]
    pub fn reset(&mut self, min_be: u8) {
        self.nb = 0;
        self.be = min_be;
    }

    /// Handle CCA failure. Returns true if can retry, false if limit exceeded.
    #[inline]
    pub fn on_failure(&mut self, max_backoffs: u8, max_be: u8) -> bool {
        self.nb += 1;
        self.be = self.be.saturating_add(1).min(max_be);
        self.nb <= max_backoffs
    }

    /// Calculate backoff delay based on current BE and RNG.
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

/// CSMA task
pub struct CsmaTask<RadioDriverImpl: DriverConfig> {
    /// Current scheduler state
    pub state: CsmaState,
    /// Backoff state
    pub backoff: Backoff,
    /// TX retry count
    pub tx_retries: u8,

    /// PHY Channel
    pub channel: Channel,
    /// Base time for scheduling
    pub base_time: NsInstant,

    /// Current TX token
    pub tx_token: Option<ResponseToken>,
    /// Next operation info
    pub pipelined: Option<Pipelined>,

    /// Pending TX from NACK recovery
    pub pending_tx: Option<TxRequest>,
    /// RX frame buffer
    pub rx_frame: Option<RadioFrame<RadioFrameUnsized>>,

    /// Marker for Radio Driver Implementation
    _marker: PhantomData<RadioDriverImpl>,
}

impl<RadioDriverImpl: DriverConfig> CsmaTask<RadioDriverImpl> {
    /// Create a new CSMA task.
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

    /// Take the TX token (panics if none).
    #[inline]
    pub fn take_tx_token(&mut self) -> ResponseToken {
        self.tx_token.take().expect("no tx token")
    }

    /// Calculate backoff time from base_time.
    #[inline]
    pub fn backoff_time(&mut self, rng: &mut dyn RngCore) -> NsInstant {
        self.base_time + self.backoff.delay::<RadioDriverImpl>(rng)
    }
}
