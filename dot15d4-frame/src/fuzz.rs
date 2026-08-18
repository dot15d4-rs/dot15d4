use core::marker::PhantomData;

use dot15d4_driver::{
    radio::{
        frame::{RadioFrameRepr, RadioFrameUnsized},
        phy::{OQpsk250KBit, Phy},
        DriverConfig, FcsTwoBytes,
    },
    timer::{
        HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
        OptionalNsInstant, RadioTimerApi, RadioTimerError, TimedSignal,
    },
};

/// A driver config for fuzzing. It uses the standard O-QPSK 250 kbit/s PHY and
/// a two-byte FCS, which matches the frame layout the parser expects.
pub struct FuzzDriverConfig;

impl DriverConfig for FuzzDriverConfig {
    const HEADROOM: u8 = 1;
    const TAILROOM: u8 = 2;
    const MAX_SDU_LENGTH: u16 = 127;
    type Fcs = FcsTwoBytes;
    type Timer = FuzzTimer<Sleeping>;
    type Phy = Phy<OQpsk250KBit>;
}

/// The frame layout implied by [`FuzzDriverConfig`].
///
/// The generic arguments are named because both frame states define `new`.
const FRAME_REPR: RadioFrameRepr<FuzzDriverConfig, RadioFrameUnsized> =
    RadioFrameRepr::<FuzzDriverConfig, RadioFrameUnsized>::new();

/// Buffer length a target must allocate for one frame.
///
/// The buffer must be exactly this long. An incoming frame requires at least
/// this many bytes, because the MPDU length is unknown when the buffer is
/// handed to the radio. A longer buffer is rejected when the frame is turned
/// back into a radio frame.
pub const BUFFER_LENGTH: usize = FRAME_REPR.max_buffer_length() as usize;

/// Longest MPDU, without the FCS, that fits in such a buffer.
///
/// This is the PHY limit minus the FCS, so it is the largest frame a radio can
/// deliver. Longer inputs are not reachable over the air.
pub const MAX_MPDU_LENGTH_WO_FCS: usize = FRAME_REPR.max_sdu_length_wo_fcs() as usize;

/// A timer type that satisfies the driver bound but is never called.
#[derive(Clone, Copy)]
pub struct FuzzTimer<State> {
    _state: PhantomData<State>,
}

#[derive(Clone, Copy)]
pub struct Sleeping;
#[derive(Clone, Copy)]
pub struct Running;

impl RadioTimerApi for FuzzTimer<Sleeping> {
    const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);
    const GUARD_TIME: NsDuration = NsDuration::from_ticks(0);

    type HighPrecisionTimer = FuzzTimer<Running>;

    fn now(&self) -> NsInstant {
        unreachable!("the fuzz timer is never called")
    }

    async unsafe fn wait_until(&mut self, _instant: NsInstant) -> Result<(), RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }

    fn start_high_precision_timer(
        &self,
        _start_at: OptionalNsInstant,
    ) -> Result<Self::HighPrecisionTimer, RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }
}

impl HighPrecisionTimer for FuzzTimer<Running> {
    const TICK_PERIOD: NsDuration = NsDuration::from_ticks(0);

    fn schedule_timed_signal(&self, _timed_signal: TimedSignal) -> Result<&Self, RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }

    fn schedule_timed_signal_unless(
        &self,
        _timed_signal: TimedSignal,
        _event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }

    fn schedule_timed_signal_if(
        &self,
        _timed_signal: TimedSignal,
        _event: HardwareEvent,
    ) -> Result<&Self, RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }

    fn commit_timed_signal(&self) {
        unreachable!("the fuzz timer is never called")
    }

    async unsafe fn wait_for(&mut self, _signal: HardwareSignal) {
        unreachable!("the fuzz timer is never called")
    }

    fn observe_event(&self, _event: HardwareEvent) -> Result<&Self, RadioTimerError> {
        unreachable!("the fuzz timer is never called")
    }

    fn poll_event(&self, _event: HardwareEvent) -> OptionalNsInstant {
        unreachable!("the fuzz timer is never called")
    }

    fn reset(&self) {
        unreachable!("the fuzz timer is never called")
    }
}
