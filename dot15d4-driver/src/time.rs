//! Time structures.
//!
//! - [`Instant`] is used to represent a point in time.
//! - [`Duration`] is used to represent a duration of time.

use core::marker::PhantomData;

use crate::{DriverConfig, RadioTimerApi};

pub trait Frequency {
    const FREQUENCY: u32;
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct Milliseconds;

impl Frequency for Milliseconds {
    const FREQUENCY: u32 = 1_000;
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct Microseconds;

impl Frequency for Microseconds {
    const FREQUENCY: u32 = 1_000_000;
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct Nanoseconds;

impl Frequency for Nanoseconds {
    const FREQUENCY: u32 = 1_000_000_000;
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct SymbolsOQpsk250kB;

impl Frequency for SymbolsOQpsk250kB {
    // 250kB/s = 31.25kb/s = 62.5ksymbol/s (1 byte = 8 bit = 2 O-QPSK symbols)
    const FREQUENCY: u32 = 62_500;
}

// The radio timer's high-precision timer frequency in Hertz.
pub const fn timer_frequency<F: Frequency>() -> u32 {
    <F as Frequency>::FREQUENCY
}

/// Converts ticks between different frequencies while rounding down.
const fn convert_rounding_down(ticks: u64, from_frequency: u32, to_frequency: u32) -> u64 {
    (ticks * to_frequency as u64) / from_frequency as u64
}

/// Converts ticks between different frequencies while rounding up.
const fn convert_rounding_up(ticks: u64, from_frequency: u32, to_frequency: u32) -> u64 {
    (ticks * to_frequency as u64).div_ceil(from_frequency as u64)
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct Instant<F: Frequency> {
    tick: u64, // in high-precision radio timer ticks
    frequency: PhantomData<F>,
}

impl<F: Frequency> Instant<F> {
    pub const NEVER: Self = Self::new(u64::MAX);

    pub const fn new(tick: u64) -> Self {
        Self {
            tick,
            frequency: PhantomData,
        }
    }

    pub const fn frequency(&self) -> u32 {
        timer_frequency::<F>()
    }

    /// Converts this [`Instant`] into an instant based on a different
    /// frequency.
    ///
    /// This operation rounds down to an oscillator tick of the target frequency
    /// internally.
    ///
    /// Note: This is an expensive and lossy operation. It should neither be
    ///       called on time-critical paths nor where full timing precision is
    ///       required.
    // Note: Cannot implement the Into trait as it will conflict with the
    //       internal blanket implementation.
    pub const fn convert_into_rounding_down<ToFrequency: Frequency>(&self) -> Instant<ToFrequency> {
        Instant::new(convert_rounding_down(
            self.tick,
            timer_frequency::<F>(),
            timer_frequency::<ToFrequency>(),
        ))
    }

    /// Converts this [`Instant`] into an instant based on a different
    /// frequency.
    ///
    /// This operation rounds up to an oscillator tick of the target frequency
    /// internally.
    ///
    /// Note: This is an expensive and lossy operation. It should neither be
    ///       called on time-critical paths nor where full timing precision is
    ///       required.
    // Note: Cannot implement the Into trait as it will conflict with the
    //       internal blanket implementation.
    pub const fn convert_into_rounding_up<ToFrequency: Frequency>(&self) -> Instant<ToFrequency> {
        Instant::new(convert_rounding_up(
            self.tick,
            timer_frequency::<F>(),
            timer_frequency::<ToFrequency>(),
        ))
    }

    pub const fn tick(&self) -> u64 {
        self.tick
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Eq, Ord)]
#[cfg_attr(feature = "fuzz", derive(arbitrary::Arbitrary))]
pub struct Duration<F: Frequency> {
    ticks: i64, // in high-precision radio timer ticks
    frequency: PhantomData<F>,
}

impl<F: Frequency> Duration<F> {
    pub const ZERO: Self = Self::new(0);

    pub const fn new(ticks: i64) -> Self {
        Self {
            ticks,
            frequency: PhantomData,
        }
    }

    pub const fn frequency(&self) -> u32 {
        timer_frequency::<F>()
    }

    /// Converts this [`Duration`] into the same duration based on a different
    /// frequency.
    ///
    /// This operation rounds oscillator ticks of the target frequency
    /// internally towards zero.
    ///
    /// Note: This is an expensive and lossy operation. It should neither be
    ///       called on time-critical paths nor where full timing precision is
    ///       required.
    // Note: Cannot implement the Into trait as it will conflict with the
    //       internal blanket implementation.
    pub const fn convert_into_rounding_down<ToFrequency: Frequency>(
        &self,
    ) -> Duration<ToFrequency> {
        let sign = self.ticks.signum();
        Duration::new(
            sign * convert_rounding_down(
                (sign * self.ticks) as u64,
                timer_frequency::<F>(),
                timer_frequency::<ToFrequency>(),
            ) as i64,
        )
    }

    /// Converts this [`Duration`] into the same duration based on a different
    /// frequency.
    ///
    /// This operation rounds oscillator ticks of the target frequency
    /// internally towards positive/negative infinity.
    ///
    /// Note: This is an expensive and lossy operation. It should neither be
    ///       called on time-critical paths nor where full timing precision is
    ///       required.
    // Note: Cannot implement the Into trait as it will conflict with the
    //       internal blanket implementation.
    pub const fn convert_into_rounding_up<ToFrequency: Frequency>(&self) -> Duration<ToFrequency> {
        let sign = self.ticks.signum();
        Duration::new(
            sign * convert_rounding_up(
                (sign * self.ticks) as u64,
                timer_frequency::<F>(),
                timer_frequency::<ToFrequency>(),
            ) as i64,
        )
    }

    pub const fn ticks(&self) -> i64 {
        self.ticks
    }
}

// Note: Instants cannot be added, multiplied, divided or negated. The
//       difference between instances is defined and yields a duration.
impl<F: Frequency> core::ops::Sub for Instant<F> {
    type Output = Duration<F>;

    fn sub(self, rhs: Instant<F>) -> Self::Output {
        let ticks = if self.tick >= rhs.tick {
            (self.tick - rhs.tick) as i64
        } else {
            -((rhs.tick - self.tick) as i64)
        };
        Duration {
            ticks,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::ops::Add<Duration<F>> for Instant<F> {
    type Output = Self;

    fn add(self, rhs: Duration<F>) -> Self::Output {
        let tick = if rhs.ticks > 0 {
            self.tick + (rhs.ticks as u64)
        } else {
            self.tick - (-rhs.ticks) as u64
        };
        Self {
            tick,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::ops::Sub<Duration<F>> for Instant<F> {
    type Output = Self;

    fn sub(self, rhs: Duration<F>) -> Self::Output {
        self + (-rhs)
    }
}

impl<F: Frequency> core::ops::Add for Duration<F> {
    type Output = Self;

    fn add(self, rhs: Duration<F>) -> Self::Output {
        Self {
            ticks: self.ticks + rhs.ticks,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::ops::Sub for Duration<F> {
    type Output = Self;

    fn sub(self, rhs: Duration<F>) -> Self::Output {
        self + (-rhs)
    }
}

impl<F: Frequency> core::ops::Neg for Duration<F> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            ticks: -self.ticks,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::ops::Mul<usize> for Duration<F> {
    type Output = Self;

    fn mul(self, rhs: usize) -> Self::Output {
        Self {
            ticks: self.ticks * rhs as i64,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::ops::Mul<Duration<F>> for usize {
    type Output = Duration<F>;

    fn mul(self, rhs: Duration<F>) -> Self::Output {
        rhs * self
    }
}

impl<F: Frequency> core::ops::Div<usize> for Duration<F> {
    type Output = Self;

    // TODO: Should we really support this expensive and lossy operation?
    fn div(self, rhs: usize) -> Self::Output {
        Self {
            ticks: self.ticks / rhs as i64,
            frequency: PhantomData,
        }
    }
}

impl<F: Frequency> core::fmt::Display for Instant<F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{:.2}ms (tick {})",
            self.convert_into_rounding_down::<Microseconds>().tick() as f32 / 1000.0,
            self.tick
        )
    }
}

impl<F: Frequency> core::fmt::Display for Duration<F> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{:.2}ms ({} ticks)",
            self.convert_into_rounding_down::<Microseconds>().ticks() as f32 / 1000.0,
            self.ticks
        )
    }
}

pub type Timer<RadioDriverImpl> = <RadioDriverImpl as DriverConfig>::Timer;

pub fn now<Timer: RadioTimerApi>() -> Instant<Timer> {
    Timer::now()
}

pub fn schedule_alarm<Timer: RadioTimerApi>(at: Instant<Timer>) {
    Timer::schedule_alarm(at)
}

pub async fn wait_for_alarm<Timer: RadioTimerApi>() -> Instant<Timer> {
    Timer::wait_for_alarm().await
}

pub async fn wait_for_alarm_at<Timer: RadioTimerApi>(at: Instant<Timer>) {
    Timer::wait_for_alarm_at(at).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instant() {
        let a = Instant::<Microseconds>::new(100);
        assert_eq!(a.tick(), 100);
        let b: Instant<Nanoseconds> = a.convert_into_rounding_down();
        assert_eq!(b.tick(), 100_000);
        assert_eq!(b.convert_into_rounding_down::<Microseconds>().tick(), 100);
    }

    #[test]
    fn instant_operations() {
        let a = Instant::<Microseconds>::new(100);
        let b = Instant::<Microseconds>::new(50);
        assert_eq!(
            (a - b).convert_into_rounding_down::<Microseconds>().ticks(),
            50
        );
        assert_eq!(
            (a - Instant::<Microseconds>::new(50))
                .convert_into_rounding_down::<Microseconds>()
                .ticks(),
            50
        );
        assert_eq!(
            (a + Duration::<Microseconds>::new(50))
                .convert_into_rounding_down::<Microseconds>()
                .tick(),
            150
        );
    }

    #[test]
    fn duration() {
        let a = Duration::<Microseconds>::new(100);
        assert_eq!(a.ticks(), 100);
        assert_eq!((-a).ticks(), -100);
        let b: Duration<Nanoseconds> = a.convert_into_rounding_down();
        assert_eq!(b.ticks(), 100_000);
        assert_eq!(b.convert_into_rounding_down::<Microseconds>().ticks(), 100);
    }

    #[test]
    fn duration_operations() {
        let a = Duration::<Microseconds>::new(100);
        let b = Duration::<Microseconds>::new(50);
        assert_eq!(
            (a - b).convert_into_rounding_down::<Microseconds>().ticks(),
            50
        );
        assert_eq!(
            (a * 2).convert_into_rounding_down::<Microseconds>().ticks(),
            200
        );
        assert_eq!(
            (a / 2).convert_into_rounding_down::<Microseconds>().ticks(),
            50
        );
        assert_eq!(
            (a + b).convert_into_rounding_down::<Microseconds>().ticks(),
            150
        );
    }

    #[test]
    #[cfg(feature = "std")]
    fn formatting() {
        let a = Instant::<Microseconds>::new(100);
        let b = Duration::<Microseconds>::new(100);
        assert_eq!(format!("{a}"), "0.10ms (tick 100)");
        assert_eq!(format!("{b}"), "0.10ms (100 ticks)");
    }
}
