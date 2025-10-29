use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::asm::wfe;
#[cfg(all(feature = "timer-trace", not(debug_assertions)))]
use dot15d4::driver::timer::{export::ExtU64, HardwareSignal, LocalClockDuration, TimedSignal};
#[cfg(any(feature = "defmt", feature = "log"))]
use dot15d4::driver::{radio::HighPrecisionTimerOf, timer::HighPrecisionTimer};
use dot15d4::{
    driver::{
        radio::{
            frame::{
                Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
                PanId, RadioFrame,
            },
            phy::PhyConfig,
            tasks::{RadioTask, RadioTransitionResult, TaskRx, TaskTx},
            DriverConfig, PhyOf,
        },
        timer::{LocalClockInstant, RadioTimerApi},
    },
    mac::frame::{
        repr::{MpduRepr, SeqNrRepr},
        MpduWithIes,
    },
    util::allocator::BufferAllocator,
};

use crate::{TestSuite, TEST_SLOT_DURATION};

// PAN ID: 7B:3C
const PAN_ID: PanId<[u8; 2]> = PanId::new_owned([0x3C, 0x7B]);

// Address 1: 02:1A:7D:00:00:8F:12:34
const ADDR1: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0x34, 0x12, 0x8F, 0x00, 0x00, 0x7D, 0x1A, 0x02,
]));

// Address 2: 02:1A:7D:00:00:8F:56:78
const ADDR2: Address<[u8; 8]> = Address::Extended(ExtendedAddress::new_owned([
    0x78, 0x56, 0x8F, 0x00, 0x00, 0x7D, 0x1A, 0x02,
]));

const PAYLOAD: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];

const MPDU_REPR: MpduRepr<'_, MpduWithIes> = MpduRepr::new()
    .with_frame_control(SeqNrRepr::Yes)
    .with_addressing(AddressingRepr::new_legacy_addressing(
        AddressingMode::Extended,
        AddressingMode::Extended,
        true,
    ))
    .without_security()
    .without_ies();

static SEQ_NR: AtomicU8 = AtomicU8::new(0);

#[allow(dead_code)]
pub fn rx_task<Config: DriverConfig>(buffer_allocator: BufferAllocator) -> TaskRx {
    let radio_frame = RadioFrame::new::<Config>(
        buffer_allocator
            .try_allocate_buffer(<PhyOf<Config> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize)
            .unwrap(),
    );

    TaskRx { radio_frame }
}

#[allow(dead_code)]
pub fn tx_task<Config: DriverConfig>(cca: bool, buffer_allocator: BufferAllocator) -> TaskTx {
    let mut mpdu = MPDU_REPR
        .into_writer::<Config>(
            FrameVersion::Ieee802154_2006,
            FrameType::Data,
            PAYLOAD.len() as u16,
            buffer_allocator
                .try_allocate_buffer(<PhyOf<Config> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize)
                .unwrap(),
        )
        .unwrap();

    mpdu.set_sequence_number(SEQ_NR.fetch_add(1, Ordering::Relaxed));
    let mut addressing = mpdu.addressing_fields_mut();
    addressing.src_address_mut().set(&ADDR1);
    addressing.dst_address_mut().set(&ADDR2);
    addressing.dst_pan_id_mut().set(&PAN_ID);
    mpdu.frame_payload_mut().copy_from_slice(&PAYLOAD);
    let radio_frame = mpdu.into_radio_frame::<Config>();

    TaskTx { radio_frame, cca }
}

// Allocates the given test slot and returns the slot's start time.
// Panics if the slot cannot be allocated.
pub async fn allocate_test_slot<Timer: RadioTimerApi>(
    timer: &mut Timer,
    anchor_time: LocalClockInstant,
    test_suite: TestSuite,
    test_slot: usize,
    best_effort: bool,
) -> LocalClockInstant {
    const TEST_SLOT_DURATION_NS: u64 = TEST_SLOT_DURATION.ticks();

    let anchor_time_ns = anchor_time.ticks();
    let test_start_time_ns =
        anchor_time_ns + (test_suite.slot() + test_slot) as u64 * TEST_SLOT_DURATION_NS;
    let test_start_time = LocalClockInstant::from_ticks(test_start_time_ns);

    // Note: Bit banging test marker codes at an acceptable frequency only works
    //       in release mode.
    #[cfg(all(feature = "timer-trace", not(debug_assertions)))]
    if test_slot == 0 {
        // Signal the start of a new test suite by bit-banging the
        // manchester-encoded test suite number to the timer GPIO trace. This
        // requires the high precision timer to be available which is the case
        // at the beginning of each test suite as by convention the radio must
        // be off at this point.

        let test_slot_marker_time = test_start_time - 1.millis();
        let mut high_precision_timer = timer
            .start_high_precision_timer(Some(test_slot_marker_time - Timer::GUARD_TIME))
            .unwrap();

        const MANCHESTER_HALF_PERIOD_NS: u64 = LocalClockDuration::micros(20).ticks();

        let test_slot_marker_time_ns = test_slot_marker_time.ticks();
        for (index, &toggle_at_tick) in test_suite.manchester_encode().iter().enumerate() {
            if index > 0 && toggle_at_tick == 0 {
                break;
            }

            let toggle_at = LocalClockInstant::from_ticks(
                test_slot_marker_time_ns + toggle_at_tick as u64 * MANCHESTER_HALF_PERIOD_NS,
            );
            let timed_signal = TimedSignal::new(toggle_at, HardwareSignal::GpioToggle);
            high_precision_timer
                .schedule_timed_signal(timed_signal)
                .unwrap();

            // Safety: We run from the main thread (i.e. at a lower priority than the
            //         timer) and don't migrate away from it.
            unsafe { high_precision_timer.wait_for(HardwareSignal::GpioToggle) }.await;
        }
    }

    if best_effort {
        // Safety: see above.
        unsafe { timer.wait_until(test_start_time) }.await.unwrap();
    }

    test_start_time
}

impl TestSuite {
    pub fn manchester_encode(&self) -> &[u8] {
        const MANCHESTER0: [u8; 10] = TestSuite::manchester_encode_internal(0);
        const MANCHESTER1: [u8; 10] = TestSuite::manchester_encode_internal(1);
        const MANCHESTER2: [u8; 10] = TestSuite::manchester_encode_internal(2);
        const MANCHESTER3: [u8; 10] = TestSuite::manchester_encode_internal(3);
        const MANCHESTER4: [u8; 10] = TestSuite::manchester_encode_internal(4);
        const MANCHESTER5: [u8; 10] = TestSuite::manchester_encode_internal(5);
        const MANCHESTER6: [u8; 10] = TestSuite::manchester_encode_internal(6);
        const MANCHESTER7: [u8; 10] = TestSuite::manchester_encode_internal(7);

        match self {
            TestSuite::SingleTimedRxOff => &MANCHESTER0,
            TestSuite::SingleTimedTxRx => &MANCHESTER1,
            TestSuite::SingleTimedTxTxWithoutCca => &MANCHESTER2,
            TestSuite::SingleTimedTxTxWithCca => &MANCHESTER3,
            TestSuite::SingleBestEffortRxOff => &MANCHESTER4,
            TestSuite::SingleBestEffortTxRx => &MANCHESTER5,
            TestSuite::SingleBestEffortTxTxWithoutCca => &MANCHESTER6,
            TestSuite::SingleBestEffortTxTxWithCca => &MANCHESTER7,
        }
    }

    const fn manchester_encode_internal<const NUM_TICKS: usize>(n: u8) -> [u8; NUM_TICKS] {
        let num_bits = NUM_TICKS / 2;
        debug_assert!(n < (1 << (num_bits - 1)));

        let mut toggles = [0; NUM_TICKS];

        let mut current_signal_level = false;
        let mut tick = 0;
        let mut num_toggles = 0;

        // Add a 1-bit preamble.
        let n = (n << 1) | 1;

        // Process each bit, LSB first.
        let mut bit_pos: usize = 0;
        while bit_pos < num_bits {
            let bit_value = ((n >> bit_pos) & 1) == 1;

            // Manchester encoding:
            // 0: Low->High
            // 1: High->Low

            // The required signal at the start of a bit equals the bit value.
            if current_signal_level != bit_value {
                // The initial signal level doesn't match: toggle it.
                toggles[num_toggles] = tick;
                num_toggles += 1;
            }
            tick += 1;

            toggles[num_toggles] = tick;
            num_toggles += 1;
            tick += 1;

            current_signal_level = !bit_value;
            bit_pos += 1;
        }

        // Always toggle back to zero.
        if current_signal_level {
            toggles[num_toggles] = tick;
        }

        toggles
    }
}

pub fn log_timing<Config: DriverConfig, PrevTask: RadioTask, ThisTask: RadioTask>(
    _label: &str,
    _anchor_time: LocalClockInstant,
    _test_suite: TestSuite,
    _test_slot: usize,
    _entry: usize,
    _radio_transition_result: &RadioTransitionResult<Config, PrevTask, ThisTask>,
    _cca: bool,
) {
    #[cfg(any(feature = "defmt", feature = "log"))]
    {
        use crate::TEST_SLOT_DURATION_MS;

        let cca_hint = if _cca { " - with CCA" } else { "" };
        let anchor_time_ns = _anchor_time.ticks();
        let ts_ms = (_test_suite.slot() + _test_slot) * TEST_SLOT_DURATION_MS;

        let RadioTransitionResult {
            scheduled_entry,
            measured_entry,
            ..
        } = _radio_transition_result;
        let (scheduled_ns, measured_ns) = (
            scheduled_entry
                .map(|ts| ts.ticks() - anchor_time_ns)
                .unwrap_or(0),
            measured_entry.ticks() - anchor_time_ns,
        );
        let difference_ns = if scheduled_ns > 0 {
            measured_ns as i64 - scheduled_ns as i64
        } else {
            0
        };

        if _entry == 1 {
            if _test_slot == 0 {
                dot15d4::util::log::info!("Test Suite {}", _test_suite as usize);
            }

            dot15d4::util::log::info!(" @ {} ms:", ts_ms);
        }

        // We allow +/- one high precision clock tick offset between scheduled and
        // measured timestamps.
        let tolerance_ns =
            <HighPrecisionTimerOf<Config> as HighPrecisionTimer>::TICK_PERIOD.ticks();
        if scheduled_ns != 0 {
            if difference_ns.unsigned_abs() <= tolerance_ns {
                dot15d4::util::log::info!(
                    "  {}: Scheduled: {} ns, Actual: {} ns, (Error: {} ns){}\0",
                    _label,
                    scheduled_ns,
                    measured_ns,
                    difference_ns,
                    cca_hint,
                );
            } else {
                dot15d4::util::log::error!(
                    "  {}: Scheduled: {} ns, Actual: {} ns, (Error: {} ns){}\0",
                    _label,
                    scheduled_ns,
                    measured_ns,
                    difference_ns,
                    cca_hint,
                );
            }
        } else {
            dot15d4::util::log::info!("  {}: Actual: {} ns{}\0", _label, measured_ns, cca_hint,);
        }
    }
}

pub fn done() -> ! {
    loop {
        wfe();
    }
}
