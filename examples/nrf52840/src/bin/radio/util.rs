use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::asm::wfe;
#[cfg(any(feature = "defmt", feature = "log"))]
use dot15d4::driver::radio::HighPrecisionTimerOf;
#[cfg(any(
    all(feature = "timer-trace", not(debug_assertions)),
    feature = "defmt",
    feature = "log"
))]
use dot15d4::driver::timer::HighPrecisionTimer;
#[cfg(all(feature = "timer-trace", not(debug_assertions)))]
use dot15d4::driver::timer::{export::ExtU64, HardwareSignal, NsDuration, TimedSignal};
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
        timer::{NsInstant, OptionalNsInstant, RadioTimerApi},
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

pub const PAYLOAD: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
const PAYLOAD_LEN: u16 = PAYLOAD.len() as u16;

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
    let mut mpdu_writer = MPDU_REPR
        .into_writer::<Config>(
            FrameVersion::Ieee802154_2006,
            FrameType::Data,
            PAYLOAD_LEN,
            buffer_allocator
                .try_allocate_buffer(<PhyOf<Config> as PhyConfig>::PHY_MAX_PACKET_SIZE as usize)
                .unwrap(),
        )
        .unwrap();

    mpdu_writer.set_sequence_number(SEQ_NR.fetch_add(1, Ordering::Relaxed));
    let mut addressing = mpdu_writer.addressing_fields_mut();
    addressing.src_address_mut().set(&ADDR1);
    addressing.dst_address_mut().set(&ADDR2);
    addressing.dst_pan_id_mut().set(&PAN_ID);
    mpdu_writer.frame_payload_mut().copy_from_slice(&PAYLOAD);
    let radio_frame = mpdu_writer.into_radio_frame::<Config>();

    TaskTx { radio_frame, cca }
}

// Allocates the given test slot and returns the slot's start time.
// Panics if the slot cannot be allocated.
pub async fn allocate_test_slot<Timer: RadioTimerApi>(
    timer: &mut Timer,
    anchor_time: NsInstant,
    test_suite: TestSuite,
    test_slot: usize,
    best_effort: bool,
) -> NsInstant {
    const TEST_SLOT_DURATION_NS: u64 = TEST_SLOT_DURATION.ticks();

    let anchor_time_ns = anchor_time.ticks();
    let test_start_time_ns =
        anchor_time_ns + (test_suite.slot() + test_slot) as u64 * TEST_SLOT_DURATION_NS;
    let test_start_time = NsInstant::from_ticks(test_start_time_ns);

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

        const MANCHESTER_HALF_PERIOD_NS: u64 = NsDuration::micros(30).ticks();

        let test_slot_marker_time_ns = test_slot_marker_time.ticks();
        for (index, &toggle_at_tick) in test_suite.encoded_test_id().iter().enumerate() {
            if index > 0 && toggle_at_tick == 0 {
                break;
            }

            let toggle_at = NsInstant::from_ticks(
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
    pub fn encoded_test_id(&self) -> &[u8] {
        use TestSuite::*;

        #[cfg(feature = "device-sync")]
        const TEST_ID_0: [u8; 10] = TestSuite::manchester_encode(MultiTimedRx);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_1: [u8; 10] = TestSuite::manchester_encode(SingleTimedRxOff);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_2: [u8; 10] = TestSuite::manchester_encode(SingleTimedTxRx);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_3: [u8; 10] = TestSuite::manchester_encode(SingleTimedTxTxWithoutCca);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_4: [u8; 10] = TestSuite::manchester_encode(SingleTimedTxTxWithCca);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_5: [u8; 10] = TestSuite::manchester_encode(SingleBestEffortRxOff);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_6: [u8; 10] = TestSuite::manchester_encode(SingleBestEffortTxRx);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_7: [u8; 10] = TestSuite::manchester_encode(SingleBestEffortTxTxWithoutCca);
        #[cfg(not(feature = "device-sync-client"))]
        const TEST_ID_8: [u8; 10] = TestSuite::manchester_encode(SingleBestEffortTxTxWithCca);

        match self {
            #[cfg(feature = "device-sync")]
            MultiTimedRx => &TEST_ID_0,
            #[cfg(not(feature = "device-sync-client"))]
            SingleTimedRxOff => &TEST_ID_1,
            #[cfg(not(feature = "device-sync-client"))]
            SingleTimedTxRx => &TEST_ID_2,
            #[cfg(not(feature = "device-sync-client"))]
            SingleTimedTxTxWithoutCca => &TEST_ID_3,
            #[cfg(not(feature = "device-sync-client"))]
            SingleTimedTxTxWithCca => &TEST_ID_4,
            #[cfg(not(feature = "device-sync-client"))]
            SingleBestEffortRxOff => &TEST_ID_5,
            #[cfg(not(feature = "device-sync-client"))]
            SingleBestEffortTxRx => &TEST_ID_6,
            #[cfg(not(feature = "device-sync-client"))]
            SingleBestEffortTxTxWithoutCca => &TEST_ID_7,
            #[cfg(not(feature = "device-sync-client"))]
            SingleBestEffortTxTxWithCca => &TEST_ID_8,
        }
    }

    const fn manchester_encode<const NUM_TICKS: usize>(test_suite: TestSuite) -> [u8; NUM_TICKS] {
        let test_id = test_suite as u8;
        let num_bits = NUM_TICKS / 2;
        debug_assert!(test_id < (1 << (num_bits - 1)));

        let mut toggles = [0; NUM_TICKS];

        let mut current_signal_level = false;
        let mut tick = 0;
        let mut num_toggles = 0;

        // Add a 1-bit preamble.
        let test_id = (test_id << 1) | 1;

        // Process each bit, LSB first.
        let mut bit_pos: usize = 0;
        while bit_pos < num_bits {
            let bit_value = ((test_id >> bit_pos) & 1) == 1;

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

#[allow(dead_code)]
pub struct TestResult {
    pub label: &'static str,
    pub test_suite: TestSuite,
    pub test_slot: usize,
    pub test_step: usize,
    pub anchor_time: NsInstant,
    pub expected_timestamp: OptionalNsInstant,
    pub measured_timestamp: NsInstant,
    pub cca: bool,
}

impl TestResult {
    pub fn from_radio_transition_result<
        Config: DriverConfig,
        PrevTask: RadioTask,
        ThisTask: RadioTask,
    >(
        label: &'static str,
        test_suite: TestSuite,
        test_slot: usize,
        test_step: usize,
        anchor_time: NsInstant,
        radio_transition_result: &RadioTransitionResult<Config, PrevTask, ThisTask>,
        cca: bool,
    ) -> Self {
        let RadioTransitionResult {
            scheduled_entry: expected_timestamp,
            measured_entry: measured_timestamp,
            ..
        } = *radio_transition_result;
        Self {
            label,
            test_suite,
            test_slot,
            test_step,
            anchor_time,
            expected_timestamp,
            measured_timestamp,
            cca,
        }
    }
}

pub fn log_transition_result<Config: DriverConfig, PrevTask: RadioTask, ThisTask: RadioTask>(
    label: &'static str,
    test_suite: TestSuite,
    test_slot: usize,
    test_step: usize,
    anchor_time: NsInstant,
    radio_transition_result: &RadioTransitionResult<Config, PrevTask, ThisTask>,
    cca: bool,
) {
    log_test_result::<Config>(TestResult::from_radio_transition_result(
        label,
        test_suite,
        test_slot,
        test_step,
        anchor_time,
        radio_transition_result,
        cca,
    ));
}

pub fn log_test_result<Config: DriverConfig>(_test_result: TestResult) {
    #[cfg(any(feature = "defmt", feature = "log"))]
    {
        use crate::TEST_SLOT_DURATION_MS;

        let TestResult {
            label,
            test_suite,
            test_slot,
            test_step,
            anchor_time,
            expected_timestamp,
            measured_timestamp,
            cca,
        } = _test_result;

        let cca_hint = if cca { " - with CCA" } else { "" };
        let anchor_time_ns = anchor_time.ticks();
        let ts_ms = (test_suite.slot() + test_slot) * TEST_SLOT_DURATION_MS;

        let (scheduled_ns, measured_ns) = (
            expected_timestamp
                .map(|ts| ts.ticks() - anchor_time_ns)
                .unwrap_or(0),
            measured_timestamp.ticks() - anchor_time_ns,
        );
        let difference_ns = if scheduled_ns > 0 {
            measured_ns as i64 - scheduled_ns as i64
        } else {
            0
        };

        if test_step == 1 {
            if test_slot == 0 {
                dot15d4::util::log::info!("Test Suite {}", test_suite as usize);
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
                    label,
                    scheduled_ns,
                    measured_ns,
                    difference_ns,
                    cca_hint,
                );
            } else {
                dot15d4::util::log::error!(
                    "  {}: Scheduled: {} ns, Actual: {} ns, (Error: {} ns){}\0",
                    label,
                    scheduled_ns,
                    measured_ns,
                    difference_ns,
                    cca_hint,
                );
            }
        } else {
            dot15d4::util::log::info!("  {}: Actual: {} ns{}\0", label, measured_ns, cca_hint,);
        }
    }
}

pub fn done() -> ! {
    loop {
        wfe();
    }
}
