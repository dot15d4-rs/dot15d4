use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::asm::wfe;
use dot15d4::{
    driver::radio::{
        frame::{
            Address, AddressingMode, AddressingRepr, ExtendedAddress, FrameType, FrameVersion,
            PanId, RadioFrame,
        },
        phy::PhyConfig,
        tasks::{RadioTask, RadioTransitionResult, TaskRx, TaskTx},
        DriverConfig, PhyOf,
    },
    mac::frame::{
        repr::{MpduRepr, SeqNrRepr},
        MpduWithIes,
    },
    util::allocator::BufferAllocator,
};

#[cfg(any(feature = "defmt", feature = "log"))]
use dot15d4::driver::{radio::HighPrecisionTimerOf, timer::HighPrecisionTimer};

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

pub fn log_timing<Config: DriverConfig, PrevTask: RadioTask, ThisTask: RadioTask>(
    _label: &str,
    _radio_transition_result: &RadioTransitionResult<Config, PrevTask, ThisTask>,
    _cca: bool,
) {
    #[cfg(any(feature = "defmt", feature = "log"))]
    {
        let cca_hint = if _cca { " with CCA" } else { "" };

        let RadioTransitionResult {
            scheduled_entry,
            measured_entry,
            ..
        } = _radio_transition_result;
        let (scheduled_ns, measured_ns) = (
            scheduled_entry.map(|ts| ts.ticks()).unwrap_or(0),
            measured_entry.ticks(),
        );
        let difference_ns = if scheduled_ns > 0 {
            measured_ns as i64 - scheduled_ns as i64
        } else {
            0
        };

        // We allow +/- one high precision clock tick offset between scheduled and
        // measured timestamps.
        let tolerance_ns =
            <HighPrecisionTimerOf<Config> as HighPrecisionTimer>::TICK_PERIOD.ticks();
        if scheduled_ns != 0 {
            if difference_ns.unsigned_abs() <= tolerance_ns {
                dot15d4::util::log::info!(
                    "{}{}: Scheduled: {} ns, Actual: {} ns, (Error: {} ns)\0",
                    _label,
                    cca_hint,
                    scheduled_ns,
                    measured_ns,
                    difference_ns
                );
            } else {
                dot15d4::util::log::error!(
                    "{}{}: Scheduled: {} ns, Actual: {} ns, (Error: {} ns)\0",
                    _label,
                    cca_hint,
                    scheduled_ns,
                    measured_ns,
                    difference_ns
                );
            }
        } else {
            dot15d4::util::log::info!("{}{}: Actual: {} ns\0", _label, cca_hint, measured_ns,);
        }
    }
}

#[cfg(feature = "terminal")]
pub mod terminal {
    use dot15d4::util::{error, info, rtt::RTT_SYNC_BUF_LEN};
    use rtt_target::DownChannel;

    enum TerminalCommand {
        Start,
        Unknown,
        Invalid,
    }

    pub struct Terminal {
        sync_in: DownChannel,
        read_idx: usize,
        write_idx: usize,
        len: usize,
        buf: [u8; RTT_SYNC_BUF_LEN],
    }

    impl Terminal {
        pub fn new(sync_in: DownChannel) -> Self {
            Self {
                sync_in,
                read_idx: 0,
                write_idx: 0,
                len: 0,
                buf: [0; RTT_SYNC_BUF_LEN],
            }
        }

        fn buffer_is_full(&self) -> bool {
            self.len == self.buf.len()
        }

        fn buffer_capacity(&self) -> usize {
            self.buf.len()
        }

        fn reset(&mut self) {
            self.read_idx = 0;
            self.write_idx = 0;
            self.len = 0;
        }

        fn consume(&mut self, num_bytes: usize) {
            self.read_idx += num_bytes;
            self.read_idx %= self.buffer_capacity();
            self.len -= num_bytes;
        }

        fn read(&mut self) -> Result<usize, ()> {
            if self.buffer_is_full() {
                error!("Received invalid command.");
                self.reset();
                return Err(());
            }

            let writable_range = if self.write_idx >= self.read_idx {
                // Note: Equality means "empty" because we just checked that the
                //       buffer is not full.
                self.write_idx..RTT_SYNC_BUF_LEN
            } else {
                self.write_idx..self.read_idx
            };
            let remaining_buffer = &mut self.buf[writable_range];

            let bytes_read = self.sync_in.read(remaining_buffer);
            self.write_idx += bytes_read % self.buffer_capacity();
            self.len += bytes_read;

            Ok(bytes_read)
        }

        fn read_line(&mut self) -> Result<usize, ()> {
            let mut start = self.read_idx;
            let mut line_len = 0;
            loop {
                let bytes_read = self.read()?;
                if bytes_read == 0 {
                    continue;
                }

                let end = start + bytes_read;

                if let Some(index) = self.buf[start..end]
                    .iter()
                    .enumerate()
                    .find(|(_, &byte)| byte == b'\n')
                    .map(|(index, _)| index)
                {
                    line_len += index + 1;
                    return Ok(line_len);
                }

                start = end % self.buffer_capacity();
                line_len += bytes_read;
            }
        }

        fn read_command(&mut self) -> TerminalCommand {
            let line_len = if let Ok(line_len) = self.read_line() {
                // Don't compare newline character.
                line_len - 1
            } else {
                error!("Received invalid command.");
                return TerminalCommand::Invalid;
            };

            'command: for command in [TerminalCommand::Start] {
                let command_as_bytes = match command {
                    TerminalCommand::Start => b"start".as_slice(),
                    _ => unreachable!(),
                };

                let mut cursor = self.read_idx;
                let mut idx = 0;
                while idx < line_len {
                    if idx >= command_as_bytes.len() || self.buf[cursor] != command_as_bytes[idx] {
                        continue 'command;
                    }

                    idx += 1;
                    cursor = (cursor + 1) % self.buffer_capacity();
                }

                self.consume(line_len + 1);
                info!("< {}", unsafe {
                    str::from_utf8_unchecked(command_as_bytes)
                });
                return command;
            }

            self.consume(line_len + 1);
            error!("Received unknown command.");
            TerminalCommand::Unknown
        }

        pub fn start(mut self) -> Self {
            info!("> ready");
            match self.read_command() {
                TerminalCommand::Start => {}
                _ => super::done(),
            }
            self
        }

        pub fn done(self) {
            info!("> done");
        }
    }
}

pub fn done() -> ! {
    loop {
        wfe();
    }
}
