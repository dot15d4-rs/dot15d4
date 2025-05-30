#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SecurityLevelRepr {
    Mic32,
    Mic64,
    Mic128,
    EncMic32,
    EncMic64,
    EncMic128,
} // 1 byte

impl SecurityLevelRepr {
    /// MIC length in bytes.
    pub const fn mic_length(&self) -> u16 {
        match self {
            SecurityLevelRepr::Mic32 | SecurityLevelRepr::EncMic32 => 4, // 32 bits
            SecurityLevelRepr::Mic64 | SecurityLevelRepr::EncMic64 => 8, // 64 bits
            SecurityLevelRepr::Mic128 | SecurityLevelRepr::EncMic128 => 16, // 128 bits
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum KeyIdRepr {
    Implicit,
    SourceNone,
    Source4Byte,
    Source8Byte,
} // 1 byte

impl KeyIdRepr {
    pub const fn key_id_length(&self) -> u16 {
        match self {
            KeyIdRepr::Implicit => 0,
            KeyIdRepr::SourceNone => 1,
            KeyIdRepr::Source4Byte => 5,
            KeyIdRepr::Source8Byte => 9,
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct SecurityRepr {
    tsch_mode: bool,
    security_level: SecurityLevelRepr,
    key_id: KeyIdRepr,
} // 3 bytes

impl SecurityRepr {
    pub const fn new(
        tsch_mode: bool,
        security_level: SecurityLevelRepr,
        key_id: KeyIdRepr,
    ) -> Self {
        Self {
            tsch_mode,
            security_level,
            key_id,
        }
    }

    pub const fn aux_sec_header_length(&self) -> u16 {
        const SECURITY_CONTROL_LENGTH: u16 = 1;

        let frame_counter_len = if self.tsch_mode { 0 } else { 4 };
        SECURITY_CONTROL_LENGTH + frame_counter_len + self.key_id.key_id_length()
    }

    pub const fn mic_length(&self) -> u16 {
        self.security_level.mic_length()
    }
}
