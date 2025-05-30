#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SeqNrRepr {
    /// A sequence number is present in the frame.
    Yes,
    /// The sequence number is suppressed.
    No,
} // 1 byte

impl SeqNrRepr {
    pub const fn length(&self) -> u16 {
        match self {
            SeqNrRepr::Yes => 1,
            SeqNrRepr::No => 0,
        }
    }
}
