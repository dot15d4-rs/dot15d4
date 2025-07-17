#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum SeqNrRepr {
    /// The sequence number is suppressed.
    No,
    /// A sequence number is present in the frame.
    Yes,
} // 1 byte

impl SeqNrRepr {
    pub const fn length(&self) -> u16 {
        match self {
            SeqNrRepr::No => 0,
            SeqNrRepr::Yes => 1,
        }
    }
}
