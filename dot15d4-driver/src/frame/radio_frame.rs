use core::{fmt::Debug, marker::PhantomData, mem, num::NonZero, ops::Range, ptr};

use dot15d4_util::{
    allocator::{BufferAllocator, BufferToken, IntoBuffer},
    frame::{Frame, FramePdu},
};

use crate::{
    frame::{AddressingRepr, FrameControl},
    DriverConfig,
};

use super::{RadioFrameRepr, RadioFrameSized, RadioFrameUnsized};

/// Provides a simple default radio frame implementation with an externally
/// allocated buffer.
///
/// Note: This frame must not be dropped as it contains a non-droppable buffer.
#[derive(Debug, PartialEq, Eq)]
#[must_use = "Must recover the contained buffer after use."]
pub struct RadioFrame<State> {
    /// Points to the first byte of the frame's SDU.
    headroom: u8,
    /// Points to the FCS if the FCS is not managed by the driver (and therefore
    /// part of the tailroom). Will be the same as tailroom offset if the FCS is
    /// managed by the driver.
    ///
    /// In unsized state this will point to the offset of an FCS for the largest
    /// possible SDU. In sized state this will point to the actual FCS.
    offset_fcs: NonZero<u16>,
    /// The length of the FCS if the FCS is managed by upper layers. Will be
    /// zero when the FCS is managed by the driver.
    length_fcs: u8,

    /// The buffer allocated for the frame.
    ///
    /// Safety: For a sized radio frame, the buffer's capacity SHALL be greater
    ///         or equal [`RadioFrameRepr::pdu_length()`], for an unsized
    ///         frame, it must be at least
    ///         [`RadioFrameRepr::max_buffer_length()`].
    buffer: BufferToken,

    state: PhantomData<State>,
}

impl<State> IntoBuffer for RadioFrame<State> {
    fn into_buffer(self) -> BufferToken {
        self.buffer
    }
}

impl<State> RadioFrame<State> {
    pub const fn headroom_length(&self) -> u8 {
        self.headroom
    }

    pub const fn as_ptr(&self) -> *const u8 {
        self.buffer.as_ptr()
    }

    pub const fn as_mut_ptr(&mut self) -> *mut u8 {
        self.buffer.as_mut_ptr()
    }

    /// Retrieve frame information from an outgoing radio frame or an incoming
    /// frame that has been received at least up to and including the frame
    /// control field.
    ///
    /// # Safety
    ///
    /// This function SHALL only be called once the frame control field has been
    /// populated and is present in the initial bytes of the frame buffer.
    pub unsafe fn fc(&self) -> FrameControl<[u8; 2]> {
        let fc_start = self.headroom as isize;
        let fc_ptr = self.buffer.as_ref().as_ptr().offset(fc_start);
        FrameControl::new_unchecked([
            ptr::read_volatile(fc_ptr),
            ptr::read_volatile(fc_ptr.offset(1)),
        ])
    }

    /// Retrieve frame and addressing information from an outgoing radio frame
    /// or an incoming frame that has been received at least up to and including
    /// the frame control field.
    ///
    /// # Safety
    ///
    /// See [`Self::frame_control()`].
    pub unsafe fn fc_and_addressing_repr(&self) -> (FrameControl<[u8; 2]>, Option<AddressingRepr>) {
        let fc = self.fc();
        let addressing_repr = AddressingRepr::from_frame_control(fc.clone()).unwrap_or_default();
        (fc, addressing_repr)
    }

    pub fn max_frame_length_wo_fcs(&self) -> u16 {
        self.offset_fcs.get() - self.headroom_length() as u16
    }
}

impl RadioFrame<RadioFrameUnsized> {
    pub const fn new<Config: DriverConfig>(buffer: BufferToken) -> Self {
        let repr = RadioFrameRepr::<Config, RadioFrameUnsized>::new();
        debug_assert!(buffer.len() <= repr.max_buffer_length() as usize);
        Self {
            headroom: repr.headroom_length(),
            // Safety: The max SDU length is always greater than zero.
            offset_fcs: unsafe {
                NonZero::new_unchecked(repr.headroom_length() as u16 + repr.max_sdu_length_wo_fcs())
            },
            length_fcs: repr.fcs_length(),
            buffer,
            state: PhantomData,
        }
    }

    pub fn with_size(self, sdu_length_wo_fcs: NonZero<u16>) -> RadioFrame<RadioFrameSized> {
        debug_assert!(self.offset_fcs.get() - self.headroom as u16 >= sdu_length_wo_fcs.get());
        RadioFrame {
            headroom: self.headroom,
            offset_fcs: sdu_length_wo_fcs.saturating_add(self.headroom as u16),
            length_fcs: self.length_fcs,
            buffer: self.buffer,
            state: PhantomData,
        }
    }
}

#[allow(dead_code)]
impl RadioFrame<RadioFrameSized> {
    fn offset_tailroom(&self) -> NonZero<u16> {
        unsafe { NonZero::new_unchecked(self.offset_fcs.get() + self.length_fcs as u16) }
    }

    pub fn sdu_length(&self) -> NonZero<u16> {
        // Safety: We had a non-zero SDU length when creating this struct.
        unsafe {
            NonZero::new_unchecked(
                self.offset_fcs.get() + self.length_fcs as u16 - self.headroom as u16,
            )
        }
    }

    pub const fn sdu_wo_fcs_length(&self) -> NonZero<u16> {
        // Safety: We had a non-zero SDU length when creating this struct.
        unsafe { NonZero::new_unchecked(self.offset_fcs.get() - self.headroom as u16) }
    }

    pub fn forget_size<Config: DriverConfig>(self) -> RadioFrame<RadioFrameUnsized> {
        RadioFrame::new::<Config>(self.into_buffer())
    }

    /// Parse the frame control field of the radio frame.
    pub fn frame_control(&self) -> FrameControl<[u8; 2]> {
        // Safety: This is a sized frame, so it must have a frame control field.
        unsafe { self.fc() }
    }

    /// If ACK is requested for this frame, then return the sequence number of
    /// the frame, otherwise return [`None`].
    pub fn ack_seq_num(&self) -> Option<u8> {
        const FC_LEN: usize = 2;

        let fc = self.frame_control();
        if !fc.ack_request() || fc.sequence_number_suppression() {
            return None;
        }

        let seq_nr_index = self.headroom as usize + FC_LEN;
        Some(self.buffer.as_ref()[seq_nr_index])
    }

    fn headroom_range(&self) -> Option<Range<usize>> {
        if self.headroom == 0 {
            return None;
        }

        Some(0..self.headroom as usize)
    }

    fn sdu_range(&self) -> Range<usize> {
        self.headroom as usize..self.offset_tailroom().get() as usize
    }

    fn sdu_range_wo_fcs(&self) -> Range<usize> {
        self.headroom as usize..self.offset_fcs.get() as usize
    }

    fn fcs_range(&self) -> Option<Range<usize>> {
        if self.length_fcs == 0 {
            return None;
        }

        let offset_fcs = self.offset_fcs.get() as usize;
        let offset_tailroom = self.offset_tailroom().get() as usize;
        Some(offset_fcs..offset_tailroom)
    }

    fn tailroom_range(&self) -> Option<Range<usize>> {
        let offset_tailroom = self.offset_tailroom().get() as usize;
        let buffer_len = self.buffer.len();
        if offset_tailroom >= buffer_len {
            return None;
        }

        Some(offset_tailroom..buffer_len)
    }
}

impl<State> FramePdu for RadioFrame<State> {
    type Pdu = [u8];

    fn pdu_ref(&self) -> &[u8] {
        self.buffer.as_ref()
    }

    fn pdu_mut(&mut self) -> &mut [u8] {
        self.buffer.as_mut()
    }
}

impl Frame for RadioFrame<RadioFrameSized> {
    fn sdu_ref(&self) -> &[u8] {
        &self.buffer[self.sdu_range_wo_fcs()]
    }

    fn sdu_mut(&mut self) -> &mut [u8] {
        let sdu_range_wo_fcs = self.sdu_range_wo_fcs();
        &mut self.buffer[sdu_range_wo_fcs]
    }
}

/// A droppable version of the radio frame that automatically de-allocates the
/// contained buffer when dropped.
///
/// Note: Use the non-droppable version if possible as it is smaller and allows
///       for zero-allocation buffer re-use. This version mostly exists as it
///       can be conveniently moved into cancellable futures without leaking the
///       buffer.
///
/// TODO: Consider alternatives:
/// - We could implement a generic "DroppableBuffer" and either make the frame
///   generic over the buffer (with a non-droppable buffer as default) or
///   implement the DroppableRadioFrame based on a droppable buffer. This makes
///   sense as soon as we need a droppable buffer elsewhere.
/// - We could implement a "RecoverableRadioFrame" that uses
///   Cell<Option<BufferToken>> internally. It can be passed around by reference
///   which allows the allocating call site to retain ownership while also
///   passing shared ownership (via interior mutability) to the callee. This
///   version would have to implement a method "try_into_buffer()" That recovers
///   the buffer unless the callee had already consumed it.
pub struct DroppableRadioFrame<State> {
    /// Safety: The option must always be [`Some`] until dropped.
    inner: Option<RadioFrame<State>>,
    allocator: BufferAllocator,
}

impl<State> Drop for DroppableRadioFrame<State> {
    fn drop(&mut self) {
        // Safety: The radio frame owns the buffer token which can only be
        //         re-surfaced by consuming the frame itself. Therefore, when
        //         the frame is dropped, it has exclusive access to the token.
        unsafe {
            self.allocator
                .deallocate_buffer(self.inner.take().unwrap().into_buffer());
        }
    }
}

impl<State: PartialEq> PartialEq for DroppableRadioFrame<State> {
    fn eq(&self, other: &Self) -> bool {
        self.inner == other.inner
    }
}

impl<State: Eq> Eq for DroppableRadioFrame<State> {}

impl<State> IntoBuffer for DroppableRadioFrame<State> {
    fn into_buffer(mut self) -> BufferToken {
        // Safety: The option is always Some until dropped.
        let buffer = self.inner.take().unwrap().into_buffer();

        // We re-use the buffer, so it must no longer be de-allocated on drop.
        mem::forget(self);

        buffer
    }
}

impl<State: Debug> Debug for DroppableRadioFrame<State> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DroppableRadioFrame")
            .field("inner", &self.inner)
            .finish()
    }
}

impl<State> DroppableRadioFrame<State> {
    pub fn headroom_length(&self) -> u8 {
        self.inner.as_ref().unwrap().headroom_length()
    }

    pub const fn as_ptr(&self) -> *const u8 {
        self.inner.as_ref().unwrap().as_ptr()
    }

    pub const fn as_mut_ptr(&mut self) -> *mut u8 {
        self.inner.as_mut().unwrap().as_mut_ptr()
    }

    pub fn into_non_droppable_frame(mut self) -> RadioFrame<State> {
        let frame = self.inner.take().unwrap();

        // We re-use the frame, so its buffer must no longer be de-allocated on
        // drop.
        mem::forget(self);

        frame
    }
}

impl DroppableRadioFrame<RadioFrameUnsized> {
    pub fn new<Config: DriverConfig>(buffer: BufferToken, allocator: BufferAllocator) -> Self {
        let inner = Some(RadioFrame::<RadioFrameUnsized>::new::<Config>(buffer));
        Self { inner, allocator }
    }

    pub fn with_size(
        mut self,
        sdu_length_wo_fcs: NonZero<u16>,
    ) -> DroppableRadioFrame<RadioFrameSized> {
        let allocator = self.allocator;
        let inner = self.inner.take().unwrap();
        mem::forget(self);

        let inner = Some(inner.with_size(sdu_length_wo_fcs));
        DroppableRadioFrame { inner, allocator }
    }
}

impl DroppableRadioFrame<RadioFrameSized> {
    pub fn sdu_length(&self) -> NonZero<u16> {
        self.inner.as_ref().unwrap().sdu_length()
    }
}

impl<State> FramePdu for DroppableRadioFrame<State> {
    type Pdu = [u8];

    fn pdu_ref(&self) -> &[u8] {
        self.inner.as_ref().unwrap().pdu_ref()
    }

    fn pdu_mut(&mut self) -> &mut [u8] {
        self.inner.as_mut().unwrap().pdu_mut()
    }
}

impl Frame for DroppableRadioFrame<RadioFrameSized> {
    fn sdu_ref(&self) -> &[u8] {
        self.inner.as_ref().unwrap().sdu_ref()
    }

    fn sdu_mut(&mut self) -> &mut [u8] {
        self.inner.as_mut().unwrap().sdu_mut()
    }
}
