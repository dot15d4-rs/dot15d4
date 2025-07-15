//! This module provides a buffer allocator framework.

use core::{
    alloc::Layout,
    array::from_fn,
    cell::{RefCell, UnsafeCell},
    future::poll_fn,
    marker::PhantomPinned,
    ops::{Deref, DerefMut},
    pin::Pin,
    ptr::{slice_from_raw_parts_mut, NonNull},
    task::{Context, Poll, Waker},
};

use allocator_api2::alloc::{AllocError, Allocator};
use generic_array::{ArrayLength, GenericArray};
use heapless::Deque;
use typenum::{Const, ToUInt, U};

use crate::tokens::TokenGuard;

// Re-export external dependencies required to use this module to facilitate
// dependency management.
pub mod export {
    pub use allocator_api2::alloc::{AllocError, Allocator};
    pub use generic_array::ArrayLength;
    pub use static_cell::{ConstStaticCell, StaticCell};
    pub use typenum::{Const, ToUInt, U};
}

#[allow(rustdoc::broken_intra_doc_links)]
/// A token representing a non-cloneable, zerocopy, 1-aligned byte buffer that
/// safely fakes static lifetime so that it can be passed around without
/// polluting channels' and other framework structures' lifetimes.
///
/// This buffer is intended to behave as a smart pointer wrapping a previously
/// allocated &'static mut [u8].
///
/// Other than in the case of [`allocator_api2::boxed::Box`] the contained
/// buffer is not automatically de-allocated when the token is dropped. The
/// token must be manually returned to the allocator from which it was
/// allocated. This allows us to keep channels' and other framework structures'
/// generics free from the allocator type and avoid the runtime cost of carrying
/// a reference to the allocator in our messages.
///
/// Safety:
///   - The token must not be dropped but needs to be returned manually to the
///     allocator from which it was allocated once it is no longer being used.
///   - The token does not implement into_inner() but only Deref/DerefMut.
///     Thereby it does not expose the wrapped reference unless it is restrained
///     by the lifetime of the token _and_ &mut XOR & remains enforced.
///   - As any token, it cannot be cloned.
///   - Due to behaving like a mutable reference to a primitive slice, the
///     buffer is [`Send`] and [`Sync`]. Using it on a different thread is safe
///     if it is returned to the allocator from the thread that allocated it
///     unless the allocator itself is thread safe.
///
/// The buffer represented by this token can be used to back zerocopy messages,
/// e.g. in the following use cases:
/// - The message can be sent over one or several channels with non-static
///   lifetime without ever having to copy it.
/// - The message can be converted between different representations without
///   copying the underlying buffer (e.g. to convert an MPDU to a low-level
///   driver frame and back)
/// - The message can be consumed and then re-instantiated from the same buffer
///   or a pointer derived from it due to the !Copy and "no-move" semantics
///   of this buffer.
#[derive(Debug, PartialEq, Eq)]
pub struct BufferToken(
    // Note: We keep a slice (fat pointer) rather than a reference to an array
    //       (thin pointer). This costs us extra bytes for a usize but allows us
    //       to allocate variable-sized buffers depending on the required buffer
    //       size w/o polluting generics.
    // Safety: static references never move, this allows us to safely convert
    //         this buffer to a pointer and back.
    &'static mut [u8],
    TokenGuard,
);

impl BufferToken {
    /// Creates a new buffer token.
    pub const fn new(buffer: &'static mut [u8]) -> Self {
        Self(buffer, TokenGuard)
    }

    /// Consume the token.
    ///
    /// # Safety
    ///
    /// Must be called by the same allocator from which the buffer token was
    /// allocated. Calling this from outside the allocator will leak the buffer.
    pub unsafe fn consume(self) -> &'static mut [u8] {
        self.1.consume();
        self.0
    }

    /// Const proxy for [u8]::len() to work around non-const deref limitation.
    pub const fn len(&self) -> usize {
        self.0.len()
    }

    /// Const proxy for [u8]::is_empty() to work around non-const deref limitation.
    pub const fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Const proxy for [u8].as_ptr() to work around non-const deref limitation.
    pub const fn as_ptr(&self) -> *const u8 {
        self.0.as_ptr()
    }

    /// Const proxy for [u8].as_mut_ptr() to work around non-const deref
    /// limitation.
    pub const fn as_mut_ptr(&mut self) -> *mut u8 {
        self.0.as_mut_ptr()
    }
}

impl Deref for BufferToken {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl DerefMut for BufferToken {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0
    }
}

/// Generic representation of a buffer-backed entity.
pub trait IntoBuffer {
    /// Consumes the entity and returns the underlying raw buffer.
    fn into_buffer(self) -> BufferToken;
}

/// A simple, single-threaded buffer allocator backend providing a fixed number
/// (CAPACITY) of fixed-size buffers (BUFFER_SIZE). It is intended as a minimal
/// default that may be replaced by any allocator-api capable allocator backend
/// in production.
///
/// Buffers are managed by a stack of buffer pointers.
///
/// Allocator backends should provide static, re-usable zerocopy message buffers
/// to back any kind of zerocopy message.
///
/// Safety: This allocator backend is not [`Sync`], therefore we don't have to
///         care about data races when mutating inner state. It is assumed that
///         a single, pinned local `&'static mut` reference to this allocator
///         backend will exist. One way to achieve this is via
///         [`static_cell::StaticCell::init()`]. Allocator frontends built with
///         this backend may be cloned and copied as long as they operate from a
///         single thread.
pub struct BufferAllocatorBackend<const BUFFER_SIZE: usize, const CAPACITY: usize>
where
    Const<CAPACITY>: ToUInt,
    <Const<CAPACITY> as ToUInt>::Output: ArrayLength,
    Const<BUFFER_SIZE>: ToUInt,
    <Const<BUFFER_SIZE> as ToUInt>::Output: ArrayLength,
{
    buffers: GenericArray<UnsafeCell<generic_array::GenericArray<u8, U<BUFFER_SIZE>>>, U<CAPACITY>>,
    /// Safety: The pointers will be self-references to buffers. But as we
    ///         enforce static lifetime and pinning (see [`Self::pin()`]), the
    ///         pointers will never be dangling.
    free_list: UnsafeCell<Deque<NonNull<u8>, CAPACITY>>,
    _pinned: PhantomPinned,
}

impl<const BUFFER_SIZE: usize, const CAPACITY: usize> BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>
where
    Const<CAPACITY>: ToUInt,
    <Const<CAPACITY> as ToUInt>::Output: ArrayLength,
    Const<BUFFER_SIZE>: ToUInt,
    <Const<BUFFER_SIZE> as ToUInt>::Output: ArrayLength,
{
    /// Initialize a new instance.
    ///
    /// To be able to do anything useful with it, you need to pass a static
    /// reference to the newly created instance to [`Self::pin()`].
    pub fn new() -> Self {
        Self {
            buffers: GenericArray::from_array::<CAPACITY>(from_fn(|_| {
                UnsafeCell::new(GenericArray::from_array([0; BUFFER_SIZE]))
            })),
            free_list: UnsafeCell::new(Deque::new()),
            _pinned: PhantomPinned,
        }
    }

    /// Takes a fresh static mutable instance of the allocator, finalizes its
    /// initialization and returns an immutable pinned reference to it that can
    /// then be used to safely allocate buffers.
    pub fn pin(&'static mut self) -> Pin<&'static Self> {
        // Safety: Self::new() moves data out of the constructor so we only can
        //         generate stable self-references once we have a static
        //         reference to self. We rely on the same guarantees that allow
        //         us to Pin::static_ref() in the end.
        let free_list = self.free_list.get_mut();
        for i in 0..CAPACITY {
            let buffer_ptr=
                // Safety: We enforce static lifetime of the allocator and the
                //         pointers are guaranteed to be non-null.
                unsafe { NonNull::new_unchecked(self.buffers[i].get_mut().as_mut_ptr()) };
            free_list.push_front(buffer_ptr).unwrap();
        }
        Pin::static_ref(self)
    }
}

/// Safety:
/// - Memory blocks returned from this allocator point to valid, statically
///   allocated memory and therefore retain their validity forever,
/// - The interface is implemented on a pinned version of the memory backend.
///   Cloning or moving the pinned reference does not invalidate memory blocks
///   issued from this allocator. All clones and copies of the pinned pointer
///   will access the same backend.
/// - Pointers to memory blocks will be used as memory block ids and may
///   therefore be passed to any method of the allocator.
unsafe impl<const BUFFER_SIZE: usize, const CAPACITY: usize> Allocator
    for Pin<&'static BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>>
where
    Const<CAPACITY>: ToUInt,
    <Const<CAPACITY> as ToUInt>::Output: ArrayLength,
    Const<BUFFER_SIZE>: ToUInt,
    <Const<BUFFER_SIZE> as ToUInt>::Output: ArrayLength,
{
    /// Allocates a zerocopy, 1-aligned message buffer with at least the given
    /// size.
    ///
    /// A call to this method has O(1) complexity.
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        if layout.size() > BUFFER_SIZE || layout.align() != 1 {
            return Err(AllocError);
        }

        // Safety: The cell was initialized with a valid deque, so the pointer
        //         to it is properly aligned, non-null and dereferenceable. We
        //         only ever mutate this field from within the struct itself.
        //         The struct is !Sync so we don't need to care about data
        //         races.
        let free_list = unsafe { &mut *self.free_list.get() };
        free_list
            .pop_front()
            .map(|buffer_ptr| unsafe {
                // Safety: We re-construct a pointer to one of our buffers that
                //         is guaranteed to be of length BUFFER_SIZE.
                NonNull::new_unchecked(slice_from_raw_parts_mut(buffer_ptr.as_ptr(), BUFFER_SIZE))
            })
            .ok_or(AllocError)
    }

    /// Release the given buffer for re-use.
    ///
    /// A call to this method has O(1) complexity.
    ///
    /// Safety: This must only ever be called when a message buffer currently
    ///         allocated from this pool is returned to the pool. The following
    ///         needs to be guaranteed:
    ///         1. The caller must possess and hand over exclusive ownership of
    ///            the buffer.
    ///         2. The given buffer must have been allocated from this allocator
    ///            and must point to one of its buffers. For performance
    ///            reasons, this is not being checked at runtime.
    ///         3. The layout must fit the given buffer. For performance
    ///            reasons, this is not being checked at runtime.
    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        debug_assert!(layout.size() <= BUFFER_SIZE && layout.align() == 1);

        // TODO: Add debug_assert!() checking that the incoming ptr lies within
        //       the allocated memory range.

        let free_list = unsafe { &mut *self.free_list.get() };
        free_list.push_front(ptr).unwrap();
    }
}

impl<const BUFFER_SIZE: usize, const CAPACITY: usize> Default
    for BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>
where
    Const<CAPACITY>: ToUInt,
    <Const<CAPACITY> as ToUInt>::Output: ArrayLength,
    Const<BUFFER_SIZE>: ToUInt,
    <Const<BUFFER_SIZE> as ToUInt>::Output: ArrayLength,
{
    fn default() -> Self {
        Self::new()
    }
}

/// A buffer allocator backed by any [`allocator_api2::alloc::Allocator`]
/// compatible allocator.
///
/// Currently we wrap our own allocator backend by default. Interesting future
/// candidates might be:
/// - <https://github.com/pcwalton/offset-allocator>
/// - <https://crates.io/crates/ring-alloc>
#[derive(Clone, Copy)]
pub struct BufferAllocator {
    allocator: &'static dyn Allocator,
}

impl BufferAllocator {
    /// Instantiates a new buffer allocator with the given allocator backend.
    ///
    /// Multiple instances can be created from the same allocator backend
    /// instance or even copies of it. It is safe to allocate a buffer from one
    /// instance and deallocate it from another. This is ensured by the
    /// clone-guarantee of the [`Allocator`] trait, i.e. a cloned allocator must
    /// behave as the same allocator.
    pub fn new(allocator: &'static dyn Allocator) -> Self {
        Self { allocator }
    }

    /// Tries to allocate a buffer with the given size from the backing
    /// allocator.
    ///
    /// If a buffer is returned it is guaranteed to be exactly of the requested
    /// size and safely mutable during the lifetime of the buffer token.
    pub fn try_allocate_buffer(&self, size: usize) -> Result<BufferToken, AllocError> {
        self.allocator
            .allocate(Self::buffer_layout(size))
            .map(|mut buffer_ptr| {
                BufferToken::new(
                    // Safety: Mutability and validity is guaranteed by the
                    //         allocator. The buffer is guaranteed to be at
                    //         least as long as requested. We limit it to the
                    //         requested size so that the buffer length can be
                    //         used in calculations.
                    unsafe { &mut buffer_ptr.as_mut()[0..size] },
                )
            })
    }

    /// Consumes and de-allocates the given buffer token. Returns the buffer to
    /// the backing allocator.
    ///
    /// The token approach is a conscious trade-off between safety, practicality
    /// and runtime cost.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the given token was generated by this allocator
    /// instance. We could enforce this by keeping some identifier in the token
    /// (e.g. an allocator id or a pointer to the allocator instance). But we
    /// want to avoid the runtime cost of doing so and assume that the allocator
    /// itself will check for buffer validity if necessary.
    pub unsafe fn deallocate_buffer(&self, buffer_token: BufferToken) {
        let buffer = buffer_token.consume();
        self.allocator.deallocate(
            // Safety: We ensure the non-null invariants when creating the
            //         token.
            NonNull::new_unchecked(buffer.as_mut_ptr()),
            Self::buffer_layout(buffer.len()),
        );
    }

    const fn buffer_layout(size: usize) -> Layout {
        // Safety: The size will be checked by the allocator. An alignment of
        //         one is valid for a byte buffer.
        unsafe { Layout::from_size_align_unchecked(size, 1) }
    }
}

/// A list of wakers required by the [`AsyncBufferAllocator`] to build a backlog
/// of clients waiting for buffer capacity. This structure will typically be
/// allocated statically and must not be cloned or copied.
///
/// This backlog is not synchronized, so it must be used from a single executor
/// (thread).
pub struct BufferAllocatorBacklog<const CAPACITY: usize>
where
    Const<CAPACITY>: ToUInt,
    U<CAPACITY>: ArrayLength,
{
    /// The waker list deque supports O(1) access. Wakers will be called in the
    /// order they were stored, when buffer capacity becomes available.
    wakers: RefCell<Deque<Waker, CAPACITY>>,
}

impl<const CAPACITY: usize> BufferAllocatorBacklog<CAPACITY>
where
    Const<CAPACITY>: ToUInt,
    U<CAPACITY>: ArrayLength,
{
    pub const fn new() -> Self {
        Self {
            wakers: RefCell::new(Deque::new()),
        }
    }

    /// Stores the given waker. Panics if the backlog capacity is exhausted.
    fn push_waker(&self, waker: Waker) {
        self.wakers
            .borrow_mut()
            .push_front(waker)
            .expect("no capacity")
    }

    /// If one or more wakers are waiting then the waker that waits for the
    /// longest time is returned.
    fn try_pop_waker(&self) -> Option<Waker> {
        self.wakers.borrow_mut().pop_back()
    }
}

impl<const CAPACITY: usize> Default for BufferAllocatorBacklog<CAPACITY>
where
    Const<CAPACITY>: ToUInt,
    U<CAPACITY>: ArrayLength,
{
    fn default() -> Self {
        Self::new()
    }
}

/// An asynchronous version of the [`BufferAllocator`].
///
/// The allocator allows clients to wait for buffer capacity under heavy load.
///
/// Note: For best performance you should reserve enough buffer capacity that
///       even under heavy load buffer capacity is not contended. To prove so,
///       you can set the backlog capacity to zero and/or use the synchronous
///       API of the allocator.
///
/// Currently we wrap our own allocator backend by default. Interesting future
/// candidates might be:
/// - <https://github.com/pcwalton/offset-allocator>
/// - <https://crates.io/crates/ring-alloc>
#[derive(Clone, Copy)]
pub struct AsyncBufferAllocator<const BACKLOG: usize>
where
    Const<BACKLOG>: ToUInt,
    U<BACKLOG>: ArrayLength,
{
    allocator: BufferAllocator,
    backlog: &'static BufferAllocatorBacklog<BACKLOG>,
}

impl<const NUM_CLIENTS: usize> AsyncBufferAllocator<NUM_CLIENTS>
where
    Const<NUM_CLIENTS>: ToUInt,
    U<NUM_CLIENTS>: ArrayLength,
{
    /// Instantiates a new asynchronous buffer allocator based on the given
    /// synchronous allocator.
    ///
    /// Multiple instances can be created from the same [`BufferAllocator`]
    /// instance or even copies of it. It is safe to allocate a buffer from one
    /// instance and deallocate it from another. This is ensured by the
    /// clone-guarantee of the [`BufferAllocator`].
    pub fn new(
        allocator: BufferAllocator,
        backlog: &'static BufferAllocatorBacklog<NUM_CLIENTS>,
    ) -> Self {
        Self { allocator, backlog }
    }

    /// Provides access to the underlying synchronous buffer allocator.
    pub fn allocator(&self) -> BufferAllocator {
        self.allocator
    }

    /// A proxy for [`BufferAllocator::try_allocate_buffer()`] with the
    /// additional option to register a waker if a buffer cannot be allocated.
    pub fn try_allocate_buffer(
        &self,
        size: usize,
        cx: Option<&Context>,
    ) -> Result<BufferToken, AllocError> {
        self.allocator.try_allocate_buffer(size).inspect_err(|_| {
            if let Some(cx) = cx {
                self.backlog.push_waker(cx.waker().clone());
            }
        })
    }

    /// Waits until a buffer with the given size is available from the backing
    /// allocator, then allocates it and returns it.
    ///
    /// The returned buffer is guaranteed to be exactly of the requested size
    /// and safely mutable during the lifetime of the buffer token.
    ///
    /// Note: Using this method introduces a risk of deadlock unless you ensure
    ///       that at least one task owning and willing to release a buffer is
    ///       life when blocking on this method. E.g. if you hold a buffer
    ///       allocation in one task and then block waiting for a message slot
    ///       on a channel and at the same time you hold a message slot for that
    ///       channel in another task and then block waiting for a buffer there,
    ///       you risk deadlock under heavy load. To avoid deadlock always
    ///       allocate scarce resources in the exactly same order in all tasks.
    pub async fn allocate_buffer(&self, size: usize) -> BufferToken {
        poll_fn(|cx| match self.allocator.try_allocate_buffer(size) {
            Ok(buffer) => Poll::Ready(buffer),
            Err(_) => {
                self.backlog.push_waker(cx.waker().clone());
                Poll::Pending
            }
        })
        .await
    }

    /// See [`BufferAllocator::deallocate_buffer()`]
    ///
    /// # Safety
    ///
    /// See the safety section in [`BufferAllocator::deallocate_buffer()`].
    pub unsafe fn deallocate_buffer(&self, buffer_token: BufferToken) {
        self.allocator.deallocate_buffer(buffer_token);

        // If a client waits for buffers, then wake it up.
        self.backlog
            .try_pop_waker()
            .inspect(|waker| waker.wake_by_ref());
    }
}

/// A macro that relies on [`static_cell::StaticCell::init()`] to instantiate a
/// message buffer allocator.
#[macro_export]
macro_rules! buffer_allocator {
    ($size:expr, $capacity:expr) => {{
        use core::default::Default;
        use core::pin::Pin;
        use $crate::allocator::export::StaticCell;

        type AllocatorBackend = $crate::allocator::BufferAllocatorBackend<$size, $capacity>;
        static ALLOCATOR_BACKEND: StaticCell<AllocatorBackend> = StaticCell::new();
        static ALLOCATOR: StaticCell<Pin<&'static AllocatorBackend>> = StaticCell::new();
        $crate::allocator::BufferAllocator::new(
            ALLOCATOR.init(ALLOCATOR_BACKEND.init(Default::default()).pin()),
        )
    }};
}

/// A macro that relies on [`static_cell::StaticCell::init()`] to instantiate an
/// asynchronous message buffer allocator.
#[macro_export]
macro_rules! async_buffer_allocator {
    ($size:expr, $buffers:expr, $backlog: expr) => {{
        use $crate::export::ConstStaticCell;

        static ALLOCATOR_BACKLOG: ConstStaticCell<$crate::BufferAllocatorBacklog<$backlog>> =
            ConstStaticCell::new($crate::BufferAllocatorBacklog::new());
        let buffer_allocator = $crate::buffer_allocator!($size, $buffers);
        $crate::AsyncBufferAllocator::new(buffer_allocator, ALLOCATOR_BACKLOG.take())
    }};
}

#[test]
fn test() {
    use crate::allocator::BufferAllocatorBackend;
    use static_cell::StaticCell;

    fn assert_is_thread_safe<Buffer: Send + Sync>(_buffer: &Buffer) {}

    fn consumer(buf: &mut [u8], i: u8) {
        buf[1] = i
    }

    static ALLOCATOR: StaticCell<BufferAllocatorBackend<3, 1>> = StaticCell::new();
    static ALLOCATOR_BACKEND: StaticCell<Pin<&'static BufferAllocatorBackend<3, 1>>> =
        StaticCell::new();
    let allocator_backend_ref =
        ALLOCATOR_BACKEND.init(ALLOCATOR.init(BufferAllocatorBackend::new()).pin());
    let allocator = BufferAllocator::new(allocator_backend_ref);

    for i in 0..100 {
        let mut buf = allocator.try_allocate_buffer(3).expect("out of memory");
        assert_is_thread_safe(&buf);
        buf[0] = i;
        consumer(&mut buf, i);
        buf[2] = i;
        unsafe { allocator.deallocate_buffer(buf) };
    }
}
