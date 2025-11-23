//! This module provides a buffer allocator framework.

use core::{
    alloc::Layout,
    cell::UnsafeCell,
    marker::PhantomPinned,
    ops::{Deref, DerefMut},
    pin::Pin,
    ptr::NonNull,
    sync::atomic::{AtomicU8, Ordering},
};

use allocator_api2::alloc::{AllocError, Allocator};

use crate::tokens::TokenGuard;

// Re-export external dependencies required to use this module to facilitate
// dependency management.
pub mod export {
    pub use allocator_api2::alloc::{AllocError, Allocator};
    pub use static_cell::{ConstStaticCell, StaticCell};
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

/// A simple, multi-threaded buffer allocator backend providing a fixed number
/// (CAPACITY) of fixed-size buffers (BUFFER_SIZE). It is intended as a minimal
/// default that may be replaced by any allocator-api capable allocator backend
/// in production.
///
/// Buffers are managed by a stack of buffer pointers.
///
/// Allocator backends should provide static, re-usable zerocopy message buffers
/// to back any kind of zerocopy message.
///
/// Safety: As we need to hand out stable references to buffers, the allocator
///         must not be moved once it started handing out buffers. This is why
///         it is !Unpin and its methods can only be executed on a pinned static
///         instance. One way to achieve this is via
///         [`static_cell::ConstStaticCell::new()`].
///
/// This allocator backend is [`Sync`]. It may be used concurrently from clients
/// able to preempt each other. Allocator frontends built with this backend may
/// be cloned and copied freely, even across threads.
pub struct BufferAllocatorBackend<const BUFFER_SIZE: usize, const CAPACITY: usize> {
    buffers: UnsafeCell<[[u8; BUFFER_SIZE]; CAPACITY]>,
    /// Safety: The pointers will be self-references to buffers. But as we
    ///         enforce static lifetime and pinning (see [`Self::pin()`]), the
    ///         pointers will never be dangling.
    next: UnsafeCell<[u8; CAPACITY]>,
    head: AtomicU8,
    _pinned: PhantomPinned,
}

impl<const BUFFER_SIZE: usize, const CAPACITY: usize>
    BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>
{
    /// Initialize a new instance.
    ///
    /// To be able to do anything useful with it, you need to pass a static
    /// reference to the newly created instance to [`Self::pin()`].
    ///
    /// # Panics
    ///
    /// Panics if the buffer size is larger than [`u16::MAX`] or if the capacity
    /// is larger than [`u8::MAX`]. We restrict capacity to reasonable values so
    /// that the allocator can be implemented efficiently.
    pub const fn new() -> Self {
        // Buffer size is optimized for IEEE 802.15.4 frames.
        assert!(BUFFER_SIZE <= u16::MAX as usize);
        // We restrict capacity to save space in our free list.
        assert!(CAPACITY <= u8::MAX as usize);

        // Make the allocator eligible to be placed into .bss:
        // - value is all zero
        // - initializer is const
        // - no custom bit patterns
        // TODO: Test whether this actually works.
        Self {
            buffers: UnsafeCell::new([[0; BUFFER_SIZE]; CAPACITY]),
            next: UnsafeCell::new([0; CAPACITY]),
            head: AtomicU8::new(0),
            _pinned: PhantomPinned,
        }
    }

    /// Takes a fresh static mutable instance of the allocator, finalizes its
    /// initialization and returns an immutable pinned reference to it that can
    /// then be used to safely allocate buffers.
    ///
    /// Note: Initialization is done here rather than in
    ///       [`BufferAllocatorBackend::new()`] so that the allocator remains
    ///       eligible to be placed in .bss.
    pub fn pin(&'static mut self) -> Pin<&'static Self> {
        // We use `CAPACITY` as a sentinel to mark the end of the linked list as
        // it does not designate a valid index into the buffer array.
        for i in 0..CAPACITY {
            self.next.get_mut()[i] = (i + 1) as u8;
        }

        // Safety: The allocator is !Unpin. Pinning it allows us to generate
        //         stable buffer pointers for the whole lifetime of the
        //         allocator.
        Pin::static_ref(self)
    }

    // Safety: The index must be in range.
    #[inline]
    unsafe fn ptr_to_buffer(&self, index: usize) -> NonNull<[u8; BUFFER_SIZE]> {
        unsafe { NonNull::new_unchecked(self.buffers.get()) }
            .cast()
            .add(index)
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
{
    /// Allocates a zerocopy, 1-aligned message buffer with at least the given
    /// size.
    ///
    /// A call to this method has O(1) complexity under reasonable load.
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        debug_assert!(layout.size() <= BUFFER_SIZE && layout.align() == 1);

        let free_buffer = {
            let mut current = self.head.load(Ordering::Relaxed) as usize;
            loop {
                // A value of `CAPACITY` marks the end of the list, i.e. no
                // buffer is currently available.
                if current == CAPACITY {
                    return Err(AllocError);
                }

                // Safety: The `deallocate()` method will never write to a free
                //         slot. So all clients share read-only access to the
                //         current free slot.
                let next = unsafe { NonNull::new_unchecked(self.next.get()).as_ref() }[current];
                match self.head.compare_exchange_weak(
                    current as u8,
                    next,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break current,
                    Err(actual) => current = actual as usize,
                }
            }
        };

        // Safety: The buffer is now exclusively owned by the calling context
        //         and we use a checked index.
        Ok(unsafe { self.ptr_to_buffer(free_buffer) })
    }

    /// Release the given buffer for re-use.
    ///
    /// A call to this method has O(1) complexity under reasonable load.
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

        // Safety: See requirements in the method doc.
        let index = ptr
            .cast()
            .offset_from_unsigned(unsafe { self.ptr_to_buffer(0) });
        debug_assert!(index < CAPACITY);

        let mut current = self.head.load(Ordering::Relaxed);
        loop {
            // Safety:
            // - By the requirements of the method doc, the client has handed
            //   over exclusive ownership of the buffer at `index` by calling
            //   this method.
            // - Only `deallocate()` will write to slots owned by non-free
            //   buffers.
            (*unsafe { NonNull::new_unchecked(self.next.get()).as_mut() })[index] = current;
            match self.head.compare_exchange_weak(
                current,
                index as u8,
                Ordering::Release,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(actual) => current = actual,
            }
        }
    }
}

// Safety: See inline docs in `allocate()` and `deallocate()`.
unsafe impl<const BUFFER_SIZE: usize, const CAPACITY: usize> Sync
    for BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>
{
}

impl<const BUFFER_SIZE: usize, const CAPACITY: usize> Default
    for BufferAllocatorBackend<BUFFER_SIZE, CAPACITY>
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

/// A macro that relies on [`static_cell::StaticCell::init()`] to instantiate a
/// message buffer allocator.
#[macro_export]
macro_rules! buffer_allocator {
    ($size:expr, $capacity:expr) => {{
        use core::default::Default;
        use core::pin::Pin;
        use $crate::allocator::export::{ConstStaticCell, StaticCell};

        type AllocatorBackend = $crate::allocator::BufferAllocatorBackend<$size, $capacity>;
        static ALLOCATOR_BACKEND: ConstStaticCell<AllocatorBackend> =
            ConstStaticCell::new(AllocatorBackend::new());
        static ALLOCATOR: StaticCell<Pin<&'static AllocatorBackend>> = StaticCell::new();
        $crate::allocator::BufferAllocator::new(
            ALLOCATOR.init(ConstStaticCell::take(&ALLOCATOR_BACKEND).pin()),
        )
    }};
}

#[test]
fn test() {
    use crate::allocator::BufferAllocatorBackend;
    use static_cell::{ConstStaticCell, StaticCell};

    fn assert_is_thread_safe<Buffer: Send + Sync>(_buffer: &Buffer) {}

    fn consumer(buf: &mut [u8], i: u8) {
        buf[1] = i
    }

    type TestAllocator = BufferAllocatorBackend<3, 1>;
    static ALLOCATOR_BACKEND: ConstStaticCell<TestAllocator> =
        ConstStaticCell::new(BufferAllocatorBackend::new());
    static ALLOCATOR: StaticCell<Pin<&'static TestAllocator>> = StaticCell::new();
    let allocator_backend_ref = ALLOCATOR.init(ALLOCATOR_BACKEND.take().pin());
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
