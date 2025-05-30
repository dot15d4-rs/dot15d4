use heapless::Deque;

/// Helper struct that allows us to remove an inner value from [`Deque`]
/// represented as the return value of [`Deque::as_mut_slices()`].
pub(super) struct DequeWrapper<'a, const N: usize>(&'a mut Deque<u8, N>);

impl<'a, const N: usize> DequeWrapper<'a, N> {
    pub(super) fn new(deque: &'a mut Deque<u8, N>) -> Self {
        Self(deque)
    }

    /// Moves each entry in the slice up to the (index-1)'th entry one position
    /// to the back, so that the front entry becomes empty and can be removed.
    ///
    /// Note: This is O(n) right now, so we probably want a different
    ///       implementation in the long run.
    pub(super) fn remove(&mut self, index: usize) {
        let (first, second) = self.0.as_mut_slices();

        // TODO: Check whether this loop needs to be optimized.
        for i in (1..=index).rev() {
            let prev = Self::get(first, second, i - 1);
            Self::set(first, second, i, prev);
        }

        self.0.pop_front();
    }

    fn get(first: &[u8], second: &[u8], index: usize) -> u8 {
        let len_of_first = first.len();
        if index < len_of_first {
            first[index]
        } else {
            second[index - len_of_first]
        }
    }

    fn set(first: &mut [u8], second: &mut [u8], index: usize, val: u8) {
        let len_of_first = first.len();
        if index < len_of_first {
            first[index] = val;
        } else {
            second[index - len_of_first] = val;
        }
    }
}
