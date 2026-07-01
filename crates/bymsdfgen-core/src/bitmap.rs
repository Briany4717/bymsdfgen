//! Owning, channel-generic pixel buffer. Port of `core/Bitmap.h` + `BitmapRef.hpp`.
//!
//! Uses const generics for the channel count `N`, so `Bitmap<f32, 3>` and
//! `Bitmap<f32, 1>` are distinct monomorphized types with no per-pixel branching.
//! Access goes through bounds-checked slices instead of the original's raw pointer
//! arithmetic — in release builds the checks vanish for in-range indices.
//!
//! Pixels are stored row-major with row `0` at the bottom (Y points upward, the
//! msdfgen default). Output orientation is applied when saving.

/// An owned `width × height` bitmap with `N` channels per pixel of type `T`.
#[derive(Debug, Clone)]
pub struct Bitmap<T, const N: usize> {
    pub width: usize,
    pub height: usize,
    data: Vec<T>,
}

impl<T: Copy + Default, const N: usize> Bitmap<T, N> {
    /// Allocate a zero-initialized bitmap.
    pub fn new(width: usize, height: usize) -> Self {
        Bitmap {
            width,
            height,
            data: vec![T::default(); width * height * N],
        }
    }
}

impl<T, const N: usize> Bitmap<T, N> {
    #[inline]
    fn offset(&self, x: usize, y: usize) -> usize {
        N * (self.width * y + x)
    }

    /// Immutable view of pixel `(x, y)` as an `N`-channel slice.
    #[inline]
    pub fn pixel(&self, x: usize, y: usize) -> &[T] {
        let o = self.offset(x, y);
        &self.data[o..o + N]
    }

    /// Mutable view of pixel `(x, y)` as an `N`-channel slice.
    #[inline]
    pub fn pixel_mut(&mut self, x: usize, y: usize) -> &mut [T] {
        let o = self.offset(x, y);
        &mut self.data[o..o + N]
    }

    /// The whole backing buffer (row-major, `N` channels interleaved).
    #[inline]
    pub fn data(&self) -> &[T] {
        &self.data
    }

    #[inline]
    pub fn data_mut(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// Iterate rows as `(y, row_slice)` where `row_slice` has `width*N` elements.
    pub fn rows_mut(&mut self) -> impl Iterator<Item = (usize, &mut [T])> {
        let stride = self.width * N;
        self.data.chunks_mut(stride).enumerate()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alloc_and_access() {
        let mut b: Bitmap<f32, 3> = Bitmap::new(4, 2);
        assert_eq!(b.data().len(), 4 * 2 * 3);
        b.pixel_mut(1, 1).copy_from_slice(&[0.1, 0.2, 0.3]);
        assert_eq!(b.pixel(1, 1), &[0.1, 0.2, 0.3]);
    }
}
