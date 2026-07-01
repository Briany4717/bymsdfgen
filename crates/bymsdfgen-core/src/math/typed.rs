//! Zero-cost coordinate-space newtypes.
//!
//! One of the most destructive classes of bug in font/graphics engines is mixing
//! coordinate spaces — font design units, em-normalized units, shape units and
//! output pixels all end up as bare `double`s in the C++ original, so a missing
//! transform is a silent logic bug. Here each space is a distinct type carrying a
//! `PhantomData` marker; the compiler refuses to add a `Point<Pixel>` to a
//! `Point<Shape>`. The marker is erased at compile time, so this costs nothing at
//! runtime.
//!
//! The conversions between spaces live exclusively in [`crate::generator::Projection`]
//! and in the font/SVG loaders, which are the only legitimate crossing points. The
//! hot distance loops operate purely on [`super::Vector2`] in shape space.

use super::vector2::Vector2;
use std::marker::PhantomData;

/// Marker trait implemented by the coordinate-space tags below.
pub trait Space: Copy + 'static {}

/// Font design-unit space (raw outline coordinates, e.g. units-per-em grid).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DesignSpace;
/// Em-normalized space (design units divided by units-per-em).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmSpace;
/// The space the distance field is computed in (post-loader, pre-projection).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShapeSpace;
/// Output pixel space of the destination bitmap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PixelSpace;

impl Space for DesignSpace {}
impl Space for EmSpace {}
impl Space for ShapeSpace {}
impl Space for PixelSpace {}

/// A point tagged with the coordinate space it lives in.
#[derive(Debug)]
pub struct Point<S: Space> {
    raw: Vector2,
    _space: PhantomData<S>,
}

// Manual impls so the `S` bound doesn't force `S: Clone` etc.
impl<S: Space> Clone for Point<S> {
    #[inline]
    fn clone(&self) -> Self {
        *self
    }
}
impl<S: Space> Copy for Point<S> {}
impl<S: Space> PartialEq for Point<S> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.raw == other.raw
    }
}

impl<S: Space> Point<S> {
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Point {
            raw: Vector2::new(x, y),
            _space: PhantomData,
        }
    }

    /// Wrap an existing [`Vector2`], asserting it belongs to space `S`.
    #[inline]
    pub const fn from_raw(raw: Vector2) -> Self {
        Point {
            raw,
            _space: PhantomData,
        }
    }

    /// Unwrap to a plain [`Vector2`]. Use only when leaving the typed boundary.
    #[inline]
    pub const fn raw(self) -> Vector2 {
        self.raw
    }

    #[inline]
    pub fn x(self) -> f64 {
        self.raw.x
    }

    #[inline]
    pub fn y(self) -> f64 {
        self.raw.y
    }
}

/// Convenience aliases.
pub type DesignPoint = Point<DesignSpace>;
pub type EmPoint = Point<EmSpace>;
pub type ShapePoint = Point<ShapeSpace>;
pub type PixelPoint = Point<PixelSpace>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_roundtrip() {
        let p = ShapePoint::new(2.0, 3.0);
        assert_eq!(p.raw(), Vector2::new(2.0, 3.0));
        assert_eq!(PixelPoint::from_raw(p.raw()).raw(), p.raw());
    }

    // The following must NOT compile (documented as a compile-fail expectation):
    // let _ = ShapePoint::new(0.0, 0.0) == PixelPoint::new(0.0, 0.0);
}
