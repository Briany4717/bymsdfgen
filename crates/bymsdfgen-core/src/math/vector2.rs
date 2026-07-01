//! A 2-dimensional Euclidean vector backed by `f64`.
//!
//! Port of `core/Vector2.hpp`. Unlike the C++ original — where `Point2` is a bare
//! `typedef` for `Vector2` and every coordinate space collapses into the same type —
//! the *semantic* distinction between coordinate spaces is layered on top via the
//! zero-cost newtypes in [`crate::math::typed`]. `Vector2` itself remains the plain
//! workhorse used inside the hot geometry/distance loops.

use std::ops::{Add, Div, Mul, Neg, Sub};

/// A 2D vector / point in a single, untyped coordinate space.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector2 {
    pub x: f64,
    pub y: f64,
}

/// Semantic alias: a `Vector2` used as a position rather than a direction.
pub type Point2 = Vector2;

impl Vector2 {
    pub const ZERO: Vector2 = Vector2 { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Vector2 { x, y }
    }

    /// Both components set to the same value (mirrors `Vector2(double val)`).
    #[inline]
    pub const fn splat(v: f64) -> Self {
        Vector2 { x: v, y: v }
    }

    #[inline]
    pub fn squared_length(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn length(self) -> f64 {
        self.squared_length().sqrt()
    }

    /// Returns the unit-length vector in the same direction.
    ///
    /// For a zero vector, returns `(0, 0)` when `allow_zero` is true, otherwise
    /// `(0, 1)` — matching the original's `Vector2(0, !allowZero)` behaviour.
    #[inline]
    pub fn normalize(self, allow_zero: bool) -> Vector2 {
        let len = self.length();
        if len != 0.0 {
            Vector2::new(self.x / len, self.y / len)
        } else {
            Vector2::new(0.0, if allow_zero { 0.0 } else { 1.0 })
        }
    }

    /// A vector of the same length orthogonal to this one.
    #[inline]
    pub fn orthogonal(self, polarity: bool) -> Vector2 {
        if polarity {
            Vector2::new(-self.y, self.x)
        } else {
            Vector2::new(self.y, -self.x)
        }
    }

    /// A unit-length vector orthogonal to this one.
    #[inline]
    pub fn orthonormal(self, polarity: bool, allow_zero: bool) -> Vector2 {
        let len = self.length();
        if len != 0.0 {
            if polarity {
                Vector2::new(-self.y / len, self.x / len)
            } else {
                Vector2::new(self.y / len, -self.x / len)
            }
        } else {
            let z = if allow_zero { 0.0 } else { 1.0 };
            if polarity {
                Vector2::new(0.0, z)
            } else {
                Vector2::new(0.0, -z)
            }
        }
    }

    /// True when the vector is non-zero (mirrors `operator bool`).
    #[inline]
    pub fn is_nonzero(self) -> bool {
        self.x != 0.0 || self.y != 0.0
    }
}

/// Dot product.
#[inline]
pub fn dot(a: Vector2, b: Vector2) -> f64 {
    a.x * b.x + a.y * b.y
}

/// 2D cross product (returns the scalar z-component).
#[inline]
pub fn cross(a: Vector2, b: Vector2) -> f64 {
    a.x * b.y - a.y * b.x
}

impl Add for Vector2 {
    type Output = Vector2;
    #[inline]
    fn add(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vector2 {
    type Output = Vector2;
    #[inline]
    fn sub(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x - o.x, self.y - o.y)
    }
}

impl Neg for Vector2 {
    type Output = Vector2;
    #[inline]
    fn neg(self) -> Vector2 {
        Vector2::new(-self.x, -self.y)
    }
}

// Component-wise vector * vector and vector / vector.
impl Mul for Vector2 {
    type Output = Vector2;
    #[inline]
    fn mul(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x * o.x, self.y * o.y)
    }
}

impl Div for Vector2 {
    type Output = Vector2;
    #[inline]
    fn div(self, o: Vector2) -> Vector2 {
        Vector2::new(self.x / o.x, self.y / o.y)
    }
}

// Scalar on the right.
impl Mul<f64> for Vector2 {
    type Output = Vector2;
    #[inline]
    fn mul(self, s: f64) -> Vector2 {
        Vector2::new(self.x * s, self.y * s)
    }
}

impl Div<f64> for Vector2 {
    type Output = Vector2;
    #[inline]
    fn div(self, s: f64) -> Vector2 {
        Vector2::new(self.x / s, self.y / s)
    }
}

// Scalar on the left (`a * v`).
impl Mul<Vector2> for f64 {
    type Output = Vector2;
    #[inline]
    fn mul(self, v: Vector2) -> Vector2 {
        Vector2::new(self * v.x, self * v.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_and_normalize() {
        let v = Vector2::new(3.0, 4.0);
        assert_eq!(v.length(), 5.0);
        let n = v.normalize(false);
        assert!((n.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn zero_normalize() {
        assert_eq!(Vector2::ZERO.normalize(true), Vector2::ZERO);
        assert_eq!(Vector2::ZERO.normalize(false), Vector2::new(0.0, 1.0));
    }

    #[test]
    fn dot_and_cross() {
        let a = Vector2::new(1.0, 2.0);
        let b = Vector2::new(3.0, 4.0);
        assert_eq!(dot(a, b), 11.0);
        assert_eq!(cross(a, b), -2.0);
    }

    #[test]
    fn orthogonal_polarity() {
        let v = Vector2::new(1.0, 0.0);
        assert_eq!(v.orthogonal(true), Vector2::new(0.0, 1.0));
        assert_eq!(v.orthogonal(false), Vector2::new(0.0, -1.0));
    }
}
