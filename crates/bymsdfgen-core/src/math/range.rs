//! The representable distance interval. Port of `core/Range.hpp`.

/// A symmetric-or-asymmetric distance range `[lower, upper]` in shape units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    pub lower: f64,
    pub upper: f64,
}

impl Range {
    /// A symmetric range `[-width/2, +width/2]`.
    #[inline]
    pub const fn symmetric(width: f64) -> Self {
        Range {
            lower: -0.5 * width,
            upper: 0.5 * width,
        }
    }

    #[inline]
    pub const fn new(lower: f64, upper: f64) -> Self {
        Range { lower, upper }
    }

    #[inline]
    pub fn width(self) -> f64 {
        self.upper - self.lower
    }
}

impl std::ops::Mul<f64> for Range {
    type Output = Range;
    #[inline]
    fn mul(self, s: f64) -> Range {
        Range::new(self.lower * s, self.upper * s)
    }
}

impl std::ops::Div<f64> for Range {
    type Output = Range;
    #[inline]
    fn div(self, s: f64) -> Range {
        Range::new(self.lower / s, self.upper / s)
    }
}
