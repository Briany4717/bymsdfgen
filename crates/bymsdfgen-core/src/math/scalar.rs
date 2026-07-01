//! Small scalar helpers ported from `core/arithmetics.hpp`.
//!
//! These mirror the original semantics exactly (including the slightly unusual
//! `clamp` that maps out-of-range values to the nearest bound treated as 0/1).

/// Middle of three values.
#[inline]
pub fn median(a: f64, b: f64, c: f64) -> f64 {
    a.min(b).max(a.max(b).min(c))
}

/// Linear interpolation `(1-w)*a + w*b`.
#[inline]
pub fn mix(a: f64, b: f64, w: f64) -> f64 {
    (1.0 - w) * a + w * b
}

/// Clamp to `[0, 1]`.
#[inline]
pub fn clamp01(n: f64) -> f64 {
    n.clamp(0.0, 1.0)
}

/// Clamp to `[a, b]`.
#[inline]
pub fn clamp(n: f64, a: f64, b: f64) -> f64 {
    n.clamp(a, b)
}

/// `1` for positive, `-1` for negative, `0` for zero.
#[inline]
pub fn sign(n: f64) -> i32 {
    ((0.0 < n) as i32) - ((n < 0.0) as i32)
}

/// `1` for non-negative, `-1` for negative (never zero).
#[inline]
pub fn non_zero_sign(n: f64) -> i32 {
    2 * ((n > 0.0) as i32) - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_basic() {
        assert_eq!(median(1.0, 2.0, 3.0), 2.0);
        assert_eq!(median(3.0, 1.0, 2.0), 2.0);
        assert_eq!(median(2.0, 2.0, 5.0), 2.0);
    }

    #[test]
    fn signs() {
        assert_eq!(sign(-4.0), -1);
        assert_eq!(sign(0.0), 0);
        assert_eq!(sign(2.0), 1);
        assert_eq!(non_zero_sign(0.0), -1);
        assert_eq!(non_zero_sign(2.0), 1);
    }
}
