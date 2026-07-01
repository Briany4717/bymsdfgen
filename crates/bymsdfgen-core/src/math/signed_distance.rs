//! Signed distance with an alignment tiebreaker.
//!
//! Port of `core/SignedDistance.hpp`. The ordering compares by `|distance|` first
//! and uses `dot` (cosine of the angle to the nearest edge) only to break exact
//! ties, exactly as the original operators do.

use std::cmp::Ordering;

/// A signed distance together with the dot-product alignment used to
/// disambiguate which edge is genuinely closest at corners.
#[derive(Debug, Clone, Copy)]
pub struct SignedDistance {
    pub distance: f64,
    pub dot: f64,
}

impl Default for SignedDistance {
    #[inline]
    fn default() -> Self {
        // Matches the C++ default constructor: distance = -DBL_MAX, dot = 0.
        SignedDistance {
            distance: f64::MIN,
            dot: 0.0,
        }
    }
}

impl SignedDistance {
    #[inline]
    pub const fn new(distance: f64, dot: f64) -> Self {
        SignedDistance { distance, dot }
    }

    /// Total order on `(|distance|, dot)` — "closer" sorts as `Less`.
    #[inline]
    pub fn cmp_key(self, other: SignedDistance) -> Ordering {
        let a = self.distance.abs();
        let b = other.distance.abs();
        match a.partial_cmp(&b) {
            Some(Ordering::Equal) | None => {
                self.dot.partial_cmp(&other.dot).unwrap_or(Ordering::Equal)
            }
            Some(ord) => ord,
        }
    }

    #[inline]
    pub fn is_closer_than(self, other: SignedDistance) -> bool {
        self.cmp_key(other) == Ordering::Less
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closer_by_magnitude() {
        let a = SignedDistance::new(-1.0, 0.5);
        let b = SignedDistance::new(2.0, 0.0);
        assert!(a.is_closer_than(b));
        assert!(!b.is_closer_than(a));
    }

    #[test]
    fn tiebreak_by_dot() {
        let a = SignedDistance::new(1.0, 0.2);
        let b = SignedDistance::new(-1.0, 0.9);
        // equal magnitude -> smaller dot wins
        assert!(a.is_closer_than(b));
    }
}
