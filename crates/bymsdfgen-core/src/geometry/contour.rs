//! A closed contour: a contiguous run of edge segments plus their colours.
//!
//! Data-oriented layout (struct-of-arrays): `segments` and `colors` are two flat,
//! cache-friendly `Vec`s instead of the original `std::vector<EdgeHolder>` of
//! heap-allocated, individually-coloured polymorphic edges. Colours live in a
//! parallel array because they are only consulted by the multi-channel path.

use super::edge_color::EdgeColor;
use super::edge_segment::EdgeSegment;
use crate::math::Vector2;
use crate::math::scalar::sign;

#[derive(Debug, Clone, Default)]
pub struct Contour {
    pub segments: Vec<EdgeSegment>,
    /// Parallel to `segments`; defaults to `White` for newly added edges.
    pub colors: Vec<EdgeColor>,
}

#[inline]
fn shoelace(a: Vector2, b: Vector2) -> f64 {
    (b.x - a.x) * (a.y + b.y)
}

impl Contour {
    pub fn new() -> Self {
        Contour::default()
    }

    /// Append an edge (white by default).
    #[inline]
    pub fn add_edge(&mut self, edge: EdgeSegment) {
        self.segments.push(edge);
        self.colors.push(EdgeColor::White);
    }

    #[inline]
    pub fn add_edge_colored(&mut self, edge: EdgeSegment, color: EdgeColor) {
        self.segments.push(edge);
        self.colors.push(color);
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Expand `(min, max)` to include this contour.
    pub fn bound(&self, min: &mut Vector2, max: &mut Vector2) {
        for e in &self.segments {
            e.bound(min, max);
        }
    }

    /// Winding sign of the contour (+1 / 0 / -1). Port of `Contour::winding`.
    pub fn winding(&self) -> i32 {
        let n = self.segments.len();
        if n == 0 {
            return 0;
        }
        let mut total = 0.0;
        if n == 1 {
            let e = &self.segments[0];
            let (a, b, c) = (e.point(0.0), e.point(1.0 / 3.0), e.point(2.0 / 3.0));
            total += shoelace(a, b);
            total += shoelace(b, c);
            total += shoelace(c, a);
        } else if n == 2 {
            let (e0, e1) = (&self.segments[0], &self.segments[1]);
            let (a, b, c, d) = (e0.point(0.0), e0.point(0.5), e1.point(0.0), e1.point(0.5));
            total += shoelace(a, b);
            total += shoelace(b, c);
            total += shoelace(c, d);
            total += shoelace(d, a);
        } else {
            let mut prev = self.segments[n - 1].point(0.0);
            for e in &self.segments {
                let cur = e.point(0.0);
                total += shoelace(prev, cur);
                prev = cur;
            }
        }
        sign(total)
    }

    /// Reverse winding direction (reverses order and each segment).
    pub fn reverse(&mut self) {
        self.segments.reverse();
        self.colors.reverse();
        for e in &mut self.segments {
            e.reverse();
        }
    }
}
