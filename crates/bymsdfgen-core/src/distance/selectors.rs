//! Edge selectors. Port of `core/edge-selectors.{h,cpp}`.
//!
//! The C++ class hierarchy (`PerpendicularDistanceSelectorBase` + derived
//! selectors) becomes a trait [`EdgeSelector`] plus plain structs. Because edges
//! are `Copy`, the "near edge" is stored by value (`Option<EdgeSegment>`) instead
//! of a raw pointer, so there is no dangling-pointer hazard.

use super::value::{DistanceValue, MultiAndTrueDistance, MultiDistance};
use crate::geometry::{EdgeColor, EdgeSegment};
use crate::math::{SignedDistance, Vector2, cross, dot};

const DISTANCE_DELTA_FACTOR: f64 = 1.001;

/// Common interface used by the contour combiners and the distance finder.
pub trait EdgeSelector: Default + Clone {
    type Distance: DistanceValue;
    type Cache: Copy + Default;

    fn reset(&mut self, p: Vector2);
    fn add_edge(
        &mut self,
        cache: &mut Self::Cache,
        prev: &EdgeSegment,
        edge: &EdgeSegment,
        next: &EdgeSegment,
        color: EdgeColor,
    );
    fn merge(&mut self, other: &Self);
    fn distance(&self) -> Self::Distance;
}

// ---------------------------------------------------------------------------
// True distance selector
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
pub struct TrueEdgeCache {
    point: Vector2,
    abs_distance: f64,
}

#[derive(Clone, Copy)]
pub struct TrueDistanceSelector {
    p: Vector2,
    min_distance: SignedDistance,
}

impl Default for TrueDistanceSelector {
    fn default() -> Self {
        TrueDistanceSelector {
            p: Vector2::ZERO,
            min_distance: SignedDistance::default(),
        }
    }
}

impl EdgeSelector for TrueDistanceSelector {
    type Distance = f64;
    type Cache = TrueEdgeCache;

    fn reset(&mut self, p: Vector2) {
        let delta = DISTANCE_DELTA_FACTOR * (p - self.p).length();
        self.min_distance.distance +=
            crate::math::scalar::non_zero_sign(self.min_distance.distance) as f64 * delta;
        self.p = p;
    }

    fn add_edge(
        &mut self,
        cache: &mut TrueEdgeCache,
        _prev: &EdgeSegment,
        edge: &EdgeSegment,
        _next: &EdgeSegment,
        _color: EdgeColor,
    ) {
        let delta = DISTANCE_DELTA_FACTOR * (self.p - cache.point).length();
        if cache.abs_distance - delta <= self.min_distance.distance.abs() {
            let (distance, _) = edge.signed_distance(self.p);
            if distance.is_closer_than(self.min_distance) {
                self.min_distance = distance;
            }
            cache.point = self.p;
            cache.abs_distance = distance.distance.abs();
        }
    }

    fn merge(&mut self, other: &Self) {
        if other.min_distance.is_closer_than(self.min_distance) {
            self.min_distance = other.min_distance;
        }
    }

    fn distance(&self) -> f64 {
        self.min_distance.distance
    }
}

// ---------------------------------------------------------------------------
// Perpendicular distance selector base
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
pub struct PerpEdgeCache {
    point: Vector2,
    abs_distance: f64,
    a_domain_distance: f64,
    b_domain_distance: f64,
    a_perpendicular_distance: f64,
    b_perpendicular_distance: f64,
}

#[derive(Clone, Copy)]
pub struct PerpendicularDistanceSelectorBase {
    min_true_distance: SignedDistance,
    min_negative_perpendicular_distance: f64,
    min_positive_perpendicular_distance: f64,
    near_edge: Option<EdgeSegment>,
    near_edge_param: f64,
}

impl Default for PerpendicularDistanceSelectorBase {
    fn default() -> Self {
        let min_true = SignedDistance::default();
        PerpendicularDistanceSelectorBase {
            min_true_distance: min_true,
            min_negative_perpendicular_distance: -min_true.distance.abs(),
            min_positive_perpendicular_distance: min_true.distance.abs(),
            near_edge: None,
            near_edge_param: 0.0,
        }
    }
}

/// Free function port of `getPerpendicularDistance`.
fn get_perpendicular_distance(distance: &mut f64, ep: Vector2, edge_dir: Vector2) -> bool {
    let ts = dot(ep, edge_dir);
    if ts > 0.0 {
        let perpendicular_distance = cross(ep, edge_dir);
        if perpendicular_distance.abs() < distance.abs() {
            *distance = perpendicular_distance;
            return true;
        }
    }
    false
}

impl PerpendicularDistanceSelectorBase {
    fn reset_delta(&mut self, delta: f64) {
        self.min_true_distance.distance +=
            crate::math::scalar::non_zero_sign(self.min_true_distance.distance) as f64 * delta;
        self.min_negative_perpendicular_distance = -self.min_true_distance.distance.abs();
        self.min_positive_perpendicular_distance = self.min_true_distance.distance.abs();
        self.near_edge = None;
        self.near_edge_param = 0.0;
    }

    fn is_edge_relevant(&self, cache: &PerpEdgeCache, p: Vector2) -> bool {
        let delta = DISTANCE_DELTA_FACTOR * (p - cache.point).length();
        cache.abs_distance - delta <= self.min_true_distance.distance.abs()
            || cache.a_domain_distance.abs() < delta
            || cache.b_domain_distance.abs() < delta
            || (cache.a_domain_distance > 0.0
                && (if cache.a_perpendicular_distance < 0.0 {
                    cache.a_perpendicular_distance + delta
                        >= self.min_negative_perpendicular_distance
                } else {
                    cache.a_perpendicular_distance - delta
                        <= self.min_positive_perpendicular_distance
                }))
            || (cache.b_domain_distance > 0.0
                && (if cache.b_perpendicular_distance < 0.0 {
                    cache.b_perpendicular_distance + delta
                        >= self.min_negative_perpendicular_distance
                } else {
                    cache.b_perpendicular_distance - delta
                        <= self.min_positive_perpendicular_distance
                }))
    }

    fn add_edge_true_distance(&mut self, edge: &EdgeSegment, distance: SignedDistance, param: f64) {
        if distance.is_closer_than(self.min_true_distance) {
            self.min_true_distance = distance;
            self.near_edge = Some(*edge);
            self.near_edge_param = param;
        }
    }

    fn add_edge_perpendicular_distance(&mut self, distance: f64) {
        if distance <= 0.0 && distance > self.min_negative_perpendicular_distance {
            self.min_negative_perpendicular_distance = distance;
        }
        if distance >= 0.0 && distance < self.min_positive_perpendicular_distance {
            self.min_positive_perpendicular_distance = distance;
        }
    }

    fn merge(&mut self, other: &Self) {
        if other
            .min_true_distance
            .is_closer_than(self.min_true_distance)
        {
            self.min_true_distance = other.min_true_distance;
            self.near_edge = other.near_edge;
            self.near_edge_param = other.near_edge_param;
        }
        if other.min_negative_perpendicular_distance > self.min_negative_perpendicular_distance {
            self.min_negative_perpendicular_distance = other.min_negative_perpendicular_distance;
        }
        if other.min_positive_perpendicular_distance < self.min_positive_perpendicular_distance {
            self.min_positive_perpendicular_distance = other.min_positive_perpendicular_distance;
        }
    }

    fn compute_distance(&self, p: Vector2) -> f64 {
        let mut min_distance = if self.min_true_distance.distance < 0.0 {
            self.min_negative_perpendicular_distance
        } else {
            self.min_positive_perpendicular_distance
        };
        if let Some(edge) = self.near_edge {
            let mut distance = self.min_true_distance;
            edge.distance_to_perpendicular_distance(&mut distance, p, self.near_edge_param);
            if distance.distance.abs() < min_distance.abs() {
                min_distance = distance.distance;
            }
        }
        min_distance
    }

    fn true_distance(&self) -> SignedDistance {
        self.min_true_distance
    }
}

/// Shared geometry computation used by both perpendicular selectors. Applies the
/// true distance and (channel-masked) perpendicular contributions to one or more
/// bases. The closure `apply_perp` receives a perpendicular distance and feeds it
/// to whichever channel bases are active for this edge's colour.
#[allow(clippy::too_many_arguments)]
fn process_perpendicular_edge(
    p: Vector2,
    cache: &mut PerpEdgeCache,
    prev: &EdgeSegment,
    edge: &EdgeSegment,
    next: &EdgeSegment,
    distance: SignedDistance,
    mut apply_perp: impl FnMut(f64),
) {
    cache.point = p;
    cache.abs_distance = distance.distance.abs();

    let ap = p - edge.point(0.0);
    let bp = p - edge.point(1.0);
    let a_dir = edge.direction(0.0).normalize(true);
    let b_dir = edge.direction(1.0).normalize(true);
    let prev_dir = prev.direction(1.0).normalize(true);
    let next_dir = next.direction(0.0).normalize(true);
    let add = dot(ap, (prev_dir + a_dir).normalize(true));
    let bdd = -dot(bp, (b_dir + next_dir).normalize(true));
    if add > 0.0 {
        let mut pd = distance.distance;
        if get_perpendicular_distance(&mut pd, ap, -a_dir) {
            pd = -pd;
            apply_perp(pd);
        }
        cache.a_perpendicular_distance = pd;
    }
    if bdd > 0.0 {
        let mut pd = distance.distance;
        if get_perpendicular_distance(&mut pd, bp, b_dir) {
            apply_perp(pd);
        }
        cache.b_perpendicular_distance = pd;
    }
    cache.a_domain_distance = add;
    cache.b_domain_distance = bdd;
}

// ---------------------------------------------------------------------------
// Single-channel perpendicular selector
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
pub struct PerpendicularDistanceSelector {
    base: PerpendicularDistanceSelectorBase,
    p: Vector2,
}

impl EdgeSelector for PerpendicularDistanceSelector {
    type Distance = f64;
    type Cache = PerpEdgeCache;

    fn reset(&mut self, p: Vector2) {
        let delta = DISTANCE_DELTA_FACTOR * (p - self.p).length();
        self.base.reset_delta(delta);
        self.p = p;
    }

    fn add_edge(
        &mut self,
        cache: &mut PerpEdgeCache,
        prev: &EdgeSegment,
        edge: &EdgeSegment,
        next: &EdgeSegment,
        _color: EdgeColor,
    ) {
        if self.base.is_edge_relevant(cache, self.p) {
            let (distance, param) = edge.signed_distance(self.p);
            self.base.add_edge_true_distance(edge, distance, param);
            let base = &mut self.base;
            process_perpendicular_edge(self.p, cache, prev, edge, next, distance, |pd| {
                base.add_edge_perpendicular_distance(pd);
            });
        }
    }

    fn merge(&mut self, other: &Self) {
        self.base.merge(&other.base);
    }

    fn distance(&self) -> f64 {
        self.base.compute_distance(self.p)
    }
}

// ---------------------------------------------------------------------------
// Multi-channel selectors
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
pub struct MultiDistanceSelector {
    p: Vector2,
    r: PerpendicularDistanceSelectorBase,
    g: PerpendicularDistanceSelectorBase,
    b: PerpendicularDistanceSelectorBase,
}

impl MultiDistanceSelector {
    fn add_edge_inner(
        &mut self,
        cache: &mut PerpEdgeCache,
        prev: &EdgeSegment,
        edge: &EdgeSegment,
        next: &EdgeSegment,
        color: EdgeColor,
    ) {
        let relevant = (color.has_red() && self.r.is_edge_relevant(cache, self.p))
            || (color.has_green() && self.g.is_edge_relevant(cache, self.p))
            || (color.has_blue() && self.b.is_edge_relevant(cache, self.p));
        if !relevant {
            return;
        }
        let (distance, param) = edge.signed_distance(self.p);
        if color.has_red() {
            self.r.add_edge_true_distance(edge, distance, param);
        }
        if color.has_green() {
            self.g.add_edge_true_distance(edge, distance, param);
        }
        if color.has_blue() {
            self.b.add_edge_true_distance(edge, distance, param);
        }
        let (r, g, b) = (&mut self.r, &mut self.g, &mut self.b);
        process_perpendicular_edge(self.p, cache, prev, edge, next, distance, |pd| {
            if color.has_red() {
                r.add_edge_perpendicular_distance(pd);
            }
            if color.has_green() {
                g.add_edge_perpendicular_distance(pd);
            }
            if color.has_blue() {
                b.add_edge_perpendicular_distance(pd);
            }
        });
    }

    pub fn true_distance(&self) -> SignedDistance {
        let mut distance = self.r.true_distance();
        if self.g.true_distance().is_closer_than(distance) {
            distance = self.g.true_distance();
        }
        if self.b.true_distance().is_closer_than(distance) {
            distance = self.b.true_distance();
        }
        distance
    }

    fn multi_distance(&self) -> MultiDistance {
        MultiDistance {
            r: self.r.compute_distance(self.p),
            g: self.g.compute_distance(self.p),
            b: self.b.compute_distance(self.p),
        }
    }
}

impl EdgeSelector for MultiDistanceSelector {
    type Distance = MultiDistance;
    type Cache = PerpEdgeCache;

    fn reset(&mut self, p: Vector2) {
        let delta = DISTANCE_DELTA_FACTOR * (p - self.p).length();
        self.r.reset_delta(delta);
        self.g.reset_delta(delta);
        self.b.reset_delta(delta);
        self.p = p;
    }

    fn add_edge(
        &mut self,
        cache: &mut PerpEdgeCache,
        prev: &EdgeSegment,
        edge: &EdgeSegment,
        next: &EdgeSegment,
        color: EdgeColor,
    ) {
        self.add_edge_inner(cache, prev, edge, next, color);
    }

    fn merge(&mut self, other: &Self) {
        self.r.merge(&other.r);
        self.g.merge(&other.g);
        self.b.merge(&other.b);
    }

    fn distance(&self) -> MultiDistance {
        self.multi_distance()
    }
}

/// Multi-channel + true-distance alpha. Wraps [`MultiDistanceSelector`].
#[derive(Clone, Copy, Default)]
pub struct MultiAndTrueDistanceSelector {
    inner: MultiDistanceSelector,
}

impl MultiAndTrueDistanceSelector {
    pub fn true_distance(&self) -> SignedDistance {
        self.inner.true_distance()
    }
}

impl EdgeSelector for MultiAndTrueDistanceSelector {
    type Distance = MultiAndTrueDistance;
    type Cache = PerpEdgeCache;

    fn reset(&mut self, p: Vector2) {
        self.inner.reset(p);
    }

    fn add_edge(
        &mut self,
        cache: &mut PerpEdgeCache,
        prev: &EdgeSegment,
        edge: &EdgeSegment,
        next: &EdgeSegment,
        color: EdgeColor,
    ) {
        self.inner.add_edge(cache, prev, edge, next, color);
    }

    fn merge(&mut self, other: &Self) {
        self.inner.merge(&other.inner);
    }

    fn distance(&self) -> MultiAndTrueDistance {
        let md = self.inner.multi_distance();
        MultiAndTrueDistance {
            r: md.r,
            g: md.g,
            b: md.b,
            a: self.inner.true_distance().distance,
        }
    }
}
