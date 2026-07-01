//! The edge segment — a line, a quadratic Bézier, or a cubic Bézier.
//!
//! This is the single most important data-oriented change versus the C++ original.
//! There, `EdgeSegment` is an abstract base with ~11 virtual methods and three
//! heap-allocated subclasses stored behind `EdgeSegment*` pointers
//! (`std::vector<EdgeSegment*>`). Every distance evaluation chases a pointer and
//! pays a vtable lookup.
//!
//! Here it is a plain `Copy` enum whose control points live inline. Dispatch is a
//! `match` (a direct jump the optimizer can inline and vectorize) and contours can
//! store these contiguously, so the CPU streams the XY data through L1/L2 instead
//! of pointer-chasing. No heap, no vtable, no manual `clone()`/`delete`.

use crate::geometry::equation_solver::{solve_cubic, solve_quadratic};
use crate::math::scalar::{non_zero_sign, sign};
use crate::math::{SignedDistance, Vector2, cross, dot};

const CUBIC_SEARCH_STARTS: i32 = 4;
const CUBIC_SEARCH_STEPS: i32 = 4;

/// Linear interpolation of two points, matching msdfgen's `mix`:
/// `(1-t)*a + t*b`. This form returns `a` exactly at `t==0` and `b` exactly at
/// `t==1` (the `a + (b-a)*t` form does not, which breaks contour-closure checks).
#[inline]
fn vmix(a: Vector2, b: Vector2, t: f64) -> Vector2 {
    a * (1.0 - t) + b * t
}

/// An edge: linear (2 points), quadratic (3), or cubic (4 control points).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeSegment {
    Line([Vector2; 2]),
    Quadratic([Vector2; 3]),
    Cubic([Vector2; 4]),
}

impl EdgeSegment {
    /// Build a linear segment.
    #[inline]
    pub fn line(p0: Vector2, p1: Vector2) -> EdgeSegment {
        EdgeSegment::Line([p0, p1])
    }

    /// Build a quadratic, collapsing to a line if the control point is colinear.
    pub fn quadratic(p0: Vector2, p1: Vector2, p2: Vector2) -> EdgeSegment {
        if cross(p1 - p0, p2 - p1) == 0.0 {
            EdgeSegment::Line([p0, p2])
        } else {
            EdgeSegment::Quadratic([p0, p1, p2])
        }
    }

    /// Build a cubic, collapsing to a line or quadratic when degenerate.
    pub fn cubic(p0: Vector2, p1: Vector2, p2: Vector2, p3: Vector2) -> EdgeSegment {
        let p12 = p2 - p1;
        if cross(p1 - p0, p12) == 0.0 && cross(p12, p3 - p2) == 0.0 {
            return EdgeSegment::Line([p0, p3]);
        }
        let q = 1.5 * p1 - 0.5 * p0;
        if q == 1.5 * p2 - 0.5 * p3 {
            return EdgeSegment::Quadratic([p0, q, p3]);
        }
        EdgeSegment::Cubic([p0, p1, p2, p3])
    }

    /// Polynomial order: 1 = line, 2 = quadratic, 3 = cubic.
    #[inline]
    pub fn order(&self) -> usize {
        match self {
            EdgeSegment::Line(_) => 1,
            EdgeSegment::Quadratic(_) => 2,
            EdgeSegment::Cubic(_) => 3,
        }
    }

    /// Control points `[0..=order]`, padded to a fixed array.
    #[inline]
    pub fn control_points(&self) -> ([Vector2; 4], usize) {
        match self {
            EdgeSegment::Line(p) => ([p[0], p[1], Vector2::ZERO, Vector2::ZERO], 1),
            EdgeSegment::Quadratic(p) => ([p[0], p[1], p[2], Vector2::ZERO], 2),
            EdgeSegment::Cubic(p) => (*p, 3),
        }
    }

    /// Point on the curve at parameter `t in [0,1]`.
    pub fn point(&self, t: f64) -> Vector2 {
        match self {
            EdgeSegment::Line(p) => vmix(p[0], p[1], t),
            EdgeSegment::Quadratic(p) => vmix(vmix(p[0], p[1], t), vmix(p[1], p[2], t), t),
            EdgeSegment::Cubic(p) => {
                let p12 = vmix(p[1], p[2], t);
                vmix(
                    vmix(vmix(p[0], p[1], t), p12, t),
                    vmix(p12, vmix(p[2], p[3], t), t),
                    t,
                )
            }
        }
    }

    /// Tangent (unnormalized direction) at parameter `t`.
    pub fn direction(&self, t: f64) -> Vector2 {
        match self {
            EdgeSegment::Line(p) => p[1] - p[0],
            EdgeSegment::Quadratic(p) => {
                let tangent = vmix(p[1] - p[0], p[2] - p[1], t);
                if !tangent.is_nonzero() {
                    p[2] - p[0]
                } else {
                    tangent
                }
            }
            EdgeSegment::Cubic(p) => {
                let tangent = vmix(
                    vmix(p[1] - p[0], p[2] - p[1], t),
                    vmix(p[2] - p[1], p[3] - p[2], t),
                    t,
                );
                if !tangent.is_nonzero() {
                    if t == 0.0 {
                        return p[2] - p[0];
                    }
                    if t == 1.0 {
                        return p[3] - p[1];
                    }
                }
                tangent
            }
        }
    }

    /// Second derivative direction (rate of tangent change) at parameter `t`.
    pub fn direction_change(&self, t: f64) -> Vector2 {
        match self {
            EdgeSegment::Line(_) => Vector2::ZERO,
            EdgeSegment::Quadratic(p) => (p[2] - p[1]) - (p[1] - p[0]),
            EdgeSegment::Cubic(p) => vmix(
                (p[2] - p[1]) - (p[1] - p[0]),
                (p[3] - p[2]) - (p[2] - p[1]),
                t,
            ),
        }
    }

    /// Arc length of the segment.
    pub fn length(&self) -> f64 {
        match self {
            EdgeSegment::Line(p) => (p[1] - p[0]).length(),
            EdgeSegment::Quadratic(p) => {
                let ab = p[1] - p[0];
                let br = p[2] - p[1] - ab;
                let abab = dot(ab, ab);
                let abbr = dot(ab, br);
                let brbr = dot(br, br);
                let ab_len = abab.sqrt();
                let br_len = brbr.sqrt();
                let crs = cross(ab, br);
                let h = (abab + abbr + abbr + brbr).sqrt();
                (br_len * ((abbr + brbr) * h - abbr * ab_len)
                    + crs * crs * ((br_len * h + abbr + brbr) / (br_len * ab_len + abbr)).ln())
                    / (brbr * br_len)
            }
            // Cubic length has no closed form; the original does not implement it.
            EdgeSegment::Cubic(_) => f64::NAN,
        }
    }

    /// Signed distance from `origin` to the segment, plus the parameter `t` of the
    /// nearest point. Mirrors `signedDistance(origin, &param)`.
    pub fn signed_distance(&self, origin: Vector2) -> (SignedDistance, f64) {
        match self {
            EdgeSegment::Line(p) => self.linear_signed_distance(p, origin),
            EdgeSegment::Quadratic(p) => self.quadratic_signed_distance(p, origin),
            EdgeSegment::Cubic(p) => self.cubic_signed_distance(p, origin),
        }
    }

    fn linear_signed_distance(&self, p: &[Vector2; 2], origin: Vector2) -> (SignedDistance, f64) {
        let aq = origin - p[0];
        let ab = p[1] - p[0];
        let param = dot(aq, ab) / dot(ab, ab);
        let eq = (if param > 0.5 { p[1] } else { p[0] }) - origin;
        let endpoint_distance = eq.length();
        if param > 0.0 && param < 1.0 {
            let ortho_distance = dot(ab.orthonormal(false, false), aq);
            if ortho_distance.abs() < endpoint_distance {
                return (SignedDistance::new(ortho_distance, 0.0), param);
            }
        }
        let sd = SignedDistance::new(
            non_zero_sign(cross(aq, ab)) as f64 * endpoint_distance,
            dot(ab.normalize(false), eq.normalize(false)).abs(),
        );
        (sd, param)
    }

    fn quadratic_signed_distance(
        &self,
        p: &[Vector2; 3],
        origin: Vector2,
    ) -> (SignedDistance, f64) {
        let qa = p[0] - origin;
        let ab = p[1] - p[0];
        let br = p[2] - p[1] - ab;
        let a = dot(br, br);
        let b = 3.0 * dot(ab, br);
        let c = 2.0 * dot(ab, ab) + dot(qa, br);
        let d = dot(qa, ab);
        let solutions = solve_cubic(a, b, c, d);

        let mut ep_dir = self.direction(0.0);
        let mut min_distance = non_zero_sign(cross(ep_dir, qa)) as f64 * qa.length(); // from A
        let mut param = -dot(qa, ep_dir) / dot(ep_dir, ep_dir);
        {
            let distance = (p[2] - origin).length(); // from B
            if distance < min_distance.abs() {
                ep_dir = self.direction(1.0);
                min_distance = non_zero_sign(cross(ep_dir, p[2] - origin)) as f64 * distance;
                param = dot(origin - p[1], ep_dir) / dot(ep_dir, ep_dir);
            }
        }
        for &t in solutions.iter() {
            if t > 0.0 && t < 1.0 {
                let qe = qa + 2.0 * t * ab + (t * t) * br;
                let distance = qe.length();
                if distance <= min_distance.abs() {
                    min_distance = non_zero_sign(cross(ab + t * br, qe)) as f64 * distance;
                    param = t;
                }
            }
        }

        if (0.0..=1.0).contains(&param) {
            (SignedDistance::new(min_distance, 0.0), param)
        } else if param < 0.5 {
            (
                SignedDistance::new(
                    min_distance,
                    dot(self.direction(0.0).normalize(false), qa.normalize(false)).abs(),
                ),
                param,
            )
        } else {
            (
                SignedDistance::new(
                    min_distance,
                    dot(
                        self.direction(1.0).normalize(false),
                        (p[2] - origin).normalize(false),
                    )
                    .abs(),
                ),
                param,
            )
        }
    }

    fn cubic_signed_distance(&self, p: &[Vector2; 4], origin: Vector2) -> (SignedDistance, f64) {
        let qa = p[0] - origin;
        let ab = p[1] - p[0];
        let br = p[2] - p[1] - ab;
        let as_ = (p[3] - p[2]) - (p[2] - p[1]) - br;

        let mut ep_dir = self.direction(0.0);
        let mut min_distance = non_zero_sign(cross(ep_dir, qa)) as f64 * qa.length(); // from A
        let mut param = -dot(qa, ep_dir) / dot(ep_dir, ep_dir);
        {
            let distance = (p[3] - origin).length(); // from B
            if distance < min_distance.abs() {
                ep_dir = self.direction(1.0);
                min_distance = non_zero_sign(cross(ep_dir, p[3] - origin)) as f64 * distance;
                param = dot(ep_dir - (p[3] - origin), ep_dir) / dot(ep_dir, ep_dir);
            }
        }
        // Iterative minimum-distance search (Newton with curvature term).
        for i in 0..=CUBIC_SEARCH_STARTS {
            let mut t = (1.0 / CUBIC_SEARCH_STARTS as f64) * i as f64;
            let mut qe = qa + 3.0 * t * ab + 3.0 * (t * t) * br + (t * t * t) * as_;
            let mut d1 = 3.0 * ab + 6.0 * t * br + 3.0 * (t * t) * as_;
            let mut d2 = 6.0 * br + 6.0 * t * as_;
            let mut improved_t = t - dot(qe, d1) / (dot(d1, d1) + dot(qe, d2));
            if improved_t > 0.0 && improved_t < 1.0 {
                let mut remaining_steps = CUBIC_SEARCH_STEPS;
                loop {
                    t = improved_t;
                    qe = qa + 3.0 * t * ab + 3.0 * (t * t) * br + (t * t * t) * as_;
                    d1 = 3.0 * ab + 6.0 * t * br + 3.0 * (t * t) * as_;
                    remaining_steps -= 1;
                    if remaining_steps == 0 {
                        break;
                    }
                    d2 = 6.0 * br + 6.0 * t * as_;
                    improved_t = t - dot(qe, d1) / (dot(d1, d1) + dot(qe, d2));
                    if !(improved_t > 0.0 && improved_t < 1.0) {
                        break;
                    }
                }
                let distance = qe.length();
                if distance < min_distance.abs() {
                    min_distance = non_zero_sign(cross(d1, qe)) as f64 * distance;
                    param = t;
                }
            }
        }

        if (0.0..=1.0).contains(&param) {
            (SignedDistance::new(min_distance, 0.0), param)
        } else if param < 0.5 {
            (
                SignedDistance::new(
                    min_distance,
                    dot(self.direction(0.0).normalize(false), qa.normalize(false)).abs(),
                ),
                param,
            )
        } else {
            (
                SignedDistance::new(
                    min_distance,
                    dot(
                        self.direction(1.0).normalize(false),
                        (p[3] - origin).normalize(false),
                    )
                    .abs(),
                ),
                param,
            )
        }
    }

    /// Converts a true signed distance to a perpendicular one when the closest
    /// point falls outside the `[0,1]` domain. Port of
    /// `distanceToPerpendicularDistance`.
    pub fn distance_to_perpendicular_distance(
        &self,
        distance: &mut SignedDistance,
        origin: Vector2,
        param: f64,
    ) {
        if param < 0.0 {
            let dir = self.direction(0.0).normalize(false);
            let aq = origin - self.point(0.0);
            let ts = dot(aq, dir);
            if ts < 0.0 {
                let perpendicular_distance = cross(aq, dir);
                if perpendicular_distance.abs() <= distance.distance.abs() {
                    distance.distance = perpendicular_distance;
                    distance.dot = 0.0;
                }
            }
        } else if param > 1.0 {
            let dir = self.direction(1.0).normalize(false);
            let bq = origin - self.point(1.0);
            let ts = dot(bq, dir);
            if ts > 0.0 {
                let perpendicular_distance = cross(bq, dir);
                if perpendicular_distance.abs() <= distance.distance.abs() {
                    distance.distance = perpendicular_distance;
                    distance.dot = 0.0;
                }
            }
        }
    }

    /// Horizontal scanline intersections at height `y`. Writes up to 3 `(x, dy)`
    /// pairs into `out` and returns the count. `dy` is the crossing direction.
    pub fn scanline_intersections(&self, y: f64, out: &mut [(f64, i32); 3]) -> usize {
        match self {
            EdgeSegment::Line(p) => Self::linear_scanline(p, y, out),
            EdgeSegment::Quadratic(p) => Self::quadratic_scanline(p, y, out),
            EdgeSegment::Cubic(p) => Self::cubic_scanline(p, y, out),
        }
    }

    fn linear_scanline(p: &[Vector2; 2], y: f64, out: &mut [(f64, i32); 3]) -> usize {
        if (y >= p[0].y && y < p[1].y) || (y >= p[1].y && y < p[0].y) {
            let param = (y - p[0].y) / (p[1].y - p[0].y);
            out[0] = (
                crate::math::scalar::mix(p[0].x, p[1].x, param),
                sign(p[1].y - p[0].y),
            );
            1
        } else {
            0
        }
    }

    fn quadratic_scanline(p: &[Vector2; 3], y: f64, out: &mut [(f64, i32); 3]) -> usize {
        let mut total = 0usize;
        let mut next_dy = if y > p[0].y { 1 } else { -1 };
        out[total].0 = p[0].x;
        if p[0].y == y {
            if p[0].y < p[1].y || (p[0].y == p[1].y && p[0].y < p[2].y) {
                out[total].1 = 1;
                total += 1;
            } else {
                next_dy = 1;
            }
        }
        {
            let ab = p[1] - p[0];
            let br = p[2] - p[1] - ab;
            let mut t = solve_quadratic(br.y, 2.0 * ab.y, p[0].y - y).roots();
            if t.len() >= 2 && t[0] > t[1] {
                t.swap(0, 1);
            }
            let mut i = 0;
            while i < t.len() && total < 2 {
                let ti = t[i];
                if (0.0..=1.0).contains(&ti) {
                    out[total].0 = p[0].x + 2.0 * ti * ab.x + ti * ti * br.x;
                    if next_dy as f64 * (ab.y + ti * br.y) >= 0.0 {
                        out[total].1 = next_dy;
                        total += 1;
                        next_dy = -next_dy;
                    }
                }
                i += 1;
            }
        }
        if p[2].y == y {
            if next_dy > 0 && total > 0 {
                total -= 1;
                next_dy = -1;
            }
            if (p[2].y < p[1].y || (p[2].y == p[1].y && p[2].y < p[0].y)) && total < 2 {
                out[total].0 = p[2].x;
                if next_dy < 0 {
                    out[total].1 = -1;
                    total += 1;
                    next_dy = 1;
                }
            }
        }
        if next_dy != (if y >= p[2].y { 1 } else { -1 }) {
            if total > 0 {
                total -= 1;
            } else {
                if (p[2].y - y).abs() < (p[0].y - y).abs() {
                    out[total].0 = p[2].x;
                }
                out[total].1 = next_dy;
                total += 1;
            }
        }
        total
    }

    fn cubic_scanline(p: &[Vector2; 4], y: f64, out: &mut [(f64, i32); 3]) -> usize {
        let mut total = 0usize;
        let mut next_dy = if y > p[0].y { 1 } else { -1 };
        out[total].0 = p[0].x;
        if p[0].y == y {
            if p[0].y < p[1].y
                || (p[0].y == p[1].y && (p[0].y < p[2].y || (p[0].y == p[2].y && p[0].y < p[3].y)))
            {
                out[total].1 = 1;
                total += 1;
            } else {
                next_dy = 1;
            }
        }
        {
            let ab = p[1] - p[0];
            let br = p[2] - p[1] - ab;
            let as_ = (p[3] - p[2]) - (p[2] - p[1]) - br;
            let mut t = solve_cubic(as_.y, 3.0 * br.y, 3.0 * ab.y, p[0].y - y);
            // sort up to 3 roots
            if t.len() >= 2 {
                if t[0] > t[1] {
                    t.swap(0, 1);
                }
                if t.len() >= 3 && t[1] > t[2] {
                    t.swap(1, 2);
                    if t[0] > t[1] {
                        t.swap(0, 1);
                    }
                }
            }
            let mut i = 0;
            while i < t.len() && total < 3 {
                let ti = t[i];
                if (0.0..=1.0).contains(&ti) {
                    out[total].0 =
                        p[0].x + 3.0 * ti * ab.x + 3.0 * ti * ti * br.x + ti * ti * ti * as_.x;
                    if next_dy as f64 * (ab.y + 2.0 * ti * br.y + ti * ti * as_.y) >= 0.0 {
                        out[total].1 = next_dy;
                        total += 1;
                        next_dy = -next_dy;
                    }
                }
                i += 1;
            }
        }
        if p[3].y == y {
            if next_dy > 0 && total > 0 {
                total -= 1;
                next_dy = -1;
            }
            if (p[3].y < p[2].y
                || (p[3].y == p[2].y && (p[3].y < p[1].y || (p[3].y == p[1].y && p[3].y < p[0].y))))
                && total < 3
            {
                out[total].0 = p[3].x;
                if next_dy < 0 {
                    out[total].1 = -1;
                    total += 1;
                    next_dy = 1;
                }
            }
        }
        if next_dy != (if y >= p[3].y { 1 } else { -1 }) {
            if total > 0 {
                total -= 1;
            } else {
                if (p[3].y - y).abs() < (p[0].y - y).abs() {
                    out[total].0 = p[3].x;
                }
                out[total].1 = next_dy;
                total += 1;
            }
        }
        total
    }

    /// Expand `(min, max)` to include this segment's bounding box.
    pub fn bound(&self, min: &mut Vector2, max: &mut Vector2) {
        let mut pb = |p: Vector2| {
            if p.x < min.x {
                min.x = p.x;
            }
            if p.y < min.y {
                min.y = p.y;
            }
            if p.x > max.x {
                max.x = p.x;
            }
            if p.y > max.y {
                max.y = p.y;
            }
        };
        match self {
            EdgeSegment::Line(p) => {
                pb(p[0]);
                pb(p[1]);
            }
            EdgeSegment::Quadratic(p) => {
                pb(p[0]);
                pb(p[2]);
                let bot = (p[1] - p[0]) - (p[2] - p[1]);
                if bot.x != 0.0 {
                    let param = (p[1].x - p[0].x) / bot.x;
                    if param > 0.0 && param < 1.0 {
                        pb(self.point(param));
                    }
                }
                if bot.y != 0.0 {
                    let param = (p[1].y - p[0].y) / bot.y;
                    if param > 0.0 && param < 1.0 {
                        pb(self.point(param));
                    }
                }
            }
            EdgeSegment::Cubic(p) => {
                pb(p[0]);
                pb(p[3]);
                let a0 = p[1] - p[0];
                let a1 = 2.0 * (p[2] - p[1] - a0);
                let a2 = p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0];
                for &param in solve_quadratic(a2.x, a1.x, a0.x).roots().iter() {
                    if param > 0.0 && param < 1.0 {
                        pb(self.point(param));
                    }
                }
                for &param in solve_quadratic(a2.y, a1.y, a0.y).roots().iter() {
                    if param > 0.0 && param < 1.0 {
                        pb(self.point(param));
                    }
                }
            }
        }
    }

    /// Reverse the parameterization direction.
    pub fn reverse(&mut self) {
        match self {
            EdgeSegment::Line(p) => p.swap(0, 1),
            EdgeSegment::Quadratic(p) => p.swap(0, 2),
            EdgeSegment::Cubic(p) => {
                p.swap(0, 3);
                p.swap(1, 2);
            }
        }
    }

    /// Move the start point, adjusting controls (port of `moveStartPoint`).
    pub fn move_start_point(&mut self, to: Vector2) {
        match self {
            EdgeSegment::Line(p) => p[0] = to,
            EdgeSegment::Quadratic(p) => {
                let orig_s_dir = p[0] - p[1];
                let orig_p1 = p[1];
                p[1] = p[1]
                    + cross(p[0] - p[1], to - p[0]) / cross(p[0] - p[1], p[2] - p[1])
                        * (p[2] - p[1]);
                p[0] = to;
                if dot(orig_s_dir, p[0] - p[1]) < 0.0 {
                    p[1] = orig_p1;
                }
            }
            EdgeSegment::Cubic(p) => {
                p[1] = p[1] + (to - p[0]);
                p[0] = to;
            }
        }
    }

    /// Move the end point, adjusting controls (port of `moveEndPoint`).
    pub fn move_end_point(&mut self, to: Vector2) {
        match self {
            EdgeSegment::Line(p) => p[1] = to,
            EdgeSegment::Quadratic(p) => {
                let orig_e_dir = p[2] - p[1];
                let orig_p1 = p[1];
                p[1] = p[1]
                    + cross(p[2] - p[1], to - p[2]) / cross(p[2] - p[1], p[0] - p[1])
                        * (p[0] - p[1]);
                p[2] = to;
                if dot(orig_e_dir, p[2] - p[1]) < 0.0 {
                    p[1] = orig_p1;
                }
            }
            EdgeSegment::Cubic(p) => {
                p[2] = p[2] + (to - p[3]);
                p[3] = to;
            }
        }
    }

    /// Split into three equal-parameter thirds (used by `Shape::normalize`).
    pub fn split_in_thirds(&self) -> [EdgeSegment; 3] {
        match self {
            EdgeSegment::Line(_) => [
                EdgeSegment::line(self.point(0.0), self.point(1.0 / 3.0)),
                EdgeSegment::line(self.point(1.0 / 3.0), self.point(2.0 / 3.0)),
                EdgeSegment::line(self.point(2.0 / 3.0), self.point(1.0)),
            ],
            EdgeSegment::Quadratic(p) => [
                EdgeSegment::Quadratic([p[0], vmix(p[0], p[1], 1.0 / 3.0), self.point(1.0 / 3.0)]),
                EdgeSegment::Quadratic([
                    self.point(1.0 / 3.0),
                    vmix(
                        vmix(p[0], p[1], 5.0 / 9.0),
                        vmix(p[1], p[2], 4.0 / 9.0),
                        0.5,
                    ),
                    self.point(2.0 / 3.0),
                ]),
                EdgeSegment::Quadratic([self.point(2.0 / 3.0), vmix(p[1], p[2], 2.0 / 3.0), p[2]]),
            ],
            EdgeSegment::Cubic(p) => [
                EdgeSegment::Cubic([
                    p[0],
                    if p[0] == p[1] {
                        p[0]
                    } else {
                        vmix(p[0], p[1], 1.0 / 3.0)
                    },
                    vmix(
                        vmix(p[0], p[1], 1.0 / 3.0),
                        vmix(p[1], p[2], 1.0 / 3.0),
                        1.0 / 3.0,
                    ),
                    self.point(1.0 / 3.0),
                ]),
                EdgeSegment::Cubic([
                    self.point(1.0 / 3.0),
                    vmix(
                        vmix(
                            vmix(p[0], p[1], 1.0 / 3.0),
                            vmix(p[1], p[2], 1.0 / 3.0),
                            1.0 / 3.0,
                        ),
                        vmix(
                            vmix(p[1], p[2], 1.0 / 3.0),
                            vmix(p[2], p[3], 1.0 / 3.0),
                            1.0 / 3.0,
                        ),
                        2.0 / 3.0,
                    ),
                    vmix(
                        vmix(
                            vmix(p[0], p[1], 2.0 / 3.0),
                            vmix(p[1], p[2], 2.0 / 3.0),
                            2.0 / 3.0,
                        ),
                        vmix(
                            vmix(p[1], p[2], 2.0 / 3.0),
                            vmix(p[2], p[3], 2.0 / 3.0),
                            2.0 / 3.0,
                        ),
                        1.0 / 3.0,
                    ),
                    self.point(2.0 / 3.0),
                ]),
                EdgeSegment::Cubic([
                    self.point(2.0 / 3.0),
                    vmix(
                        vmix(p[1], p[2], 2.0 / 3.0),
                        vmix(p[2], p[3], 2.0 / 3.0),
                        2.0 / 3.0,
                    ),
                    if p[2] == p[3] {
                        p[3]
                    } else {
                        vmix(p[2], p[3], 2.0 / 3.0)
                    },
                    p[3],
                ]),
            ],
        }
    }

    /// Promote a quadratic to an equivalent cubic (port of `convertToCubic`).
    pub fn convert_to_cubic(&self) -> EdgeSegment {
        match self {
            EdgeSegment::Quadratic(p) => EdgeSegment::Cubic([
                p[0],
                vmix(p[0], p[1], 2.0 / 3.0),
                vmix(p[1], p[2], 1.0 / 3.0),
                p[2],
            ]),
            other => *other,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_distance_perpendicular() {
        let e = EdgeSegment::line(Vector2::new(0.0, 0.0), Vector2::new(10.0, 0.0));
        let (sd, t) = e.signed_distance(Vector2::new(5.0, 3.0));
        assert!((sd.distance.abs() - 3.0).abs() < 1e-9);
        assert!((t - 0.5).abs() < 1e-9);
    }

    #[test]
    fn line_endpoints() {
        let e = EdgeSegment::line(Vector2::new(0.0, 0.0), Vector2::new(10.0, 0.0));
        assert_eq!(e.point(0.0), Vector2::new(0.0, 0.0));
        assert_eq!(e.point(1.0), Vector2::new(10.0, 0.0));
        assert_eq!(e.point(0.5), Vector2::new(5.0, 0.0));
    }

    #[test]
    fn quadratic_collapses_to_line() {
        // colinear control point
        let e = EdgeSegment::quadratic(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(2.0, 0.0),
        );
        assert!(matches!(e, EdgeSegment::Line(_)));
    }

    #[test]
    fn reverse_line() {
        let mut e = EdgeSegment::line(Vector2::new(0.0, 0.0), Vector2::new(10.0, 0.0));
        e.reverse();
        assert_eq!(e.point(0.0), Vector2::new(10.0, 0.0));
    }

    #[test]
    fn scanline_line_crossing() {
        let e = EdgeSegment::line(Vector2::new(0.0, 0.0), Vector2::new(0.0, 10.0));
        let mut out = [(0.0, 0); 3];
        let n = e.scanline_intersections(5.0, &mut out);
        assert_eq!(n, 1);
        assert!((out[0].0 - 0.0).abs() < 1e-9);
        assert_eq!(out[0].1, 1);
    }
}
