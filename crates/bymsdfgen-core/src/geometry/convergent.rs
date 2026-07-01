//! Convergent curve ordering. Port of `core/convergent-curve-ordering.cpp`.
//!
//! For two curves meeting at a corner with (near) opposite directions, this
//! decides which exits to the left vs. right at an infinitesimal radius — used by
//! `Shape::normalize` when pushing apart convergent edge segments. The math
//! follows the original's higher-derivative cross-product expansion exactly.

use super::edge_segment::EdgeSegment;
use crate::math::scalar::sign;
use crate::math::{Vector2, cross};

fn simplify_degenerate_curve(cp: &mut [Vector2; 4], order: &mut usize) {
    if *order == 3 && (cp[1] == cp[0] || cp[1] == cp[3]) && (cp[2] == cp[0] || cp[2] == cp[3]) {
        cp[1] = cp[3];
        *order = 1;
    }
    if *order == 2 && (cp[1] == cp[0] || cp[1] == cp[2]) {
        cp[1] = cp[2];
        *order = 1;
    }
    if *order == 1 && cp[0] == cp[1] {
        *order = 0;
    }
}

/// Core routine operating on a flat control-point layout centered at `corner`
/// (index `CENTER`), with `before` points preceding and `after` points following.
fn ordering_flat(cp: &[Vector2], center: usize, before: usize, after: usize) -> i32 {
    if !(before > 0 && after > 0) {
        return 0;
    }
    let at = |off: isize| cp[(center as isize + off) as usize];
    let corner = cp[center];

    let mut a1 = at(-1) - corner;
    let mut b1 = at(1) - corner;
    let mut a2 = Vector2::ZERO;
    let mut b2 = Vector2::ZERO;
    let mut a3 = Vector2::ZERO;
    let mut b3 = Vector2::ZERO;

    if before >= 2 {
        a2 = at(-2) - at(-1) - a1;
    }
    if after >= 2 {
        b2 = at(2) - at(1) - b1;
    }
    if before >= 3 {
        a3 = at(-3) - at(-2) - (at(-2) - at(-1)) - a2;
        a2 = a2 * 3.0;
    }
    if after >= 3 {
        b3 = at(3) - at(2) - (at(2) - at(1)) - b2;
        b2 = b2 * 3.0;
    }
    a1 = a1 * before as f64;
    b1 = b1 * after as f64;

    // Non-degenerate case
    if a1.is_nonzero() && b1.is_nonzero() {
        let as_ = a1.length();
        let bs = b1.length();
        let d = as_ * cross(a1, b2) + bs * cross(a2, b1);
        if d != 0.0 {
            return sign(d);
        }
        let d = as_ * as_ * cross(a1, b3) + as_ * bs * cross(a2, b2) + bs * bs * cross(a3, b1);
        if d != 0.0 {
            return sign(d);
        }
        let d = as_ * cross(a2, b3) + bs * cross(a3, b2);
        if d != 0.0 {
            return sign(d);
        }
        return sign(cross(a3, b3));
    }

    // Degenerate after corner: swap a<->b and flip sign.
    let mut s = 1;
    if a1.is_nonzero() {
        // !b1: rotate aN into bN
        b1 = a1;
        std::mem::swap(&mut a2, &mut b2);
        std::mem::swap(&mut a3, &mut b3);
        s = -1;
    }
    if b1.is_nonzero() {
        let d = cross(a3, b1);
        if d != 0.0 {
            return s * sign(d);
        }
        let d = cross(a2, b2);
        if d != 0.0 {
            return s * sign(d);
        }
        let d = cross(a3, b2);
        if d != 0.0 {
            return s * sign(d);
        }
        let d = cross(a2, b3);
        if d != 0.0 {
            return s * sign(d);
        }
        return s * sign(cross(a3, b3));
    }

    // Degenerate on both sides.
    let d = a2.length().sqrt() * cross(a2, b3) + b2.length().sqrt() * cross(a3, b2);
    if d != 0.0 {
        return sign(d);
    }
    sign(cross(a3, b3))
}

/// Relative ordering of curves `a` (ending at the corner) and `b` (starting at it).
pub fn convergent_curve_ordering(a: &EdgeSegment, b: &EdgeSegment) -> i32 {
    let (mut a_cp, mut a_order) = a.control_points();
    let (mut b_cp, mut b_order) = b.control_points();
    if !(1..=3).contains(&a_order) || !(1..=3).contains(&b_order) {
        return 0;
    }
    if a_cp[a_order] != b_cp[0] {
        return 0;
    }
    simplify_degenerate_curve(&mut a_cp, &mut a_order);
    simplify_degenerate_curve(&mut b_cp, &mut b_order);

    // Flat layout: a_cp[0..a_order], corner (= b_cp[0]), b_cp[1..b_order].
    // Center index = a_order. Need room for up to 3 before and 3 after.
    let mut flat = [Vector2::ZERO; 7];
    let center = a_order;
    flat[..a_order].copy_from_slice(&a_cp[..a_order]);
    flat[center..(center + b_order + 1)].copy_from_slice(&b_cp[..=b_order]);
    ordering_flat(&flat, center, a_order, b_order)
}
