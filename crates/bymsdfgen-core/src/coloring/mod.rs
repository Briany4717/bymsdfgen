//! Edge coloring heuristics. Port of `core/edge-coloring.cpp`.
//!
//! Assigns each edge a channel mask so the multi-channel generator can reproduce
//! sharp corners. The deterministic PRNG (`seedExtract*`) is reproduced bit-for-bit
//! so a fixed seed yields identical colouring to the original.

use crate::geometry::{Contour, EdgeColor, EdgeSegment, Shape};
use crate::math::{Vector2, cross, dot};

const EDGE_LENGTH_PRECISION: i32 = 4;

/// Balanced trichotomy: -1 near start, 0 middle, +1 near end; sums to zero over `n`.
fn symmetrical_trichotomy(position: i32, n: i32) -> i32 {
    (3.0 + 2.875 * position as f64 / (n - 1) as f64 - 1.4375 + 0.5) as i32 - 3
}

fn is_corner(a_dir: Vector2, b_dir: Vector2, cross_threshold: f64) -> bool {
    dot(a_dir, b_dir) <= 0.0 || cross(a_dir, b_dir).abs() > cross_threshold
}

fn estimate_edge_length(edge: &EdgeSegment) -> f64 {
    let mut len = 0.0;
    let mut prev = edge.point(0.0);
    for i in 1..=EDGE_LENGTH_PRECISION {
        let cur = edge.point(1.0 / EDGE_LENGTH_PRECISION as f64 * i as f64);
        len += (cur - prev).length();
        prev = cur;
    }
    len
}

fn seed_extract2(seed: &mut u64) -> u32 {
    let v = (*seed as u32) & 1;
    *seed >>= 1;
    v
}

fn seed_extract3(seed: &mut u64) -> u32 {
    let v = (*seed % 3) as u32;
    *seed /= 3;
    v
}

fn init_color(seed: &mut u64) -> EdgeColor {
    const COLORS: [EdgeColor; 3] = [EdgeColor::Cyan, EdgeColor::Magenta, EdgeColor::Yellow];
    COLORS[seed_extract3(seed) as usize]
}

fn switch_color(color: &mut EdgeColor, seed: &mut u64) {
    let shifted = (color.bits() as u32) << (1 + seed_extract2(seed));
    *color = EdgeColor::from_bits(((shifted | (shifted >> 3)) & 7) as u8);
}

fn switch_color_banned(color: &mut EdgeColor, seed: &mut u64, banned: EdgeColor) {
    let combined = color.bits() & banned.bits();
    if combined == EdgeColor::Red.bits()
        || combined == EdgeColor::Green.bits()
        || combined == EdgeColor::Blue.bits()
    {
        *color = EdgeColor::from_bits(combined ^ EdgeColor::White.bits());
    } else {
        switch_color(color, seed);
    }
}

/// Identify corner indices within a contour.
fn identify_corners(contour: &Contour, cross_threshold: f64) -> Vec<usize> {
    let mut corners = Vec::new();
    if contour.segments.is_empty() {
        return corners;
    }
    let mut prev_direction = contour.segments[contour.segments.len() - 1].direction(1.0);
    for (index, edge) in contour.segments.iter().enumerate() {
        if is_corner(
            prev_direction.normalize(false),
            edge.direction(0.0).normalize(false),
            cross_threshold,
        ) {
            corners.push(index);
        }
        prev_direction = edge.direction(1.0);
    }
    corners
}

/// Handle the single-corner "teardrop" colouring shared by simple/ink-trap.
fn color_teardrop(contour: &mut Contour, corner: usize, colors: [EdgeColor; 3]) {
    let m = contour.segments.len();
    if m >= 3 {
        for i in 0..m {
            let idx = (corner + i) % m;
            contour.colors[idx] = colors[(1 + symmetrical_trichotomy(i as i32, m as i32)) as usize];
        }
    } else if m >= 1 {
        // Fewer than three segments: split to obtain three colours.
        let mut parts: [Option<EdgeSegment>; 7] = [None; 7];
        let split0 = contour.segments[0].split_in_thirds();
        parts[3 * corner] = Some(split0[0]);
        parts[1 + 3 * corner] = Some(split0[1]);
        parts[2 + 3 * corner] = Some(split0[2]);
        let mut part_colors: [EdgeColor; 7] = [EdgeColor::White; 7];
        if m >= 2 {
            let split1 = contour.segments[1].split_in_thirds();
            parts[3 - 3 * corner] = Some(split1[0]);
            parts[4 - 3 * corner] = Some(split1[1]);
            parts[5 - 3 * corner] = Some(split1[2]);
            part_colors[0] = colors[0];
            part_colors[1] = colors[0];
            part_colors[2] = colors[1];
            part_colors[3] = colors[1];
            part_colors[4] = colors[2];
            part_colors[5] = colors[2];
        } else {
            part_colors[0] = colors[0];
            part_colors[1] = colors[1];
            part_colors[2] = colors[2];
        }
        let mut new_segs = Vec::new();
        let mut new_cols = Vec::new();
        for i in 0..7 {
            if let Some(seg) = parts[i] {
                new_segs.push(seg);
                new_cols.push(part_colors[i]);
            }
        }
        contour.segments = new_segs;
        contour.colors = new_cols;
    }
}

/// Default heuristic. Port of `edgeColoringSimple`.
pub fn edge_coloring_simple(shape: &mut Shape, angle_threshold: f64, mut seed: u64) {
    let cross_threshold = angle_threshold.sin();
    let mut color = init_color(&mut seed);
    for contour in &mut shape.contours {
        if contour.segments.is_empty() {
            continue;
        }
        let corners = identify_corners(contour, cross_threshold);

        if corners.is_empty() {
            switch_color(&mut color, &mut seed);
            for c in &mut contour.colors {
                *c = color;
            }
        } else if corners.len() == 1 {
            let mut colors = [EdgeColor::White; 3];
            switch_color(&mut color, &mut seed);
            colors[0] = color;
            colors[1] = EdgeColor::White;
            switch_color(&mut color, &mut seed);
            colors[2] = color;
            color_teardrop(contour, corners[0], colors);
        } else {
            let corner_count = corners.len();
            let mut spline = 0usize;
            let start = corners[0];
            let m = contour.segments.len();
            switch_color(&mut color, &mut seed);
            let initial_color = color;
            for i in 0..m {
                let index = (start + i) % m;
                if spline + 1 < corner_count && corners[spline + 1] == index {
                    spline += 1;
                    let banned = if spline == corner_count - 1 {
                        initial_color
                    } else {
                        EdgeColor::Black
                    };
                    switch_color_banned(&mut color, &mut seed, banned);
                }
                contour.colors[index] = color;
            }
        }
    }
}

struct InkTrapCorner {
    index: usize,
    prev_edge_length_estimate: f64,
    minor: bool,
    color: EdgeColor,
}

/// Ink-trap-aware heuristic. Port of `edgeColoringInkTrap`.
pub fn edge_coloring_ink_trap(shape: &mut Shape, angle_threshold: f64, mut seed: u64) {
    let cross_threshold = angle_threshold.sin();
    let mut color = init_color(&mut seed);
    for contour in &mut shape.contours {
        if contour.segments.is_empty() {
            continue;
        }
        let mut spline_length = 0.0;
        let mut corners: Vec<InkTrapCorner> = Vec::new();
        {
            let mut prev_direction = contour.segments[contour.segments.len() - 1].direction(1.0);
            for (index, edge) in contour.segments.iter().enumerate() {
                if is_corner(
                    prev_direction.normalize(false),
                    edge.direction(0.0).normalize(false),
                    cross_threshold,
                ) {
                    corners.push(InkTrapCorner {
                        index,
                        prev_edge_length_estimate: spline_length,
                        minor: false,
                        color: EdgeColor::Black,
                    });
                    spline_length = 0.0;
                }
                spline_length += estimate_edge_length(edge);
                prev_direction = edge.direction(1.0);
            }
        }

        if corners.is_empty() {
            switch_color(&mut color, &mut seed);
            for c in &mut contour.colors {
                *c = color;
            }
        } else if corners.len() == 1 {
            let mut colors = [EdgeColor::White; 3];
            switch_color(&mut color, &mut seed);
            colors[0] = color;
            colors[1] = EdgeColor::White;
            switch_color(&mut color, &mut seed);
            colors[2] = color;
            color_teardrop(contour, corners[0].index, colors);
        } else {
            let corner_count = corners.len();
            let mut major_corner_count = corner_count;
            if corner_count > 3 {
                corners[0].prev_edge_length_estimate += spline_length;
                for i in 0..corner_count {
                    if corners[i].prev_edge_length_estimate
                        > corners[(i + 1) % corner_count].prev_edge_length_estimate
                        && corners[(i + 1) % corner_count].prev_edge_length_estimate
                            < corners[(i + 2) % corner_count].prev_edge_length_estimate
                    {
                        corners[i].minor = true;
                        major_corner_count -= 1;
                    }
                }
            }
            let mut initial_color = EdgeColor::Black;
            for corner in corners.iter_mut().take(corner_count) {
                if !corner.minor {
                    major_corner_count -= 1;
                    let banned = if major_corner_count == 0 {
                        initial_color
                    } else {
                        EdgeColor::Black
                    };
                    switch_color_banned(&mut color, &mut seed, banned);
                    corner.color = color;
                    if initial_color == EdgeColor::Black {
                        initial_color = color;
                    }
                }
            }
            for i in 0..corner_count {
                if corners[i].minor {
                    let next_color = corners[(i + 1) % corner_count].color;
                    corners[i].color = EdgeColor::from_bits(
                        (color.bits() & next_color.bits()) ^ EdgeColor::White.bits(),
                    );
                } else {
                    color = corners[i].color;
                }
            }
            let mut spline = 0usize;
            let start = corners[0].index;
            color = corners[0].color;
            let m = contour.segments.len();
            for i in 0..m {
                let index = (start + i) % m;
                if spline + 1 < corner_count && corners[spline + 1].index == index {
                    spline += 1;
                    color = corners[spline].color;
                }
                contour.colors[index] = color;
            }
        }
    }
}

// Re-export the experimental distance-based variant.
mod by_distance;
pub use by_distance::edge_coloring_by_distance;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vector2;

    fn square() -> Shape {
        let mut shape = Shape::new();
        let c = shape.add_contour_mut();
        let pts = [
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 1.0),
        ];
        for i in 0..4 {
            c.add_edge(EdgeSegment::line(pts[i], pts[(i + 1) % 4]));
        }
        shape
    }

    #[test]
    fn simple_is_deterministic() {
        let mut a = square();
        let mut b = square();
        edge_coloring_simple(&mut a, 3.0, 0);
        edge_coloring_simple(&mut b, 3.0, 0);
        assert_eq!(a.contours[0].colors, b.contours[0].colors);
        // Every edge must use at least two channels.
        for c in &a.contours[0].colors {
            assert!(c.bits().count_ones() >= 2, "color {c:?} needs 2+ channels");
        }
    }

    #[test]
    fn ink_trap_runs() {
        let mut s = square();
        edge_coloring_ink_trap(&mut s, 3.0, 7);
        assert_eq!(s.contours[0].colors.len(), 4);
    }
}
