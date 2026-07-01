//! A shape: a set of closed contours plus a Y-axis orientation flag.
//! Port of `core/Shape.{h,cpp}`.

use super::contour::Contour;
use super::convergent::convergent_curve_ordering;
use super::edge_segment::EdgeSegment;
use crate::math::scalar::{mix, sign};
use crate::math::{Vector2, dot};

/// Matches `MSDFGEN_CORNER_DOT_EPSILON` in `Shape.h`.
const CORNER_DOT_EPSILON: f64 = 0.000_001;
/// Matches `DECONVERGE_OVERSHOOT` in `Shape.cpp`.
const DECONVERGE_OVERSHOOT: f64 = 1.111_111_111_111_111_1;
const LARGE_VALUE: f64 = 1e240;

/// Axis-aligned bounds: left, bottom, right, top.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub l: f64,
    pub b: f64,
    pub r: f64,
    pub t: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Shape {
    pub contours: Vec<Contour>,
    /// When true, the shape's Y axis points downward (non-default orientation).
    pub inverse_y_axis: bool,
}

fn deconverge_edge(edge: &mut EdgeSegment, param: i32, vector: Vector2) {
    if matches!(edge, EdgeSegment::Quadratic(_)) {
        *edge = edge.convert_to_cubic();
    }
    if let EdgeSegment::Cubic(p) = edge {
        match param {
            0 => p[1] = p[1] + (p[1] - p[0]).length() * vector,
            1 => p[2] = p[2] + (p[2] - p[3]).length() * vector,
            _ => {}
        }
    }
}

impl Shape {
    pub fn new() -> Self {
        Shape::default()
    }

    pub fn add_contour(&mut self, contour: Contour) {
        self.contours.push(contour);
    }

    /// Push and return a mutable reference to a fresh contour.
    pub fn add_contour_mut(&mut self) -> &mut Contour {
        self.contours.push(Contour::new());
        self.contours.last_mut().unwrap()
    }

    /// Every contour must be closed and consistent (each edge starts where the
    /// previous ended). Port of `Shape::validate`.
    pub fn validate(&self) -> bool {
        for contour in &self.contours {
            if let Some(last) = contour.segments.last() {
                let mut corner = last.point(1.0);
                for edge in &contour.segments {
                    if edge.point(0.0) != corner {
                        return false;
                    }
                    corner = edge.point(1.0);
                }
            }
        }
        true
    }

    /// Split single-edge contours into thirds and push apart convergent corners.
    /// Port of `Shape::normalize`.
    pub fn normalize(&mut self) {
        for contour in &mut self.contours {
            let n = contour.segments.len();
            if n == 1 {
                let parts = contour.segments[0].split_in_thirds();
                let color = contour.colors[0];
                contour.segments = parts.to_vec();
                contour.colors = vec![color; 3];
            } else if n >= 2 {
                let mut prev_idx = n - 1;
                for cur_idx in 0..n {
                    let prev_dir = contour.segments[prev_idx].direction(1.0).normalize(false);
                    let cur_dir = contour.segments[cur_idx].direction(0.0).normalize(false);
                    if dot(prev_dir, cur_dir) < CORNER_DOT_EPSILON - 1.0 {
                        let e = CORNER_DOT_EPSILON - 1.0;
                        let factor = DECONVERGE_OVERSHOOT * (1.0 - e * e).sqrt() / e;
                        let mut axis = factor * (cur_dir - prev_dir).normalize(false);
                        if convergent_curve_ordering(
                            &contour.segments[prev_idx],
                            &contour.segments[cur_idx],
                        ) < 0
                        {
                            axis = -axis;
                        }
                        deconverge_edge(&mut contour.segments[prev_idx], 1, axis.orthogonal(true));
                        deconverge_edge(&mut contour.segments[cur_idx], 0, axis.orthogonal(false));
                    }
                    prev_idx = cur_idx;
                }
            }
        }
    }

    /// Expand `(min, max)` over all contours.
    pub fn bound(&self, min: &mut Vector2, max: &mut Vector2) {
        for c in &self.contours {
            c.bound(min, max);
        }
    }

    /// Axis-aligned bounds, optionally expanded by a uniform `border`.
    pub fn get_bounds(&self, border: f64) -> Bounds {
        let mut min = Vector2::new(LARGE_VALUE, LARGE_VALUE);
        let mut max = Vector2::new(-LARGE_VALUE, -LARGE_VALUE);
        self.bound(&mut min, &mut max);
        let mut bounds = Bounds {
            l: min.x,
            b: min.y,
            r: max.x,
            t: max.y,
        };
        if border > 0.0 {
            bounds.l -= border;
            bounds.b -= border;
            bounds.r += border;
            bounds.t += border;
        }
        bounds
    }

    pub fn edge_count(&self) -> usize {
        self.contours.iter().map(|c| c.segments.len()).sum()
    }

    /// Ensure all contours wind consistently (outer CCW, holes CW). Port of
    /// `Shape::orientContours`.
    pub fn orient_contours(&mut self) {
        #[derive(Clone, Copy)]
        struct Inter {
            x: f64,
            direction: i32,
            contour_index: usize,
        }

        let ratio = 0.5 * (5.0_f64.sqrt() - 1.0); // irrational, avoids hitting corners
        let nc = self.contours.len();
        let mut orientations = vec![0i32; nc];
        let mut intersections: Vec<Inter> = Vec::new();
        let mut buf = [(0.0, 0); 3];

        for i in 0..nc {
            if orientations[i] != 0 || self.contours[i].segments.is_empty() {
                continue;
            }
            // Find a Y that crosses the contour.
            let y0 = self.contours[i].segments[0].point(0.0).y;
            let mut y1 = y0;
            for edge in &self.contours[i].segments {
                if y0 != y1 {
                    break;
                }
                y1 = edge.point(1.0).y;
            }
            for edge in &self.contours[i].segments {
                if y0 != y1 {
                    break;
                }
                y1 = edge.point(ratio).y; // horizontal-line fallback
            }
            let y = mix(y0, y1, ratio);

            for (j, contour) in self.contours.iter().enumerate() {
                for edge in &contour.segments {
                    let n = edge.scanline_intersections(y, &mut buf);
                    for &(x, dy) in buf.iter().take(n) {
                        intersections.push(Inter {
                            x,
                            direction: dy,
                            contour_index: j,
                        });
                    }
                }
            }

            if !intersections.is_empty() {
                intersections
                    .sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal));
                // Disqualify coincident intersections.
                for j in 1..intersections.len() {
                    if intersections[j].x == intersections[j - 1].x {
                        intersections[j].direction = 0;
                        intersections[j - 1].direction = 0;
                    }
                }
                for (j, inter) in intersections.iter().enumerate() {
                    if inter.direction != 0 {
                        orientations[inter.contour_index] +=
                            2 * (((j & 1) as i32) ^ ((inter.direction > 0) as i32)) - 1;
                    }
                }
                intersections.clear();
            }
        }

        for (i, contour) in self.contours.iter_mut().enumerate() {
            if orientations[i] < 0 {
                contour.reverse();
            }
        }
        // suppress unused warning of `sign` import in some build configs
        let _ = sign;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::edge_color::EdgeColor;

    fn square() -> Shape {
        let mut shape = Shape::new();
        let c = shape.add_contour_mut();
        let pts = [
            Vector2::new(0.0, 0.0),
            Vector2::new(10.0, 0.0),
            Vector2::new(10.0, 10.0),
            Vector2::new(0.0, 10.0),
        ];
        for i in 0..4 {
            c.add_edge_colored(
                EdgeSegment::line(pts[i], pts[(i + 1) % 4]),
                EdgeColor::White,
            );
        }
        shape
    }

    #[test]
    fn validate_closed_square() {
        assert!(square().validate());
    }

    #[test]
    fn bounds_square() {
        let b = square().get_bounds(0.0);
        assert_eq!((b.l, b.b, b.r, b.t), (0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn winding_and_orient() {
        let mut s = square();
        // msdfgen's shoelace convention yields -1 for this CCW vertex order.
        assert_eq!(s.contours[0].winding(), -1);
        s.orient_contours();
        // After orientation a lone outer contour stays a valid, non-degenerate loop.
        assert!(s.contours[0].winding() != 0);
    }

    #[test]
    fn single_edge_contour_splits() {
        let mut s = Shape::new();
        let c = s.add_contour_mut();
        // a single cubic loop
        c.add_edge(EdgeSegment::cubic(
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 4.0),
            Vector2::new(-2.0, 4.0),
            Vector2::new(0.0, 0.0),
        ));
        s.normalize();
        assert_eq!(s.contours[0].segments.len(), 3);
    }
}
