//! Scanline rasterization and distance-field sign correction.
//! Port of `core/rasterization.cpp` and `Shape::scanline`.

use super::scanline::{FillRule, Intersection, Scanline};
use crate::bitmap::Bitmap;
use crate::generator::Projection;
use crate::geometry::Shape;
use crate::math::scalar::median;

/// Fill `line` with the shape's intersections at height `y`.
pub fn shape_scanline(shape: &Shape, line: &mut Scanline, y: f64) {
    let mut intersections: Vec<Intersection> = Vec::new();
    let mut buf = [(0.0, 0); 3];
    for contour in &shape.contours {
        for edge in &contour.segments {
            let n = edge.scanline_intersections(y, &mut buf);
            for &(x, dy) in buf.iter().take(n) {
                intersections.push(Intersection { x, direction: dy });
            }
        }
    }
    line.set_intersections(intersections);
}

/// Rasterize the shape into a monochrome coverage bitmap.
pub fn rasterize(
    output: &mut Bitmap<f32, 1>,
    shape: &Shape,
    projection: &Projection,
    fill: FillRule,
) {
    let mut scanline = Scanline::new();
    for y in 0..output.height {
        shape_scanline(shape, &mut scanline, projection.unproject_y(y as f64 + 0.5));
        for x in 0..output.width {
            let f = scanline.filled(projection.unproject_x(x as f64 + 0.5), fill);
            output.pixel_mut(x, y)[0] = f as i32 as f32;
        }
    }
}

/// Flip the sign of single-channel SDF pixels whose sign disagrees with the
/// rasterized fill. Port of `distanceSignCorrection` (1 channel).
pub fn distance_sign_correction_1(
    sdf: &mut Bitmap<f32, 1>,
    shape: &Shape,
    projection: &Projection,
    sdf_zero_value: f32,
    fill: FillRule,
) {
    let double_zero = sdf_zero_value + sdf_zero_value;
    let mut scanline = Scanline::new();
    for y in 0..sdf.height {
        shape_scanline(shape, &mut scanline, projection.unproject_y(y as f64 + 0.5));
        for x in 0..sdf.width {
            let filled = scanline.filled(projection.unproject_x(x as f64 + 0.5), fill);
            let sd = &mut sdf.pixel_mut(x, y)[0];
            if (*sd > sdf_zero_value) != filled {
                *sd = double_zero - *sd;
            }
        }
    }
}

/// Multi-channel sign correction (3 or 4 channels). Port of
/// `multiDistanceSignCorrection`.
pub fn distance_sign_correction_multi<const N: usize>(
    sdf: &mut Bitmap<f32, N>,
    shape: &Shape,
    projection: &Projection,
    sdf_zero_value: f32,
    fill: FillRule,
) {
    let w = sdf.width;
    let h = sdf.height;
    if w == 0 || h == 0 {
        return;
    }
    let double_zero = sdf_zero_value + sdf_zero_value;
    let mut scanline = Scanline::new();
    let mut ambiguous = false;
    let mut match_map = vec![0i8; w * h];

    for y in 0..h {
        shape_scanline(shape, &mut scanline, projection.unproject_y(y as f64 + 0.5));
        for x in 0..w {
            let filled = scanline.filled(projection.unproject_x(x as f64 + 0.5), fill);
            let msd = sdf.pixel_mut(x, y);
            let sd = median(msd[0] as f64, msd[1] as f64, msd[2] as f64) as f32;
            if sd == sdf_zero_value {
                ambiguous = true;
            } else if (sd > sdf_zero_value) != filled {
                msd[0] = double_zero - msd[0];
                msd[1] = double_zero - msd[1];
                msd[2] = double_zero - msd[2];
                match_map[y * w + x] = -1;
            } else {
                match_map[y * w + x] = 1;
            }
            if N >= 4 && (msd[3] > sdf_zero_value) != filled {
                msd[3] = double_zero - msd[3];
            }
        }
    }

    // Resolve ambiguous pixels by neighbour vote (avoids inverted-shape artifacts).
    if ambiguous {
        for y in 0..h {
            for x in 0..w {
                if match_map[y * w + x] == 0 {
                    let mut neighbor = 0i32;
                    if x > 0 {
                        neighbor += match_map[y * w + x - 1] as i32;
                    }
                    if x < w - 1 {
                        neighbor += match_map[y * w + x + 1] as i32;
                    }
                    if y > 0 {
                        neighbor += match_map[(y - 1) * w + x] as i32;
                    }
                    if y < h - 1 {
                        neighbor += match_map[(y + 1) * w + x] as i32;
                    }
                    if neighbor < 0 {
                        let msd = sdf.pixel_mut(x, y);
                        msd[0] = double_zero - msd[0];
                        msd[1] = double_zero - msd[1];
                        msd[2] = double_zero - msd[2];
                    }
                }
            }
        }
    }
}
