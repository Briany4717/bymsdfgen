//! Per-pixel distance-field generators. Port of `core/msdfgen.cpp` entry points
//! (`generateSDF` / `generatePSDF` / `generateMSDF` / `generateMTSDF`).
//!
//! Parallel row iteration (feature `parallel`, via rayon) is layered on directly
//! in these generators; row order does not affect the result.

use super::config::{GeneratorConfig, MsdfGeneratorConfig};
use super::projection::SdfTransformation;
use crate::bitmap::Bitmap;
use crate::correction::msdf_error_correction;
use crate::distance::combiners::{
    ContourCombiner, OverlappingContourCombiner, SimpleContourCombiner,
};
use crate::distance::finder::ShapeDistanceFinder;
use crate::distance::selectors::{
    MultiAndTrueDistanceSelector, MultiDistanceSelector, PerpendicularDistanceSelector,
    TrueDistanceSelector,
};
use crate::distance::value::DistanceValue;
use crate::geometry::Shape;
use crate::math::typed::PixelPoint;

/// Core generation loop, generic over the contour combiner `C` and channel count.
///
/// With the `parallel` feature each output row is processed on a rayon worker that
/// builds its own [`ShapeDistanceFinder`] (thread-local cache). Rows are disjoint
/// slices, so the borrow checker proves there is no data race at compile time —
/// the "fearless concurrency" replacement for the C++ OpenMP `#pragma`.
pub(crate) fn generate_field<C, const N: usize>(
    output: &mut Bitmap<f32, N>,
    shape: &Shape,
    transformation: &SdfTransformation,
) where
    C: ContourCombiner,
{
    let width = output.width;

    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        output
            .data_mut()
            .par_chunks_mut(width * N)
            .enumerate()
            .for_each(|(y, row)| {
                let mut finder = ShapeDistanceFinder::<C>::new(shape);
                for x in 0..width {
                    let p = transformation
                        .projection
                        .unproject(PixelPoint::new(x as f64 + 0.5, y as f64 + 0.5))
                        .raw();
                    let distance = finder.distance(p);
                    distance
                        .write_mapped(&transformation.distance_mapping, &mut row[x * N..x * N + N]);
                }
            });
    }

    #[cfg(not(feature = "parallel"))]
    {
        let mut finder = ShapeDistanceFinder::<C>::new(shape);
        for y in 0..output.height {
            for x in 0..width {
                let p = transformation
                    .projection
                    .unproject(PixelPoint::new(x as f64 + 0.5, y as f64 + 0.5))
                    .raw();
                let distance = finder.distance(p);
                distance.write_mapped(&transformation.distance_mapping, output.pixel_mut(x, y));
            }
        }
    }
}

/// Generate a conventional (true) single-channel SDF.
pub fn generate_sdf(
    output: &mut Bitmap<f32, 1>,
    shape: &Shape,
    transformation: &SdfTransformation,
    config: &GeneratorConfig,
) {
    if config.overlap_support {
        generate_field::<OverlappingContourCombiner<TrueDistanceSelector>, 1>(
            output,
            shape,
            transformation,
        );
    } else {
        generate_field::<SimpleContourCombiner<TrueDistanceSelector>, 1>(
            output,
            shape,
            transformation,
        );
    }
}

/// Generate a perpendicular single-channel SDF.
pub fn generate_psdf(
    output: &mut Bitmap<f32, 1>,
    shape: &Shape,
    transformation: &SdfTransformation,
    config: &GeneratorConfig,
) {
    if config.overlap_support {
        generate_field::<OverlappingContourCombiner<PerpendicularDistanceSelector>, 1>(
            output,
            shape,
            transformation,
        );
    } else {
        generate_field::<SimpleContourCombiner<PerpendicularDistanceSelector>, 1>(
            output,
            shape,
            transformation,
        );
    }
}

/// Generate a 3-channel MSDF (with error correction).
pub fn generate_msdf(
    output: &mut Bitmap<f32, 3>,
    shape: &Shape,
    transformation: &SdfTransformation,
    config: &MsdfGeneratorConfig,
) {
    if config.overlap_support {
        generate_field::<OverlappingContourCombiner<MultiDistanceSelector>, 3>(
            output,
            shape,
            transformation,
        );
    } else {
        generate_field::<SimpleContourCombiner<MultiDistanceSelector>, 3>(
            output,
            shape,
            transformation,
        );
    }
    msdf_error_correction(output, shape, transformation, config);
}

// (MTSDF below)

/// Generate a 4-channel MTSDF (with error correction).
pub fn generate_mtsdf(
    output: &mut Bitmap<f32, 4>,
    shape: &Shape,
    transformation: &SdfTransformation,
    config: &MsdfGeneratorConfig,
) {
    if config.overlap_support {
        generate_field::<OverlappingContourCombiner<MultiAndTrueDistanceSelector>, 4>(
            output,
            shape,
            transformation,
        );
    } else {
        generate_field::<SimpleContourCombiner<MultiAndTrueDistanceSelector>, 4>(
            output,
            shape,
            transformation,
        );
    }
    msdf_error_correction(output, shape, transformation, config);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generator::{DistanceMapping, Projection};
    use crate::geometry::EdgeSegment;
    use crate::math::{Range, Vector2};

    fn unit_square() -> Shape {
        let mut shape = Shape::new();
        let c = shape.add_contour_mut();
        // Centered with margin so the frame has both inside and outside pixels.
        let pts = [
            Vector2::new(0.5, 0.5),
            Vector2::new(1.5, 0.5),
            Vector2::new(1.5, 1.5),
            Vector2::new(0.5, 1.5),
        ];
        for i in 0..4 {
            c.add_edge(EdgeSegment::line(pts[i], pts[(i + 1) % 4]));
        }
        shape.normalize();
        shape.orient_contours();
        shape
    }

    #[test]
    fn sdf_inside_is_brighter_than_outside() {
        let shape = unit_square();
        let n = 32usize;
        let t = SdfTransformation::new(
            Projection::new(Vector2::splat(n as f64 / 2.0), Vector2::ZERO),
            DistanceMapping::from_range(Range::symmetric(0.25)),
        );
        let mut sdf: Bitmap<f32, 1> = Bitmap::new(n, n);
        generate_sdf(&mut sdf, &shape, &t, &GeneratorConfig::default());
        let center = sdf.pixel(n / 2, n / 2)[0];
        let corner = sdf.pixel(0, 0)[0];
        assert!(center > 0.5, "center {center} should be inside");
        assert!(corner < 0.5, "corner {corner} should be outside");
    }

    #[test]
    fn msdf_median_inside_outside() {
        let shape = unit_square();
        let n = 32usize;
        let t = SdfTransformation::new(
            Projection::new(Vector2::splat(n as f64 / 2.0), Vector2::ZERO),
            DistanceMapping::from_range(Range::symmetric(0.25)),
        );
        let mut msdf: Bitmap<f32, 3> = Bitmap::new(n, n);
        generate_msdf(&mut msdf, &shape, &t, &MsdfGeneratorConfig::default());
        let med = |p: &[f32]| crate::math::scalar::median(p[0] as f64, p[1] as f64, p[2] as f64);
        assert!(med(msdf.pixel(n / 2, n / 2)) > 0.5);
        assert!(med(msdf.pixel(0, 0)) < 0.5);
    }
}
