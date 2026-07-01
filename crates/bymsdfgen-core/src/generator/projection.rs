//! Spatial and distance-value transformations.
//! Ports of `Projection`, `DistanceMapping`, `SDFTransformation`.

use crate::math::typed::{PixelPoint, ShapePoint};
use crate::math::{Range, Vector2};

/// Affine transformation between shape space and pixel space.
#[derive(Debug, Clone, Copy)]
pub struct Projection {
    scale: Vector2,
    translate: Vector2,
}

impl Default for Projection {
    fn default() -> Self {
        Projection {
            scale: Vector2::splat(1.0),
            translate: Vector2::ZERO,
        }
    }
}

impl Projection {
    pub fn new(scale: Vector2, translate: Vector2) -> Self {
        Projection { scale, translate }
    }

    /// Shape coordinate → pixel coordinate (typed boundary crossing).
    #[inline]
    pub fn project(&self, coord: ShapePoint) -> PixelPoint {
        PixelPoint::from_raw(self.scale * (coord.raw() + self.translate))
    }

    /// Pixel coordinate → shape coordinate (typed boundary crossing).
    #[inline]
    pub fn unproject(&self, coord: PixelPoint) -> ShapePoint {
        ShapePoint::from_raw(coord.raw() / self.scale - self.translate)
    }

    #[inline]
    pub fn project_vector(&self, v: Vector2) -> Vector2 {
        self.scale * v
    }

    #[inline]
    pub fn unproject_vector(&self, v: Vector2) -> Vector2 {
        v / self.scale
    }

    #[inline]
    pub fn project_x(&self, x: f64) -> f64 {
        self.scale.x * (x + self.translate.x)
    }
    #[inline]
    pub fn project_y(&self, y: f64) -> f64 {
        self.scale.y * (y + self.translate.y)
    }
    #[inline]
    pub fn unproject_x(&self, x: f64) -> f64 {
        x / self.scale.x - self.translate.x
    }
    #[inline]
    pub fn unproject_y(&self, y: f64) -> f64 {
        y / self.scale.y - self.translate.y
    }

    #[inline]
    pub fn scale(&self) -> Vector2 {
        self.scale
    }
    #[inline]
    pub fn translate(&self) -> Vector2 {
        self.translate
    }
}

/// Linear mapping of signed distance values into normalized field values.
#[derive(Debug, Clone, Copy)]
pub struct DistanceMapping {
    scale: f64,
    translate: f64,
}

impl Default for DistanceMapping {
    fn default() -> Self {
        DistanceMapping {
            scale: 1.0,
            translate: 0.0,
        }
    }
}

impl DistanceMapping {
    /// Maps `[range.lower, range.upper]` onto `[0, 1]`.
    pub fn from_range(range: Range) -> Self {
        DistanceMapping {
            scale: 1.0 / (range.upper - range.lower),
            translate: -range.lower,
        }
    }

    /// The inverse mapping that takes a `range` back from normalized values.
    pub fn inverse_of_range(range: Range) -> Self {
        let range_width = range.upper - range.lower;
        DistanceMapping {
            scale: range_width,
            translate: range.lower / if range_width != 0.0 { range_width } else { 1.0 },
        }
    }

    fn raw(scale: f64, translate: f64) -> Self {
        DistanceMapping { scale, translate }
    }

    /// Map an absolute distance.
    #[inline]
    pub fn map(&self, d: f64) -> f64 {
        self.scale * (d + self.translate)
    }

    /// Map a distance *delta* (no translation applied).
    #[inline]
    pub fn map_delta(&self, d: f64) -> f64 {
        self.scale * d
    }

    pub fn inverse(&self) -> Self {
        DistanceMapping::raw(1.0 / self.scale, -self.scale * self.translate)
    }
}

/// Combined spatial + distance-value transformation.
#[derive(Debug, Clone, Copy, Default)]
pub struct SdfTransformation {
    pub projection: Projection,
    pub distance_mapping: DistanceMapping,
}

impl SdfTransformation {
    pub fn new(projection: Projection, distance_mapping: DistanceMapping) -> Self {
        SdfTransformation {
            projection,
            distance_mapping,
        }
    }
}
