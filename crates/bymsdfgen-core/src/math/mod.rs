//! Math foundations: vectors, scalar helpers, signed distance, ranges and the
//! zero-cost coordinate-space newtypes.

pub mod range;
pub mod scalar;
pub mod signed_distance;
pub mod typed;
pub mod vector2;

pub use range::Range;
pub use signed_distance::SignedDistance;
pub use typed::{
    DesignPoint, DesignSpace, EmPoint, EmSpace, PixelPoint, PixelSpace, Point, ShapePoint,
    ShapeSpace, Space,
};
pub use vector2::{Point2, Vector2, cross, dot};
