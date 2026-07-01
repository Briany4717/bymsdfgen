//! Per-edge distance selection and per-contour combination.

pub mod combiners;
pub mod finder;
pub mod selectors;
pub mod value;

pub use combiners::{ContourCombiner, OverlappingContourCombiner, SimpleContourCombiner};
pub use finder::ShapeDistanceFinder;
pub use selectors::{
    EdgeSelector, MultiAndTrueDistanceSelector, MultiDistanceSelector,
    PerpendicularDistanceSelector, TrueDistanceSelector,
};
pub use value::{DistanceValue, MultiAndTrueDistance, MultiDistance};
