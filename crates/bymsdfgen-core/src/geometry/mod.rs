//! Data-oriented shape geometry: edge segments, contours, shapes.

pub mod contour;
pub mod convergent;
pub mod edge_color;
pub mod edge_segment;
pub mod equation_solver;
pub mod shape;

pub use contour::Contour;
pub use convergent::convergent_curve_ordering;
pub use edge_color::EdgeColor;
pub use edge_segment::EdgeSegment;
pub use shape::{Bounds, Shape};
