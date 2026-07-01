//! # bymsdfgen-core
//!
//! The core crate of `bymsdfgen`, a **pure-Rust msdf generator** and pure-Rust
//! alternative to msdfgen: generation of conventional and multi-channel signed
//! distance fields from vector shapes, with no C++ toolchain and no FFI.
//!
//! Design highlights versus the C++ original:
//! - **Zero-cost dispatch**: edges are a `Copy` [`geometry::EdgeSegment`] enum
//!   (`match`) instead of a virtual class hierarchy behind heap pointers.
//! - **Data-oriented layout**: contours store segments in flat, contiguous `Vec`s.
//! - **Typed coordinate spaces**: [`math::typed`] newtypes make space mismatches a
//!   compile error rather than a silent logic bug.
//! - **Fearless parallelism**: row-parallel generation via rayon (feature
//!   `parallel`) with compile-time data-race freedom.

pub mod bitmap;
pub mod coloring;
pub mod correction;
pub mod distance;
pub mod generator;
pub mod geometry;
pub mod math;
pub mod raster;

pub use bitmap::Bitmap;
pub use generator::{
    DistanceMapping, ErrorCorrectionConfig, ErrorCorrectionMode, GeneratorConfig,
    MsdfGeneratorConfig, Projection, SdfTransformation, generate_msdf, generate_mtsdf,
    generate_psdf, generate_sdf,
};
pub use geometry::{Bounds, Contour, EdgeColor, EdgeSegment, Shape};
pub use math::{Range, SignedDistance, Vector2};
pub use raster::{FillRule, Scanline};
