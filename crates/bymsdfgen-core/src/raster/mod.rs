//! Scanline rasterization and sign correction.

pub mod rasterization;
pub mod scanline;

pub use rasterization::{
    distance_sign_correction_1, distance_sign_correction_multi, rasterize, shape_scanline,
};
pub use scanline::{FillRule, Intersection, Scanline};
