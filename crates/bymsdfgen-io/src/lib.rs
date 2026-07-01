//! I/O extensions for `bymsdfgen`, the pure-Rust msdf generator and pure-Rust
//! alternative to msdfgen: font loading (ttf-parser), image/raw output, and the
//! text shape-description format. All pure Rust — no native FFI.

pub mod font;
pub mod image_out;
pub mod shape_desc;
pub mod svg;

pub use font::{Font, FontCoordinateScaling, FontMetrics};
pub use image_out::{ImageFormat, save};
pub use shape_desc::{parse_shape, write_shape};
pub use svg::load_svg_shape;
