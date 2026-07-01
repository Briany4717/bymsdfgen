//! Distance-field generation: projection/mapping, configs, and the generators.

pub mod config;
pub mod generate;
pub mod projection;

pub use config::{
    DistanceCheckMode, ErrorCorrectionConfig, ErrorCorrectionMode, GeneratorConfig,
    MsdfGeneratorConfig,
};
pub use generate::{generate_msdf, generate_mtsdf, generate_psdf, generate_sdf};
pub use projection::{DistanceMapping, Projection, SdfTransformation};
