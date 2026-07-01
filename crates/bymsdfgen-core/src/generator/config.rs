//! Generator and error-correction configuration. Port of `generator-config.h`.

/// Error-correction mode of operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCorrectionMode {
    /// Skip the error-correction pass.
    Disabled,
    /// Correct all discontinuities regardless of edge impact.
    Indiscriminate,
    /// Correct artifacts only when edges/corners are not affected.
    EdgePriority,
    /// Only correct artifacts at edges.
    EdgeOnly,
}

/// Whether to compute exact shape distance at suspected artifacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceCheckMode {
    DoNotCheckDistance,
    CheckDistanceAtEdge,
    AlwaysCheckDistance,
}

pub const DEFAULT_MIN_DEVIATION_RATIO: f64 = 1.111_111_111_111_111_1;
pub const DEFAULT_MIN_IMPROVE_RATIO: f64 = 1.111_111_111_111_111_1;

/// Configuration of the MSDF error-correction pass.
#[derive(Debug, Clone, Copy)]
pub struct ErrorCorrectionConfig {
    pub mode: ErrorCorrectionMode,
    pub distance_check_mode: DistanceCheckMode,
    pub min_deviation_ratio: f64,
    pub min_improve_ratio: f64,
}

impl Default for ErrorCorrectionConfig {
    fn default() -> Self {
        ErrorCorrectionConfig {
            mode: ErrorCorrectionMode::EdgePriority,
            distance_check_mode: DistanceCheckMode::CheckDistanceAtEdge,
            min_deviation_ratio: DEFAULT_MIN_DEVIATION_RATIO,
            min_improve_ratio: DEFAULT_MIN_IMPROVE_RATIO,
        }
    }
}

/// Configuration of the distance-field generator.
#[derive(Debug, Clone, Copy)]
pub struct GeneratorConfig {
    /// Use the overlapping-contour-aware algorithm.
    pub overlap_support: bool,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        GeneratorConfig {
            overlap_support: true,
        }
    }
}

/// Configuration of the multi-channel generator (adds error correction).
#[derive(Debug, Clone, Copy)]
pub struct MsdfGeneratorConfig {
    pub overlap_support: bool,
    pub error_correction: ErrorCorrectionConfig,
}

impl Default for MsdfGeneratorConfig {
    fn default() -> Self {
        MsdfGeneratorConfig {
            overlap_support: true,
            error_correction: ErrorCorrectionConfig::default(),
        }
    }
}

impl MsdfGeneratorConfig {
    pub fn new(overlap_support: bool, error_correction: ErrorCorrectionConfig) -> Self {
        MsdfGeneratorConfig {
            overlap_support,
            error_correction,
        }
    }
}
