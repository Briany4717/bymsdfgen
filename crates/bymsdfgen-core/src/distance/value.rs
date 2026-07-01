//! Distance value types and the trait abstracting over them.
//! Ports the `MultiDistance` / `MultiAndTrueDistance` structs plus the
//! `initDistance` / `resolveDistance` helpers from `contour-combiners.cpp`.

use crate::generator::DistanceMapping;
use crate::math::scalar::median;

/// One distance per colour channel.
#[derive(Debug, Clone, Copy)]
pub struct MultiDistance {
    pub r: f64,
    pub g: f64,
    pub b: f64,
}

/// Multi-channel distance plus a true-distance alpha channel.
#[derive(Debug, Clone, Copy)]
pub struct MultiAndTrueDistance {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

/// Trait implemented by every distance flavour a selector can produce.
pub trait DistanceValue: Copy {
    /// The "uninitialized" value (`-DBL_MAX` everywhere).
    fn neg_max() -> Self;
    /// Collapse to a single scalar (median for multi-channel).
    fn resolve(self) -> f64;
    /// Write the mapped distance value(s) into a pixel's channel slice.
    fn write_mapped(self, mapping: &DistanceMapping, px: &mut [f32]);
}

impl DistanceValue for f64 {
    #[inline]
    fn neg_max() -> Self {
        f64::MIN
    }
    #[inline]
    fn resolve(self) -> f64 {
        self
    }
    #[inline]
    fn write_mapped(self, mapping: &DistanceMapping, px: &mut [f32]) {
        px[0] = mapping.map(self) as f32;
    }
}

impl DistanceValue for MultiDistance {
    #[inline]
    fn neg_max() -> Self {
        MultiDistance {
            r: f64::MIN,
            g: f64::MIN,
            b: f64::MIN,
        }
    }
    #[inline]
    fn resolve(self) -> f64 {
        median(self.r, self.g, self.b)
    }
    #[inline]
    fn write_mapped(self, mapping: &DistanceMapping, px: &mut [f32]) {
        px[0] = mapping.map(self.r) as f32;
        px[1] = mapping.map(self.g) as f32;
        px[2] = mapping.map(self.b) as f32;
    }
}

impl DistanceValue for MultiAndTrueDistance {
    #[inline]
    fn neg_max() -> Self {
        MultiAndTrueDistance {
            r: f64::MIN,
            g: f64::MIN,
            b: f64::MIN,
            a: f64::MIN,
        }
    }
    #[inline]
    fn resolve(self) -> f64 {
        median(self.r, self.g, self.b)
    }
    #[inline]
    fn write_mapped(self, mapping: &DistanceMapping, px: &mut [f32]) {
        px[0] = mapping.map(self.r) as f32;
        px[1] = mapping.map(self.g) as f32;
        px[2] = mapping.map(self.b) as f32;
        px[3] = mapping.map(self.a) as f32;
    }
}
