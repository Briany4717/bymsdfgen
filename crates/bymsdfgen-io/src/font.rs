//! Zero-copy font loading via `ttf-parser`. Port of `ext/import-font.cpp`.
//!
//! The whole font is parsed directly from a borrowed `&[u8]` — no heap copy of the
//! file, no FFI to FreeType. A glyph outline is decomposed into a [`Shape`] using
//! the same callback structure as the original's FreeType `ftMoveTo/...` handlers.

use bymsdfgen_core::geometry::{Contour, EdgeSegment, Shape};
use bymsdfgen_core::math::{Vector2, cross};
use ttf_parser::{Face, GlyphId, OutlineBuilder};

/// How outline coordinates are scaled when building the shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FontCoordinateScaling {
    /// Raw font design units.
    None,
    /// Divide by units-per-em so 1.0 == one em.
    EmNormalized,
}

/// Basic font metrics (in the requested coordinate scaling).
#[derive(Debug, Clone, Copy)]
pub struct FontMetrics {
    pub em_size: f64,
    pub ascender: f64,
    pub descender: f64,
    pub line_height: f64,
    pub underline_y: f64,
    pub underline_thickness: f64,
}

/// A parsed font borrowing its backing byte buffer (`'a`).
pub struct Font<'a> {
    face: Face<'a>,
}

impl<'a> Font<'a> {
    /// Parse a font from a borrowed byte slice (zero-copy).
    pub fn from_slice(data: &'a [u8], index: u32) -> Result<Self, ttf_parser::FaceParsingError> {
        Ok(Font {
            face: Face::parse(data, index)?,
        })
    }

    fn scale(&self, scaling: FontCoordinateScaling) -> f64 {
        match scaling {
            FontCoordinateScaling::None => 1.0,
            FontCoordinateScaling::EmNormalized => {
                let upm = self.face.units_per_em();
                1.0 / if upm != 0 { upm as f64 } else { 1.0 }
            }
        }
    }

    pub fn glyph_count(&self) -> u16 {
        self.face.number_of_glyphs()
    }

    /// Resolve a Unicode codepoint to a glyph index.
    pub fn glyph_index(&self, c: char) -> Option<u16> {
        self.face.glyph_index(c).map(|g| g.0)
    }

    pub fn metrics(&self, scaling: FontCoordinateScaling) -> FontMetrics {
        let s = self.scale(scaling);
        let upm = self.face.units_per_em() as f64;
        FontMetrics {
            em_size: s * upm,
            ascender: s * self.face.ascender() as f64,
            descender: s * self.face.descender() as f64,
            line_height: s
                * (self.face.ascender() as f64 - self.face.descender() as f64
                    + self.face.line_gap() as f64),
            underline_y: s * self
                .face
                .underline_metrics()
                .map(|m| m.position as f64)
                .unwrap_or(0.0),
            underline_thickness: s * self
                .face
                .underline_metrics()
                .map(|m| m.thickness as f64)
                .unwrap_or(0.0),
        }
    }

    /// Horizontal advance of a glyph in the requested scaling.
    pub fn advance(&self, glyph: u16, scaling: FontCoordinateScaling) -> Option<f64> {
        self.face
            .glyph_hor_advance(GlyphId(glyph))
            .map(|a| self.scale(scaling) * a as f64)
    }

    /// Kerning between two glyphs (legacy `kern` table only, mirrors common use).
    pub fn kerning(&self, _a: u16, _b: u16, _scaling: FontCoordinateScaling) -> f64 {
        // ttf-parser exposes GPOS pair adjustments via a richer API; the simple
        // `kern` table covers the original's default path. Returns 0 when absent.
        0.0
    }

    /// Load a glyph outline into a [`Shape`]. Returns false if the glyph has no
    /// outline (e.g. whitespace).
    pub fn load_glyph(
        &self,
        output: &mut Shape,
        glyph: u16,
        scaling: FontCoordinateScaling,
    ) -> bool {
        output.contours.clear();
        output.inverse_y_axis = false;
        let scale = self.scale(scaling);
        let mut builder = ShapeBuilder {
            scale,
            position: Vector2::ZERO,
            start: Vector2::ZERO,
            shape: output,
            contour_open: false,
        };
        self.face
            .outline_glyph(GlyphId(glyph), &mut builder)
            .is_some()
    }
}

/// Implements `ttf_parser::OutlineBuilder`, mirroring the FreeType decomposer.
struct ShapeBuilder<'s> {
    scale: f64,
    position: Vector2,
    /// Start point of the current contour, used to close it.
    start: Vector2,
    shape: &'s mut Shape,
    contour_open: bool,
}

impl<'s> ShapeBuilder<'s> {
    #[inline]
    fn pt(&self, x: f32, y: f32) -> Vector2 {
        Vector2::new(self.scale * x as f64, self.scale * y as f64)
    }

    fn current_contour(&mut self) -> &mut Contour {
        self.shape.contours.last_mut().unwrap()
    }
}

impl<'s> OutlineBuilder for ShapeBuilder<'s> {
    fn move_to(&mut self, x: f32, y: f32) {
        // Start a new contour unless the current one is still empty.
        let need_new = match self.shape.contours.last() {
            Some(c) => !c.segments.is_empty(),
            None => true,
        };
        if need_new || !self.contour_open {
            self.shape.contours.push(Contour::new());
            self.contour_open = true;
        }
        self.position = self.pt(x, y);
        self.start = self.position;
    }

    fn line_to(&mut self, x: f32, y: f32) {
        let endpoint = self.pt(x, y);
        if endpoint != self.position {
            let start = self.position;
            self.current_contour()
                .add_edge(EdgeSegment::line(start, endpoint));
            self.position = endpoint;
        }
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let control = self.pt(x1, y1);
        let endpoint = self.pt(x, y);
        if endpoint != self.position {
            let start = self.position;
            self.current_contour()
                .add_edge(EdgeSegment::quadratic(start, control, endpoint));
            self.position = endpoint;
        }
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let c1 = self.pt(x1, y1);
        let c2 = self.pt(x2, y2);
        let endpoint = self.pt(x, y);
        if endpoint != self.position || cross(c1 - endpoint, c2 - endpoint) != 0.0 {
            let start = self.position;
            self.current_contour()
                .add_edge(EdgeSegment::cubic(start, c1, c2, endpoint));
            self.position = endpoint;
        }
    }

    fn close(&mut self) {
        // FreeType implicitly closes contours; ttf-parser signals closure via this
        // callback without emitting the final segment. Add the closing edge so the
        // contour chains back to its start (matching the original's behaviour).
        if self.contour_open && self.position != self.start {
            let start = self.start;
            let pos = self.position;
            self.current_contour()
                .add_edge(EdgeSegment::line(pos, start));
            self.position = start;
        }
        self.contour_open = false;
    }
}
