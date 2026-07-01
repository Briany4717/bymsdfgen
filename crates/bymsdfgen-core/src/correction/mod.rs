//! MSDF error correction. Port of `core/MSDFErrorCorrection.cpp` and
//! `core/msdf-error-correction.cpp` (modern, non-legacy path).
//!
//! Detects bilinear-interpolation artifacts in a multi-channel SDF and collapses
//! affected texels to single-channel. The artifact classifier is expressed as a
//! trait with two implementations (SDF-only and exact-shape-distance) so the
//! detection routines stay shared, mirroring the C++ templates.

use std::cell::RefCell;

use crate::bitmap::Bitmap;
use crate::distance::combiners::{
    ContourCombiner, OverlappingContourCombiner, SimpleContourCombiner,
};
use crate::distance::finder::ShapeDistanceFinder;
use crate::distance::selectors::PerpendicularDistanceSelector;
use crate::generator::config::{DistanceCheckMode, ErrorCorrectionMode, MsdfGeneratorConfig};
use crate::generator::projection::SdfTransformation;
use crate::geometry::Shape;
use crate::math::Vector2;
use crate::math::scalar::{clamp, median as median_f64, mix};
use crate::math::typed::{PixelPoint, ShapePoint};

const ARTIFACT_T_EPSILON: f64 = 0.01;
const PROTECTION_RADIUS_TOLERANCE: f64 = 1.001;

const FLAG_ERROR: u8 = 1;
const FLAG_PROTECTED: u8 = 2;

const CLASSIFIER_FLAG_CANDIDATE: i32 = 0x01;
const CLASSIFIER_FLAG_ARTIFACT: i32 = 0x02;

#[inline]
fn medf(p: &[f32]) -> f32 {
    median_f64(p[0] as f64, p[1] as f64, p[2] as f64) as f32
}

#[inline]
fn mixf(a: f32, b: f32, t: f64) -> f32 {
    mix(a as f64, b as f64, t) as f32
}

/// Bilinear sample of an N-channel bitmap at pixel-space `pos`. Port of `interpolate`.
fn interpolate<const N: usize>(out: &mut [f32], sdf: &Bitmap<f32, N>, mut pos: Vector2) {
    pos.x = clamp(pos.x, 0.0, sdf.width as f64);
    pos.y = clamp(pos.y, 0.0, sdf.height as f64);
    pos.x -= 0.5;
    pos.y -= 0.5;
    let mut l = pos.x.floor() as isize;
    let mut b = pos.y.floor() as isize;
    let mut r = l + 1;
    let mut t = b + 1;
    let lr = pos.x - l as f64;
    let bt = pos.y - b as f64;
    let cw = sdf.width as isize - 1;
    let ch = sdf.height as isize - 1;
    l = l.clamp(0, cw);
    r = r.clamp(0, cw);
    b = b.clamp(0, ch);
    t = t.clamp(0, ch);
    let (l, r, b, t) = (l as usize, r as usize, b as usize, t as usize);
    for (i, out) in out.iter_mut().enumerate().take(N) {
        *out = mixf(
            mixf(sdf.pixel(l, b)[i], sdf.pixel(r, b)[i], lr),
            mixf(sdf.pixel(l, t)[i], sdf.pixel(r, t)[i], lr),
            bt,
        );
    }
}

// --- artifact classifier ---------------------------------------------------

/// Shared classifier interface. `range_test` is identical for all classifiers;
/// `evaluate` differs (SDF-only vs. exact-distance).
trait Classifier {
    fn span(&self) -> f64;
    fn protected_flag(&self) -> bool;

    fn range_test(&self, at: f64, bt: f64, xt: f64, am: f32, bm: f32, xm: f32) -> i32 {
        let span = self.span();
        if (am > 0.5 && bm > 0.5 && xm <= 0.5)
            || (am < 0.5 && bm < 0.5 && xm >= 0.5)
            || (!self.protected_flag() && medf3(am, bm, xm) != xm)
        {
            let ax_span = (xt - at) * span;
            let bx_span = (bt - xt) * span;
            if !(xm as f64 >= am as f64 - ax_span
                && xm as f64 <= am as f64 + ax_span
                && xm as f64 >= bm as f64 - bx_span
                && xm as f64 <= bm as f64 + bx_span)
            {
                return CLASSIFIER_FLAG_CANDIDATE | CLASSIFIER_FLAG_ARTIFACT;
            }
            return CLASSIFIER_FLAG_CANDIDATE;
        }
        0
    }

    fn evaluate(&self, t: f64, m: f32, flags: i32) -> bool;
}

#[inline]
fn medf3(a: f32, b: f32, c: f32) -> f32 {
    median_f64(a as f64, b as f64, c as f64) as f32
}

/// SDF-only classifier.
struct BaseClassifier {
    span: f64,
    protected_flag: bool,
}

impl Classifier for BaseClassifier {
    fn span(&self) -> f64 {
        self.span
    }
    fn protected_flag(&self) -> bool {
        self.protected_flag
    }
    fn evaluate(&self, _t: f64, _m: f32, flags: i32) -> bool {
        flags & CLASSIFIER_FLAG_ARTIFACT != 0
    }
}

// --- interpolation helpers used by the detectors ---------------------------

fn interpolated_median_linear(a: &[f32], b: &[f32], t: f64) -> f32 {
    medf3(
        mixf(a[0], b[0], t),
        mixf(a[1], b[1], t),
        mixf(a[2], b[2], t),
    )
}

fn interpolated_median_quad(a: &[f32], l: &[f32], q: &[f32], t: f64) -> f32 {
    let f = |a: f32, l: f32, q: f32| (t * (t * q as f64 + l as f64) + a as f64) as f32;
    medf3(
        f(a[0], l[0], q[0]),
        f(a[1], l[1], q[1]),
        f(a[2], l[2], q[2]),
    )
}

fn has_linear_artifact_inner<C: Classifier>(
    classifier: &C,
    am: f32,
    bm: f32,
    a: &[f32],
    b: &[f32],
    d_a: f32,
    d_b: f32,
) -> bool {
    let t = d_a as f64 / (d_a - d_b) as f64;
    if t > ARTIFACT_T_EPSILON && t < 1.0 - ARTIFACT_T_EPSILON {
        let xm = interpolated_median_linear(a, b, t);
        return classifier.evaluate(t, xm, classifier.range_test(0.0, 1.0, t, am, bm, xm));
    }
    false
}

fn has_linear_artifact<C: Classifier>(classifier: &C, am: f32, a: &[f32], b: &[f32]) -> bool {
    let bm = medf(b);
    (am - 0.5).abs() >= (bm - 0.5).abs()
        && (has_linear_artifact_inner(classifier, am, bm, a, b, a[1] - a[0], b[1] - b[0])
            || has_linear_artifact_inner(classifier, am, bm, a, b, a[2] - a[1], b[2] - b[1])
            || has_linear_artifact_inner(classifier, am, bm, a, b, a[0] - a[2], b[0] - b[2]))
}

#[allow(clippy::too_many_arguments)]
fn has_diagonal_artifact_inner<C: Classifier>(
    classifier: &C,
    am: f32,
    dm: f32,
    a: &[f32],
    l: &[f32],
    q: &[f32],
    d_a: f32,
    d_bc: f32,
    d_d: f32,
    t_ex0: f64,
    t_ex1: f64,
) -> bool {
    use crate::geometry::equation_solver::solve_quadratic;
    let solutions = solve_quadratic(
        (d_d - d_bc + d_a) as f64,
        (d_bc - d_a - d_a) as f64,
        d_a as f64,
    )
    .roots();
    for &ti in solutions.iter() {
        if ti > ARTIFACT_T_EPSILON && ti < 1.0 - ARTIFACT_T_EPSILON {
            let xm = interpolated_median_quad(a, l, q, ti);
            let mut range_flags = classifier.range_test(0.0, 1.0, ti, am, dm, xm);
            let mut t_end = [0.0f64; 2];
            let mut em = [0.0f32; 2];
            if t_ex0 > 0.0 && t_ex0 < 1.0 {
                t_end[0] = 0.0;
                t_end[1] = 1.0;
                em[0] = am;
                em[1] = dm;
                let idx = (t_ex0 > ti) as usize;
                t_end[idx] = t_ex0;
                em[idx] = interpolated_median_quad(a, l, q, t_ex0);
                range_flags |= classifier.range_test(t_end[0], t_end[1], ti, em[0], em[1], xm);
            }
            if t_ex1 > 0.0 && t_ex1 < 1.0 {
                t_end[0] = 0.0;
                t_end[1] = 1.0;
                em[0] = am;
                em[1] = dm;
                let idx = (t_ex1 > ti) as usize;
                t_end[idx] = t_ex1;
                em[idx] = interpolated_median_quad(a, l, q, t_ex1);
                range_flags |= classifier.range_test(t_end[0], t_end[1], ti, em[0], em[1], xm);
            }
            if classifier.evaluate(ti, xm, range_flags) {
                return true;
            }
        }
    }
    false
}

fn has_diagonal_artifact<C: Classifier>(
    classifier: &C,
    am: f32,
    a: &[f32],
    b: &[f32],
    c: &[f32],
    d: &[f32],
) -> bool {
    let dm = medf(d);
    if (am - 0.5).abs() >= (dm - 0.5).abs() {
        let abc = [a[0] - b[0] - c[0], a[1] - b[1] - c[1], a[2] - b[2] - c[2]];
        let l = [-a[0] - abc[0], -a[1] - abc[1], -a[2] - abc[2]];
        let q = [d[0] + abc[0], d[1] + abc[1], d[2] + abc[2]];
        let t_ex = [
            -0.5 * l[0] as f64 / q[0] as f64,
            -0.5 * l[1] as f64 / q[1] as f64,
            -0.5 * l[2] as f64 / q[2] as f64,
        ];
        return has_diagonal_artifact_inner(
            classifier,
            am,
            dm,
            a,
            &l,
            &q,
            a[1] - a[0],
            b[1] - b[0] + c[1] - c[0],
            d[1] - d[0],
            t_ex[0],
            t_ex[1],
        ) || has_diagonal_artifact_inner(
            classifier,
            am,
            dm,
            a,
            &l,
            &q,
            a[2] - a[1],
            b[2] - b[1] + c[2] - c[1],
            d[2] - d[1],
            t_ex[1],
            t_ex[2],
        ) || has_diagonal_artifact_inner(
            classifier,
            am,
            dm,
            a,
            &l,
            &q,
            a[0] - a[2],
            b[0] - b[2] + c[0] - c[2],
            d[0] - d[2],
            t_ex[2],
            t_ex[0],
        );
    }
    false
}

// --- the corrector ---------------------------------------------------------

struct MsdfErrorCorrection<'a> {
    stencil: Vec<u8>,
    width: usize,
    height: usize,
    transformation: &'a SdfTransformation,
    min_deviation_ratio: f64,
    min_improve_ratio: f64,
}

impl<'a> MsdfErrorCorrection<'a> {
    fn new(
        width: usize,
        height: usize,
        transformation: &'a SdfTransformation,
        cfg: &MsdfGeneratorConfig,
    ) -> Self {
        MsdfErrorCorrection {
            stencil: vec![0u8; width * height],
            width,
            height,
            transformation,
            min_deviation_ratio: cfg.error_correction.min_deviation_ratio,
            min_improve_ratio: cfg.error_correction.min_improve_ratio,
        }
    }

    #[inline]
    fn st(&mut self, x: usize, y: usize) -> &mut u8 {
        &mut self.stencil[y * self.width + x]
    }

    fn protect_all(&mut self) {
        for s in &mut self.stencil {
            *s |= FLAG_PROTECTED;
        }
    }

    fn protect_corners(&mut self, shape: &Shape) {
        for contour in &shape.contours {
            if contour.segments.is_empty() {
                continue;
            }
            let n = contour.segments.len();
            let mut prev_color = contour.colors[n - 1];
            for i in 0..n {
                let color = contour.colors[i];
                let common = (prev_color.bits() & color.bits()) as i32;
                if common & (common - 1) == 0 {
                    let p = self
                        .transformation
                        .projection
                        .project(ShapePoint::from_raw(contour.segments[i].point(0.0)))
                        .raw();
                    let l = (p.x - 0.5).floor() as isize;
                    let b = (p.y - 0.5).floor() as isize;
                    let r = l + 1;
                    let t = b + 1;
                    let w = self.width as isize;
                    let h = self.height as isize;
                    if l < w && b < h && r >= 0 && t >= 0 {
                        let mut mark = |x: isize, y: isize| {
                            if x >= 0 && y >= 0 && x < w && y < h {
                                self.stencil[y as usize * self.width + x as usize] |=
                                    FLAG_PROTECTED;
                            }
                        };
                        mark(l, b);
                        mark(r, b);
                        mark(l, t);
                        mark(r, t);
                    }
                }
                prev_color = color;
            }
        }
    }

    fn protect_edges<const N: usize>(&mut self, sdf: &Bitmap<f32, N>) {
        let dm = self.transformation.distance_mapping;
        let proj = self.transformation.projection;
        let len = |v: Vector2| proj.unproject_vector(v).length();
        // Horizontal
        let radius =
            (PROTECTION_RADIUS_TOLERANCE * len(Vector2::new(dm.map_delta(1.0), 0.0))) as f32;
        for y in 0..self.height {
            for x in 0..self.width - 1 {
                let left = sdf.pixel(x, y);
                let right = sdf.pixel(x + 1, y);
                let lm = medf(left);
                let rm = medf(right);
                if (lm - 0.5).abs() + (rm - 0.5).abs() < radius {
                    let mask = edge_between_texels(left, right);
                    let pl = protect_extreme(left, lm, mask);
                    let pr = protect_extreme(right, rm, mask);
                    if pl {
                        *self.st(x, y) |= FLAG_PROTECTED;
                    }
                    if pr {
                        *self.st(x + 1, y) |= FLAG_PROTECTED;
                    }
                }
            }
        }
        // Vertical
        let radius =
            (PROTECTION_RADIUS_TOLERANCE * len(Vector2::new(0.0, dm.map_delta(1.0)))) as f32;
        for y in 0..self.height - 1 {
            for x in 0..self.width {
                let bottom = sdf.pixel(x, y);
                let top = sdf.pixel(x, y + 1);
                let bm = medf(bottom);
                let tm = medf(top);
                if (bm - 0.5).abs() + (tm - 0.5).abs() < radius {
                    let mask = edge_between_texels(bottom, top);
                    let pb = protect_extreme(bottom, bm, mask);
                    let pt = protect_extreme(top, tm, mask);
                    if pb {
                        *self.st(x, y) |= FLAG_PROTECTED;
                    }
                    if pt {
                        *self.st(x, y + 1) |= FLAG_PROTECTED;
                    }
                }
            }
        }
        // Diagonal
        let radius = (PROTECTION_RADIUS_TOLERANCE
            * len(Vector2::new(dm.map_delta(1.0), dm.map_delta(1.0)))) as f32;
        for y in 0..self.height - 1 {
            for x in 0..self.width - 1 {
                let lb = sdf.pixel(x, y);
                let rb = sdf.pixel(x + 1, y);
                let lt = sdf.pixel(x, y + 1);
                let rt = sdf.pixel(x + 1, y + 1);
                let mlb = medf(lb);
                let mrb = medf(rb);
                let mlt = medf(lt);
                let mrt = medf(rt);
                if (mlb - 0.5).abs() + (mrt - 0.5).abs() < radius {
                    let mask = edge_between_texels(lb, rt);
                    let p0 = protect_extreme(lb, mlb, mask);
                    let p1 = protect_extreme(rt, mrt, mask);
                    if p0 {
                        *self.st(x, y) |= FLAG_PROTECTED;
                    }
                    if p1 {
                        *self.st(x + 1, y + 1) |= FLAG_PROTECTED;
                    }
                }
                if (mrb - 0.5).abs() + (mlt - 0.5).abs() < radius {
                    let mask = edge_between_texels(rb, lt);
                    let p0 = protect_extreme(rb, mrb, mask);
                    let p1 = protect_extreme(lt, mlt, mask);
                    if p0 {
                        *self.st(x + 1, y) |= FLAG_PROTECTED;
                    }
                    if p1 {
                        *self.st(x, y + 1) |= FLAG_PROTECTED;
                    }
                }
            }
        }
    }

    fn spans(&self) -> (f64, f64, f64) {
        let dm = self.transformation.distance_mapping;
        let proj = self.transformation.projection;
        let r = self.min_deviation_ratio;
        let len = |v: Vector2| proj.unproject_vector(v).length();
        (
            r * len(Vector2::new(dm.map_delta(1.0), 0.0)),
            r * len(Vector2::new(0.0, dm.map_delta(1.0))),
            r * len(Vector2::new(dm.map_delta(1.0), dm.map_delta(1.0))),
        )
    }

    /// SDF-only artifact detection. Port of `findErrors` (no shape).
    fn find_errors<const N: usize>(&mut self, sdf: &Bitmap<f32, N>) {
        let (h_span, v_span, d_span) = self.spans();
        let w = self.width;
        let h = self.height;
        for y in 0..h {
            for x in 0..w {
                let c = sdf.pixel(x, y);
                let cm = medf(c);
                let protected_flag = self.stencil[y * w + x] & FLAG_PROTECTED != 0;
                let bc = |span: f64| BaseClassifier {
                    span,
                    protected_flag,
                };
                let mut artifact = false;
                if x > 0 {
                    artifact |= has_linear_artifact(&bc(h_span), cm, c, sdf.pixel(x - 1, y));
                }
                if !artifact && y > 0 {
                    artifact |= has_linear_artifact(&bc(v_span), cm, c, sdf.pixel(x, y - 1));
                }
                if !artifact && x < w - 1 {
                    artifact |= has_linear_artifact(&bc(h_span), cm, c, sdf.pixel(x + 1, y));
                }
                if !artifact && y < h - 1 {
                    artifact |= has_linear_artifact(&bc(v_span), cm, c, sdf.pixel(x, y + 1));
                }
                if !artifact && x > 0 && y > 0 {
                    artifact |= has_diagonal_artifact(
                        &bc(d_span),
                        cm,
                        c,
                        sdf.pixel(x - 1, y),
                        sdf.pixel(x, y - 1),
                        sdf.pixel(x - 1, y - 1),
                    );
                }
                if !artifact && x < w - 1 && y > 0 {
                    artifact |= has_diagonal_artifact(
                        &bc(d_span),
                        cm,
                        c,
                        sdf.pixel(x + 1, y),
                        sdf.pixel(x, y - 1),
                        sdf.pixel(x + 1, y - 1),
                    );
                }
                if !artifact && x > 0 && y < h - 1 {
                    artifact |= has_diagonal_artifact(
                        &bc(d_span),
                        cm,
                        c,
                        sdf.pixel(x - 1, y),
                        sdf.pixel(x, y + 1),
                        sdf.pixel(x - 1, y + 1),
                    );
                }
                if !artifact && x < w - 1 && y < h - 1 {
                    artifact |= has_diagonal_artifact(
                        &bc(d_span),
                        cm,
                        c,
                        sdf.pixel(x + 1, y),
                        sdf.pixel(x, y + 1),
                        sdf.pixel(x + 1, y + 1),
                    );
                }
                if artifact {
                    self.stencil[y * w + x] |= FLAG_ERROR;
                }
            }
        }
    }

    /// Exact-distance artifact detection. Port of `findErrors<ContourCombiner>`.
    fn find_errors_with_shape<C, const N: usize>(&mut self, sdf: &Bitmap<f32, N>, shape: &Shape)
    where
        C: ContourCombiner<Selector = PerpendicularDistanceSelector>,
    {
        let (h_span, v_span, d_span) = self.spans();
        let w = self.width;
        let h = self.height;
        let texel_size = self
            .transformation
            .projection
            .unproject_vector(Vector2::splat(1.0));
        let checker = ShapeChecker::<C, N> {
            finder: RefCell::new(ShapeDistanceFinder::<C>::new(shape)),
            sdf,
            transformation: self.transformation,
            min_improve_ratio: self.min_improve_ratio,
            texel_size,
        };

        for y in 0..h {
            for x in 0..w {
                if self.stencil[y * w + x] & FLAG_ERROR != 0 {
                    continue;
                }
                let c = sdf.pixel(x, y);
                let protected_flag = self.stencil[y * w + x] & FLAG_PROTECTED != 0;
                let texel = Texel {
                    shape_coord: self
                        .transformation
                        .projection
                        .unproject(PixelPoint::new(x as f64 + 0.5, y as f64 + 0.5))
                        .raw(),
                    sdf_coord: Vector2::new(x as f64 + 0.5, y as f64 + 0.5),
                    msd: c,
                    protected_flag,
                };
                let cm = medf(c);
                macro_rules! cls {
                    ($dir:expr, $span:expr) => {
                        make_classifier(&checker, &texel, protected_flag, $dir, $span)
                    };
                }
                let mut artifact = false;
                if x > 0 {
                    artifact |= has_linear_artifact(
                        &cls!(Vector2::new(-1.0, 0.0), h_span),
                        cm,
                        c,
                        sdf.pixel(x - 1, y),
                    );
                }
                if !artifact && y > 0 {
                    artifact |= has_linear_artifact(
                        &cls!(Vector2::new(0.0, -1.0), v_span),
                        cm,
                        c,
                        sdf.pixel(x, y - 1),
                    );
                }
                if !artifact && x < w - 1 {
                    artifact |= has_linear_artifact(
                        &cls!(Vector2::new(1.0, 0.0), h_span),
                        cm,
                        c,
                        sdf.pixel(x + 1, y),
                    );
                }
                if !artifact && y < h - 1 {
                    artifact |= has_linear_artifact(
                        &cls!(Vector2::new(0.0, 1.0), v_span),
                        cm,
                        c,
                        sdf.pixel(x, y + 1),
                    );
                }
                if !artifact && x > 0 && y > 0 {
                    artifact |= has_diagonal_artifact(
                        &cls!(Vector2::new(-1.0, -1.0), d_span),
                        cm,
                        c,
                        sdf.pixel(x - 1, y),
                        sdf.pixel(x, y - 1),
                        sdf.pixel(x - 1, y - 1),
                    );
                }
                if !artifact && x < w - 1 && y > 0 {
                    artifact |= has_diagonal_artifact(
                        &cls!(Vector2::new(1.0, -1.0), d_span),
                        cm,
                        c,
                        sdf.pixel(x + 1, y),
                        sdf.pixel(x, y - 1),
                        sdf.pixel(x + 1, y - 1),
                    );
                }
                if !artifact && x > 0 && y < h - 1 {
                    artifact |= has_diagonal_artifact(
                        &cls!(Vector2::new(-1.0, 1.0), d_span),
                        cm,
                        c,
                        sdf.pixel(x - 1, y),
                        sdf.pixel(x, y + 1),
                        sdf.pixel(x - 1, y + 1),
                    );
                }
                if !artifact && x < w - 1 && y < h - 1 {
                    artifact |= has_diagonal_artifact(
                        &cls!(Vector2::new(1.0, 1.0), d_span),
                        cm,
                        c,
                        sdf.pixel(x + 1, y),
                        sdf.pixel(x, y + 1),
                        sdf.pixel(x + 1, y + 1),
                    );
                }
                if artifact {
                    self.stencil[y * w + x] |= FLAG_ERROR;
                }
            }
        }
    }

    /// Collapse error texels to single-channel. Port of `apply`.
    fn apply<const N: usize>(&self, sdf: &mut Bitmap<f32, N>) {
        for y in 0..self.height {
            for x in 0..self.width {
                if self.stencil[y * self.width + x] & FLAG_ERROR != 0 {
                    let px = sdf.pixel_mut(x, y);
                    let m = medf(px);
                    px[0] = m;
                    px[1] = m;
                    px[2] = m;
                }
            }
        }
    }
}

fn edge_between_texels_channel(a: &[f32], b: &[f32], channel: usize) -> bool {
    let t = (a[channel] as f64 - 0.5) / (a[channel] - b[channel]) as f64;
    if t > 0.0 && t < 1.0 {
        let c = [
            mixf(a[0], b[0], t),
            mixf(a[1], b[1], t),
            mixf(a[2], b[2], t),
        ];
        return medf3(c[0], c[1], c[2]) == c[channel];
    }
    false
}

fn edge_between_texels(a: &[f32], b: &[f32]) -> i32 {
    (edge_between_texels_channel(a, b, 0) as i32)
        + 2 * (edge_between_texels_channel(a, b, 1) as i32)
        + 4 * (edge_between_texels_channel(a, b, 2) as i32)
}

/// Whether texel `msd` (median `m`) has a non-median channel present in `mask`.
fn protect_extreme(msd: &[f32], m: f32, mask: i32) -> bool {
    (mask & 1 != 0 && msd[0] != m)
        || (mask & 2 != 0 && msd[1] != m)
        || (mask & 4 != 0 && msd[2] != m)
}

// --- exact-distance classifier ---------------------------------------------

struct Texel<'a> {
    shape_coord: Vector2,
    sdf_coord: Vector2,
    msd: &'a [f32],
    #[allow(dead_code)]
    protected_flag: bool,
}

struct ShapeChecker<'a, C: ContourCombiner, const N: usize> {
    finder: RefCell<ShapeDistanceFinder<'a, C>>,
    sdf: &'a Bitmap<f32, N>,
    transformation: &'a SdfTransformation,
    min_improve_ratio: f64,
    texel_size: Vector2,
}

struct ShapeArtifactClassifier<'c, 't, C: ContourCombiner, const N: usize> {
    checker: &'c ShapeChecker<'c, C, N>,
    texel: &'t Texel<'t>,
    direction: Vector2,
    span: f64,
    protected_flag: bool,
}

/// Free constructor (avoids closure variance issues at the call sites).
fn make_classifier<'c, 't, C, const N: usize>(
    checker: &'c ShapeChecker<'c, C, N>,
    texel: &'t Texel<'t>,
    protected_flag: bool,
    direction: Vector2,
    span: f64,
) -> ShapeArtifactClassifier<'c, 't, C, N>
where
    C: ContourCombiner<Selector = PerpendicularDistanceSelector>,
{
    ShapeArtifactClassifier {
        checker,
        texel,
        direction,
        span,
        protected_flag,
    }
}

impl<'c, 't, C, const N: usize> Classifier for ShapeArtifactClassifier<'c, 't, C, N>
where
    C: ContourCombiner<Selector = PerpendicularDistanceSelector>,
{
    fn span(&self) -> f64 {
        self.span
    }
    fn protected_flag(&self) -> bool {
        self.protected_flag
    }
    fn evaluate(&self, t: f64, _m: f32, flags: i32) -> bool {
        if flags & CLASSIFIER_FLAG_CANDIDATE == 0 {
            return false;
        }
        if flags & CLASSIFIER_FLAG_ARTIFACT != 0 {
            return true;
        }
        let t_vector = t * self.direction;
        let mut old_msd = [0.0f32; 4];
        let sdf_coord = self.texel.sdf_coord + t_vector;
        interpolate(&mut old_msd[..N], self.checker.sdf, sdf_coord);
        let a_weight = (1.0 - t_vector.x.abs()) * (1.0 - t_vector.y.abs());
        let a_psd = medf(self.texel.msd);
        let new_msd = [
            (old_msd[0] as f64 + a_weight * (a_psd - self.texel.msd[0]) as f64) as f32,
            (old_msd[1] as f64 + a_weight * (a_psd - self.texel.msd[1]) as f64) as f32,
            (old_msd[2] as f64 + a_weight * (a_psd - self.texel.msd[2]) as f64) as f32,
        ];
        let old_psd = medf3(old_msd[0], old_msd[1], old_msd[2]);
        let new_psd = medf3(new_msd[0], new_msd[1], new_msd[2]);
        let dm = self.checker.transformation.distance_mapping;
        let sample = self.texel.shape_coord + t_vector * self.checker.texel_size;
        let dist = self.checker.finder.borrow_mut().distance(sample);
        let ref_psd = dm.map(dist) as f32;
        self.checker.min_improve_ratio * ((new_psd - ref_psd).abs() as f64)
            < (old_psd - ref_psd).abs() as f64
    }
}

/// Top-level entry point. Port of `msdfErrorCorrection` / `msdfErrorCorrectionInner`.
pub fn msdf_error_correction<const N: usize>(
    output: &mut Bitmap<f32, N>,
    shape: &Shape,
    transformation: &SdfTransformation,
    config: &MsdfGeneratorConfig,
) {
    if config.error_correction.mode == ErrorCorrectionMode::Disabled {
        return;
    }
    let mut ec = MsdfErrorCorrection::new(output.width, output.height, transformation, config);

    match config.error_correction.mode {
        ErrorCorrectionMode::Disabled | ErrorCorrectionMode::Indiscriminate => {}
        ErrorCorrectionMode::EdgePriority => {
            ec.protect_corners(shape);
            ec.protect_edges(output);
        }
        ErrorCorrectionMode::EdgeOnly => {
            ec.protect_all();
        }
    }

    let dcm = config.error_correction.distance_check_mode;
    if dcm == DistanceCheckMode::DoNotCheckDistance
        || (dcm == DistanceCheckMode::CheckDistanceAtEdge
            && config.error_correction.mode != ErrorCorrectionMode::EdgeOnly)
    {
        ec.find_errors(output);
        if dcm == DistanceCheckMode::CheckDistanceAtEdge {
            ec.protect_all();
        }
    }
    if dcm == DistanceCheckMode::AlwaysCheckDistance
        || dcm == DistanceCheckMode::CheckDistanceAtEdge
    {
        if config.overlap_support {
            ec.find_errors_with_shape::<OverlappingContourCombiner<PerpendicularDistanceSelector>, N>(
                output, shape,
            );
        } else {
            ec.find_errors_with_shape::<SimpleContourCombiner<PerpendicularDistanceSelector>, N>(
                output, shape,
            );
        }
    }
    ec.apply(output);
}
