//! `bymsdfgen` — standalone CLI for the pure-Rust msdf generator and pure-Rust
//! alternative to msdfgen. Clean clap-based replacement for the original's
//! hand-rolled argument parser in `main.cpp`. No global state: configuration flows
//! through the typed [`Cli`] struct.

use std::fs;
use std::io::Read;

use anyhow::{Context, Result, anyhow, bail};
use clap::{Parser, ValueEnum};

use bymsdfgen_core::bitmap::Bitmap;
use bymsdfgen_core::coloring::{
    edge_coloring_by_distance, edge_coloring_ink_trap, edge_coloring_simple,
};
use bymsdfgen_core::correction::msdf_error_correction;
use bymsdfgen_core::generator::config::{
    DistanceCheckMode, ErrorCorrectionConfig, ErrorCorrectionMode,
};
use bymsdfgen_core::generator::{
    DistanceMapping, GeneratorConfig, MsdfGeneratorConfig, Projection, SdfTransformation,
    generate_msdf, generate_mtsdf, generate_psdf, generate_sdf,
};
use bymsdfgen_core::geometry::Shape;
use bymsdfgen_core::math::{Range, Vector2};
use bymsdfgen_core::raster::{
    FillRule, distance_sign_correction_1, distance_sign_correction_multi,
};

use bymsdfgen_io::font::{Font, FontCoordinateScaling};
use bymsdfgen_io::image_out::{self, ImageFormat};
use bymsdfgen_io::{parse_shape, write_shape};

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    Sdf,
    Psdf,
    Msdf,
    Mtsdf,
    Metrics,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum Coloring {
    Simple,
    Inktrap,
    Distance,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum Fill {
    Nonzero,
    Evenodd,
    Positive,
    Negative,
}

/// Multi-channel signed distance field generator (pure-Rust msdfgen twin).
#[derive(Parser, Debug)]
#[command(name = "bymsdfgen", version, about)]
struct Cli {
    /// Generation mode.
    #[arg(value_enum, default_value_t = Mode::Msdf)]
    mode: Mode,

    /// Load a glyph from a font file (use with --char).
    #[arg(long, value_name = "FILE")]
    font: Option<String>,
    /// Character code: decimal (65), hex (0x41), 'A', or glyph index g34.
    #[arg(long, value_name = "CODE")]
    char: Option<String>,
    /// Load the last path of an SVG file.
    #[arg(long, value_name = "FILE")]
    svg: Option<String>,
    /// Load a text shape description from a file.
    #[arg(long, value_name = "FILE")]
    shapedesc: Option<String>,
    /// Inline text shape description.
    #[arg(long, value_name = "DEF")]
    defineshape: Option<String>,
    /// Read a shape description from standard input.
    #[arg(long)]
    stdin: bool,

    /// Output file (format deduced from extension unless --format given).
    #[arg(short, long, value_name = "FILE")]
    output: Option<String>,
    /// Explicit output format.
    #[arg(long, value_name = "FMT")]
    format: Option<String>,

    /// Output dimensions in pixels: WIDTH HEIGHT.
    #[arg(long, num_args = 2, value_names = ["W", "H"], default_values_t = [64u32, 64u32])]
    dimensions: Vec<u32>,
    /// Distance range in shape units.
    #[arg(long)]
    range: Option<f64>,
    /// Distance range in output pixels (default 2).
    #[arg(long)]
    pxrange: Option<f64>,
    /// Scale from shape units to pixels.
    #[arg(long)]
    scale: Option<f64>,
    /// Translation in shape units: X Y.
    #[arg(long, num_args = 2, value_names = ["X", "Y"])]
    translate: Option<Vec<f64>>,
    /// Auto-fit the shape into the output (preview only).
    #[arg(long)]
    autoframe: bool,

    /// Maximum corner angle; append 'D' for degrees (e.g. 171.9D).
    #[arg(long, default_value = "3.0")]
    angle: String,
    /// Edge coloring strategy (msdf/mtsdf only).
    #[arg(long, value_enum, default_value_t = Coloring::Simple)]
    edgecolors: Coloring,
    /// Coloring PRNG seed.
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// Skip edge coloring (leave all edges white).
    #[arg(long)]
    no_coloring: bool,

    /// Disable the overlapping-contour algorithm (faster, no overlap support).
    #[arg(long)]
    no_overlap: bool,
    /// Fill rule used for sign correction.
    #[arg(long, value_enum, default_value_t = Fill::Nonzero)]
    fillrule: Fill,
    /// Disable the scanline sign-correction pass.
    #[arg(long)]
    no_scanline: bool,

    /// Print shape metrics to stderr.
    #[arg(long)]
    printmetrics: bool,
    /// Export the (coloured) shape description to a file.
    #[arg(long, value_name = "FILE")]
    exportshape: Option<String>,

    /// Worker threads (0 = rayon default).
    #[arg(long, default_value_t = 0)]
    threads: usize,
}

fn parse_char_code(s: &str) -> Result<CharCode> {
    let s = s.trim();
    if let Some(rest) = s.strip_prefix('g') {
        let idx = parse_uint(rest)?;
        return Ok(CharCode::Glyph(idx as u16));
    }
    if s.len() >= 3 && s.starts_with('\'') && s.ends_with('\'') {
        let inner: Vec<char> = s[1..s.len() - 1].chars().collect();
        if inner.len() == 1 {
            return Ok(CharCode::Unicode(inner[0]));
        }
        bail!("invalid character literal: {s}");
    }
    let v = parse_uint(s)?;
    char::from_u32(v as u32)
        .map(CharCode::Unicode)
        .ok_or_else(|| anyhow!("invalid unicode value: {s}"))
}

fn parse_uint(s: &str) -> Result<u64> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Ok(u64::from_str_radix(hex, 16)?)
    } else {
        Ok(s.parse::<u64>()?)
    }
}

enum CharCode {
    Unicode(char),
    Glyph(u16),
}

fn parse_angle(s: &str) -> Result<f64> {
    let s = s.trim();
    if let Some(deg) = s.strip_suffix('D').or_else(|| s.strip_suffix('d')) {
        Ok(deg.parse::<f64>()?.to_radians())
    } else {
        Ok(s.parse::<f64>()?)
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    if cli.threads > 0 {
        let _ = rayon::ThreadPoolBuilder::new()
            .num_threads(cli.threads)
            .build_global();
    }

    run(cli)
}

fn run(cli: Cli) -> Result<()> {
    let width = cli.dimensions[0] as usize;
    let height = cli.dimensions[1] as usize;

    // --- load shape ---
    let mut glyph_advance = 0.0f64;
    let mut shape = load_shape(&cli, &mut glyph_advance)?;

    if !shape.validate() {
        bail!("the shape is invalid (contours are not properly closed)");
    }
    shape.normalize();

    let bounds = shape.get_bounds(0.0);

    // --- framing ---
    let mut scale = Vector2::splat(cli.scale.unwrap_or(1.0));
    let scale_specified = cli.scale.is_some();
    let mut translate = match &cli.translate {
        Some(v) => Vector2::new(v[0], v[1]),
        None => Vector2::ZERO,
    };

    // Range handling: pixel range by default (2px), or unit range if --range.
    let range_px = cli.pxrange.unwrap_or(2.0);
    let unit_range_mode = cli.range.is_some();
    let mut range = Range::symmetric(cli.range.unwrap_or(1.0));
    let px_range = Range::symmetric(range_px);

    if cli.autoframe {
        let (mut l, mut b, mut r, mut t) = (bounds.l, bounds.b, bounds.r, bounds.t);
        let mut frame = Vector2::new(width as f64, height as f64);
        if !scale_specified {
            if unit_range_mode {
                l += range.lower;
                b += range.lower;
                r -= range.lower;
                t -= range.lower;
            } else {
                frame = frame + Vector2::splat(2.0 * px_range.lower);
            }
        }
        if l >= r || b >= t {
            l = 0.0;
            b = 0.0;
            r = 1.0;
            t = 1.0;
        }
        if frame.x <= 0.0 || frame.y <= 0.0 {
            bail!("cannot fit the specified pixel range");
        }
        let dims = Vector2::new(r - l, t - b);
        if scale_specified {
            translate = (frame / scale - dims) * 0.5 - Vector2::new(l, b);
        } else if dims.x * frame.y < dims.y * frame.x {
            translate = Vector2::new(0.5 * (frame.x / frame.y * dims.y - dims.x) - l, -b);
            scale = Vector2::splat(frame.y / dims.y);
        } else {
            translate = Vector2::new(-l, 0.5 * (frame.y / frame.x * dims.x - dims.y) - b);
            scale = Vector2::splat(frame.x / dims.x);
        }
        if !unit_range_mode && !scale_specified {
            translate = translate - Vector2::splat(px_range.lower) / scale;
        }
    }

    if !unit_range_mode {
        range = px_range / scale.x.min(scale.y);
    }

    // --- metrics ---
    if cli.printmetrics || cli.mode == Mode::Metrics {
        eprintln!(
            "Y-axis {}",
            if shape.inverse_y_axis {
                "downward"
            } else {
                "upward"
            }
        );
        if bounds.l < bounds.r && bounds.b < bounds.t {
            eprintln!(
                "bounds = {}, {}, {}, {}",
                bounds.l, bounds.b, bounds.r, bounds.t
            );
        }
        if glyph_advance != 0.0 {
            eprintln!("advance = {glyph_advance}");
        }
        if cli.autoframe {
            if !scale_specified {
                eprintln!("scale = {}", scale.x);
            }
            eprintln!("translate = {}, {}", translate.x, translate.y);
        }
        eprintln!("range {} to {}", range.lower, range.upper);
        if cli.mode == Mode::Metrics {
            return Ok(());
        }
    }

    // --- export shape (after coloring, below) ---
    let transformation = SdfTransformation::new(
        Projection::new(scale, translate),
        DistanceMapping::from_range(range),
    );
    let projection = transformation.projection;
    let fill = match cli.fillrule {
        Fill::Nonzero => FillRule::NonZero,
        Fill::Evenodd => FillRule::Odd,
        Fill::Positive => FillRule::Positive,
        Fill::Negative => FillRule::Negative,
    };
    let scanline = !cli.no_scanline;
    let overlap = !cli.no_overlap;

    // Match msdfgen's default pipeline: when the scanline pass is active, the
    // distance field is generated WITHOUT error correction; sign correction runs;
    // then a post error-correction pass with EDGE_PRIORITY + DO_NOT_CHECK_DISTANCE.
    let sdf_zero = if range.lower != range.upper {
        (range.lower / (range.lower - range.upper)) as f32
    } else {
        0.5
    };

    let gen_cfg = GeneratorConfig {
        overlap_support: overlap,
    };
    let gen_msdf_cfg = MsdfGeneratorConfig {
        overlap_support: overlap,
        error_correction: ErrorCorrectionConfig {
            // disabled during generation when a scanline pass follows
            mode: if scanline {
                ErrorCorrectionMode::Disabled
            } else {
                ErrorCorrectionMode::EdgePriority
            },
            ..Default::default()
        },
    };
    let post_msdf_cfg = MsdfGeneratorConfig {
        overlap_support: overlap,
        error_correction: ErrorCorrectionConfig {
            mode: ErrorCorrectionMode::EdgePriority,
            distance_check_mode: DistanceCheckMode::DoNotCheckDistance,
            ..Default::default()
        },
    };

    // Output file/format resolution.
    let output_path = cli
        .output
        .clone()
        .unwrap_or_else(|| "output.png".to_string());
    let format = resolve_format(&cli, &output_path)?;

    match cli.mode {
        Mode::Sdf => {
            let mut sdf: Bitmap<f32, 1> = Bitmap::new(width, height);
            generate_sdf(&mut sdf, &shape, &transformation, &gen_cfg);
            if scanline {
                distance_sign_correction_1(&mut sdf, &shape, &projection, sdf_zero, fill);
            }
            image_out::save(&sdf, &output_path, format)?;
        }
        Mode::Psdf => {
            let mut sdf: Bitmap<f32, 1> = Bitmap::new(width, height);
            generate_psdf(&mut sdf, &shape, &transformation, &gen_cfg);
            if scanline {
                distance_sign_correction_1(&mut sdf, &shape, &projection, sdf_zero, fill);
            }
            image_out::save(&sdf, &output_path, format)?;
        }
        Mode::Msdf => {
            color_edges(&cli, &mut shape)?;
            maybe_export_shape(&cli, &shape)?;
            let mut msdf: Bitmap<f32, 3> = Bitmap::new(width, height);
            generate_msdf(&mut msdf, &shape, &transformation, &gen_msdf_cfg);
            if scanline {
                distance_sign_correction_multi(&mut msdf, &shape, &projection, sdf_zero, fill);
                msdf_error_correction(&mut msdf, &shape, &transformation, &post_msdf_cfg);
            }
            image_out::save(&msdf, &output_path, format)?;
        }
        Mode::Mtsdf => {
            color_edges(&cli, &mut shape)?;
            maybe_export_shape(&cli, &shape)?;
            let mut mtsdf: Bitmap<f32, 4> = Bitmap::new(width, height);
            generate_mtsdf(&mut mtsdf, &shape, &transformation, &gen_msdf_cfg);
            if scanline {
                distance_sign_correction_multi(&mut mtsdf, &shape, &projection, sdf_zero, fill);
                msdf_error_correction(&mut mtsdf, &shape, &transformation, &post_msdf_cfg);
            }
            image_out::save(&mtsdf, &output_path, format)?;
        }
        Mode::Metrics => unreachable!(),
    }

    eprintln!("wrote {output_path}");
    Ok(())
}

fn color_edges(cli: &Cli, shape: &mut Shape) -> Result<()> {
    if cli.no_coloring {
        return Ok(());
    }
    let angle = parse_angle(&cli.angle)?;
    match cli.edgecolors {
        Coloring::Simple => edge_coloring_simple(shape, angle, cli.seed),
        Coloring::Inktrap => edge_coloring_ink_trap(shape, angle, cli.seed),
        Coloring::Distance => edge_coloring_by_distance(shape, angle, cli.seed),
    }
    Ok(())
}

fn maybe_export_shape(cli: &Cli, shape: &Shape) -> Result<()> {
    if let Some(path) = &cli.exportshape {
        fs::write(path, write_shape(shape)).with_context(|| format!("writing {path}"))?;
    }
    Ok(())
}

fn resolve_format(cli: &Cli, output_path: &str) -> Result<ImageFormat> {
    if let Some(f) = &cli.format {
        return match f.to_ascii_lowercase().as_str() {
            "png" => Ok(ImageFormat::Png),
            "bmp" => Ok(ImageFormat::Bmp),
            "rgba" => Ok(ImageFormat::Rgba),
            "fl32" => Ok(ImageFormat::Fl32),
            "text" | "txt" => Ok(ImageFormat::Text),
            "textfloat" => Ok(ImageFormat::TextFloat),
            "bin" => Ok(ImageFormat::Bin),
            "binfloat" => Ok(ImageFormat::BinFloat),
            "binfloatbe" => Ok(ImageFormat::BinFloatBe),
            other => bail!("unknown format: {other}"),
        };
    }
    let ext = output_path.rsplit('.').next().unwrap_or("");
    ImageFormat::from_extension(ext)
        .ok_or_else(|| anyhow!("cannot deduce format from '{output_path}'; use --format"))
}

fn load_shape(cli: &Cli, glyph_advance: &mut f64) -> Result<Shape> {
    let inputs = [
        cli.font.is_some(),
        cli.svg.is_some(),
        cli.shapedesc.is_some(),
        cli.defineshape.is_some(),
        cli.stdin,
    ];
    if inputs.iter().filter(|&&x| x).count() != 1 {
        bail!("specify exactly one input: --font, --svg, --shapedesc, --defineshape or --stdin");
    }

    if let Some(font_path) = &cli.font {
        let code = cli
            .char
            .as_deref()
            .ok_or_else(|| anyhow!("--font requires --char"))?;
        let data = fs::read(font_path).with_context(|| format!("reading {font_path}"))?;
        let font = Font::from_slice(&data, 0).map_err(|e| anyhow!("parsing font: {e}"))?;
        let scaling = FontCoordinateScaling::EmNormalized;
        let glyph = match parse_char_code(code)? {
            CharCode::Unicode(c) => font
                .glyph_index(c)
                .ok_or_else(|| anyhow!("glyph for {c:?} not found"))?,
            CharCode::Glyph(i) => i,
        };
        let mut shape = Shape::new();
        if !font.load_glyph(&mut shape, glyph, scaling) {
            // Whitespace / empty outline is not an error, but warn.
            eprintln!("warning: glyph has no outline");
        }
        *glyph_advance = font.advance(glyph, scaling).unwrap_or(0.0);
        return Ok(shape);
    }

    if let Some(svg_path) = &cli.svg {
        let text = fs::read_to_string(svg_path).with_context(|| format!("reading {svg_path}"))?;
        return bymsdfgen_io::load_svg_shape(&text).map_err(|e| anyhow!("{e}"));
    }

    if let Some(path) = &cli.shapedesc {
        let text = fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
        return parse_shape(&text).map_err(|e| anyhow!("{e}"));
    }

    if let Some(def) = &cli.defineshape {
        return parse_shape(def).map_err(|e| anyhow!("{e}"));
    }

    if cli.stdin {
        let mut buf = String::new();
        std::io::stdin().read_to_string(&mut buf)?;
        return parse_shape(&buf).map_err(|e| anyhow!("{e}"));
    }

    unreachable!()
}
