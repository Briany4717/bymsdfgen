# bymsdfgen

**bymsdfgen is a pure-Rust msdf generator** — a pure-Rust alternative to msdfgen, the
[Chlumský original](https://github.com/Chlumsky/msdfgen), with no C++ toolchain and no FFI.

[![github](https://img.shields.io/badge/github-briany4717/bymsdfgen-ffa190.svg?style=for-the-badge&logo=github)](https://github.com/Briany4717/bymsdfgen)
[![crates.io](https://img.shields.io/crates/v/bymsdfgen-core.svg?style=for-the-badge)](https://crates.io/crates/bymsdfgen-core)
[![docs.rs](https://img.shields.io/docsrs/bymsdfgen-core?style=for-the-badge)](https://docs.rs/bymsdfgen-core)
[![MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)

A generator of (multi-channel) signed distance fields from font glyphs and vector shapes. It is not a
mechanical transliteration of the C++: the geometry layer is redesigned around data-oriented
principles and Rust's zero-cost abstractions to remove the original's main sources of
overhead and entire classes of bugs.

This system was created as a core dependency for byard, a high-performance  UI engine.

## Empowering Next-Gen Rust UI

Modern graphic applications and UI engines like byard demand instantaneous font rasterization, 
high frame-rates, and dynamic vector rendering at arbitrary scales. Traditional CPU-based glyph 
rasterizers fail to maintain sharpness when scaled or rotated in 3D space, and caching arbitrary 
text sizes into traditional texture atlases quickly starves GPU VRAM.

Multi-channel Signed Distance Fields (MSDF) solve this by storing distance vectors across 
three color channels, allowing simple pixel shaders to reconstruct razor-sharp vector shapes at any 
zoom level with minimal memory overhead.

However, using the original C++ `msdfgen` in a modern Rust engine introduces severe friction:

The FFI & Toolchain Nightmare: Compiling C++ code with complex dependencies (like FreeType and libpng) 
for target platforms like Android, iOS, or WebAssembly (WASM) is notoriously fragile.

The WASM Speed Bump: For web-based instances of `byard, emscripten-compiled C++ bindings 
introduce boundary overhead and bloat the payload size.

Runtime Allocation Overhead: The C++ original aggressively allocates on the heap, 
risking garbage-collection-like stuttering on UI render loops.

`bymsdfgen` solves all of this. It brings a zero-FFI, compile-anywhere, memory-optimized 
MSDF generator directly to the Rust community.
## Why a rewrite instead of a port

| msdfgen (C++) | bymsdfgen (Rust) |
|---|---|
| `std::vector<EdgeSegment*>` — pointer chasing, cache misses | `EdgeSegment` is a `Copy` **enum**; contours store segments in flat, contiguous `Vec`s (struct-of-arrays) |
| Virtual `EdgeSegment` hierarchy → vtable dispatch | `match` on an enum → direct jumps, inlining, auto-vectorization |
| Manual `new`/`delete` in `EdgeHolder` | Ownership; no manual memory management, no `clone()`/`delete` |
| Bare `double` for every coordinate space (silent space-mix bugs) | `Point<DesignSpace/EmSpace/ShapeSpace/PixelSpace>` newtypes — mixing spaces is a **compile error** (zero runtime cost via `PhantomData`) |
| OpenMP `#pragma` (race conditions still compile) | rayon row-parallelism; data-race freedom proven at compile time |
| FreeType + libpng + TinyXML2 (native deps) | `ttf-parser` (zero-copy from `&[u8]`), `png`, hand-written SVG path parser — **no FFI** |
| Hand-rolled CLI arg parsing, global state, `_legacy` API duplication | `clap`; one clean API, no globals |

Numeric type is `f64` throughout, matching the original for output parity.

## Layout

```
crates/
  bymsdfgen-core   # geometry + distance fields. No I/O deps. Feature `parallel` (rayon, default on)
  bymsdfgen-io     # font (ttf-parser), image/raw output, shape-description, SVG path import
  bymsdfgen-cli    # the `bymsdfgen` binary
```

## Build & test

```sh
cargo build --release
cargo test
```

## CLI usage

```
bymsdfgen <mode> <input> [options]
```

Modes: `sdf`, `psdf`, `msdf` (default), `mtsdf`, `metrics`.

Inputs (exactly one): `--font <ttf> --char <code>`, `--svg <file>`,
`--shapedesc <file>`, `--defineshape <def>`, `--stdin`.
Character codes: decimal (`65`), hex (`0x41`), literal (`'A'`), or glyph index (`g34`).

Common options: `-o/--output`, `--format`, `--dimensions W H`, `--range`/`--pxrange`,
`--scale`, `--translate X Y`, `--autoframe`, `--angle <a|aD>`,
`--edgecolors simple|inktrap|distance`, `--seed`, `--fillrule`, `--no-overlap`,
`--no-scanline`, `--printmetrics`, `--exportshape`, `--threads`.

Example (mirrors the original's headline example):

```sh
bymsdfgen msdf --font /path/Arial.ttf --char "'M'" -o msdf.png \
  --dimensions 32 32 --pxrange 4 --autoframe
```

## Library

```rust
use bymsdfgen_core::{Bitmap, Shape, Range};
use bymsdfgen_core::generator::{generate_msdf, Projection, DistanceMapping, SdfTransformation, MsdfGeneratorConfig};
use bymsdfgen_core::coloring::edge_coloring_simple;
use bymsdfgen_core::math::Vector2;
use bymsdfgen_io::font::{Font, FontCoordinateScaling};

let data = std::fs::read("font.ttf")?;
let font = Font::from_slice(&data, 0).unwrap();
let mut shape = Shape::new();
font.load_glyph(&mut shape, font.glyph_index('A').unwrap(), FontCoordinateScaling::EmNormalized);
shape.normalize();
edge_coloring_simple(&mut shape, 3.0, 0);

let t = SdfTransformation::new(
    Projection::new(Vector2::splat(32.0), Vector2::splat(0.125)),
    DistanceMapping::from_range(Range::symmetric(0.125)),
);
let mut msdf: Bitmap<f32, 3> = Bitmap::new(32, 32);
generate_msdf(&mut msdf, &shape, &t, &MsdfGeneratorConfig::default());
bymsdfgen_io::image_out::save_png(&msdf, "out.png")?;
```

## Parity notes

- The full MSDF error-correction pass (corner/edge protection, SDF-only and exact-distance
  artifact detection) is implemented.
- SVG import parses the last `<path>` of a file (M/L/H/V/C/S/Q/T/A/Z). Skia-based
  self-intersection resolution (`FULL_PREPROCESS`) has no pure-Rust equivalent and is not
  included; winding/sign correction via the scanline pass is.
