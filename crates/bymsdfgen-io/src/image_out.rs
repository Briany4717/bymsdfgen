//! Distance-field output encoders. Port of `ext/save-png.cpp`, `core/save-bmp`
//! and the raw/text writers in `main.cpp`'s `writeOutput`.
//!
//! Pixels are stored bottom-up (Y up) in [`Bitmap`]; file formats that are
//! top-down (PNG) are flipped on write so output is upright.

use std::fs::File;
use std::io::{self, BufWriter, Write};

use bymsdfgen_core::bitmap::Bitmap;

/// Port of `pixelFloatToByte`: `byte(~int(255.5 - 255*clamp(x)))`.
#[inline]
pub fn pixel_float_to_byte(x: f32) -> u8 {
    let c = x.clamp(0.0, 1.0);
    (!((255.5 - 255.0 * c) as i32)) as u8
}

/// Output image format selector. Mirrors the CLI's `-format` options.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    Png,
    Bmp,
    Rgba,
    Fl32,
    Text,
    TextFloat,
    Bin,
    BinFloat,
    BinFloatBe,
}

impl ImageFormat {
    /// Deduce the format from a file extension.
    pub fn from_extension(ext: &str) -> Option<ImageFormat> {
        Some(match ext.to_ascii_lowercase().as_str() {
            "png" => ImageFormat::Png,
            "bmp" => ImageFormat::Bmp,
            "rgba" => ImageFormat::Rgba,
            "fl32" => ImageFormat::Fl32,
            "txt" | "text" => ImageFormat::Text,
            "bin" => ImageFormat::Bin,
            _ => return None,
        })
    }
}

/// Save a bitmap to PNG (top-down). Channels: 1=gray, 3=RGB, 4=RGBA.
pub fn save_png<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let color = match N {
        1 => png::ColorType::Grayscale,
        3 => png::ColorType::Rgb,
        4 => png::ColorType::Rgba,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "unsupported channel count for PNG",
            ));
        }
    };
    let file = File::create(path)?;
    let w = BufWriter::new(file);
    let mut encoder = png::Encoder::new(w, bitmap.width as u32, bitmap.height as u32);
    encoder.set_color(color);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder
        .write_header()
        .map_err(|e| io::Error::other(e.to_string()))?;

    let mut data = Vec::with_capacity(bitmap.width * bitmap.height * N);
    for y in (0..bitmap.height).rev() {
        for x in 0..bitmap.width {
            for &v in bitmap.pixel(x, y) {
                data.push(pixel_float_to_byte(v));
            }
        }
    }
    writer
        .write_image_data(&data)
        .map_err(|e| io::Error::other(e.to_string()))
}

/// Save an uncompressed BMP (bottom-up, which is BMP's native order).
pub fn save_bmp<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let (w, h) = (bitmap.width, bitmap.height);
    let bpp = match N {
        1 => 8u16,
        3 => 24,
        4 => 32,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "unsupported channel count for BMP",
            ));
        }
    };
    let palette_size = if N == 1 { 256 * 4 } else { 0 };
    let row_bytes = (N * w + 3) & !3; // rows padded to 4 bytes
    let pixel_data = row_bytes * h;
    let header_size = 14 + 40 + palette_size;
    let file_size = header_size + pixel_data;

    let mut out = BufWriter::new(File::create(path)?);
    // BITMAPFILEHEADER
    out.write_all(b"BM")?;
    out.write_all(&(file_size as u32).to_le_bytes())?;
    out.write_all(&0u32.to_le_bytes())?;
    out.write_all(&(header_size as u32).to_le_bytes())?;
    // BITMAPINFOHEADER
    out.write_all(&40u32.to_le_bytes())?;
    out.write_all(&(w as i32).to_le_bytes())?;
    out.write_all(&(h as i32).to_le_bytes())?;
    out.write_all(&1u16.to_le_bytes())?;
    out.write_all(&bpp.to_le_bytes())?;
    out.write_all(&0u32.to_le_bytes())?; // BI_RGB
    out.write_all(&(pixel_data as u32).to_le_bytes())?;
    out.write_all(&2835i32.to_le_bytes())?; // 72 DPI
    out.write_all(&2835i32.to_le_bytes())?;
    out.write_all(&0u32.to_le_bytes())?;
    out.write_all(&0u32.to_le_bytes())?;
    // Grayscale palette
    if N == 1 {
        for i in 0..256u32 {
            let b = i as u8;
            out.write_all(&[b, b, b, 0])?;
        }
    }
    // Pixel rows, bottom-up (matches BMP), BGR(A) order.
    let pad = row_bytes - N * w;
    for y in 0..h {
        for x in 0..w {
            let px = bitmap.pixel(x, y);
            match N {
                1 => out.write_all(&[pixel_float_to_byte(px[0])])?,
                3 => out.write_all(&[
                    pixel_float_to_byte(px[2]),
                    pixel_float_to_byte(px[1]),
                    pixel_float_to_byte(px[0]),
                ])?,
                4 => out.write_all(&[
                    pixel_float_to_byte(px[2]),
                    pixel_float_to_byte(px[1]),
                    pixel_float_to_byte(px[0]),
                    pixel_float_to_byte(px[3]),
                ])?,
                _ => unreachable!(),
            }
        }
        for _ in 0..pad {
            out.write_all(&[0])?;
        }
    }
    out.flush()
}

/// Raw 8-bit RGBA bytes (expanded from any channel count), bottom-up.
pub fn save_rgba<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for y in 0..bitmap.height {
        for x in 0..bitmap.width {
            let px = bitmap.pixel(x, y);
            let rgba = match N {
                1 => {
                    let v = pixel_float_to_byte(px[0]);
                    [v, v, v, 255]
                }
                3 => [
                    pixel_float_to_byte(px[0]),
                    pixel_float_to_byte(px[1]),
                    pixel_float_to_byte(px[2]),
                    255,
                ],
                _ => [
                    pixel_float_to_byte(px[0]),
                    pixel_float_to_byte(px[1]),
                    pixel_float_to_byte(px[2]),
                    pixel_float_to_byte(px[3]),
                ],
            };
            out.write_all(&rgba)?;
        }
    }
    out.flush()
}

/// Raw little-endian f32 channel data, bottom-up.
pub fn save_fl32<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for &v in bitmap.data() {
        out.write_all(&v.to_le_bytes())?;
    }
    out.flush()
}

/// Hexadecimal text dump of 8-bit values.
pub fn write_text<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for y in (0..bitmap.height).rev() {
        for x in 0..bitmap.width {
            for (i, &v) in bitmap.pixel(x, y).iter().enumerate() {
                if i > 0 || x > 0 {
                    write!(out, " ")?;
                }
                write!(out, "{:02x}", pixel_float_to_byte(v))?;
            }
        }
        writeln!(out)?;
    }
    out.flush()
}

/// Floating-point text dump.
pub fn write_text_float<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for y in (0..bitmap.height).rev() {
        for x in 0..bitmap.width {
            for (i, &v) in bitmap.pixel(x, y).iter().enumerate() {
                if i > 0 || x > 0 {
                    write!(out, " ")?;
                }
                write!(out, "{v}")?;
            }
        }
        writeln!(out)?;
    }
    out.flush()
}

/// Raw 8-bit byte stream (bottom-up).
pub fn write_bin<const N: usize>(bitmap: &Bitmap<f32, N>, path: &str) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for &v in bitmap.data() {
        out.write_all(&[pixel_float_to_byte(v)])?;
    }
    out.flush()
}

/// Raw f32 stream, configurable endianness.
pub fn write_bin_float<const N: usize>(
    bitmap: &Bitmap<f32, N>,
    path: &str,
    big_endian: bool,
) -> io::Result<()> {
    let mut out = BufWriter::new(File::create(path)?);
    for &v in bitmap.data() {
        let bytes = if big_endian {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        };
        out.write_all(&bytes)?;
    }
    out.flush()
}

/// Dispatch to the right encoder by [`ImageFormat`].
pub fn save<const N: usize>(
    bitmap: &Bitmap<f32, N>,
    path: &str,
    format: ImageFormat,
) -> io::Result<()> {
    match format {
        ImageFormat::Png => save_png(bitmap, path),
        ImageFormat::Bmp => save_bmp(bitmap, path),
        ImageFormat::Rgba => save_rgba(bitmap, path),
        ImageFormat::Fl32 => save_fl32(bitmap, path),
        ImageFormat::Text => write_text(bitmap, path),
        ImageFormat::TextFloat => write_text_float(bitmap, path),
        ImageFormat::Bin => write_bin(bitmap, path),
        ImageFormat::BinFloat => write_bin_float(bitmap, path, false),
        ImageFormat::BinFloatBe => write_bin_float(bitmap, path, true),
    }
}
