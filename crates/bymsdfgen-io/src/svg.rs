//! Minimal pure-Rust SVG path import. Port of the path-`d` handling in
//! `ext/import-svg.cpp` (without the optional TinyXML2/Skia dependencies).
//!
//! Only the last `<path d="...">` of a file is used, matching the original's
//! documented behaviour. Supports M/L/H/V/C/S/Q/T/A/Z (absolute and relative);
//! elliptical arcs are approximated by cubic Béziers.

use bymsdfgen_core::geometry::{Contour, EdgeSegment, Shape};
use bymsdfgen_core::math::Vector2;
use std::f64::consts::PI;

const ARC_SEGMENTS_PER_PI: f64 = 2.0;

#[derive(Debug)]
pub struct SvgError(pub String);
impl std::fmt::Display for SvgError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SVG import error: {}", self.0)
    }
}
impl std::error::Error for SvgError {}

/// Load a shape from an SVG file's last path element.
pub fn load_svg_shape(svg_text: &str) -> Result<Shape, SvgError> {
    let d = extract_last_path_d(svg_text)
        .ok_or_else(|| SvgError("no <path d=\"...\"> found".into()))?;
    let mut shape = build_shape_from_svg_path(&d)?;
    shape.inverse_y_axis = true; // SVG uses a downward Y axis
    Ok(shape)
}

/// Extract the `d` attribute of the last `<path>` element (naive scan).
fn extract_last_path_d(svg: &str) -> Option<String> {
    let mut search_from = 0;
    let mut last: Option<String> = None;
    while let Some(rel) = svg[search_from..].find("<path") {
        let tag_start = search_from + rel;
        let tag_end = svg[tag_start..].find('>').map(|o| tag_start + o)?;
        let tag = &svg[tag_start..tag_end];
        if let Some(dpos) = tag.find("d=") {
            let after = &tag[dpos + 2..];
            let quote = after.chars().next();
            if let Some(q) = quote {
                if q == '"' || q == '\'' {
                    if let Some(endq) = after[1..].find(q) {
                        last = Some(after[1..1 + endq].to_string());
                    }
                }
            }
        }
        search_from = tag_end + 1;
    }
    last
}

/// Lightweight tokenizer over SVG path number/flag streams.
struct PathReader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> PathReader<'a> {
    fn new(s: &'a str) -> Self {
        PathReader {
            bytes: s.as_bytes(),
            pos: 0,
        }
    }

    fn skip_sep(&mut self) {
        while self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            if c == b' ' || c == b',' || c == b'\t' || c == b'\n' || c == b'\r' {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn peek_command(&mut self) -> Option<u8> {
        self.skip_sep();
        if self.pos < self.bytes.len() && self.bytes[self.pos].is_ascii_alphabetic() {
            Some(self.bytes[self.pos])
        } else {
            None
        }
    }

    fn next_command(&mut self) -> Option<u8> {
        let c = self.peek_command()?;
        self.pos += 1;
        Some(c)
    }

    fn more_numbers(&mut self) -> bool {
        self.skip_sep();
        self.pos < self.bytes.len() && !self.bytes[self.pos].is_ascii_alphabetic()
    }

    fn number(&mut self) -> Result<f64, SvgError> {
        self.skip_sep();
        let start = self.pos;
        let mut seen_dot = false;
        let mut seen_e = false;
        if self.pos < self.bytes.len()
            && (self.bytes[self.pos] == b'+' || self.bytes[self.pos] == b'-')
        {
            self.pos += 1;
        }
        while self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            match c {
                b'0'..=b'9' => self.pos += 1,
                b'.' if !seen_dot && !seen_e => {
                    seen_dot = true;
                    self.pos += 1;
                }
                b'e' | b'E' if !seen_e => {
                    seen_e = true;
                    self.pos += 1;
                    if self.pos < self.bytes.len()
                        && (self.bytes[self.pos] == b'+' || self.bytes[self.pos] == b'-')
                    {
                        self.pos += 1;
                    }
                }
                _ => break,
            }
        }
        let s = std::str::from_utf8(&self.bytes[start..self.pos]).unwrap_or("");
        s.parse::<f64>()
            .map_err(|_| SvgError(format!("bad number '{s}'")))
    }

    fn flag(&mut self) -> Result<bool, SvgError> {
        self.skip_sep();
        if self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            if c == b'0' || c == b'1' {
                self.pos += 1;
                return Ok(c == b'1');
            }
        }
        Err(SvgError("expected arc flag (0/1)".into()))
    }

    fn coord(&mut self) -> Result<Vector2, SvgError> {
        let x = self.number()?;
        let y = self.number()?;
        Ok(Vector2::new(x, y))
    }
}

/// Build a [`Shape`] from an SVG path data string.
pub fn build_shape_from_svg_path(d: &str) -> Result<Shape, SvgError> {
    let mut shape = Shape::new();
    let mut reader = PathReader::new(d);

    let mut contour: Option<Contour> = None;
    let mut start = Vector2::ZERO;
    let mut cur = Vector2::ZERO;
    let mut prev_control: Option<Vector2> = None;
    let mut prev_cmd = 0u8;

    macro_rules! push_contour {
        () => {
            if let Some(c) = contour.take() {
                if !c.segments.is_empty() {
                    shape.add_contour(c);
                }
            }
        };
    }

    while let Some(cmd) = reader.next_command() {
        let rel = cmd.is_ascii_lowercase();
        let up = cmd.to_ascii_uppercase();
        match up {
            b'M' => {
                push_contour!();
                let mut p = reader.coord()?;
                if rel {
                    p = cur + p;
                }
                cur = p;
                start = p;
                contour = Some(Contour::new());
                // Subsequent implicit pairs are line-to.
                while reader.more_numbers() {
                    let mut q = reader.coord()?;
                    if rel {
                        q = cur + q;
                    }
                    add_line(contour.as_mut().unwrap(), cur, q);
                    cur = q;
                }
                prev_control = None;
            }
            b'L' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let mut q = reader.coord()?;
                    if rel {
                        q = cur + q;
                    }
                    add_line(c, cur, q);
                    cur = q;
                }
                prev_control = None;
            }
            b'H' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let mut x = reader.number()?;
                    if rel {
                        x += cur.x;
                    }
                    let q = Vector2::new(x, cur.y);
                    add_line(c, cur, q);
                    cur = q;
                }
                prev_control = None;
            }
            b'V' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let mut y = reader.number()?;
                    if rel {
                        y += cur.y;
                    }
                    let q = Vector2::new(cur.x, y);
                    add_line(c, cur, q);
                    cur = q;
                }
                prev_control = None;
            }
            b'C' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let (mut c1, mut c2, mut end) =
                        (reader.coord()?, reader.coord()?, reader.coord()?);
                    if rel {
                        c1 = cur + c1;
                        c2 = cur + c2;
                        end = cur + end;
                    }
                    c.add_edge(EdgeSegment::cubic(cur, c1, c2, end));
                    prev_control = Some(c2);
                    cur = end;
                }
            }
            b'S' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let (mut c2, mut end) = (reader.coord()?, reader.coord()?);
                    if rel {
                        c2 = cur + c2;
                        end = cur + end;
                    }
                    let c1 = if matches!(prev_cmd, b'C' | b'S') {
                        cur + (cur - prev_control.unwrap_or(cur))
                    } else {
                        cur
                    };
                    c.add_edge(EdgeSegment::cubic(cur, c1, c2, end));
                    prev_control = Some(c2);
                    cur = end;
                }
            }
            b'Q' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let (mut ctrl, mut end) = (reader.coord()?, reader.coord()?);
                    if rel {
                        ctrl = cur + ctrl;
                        end = cur + end;
                    }
                    c.add_edge(EdgeSegment::quadratic(cur, ctrl, end));
                    prev_control = Some(ctrl);
                    cur = end;
                }
            }
            b'T' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let mut end = reader.coord()?;
                    if rel {
                        end = cur + end;
                    }
                    let ctrl = if matches!(prev_cmd, b'Q' | b'T') {
                        cur + (cur - prev_control.unwrap_or(cur))
                    } else {
                        cur
                    };
                    c.add_edge(EdgeSegment::quadratic(cur, ctrl, end));
                    prev_control = Some(ctrl);
                    cur = end;
                }
            }
            b'A' => {
                let c = need_contour(&mut contour);
                while reader.more_numbers() {
                    let rx = reader.number()?;
                    let ry = reader.number()?;
                    let x_rot = reader.number()?;
                    let large = reader.flag()?;
                    let sweep = reader.flag()?;
                    let mut end = reader.coord()?;
                    if rel {
                        end = cur + end;
                    }
                    arc_to_cubics(c, cur, rx, ry, x_rot.to_radians(), large, sweep, end);
                    cur = end;
                }
                prev_control = None;
            }
            b'Z' => {
                if let Some(c) = contour.as_mut() {
                    if cur != start {
                        add_line(c, cur, start);
                    }
                    cur = start;
                }
                prev_control = None;
            }
            _ => {
                return Err(SvgError(format!(
                    "unsupported path command '{}'",
                    up as char
                )));
            }
        }
        prev_cmd = up;
    }
    push_contour!();
    Ok(shape)
}

fn need_contour(contour: &mut Option<Contour>) -> &mut Contour {
    contour.get_or_insert_with(Contour::new)
}

fn add_line(contour: &mut Contour, a: Vector2, b: Vector2) {
    if a != b {
        contour.add_edge(EdgeSegment::line(a, b));
    }
}

/// Convert an SVG elliptical arc to a sequence of cubic Béziers.
#[allow(clippy::too_many_arguments)]
fn arc_to_cubics(
    contour: &mut Contour,
    from: Vector2,
    mut rx: f64,
    mut ry: f64,
    phi: f64,
    large: bool,
    sweep: bool,
    to: Vector2,
) {
    if rx == 0.0 || ry == 0.0 || from == to {
        add_line(contour, from, to);
        return;
    }
    rx = rx.abs();
    ry = ry.abs();
    let (cos_p, sin_p) = (phi.cos(), phi.sin());
    // Step 1: compute (x1', y1')
    let dx = (from.x - to.x) / 2.0;
    let dy = (from.y - to.y) / 2.0;
    let x1p = cos_p * dx + sin_p * dy;
    let y1p = -sin_p * dx + cos_p * dy;
    // Correct radii
    let lambda = x1p * x1p / (rx * rx) + y1p * y1p / (ry * ry);
    if lambda > 1.0 {
        let s = lambda.sqrt();
        rx *= s;
        ry *= s;
    }
    // Step 2: center'
    let num = (rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p).max(0.0);
    let den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
    let mut coef = if den != 0.0 { (num / den).sqrt() } else { 0.0 };
    if large == sweep {
        coef = -coef;
    }
    let cxp = coef * rx * y1p / ry;
    let cyp = -coef * ry * x1p / rx;
    // Step 3: center
    let cx = cos_p * cxp - sin_p * cyp + (from.x + to.x) / 2.0;
    let cy = sin_p * cxp + cos_p * cyp + (from.y + to.y) / 2.0;
    // Step 4: angles
    let ang = |ux: f64, uy: f64, vx: f64, vy: f64| {
        let dot = ux * vx + uy * vy;
        let len = ((ux * ux + uy * uy) * (vx * vx + vy * vy)).sqrt();
        let mut a = (dot / len).clamp(-1.0, 1.0).acos();
        if ux * vy - uy * vx < 0.0 {
            a = -a;
        }
        a
    };
    let theta1 = ang(1.0, 0.0, (x1p - cxp) / rx, (y1p - cyp) / ry);
    let mut dtheta = ang(
        (x1p - cxp) / rx,
        (y1p - cyp) / ry,
        (-x1p - cxp) / rx,
        (-y1p - cyp) / ry,
    );
    if !sweep && dtheta > 0.0 {
        dtheta -= 2.0 * PI;
    } else if sweep && dtheta < 0.0 {
        dtheta += 2.0 * PI;
    }

    let segments = (ARC_SEGMENTS_PER_PI * dtheta.abs() / PI).ceil().max(1.0) as usize;
    let delta = dtheta / segments as f64;
    let t = 4.0 / 3.0 * (delta / 4.0).tan();

    let mut start_pt = from;
    let mut angle = theta1;
    for _ in 0..segments {
        let end_angle = angle + delta;
        let p_on = |a: f64| {
            let (ca, sa) = (a.cos(), a.sin());
            Vector2::new(
                cx + rx * ca * cos_p - ry * sa * sin_p,
                cy + rx * ca * sin_p + ry * sa * cos_p,
            )
        };
        let d_on = |a: f64| {
            let (ca, sa) = (a.cos(), a.sin());
            Vector2::new(
                -rx * sa * cos_p - ry * ca * sin_p,
                -rx * sa * sin_p + ry * ca * cos_p,
            )
        };
        let end_pt = p_on(end_angle);
        let c1 = start_pt + t * d_on(angle);
        let c2 = end_pt - t * d_on(end_angle);
        contour.add_edge(EdgeSegment::cubic(start_pt, c1, c2, end_pt));
        start_pt = end_pt;
        angle = end_angle;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_path() {
        let s = build_shape_from_svg_path("M0,0 L10,0 L10,10 L0,10 Z").unwrap();
        assert_eq!(s.contours.len(), 1);
        assert_eq!(s.contours[0].segments.len(), 4);
    }

    #[test]
    fn extract_path_from_svg() {
        let svg = r#"<svg><path d="M0,0 L5,5 Z"/></svg>"#;
        let s = load_svg_shape(svg).unwrap();
        assert_eq!(s.contours.len(), 1);
        assert!(s.inverse_y_axis);
    }

    #[test]
    fn cubic_command() {
        let s = build_shape_from_svg_path("M0,0 C1,2 2,-1 3,0").unwrap();
        assert!(matches!(s.contours[0].segments[0], EdgeSegment::Cubic(_)));
    }
}
