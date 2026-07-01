//! Text shape description parser/serializer. Port of `core/shape-description.cpp`.
//!
//! Grammar (per the project README):
//! - optional leading `@y-up` / `@y-down`
//! - contours wrapped in braces: `{ ... }`
//! - points are `x, y`, separated by `;`
//! - the last point may be `#` (equal to the first)
//! - an edge spec may sit between two points: a colour (`c`/`m`/`y`/`w`) and/or
//!   one or two Bézier control points in parentheses.

use bymsdfgen_core::geometry::{Contour, EdgeColor, EdgeSegment, Shape};
use bymsdfgen_core::math::Vector2;

#[derive(Debug)]
pub struct ParseError(pub String);

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "shape description parse error: {}", self.0)
    }
}
impl std::error::Error for ParseError {}

fn err<T>(msg: impl Into<String>) -> Result<T, ParseError> {
    Err(ParseError(msg.into()))
}

/// Split `s` on top-level `delim`, ignoring delimiters inside parentheses.
fn split_top_level(s: &str, delim: char) -> Vec<String> {
    let mut out = Vec::new();
    let mut depth = 0i32;
    let mut cur = String::new();
    for c in s.chars() {
        match c {
            '(' => {
                depth += 1;
                cur.push(c);
            }
            ')' => {
                depth -= 1;
                cur.push(c);
            }
            _ if c == delim && depth == 0 => {
                out.push(cur.trim().to_string());
                cur.clear();
            }
            _ => cur.push(c),
        }
    }
    out.push(cur.trim().to_string());
    out
}

fn parse_coord(s: &str) -> Result<Vector2, ParseError> {
    let parts = split_top_level(s, ',');
    if parts.len() != 2 {
        return err(format!("expected 'x, y', got '{s}'"));
    }
    let x = parts[0]
        .trim()
        .parse::<f64>()
        .map_err(|_| ParseError(format!("bad number '{}'", parts[0])))?;
    let y = parts[1]
        .trim()
        .parse::<f64>()
        .map_err(|_| ParseError(format!("bad number '{}'", parts[1])))?;
    Ok(Vector2::new(x, y))
}

fn color_from_char(c: char) -> Option<EdgeColor> {
    match c {
        'c' | 'C' => Some(EdgeColor::Cyan),
        'm' | 'M' => Some(EdgeColor::Magenta),
        'y' | 'Y' => Some(EdgeColor::Yellow),
        'w' | 'W' => Some(EdgeColor::White),
        _ => None,
    }
}

fn build_edge(start: Vector2, controls: &[Vector2], end: Vector2) -> EdgeSegment {
    match controls.len() {
        0 => EdgeSegment::line(start, end),
        1 => EdgeSegment::quadratic(start, controls[0], end),
        _ => EdgeSegment::cubic(start, controls[0], controls[1], end),
    }
}

/// Parse a full shape description.
pub fn parse_shape(input: &str) -> Result<Shape, ParseError> {
    let mut shape = Shape::new();
    let trimmed = input.trim();

    // Optional Y-axis directive.
    let mut rest = trimmed;
    if let Some(s) = rest.strip_prefix("@y-up") {
        shape.inverse_y_axis = false;
        rest = s.trim_start();
    } else if let Some(s) = rest.strip_prefix("@y-down") {
        shape.inverse_y_axis = true;
        rest = s.trim_start();
    }

    let bytes = rest.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        // find next '{'
        while i < bytes.len() && bytes[i] != b'{' {
            i += 1;
        }
        if i >= bytes.len() {
            break;
        }
        let start_brace = i + 1;
        // find matching '}' (no nesting of braces in this grammar)
        let mut j = start_brace;
        while j < bytes.len() && bytes[j] != b'}' {
            j += 1;
        }
        if j >= bytes.len() {
            return err("unterminated contour (missing '}')");
        }
        let body = &rest[start_brace..j];
        shape.add_contour(parse_contour(body)?);
        i = j + 1;
    }

    Ok(shape)
}

fn parse_contour(body: &str) -> Result<Contour, ParseError> {
    let mut contour = Contour::new();
    let items = split_top_level(body, ';');

    let mut start: Option<Vector2> = None;
    let mut prev: Option<Vector2> = None;
    let mut pending_color = EdgeColor::White;
    let mut pending_controls: Vec<Vector2> = Vec::new();

    for raw in &items {
        let item = raw.trim();
        if item.is_empty() {
            continue;
        }
        if item == "#" {
            let s = start.ok_or_else(|| ParseError("'#' before any point".into()))?;
            let p = prev.ok_or_else(|| ParseError("'#' before any point".into()))?;
            contour.add_edge_colored(build_edge(p, &pending_controls, s), pending_color);
            prev = Some(s);
            pending_controls.clear();
            pending_color = EdgeColor::White;
            continue;
        }

        let first = item.chars().next().unwrap();
        let is_coord = first.is_ascii_digit() || first == '-' || first == '+' || first == '.';
        if is_coord {
            let coord = parse_coord(item)?;
            match prev {
                None => {
                    start = Some(coord);
                    prev = Some(coord);
                }
                Some(p) => {
                    contour
                        .add_edge_colored(build_edge(p, &pending_controls, coord), pending_color);
                    prev = Some(coord);
                    pending_controls.clear();
                    pending_color = EdgeColor::White;
                }
            }
        } else {
            // Edge spec: optional colour, optional control points in parens.
            let mut chars = item.char_indices().peekable();
            while let Some(&(idx, c)) = chars.peek() {
                if c.is_whitespace() {
                    chars.next();
                    continue;
                }
                if let Some(col) = color_from_char(c) {
                    pending_color = col;
                    chars.next();
                } else if c == '(' {
                    // parse to matching ')'
                    let close = item[idx..]
                        .find(')')
                        .map(|o| idx + o)
                        .ok_or_else(|| ParseError("unterminated '('".into()))?;
                    let inner = &item[idx + 1..close];
                    pending_controls.clear();
                    for cs in split_top_level(inner, ';') {
                        if !cs.trim().is_empty() {
                            pending_controls.push(parse_coord(&cs)?);
                        }
                    }
                    // advance past close
                    while let Some(&(k, _)) = chars.peek() {
                        if k > close {
                            break;
                        }
                        chars.next();
                    }
                } else {
                    return err(format!("unexpected character '{c}' in edge spec"));
                }
            }
        }
    }

    Ok(contour)
}

fn color_char(color: EdgeColor) -> char {
    match color {
        EdgeColor::Cyan => 'c',
        EdgeColor::Magenta => 'm',
        EdgeColor::Yellow => 'y',
        _ => 'w',
    }
}

fn fmt_coord(p: Vector2) -> String {
    format!("{:.12}, {:.12}", p.x, p.y)
}

/// Serialize a shape back to the text description (with edge colours).
pub fn write_shape(shape: &Shape) -> String {
    let mut out = String::new();
    out.push_str(if shape.inverse_y_axis {
        "@y-down\n"
    } else {
        "@y-up\n"
    });
    for contour in &shape.contours {
        if contour.segments.is_empty() {
            continue;
        }
        out.push_str("{ ");
        out.push_str(&fmt_coord(contour.segments[0].point(0.0)));
        for (i, seg) in contour.segments.iter().enumerate() {
            out.push_str("; ");
            out.push(color_char(contour.colors[i]));
            out.push_str("; ");
            let controls: &[Vector2] = match seg {
                EdgeSegment::Line(_) => &[],
                EdgeSegment::Quadratic(p) => std::slice::from_ref(&p[1]),
                EdgeSegment::Cubic(p) => &p[1..3],
            };
            if !controls.is_empty() {
                out.push('(');
                for (k, c) in controls.iter().enumerate() {
                    if k > 0 {
                        out.push_str("; ");
                    }
                    out.push_str(&fmt_coord(*c));
                }
                out.push_str("); ");
            }
            out.push_str(&fmt_coord(seg.point(1.0)));
        }
        out.push_str(" }\n");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_square() {
        let s = parse_shape("{ -1, -1; m; -1, +1; y; +1, +1; m; +1, -1; y; # }").unwrap();
        assert_eq!(s.contours.len(), 1);
        assert_eq!(s.contours[0].segments.len(), 4);
        assert_eq!(s.contours[0].colors[0], EdgeColor::Magenta);
    }

    #[test]
    fn parse_teardrop_cubic() {
        let s = parse_shape("{ 0, 1; (+1.6, -0.8; -1.6, -0.8); # }").unwrap();
        assert_eq!(s.contours.len(), 1);
        assert!(matches!(s.contours[0].segments[0], EdgeSegment::Cubic(_)));
    }

    #[test]
    fn roundtrip() {
        let s = parse_shape("{ 0,0; 2,0; 2,2; 0,2; # }").unwrap();
        let text = write_shape(&s);
        let s2 = parse_shape(&text).unwrap();
        assert_eq!(s2.contours[0].segments.len(), s.contours[0].segments.len());
    }
}
