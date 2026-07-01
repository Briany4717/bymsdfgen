//! Experimental distance-based edge coloring. Port of `edgeColoringByDistance`.
//!
//! Treats splines as graph vertices and assigns the three channel colours so that
//! nearby splines differ, via a greedy second-degree graph colouring. This mirrors
//! the original's WIP implementation, including its heuristics.

use super::{identify_corners, symmetrical_trichotomy};
use crate::geometry::{EdgeColor, EdgeSegment, Shape};
use std::collections::VecDeque;

const MAX_RECOLOR_STEPS: i32 = 16;
const EDGE_DISTANCE_PRECISION: i32 = 16;

fn edge_to_edge_distance(a: &EdgeSegment, b: &EdgeSegment, precision: i32) -> f64 {
    if a.point(0.0) == b.point(0.0)
        || a.point(0.0) == b.point(1.0)
        || a.point(1.0) == b.point(0.0)
        || a.point(1.0) == b.point(1.0)
    {
        return 0.0;
    }
    let ifac = 1.0 / precision as f64;
    let mut min_distance = (b.point(0.0) - a.point(0.0)).length();
    for i in 0..=precision {
        let t = ifac * i as f64;
        let d = a.signed_distance(b.point(t)).0.distance.abs();
        min_distance = min_distance.min(d);
    }
    for i in 0..=precision {
        let t = ifac * i as f64;
        let d = b.signed_distance(a.point(t)).0.distance.abs();
        min_distance = min_distance.min(d);
    }
    min_distance
}

fn spline_to_spline_distance(
    segments: &[EdgeSegment],
    a_start: usize,
    a_end: usize,
    b_start: usize,
    b_end: usize,
    precision: i32,
) -> f64 {
    let mut min_distance = f64::MAX;
    for ai in a_start..a_end {
        for bi in b_start..b_end {
            if min_distance == 0.0 {
                break;
            }
            let d = edge_to_edge_distance(&segments[ai], &segments[bi], precision);
            min_distance = min_distance.min(d);
        }
    }
    min_distance
}

fn vertex_possible_colors(coloring: &[i32], edge_vector: &[i32], vertex_count: usize) -> i32 {
    let mut used = 0;
    for i in 0..vertex_count {
        if edge_vector[i] != 0 {
            used |= 1 << coloring[i];
        }
    }
    7 & !used
}

fn uncolor_same_neighbors(
    uncolored: &mut VecDeque<usize>,
    coloring: &mut [i32],
    edge_matrix: &[i32],
    vertex: usize,
    vertex_count: usize,
) {
    for i in (vertex + 1)..vertex_count {
        if edge_matrix[vertex * vertex_count + i] != 0 && coloring[i] == coloring[vertex] {
            coloring[i] = -1;
            uncolored.push_back(i);
        }
    }
    for i in 0..vertex {
        if edge_matrix[vertex * vertex_count + i] != 0 && coloring[i] == coloring[vertex] {
            coloring[i] = -1;
            uncolored.push_back(i);
        }
    }
}

const FIRST_POSSIBLE_COLOR: [i32; 8] = [-1, 0, 1, 0, 2, 2, 1, 0];

fn color_second_degree_graph(
    coloring: &mut [i32],
    edge_matrix: &[i32],
    vertex_count: usize,
    seed: &mut u64,
) {
    for i in 0..vertex_count {
        let mut possible = 7;
        for j in 0..i {
            if edge_matrix[i * vertex_count + j] != 0 {
                possible &= !(1 << coloring[j]);
            }
        }
        let color = match possible {
            1 => 0,
            2 => 1,
            3 => super::seed_extract2(seed) as i32,
            4 => 2,
            5 => ((super::seed_extract2(seed) == 0) as i32) << 1,
            6 => super::seed_extract2(seed) as i32 + 1,
            7 => (super::seed_extract3(seed) as i32 + i as i32) % 3,
            _ => 0,
        };
        coloring[i] = color;
    }
}

#[allow(clippy::too_many_arguments)]
fn try_add_edge(
    coloring: &mut [i32],
    edge_matrix: &mut [i32],
    vertex_count: usize,
    vertex_a: usize,
    vertex_b: usize,
    coloring_buffer: &mut [i32],
) {
    edge_matrix[vertex_a * vertex_count + vertex_b] = 1;
    edge_matrix[vertex_b * vertex_count + vertex_a] = 1;
    if coloring[vertex_a] != coloring[vertex_b] {
        return;
    }
    let b_possible = {
        let row = &edge_matrix[vertex_b * vertex_count..vertex_b * vertex_count + vertex_count];
        vertex_possible_colors(coloring, row, vertex_count)
    };
    if b_possible != 0 {
        coloring[vertex_b] = FIRST_POSSIBLE_COLOR[b_possible as usize];
        return;
    }
    coloring_buffer.copy_from_slice(&coloring[..vertex_count]);
    let mut uncolored: VecDeque<usize> = VecDeque::new();
    {
        let buf = &mut *coloring_buffer;
        buf[vertex_b] = FIRST_POSSIBLE_COLOR[(7 & !(1 << coloring[vertex_a])) as usize];
        uncolor_same_neighbors(&mut uncolored, buf, edge_matrix, vertex_b, vertex_count);
        let mut step = 0;
        while let Some(i) = uncolored.pop_front() {
            if step >= MAX_RECOLOR_STEPS {
                uncolored.push_front(i);
                break;
            }
            let possible = {
                let row = &edge_matrix[i * vertex_count..i * vertex_count + vertex_count];
                vertex_possible_colors(buf, row, vertex_count)
            };
            if possible != 0 {
                buf[i] = FIRST_POSSIBLE_COLOR[possible as usize];
                continue;
            }
            loop {
                buf[i] = step % 3;
                step += 1;
                if !(edge_matrix[i * vertex_count + vertex_a] != 0 && buf[i] == coloring[vertex_a])
                {
                    break;
                }
            }
            uncolor_same_neighbors(&mut uncolored, buf, edge_matrix, i, vertex_count);
        }
    }
    if !uncolored.is_empty() {
        edge_matrix[vertex_a * vertex_count + vertex_b] = 0;
        edge_matrix[vertex_b * vertex_count + vertex_a] = 0;
        return;
    }
    coloring[..vertex_count].copy_from_slice(&coloring_buffer[..vertex_count]);
}

pub fn edge_coloring_by_distance(shape: &mut Shape, angle_threshold: f64, mut seed: u64) {
    let cross_threshold = angle_threshold.sin();

    // Collected segments (flat copies) plus a back-reference (contour, index) so
    // final colours can be written into the shape's SoA arrays.
    let mut segments: Vec<EdgeSegment> = Vec::new();
    let mut refs: Vec<(usize, usize)> = Vec::new();
    let mut spline_starts: Vec<usize> = Vec::new();

    let contour_count = shape.contours.len();
    for ci in 0..contour_count {
        if shape.contours[ci].segments.is_empty() {
            continue;
        }
        let corners = identify_corners(&shape.contours[ci], cross_threshold);
        spline_starts.push(segments.len());

        if corners.is_empty() {
            for si in 0..shape.contours[ci].segments.len() {
                segments.push(shape.contours[ci].segments[si]);
                refs.push((ci, si));
            }
        } else if corners.len() == 1 {
            let corner = corners[0];
            let m = shape.contours[ci].segments.len();
            if m >= 3 {
                for i in 0..m {
                    if i == m / 2 {
                        spline_starts.push(segments.len());
                    }
                    let idx = (corner + i) % m;
                    if symmetrical_trichotomy(i as i32, m as i32) != 0 {
                        segments.push(shape.contours[ci].segments[idx]);
                        refs.push((ci, idx));
                    } else {
                        shape.contours[ci].colors[idx] = EdgeColor::White;
                    }
                }
            } else {
                // Split fewer-than-three-segment contours.
                let mut parts: [Option<EdgeSegment>; 7] = [None; 7];
                let s0 = shape.contours[ci].segments[0].split_in_thirds();
                parts[3 * corner] = Some(s0[0]);
                parts[1 + 3 * corner] = Some(s0[1]);
                parts[2 + 3 * corner] = Some(s0[2]);
                let collect_after_rebuild: Vec<usize> = if m >= 2 {
                    let s1 = shape.contours[ci].segments[1].split_in_thirds();
                    parts[3 - 3 * corner] = Some(s1[0]);
                    parts[4 - 3 * corner] = Some(s1[1]);
                    parts[5 - 3 * corner] = Some(s1[2]);
                    vec![0, 1, 4, 5]
                } else {
                    vec![0, 2]
                };
                // Rebuild contour with all non-empty parts. Collected (non-white)
                // parts are recoloured later by the graph colouring; the rest stay
                // white, which is the correct default here.
                let mut new_segs = Vec::new();
                let mut new_cols = Vec::new();
                let mut part_to_new: [Option<usize>; 7] = [None; 7];
                for (i, part) in parts.iter().enumerate() {
                    if let Some(seg) = part {
                        part_to_new[i] = Some(new_segs.len());
                        new_segs.push(*seg);
                        new_cols.push(EdgeColor::White);
                    }
                }
                shape.contours[ci].segments = new_segs;
                shape.contours[ci].colors = new_cols;
                // Collect splines: insert a spline break before index 4/2 group.
                let break_at = if m >= 2 { 4 } else { 2 };
                for &p in &collect_after_rebuild {
                    if p == break_at {
                        spline_starts.push(segments.len());
                    }
                    let ni = part_to_new[p].unwrap();
                    segments.push(shape.contours[ci].segments[ni]);
                    refs.push((ci, ni));
                }
            }
        } else {
            let corner_count = corners.len();
            let mut spline = 0usize;
            let start = corners[0];
            let m = shape.contours[ci].segments.len();
            for i in 0..m {
                let index = (start + i) % m;
                if spline + 1 < corner_count && corners[spline + 1] == index {
                    spline_starts.push(segments.len());
                    spline += 1;
                }
                segments.push(shape.contours[ci].segments[index]);
                refs.push((ci, index));
            }
        }
    }
    spline_starts.push(segments.len());

    let spline_count = spline_starts.len() - 1;
    if spline_count == 0 {
        return;
    }

    // Distance matrix between splines.
    let mut distance_matrix = vec![0.0f64; spline_count * spline_count];
    for i in 0..spline_count {
        distance_matrix[i * spline_count + i] = -1.0;
        for j in (i + 1)..spline_count {
            let d = spline_to_spline_distance(
                &segments,
                spline_starts[i],
                spline_starts[i + 1],
                spline_starts[j],
                spline_starts[j + 1],
                EDGE_DISTANCE_PRECISION,
            );
            distance_matrix[i * spline_count + j] = d;
            distance_matrix[j * spline_count + i] = d;
        }
    }

    // Sorted list of graph edges (i<j) by distance.
    let mut graph_edges: Vec<(usize, usize)> = Vec::new();
    for i in 0..spline_count {
        for j in (i + 1)..spline_count {
            graph_edges.push((i, j));
        }
    }
    graph_edges.sort_by(|&(ai, aj), &(bi, bj)| {
        distance_matrix[ai * spline_count + aj]
            .partial_cmp(&distance_matrix[bi * spline_count + bj])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut edge_matrix = vec![0i32; spline_count * spline_count];
    let mut next_edge = 0;
    while next_edge < graph_edges.len() {
        let (i, j) = graph_edges[next_edge];
        if distance_matrix[i * spline_count + j] != 0.0 {
            break;
        }
        edge_matrix[i * spline_count + j] = 1;
        edge_matrix[j * spline_count + i] = 1;
        next_edge += 1;
    }

    let mut coloring = vec![0i32; spline_count];
    color_second_degree_graph(&mut coloring, &edge_matrix, spline_count, &mut seed);
    let mut buffer = vec![0i32; spline_count];
    while next_edge < graph_edges.len() {
        let (i, j) = graph_edges[next_edge];
        try_add_edge(
            &mut coloring,
            &mut edge_matrix,
            spline_count,
            i,
            j,
            &mut buffer,
        );
        next_edge += 1;
    }

    const COLORS: [EdgeColor; 3] = [EdgeColor::Yellow, EdgeColor::Cyan, EdgeColor::Magenta];
    let mut spline: isize = -1;
    for (i, &(ci, si)) in refs.iter().enumerate() {
        if spline_starts[(spline + 1) as usize] == i {
            spline += 1;
        }
        shape.contours[ci].colors[si] = COLORS[coloring[spline as usize] as usize];
    }
}
