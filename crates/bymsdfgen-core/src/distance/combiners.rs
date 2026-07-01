//! Contour combiners. Port of `core/contour-combiners.cpp`.
//!
//! The C++ templates become generics over the [`EdgeSelector`] trait — the same
//! zero-cost monomorphization, but with no manual template instantiation list.

use super::selectors::EdgeSelector;
use super::value::DistanceValue;
use crate::geometry::Shape;
use crate::math::Vector2;

/// Combines per-contour selector results into a single shape distance.
pub trait ContourCombiner: Clone {
    type Selector: EdgeSelector;

    fn new(shape: &Shape) -> Self;
    fn reset(&mut self, p: Vector2);
    fn edge_selector(&mut self, i: usize) -> &mut Self::Selector;
    fn distance(&self) -> <Self::Selector as EdgeSelector>::Distance;
}

/// Ignores contour topology — a single selector spanning the whole shape.
#[derive(Clone)]
pub struct SimpleContourCombiner<S: EdgeSelector> {
    shape_edge_selector: S,
}

impl<S: EdgeSelector> ContourCombiner for SimpleContourCombiner<S> {
    type Selector = S;

    fn new(_shape: &Shape) -> Self {
        SimpleContourCombiner {
            shape_edge_selector: S::default(),
        }
    }

    fn reset(&mut self, p: Vector2) {
        self.shape_edge_selector.reset(p);
    }

    fn edge_selector(&mut self, _i: usize) -> &mut S {
        &mut self.shape_edge_selector
    }

    fn distance(&self) -> S::Distance {
        self.shape_edge_selector.distance()
    }
}

/// Handles overlapping contours by tracking each contour's winding.
#[derive(Clone)]
pub struct OverlappingContourCombiner<S: EdgeSelector> {
    p: Vector2,
    windings: Vec<i32>,
    edge_selectors: Vec<S>,
}

impl<S: EdgeSelector> ContourCombiner for OverlappingContourCombiner<S> {
    type Selector = S;

    fn new(shape: &Shape) -> Self {
        let windings: Vec<i32> = shape.contours.iter().map(|c| c.winding()).collect();
        let edge_selectors = vec![S::default(); shape.contours.len()];
        OverlappingContourCombiner {
            p: Vector2::ZERO,
            windings,
            edge_selectors,
        }
    }

    fn reset(&mut self, p: Vector2) {
        self.p = p;
        for s in &mut self.edge_selectors {
            s.reset(p);
        }
    }

    fn edge_selector(&mut self, i: usize) -> &mut S {
        &mut self.edge_selectors[i]
    }

    fn distance(&self) -> S::Distance {
        let contour_count = self.edge_selectors.len();
        let mut shape_sel = S::default();
        let mut inner_sel = S::default();
        let mut outer_sel = S::default();
        shape_sel.reset(self.p);
        inner_sel.reset(self.p);
        outer_sel.reset(self.p);

        for i in 0..contour_count {
            let edge_distance = self.edge_selectors[i].distance();
            shape_sel.merge(&self.edge_selectors[i]);
            if self.windings[i] > 0 && edge_distance.resolve() >= 0.0 {
                inner_sel.merge(&self.edge_selectors[i]);
            }
            if self.windings[i] < 0 && edge_distance.resolve() <= 0.0 {
                outer_sel.merge(&self.edge_selectors[i]);
            }
        }

        let shape_distance = shape_sel.distance();
        let inner_distance = inner_sel.distance();
        let outer_distance = outer_sel.distance();
        let inner_scalar = inner_distance.resolve();
        let outer_scalar = outer_distance.resolve();

        let mut distance;
        let winding;
        if inner_scalar >= 0.0 && inner_scalar.abs() <= outer_scalar.abs() {
            distance = inner_distance;
            winding = 1;
            for i in 0..contour_count {
                if self.windings[i] > 0 {
                    let cd = self.edge_selectors[i].distance();
                    if cd.resolve().abs() < outer_scalar.abs() && cd.resolve() > distance.resolve()
                    {
                        distance = cd;
                    }
                }
            }
        } else if outer_scalar <= 0.0 && outer_scalar.abs() < inner_scalar.abs() {
            distance = outer_distance;
            winding = -1;
            for i in 0..contour_count {
                if self.windings[i] < 0 {
                    let cd = self.edge_selectors[i].distance();
                    if cd.resolve().abs() < inner_scalar.abs() && cd.resolve() < distance.resolve()
                    {
                        distance = cd;
                    }
                }
            }
        } else {
            return shape_distance;
        }

        for i in 0..contour_count {
            if self.windings[i] != winding {
                let cd = self.edge_selectors[i].distance();
                if cd.resolve() * distance.resolve() >= 0.0
                    && cd.resolve().abs() < distance.resolve().abs()
                {
                    distance = cd;
                }
            }
        }
        if distance.resolve() == shape_distance.resolve() {
            distance = shape_distance;
        }
        distance
    }
}
