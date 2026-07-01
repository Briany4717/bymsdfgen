//! Shape distance finder. Port of `core/ShapeDistanceFinder.hpp`.
//!
//! Holds a reusable per-edge cache so consecutive queries (which are spatially
//! coherent — adjacent pixels) can skip far-away edges cheaply.

use super::combiners::ContourCombiner;
use super::selectors::EdgeSelector;
use crate::geometry::Shape;
use crate::math::Vector2;

pub struct ShapeDistanceFinder<'a, C: ContourCombiner> {
    shape: &'a Shape,
    combiner: C,
    cache: Vec<<C::Selector as EdgeSelector>::Cache>,
}

impl<'a, C: ContourCombiner> ShapeDistanceFinder<'a, C> {
    pub fn new(shape: &'a Shape) -> Self {
        let combiner = C::new(shape);
        let cache = vec![<C::Selector as EdgeSelector>::Cache::default(); shape.edge_count()];
        ShapeDistanceFinder {
            shape,
            combiner,
            cache,
        }
    }

    /// Distance from `origin` (in shape space) to the shape.
    pub fn distance(&mut self, origin: Vector2) -> <C::Selector as EdgeSelector>::Distance {
        let shape = self.shape;
        self.combiner.reset(origin);
        let mut ci = 0;
        for (contour_index, contour) in shape.contours.iter().enumerate() {
            let n = contour.segments.len();
            if n == 0 {
                continue;
            }
            let selector = self.combiner.edge_selector(contour_index);
            // Process each edge as the "current" one with cyclic neighbours, matching
            // the original's prev/cur/next walk.
            let mut prev = n.saturating_sub(2);
            let mut cur = n - 1;
            for next in 0..n {
                selector.add_edge(
                    &mut self.cache[ci],
                    &contour.segments[prev],
                    &contour.segments[cur],
                    &contour.segments[next],
                    contour.colors[cur],
                );
                ci += 1;
                prev = cur;
                cur = next;
            }
        }
        self.combiner.distance()
    }

    /// One-shot distance (no persistent cache), mirrors `oneShotDistance`.
    pub fn one_shot(shape: &Shape, origin: Vector2) -> <C::Selector as EdgeSelector>::Distance {
        let mut finder = ShapeDistanceFinder::<C>::new(shape);
        finder.distance(origin)
    }
}
