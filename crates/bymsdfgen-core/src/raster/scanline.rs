//! Horizontal scanline through a shape. Port of `core/Scanline.{h,cpp}`.

/// How the running intersection total is turned into a fill decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillRule {
    NonZero,
    /// "even-odd"
    Odd,
    Positive,
    Negative,
}

impl FillRule {
    #[inline]
    pub fn interpret(self, intersections: i32) -> bool {
        match self {
            FillRule::NonZero => intersections != 0,
            FillRule::Odd => intersections & 1 != 0,
            FillRule::Positive => intersections > 0,
            FillRule::Negative => intersections < 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Intersection {
    pub x: f64,
    /// After [`Scanline::set_intersections`], this holds the *accumulated* direction.
    pub direction: i32,
}

/// A set of x-intersections at a fixed y, sorted with prefix-summed directions.
#[derive(Debug, Clone, Default)]
pub struct Scanline {
    intersections: Vec<Intersection>,
}

impl Scanline {
    pub fn new() -> Self {
        Scanline::default()
    }

    /// Replace the intersection list and sort + accumulate directions.
    pub fn set_intersections(&mut self, mut intersections: Vec<Intersection>) {
        intersections.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal));
        let mut total = 0;
        for i in &mut intersections {
            total += i.direction;
            i.direction = total;
        }
        self.intersections = intersections;
    }

    /// Index of the rightmost intersection with `x_i <= x`, or `None`.
    #[inline]
    fn move_to(&self, x: f64) -> Option<usize> {
        if self.intersections.is_empty() {
            return None;
        }
        let count = self.intersections.partition_point(|i| i.x <= x);
        if count == 0 { None } else { Some(count - 1) }
    }

    pub fn count_intersections(&self, x: f64) -> i32 {
        match self.move_to(x) {
            Some(i) => i as i32 + 1,
            None => 0,
        }
    }

    pub fn sum_intersections(&self, x: f64) -> i32 {
        match self.move_to(x) {
            Some(i) => self.intersections[i].direction,
            None => 0,
        }
    }

    pub fn filled(&self, x: f64, fill_rule: FillRule) -> bool {
        fill_rule.interpret(self.sum_intersections(x))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fill_inside_outside() {
        let mut s = Scanline::new();
        s.set_intersections(vec![
            Intersection {
                x: 1.0,
                direction: 1,
            },
            Intersection {
                x: 3.0,
                direction: -1,
            },
        ]);
        assert!(!s.filled(0.0, FillRule::NonZero));
        assert!(s.filled(2.0, FillRule::NonZero));
        assert!(!s.filled(4.0, FillRule::NonZero));
    }
}
