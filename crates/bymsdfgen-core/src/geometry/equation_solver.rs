//! Closed-form polynomial root finders. Port of `core/equation-solver.cpp`.
//!
//! The C++ versions write roots into a caller-provided `double[]` and return a
//! count (with `-1` meaning "infinitely many"). Here we return a stack-allocated
//! [`Roots`] (`ArrayVec`) — no heap, no out-parameter/count desync. The special
//! "infinite solutions" case (`0 == 0`) is reported via [`SolveResult`].

use arrayvec::ArrayVec;

/// Up to three real roots, stored inline on the stack.
pub type Roots = ArrayVec<f64, 3>;

/// Result of [`solve_quadratic`]: either a finite set of roots or "all reals".
#[derive(Debug, Clone)]
pub enum SolveResult {
    Roots(Roots),
    /// The degenerate `0 == 0` case (equation satisfied for every x).
    Infinite,
}

impl SolveResult {
    /// Treat the infinite case as an empty root set (the common caller need).
    #[inline]
    pub fn roots(self) -> Roots {
        match self {
            SolveResult::Roots(r) => r,
            SolveResult::Infinite => Roots::new(),
        }
    }
}

/// Solve `a x^2 + b x + c = 0`.
pub fn solve_quadratic(a: f64, b: f64, c: f64) -> SolveResult {
    let mut out = Roots::new();
    // a == 0 -> linear equation (also when b dominates a numerically)
    if a == 0.0 || b.abs() > 1e12 * a.abs() {
        if b == 0.0 {
            if c == 0.0 {
                return SolveResult::Infinite; // 0 == 0
            }
            return SolveResult::Roots(out);
        }
        out.push(-c / b);
        return SolveResult::Roots(out);
    }
    let mut dscr = b * b - 4.0 * a * c;
    if dscr > 0.0 {
        dscr = dscr.sqrt();
        out.push((-b + dscr) / (2.0 * a));
        out.push((-b - dscr) / (2.0 * a));
    } else if dscr == 0.0 {
        out.push(-b / (2.0 * a));
    }
    SolveResult::Roots(out)
}

fn solve_cubic_normed(mut a: f64, b: f64, c: f64) -> Roots {
    use std::f64::consts::PI;
    let mut out = Roots::new();
    let a2 = a * a;
    let mut q = (1.0 / 9.0) * (a2 - 3.0 * b);
    let r = (1.0 / 54.0) * (a * (2.0 * a2 - 9.0 * b) + 27.0 * c);
    let r2 = r * r;
    let q3 = q * q * q;
    a *= 1.0 / 3.0;
    if r2 < q3 {
        let mut t = r / q3.sqrt();
        t = t.clamp(-1.0, 1.0);
        t = t.acos();
        q = -2.0 * q.sqrt();
        out.push(q * (1.0 / 3.0 * t).cos() - a);
        out.push(q * (1.0 / 3.0 * (t + 2.0 * PI)).cos() - a);
        out.push(q * (1.0 / 3.0 * (t - 2.0 * PI)).cos() - a);
    } else {
        let u = (if r < 0.0 { 1.0 } else { -1.0 }) * (r.abs() + (r2 - q3).sqrt()).powf(1.0 / 3.0);
        let v = if u == 0.0 { 0.0 } else { q / u };
        out.push((u + v) - a);
        if u == v || (u - v).abs() < 1e-12 * (u + v).abs() {
            out.push(-0.5 * (u + v) - a);
        }
    }
    out
}

/// Solve `a x^3 + b x^2 + c x + d = 0`.
pub fn solve_cubic(a: f64, b: f64, c: f64, d: f64) -> Roots {
    if a != 0.0 {
        let bn = b / a;
        if bn.abs() < 1e6 {
            // above this ratio numerical error exceeds treating a as zero
            return solve_cubic_normed(bn, c / a, d / a);
        }
    }
    solve_quadratic(b, c, d).roots()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sorted(mut r: Roots) -> Vec<f64> {
        r.sort_by(|a, b| a.partial_cmp(b).unwrap());
        r.to_vec()
    }

    #[test]
    fn quadratic_two_roots() {
        // x^2 - 3x + 2 = (x-1)(x-2)
        let r = sorted(solve_quadratic(1.0, -3.0, 2.0).roots());
        assert!((r[0] - 1.0).abs() < 1e-9);
        assert!((r[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn quadratic_linear_fallback() {
        // 0 x^2 + 2x - 4 = 0 -> x = 2
        let r = solve_quadratic(0.0, 2.0, -4.0).roots();
        assert_eq!(r.len(), 1);
        assert!((r[0] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn quadratic_infinite() {
        assert!(matches!(
            solve_quadratic(0.0, 0.0, 0.0),
            SolveResult::Infinite
        ));
    }

    #[test]
    fn cubic_three_roots() {
        // (x)(x-1)(x+1) = x^3 - x
        let r = sorted(solve_cubic(1.0, 0.0, -1.0, 0.0));
        assert_eq!(r.len(), 3);
        assert!((r[0] + 1.0).abs() < 1e-9);
        assert!((r[1]).abs() < 1e-9);
        assert!((r[2] - 1.0).abs() < 1e-9);
    }
}
