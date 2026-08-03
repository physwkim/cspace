// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The state space abstraction and its `R^n` implementation.

use rand::{Rng, RngExt};

use crate::error::SbpError;

/// A metric space of planner states.
///
/// [`distance`](StateSpace::distance) must be a true metric on `State`:
/// non-negative, zero exactly when the two states are equal, symmetric
/// (`distance(a, b) == distance(b, a)`), and satisfying the triangle
/// inequality. [`crate::nn::Gnat`], this crate's nearest-neighbour index,
/// relies on exactly those three properties for the pruning bound it uses to
/// avoid a full scan — see its doc comment. Nothing here checks the
/// contract; a `StateSpace` that violates it makes `Gnat::nearest` silently
/// wrong rather than panicking, so get the metric right.
///
/// The trait exists so this crate is not Euclidean-by-assumption:
/// `distance` and `interpolate` are per-space operations rather than a fixed
/// formula, which is what lets a future space compose a wraparound revolute
/// joint (shortest arc, not linear difference) or an SO(3) orientation
/// (geodesic, not linear blend) without changing anything in [`crate::nn`]
/// or [`crate::rrt_connect`] — both are written only against this trait.
pub trait StateSpace {
    /// A single point in the space.
    type State: Clone;

    /// Number of scalar degrees of freedom.
    fn dimension(&self) -> usize;

    /// Distance between two states. Must be a metric — see the trait docs.
    fn distance(&self, a: &Self::State, b: &Self::State) -> f64;

    /// The state a fraction `t` of the way from `from` to `to` along this
    /// space's own geodesic. `t <= 0.0` gives `from` back, `t >= 1.0` gives
    /// `to` back, exactly (not just within tolerance): callers rely on
    /// `t = 1.0` reproducing `to` bit-for-bit to recognise "the tree reached
    /// its target exactly" without a separate distance check.
    fn interpolate(&self, from: &Self::State, to: &Self::State, t: f64) -> Self::State;

    /// Clamps `state` into this space's bounds in place.
    fn enforce_bounds(&self, state: &mut Self::State);

    /// Whether `state` is within this space's bounds.
    fn satisfies_bounds(&self, state: &Self::State) -> bool;

    /// Draws a state uniformly at random from this space's bounds.
    fn sample_uniform<R: Rng>(&self, rng: &mut R) -> Self::State;

    /// Draws a state uniformly at random within `radius` of `center`
    /// (clipped to this space's bounds).
    fn sample_near<R: Rng>(&self, rng: &mut R, center: &Self::State, radius: f64) -> Self::State;
}

/// Plain bounded `R^n` with the Euclidean metric: `distance` is the L2 norm
/// of the difference, `interpolate` is a linear blend.
///
/// This is the one concrete [`StateSpace`] this crate ships for Phase 7's
/// initial scope — no wraparound, no orientation, just axis-aligned box
/// bounds. Compound spaces (a revolute joint with wraparound, SO(3) for a
/// floating joint's orientation, a product space combining several joints)
/// are future work layered on the same trait; nothing here assumes they
/// will look like this one.
#[derive(Debug, Clone, PartialEq)]
pub struct RealVectorSpace {
    bounds: Vec<(f64, f64)>,
}

impl RealVectorSpace {
    /// Builds a space from per-dimension `(min, max)` bounds.
    ///
    /// # Errors
    /// [`SbpError::NoDimensions`] if `bounds` is empty. [`SbpError::InvalidBound`]
    /// if any bound is non-finite or has `min > max`.
    pub fn new(bounds: Vec<(f64, f64)>) -> Result<Self, SbpError> {
        if bounds.is_empty() {
            return Err(SbpError::NoDimensions);
        }
        for (index, &(min, max)) in bounds.iter().enumerate() {
            if !min.is_finite() || !max.is_finite() || min > max {
                return Err(SbpError::InvalidBound { index, min, max });
            }
        }
        Ok(Self { bounds })
    }

    /// The `(min, max)` bound at `index`.
    pub fn bound(&self, index: usize) -> (f64, f64) {
        self.bounds[index]
    }
}

impl StateSpace for RealVectorSpace {
    type State = Vec<f64>;

    fn dimension(&self) -> usize {
        self.bounds.len()
    }

    fn distance(&self, a: &Vec<f64>, b: &Vec<f64>) -> f64 {
        debug_assert_eq!(a.len(), self.dimension());
        debug_assert_eq!(b.len(), self.dimension());
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y) * (x - y))
            .sum::<f64>()
            .sqrt()
    }

    fn interpolate(&self, from: &Vec<f64>, to: &Vec<f64>, t: f64) -> Vec<f64> {
        debug_assert_eq!(from.len(), self.dimension());
        debug_assert_eq!(to.len(), self.dimension());
        if t <= 0.0 {
            return from.clone();
        }
        if t >= 1.0 {
            return to.clone();
        }
        from.iter().zip(to).map(|(f, g)| f + (g - f) * t).collect()
    }

    fn enforce_bounds(&self, state: &mut Vec<f64>) {
        debug_assert_eq!(state.len(), self.dimension());
        for (v, &(min, max)) in state.iter_mut().zip(&self.bounds) {
            *v = v.clamp(min, max);
        }
    }

    fn satisfies_bounds(&self, state: &Vec<f64>) -> bool {
        debug_assert_eq!(state.len(), self.dimension());
        state
            .iter()
            .zip(&self.bounds)
            .all(|(&v, &(min, max))| v >= min && v <= max)
    }

    fn sample_uniform<R: Rng>(&self, rng: &mut R) -> Vec<f64> {
        self.bounds
            .iter()
            .map(|&(min, max)| rng.random_range(min..=max))
            .collect()
    }

    fn sample_near<R: Rng>(&self, rng: &mut R, center: &Vec<f64>, radius: f64) -> Vec<f64> {
        debug_assert_eq!(center.len(), self.dimension());
        center
            .iter()
            .zip(&self.bounds)
            .map(|(&c, &(min, max))| {
                let lo = (c - radius).max(min);
                let hi = (c + radius).min(max);
                if lo >= hi {
                    c.clamp(min, max)
                } else {
                    rng.random_range(lo..=hi)
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    fn space() -> RealVectorSpace {
        RealVectorSpace::new(vec![(-1.0, 1.0), (0.0, 10.0)]).unwrap()
    }

    #[test]
    fn empty_bounds_is_no_dimensions() {
        assert_eq!(RealVectorSpace::new(vec![]), Err(SbpError::NoDimensions));
    }

    #[test]
    fn inverted_bound_is_rejected() {
        assert_eq!(
            RealVectorSpace::new(vec![(1.0, -1.0)]),
            Err(SbpError::InvalidBound {
                index: 0,
                min: 1.0,
                max: -1.0
            })
        );
    }

    #[test]
    fn non_finite_bound_is_rejected() {
        assert_eq!(
            RealVectorSpace::new(vec![(0.0, f64::INFINITY)]),
            Err(SbpError::InvalidBound {
                index: 0,
                min: 0.0,
                max: f64::INFINITY
            })
        );
    }

    #[test]
    fn distance_is_euclidean() {
        let s = space();
        assert_eq!(s.distance(&vec![0.0, 0.0], &vec![3.0, 4.0]), 5.0);
    }

    #[test]
    fn distance_is_symmetric() {
        let s = space();
        let a = vec![-1.0, 2.0];
        let b = vec![0.5, 9.0];
        assert_eq!(s.distance(&a, &b), s.distance(&b, &a));
    }

    #[test]
    fn interpolate_endpoints_are_exact() {
        let s = space();
        let a = vec![-1.0, 0.0];
        let b = vec![1.0, 10.0];
        assert_eq!(s.interpolate(&a, &b, 0.0), a);
        assert_eq!(s.interpolate(&a, &b, 1.0), b);
    }

    #[test]
    fn interpolate_midpoint() {
        let s = space();
        let a = vec![0.0, 0.0];
        let b = vec![1.0, 10.0];
        assert_eq!(s.interpolate(&a, &b, 0.5), vec![0.5, 5.0]);
    }

    #[test]
    fn enforce_bounds_clamps() {
        let s = space();
        let mut state = vec![-5.0, 20.0];
        s.enforce_bounds(&mut state);
        assert_eq!(state, vec![-1.0, 10.0]);
    }

    #[test]
    fn satisfies_bounds_checks_every_dimension() {
        let s = space();
        assert!(s.satisfies_bounds(&vec![0.0, 5.0]));
        assert!(!s.satisfies_bounds(&vec![2.0, 5.0]));
        assert!(!s.satisfies_bounds(&vec![0.0, 11.0]));
    }

    #[test]
    fn sample_uniform_stays_in_bounds() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        for _ in 0..1000 {
            assert!(s.satisfies_bounds(&s.sample_uniform(&mut rng)));
        }
    }

    #[test]
    fn sample_near_stays_in_bounds_and_within_radius_or_clamped() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        let center = vec![0.0, 5.0];
        for _ in 0..1000 {
            let sample = s.sample_near(&mut rng, &center, 0.1);
            assert!(s.satisfies_bounds(&sample));
            for (c, v) in center.iter().zip(&sample) {
                assert!((c - v).abs() <= 0.1 + 1e-12);
            }
        }
    }
}
