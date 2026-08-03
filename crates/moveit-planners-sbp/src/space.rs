// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The state space abstraction and its `R^n` implementation.

use rand::{Rng, RngExt};

use crate::error::SbpError;
use crate::sampling::{sample_ball_radius_fraction, sample_unit_vector};

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
/// formula, which is what lets a space compose a wraparound revolute joint
/// (shortest arc, not linear difference, see `So2Space`) or an SO(3)
/// orientation (geodesic, not linear blend, see `Se3Space`) without changing
/// anything in [`crate::nn`] or [`crate::rrt_connect`] — both are written
/// only against this trait.
///
/// # Object safety
///
/// Every method here takes `&mut dyn Rng` rather than a generic `<R: Rng>`
/// parameter, which costs one dynamic dispatch per random draw but is what
/// makes `StateSpace` itself object-safe: `CompoundSpace` holds a
/// heterogeneous list of subspaces (a revolute joint next to a floating
/// joint next to a prismatic one) behind `Box<dyn StateSpace<State = _>>`,
/// which is only expressible at all if the trait has no generic methods to
/// put in a vtable. An earlier version of this trait used `<R: Rng>` and had
/// to be changed for exactly this reason — see
/// `crates/moveit-planners-sbp`'s commit history for the record of that
/// change and why it was necessary rather than a `CompoundSpace`-only
/// workaround.
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
    fn sample_uniform(&self, rng: &mut dyn Rng) -> Self::State;

    /// Draws a state uniformly at random from the ball of `radius` around
    /// `center` under [`distance`](StateSpace::distance) (clipped to this
    /// space's bounds) — not merely *within* `radius`, but distributed
    /// uniformly over the ball's volume, so a caller sampling many draws
    /// gets a sample that thins out from the centre the way a uniform
    /// distribution over a disc or sphere actually does, not one
    /// artificially concentrated toward the ball's corners or shell.
    fn sample_near(&self, rng: &mut dyn Rng, center: &Self::State, radius: f64) -> Self::State;
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

    fn sample_uniform(&self, rng: &mut dyn Rng) -> Vec<f64> {
        self.bounds
            .iter()
            .map(|&(min, max)| rng.random_range(min..=max))
            .collect()
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &Vec<f64>, radius: f64) -> Vec<f64> {
        debug_assert_eq!(center.len(), self.dimension());
        if radius <= 0.0 {
            let mut state = center.clone();
            self.enforce_bounds(&mut state);
            return state;
        }

        let dim = self.dimension();
        // See `crate::sampling` for why this is a direction drawn uniformly
        // on the unit sphere (via `sample_unit_vector`, no rejection) rather
        // than the exponentially-expensive-in-dimension rejection sampling
        // this method used to do — see
        // `sample_near_completes_at_planning_relevant_dimension` below,
        // this method's regression test for that history.
        let direction = sample_unit_vector(rng, dim);
        let r = radius * sample_ball_radius_fraction(rng, dim);

        let mut state: Vec<f64> = center
            .iter()
            .zip(&direction)
            .map(|(c, d)| c + d * r)
            .collect();
        self.enforce_bounds(&mut state);
        state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::assert_metric_and_interpolation_axioms;
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
    fn sample_near_stays_within_the_ball_and_in_bounds() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        let center = vec![0.0, 5.0];
        let radius = 0.5;
        for _ in 0..2000 {
            let sample = s.sample_near(&mut rng, &center, radius);
            assert!(s.satisfies_bounds(&sample));
            assert!(
                s.distance(&center, &sample) <= radius + 1e-9,
                "sample {sample:?} is farther than radius {radius} from center {center:?}"
            );
        }
    }

    #[test]
    fn sample_near_zero_radius_returns_clamped_center() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        assert_eq!(
            s.sample_near(&mut rng, &vec![0.0, 5.0], 0.0),
            vec![0.0, 5.0]
        );
    }

    /// The property a box-shaped `sample_near` gets wrong: a distribution
    /// uniform over an n-ball's *volume* puts a fraction `(1/2)^n` of its
    /// mass within half the radius, since enclosed volume scales as `r^n`.
    /// A box has no such property (its mass is not even confined to the
    /// ball at all — its corners lie outside it), so this is the property
    /// that distinguishes the two implementations, not merely "stays within
    /// radius".
    #[test]
    fn sample_near_is_uniform_over_the_ball_volume() {
        let s =
            RealVectorSpace::new(vec![(-100.0, 100.0), (-100.0, 100.0), (-100.0, 100.0)]).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(4);
        let center = vec![0.0, 0.0, 0.0];
        let radius = 10.0;
        let n = 20_000;
        let within_half = (0..n)
            .filter(|_| {
                let sample = s.sample_near(&mut rng, &center, radius);
                s.distance(&center, &sample) <= radius / 2.0
            })
            .count();
        let observed = within_half as f64 / n as f64;
        let expected = 0.5_f64.powi(3); // (1/2)^dim, dim == 3
        assert!(
            (observed - expected).abs() < 0.02,
            "expected about {expected:.3} of samples within half the radius \
             (uniform over the ball's volume), got {observed:.3}"
        );
    }

    /// Regression test for the rejection-sampling version of `sample_near`:
    /// its acceptance probability collapses exponentially with dimension
    /// (about 1 in 86,000 draws at `dim = 15`), so at the 14-15 DOF
    /// planning groups this crate targets (`fixtures/dual_arm_panda.urdf`
    /// is 14 DOF, PR2's `arms_and_torso` group is 15) it would make this
    /// test take on the order of minutes to hours instead of milliseconds.
    /// No wall-clock assertion here on purpose (see `Termination`'s doc
    /// comment for why a duration comparison is the wrong tool for this):
    /// the signal is that a `standard_normal`-based `sample_near` finishes
    /// a normal test run at all, and a rejection-based one does not.
    #[test]
    fn sample_near_completes_at_planning_relevant_dimension() {
        let bounds: Vec<(f64, f64)> = (0..15).map(|_| (-100.0, 100.0)).collect();
        let s = RealVectorSpace::new(bounds).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(5);
        let center = vec![0.0; 15];
        for _ in 0..5000 {
            let sample = s.sample_near(&mut rng, &center, 1.0);
            assert!(s.distance(&center, &sample) <= 1.0 + 1e-9);
        }
    }

    #[test]
    fn metric_and_interpolation_axioms_hold() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(6);
        assert_metric_and_interpolation_axioms(
            &s,
            &mut rng,
            |rng| s.sample_uniform(rng),
            2000,
            1e-9,
        );
    }
}
