// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `SO(2)`: a single wraparound revolute (continuous) joint.

use std::f64::consts::PI;

use rand::{Rng, RngExt};

use crate::space::StateSpace;

/// Wraps `theta` into `[-PI, PI)`.
fn normalize_angle(theta: f64) -> f64 {
    (theta + PI).rem_euclid(2.0 * PI) - PI
}

/// A single continuous (wraparound) revolute joint: the circle `SO(2)`.
///
/// Unlike [`RealVectorSpace`](crate::space::RealVectorSpace), this space has
/// no bounds to violate — every finite angle is reachable and every state is
/// stored normalized into `[-PI, PI)`. [`distance`](StateSpace::distance) and
/// [`interpolate`](StateSpace::interpolate) both go by the shorter arc: the
/// angles `3.0` and `-3.0` are close on this space (about `0.28` rad apart)
/// even though they are `6.0` apart as raw numbers, because the joint can
/// turn the other way around.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct So2Space;

impl So2Space {
    /// Builds the space. Takes no parameters: `SO(2)` has no bounds to
    /// configure.
    pub fn new() -> Self {
        Self
    }
}

impl StateSpace for So2Space {
    type State = f64;

    fn dimension(&self) -> usize {
        1
    }

    fn distance(&self, a: &f64, b: &f64) -> f64 {
        normalize_angle(a - b).abs()
    }

    fn interpolate(&self, from: &f64, to: &f64, t: f64) -> f64 {
        if t <= 0.0 {
            return *from;
        }
        if t >= 1.0 {
            return *to;
        }
        // The shorter signed arc from `from` to `to`, in `[-PI, PI)`: adding
        // any multiple of `2*PI` to `to` doesn't change where it points, so
        // this picks the direction that doesn't cross more than half the
        // circle, then walks `t` of the way along it.
        let diff = normalize_angle(to - from);
        normalize_angle(from + diff * t)
    }

    fn enforce_bounds(&self, state: &mut f64) {
        *state = normalize_angle(*state);
    }

    fn satisfies_bounds(&self, state: &f64) -> bool {
        state.is_finite()
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> f64 {
        rng.random_range(-PI..PI)
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &f64, radius: f64) -> f64 {
        if radius <= 0.0 {
            return normalize_angle(*center);
        }
        // A ball on a 1-dimensional space is just an interval: unlike
        // `RealVectorSpace::sample_near`, there's no shell-concentration
        // effect to correct for (see `crate::sampling::sample_ball_radius_fraction`'s
        // doc comment for why higher dimensions need that correction), so a
        // plain uniform draw over the interval is already volume-uniform.
        // Capping at `PI` on each side is what makes a radius that covers
        // the whole circle sample the whole circle instead of only the
        // reachable-without-wrapping part.
        let r = radius.min(PI);
        let offset = rng.random_range(-r..=r);
        normalize_angle(center + offset)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{
        assert_metric_and_interpolation_axioms, assert_sample_near_stays_within_radius,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    #[test]
    fn distance_takes_the_shorter_arc() {
        let s = So2Space::new();
        // Raw difference is 6.0; the shorter way around is 2*PI - 6.0.
        let d = s.distance(&3.0, &-3.0);
        assert!((d - (2.0 * PI - 6.0)).abs() < 1e-9, "distance = {d}");
    }

    #[test]
    fn interpolate_takes_the_shorter_arc() {
        let s = So2Space::new();
        // Halfway from 3.0 to -3.0 the shorter way is near the +/-PI
        // wraparound point, not near 0.0 (which a linear blend would give).
        let mid = s.interpolate(&3.0, &-3.0, 0.5);
        assert!(
            mid.abs() > 3.0,
            "interpolate(3.0, -3.0, 0.5) = {mid}, expected near +/-PI"
        );
    }

    #[test]
    fn interpolate_endpoints_are_exact() {
        let s = So2Space::new();
        assert_eq!(s.interpolate(&2.5, &-2.5, 0.0), 2.5);
        assert_eq!(s.interpolate(&2.5, &-2.5, 1.0), -2.5);
    }

    #[test]
    fn enforce_bounds_normalizes() {
        let s = So2Space::new();
        let mut state = 4.0 * PI + 0.1;
        s.enforce_bounds(&mut state);
        assert!((state - 0.1).abs() < 1e-9, "state = {state}");
        assert!(s.satisfies_bounds(&state));
    }

    #[test]
    fn non_finite_state_fails_bounds() {
        let s = So2Space::new();
        assert!(!s.satisfies_bounds(&f64::NAN));
        assert!(!s.satisfies_bounds(&f64::INFINITY));
    }

    #[test]
    fn sample_near_crosses_the_wraparound() {
        let s = So2Space::new();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let center = 3.0;
        let radius = 0.5;
        // 3.0 + 0.5 = 3.5 > PI, so some draws must land past the wraparound
        // point and come back out near -PI (the "-3.0 side").
        let crossed = (0..500).any(|_| s.sample_near(&mut rng, &center, radius) < -2.5);
        assert!(
            crossed,
            "500 draws of sample_near(3.0, 0.5) never crossed the wraparound point"
        );
    }

    #[test]
    fn metric_and_interpolation_axioms_hold() {
        let s = So2Space::new();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        assert_metric_and_interpolation_axioms(
            &s,
            &mut rng,
            |rng| s.sample_uniform(rng),
            2000,
            1e-9,
        );
    }

    #[test]
    fn sample_near_stays_within_radius() {
        let s = So2Space::new();
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        for &radius in &[0.1, 1.0, PI, 10.0] {
            let center = s.sample_uniform(&mut rng);
            assert_sample_near_stays_within_radius(&s, &mut rng, &center, radius, 500, 1e-9);
        }
    }
}
