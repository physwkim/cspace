// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `SO(2)`: a single wraparound revolute (continuous) joint.

use std::f64::consts::PI;

use rand::{Rng, RngExt};

use crate::sbp::space::StateSpace;

/// Wraps `theta` into `[-PI, PI)`.
fn normalize_angle(theta: f64) -> f64 {
    (theta + PI).rem_euclid(2.0 * PI) - PI
}

/// A single continuous (wraparound) revolute joint: the circle `SO(2)`.
///
/// Unlike [`RealVectorSpace`](crate::sbp::space::RealVectorSpace), this space has
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
        // effect to correct for (see `crate::sbp::sampling::sample_ball_radius_fraction`'s
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
    use crate::sbp::test_support::{
        assert_metric_and_interpolation_axioms, assert_sample_near_stays_within_radius,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    /// Matches upstream `RevoluteJointModel::distance`'s continuous branch
    /// (`revolute_joint_model.cpp:173-179`) transcribed independently here:
    /// `d = fmod(fabs(v1 - v2), 2*PI); d > PI ? 2*PI - d : d` (Rust's `%`
    /// agrees with C's `fmod` for the non-negative inputs `fabs` always
    /// produces). Includes a case that crosses the wraparound point, where
    /// `fmod`'s result and this space's own `normalize_angle` take different
    /// intermediate routes to the same answer.
    #[test]
    fn distance_matches_upstream_continuous_revolute_formula() {
        let s = So2Space::new();
        for &(v1, v2) in &[
            (0.0, 0.0),
            (1.0, 2.0),
            (3.0, -3.0),
            (-PI + 0.1, PI - 0.1),
            (2.9, -2.9),
        ] {
            let d = (v1 - v2).abs() % (2.0 * PI);
            let upstream = if d > PI { 2.0 * PI - d } else { d };
            let actual = s.distance(&v1, &v2);
            assert!(
                (actual - upstream).abs() < 1e-9,
                "distance({v1}, {v2}) = {actual}, upstream fmod formula = {upstream}"
            );
        }
    }

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

    // PORTING-PLAN.md:1269 records that the StateSpace trait's ability to
    // carry wraparound has "so far been a comment's assertion, never
    // verified" — the following are boundary-value tests, not the random
    // draws `assert_metric_and_interpolation_axioms` already runs below,
    // because a random f64 draw essentially never lands exactly on the seam
    // or exactly PI apart.

    /// Both crossing directions, constructed rather than drawn at random:
    /// two points close together *across* the seam, each on the far side of
    /// PI from the other's raw value.
    #[test]
    fn interpolate_crosses_the_seam_from_the_negative_side() {
        let s = So2Space::new();
        let from = -PI + 0.1;
        let to = PI - 0.1;
        // The short way from `from` to `to` is 0.2 rad, going negative
        // (through -PI, wrapping to the +PI side) — not the 2*PI - 0.2 rad
        // the raw values would suggest.
        let d = s.distance(&from, &to);
        assert!((d - 0.2).abs() < 1e-9, "distance = {d}");
        let mid = s.interpolate(&from, &to, 0.5);
        assert!(
            mid.abs() > PI - 0.11,
            "interpolate({from}, {to}, 0.5) = {mid}, expected near +/-PI (crossing the seam)"
        );
    }

    /// The mirror image of the above: same two points, arguments swapped,
    /// crossing from the positive side instead.
    #[test]
    fn interpolate_crosses_the_seam_from_the_positive_side() {
        let s = So2Space::new();
        let from = PI - 0.1;
        let to = -PI + 0.1;
        let d = s.distance(&from, &to);
        assert!((d - 0.2).abs() < 1e-9, "distance = {d}");
        let mid = s.interpolate(&from, &to, 0.5);
        assert!(
            mid.abs() > PI - 0.11,
            "interpolate({from}, {to}, 0.5) = {mid}, expected near +/-PI (crossing the seam)"
        );
        // The midpoint of a geodesic does not depend on which endpoint is
        // "from" and which is "to".
        let other_mid = s.interpolate(&to, &from, 0.5);
        assert!(
            (mid - other_mid).abs() < 1e-9,
            "interpolate(a, b, 0.5) = {mid} but interpolate(b, a, 0.5) = {other_mid}"
        );
    }

    /// Exactly PI apart: both arcs are the same length, so the choice of
    /// direction is a tie, not a bug either way. What must hold regardless
    /// of which side `normalize_angle`'s tie-break lands on is that the
    /// midpoint still splits the distance exactly in half on both sides —
    /// this is the case a trait that quietly special-cased "the shorter
    /// arc" without handling the tie could return a mid-point that is not
    /// equidistant, or produce a non-finite value from a 0/0 in the
    /// direction computation.
    #[test]
    fn interpolate_at_exactly_pi_apart_still_splits_the_distance_evenly() {
        let s = So2Space::new();
        let from = 0.0;
        let to = PI;
        let whole = s.distance(&from, &to);
        assert!((whole - PI).abs() < 1e-9, "distance(0, PI) = {whole}");
        let mid = s.interpolate(&from, &to, 0.5);
        assert!(mid.is_finite(), "interpolate(0, PI, 0.5) = {mid}");
        let d_from_mid = s.distance(&from, &mid);
        let d_mid_to = s.distance(&mid, &to);
        assert!(
            (d_from_mid - PI / 2.0).abs() < 1e-9,
            "distance(from, mid) = {d_from_mid}, expected PI/2"
        );
        assert!(
            (d_mid_to - PI / 2.0).abs() < 1e-9,
            "distance(mid, to) = {d_mid_to}, expected PI/2"
        );
    }

    /// Distance symmetry and the triangle inequality at constructed points
    /// straddling the seam, not the random pairs
    /// `assert_metric_and_interpolation_axioms` already draws below.
    #[test]
    fn distance_symmetry_and_triangle_inequality_hold_across_the_seam() {
        let s = So2Space::new();
        let a = -PI + 0.05; // just past -PI
        let b = PI - 0.05; // just past +PI (the seam-adjacent point)
        let c = 0.1; // on the far side of the circle from the seam
        let d_ab = s.distance(&a, &b);
        let d_ba = s.distance(&b, &a);
        assert!(
            (d_ab - d_ba).abs() < 1e-9,
            "distance(a,b) = {d_ab}, distance(b,a) = {d_ba}"
        );
        let d_ac = s.distance(&a, &c);
        let d_bc = s.distance(&b, &c);
        assert!(
            d_ac <= d_ab + d_bc + 1e-9,
            "triangle inequality violated across the seam: distance(a,c) = {d_ac} > \
             distance(a,b) + distance(b,c) = {}",
            d_ab + d_bc
        );
    }
}
