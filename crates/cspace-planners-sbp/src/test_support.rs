// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Property-test helpers shared by every `StateSpace` implementation's test
//! module, so the metric-axiom and interpolation checks are written once
//! rather than copy-pasted (and drifting) per space.

use std::fmt::Debug;

use rand::{Rng, RngExt};

use crate::space::StateSpace;

/// Asserts the metric axioms (non-negativity, identity, symmetry, triangle
/// inequality) and the interpolation contract (`interpolate(a, b, 0) == a`,
/// `interpolate(a, b, 1) == b`, `distance(a, interpolate(a, b, t)) ~= t *
/// distance(a, b)`) over `pairs` random `(a, b, c, t)` draws from
/// `random_state`.
///
/// `tolerance` is the floating-point slack for the symmetry, triangle and
/// interpolation-distance checks; a space whose `interpolate` is not exactly
/// affine in `distance` (a rotation's slerp, for instance) may need a looser
/// tolerance than `RealVectorSpace`'s exact linear blend. Identity is
/// asserted exactly and takes no slack from it.
pub(crate) fn assert_metric_and_interpolation_axioms<S: StateSpace>(
    space: &S,
    rng: &mut dyn Rng,
    mut random_state: impl FnMut(&mut dyn Rng) -> S::State,
    pairs: usize,
    tolerance: f64,
) where
    S::State: PartialEq + Debug,
{
    for _ in 0..pairs {
        let a = random_state(rng);
        let b = random_state(rng);
        let c = random_state(rng);

        // Exactly zero, deliberately not within `tolerance`: identity is the
        // one axiom with no reason to be approximate, and every space here
        // can hold it by construction. `tolerance` exists for the triangle
        // inequality and for interpolation that is not exactly affine in
        // distance (slerp); folding identity into that same slack is what
        // let `Se3Space` ship a `2 * acos(|dot|)` rotation distance whose
        // `distance(a, a)` was 6.7e-8.
        assert_eq!(
            space.distance(&a, &a),
            0.0,
            "distance(a, a) is not zero, a = {a:?}"
        );

        let d_ab = space.distance(&a, &b);
        let d_ba = space.distance(&b, &a);
        assert!(
            d_ab >= 0.0,
            "distance(a, b) = {d_ab} is negative, a = {a:?}, b = {b:?}"
        );
        assert!(
            (d_ab - d_ba).abs() < tolerance,
            "distance not symmetric: distance(a,b) = {d_ab}, distance(b,a) = {d_ba}, \
             a = {a:?}, b = {b:?}"
        );

        let d_ac = space.distance(&a, &c);
        let d_bc = space.distance(&b, &c);
        assert!(
            d_ac <= d_ab + d_bc + tolerance,
            "triangle inequality violated: distance(a,c) = {d_ac} > \
             distance(a,b) + distance(b,c) = {} (a = {a:?}, b = {b:?}, c = {c:?})",
            d_ab + d_bc
        );

        assert_eq!(
            space.interpolate(&a, &b, 0.0),
            a,
            "interpolate(a, b, 0.0) != a"
        );
        assert_eq!(
            space.interpolate(&a, &b, 1.0),
            b,
            "interpolate(a, b, 1.0) != b"
        );

        let t = rng.random_range(0.0..=1.0);
        let mid = space.interpolate(&a, &b, t);
        let d_a_mid = space.distance(&a, &mid);
        let expected = t * d_ab;
        assert!(
            (d_a_mid - expected).abs() < tolerance,
            "distance(a, interpolate(a,b,t)) = {d_a_mid}, expected ~{expected} \
             (t = {t}, distance(a,b) = {d_ab}, a = {a:?}, b = {b:?})"
        );
    }
}

/// Asserts `sample_near` stays within `radius` of `center` under `space`'s
/// own metric, over `draws` random draws.
pub(crate) fn assert_sample_near_stays_within_radius<S: StateSpace>(
    space: &S,
    rng: &mut dyn Rng,
    center: &S::State,
    radius: f64,
    draws: usize,
    tolerance: f64,
) {
    for _ in 0..draws {
        let sample = space.sample_near(rng, center, radius);
        let d = space.distance(center, &sample);
        assert!(
            d <= radius + tolerance,
            "sample_near returned a state at distance {d} from centre, > radius {radius}"
        );
    }
}
