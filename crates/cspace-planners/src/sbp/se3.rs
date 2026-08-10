// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `SE(3)`: a floating joint's translation plus orientation.

use std::f64::consts::PI;

use rand::{Rng, RngExt};

use crate::sbp::error::SbpError;
use crate::sbp::sampling::{sample_ball_radius_fraction, sample_unit_vector};
use crate::sbp::space::{RealVectorSpace, StateSpace};

/// A unit quaternion `(w, x, y, z)`.
type Quat = [f64; 4];

fn quat_dot(a: &Quat, b: &Quat) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// Scale `q` to unit length, falling back to the identity rotation for any
/// `q` that has no direction to preserve.
///
/// The fallback is not defensive padding: `Se3State::rotation` is a public
/// field and [`StateSpace::enforce_bounds`]'s contract is to make *any*
/// state satisfy [`StateSpace::satisfies_bounds`], which requires finite
/// components. Without it, `[0.0; 4]` — or a quaternion whose squared norm
/// underflows to zero, or one carrying a `NaN`/infinity — normalizes to
/// all-`NaN`, and since every `NaN` comparison is false the poisoned state
/// then propagates silently through `distance` and `slerp` rather than
/// being rejected. `norm` covers all three cases at once: it is `NaN` for a
/// `NaN` component, infinite for an infinite one, and exactly `0.0` only
/// when no direction survives.
///
/// Resetting to identity rather than erroring matches this port's existing
/// rule for the same input, `FloatingJointModel::normalizeRotation`
/// (`cspace-model`'s `joint::floating::normalize_rotation`), which upstream
/// also resets to identity below its own near-zero threshold.
fn quat_normalize(q: Quat) -> Quat {
    let norm = quat_dot(&q, &q).sqrt();
    if !norm.is_finite() || norm == 0.0 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
}

/// Hamilton product `a * b`: the rotation that applies `b` first, then `a`.
fn quat_mul(a: &Quat, b: &Quat) -> Quat {
    let (w1, x1, y1, z1) = (a[0], a[1], a[2], a[3]);
    let (w2, x2, y2, z2) = (b[0], b[1], b[2], b[3]);
    [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]
}

/// The unit quaternion for a rotation of `angle` radians about `axis`
/// (assumed unit length): the exponential map from `so(3)` to `SO(3)`.
fn exp_map(axis: &[f64], angle: f64) -> Quat {
    let half = angle / 2.0;
    let s = half.sin();
    [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// Angle in `[0, PI]` between the rotations `a` and `b`.
///
/// `a` and `b` are unit quaternions, and a quaternion and its negation
/// represent the same rotation (the double cover of `SO(3)` by the unit
/// sphere `S^3`), so the near representative is chosen first and
/// `rotation_distance(a, b) == rotation_distance(a, -b)` holds for every
/// `a` and `b` by construction rather than by a sign check at each call
/// site.
///
/// Matches upstream `FloatingJointModel::distanceRotation`, which is Eigen's
/// `Quaterniond::angularDistance` — `2 * acos(|a . b|)`, the true `SO(3)`
/// rotation angle (e.g. a 30-degree rotation about any axis is `30 degrees`
/// apart from identity by this measure, not `15`).
///
/// The angle here is computed as `4 * atan2(|a - b|, |a + b|)` rather than
/// that textbook `2 * acos(a . b)` directly, because `acos` has an infinite
/// derivative at `1.0`: a dot product one ULP below it — already as good as
/// normalizing a quaternion can get — yields an angle around `1e-8` rather
/// than `0` through `acos`, so `distance(a, a)` stops being zero. `atan2`
/// reads the same angle off a chord length that is itself small there, and
/// loses nothing: `distance(a, a)` is exactly `0.0`. The two are exactly
/// equal in real arithmetic: for unit vectors `a`, `near`, the half-angle
/// identity `tan(phi / 2) = |a - near| / |a + near|` gives
/// `2 * atan2(|a - near|, |a + near|) = phi = acos(a . near)`, so doubling
/// that again — `4 * atan2(...)` — is exactly `2 * acos(a . near)`. An
/// earlier version of this function used the `2 *` factor from that
/// intermediate identity directly, which silently returned half the real
/// rotation angle everywhere (`distance`, and `sample_near`'s exp-map-radius
/// reasoning below, both consumed that halved value consistently with each
/// other, so no metric axiom caught it — only a boundary check against a
/// rotation of *known* angle did, see this module's
/// `interpolate_takes_the_shorter_arc_even_from_the_far_quaternion_representative`
/// test).
fn rotation_distance(a: &Quat, b: &Quat) -> f64 {
    // The near representative: with a positive dot product, `a - b` is the
    // short chord and `a + b` the long one.
    let near = if quat_dot(a, b) < 0.0 {
        [-b[0], -b[1], -b[2], -b[3]]
    } else {
        *b
    };
    let chord =
        |f: fn(f64, f64) -> f64| (0..4).map(|i| f(a[i], near[i]).powi(2)).sum::<f64>().sqrt();
    4.0 * chord(|x, y| x - y).atan2(chord(|x, y| x + y))
}

/// Spherical linear interpolation, taking the shorter of the two arcs a
/// quaternion pair spans (one via `b`, one via `-b`, both representing the
/// same pair of rotations): unlike [`rotation_distance`], picking the
/// shorter arc here is a legitimate part of what "interpolate" means for a
/// rotation (walking the short way around), not a double-cover workaround,
/// so the sign flip belongs in this function and not in `rotation_distance`.
fn slerp(a: &Quat, mut b: Quat, t: f64) -> Quat {
    let mut dot = quat_dot(a, &b);
    if dot < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
        dot = -dot;
    }
    if dot > 0.9995 {
        // Nearly identical (or antipodal-then-flipped) rotations: sin(theta)
        // in the general formula below is too close to 0 to divide by
        // safely, so fall back to a linear blend, which is an accurate
        // approximation of slerp in this regime anyway.
        let lerped = [
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
            a[2] + t * (b[2] - a[2]),
            a[3] + t * (b[3] - a[3]),
        ];
        return quat_normalize(lerped);
    }
    let theta_0 = dot.acos();
    let theta = theta_0 * t;
    let sin_theta_0 = theta_0.sin();
    let s0 = (theta_0 - theta).sin() / sin_theta_0;
    let s1 = theta.sin() / sin_theta_0;
    [
        s0 * a[0] + s1 * b[0],
        s0 * a[1] + s1 * b[1],
        s0 * a[2] + s1 * b[2],
        s0 * a[3] + s1 * b[3],
    ]
}

/// A point in `SE(3)`: translation plus orientation.
#[derive(Debug, Clone, PartialEq)]
pub struct Se3State {
    /// Position in `R^3`.
    pub translation: [f64; 3],
    /// Orientation as a unit quaternion `(w, x, y, z)`. `q` and `-q`
    /// represent the same rotation; see [`Se3Space`]'s doc comment.
    pub rotation: [f64; 4],
}

/// `R^3 x SO(3)`: a floating joint's state space.
///
/// [`distance`](StateSpace::distance) is `translation_distance +
/// rotation_weight * rotation_distance`: a weighted sum of two independent
/// metrics (Euclidean on the translation, geodesic angle on the rotation),
/// which is itself a metric (a nonnegative weighted sum of metrics satisfies
/// the triangle inequality termwise). `rotation_weight` exists because the
/// two components have different units (metres vs. radians) with no
/// canonical exchange rate between them — matching OMPL's `SE3StateSpace`,
/// which gives its rotation subspace a configurable weight relative to
/// translation for the same reason.
///
/// Orientation is a unit quaternion. `SO(3)` is double-covered by the unit
/// quaternions (`q` and `-q` are the same rotation), so this module's
/// private `rotation_distance` picks the near representative of the pair
/// before measuring: that makes `distance(a, b) == distance(a, b_negated)`
/// hold unconditionally, not via a sign check duplicated at every call site.
#[derive(Debug, Clone, PartialEq)]
pub struct Se3Space {
    translation: RealVectorSpace,
    rotation_weight: f64,
}

impl Se3Space {
    /// Builds a space from per-axis translation `(min, max)` bounds and a
    /// rotation weight (see the struct docs).
    ///
    /// # Errors
    /// [`SbpError::InvalidBound`] if a translation bound is non-finite or
    /// has `min > max`. [`SbpError::InvalidWeight`] if `rotation_weight` is
    /// negative or non-finite.
    pub fn new(
        translation_bounds: [(f64, f64); 3],
        rotation_weight: f64,
    ) -> Result<Self, SbpError> {
        if !rotation_weight.is_finite() || rotation_weight < 0.0 {
            return Err(SbpError::InvalidWeight {
                value: rotation_weight,
            });
        }
        let translation = RealVectorSpace::new(translation_bounds.to_vec())?;
        Ok(Self {
            translation,
            rotation_weight,
        })
    }
}

impl StateSpace for Se3Space {
    type State = Se3State;

    fn dimension(&self) -> usize {
        6
    }

    fn distance(&self, a: &Se3State, b: &Se3State) -> f64 {
        let translation_d = self
            .translation
            .distance(&a.translation.to_vec(), &b.translation.to_vec());
        translation_d + self.rotation_weight * rotation_distance(&a.rotation, &b.rotation)
    }

    fn interpolate(&self, from: &Se3State, to: &Se3State, t: f64) -> Se3State {
        if t <= 0.0 {
            return from.clone();
        }
        if t >= 1.0 {
            return to.clone();
        }
        let translation =
            self.translation
                .interpolate(&from.translation.to_vec(), &to.translation.to_vec(), t);
        Se3State {
            translation: [translation[0], translation[1], translation[2]],
            rotation: slerp(&from.rotation, to.rotation, t),
        }
    }

    fn enforce_bounds(&self, state: &mut Se3State) {
        let mut translation = state.translation.to_vec();
        self.translation.enforce_bounds(&mut translation);
        state.translation = [translation[0], translation[1], translation[2]];
        state.rotation = quat_normalize(state.rotation);
    }

    fn satisfies_bounds(&self, state: &Se3State) -> bool {
        self.translation
            .satisfies_bounds(&state.translation.to_vec())
            && state.rotation.iter().all(|c| c.is_finite())
            && (quat_dot(&state.rotation, &state.rotation) - 1.0).abs() < 1e-6
    }

    fn sample_uniform(&self, rng: &mut dyn Rng) -> Se3State {
        let translation = self.translation.sample_uniform(rng);
        // Independent standard-normal coordinates on S^3, normalized, are
        // uniform on the sphere (see `crate::sbp::sampling::sample_unit_vector`);
        // under the 2:1 covering map from S^3 to SO(3) that is exactly Haar
        // measure on SO(3), the natural notion of "uniform rotation".
        let rotation = sample_unit_vector(rng, 4);
        Se3State {
            translation: [translation[0], translation[1], translation[2]],
            rotation: [rotation[0], rotation[1], rotation[2], rotation[3]],
        }
    }

    fn sample_near(&self, rng: &mut dyn Rng, center: &Se3State, radius: f64) -> Se3State {
        if radius <= 0.0 {
            let mut state = center.clone();
            self.enforce_bounds(&mut state);
            return state;
        }
        // distance is translation_distance + rotation_weight *
        // rotation_distance: a weighted sum of two independent metrics. A
        // uniform split of the radius budget -- up to `u * radius` for
        // translation, the rest for the weighted rotation term -- keeps
        // their sum within `radius` for any split `u`, so drawing `u`
        // uniformly at random and honouring it exactly on each side is
        // sufficient (this is the two-part case of a Dirichlet/simplex
        // budget split; with only two parts it reduces to a single uniform
        // draw and its complement).
        let u: f64 = rng.random_range(0.0..=1.0);
        let translation_radius = u * radius;
        let rotation_budget = (1.0 - u) * radius;

        let translation =
            self.translation
                .sample_near(rng, &center.translation.to_vec(), translation_radius);

        let rotation = if self.rotation_weight <= 0.0 {
            // A weight of 0 means rotation contributes nothing to distance,
            // so any rotation is within budget; sample freely instead of
            // dividing by zero.
            let v = sample_unit_vector(rng, 4);
            [v[0], v[1], v[2], v[3]]
        } else {
            // Draw an angle-axis offset volume-uniformly within the
            // tangent-space ball of radius `rotation_radius` (reusing the
            // same n-ball technique as `RealVectorSpace::sample_near`, at
            // dim 3) and apply it via the exponential map. This is uniform
            // in the tangent space, not exactly Haar-uniform on SO(3) (the
            // exponential map's volume element is `(2*sin(theta/2)/theta)^2`,
            // not constant) but it is exact on the property this method
            // must guarantee: composing a rotation of angle `theta` with
            // `center` lands exactly `theta` away under the bi-invariant
            // geodesic metric, regardless of axis or `center`, so the
            // result never exceeds `rotation_radius <= rotation_budget /
            // rotation_weight`.
            let rotation_radius = (rotation_budget / self.rotation_weight).min(PI);
            let angle = rotation_radius * sample_ball_radius_fraction(rng, 3);
            let axis = sample_unit_vector(rng, 3);
            let offset = exp_map(&axis, angle);
            quat_normalize(quat_mul(&offset, &center.rotation))
        };

        Se3State {
            translation: [translation[0], translation[1], translation[2]],
            rotation,
        }
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

    fn space() -> Se3Space {
        Se3Space::new([(-10.0, 10.0), (-10.0, 10.0), (-10.0, 10.0)], 1.0).unwrap()
    }

    #[test]
    fn negative_weight_is_rejected() {
        assert_eq!(
            Se3Space::new([(-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)], -1.0),
            Err(SbpError::InvalidWeight { value: -1.0 })
        );
    }

    #[test]
    fn non_finite_weight_is_rejected() {
        // Not assert_eq!: NaN != NaN, so comparing the whole Result would
        // fail even on the correct error variant.
        match Se3Space::new([(-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)], f64::NAN) {
            Err(SbpError::InvalidWeight { value }) => assert!(value.is_nan()),
            other => panic!("expected Err(InvalidWeight {{ value: NaN }}), got {other:?}"),
        }
    }

    /// Matches upstream `FloatingJointModel::distanceTranslation`
    /// (`floating_joint_model.cpp:120-126`), `sqrt(dx^2 + dy^2 + dz^2)`, at a
    /// Pythagorean quadruple (`3, 4, 12 -> 13`) so the expected value is
    /// exact rather than a value this same implementation produced. Both
    /// states share a rotation, so `rotation_distance`'s contribution is
    /// exactly `0.0` and does not need to be subtracted out.
    #[test]
    fn translation_distance_matches_upstream_floating_joint_translation_at_a_known_value() {
        let s = Se3Space::new([(-20.0, 20.0), (-20.0, 20.0), (-20.0, 20.0)], 1.0).unwrap();
        let identity: Quat = [1.0, 0.0, 0.0, 0.0];
        let from = Se3State {
            translation: [0.0, 0.0, 0.0],
            rotation: identity,
        };
        let to = Se3State {
            translation: [3.0, 4.0, 12.0],
            rotation: identity,
        };
        assert_eq!(s.distance(&from, &to), 13.0);
    }

    /// Matches upstream `FloatingJointModel::distance` (`floating_joint_model.cpp:115-118`),
    /// `distanceTranslation + angular_distance_weight_ * distanceRotation`,
    /// with both terms nonzero and a non-unit weight, so a bug that dropped
    /// or misweighted either term would show up against a value computed
    /// independently of this implementation: translation `(3, 4, 0) -> 5.0`
    /// (a 3-4-5 triangle), rotation exactly `60` degrees about the z-axis
    /// (`exp_map`, this module's own SO(3) exponential map, not
    /// `Se3Space::distance` -- the value under test), `rotation_weight = 2.0`.
    #[test]
    fn distance_matches_upstream_floating_joint_weighted_sum_at_a_known_value() {
        let s = Se3Space::new([(-20.0, 20.0), (-20.0, 20.0), (-20.0, 20.0)], 2.0).unwrap();
        let from = Se3State {
            translation: [0.0, 0.0, 0.0],
            rotation: [1.0, 0.0, 0.0, 0.0],
        };
        let to = Se3State {
            translation: [3.0, 4.0, 0.0],
            rotation: exp_map(&[0.0, 0.0, 1.0], 60.0_f64.to_radians()),
        };
        let expected = 5.0 + 2.0 * 60.0_f64.to_radians();
        let actual = s.distance(&from, &to);
        assert!(
            (actual - expected).abs() < 1e-9,
            "distance = {actual}, expected translation 5.0 + weight 2.0 * 60 degrees = {expected}"
        );
    }

    #[test]
    fn distance_ignores_quaternion_sign() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        for _ in 0..500 {
            let a = s.sample_uniform(&mut rng);
            let b = s.sample_uniform(&mut rng);
            let mut b_negated = b.clone();
            b_negated.rotation = [
                -b.rotation[0],
                -b.rotation[1],
                -b.rotation[2],
                -b.rotation[3],
            ];
            let d = s.distance(&a, &b);
            let d_negated = s.distance(&a, &b_negated);
            assert!(
                (d - d_negated).abs() < 1e-9,
                "distance(a, b) = {d}, distance(a, -b) = {d_negated}"
            );
        }
    }

    #[test]
    fn rotation_distance_between_identical_rotations_is_zero() {
        // Exactly zero, not "within some epsilon": `rotation_distance`'s
        // atan2 form reads a small angle off a small chord, so there is no
        // precision floor to leave room for. An earlier `2 * acos(|dot|)`
        // needed 1e-6 here.
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(2);
        for _ in 0..500 {
            let a = s.sample_uniform(&mut rng);
            assert_eq!(s.distance(&a, &a), 0.0);
        }
    }

    #[test]
    fn interpolate_endpoints_are_exact() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let a = s.sample_uniform(&mut rng);
        let b = s.sample_uniform(&mut rng);
        assert_eq!(s.interpolate(&a, &b, 0.0), a);
        assert_eq!(s.interpolate(&a, &b, 1.0), b);
    }

    #[test]
    fn enforce_bounds_renormalizes_rotation() {
        let s = space();
        let mut state = Se3State {
            translation: [0.0, 0.0, 0.0],
            rotation: [2.0, 0.0, 0.0, 0.0],
        };
        s.enforce_bounds(&mut state);
        assert!((quat_dot(&state.rotation, &state.rotation) - 1.0).abs() < 1e-9);
    }

    /// `enforce_bounds` must leave `satisfies_bounds` true for *every* input,
    /// including rotations no scaling can turn into a unit quaternion.
    /// `Se3State`'s fields are public, and `sample_near` with a non-positive
    /// radius hands its caller's `center` straight to `enforce_bounds`, so
    /// these are reachable without any unsafe or private access.
    #[test]
    fn enforce_bounds_on_an_unnormalizable_rotation_still_satisfies_bounds() {
        let s = space();
        for rotation in [
            [0.0, 0.0, 0.0, 0.0],
            [f64::NAN, 0.0, 0.0, 0.0],
            [f64::INFINITY, 0.0, 0.0, 0.0],
        ] {
            let mut state = Se3State {
                translation: [0.0, 0.0, 0.0],
                rotation,
            };
            s.enforce_bounds(&mut state);
            assert!(
                s.satisfies_bounds(&state),
                "enforce_bounds({rotation:?}) left {:?}, which satisfies_bounds rejects",
                state.rotation
            );
        }
    }

    #[test]
    fn metric_and_interpolation_axioms_hold() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(4);
        assert_metric_and_interpolation_axioms(
            &s,
            &mut rng,
            |rng| s.sample_uniform(rng),
            2000,
            1e-6,
        );
    }

    #[test]
    fn sample_near_stays_within_radius() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(5);
        for &radius in &[0.01, 0.5, 2.0, 10.0] {
            let center = s.sample_uniform(&mut rng);
            assert_sample_near_stays_within_radius(&s, &mut rng, &center, radius, 500, 1e-9);
        }
    }

    #[test]
    fn sample_near_zero_weight_ignores_rotation_budget() {
        let s = Se3Space::new([(-10.0, 10.0), (-10.0, 10.0), (-10.0, 10.0)], 0.0).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(6);
        let center = s.sample_uniform(&mut rng);
        assert_sample_near_stays_within_radius(&s, &mut rng, &center, 1.0, 500, 1e-9);
    }

    // PORTING-PLAN.md:1269 records that whether the StateSpace trait can
    // carry SO(3) has "so far been a comment's assertion, never verified".
    // `distance_ignores_quaternion_sign` above already checks `distance`;
    // the boundary `distance` alone cannot exercise is `interpolate`'s own
    // internal sign correction (`slerp`'s `dot < 0` branch) — constructed
    // cases, not the random pairs `metric_and_interpolation_axioms_hold`
    // below draws (a random quaternion pair does land on both sides of
    // `dot == 0` over 2000 draws, but incidentally; these pin the specific
    // near/far representative pair down explicitly).

    /// A small, known rotation (30 degrees about the x-axis) given as its
    /// *far* quaternion representative (`-b`, `quat_dot(identity, -b) < 0`).
    /// `slerp` must still walk the 30-degree arc, not interpret `-b` as a
    /// near-180-degree-away target and take the long way around.
    #[test]
    fn interpolate_takes_the_shorter_arc_even_from_the_far_quaternion_representative() {
        let s = space();
        let identity: Quat = [1.0, 0.0, 0.0, 0.0];
        let near = exp_map(&[1.0, 0.0, 0.0], 30.0_f64.to_radians());
        assert!(
            quat_dot(&identity, &near) > 0.0,
            "test setup: `near` should already be identity's near representative"
        );
        let far = [-near[0], -near[1], -near[2], -near[3]];
        assert!(
            quat_dot(&identity, &far) < 0.0,
            "test setup: `far` should be identity's far representative"
        );

        let from = Se3State {
            translation: [0.0, 0.0, 0.0],
            rotation: identity,
        };
        let to = Se3State {
            translation: [0.0, 0.0, 0.0],
            rotation: far,
        };

        // The true angular separation is 30 degrees (rotation_distance
        // already picks the near representative), and the halfway point
        // must be 15 degrees along that arc — not the ~165-degree halfway
        // point a naive quaternion-space lerp of `identity` to `far` would
        // produce by going through the long way.
        let whole = s.distance(&from, &to);
        assert!(
            (whole - 30.0_f64.to_radians()).abs() < 1e-9,
            "distance(identity, far) = {whole}, expected 30 degrees"
        );
        let mid = s.interpolate(&from, &to, 0.5);
        let d_from_mid = rotation_distance(&identity, &mid.rotation);
        assert!(
            (d_from_mid - 15.0_f64.to_radians()).abs() < 1e-6,
            "rotation_distance(identity, midpoint) = {d_from_mid} rad, expected 15 degrees \
             (interpolate took the long way around instead of the short one)"
        );
    }

    /// `to` and `-to` represent the same target rotation, so interpolating
    /// toward either from the same `from` must trace the same physical path
    /// at every `t` — a trait that treated the quaternion as an opaque
    /// coordinate vector (no sign correction) would produce two different
    /// paths for what is the same request.
    #[test]
    fn interpolate_is_invariant_to_which_quaternion_represents_the_target() {
        let s = space();
        let mut rng = ChaCha8Rng::seed_from_u64(7);
        for _ in 0..200 {
            let from = s.sample_uniform(&mut rng);
            let to = s.sample_uniform(&mut rng);
            let mut to_negated = to.clone();
            to_negated.rotation = [
                -to.rotation[0],
                -to.rotation[1],
                -to.rotation[2],
                -to.rotation[3],
            ];
            for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
                let via_to = s.interpolate(&from, &to, t);
                let via_negated = s.interpolate(&from, &to_negated, t);
                let d = rotation_distance(&via_to.rotation, &via_negated.rotation);
                assert!(
                    d < 1e-6,
                    "interpolate(from, to, {t}) and interpolate(from, -to, {t}) diverge by \
                     {d} rad in rotation; t={t}, from={from:?}, to={to:?}"
                );
            }
        }
    }
}
