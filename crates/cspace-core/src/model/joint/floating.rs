// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/floating_joint_model.hpp
//   moveit_core/robot_model/src/floating_joint_model.cpp

use std::f64::consts::PI;

use crate::geometry::quaternion::slerp_coefficients;
use crate::geometry::{Isometry3, UnitQuaternion};
use nalgebra::Quaternion;

use super::bounds::VariableBounds;

/// A floating joint: 6 degrees of freedom, unconstrained translation plus
/// rotation, represented as `[trans_x, trans_y, trans_z, rot_x, rot_y,
/// rot_z, rot_w]` (7 variables — the quaternion is redundant).
///
/// Upstream `moveit::core::FloatingJointModel`.
///
/// # Two invariants worth stating explicitly
///
/// 1. The translation variables are `position_bounded == true` with
///    `min`/`max` `-inf`/`inf` (see [`VariableBounds`]'s doc comment for
///    why that is not a contradiction). Do not derive "is this bounded"
///    from "is the range finite" anywhere downstream of this joint.
/// 2. `rot_x..rot_w` must stay a unit quaternion. Any per-variable operation
///    on just one of the four components (a naive per-variable clamp, a
///    per-variable random perturbation) breaks that invariant.
///    [`FloatingJoint::normalize_rotation`] is the only repair operation;
///    `enforce_position_bounds` always calls it first.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloatingJoint {
    angular_distance_weight: f64,
}

impl Default for FloatingJoint {
    fn default() -> Self {
        Self {
            angular_distance_weight: 1.0,
        }
    }
}

impl FloatingJoint {
    /// The weight applied to the rotational component of `FloatingJoint::distance`
    /// relative to the translational component (which has weight 1).
    pub fn angular_distance_weight(&self) -> f64 {
        self.angular_distance_weight
    }

    /// Set [`FloatingJoint::angular_distance_weight`].
    pub fn set_angular_distance_weight(&mut self, weight: f64) {
        self.angular_distance_weight = weight;
    }

    pub(super) fn default_position(bounds: &[VariableBounds; 7]) -> [f64; 7] {
        let mut values = [0.0; 7];
        for i in 0..3 {
            values[i] = if bounds[i].min_position <= 0.0 && bounds[i].max_position >= 0.0 {
                0.0
            } else {
                (bounds[i].min_position + bounds[i].max_position) / 2.0
            };
        }
        values[6] = 1.0; // identity quaternion: rot_w = 1
        values
    }

    pub(super) fn maximum_extent(&self, bounds: &[VariableBounds; 7]) -> f64 {
        let dx = bounds[0].max_position - bounds[0].min_position;
        let dy = bounds[1].max_position - bounds[1].min_position;
        let dz = bounds[2].max_position - bounds[2].min_position;
        (dx * dx + dy * dy + dz * dz).sqrt() + PI * 0.5 * self.angular_distance_weight
    }

    pub(super) fn satisfies_position_bounds(
        values: &[f64; 7],
        bounds: &[VariableBounds; 7],
        margin: f64,
    ) -> bool {
        for i in 0..3 {
            if values[i] < bounds[i].min_position - margin
                || values[i] > bounds[i].max_position + margin
            {
                return false;
            }
        }
        let norm_sqr: f64 = values[3..7].iter().map(|v| v * v).sum();
        (norm_sqr - 1.0).abs() <= f64::EPSILON * 10.0
    }

    /// Normalize `values[3..7]` (the quaternion) to unit length in place.
    /// If its norm is degenerately close to zero, reset to the identity
    /// quaternion rather than dividing by (near) zero. Upstream
    /// `FloatingJointModel::normalizeRotation`.
    ///
    /// Returns `true` if a change was made.
    pub fn normalize_rotation(values: &mut [f64; 7]) -> bool {
        let norm_sqr: f64 = values[3..7].iter().map(|v| v * v).sum();
        if (norm_sqr - 1.0).abs() <= f64::EPSILON * 100.0 {
            return false;
        }
        let norm = norm_sqr.sqrt();
        if norm < f64::EPSILON * 100.0 {
            values[3] = 0.0;
            values[4] = 0.0;
            values[5] = 0.0;
            values[6] = 1.0;
        } else {
            values[3] /= norm;
            values[4] /= norm;
            values[5] /= norm;
            values[6] /= norm;
        }
        true
    }

    pub(super) fn enforce_position_bounds(
        values: &mut [f64; 7],
        bounds: &[VariableBounds; 7],
    ) -> bool {
        let mut result = Self::normalize_rotation(values);
        for i in 0..3 {
            if values[i] < bounds[i].min_position {
                values[i] = bounds[i].min_position;
                result = true;
            } else if values[i] > bounds[i].max_position {
                values[i] = bounds[i].max_position;
                result = true;
            }
        }
        result
    }

    pub(super) fn distance(&self, values1: &[f64; 7], values2: &[f64; 7]) -> f64 {
        Self::distance_translation(values1, values2)
            + self.angular_distance_weight * Self::distance_rotation(values1, values2)
    }

    /// The translational component of `FloatingJoint::distance`: Euclidean
    /// distance between `values1[0..3]` and `values2[0..3]`.
    pub fn distance_translation(values1: &[f64; 7], values2: &[f64; 7]) -> f64 {
        let dx = values1[0] - values2[0];
        let dy = values1[1] - values2[1];
        let dz = values1[2] - values2[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Upstream normalizes both quaternions before comparing (even though
    /// `satisfies_position_bounds` elsewhere treats a
    /// non-unit quaternion as out of bounds) — reproduced as-is. See
    /// `normalize_quaternion_or_identity`'s doc comment for the zero-input
    /// divergence this guards against.
    pub fn distance_rotation(values1: &[f64; 7], values2: &[f64; 7]) -> f64 {
        let q1 = normalize_quaternion_or_identity(quaternion_from_xyzw(values1));
        let q2 = normalize_quaternion_or_identity(quaternion_from_xyzw(values2));
        q1.angle_to(&q2)
    }

    /// Upstream slerps `from`/`to` directly, without normalizing first
    /// (unlike [`FloatingJoint::distance_rotation`]). Skips the slerp
    /// entirely (copies `from`'s quaternion) when `from` and `to` agree to
    /// within `f64::EPSILON` summed over every component, matching
    /// upstream's guard against the degenerate near-identical case.
    ///
    /// The slerp itself is [`crate::geometry::quaternion::slerp_coefficients`],
    /// a transcription of `Eigen::QuaternionBase::slerp` rather than a call to
    /// `nalgebra`'s `try_slerp`; that function's doc comment names the three
    /// measured divergences that forced the transcription, all of them found
    /// on this joint.
    pub(super) fn interpolate(from: &[f64; 7], to: &[f64; 7], t: f64) -> [f64; 7] {
        let mut state = [0.0; 7];
        for i in 0..3 {
            state[i] = from[i] + (to[i] - from[i]) * t;
        }

        let quat_diff: f64 = (3..7).map(|i| (from[i] - to[i]).abs()).sum();
        if quat_diff > f64::EPSILON {
            state[3..7].copy_from_slice(&slerp_coefficients(
                from[3..7].try_into().expect("four rotation variables"),
                to[3..7].try_into().expect("four rotation variables"),
                t,
            ));
        } else {
            state[3..7].copy_from_slice(&from[3..7]);
        }
        state
    }

    /// See `normalize_quaternion_or_identity`'s doc comment for the
    /// zero-quaternion divergence this guards against.
    pub(super) fn compute_transform(values: &[f64; 7]) -> Isometry3 {
        Isometry3::from_parts(
            nalgebra::Translation3::new(values[0], values[1], values[2]),
            normalize_quaternion_or_identity(quaternion_from_xyzw(values)),
        )
    }

    /// `transform.rotation` is already a `UnitQuaternion` by nalgebra's type
    /// (unlike upstream, which extracts a `Quaterniond` from a general
    /// `Matrix3d` and asserts it is orthonormal) — no renormalization is
    /// possible or necessary here.
    pub(super) fn compute_variable_positions(transform: &Isometry3) -> [f64; 7] {
        let t = transform.translation.vector;
        let q = transform.rotation.quaternion();
        [t.x, t.y, t.z, q.i, q.j, q.k, q.w]
    }
}

fn quaternion_from_xyzw(values: &[f64; 7]) -> Quaternion<f64> {
    Quaternion::new(values[6], values[3], values[4], values[5])
}

/// `Eigen::QuaternionBase::normalized()` routes through
/// `MatrixBase::normalized()`, which guards its zero-norm case (`if (z > 0)
/// ... else return n;`, `Dot.h`) and returns the coefficients unchanged
/// rather than dividing by zero. `UnitQuaternion::new_normalize` has no such
/// guard and divides unconditionally, turning a zero quaternion into `[NaN;
/// 4]`.
///
/// A zero quaternion is reachable here as an everyday input, not a
/// contrivance: a zeroed `[f64; 7]` floating-joint state has `rot_w == 0.0`
/// along with `rot_x/y/z`. Upstream's own `normalizeRotation`
/// (`floating_joint_model.cpp:179-205`) has an explicit arm logging
/// "Quaternion is zero in RobotState representation. Setting to identity"
/// for exactly this case — but `distance_rotation`/`compute_transform`
/// (`distanceRotation`/`computeTransform`, `:131-132`/`:235`) never call it;
/// they use the bare `.normalized()`, so upstream lands on the identity
/// rotation by Eigen's guard alone, not by that logged check.
///
/// Measured directly against this repo's Eigen 3.4.0 oracle image, not
/// inferred: `Quaterniond(0,0,0,0).normalized()` keeps the coefficients
/// `(0,0,0,0)`, and `.toRotationMatrix()` on that is exactly the 3x3
/// identity (every off-diagonal term has an `x`/`y`/`z` factor and vanishes;
/// every diagonal term is `1 - 2*(a^2+b^2)` with `a=b=0`) — so
/// `angularDistance` between two zero quaternions is exactly `0`.
/// `nalgebra`'s `UnitQuaternion` has no zero representation at all (it
/// always carries a unit-norm invariant), so storing the identity here is
/// the closest faithful choice: it reproduces the observable rotation and
/// `angularDistance`/`angle_to` exactly, but not upstream's raw zero
/// coefficients, which this type cannot represent.
fn normalize_quaternion_or_identity(q: Quaternion<f64>) -> UnitQuaternion {
    if q.norm() == 0.0 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::new_normalize(q)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn infinite_translation_bounds() -> [VariableBounds; 7] {
        let translation = VariableBounds {
            min_position: -f64::INFINITY,
            max_position: f64::INFINITY,
            position_bounded: true,
            ..Default::default()
        };
        let quaternion = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            position_bounded: true,
            ..Default::default()
        };
        [
            translation,
            translation,
            translation,
            quaternion,
            quaternion,
            quaternion,
            quaternion,
        ]
    }

    const IDENTITY: [f64; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];

    #[test]
    fn position_bounded_true_with_infinite_range_accepts_any_finite_translation() {
        let bounds = infinite_translation_bounds();
        let values = [1e300, -1e300, 42.0, 0.0, 0.0, 0.0, 1.0];
        assert!(FloatingJoint::satisfies_position_bounds(
            &values, &bounds, 0.0
        ));
    }

    #[test]
    fn satisfies_position_bounds_rejects_non_unit_quaternion() {
        let bounds = infinite_translation_bounds();
        let mut values = IDENTITY;
        values[3] = 0.5; // no longer unit length
        assert!(!FloatingJoint::satisfies_position_bounds(
            &values, &bounds, 0.0
        ));
    }

    #[test]
    fn satisfies_position_bounds_accepts_unit_quaternion() {
        let bounds = infinite_translation_bounds();
        assert!(FloatingJoint::satisfies_position_bounds(
            &IDENTITY, &bounds, 0.0
        ));
    }

    #[test]
    fn normalize_rotation_is_noop_when_already_unit() {
        let mut values = IDENTITY;
        assert!(!FloatingJoint::normalize_rotation(&mut values));
        assert_eq!(values, IDENTITY);
    }

    #[test]
    fn normalize_rotation_rescales_non_unit_quaternion() {
        let mut values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0];
        assert!(FloatingJoint::normalize_rotation(&mut values));
        let norm_sqr: f64 = values[3..7].iter().map(|v| v * v).sum();
        // Every component divides by the exact norm 2.0 -- a power of two,
        // so the division is exact under IEEE 754.
        assert_eq!(norm_sqr, 1.0);
    }

    #[test]
    fn normalize_rotation_resets_to_identity_when_norm_near_zero() {
        let mut values = [0.0, 0.0, 0.0, 1e-30, 0.0, 0.0, 0.0];
        assert!(FloatingJoint::normalize_rotation(&mut values));
        assert_eq!(&values[3..7], &IDENTITY[3..7]);
    }

    #[test]
    fn enforce_position_bounds_normalizes_rotation_even_when_translation_is_in_bounds() {
        let bounds = infinite_translation_bounds();
        let mut values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0];
        assert!(FloatingJoint::enforce_position_bounds(&mut values, &bounds));
        let norm_sqr: f64 = values[3..7].iter().map(|v| v * v).sum();
        // Same 2.0-norm quaternion as `normalize_rotation_rescales_non_unit_quaternion`;
        // exact for the same reason.
        assert_eq!(norm_sqr, 1.0);
    }

    #[test]
    fn interpolate_copies_from_quaternion_when_within_epsilon() {
        let from = IDENTITY;
        let mut to = IDENTITY;
        to[0] = 5.0; // translation differs, quaternion is bit-for-bit identical
        let state = FloatingJoint::interpolate(&from, &to, 0.5);
        assert_eq!(&state[3..7], &from[3..7]);
    }

    #[test]
    fn interpolate_returns_from_for_an_exactly_antipodal_pair() {
        // `to` is `-from`: the same rotation, the opposite sign. The dot
        // product is exactly -1, so Eigen lerps with a negated `scale1` and
        // reconstructs `from` at every `t`. This is the boundary where
        // `nalgebra`'s `try_slerp` gives up (`None`) and the `nlerp` fallback
        // that used to catch it divides a zero-norm sum by its own length at
        // `t = 0.5`.
        let from = IDENTITY;
        let to = [0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0];
        for t in [0.0, 0.5, 1.0] {
            let state = FloatingJoint::interpolate(&from, &to, t);
            assert_eq!(&state[3..7], &from[3..7], "t = {t}");
        }
    }

    /// A zeroed `[f64; 7]` state has a zero quaternion (`rot_w == 0.0`
    /// along with `rot_x/y/z`), reachable as an everyday default, not a
    /// contrivance. `UnitQuaternion::new_normalize` alone would divide by a
    /// zero norm and give `[NaN; 4]`; upstream's `.normalized()` guard keeps
    /// the coefficients and renders as the identity rotation instead. Fails
    /// before `normalize_quaternion_or_identity`'s fix (rotation is NaN),
    /// passes after (exactly the identity).
    #[test]
    fn compute_transform_on_a_zero_quaternion_gives_identity_not_nan() {
        let values = [0.0; 7];
        let transform = FloatingJoint::compute_transform(&values);
        let q = transform.rotation.quaternion();
        assert_eq!((q.i, q.j, q.k, q.w), (0.0, 0.0, 0.0, 1.0));
    }

    /// Same divergence as `compute_transform_on_a_zero_quaternion_gives_identity_not_nan`,
    /// through `distance_rotation` instead: both inputs' zero quaternions
    /// resolve to the identity rotation, so their angular distance is
    /// exactly `0.0`, not NaN.
    #[test]
    fn distance_rotation_between_two_zero_quaternions_is_zero_not_nan() {
        let zero = [0.0; 7];
        assert_eq!(FloatingJoint::distance_rotation(&zero, &zero), 0.0);
    }

    /// Demonstrated opposite: an ordinary non-unit quaternion (`rot_w ==
    /// 2.0`, norm 2.0, not zero) still normalizes to the same rotation it
    /// did before this fix -- `new_normalize` divides by the exact norm 2.0,
    /// landing on the identity exactly under IEEE 754 -- so the zero-norm
    /// guard does not turn every input into a no-op identity.
    #[test]
    fn distance_rotation_still_normalizes_an_ordinary_non_unit_quaternion() {
        let non_unit = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0];
        assert_eq!(FloatingJoint::distance_rotation(&non_unit, &IDENTITY), 0.0);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_positions() {
        let values = [1.0, -2.0, 3.0, 0.5, 0.5, 0.5, 0.5];
        let transform = FloatingJoint::compute_transform(&values);
        let recovered = FloatingJoint::compute_variable_positions(&transform);
        // Measured exact for these inputs; not asserted as a general
        // property of the round trip.
        for (a, b) in values.iter().zip(recovered.iter()) {
            assert_eq!(*a, *b);
        }
    }
}
