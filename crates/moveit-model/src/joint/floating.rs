// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/floating_joint_model.hpp
//   moveit_core/robot_model/src/floating_joint_model.cpp

use std::f64::consts::PI;

use moveit_geometry::{Isometry3, UnitQuaternion};
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
///    [`FloatingJoint::enforce_position_bounds`] always calls it first.
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
    /// The weight applied to the rotational component of [`FloatingJoint::distance`]
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

    /// The translational component of [`FloatingJoint::distance`]: Euclidean
    /// distance between `values1[0..3]` and `values2[0..3]`.
    pub fn distance_translation(values1: &[f64; 7], values2: &[f64; 7]) -> f64 {
        let dx = values1[0] - values2[0];
        let dy = values1[1] - values2[1];
        let dz = values1[2] - values2[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Upstream normalizes both quaternions before comparing (even though
    /// [`FloatingJoint::satisfies_position_bounds`] elsewhere treats a
    /// non-unit quaternion as out of bounds) — reproduced as-is.
    pub fn distance_rotation(values1: &[f64; 7], values2: &[f64; 7]) -> f64 {
        let q1 = UnitQuaternion::new_normalize(quaternion_from_xyzw(values1));
        let q2 = UnitQuaternion::new_normalize(quaternion_from_xyzw(values2));
        q1.angle_to(&q2)
    }

    /// Upstream slerps `from`/`to` directly, without normalizing first
    /// (unlike [`FloatingJoint::distance_rotation`]) — reproduced with
    /// `new_unchecked` rather than `new_normalize`. Skips the slerp
    /// entirely (copies `from`'s quaternion) when `from` and `to` agree to
    /// within `f64::EPSILON` on every component, matching upstream's guard
    /// against the degenerate near-identical case.
    pub(super) fn interpolate(from: &[f64; 7], to: &[f64; 7], t: f64) -> [f64; 7] {
        let mut state = [0.0; 7];
        for i in 0..3 {
            state[i] = from[i] + (to[i] - from[i]) * t;
        }

        let quat_diff: f64 = (3..7).map(|i| (from[i] - to[i]).abs()).sum();
        if quat_diff > f64::EPSILON {
            let q1 = UnitQuaternion::new_unchecked(quaternion_from_xyzw(from));
            let q2 = UnitQuaternion::new_unchecked(quaternion_from_xyzw(to));
            // nalgebra's `slerp` panics when q1/q2 are ~180 degrees apart;
            // Eigen's does not. Falling back to a normalized linear
            // interpolation there avoids a panic upstream never raises.
            let q = q1
                .try_slerp(&q2, t, f64::EPSILON)
                .unwrap_or_else(|| q1.nlerp(&q2, t));
            state[3] = q.i;
            state[4] = q.j;
            state[5] = q.k;
            state[6] = q.w;
        } else {
            state[3] = from[3];
            state[4] = from[4];
            state[5] = from[5];
            state[6] = from[6];
        }
        state
    }

    pub(super) fn compute_transform(values: &[f64; 7]) -> Isometry3 {
        Isometry3::from_parts(
            nalgebra::Translation3::new(values[0], values[1], values[2]),
            UnitQuaternion::new_normalize(quaternion_from_xyzw(values)),
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

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

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
        assert_relative_eq!(norm_sqr, 1.0, epsilon = 1e-12);
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
        assert_relative_eq!(norm_sqr, 1.0, epsilon = 1e-12);
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
    fn interpolate_does_not_panic_on_antipodal_quaternions() {
        // from = identity, to = 180-degree rotation about x: the two
        // quaternions are pi apart, nalgebra's plain slerp panics on this
        // ("ambiguous configuration"); interpolate must fall back instead.
        let from = IDENTITY;
        let to = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let state = FloatingJoint::interpolate(&from, &to, 0.5);
        let norm_sqr: f64 = state[3..7].iter().map(|v| v * v).sum();
        assert_relative_eq!(norm_sqr, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_positions() {
        let values = [1.0, -2.0, 3.0, 0.5, 0.5, 0.5, 0.5];
        let transform = FloatingJoint::compute_transform(&values);
        let recovered = FloatingJoint::compute_variable_positions(&transform);
        for (a, b) in values.iter().zip(recovered.iter()) {
            assert_relative_eq!(*a, *b, epsilon = 1e-9);
        }
    }
}
