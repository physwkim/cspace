// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/revolute_joint_model.hpp
//   moveit_core/robot_model/src/revolute_joint_model.cpp

use std::f64::consts::PI;

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};

use super::bounds::VariableBounds;

/// A revolute joint: one degree of freedom, rotation about a fixed axis.
///
/// Upstream `moveit::core::RevoluteJointModel`. The axis and the `continuous`
/// flag live here; the joint's single [`VariableBounds`] lives in the
/// owning [`crate::joint::JointModel::variable_bounds`], because
/// `set_continuous` (upstream `setContinuous`) mutates that bound as a side
/// effect and both must move together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJoint {
    axis: Vector3,
    continuous: bool,
}

impl Default for RevoluteJoint {
    /// Matches upstream's constructor: zero axis, not continuous. A zero
    /// axis is degenerate (as it is upstream — `Eigen::Vector3d::normalized()`
    /// on a zero vector is also NaN); callers are expected to call
    /// [`RevoluteJoint::set_axis`] before use, mirroring upstream's
    /// construct-then-`setAxis` sequence.
    fn default() -> Self {
        Self {
            axis: Vector3::zeros(),
            continuous: false,
        }
    }
}

impl RevoluteJoint {
    /// The axis of rotation, always unit length (or NaN if never set from a
    /// non-degenerate vector — see [`RevoluteJoint::default`]).
    pub fn axis(&self) -> Vector3 {
        self.axis
    }

    /// Set the axis of rotation. Upstream `RevoluteJointModel::setAxis`,
    /// which normalizes.
    pub fn set_axis(&mut self, axis: Vector3) {
        self.axis = axis.normalize();
    }

    /// Whether this joint wraps around (no position limit, `interpolate` and
    /// `distance` take the shorter way around the circle).
    pub fn is_continuous(&self) -> bool {
        self.continuous
    }

    /// Set the `continuous` flag directly, without touching bounds.
    ///
    /// Only [`crate::joint::JointModel::set_continuous`] calls this — it
    /// additionally mutates the joint's [`VariableBounds`], which live on
    /// the owning `JointModel`, not here (see this type's doc comment).
    pub(super) fn set_continuous_flag(&mut self, flag: bool) {
        self.continuous = flag;
    }

    pub(super) fn default_position(bounds: &VariableBounds) -> f64 {
        if bounds.min_position <= 0.0 && bounds.max_position >= 0.0 {
            0.0
        } else {
            (bounds.min_position + bounds.max_position) / 2.0
        }
    }

    pub(super) fn maximum_extent(bounds: &VariableBounds) -> f64 {
        bounds.max_position - bounds.min_position
    }

    pub(super) fn satisfies_position_bounds(
        &self,
        value: f64,
        bounds: &VariableBounds,
        margin: f64,
    ) -> bool {
        if self.continuous {
            true
        } else {
            value >= bounds.min_position - margin && value <= bounds.max_position + margin
        }
    }

    /// Bring `*value` into `[-pi, pi]` by adding/subtracting multiples of
    /// `2*pi`, for a continuous joint; clamp to `bounds` otherwise. Always
    /// returns `true`, matching upstream (which returns an unconditional
    /// `true` for this joint type, unlike the other joint kinds).
    pub(super) fn enforce_position_bounds(&self, value: &mut f64, bounds: &VariableBounds) -> bool {
        if self.continuous {
            if *value <= -PI || *value > PI {
                *value %= 2.0 * PI;
                if *value <= -PI {
                    *value += 2.0 * PI;
                } else if *value > PI {
                    *value -= 2.0 * PI;
                }
            }
        } else if *value < bounds.min_position {
            *value = bounds.min_position;
        } else if *value > bounds.max_position {
            *value = bounds.max_position;
        }
        true
    }

    /// Add/subtract multiples of `2*pi` to bring `*value` back into
    /// `bounds`. Upstream applies this regardless of the `continuous` flag —
    /// it operates purely on `bounds`, so it is a no-op whenever `*value`
    /// is already inside them.
    pub(super) fn harmonize_position(value: &mut f64, bounds: &VariableBounds) -> bool {
        let mut modified = false;
        if *value < bounds.min_position {
            while *value + 2.0 * PI <= bounds.max_position {
                *value += 2.0 * PI;
                modified = true;
            }
        } else if *value > bounds.max_position {
            while *value - 2.0 * PI >= bounds.min_position {
                *value -= 2.0 * PI;
                modified = true;
            }
        }
        modified
    }

    pub(super) fn interpolate(&self, from: f64, to: f64, t: f64) -> f64 {
        if self.continuous {
            let diff = to - from;
            if diff.abs() <= PI {
                from + diff * t
            } else {
                let diff = if diff > 0.0 {
                    2.0 * PI - diff
                } else {
                    -2.0 * PI - diff
                };
                let mut state = from - diff * t;
                if state > PI {
                    state -= 2.0 * PI;
                } else if state < -PI {
                    state += 2.0 * PI;
                }
                state
            }
        } else {
            from + (to - from) * t
        }
    }

    pub(super) fn distance(&self, value1: f64, value2: f64) -> f64 {
        if self.continuous {
            let d = (value1 - value2).abs() % (2.0 * PI);
            if d > PI { 2.0 * PI - d } else { d }
        } else {
            (value1 - value2).abs()
        }
    }

    /// Rotation by `value` about [`RevoluteJoint::axis`].
    ///
    /// Upstream hand-expands the Rodrigues rotation matrix into the
    /// isometry's raw column-major storage (with the simpler
    /// `Eigen::Isometry3d(Eigen::AngleAxisd(value, axis_))` form left in a
    /// comment — evidently optimized away for a hot FK path). This port
    /// uses that simpler, equivalent form: nalgebra's axis-angle
    /// construction, not hand-rolled matrix coefficients.
    pub(super) fn compute_transform(&self, value: f64) -> Isometry3 {
        Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_unchecked(self.axis), value),
        )
    }

    /// Recover the rotation angle about [`RevoluteJoint::axis`] from a
    /// transform produced by [`RevoluteJoint::compute_transform`].
    ///
    /// Upstream picks the axis component with the largest absolute value to
    /// avoid dividing by a near-zero component; ties are broken by whichever
    /// component `Iterator::max_by` keeps (upstream's `Eigen::maxCoeff` does
    /// not document its tie-break either). Every axis in the panda and
    /// fanuc fixtures is a single unit basis vector, so no tie is possible
    /// there.
    pub(super) fn compute_variable_position(&self, transform: &Isometry3) -> f64 {
        let q = transform.rotation.quaternion();
        let components = [(self.axis.x, q.i), (self.axis.y, q.j), (self.axis.z, q.k)];
        let (axis_val, q_val) = components
            .into_iter()
            .max_by(|a, b| a.0.abs().partial_cmp(&b.0.abs()).unwrap())
            .expect("axis has three components");
        2.0 * (q_val / axis_val).atan2(q.w)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn bounded() -> (RevoluteJoint, VariableBounds) {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 1.0));
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            position_bounded: true,
            ..Default::default()
        };
        (joint, bounds)
    }

    fn continuous() -> (RevoluteJoint, VariableBounds) {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 1.0));
        joint.continuous = true;
        let bounds = VariableBounds {
            min_position: -PI,
            max_position: PI,
            ..Default::default()
        };
        (joint, bounds)
    }

    #[test]
    fn set_axis_normalizes() {
        let mut joint = RevoluteJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 5.0));
        // normalize() divides an axis-aligned vector by its own exact norm
        // (5.0), giving (0.0, 0.0, 1.0) exactly; its norm is sqrt(1.0) = 1.0
        // exactly under IEEE 754 -- a structural identity, not a value
        // measured for this input alone.
        assert_eq!(joint.axis().norm(), 1.0);
    }

    #[test]
    fn satisfies_position_bounds_at_and_outside_boundary_when_bounded() {
        let (joint, bounds) = bounded();
        assert!(joint.satisfies_position_bounds(1.0, &bounds, 0.0));
        assert!(!joint.satisfies_position_bounds(1.0 + f64::EPSILON * 4.0, &bounds, 0.0));
        assert!(joint.satisfies_position_bounds(1.0 + 0.5, &bounds, 0.5));
    }

    #[test]
    fn satisfies_position_bounds_ignores_bounds_when_continuous() {
        let (joint, bounds) = continuous();
        assert!(joint.satisfies_position_bounds(1000.0, &bounds, 0.0));
    }

    #[test]
    fn enforce_position_bounds_clamps_when_bounded() {
        let (joint, bounds) = bounded();
        let mut value = 5.0;
        assert!(joint.enforce_position_bounds(&mut value, &bounds));
        assert_eq!(value, 1.0);
    }

    #[test]
    fn enforce_position_bounds_wraps_when_continuous() {
        let (joint, bounds) = continuous();
        let mut value = PI + 0.5;
        joint.enforce_position_bounds(&mut value, &bounds);
        // `(PI + 0.5) - 2*PI + 0.5`-shaped wraparound measured exact for this
        // input; not asserted as a general property of `PI` arithmetic.
        assert_eq!(value, -PI + 0.5);
    }

    #[test]
    fn enforce_position_bounds_leaves_value_at_exactly_pi_when_continuous() {
        let (joint, bounds) = continuous();
        let mut value = PI;
        joint.enforce_position_bounds(&mut value, &bounds);
        assert_eq!(value, PI);
    }

    #[test]
    fn harmonize_position_wraps_regardless_of_continuous_flag() {
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let mut value = -1.0 - 2.0 * PI;
        assert!(RevoluteJoint::harmonize_position(&mut value, &bounds));
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(value, -1.0);
    }

    #[test]
    fn harmonize_position_is_noop_inside_bounds() {
        let bounds = VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let mut value = 0.5;
        assert!(!RevoluteJoint::harmonize_position(&mut value, &bounds));
        assert_eq!(value, 0.5);
    }

    #[test]
    fn interpolate_wraps_the_short_way_when_continuous() {
        let (joint, _bounds) = continuous();
        // From just past +pi to just past -pi the short way is forward through pi,
        // not backward across zero.
        let state = joint.interpolate(PI - 0.1, -PI + 0.1, 0.5);
        // Measured exact for this input; not asserted as a general property.
        assert_eq!(state.abs(), PI);
    }

    #[test]
    fn interpolate_is_linear_when_bounded() {
        let (joint, _bounds) = bounded();
        // Non-continuous branch is `from + (to - from) * t`; 0.0 + (1.0 -
        // 0.0) * 0.5 = 0.5 exactly under IEEE 754, not a value measured for
        // this input alone.
        assert_eq!(joint.interpolate(0.0, 1.0, 0.5), 0.5);
    }

    #[test]
    fn distance_takes_the_short_way_when_continuous() {
        let (joint, _bounds) = continuous();
        // The short-way distance goes through a modulo-based wrap, which
        // leaves a 1-ULP residue here (0.20000000000000018 vs 0.2) rather
        // than landing on the literal exactly.
        assert_relative_eq!(
            joint.distance(-PI + 0.1, PI - 0.1),
            0.2,
            epsilon = 1e-15,
            max_relative = 0.0
        );
    }

    #[test]
    fn distance_is_linear_when_bounded() {
        let (joint, _bounds) = bounded();
        // Non-continuous branch is `(value1 - value2).abs()`; (-1.0 -
        // 1.0).abs() = 2.0 exactly under IEEE 754, not a value measured for
        // this input alone.
        assert_eq!(joint.distance(-1.0, 1.0), 2.0);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_position() {
        let (joint, _bounds) = bounded();
        for value in [-0.75_f64, 0.0, 0.9] {
            let transform = joint.compute_transform(value);
            let recovered = joint.compute_variable_position(&transform);
            // Measured exact for these inputs; not asserted as a general
            // property of the round trip.
            assert_eq!(recovered, value);
        }
    }
}
