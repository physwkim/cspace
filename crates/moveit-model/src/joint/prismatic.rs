// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/prismatic_joint_model.hpp
//   moveit_core/robot_model/src/prismatic_joint_model.cpp

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};

use super::bounds::VariableBounds;

/// A prismatic joint: one degree of freedom, translation along a fixed axis.
///
/// Upstream `moveit::core::PrismaticJointModel`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrismaticJoint {
    axis: Vector3,
}

impl Default for PrismaticJoint {
    fn default() -> Self {
        Self {
            axis: Vector3::zeros(),
        }
    }
}

impl PrismaticJoint {
    /// The axis of translation. Not necessarily unit length: upstream
    /// `setAxis` stores it as given, unlike [`super::RevoluteJoint::set_axis`]
    /// which normalizes.
    pub fn axis(&self) -> Vector3 {
        self.axis
    }

    /// Set the axis of translation. Stored exactly as given — see
    /// [`PrismaticJoint::axis`].
    pub fn set_axis(&mut self, axis: Vector3) {
        self.axis = axis;
    }

    pub(super) fn default_position(bounds: &VariableBounds) -> f64 {
        if bounds.min_position <= 0.0 && bounds.max_position >= 0.0 {
            0.0
        } else {
            (bounds.min_position + bounds.max_position) / 2.0
        }
    }

    /// Upstream: `own_bounds.max_position - other_bounds.min_position`. This
    /// mixes the joint's *own* default bounds with the *caller-supplied*
    /// `other_bounds`'s minimum — asymmetric on purpose or not, that is
    /// upstream's literal formula and this port reproduces it rather than
    /// second-guessing it.
    pub(super) fn maximum_extent(
        own_bounds: &VariableBounds,
        other_bounds: &VariableBounds,
    ) -> f64 {
        own_bounds.max_position - other_bounds.min_position
    }

    pub(super) fn satisfies_position_bounds(
        value: f64,
        bounds: &VariableBounds,
        margin: f64,
    ) -> bool {
        value >= bounds.min_position - margin && value <= bounds.max_position + margin
    }

    pub(super) fn enforce_position_bounds(value: &mut f64, bounds: &VariableBounds) -> bool {
        if *value < bounds.min_position {
            *value = bounds.min_position;
            true
        } else if *value > bounds.max_position {
            *value = bounds.max_position;
            true
        } else {
            false
        }
    }

    pub(super) fn distance(value1: f64, value2: f64) -> f64 {
        (value1 - value2).abs()
    }

    pub(super) fn interpolate(from: f64, to: f64, t: f64) -> f64 {
        from + (to - from) * t
    }

    pub(super) fn compute_transform(&self, value: f64) -> Isometry3 {
        Isometry3::from_parts(
            nalgebra::Translation3::from(self.axis * value),
            UnitQuaternion::identity(),
        )
    }

    pub(super) fn compute_variable_position(&self, transform: &Isometry3) -> f64 {
        transform.translation.vector.dot(&self.axis)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn bounds() -> VariableBounds {
        VariableBounds {
            min_position: -1.0,
            max_position: 2.0,
            position_bounded: true,
            ..Default::default()
        }
    }

    #[test]
    fn set_axis_does_not_normalize() {
        let mut joint = PrismaticJoint::default();
        joint.set_axis(Vector3::new(0.0, 0.0, 5.0));
        // Prismatic does not normalize, so this is sqrt(0^2+0^2+5.0^2) =
        // sqrt(25.0) = 5.0 exactly under IEEE 754's correctly-rounded sqrt --
        // a structural identity, not a value measured for this input alone.
        assert_eq!(joint.axis().norm(), 5.0);
    }

    #[test]
    fn satisfies_position_bounds_at_and_outside_boundary() {
        let bounds = bounds();
        assert!(PrismaticJoint::satisfies_position_bounds(2.0, &bounds, 0.0));
        assert!(!PrismaticJoint::satisfies_position_bounds(
            2.0 + f64::EPSILON * 4.0,
            &bounds,
            0.0
        ));
        assert!(PrismaticJoint::satisfies_position_bounds(2.5, &bounds, 0.5));
    }

    #[test]
    fn enforce_position_bounds_clamps_only_when_outside() {
        let bounds = bounds();
        let mut value = 5.0;
        assert!(PrismaticJoint::enforce_position_bounds(&mut value, &bounds));
        assert_eq!(value, 2.0);

        let mut value = 1.0;
        assert!(!PrismaticJoint::enforce_position_bounds(
            &mut value, &bounds
        ));
        assert_eq!(value, 1.0);
    }

    #[test]
    fn maximum_extent_mixes_own_max_with_other_min() {
        let own = VariableBounds {
            max_position: 5.0,
            ..Default::default()
        };
        let other = VariableBounds {
            min_position: -3.0,
            ..Default::default()
        };
        // `own.max_position - other.min_position` = 5.0 - (-3.0) = 8.0
        // exactly under IEEE 754, not a value measured for this input alone.
        assert_eq!(PrismaticJoint::maximum_extent(&own, &other), 8.0);
    }

    #[test]
    fn compute_transform_round_trips_through_compute_variable_position() {
        let mut joint = PrismaticJoint::default();
        joint.set_axis(Vector3::new(0.0, 1.0, 0.0));
        for value in [-1.5_f64, 0.0, 2.0] {
            let transform = joint.compute_transform(value);
            assert_relative_eq!(
                joint.compute_variable_position(&transform),
                value,
                epsilon = 1e-12
            );
        }
    }
}
