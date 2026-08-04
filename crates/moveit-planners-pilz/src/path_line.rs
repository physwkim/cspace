// Copyright (c) 2004-2005, Erwin Aertbelien, Div. PMA, Dep. of Mech. Eng., K.U.Leuven
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from orocos_kinematics_dynamics @ v1.5.1 (see
// `crates/moveit-state/src/dynamics.rs` for how this workspace pins and
// verifies that checkout against the oracle image's compiled `liborocos-kdl`):
//   orocos_kdl/src/path_line.{hpp,cpp}
//   orocos_kdl/src/rotational_interpolation_sa.{hpp,cpp}
//   orocos_kdl/src/frames.cpp (`Rotation::GetRotAngle`, `Rotation::Rot2`)
// used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_lin.cpp
// (`TrajectoryGeneratorLIN::setPathLIN`).

//! A straight-line Cartesian path with single-axis rotational interpolation
//! ([`PathLine`]), ported from `KDL::Path_Line` composed with
//! `KDL::RotationalInterpolation_SingleAxis` — the only
//! `RotationalInterpolation` upstream's `TrajectoryGeneratorLIN` ever
//! constructs, so the two are folded into one type here rather than kept as
//! a trait plus one implementor.
//!
//! # Deviations from upstream
//!
//! - **`RotationalInterpolation_SingleAxis` is not a separate type.**
//!   Upstream's `Path_Line` holds a `RotationalInterpolation*` so a caller
//!   could plug in a different interpolation strategy; `TrajectoryGeneratorLIN`
//!   never does (`setPathLIN` always `new`s a
//!   `RotationalInterpolation_SingleAxis`), so the polymorphism has no reader
//!   here. Its `SetStartEnd`/`Pos` logic is inlined into [`PathLine::new`]/
//!   [`PathLine::pos`] directly.
//! - **Only the `Frame`-to-`Frame` constructor, `PathLength`, and `Pos` are
//!   ported.** The `Frame`-plus-`Twist` constructor and `Vel`/`Acc`/`Write`/
//!   `Clone`/`LengthToS` have no caller: `generate_joint_trajectory` (this
//!   crate's `trajectory_functions` module) only ever calls
//!   [`crate::trajectory_functions::CartesianPath::duration`]/`pos`, composed
//!   from [`PathLine::path_length`]/[`PathLine::pos`] by
//!   `TrajectoryGeneratorLIN`'s own Cartesian trajectory segment — see this
//!   crate's `deny(warnings)` policy on dead code.
//! - **`Rotation::GetRotAngle`'s `eps`/`eps2` use
//!   [`crate::velocity_profile::KDL_EPSILON`].** Matches upstream's own
//!   default parameter (`GetRotAngle(Vector&, double eps=epsilon)`, KDL's
//!   `epsilon = 1e-6`).

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use nalgebra::Unit;

use crate::velocity_profile::KDL_EPSILON;

/// `KDL::Vector::Normalize`: normalizes in place and returns the original
/// norm, except a norm below `eps` yields the unit X axis and a returned
/// norm of `0.0` (upstream's zero-length convention, not IEEE `NaN`).
fn kdl_normalize(v: Vector3, eps: f64) -> (Vector3, f64) {
    let norm = v.norm();
    if norm < eps {
        (Vector3::new(1.0, 0.0, 0.0), 0.0)
    } else {
        (v / norm, norm)
    }
}

/// `KDL::Rotation::GetRotAngle`: the angle and unit axis of `rotation`'s
/// rotation-vector (log map), with upstream's `angle == 0` and `angle == PI`
/// singularity handling.
fn get_rot_angle(rotation: &UnitQuaternion, eps: f64) -> (f64, Vector3) {
    let m = rotation.to_rotation_matrix();
    let d = |r: usize, c: usize| m[(r, c)];
    let eps2 = eps * 10.0;

    if (d(0, 1) - d(1, 0)).abs() < eps
        && (d(0, 2) - d(2, 0)).abs() < eps
        && (d(1, 2) - d(2, 1)).abs() < eps
    {
        if (d(0, 1) + d(1, 0)).abs() < eps2
            && (d(0, 2) + d(2, 0)).abs() < eps2
            && (d(1, 2) + d(2, 1)).abs() < eps2
            && (d(0, 0) + d(1, 1) + d(2, 2) - 3.0).abs() < eps2
        {
            return (0.0, Vector3::new(0.0, 0.0, 1.0));
        }

        let xx = (d(0, 0) + 1.0) / 2.0;
        let yy = (d(1, 1) + 1.0) / 2.0;
        let zz = (d(2, 2) + 1.0) / 2.0;
        let xy = (d(0, 1) + d(1, 0)) / 4.0;
        let xz = (d(0, 2) + d(2, 0)) / 4.0;
        let yz = (d(1, 2) + d(2, 1)) / 4.0;
        let axis = if xx > yy && xx > zz {
            let x = xx.sqrt();
            Vector3::new(x, xy / x, xz / x)
        } else if yy > zz {
            let y = yy.sqrt();
            Vector3::new(xy / y, y, yz / y)
        } else {
            let z = zz.sqrt();
            Vector3::new(xz / z, yz / z, z)
        };
        return (std::f64::consts::PI, axis);
    }

    let f = (d(0, 0) + d(1, 1) + d(2, 2) - 1.0) / 2.0;
    let axis = Vector3::new(d(2, 1) - d(1, 2), d(0, 2) - d(2, 0), d(1, 0) - d(0, 1));
    let angle = (axis.norm() / 2.0).atan2(f);
    let (axis, _) = kdl_normalize(axis, eps);
    (angle, axis)
}

/// A straight-line Cartesian path from a start to a goal pose, with
/// single-axis rotational interpolation. Ported from `KDL::Path_Line`
/// (composed with `KDL::RotationalInterpolation_SingleAxis`) — see the
/// [module docs](self) for what is and is not ported.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathLine {
    orient_start: UnitQuaternion,
    rot_axis: Vector3,
    v_base_start: Vector3,
    v_start_end: Vector3,
    path_length: f64,
    scale_lin: f64,
    scale_rot: f64,
}

impl PathLine {
    /// Upstream `Path_Line(F_base_start, F_base_end, orient, eqradius,
    /// aggregate)`, `orient` fixed to `RotationalInterpolation_SingleAxis`
    /// (see the [module docs](self)).
    ///
    /// `eqradius` is the equivalent radius balancing rotational against
    /// translational path length — see `Path_Line`'s own constructor doc
    /// comment in `orocos_kdl/src/path_line.hpp` for the full rationale.
    pub fn new(start: &Isometry3, goal: &Isometry3, eqradius: f64) -> Self {
        let (v_start_end, dist) = kdl_normalize(
            goal.translation.vector - start.translation.vector,
            KDL_EPSILON,
        );
        let r_start_end = start.rotation.inverse() * goal.rotation;
        let (angle, rot_axis) = get_rot_angle(&r_start_end, KDL_EPSILON);

        let (path_length, scale_lin, scale_rot) = if angle != 0.0 && angle * eqradius > dist {
            (angle * eqradius, dist / (angle * eqradius), 1.0 / eqradius)
        } else if dist != 0.0 {
            (dist, 1.0, angle / dist)
        } else {
            (0.0, 1.0, 1.0)
        };

        Self {
            orient_start: start.rotation,
            rot_axis,
            v_base_start: start.translation.vector,
            v_start_end,
            path_length,
            scale_lin,
            scale_rot,
        }
    }

    /// Upstream `PathLength`.
    pub fn path_length(&self) -> f64 {
        self.path_length
    }

    /// Upstream `Pos`.
    pub fn pos(&self, s: f64) -> Isometry3 {
        let theta = s * self.scale_rot;
        let rotation = self.orient_start
            * UnitQuaternion::from_axis_angle(&Unit::new_unchecked(self.rot_axis), theta);
        let translation = self.v_base_start + self.v_start_end * s * self.scale_lin;
        Isometry3::from_parts(translation.into(), rotation)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    // -- pos: endpoints of the parametrization match the constructor's
    // start/goal exactly --

    #[test]
    fn pos_at_zero_and_path_length_reproduces_start_and_goal() {
        let start = Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(1.0, 2.0, 3.0).into(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        let path = PathLine::new(&start, &goal, 1.0);

        let at_start = path.pos(0.0);
        assert_relative_eq!(
            at_start.translation.vector,
            start.translation.vector,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            at_start.rotation.quaternion().coords,
            start.rotation.quaternion().coords,
            epsilon = 1e-12
        );

        let at_end = path.pos(path.path_length());
        assert_relative_eq!(
            at_end.translation.vector,
            goal.translation.vector,
            epsilon = 1e-9
        );
        // Quaternion double-cover: q and -q represent the same rotation.
        let same_rotation =
            (at_end.rotation.quaternion().coords - goal.rotation.quaternion().coords).norm() < 1e-9
                || (at_end.rotation.quaternion().coords + goal.rotation.quaternion().coords).norm()
                    < 1e-9;
        assert!(
            same_rotation,
            "{:?} != +/-{:?}",
            at_end.rotation, goal.rotation
        );
    }

    // -- pos: pure translation (identical orientation) is a straight
    // interpolation, path_length equals the Euclidean distance --

    #[test]
    fn pure_translation_path_length_is_euclidean_distance() {
        let start = Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(3.0, 4.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let path = PathLine::new(&start, &goal, 1.0);
        assert_relative_eq!(path.path_length(), 5.0, epsilon = 1e-12);

        let midpoint = path.pos(2.5);
        assert_relative_eq!(
            midpoint.translation.vector,
            Vector3::new(1.5, 2.0, 0.0),
            epsilon = 1e-12
        );
    }

    // -- pos: identical start/goal is a zero-length, well-defined path (no
    // division by zero) --

    #[test]
    fn identical_start_and_goal_is_a_degenerate_zero_length_path() {
        let pose = Isometry3::from_parts(
            Vector3::new(1.0, 1.0, 1.0).into(),
            UnitQuaternion::identity(),
        );
        let path = PathLine::new(&pose, &pose, 1.0);
        assert_relative_eq!(path.path_length(), 0.0);
        assert_relative_eq!(
            path.pos(0.0).translation.vector,
            pose.translation.vector,
            epsilon = 1e-12
        );
    }
}
