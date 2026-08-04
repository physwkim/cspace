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
//! - **[`get_rot_angle`]'s `eps` uses [`crate::velocity_profile::KDL_EPSILON`].**
//!   Matches upstream's own default parameter
//!   (`GetRotAngle(Vector&, double eps=epsilon)`, KDL's `epsilon = 1e-6`) —
//!   see that function's own doc comment for why it needs only one epsilon,
//!   not upstream's `eps`/`eps2` pair.

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use nalgebra::Unit;

use crate::velocity_profile::KDL_EPSILON;

/// Normalizes `v`, returning `(unit direction, original norm)`.
///
/// # Not transcribed from `KDL::Vector::Normalize`
///
/// A norm below `eps` is elementary-math-undefined for the "direction"
/// half of the answer: division by (numerically) zero has no direction to
/// recover, so this port reports the direction as the zero vector rather
/// than picking an arbitrary nonzero axis, and reports the norm as exactly
/// `0.0` (the honest value once the magnitude is indistinguishable from
/// numerical noise, not a near-zero float or IEEE `NaN`). This is a
/// genuinely different choice from `Vector::Normalize`'s own degenerate
/// branch (which returns the unit X axis, `frames.cpp:147-156`), not a
/// restatement of it, and it does not change any currently reachable
/// observable output of this crate's callers:
///
/// - [`PathLine::new`]'s `scale_lin`/`path_length` derivation multiplies
///   this direction by a coefficient that is itself provably `0.0`
///   whenever the norm-`0.0` branch fires here (see that function's own
///   doc comment).
/// - [`get_rot_angle`]'s own internal call never actually reaches this
///   branch: the vector it normalizes has a norm that is provably at
///   least `eps` by the time that call executes, given the singularity
///   check earlier in the same function already excluded the case where
///   it would not be.
/// - [`crate::path_circle::PathCircle::new`]'s `radius`-producing call
///   returns an error immediately upon seeing `norm < eps`, before its
///   direction is ever read.
/// - The one caller that does not gate on the norm before reading the
///   direction back out — `PathCircle::new`'s auxiliary-point
///   normalization, whose result feeds a subsequent cross product before
///   *that* result's own norm is checked — collapses deterministically to
///   `z_norm == 0.0 < eps` (an immediate rejection) with a zero-vector
///   direction here, a defensible degenerate answer for a construction
///   request whose auxiliary point coincides with its own center. It does
///   not reproduce `Vector::Normalize`'s own value-dependent behavior for
///   that specific malformed input (which depends on the incidental
///   alignment between the caller's other axis and the arbitrary unit-X
///   fallback), but no fixture in this crate exercises that input.
///
/// `pub(crate)`: also used by [`crate::path_circle::PathCircle`] as
/// itemized above.
pub(crate) fn kdl_normalize(v: Vector3, eps: f64) -> (Vector3, f64) {
    let norm = v.norm();
    if norm < eps {
        (Vector3::zeros(), 0.0)
    } else {
        (v / norm, norm)
    }
}

/// The angle (in `[0, pi]`) and unit axis of `rotation`'s rotation-vector
/// (log map): a rotation of `angle` radians about `axis` reproduces
/// `rotation`.
///
/// # Not transcribed from `Rotation::GetRotAngle`
///
/// `GetRotAngle` operates on a raw 3x3 rotation matrix, which has no
/// numerically robust way to recover the half-angle and axis directly at
/// either `angle == 0` (axis undefined) or `angle == PI` (the matrix's
/// antisymmetric part — the part `GetRotAngle`'s general case reads the
/// axis from — vanishes there, forcing a second, unrelated extraction from
/// the symmetric part's largest diagonal entry). `rotation` here is
/// already a [`UnitQuaternion`], not a matrix, and a unit quaternion
/// carries the half-angle directly by construction: `w = cos(angle/2)`,
/// `(x, y, z) = sin(angle/2) * axis` (the standard axis-angle-to-quaternion
/// identity every quaternion library, including this one, builds
/// `from_axis_angle` from). Inverting it, `angle = 2*atan2(|xyz|, |w|)` and
/// `axis = xyz / |xyz|`, is valid across the entire range with exactly one
/// singularity, at `angle == 0` (where `sin(0) == 0` makes `xyz` the zero
/// vector and the axis genuinely undefined) — `sin(angle/2)` is never zero
/// anywhere in `(0, pi]`, so unlike the matrix formulation there is no
/// second singularity at `angle == PI` to handle. This is exactly
/// [`UnitQuaternion::axis_angle`]'s own formula (it also takes `|w|` rather
/// than `w`, which is what keeps the returned angle in `[0, pi]` regardless
/// of the quaternion's double-cover sign).
///
/// The one remaining singularity (`angle < eps`, generalizing exact
/// `angle == 0` to "close enough that the axis is dominated by rounding
/// noise") is handled by substituting a fixed placeholder axis, the same
/// way [`kdl_normalize`] substitutes a fixed placeholder direction for a
/// vector too short to have one — see that function's doc comment for why
/// a fixed placeholder here does not change any observable output of this
/// crate's callers: [`PathLine::new`]'s `scale_rot` is itself `0.0`
/// whenever `angle == 0`, so [`PathLine::pos`]'s `theta` is always exactly
/// `0.0` there regardless of which axis this returns.
///
/// `pub(crate)`: also used by [`crate::path_circle::PathCircle`], whose
/// `RotationalInterpolation_SingleAxis` component is the identical
/// convention `PathLine` folds in — see that type's own module doc.
pub(crate) fn get_rot_angle(rotation: &UnitQuaternion, eps: f64) -> (f64, Vector3) {
    match rotation.axis_angle() {
        Some((axis, angle)) if angle >= eps => (angle, axis.into_inner()),
        _ => (0.0, Vector3::zeros()),
    }
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

    // -- get_rot_angle: round-trips through the boundaries the old
    // matrix-based derivation special-cased (angle == 0, angle == PI along
    // several axes) and a generic non-singular rotation, verified by
    // reconstructing the rotation from (angle, axis) rather than comparing
    // axis values directly -- a rotation of PI about `axis` and about
    // `-axis` are the same rotation, so the axis itself is only defined up
    // to sign exactly at that boundary. --

    fn assert_get_rot_angle_round_trips(rotation: UnitQuaternion) {
        let (angle, axis) = get_rot_angle(&rotation, KDL_EPSILON);
        assert!((0.0..=std::f64::consts::PI).contains(&angle), "{angle}");
        let reconstructed = UnitQuaternion::from_axis_angle(&Unit::new_unchecked(axis), angle);
        let same_rotation =
            (reconstructed.quaternion().coords - rotation.quaternion().coords).norm() < 1e-9
                || (reconstructed.quaternion().coords + rotation.quaternion().coords).norm() < 1e-9;
        assert!(
            same_rotation,
            "{reconstructed:?} != +/-{rotation:?} (angle {angle}, axis {axis:?})"
        );
    }

    #[test]
    fn get_rot_angle_at_identity_is_zero_with_a_well_defined_axis() {
        let (angle, axis) = get_rot_angle(&UnitQuaternion::identity(), KDL_EPSILON);
        assert_relative_eq!(angle, 0.0);
        assert!(axis.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn get_rot_angle_round_trips_at_pi_about_each_axis() {
        for axis in [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()] {
            assert_get_rot_angle_round_trips(UnitQuaternion::from_axis_angle(
                &axis,
                std::f64::consts::PI,
            ));
        }
    }

    #[test]
    fn get_rot_angle_round_trips_at_pi_about_an_arbitrary_axis() {
        let axis = Unit::new_normalize(Vector3::new(1.0, 2.0, 3.0));
        assert_get_rot_angle_round_trips(UnitQuaternion::from_axis_angle(
            &axis,
            std::f64::consts::PI,
        ));
    }

    #[test]
    fn get_rot_angle_round_trips_for_a_generic_non_singular_rotation() {
        assert_get_rot_angle_round_trips(UnitQuaternion::from_euler_angles(0.3, -0.7, 1.1));
    }

    #[test]
    fn get_rot_angle_below_eps_snaps_to_exactly_zero() {
        let tiny = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), KDL_EPSILON / 10.0);
        let (angle, _) = get_rot_angle(&tiny, KDL_EPSILON);
        assert_eq!(angle, 0.0);
    }
}
