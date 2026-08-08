// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_lin.cpp
// (`TrajectoryGeneratorLIN::setPathLIN`).

//! A straight-line Cartesian path with single-axis rotational interpolation
//! ([`PathLine`]), playing the role of `KDL::Path_Line` composed with
//! `KDL::RotationalInterpolation_SingleAxis` — the only
//! `RotationalInterpolation` upstream's `TrajectoryGeneratorLIN` ever
//! constructs, so the two are folded into one type here rather than kept as
//! a trait plus one implementor. See below for why this is *not* a
//! line-by-line port of either.
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
//!   provided.** The `Frame`-plus-`Twist` constructor and `Vel`/`Acc`/`Write`/
//!   `Clone`/`LengthToS` have no caller: `generate_joint_trajectory` (this
//!   crate's `trajectory_functions` module) only ever calls
//!   [`crate::trajectory_functions::CartesianPath::duration`]/`pos`, composed
//!   from [`PathLine::path_length`]/[`PathLine::pos`] by
//!   `TrajectoryGeneratorLIN`'s own Cartesian trajectory segment — see this
//!   crate's `deny(warnings)` policy on dead code.
//!
//! # Why this file stays BSD-3-Clause
//!
//! `KDL::Path_Line`, `RotationalInterpolation_SingleAxis` and
//! `Rotation::GetRotAngle`/`Vector::Normalize` are LGPL-2.1-or-later
//! (`third_party/orocos_kinematics_dynamics/`), heavier copyleft than this
//! workspace's BSD-3-Clause. Nothing in this file is transcribed from
//! them: `kdl_normalize`, `get_rot_angle` and [`PathLine::new`] are
//! each derived independently (elementary vector algebra, the standard
//! quaternion axis-angle identity, and the same multi-motion
//! synchronization already used by
//! [`crate::trajectory_generator_ptp::TrajectoryGeneratorPtp`], respectively
//! — see each function's own doc comment for the derivation). What is
//! reused from the LGPL sources is *interface facts*, not expression: the
//! `eqradius` convention balancing translational against rotational arc
//! length into one path parameter (named here by the same convention
//! `Path_Line`'s own constructor doc comment in
//! `orocos_kdl/src/path_line.hpp` uses), and the general shape of a
//! "start pose, goal pose, single rotation axis" Cartesian path — not the
//! algorithms that fill it in. Equivalence with upstream is proven the
//! same way every other generator in this crate proves it: oracle parity
//! on captured fixtures (`tests/pilz_trajectory_lin_parity.rs`,
//! `tests/pilz_trajectory_circ_parity.rs` for [`crate::path_circle::PathCircle`]'s
//! shared use of `kdl_normalize`/`get_rot_angle`), not line
//! correspondence.

use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use nalgebra::Unit;

use crate::numeric::cxx_max;
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
/// branch (which returns the unit X axis, `orocos_kdl/frames.cpp:147-156`), not a
/// restatement of it, and it does not change any currently reachable
/// observable output of this crate's callers:
///
/// - [`PathLine::new`]'s `scale_lin`/`path_length` derivation multiplies
///   this direction by a coefficient that is itself provably `0.0`
///   whenever the norm-`0.0` branch fires here (see that function's own
///   doc comment).
/// - [`crate::path_circle::PathCircle::new`]'s `radius`-producing call and
///   its auxiliary-point-normalizing call both return an error immediately
///   upon seeing `norm < eps`, before either direction is ever read (own
///   doc comment).
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
/// noise") is handled by substituting a fixed placeholder axis rather than
/// an undefined one — unlike [`kdl_normalize`]'s placeholder *direction*
/// (which this port deliberately changed to the zero vector, a genuinely
/// different choice from upstream's, see that function's doc comment),
/// this placeholder *axis* must actually be a unit vector: the return type
/// is `Unit<Vector3>`, so there is no representable "not a unit vector"
/// value to fall back to, and every caller (`UnitQuaternion::from_axis_angle`)
/// requires one regardless of the angle it is paired with. `Vector3::z_axis()`
/// is used here — the same constant upstream's own `GetRotAngle` returns in
/// this branch (`frames.cpp`'s `Choose 0, 0, 1`) — reused as the interface
/// fact it is (a numeric constant, not expression; this port's own
/// `moveit-scene`-style bucket-3 classification), not a restatement of
/// upstream's derivation. This does not change any observable output of
/// this crate's callers at any tolerance they check: [`PathLine::new`]'s
/// `scale_rot` is itself `0.0` whenever `angle == 0`, so [`PathLine::pos`]'s
/// `theta` is `0.0` there in the one caller that reaches `angle == 0`
/// exactly, and see that function's own doc comment for the one caller
/// that reaches a nonzero-but-negligible `theta` in this branch instead.
///
/// `pub(crate)`: also used by [`crate::path_circle::PathCircle`], whose
/// `RotationalInterpolation_SingleAxis` component is the identical
/// convention `PathLine` folds in — see that type's own module doc.
pub(crate) fn get_rot_angle(rotation: &UnitQuaternion, eps: f64) -> (f64, Unit<Vector3>) {
    match rotation.axis_angle() {
        Some((axis, angle)) if angle >= eps => (angle, axis),
        _ => (0.0, Vector3::z_axis()),
    }
}

/// A straight-line Cartesian path from a start to a goal pose, with
/// single-axis rotational interpolation. Ported from `KDL::Path_Line`
/// (composed with `KDL::RotationalInterpolation_SingleAxis`) — see the
/// [module docs](self) for what is and is not ported.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathLine {
    orient_start: UnitQuaternion,
    rot_axis: Unit<Vector3>,
    v_base_start: Vector3,
    v_start_end: Vector3,
    path_length: f64,
    scale_lin: f64,
    scale_rot: f64,
}

impl PathLine {
    /// Builds the straight-line path from `start` to `goal`, with a
    /// single-axis rotational interpolation folded in (see the
    /// [module docs](self)).
    ///
    /// `eqradius` is the equivalent radius balancing rotational against
    /// translational path length — see `Path_Line`'s own constructor doc
    /// comment in `orocos_kdl/src/path_line.hpp` for the full rationale.
    /// This is an interface fact this port reuses by name (as
    /// [`crate::path_circle::PathCircle`] does too), not upstream
    /// expression.
    ///
    /// # Not transcribed from `Path_Line`'s constructor /
    /// `RotationalInterpolation_SingleAxis::SetStartEnd`
    ///
    /// A single arclength parameter `s` must drive both the translation
    /// (over distance `dist`) and the rotation (over `angle` radians,
    /// converted to `angle * eqradius` length-equivalent units) to
    /// complete exactly at `s == path_length`, at whatever constant rate
    /// each needs. This is the same "pace every independent motion to the
    /// one that needs the longest run" problem
    /// [`crate::trajectory_generator_ptp::TrajectoryGeneratorPtp`] already
    /// solves by synchronizing every joint to its slowest one — here there
    /// are only two "joints" (translation and rotation, already reduced to
    /// one comparable unit by `eqradius`), so `path_length` is simply
    /// whichever of the two is longer, `cxx_max(dist, angle * eqradius)` —
    /// not plain [`f64::max`]: the constructor's three-way
    /// `if (alpha != 0 && alpha*eqradius > dist) ... else if (dist != 0) ...
    /// else` (`orocos_kdl/src/path_line.cpp`) returns a NaN `dist` and
    /// discards a NaN `angle * eqradius`, the same asymmetry this crate's
    /// (private) `numeric` module doc derives for literal `std::max` calls,
    /// even though this branch never spells the name `std::max` — see this
    /// crate's `path_line::tests::path_length_keeps_a_nan_distance_not_the_finite_rotation_length`
    /// for the derivation traced through each arm of the branch — and
    /// each part's rate (`scale_lin`, `scale_rot`) is that part's own
    /// extent divided by `path_length`, so it reaches its full extent
    /// exactly when `s` reaches `path_length`. When both extents are zero
    /// (`start` and `goal` coincide, including in orientation), `path_length`
    /// is `0.0` and there is nothing to divide by — `scale_lin`/`scale_rot`
    /// are left at a placeholder `1.0`. This placeholder is *not* only ever
    /// read at `s == 0.0`: `TrajectoryGeneratorLIN`'s own zero-length
    /// fallback (`trajectory_generator_lin.rs`) substitutes
    /// `set_profile(0.0, f64::EPSILON)` in this case, so [`PathLine::pos`]
    /// is evaluated across `s` in `[0.0, f64::EPSILON]` (~`2.22e-16`), not
    /// just at the one point. The placeholder is still unobservable there,
    /// but for a floating-point-magnitude reason, not a "never reached"
    /// one: `theta = s * scale_rot` reaches that same `~2.22e-16` scale, and
    /// `cos(theta/2)` rounds to exactly `1.0` in `f64` (the argument is far
    /// below the precision needed to perturb `1.0`) regardless of
    /// `scale_rot`'s placeholder value, while `v_start_end` (this
    /// degenerate case's `kdl_normalize` result) is exactly
    /// `Vector3::zeros()`, so the translation term `v_start_end * s *
    /// scale_lin` is exactly zero for any `s`/`scale_lin` regardless of the
    /// placeholder either. `sin(theta/2)` at this scale is *not* exactly
    /// zero (it rounds to the same `~1e-16` magnitude as `theta/2` itself),
    /// so the resulting rotation is not bit-identical to the identity
    /// quaternion — but a rotation of order `1e-16` radians is many orders
    /// below every fixture's comparison tolerance in this crate (`1e-9` or
    /// looser) and below any physically meaningful distinction, which is
    /// the sense in which this placeholder is unobservable.
    pub fn new(start: &Isometry3, goal: &Isometry3, eqradius: f64) -> Self {
        let (v_start_end, dist) = kdl_normalize(
            goal.translation.vector - start.translation.vector,
            KDL_EPSILON,
        );
        let r_start_end = start.rotation.inverse() * goal.rotation;
        let (angle, rot_axis) = get_rot_angle(&r_start_end, KDL_EPSILON);

        let path_length = cxx_max(dist, angle * eqradius);
        let (scale_lin, scale_rot) = if path_length > 0.0 {
            (dist / path_length, angle / path_length)
        } else {
            (1.0, 1.0)
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

    /// Upstream `LengthToS`: the arc-length parameter at which this path has
    /// travelled `length` in translation.
    ///
    /// Upstream divides by `scalelin` with no guard, so a rotation-only line
    /// (`scale_lin == 0.0`) returns an infinity or a `NaN` rather than
    /// failing — reproduced rather than turned into an error, since
    /// [`crate::path_rounded_composite::PathRoundedComposite`], its only
    /// caller here, has already rejected the zero-translation case (its
    /// `Not_Feasible` codes 2 and 3) before it can reach this.
    pub fn length_to_s(&self, length: f64) -> f64 {
        length / self.scale_lin
    }

    /// Upstream `Pos`.
    pub fn pos(&self, s: f64) -> Isometry3 {
        let theta = s * self.scale_rot;
        let rotation = self.orient_start * UnitQuaternion::from_axis_angle(&self.rot_axis, theta);
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

    // -- new: when the rotation's equivalent length (angle * eqradius)
    // exceeds the translation distance, path_length is paced by the
    // rotation, not the translation -- the boundary `pure_translation...`
    // and `identical_start_and_goal...` above do not exercise --

    #[test]
    fn rotation_dominates_path_length_when_its_equivalent_length_is_longer() {
        let start = Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(0.01, 0.0, 0.0).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let path = PathLine::new(&start, &goal, 1.0);
        // angle * eqradius == pi/2 ~= 1.57, dist == 0.01: rotation dominates.
        assert_relative_eq!(
            path.path_length(),
            std::f64::consts::FRAC_PI_2,
            epsilon = 1e-12
        );

        let at_end = path.pos(path.path_length());
        assert_relative_eq!(
            at_end.translation.vector,
            goal.translation.vector,
            epsilon = 1e-9
        );
        assert_relative_eq!(
            at_end.rotation.quaternion().coords,
            goal.rotation.quaternion().coords,
            epsilon = 1e-9
        );
    }

    // -- new: `path_length`'s `dist.max(angle * eqradius)` must reproduce
    // `Path_Line`'s constructor's NaN behavior (`std::max`-shaped: a NaN
    // `dist` propagates, a NaN `angle * eqradius` is discarded), not IEEE
    // `f64::max`'s (discards NaN wherever it sits) -- see `crate::numeric`'s
    // module doc for the derivation from the constructor's three-way
    // `if (alpha != 0 && alpha*eqradius > dist) ... else if (dist != 0) ...
    // else` in `orocos_kdl/src/path_line.cpp`. --

    #[test]
    fn path_length_keeps_a_nan_distance_not_the_finite_rotation_length() {
        let start = Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(f64::NAN, 0.0, 0.0).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        let path = PathLine::new(&start, &goal, 1.0);
        assert!(path.path_length().is_nan(), "{}", path.path_length());
    }

    #[test]
    fn path_length_discards_a_nan_rotation_length_and_keeps_the_finite_distance() {
        let start = Isometry3::from_parts(
            Vector3::new(0.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(3.0, 4.0, 0.0).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        );
        // Demonstrated opposite of the case above: a NaN `eqradius` makes
        // `angle * eqradius` NaN while `dist` (5.0, the 3-4-5 translation
        // above) stays finite. The NaN operand is second here, not first, so
        // it must be discarded rather than propagated -- this already passes
        // on plain `f64::max` too, which is what makes it the case that
        // catches a fix that over-corrects into always propagating NaN.
        let path = PathLine::new(&start, &goal, f64::NAN);
        assert_relative_eq!(path.path_length(), 5.0, epsilon = 1e-12);
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

    /// The test above only checks `pos(0.0)`. `PathLine::new`'s own doc
    /// comment argues `scale_lin`'s zero-length placeholder is unobservable
    /// "for any `s`", not just within `TrajectoryGeneratorLIN`'s bounded
    /// `s in [0, f64::EPSILON]` fallback -- because `v_start_end` is exactly
    /// `Vector3::zeros()` in this branch, so `v_start_end * s * scale_lin`
    /// is exactly zero regardless of `s`. `PathLine::new`/[`PathLine::pos`]
    /// are both `pub fn`, so an external caller can reach an arbitrarily
    /// large `s` directly, bypassing that one caller's bound entirely --
    /// measured here up to `s = 1000.0`. This does not change behavior; it
    /// closes a coverage gap the "for any `s`" claim already had.
    #[test]
    fn zero_length_path_stays_at_start_for_any_s_not_just_zero() {
        let pose = Isometry3::from_parts(
            Vector3::new(1.0, 1.0, 1.0).into(),
            UnitQuaternion::identity(),
        );
        let path = PathLine::new(&pose, &pose, 1.0);
        for s in [1e-16, 1e-6, 1.0, 1000.0] {
            assert_relative_eq!(
                path.pos(s).translation.vector,
                pose.translation.vector,
                epsilon = 1e-12
            );
        }
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
        let reconstructed = UnitQuaternion::from_axis_angle(&axis, angle);
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
        assert_relative_eq!(axis.into_inner().norm(), 1.0);
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
