// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2019, PickNik Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/cartesian_interpolator.hpp
//   moveit_core/robot_state/src/cartesian_interpolator.cpp

//! Straight-line Cartesian path interpolation: the sequence of joint states
//! that walks a robot link along a Cartesian path
//! ([`CartesianInterpolator`]), and the joint-space jump detection that
//! truncates such a path where consecutive waypoints are too far apart
//! ([`has_joint_space_jump`]/[`check_joint_space_jump`]).
//!
//! Upstream `moveit::core::CartesianInterpolator`. Given a start state and a
//! target pose for one link, the path is divided into `steps` waypoints from
//! [`MaxEefStep`] (see [`CartesianInterpolator::to_pose`] for the exact
//! count), each waypoint's Cartesian pose is interpolated
//! (slerp for rotation, lerp for translation), and IK is solved for it
//! seeded from the previous waypoint's solution. The return value is the
//! fraction of the path achieved before IK first failed.
//!
//! On top of that, every interval between two accepted waypoints is
//! *validated*: the joint-space midpoint of its two states is run through
//! forward kinematics and compared against the Cartesian midpoint of its two
//! poses. If they disagree by more than [`CartesianPrecision`], the interval
//! is bisected and the halves are validated recursively, down to
//! [`CartesianPrecision::max_resolution`]. This is what keeps the joint path
//! actually following the straight line rather than merely hitting the
//! sampled waypoints on it.
//!
//! # Why this lives in `cspace_core::kinematics` and not `cspace_core::state`
//!
//! Upstream puts this file in `moveit_core/robot_state/` next to
//! `RobotState`, whose `setFromIK` it is written on. This port cannot, and
//! neither can `setFromIK`: both need a [`KinematicsSolver`], and
//! `cspace_core::kinematics` already depends on
//! `cspace_core::state` — so a `cspace_core::state -> cspace_core::kinematics` edge would be a
//! dependency cycle cargo rejects outright, not a layering question anyone
//! gets to sign off on. Placing it here adds **no** new dependency edge:
//! `cspace_core::error`, `cspace_core::geometry`, `cspace_core::model` and `cspace_core::state` are
//! already this crate's four dependencies, and they are all this module
//! needs. Upstream's own header agrees the file is misplaced —
//! `cartesian_interpolator.hpp:107` carries a
//! `TODO(mlautman): Eventually, this planner should be moved out of
//! robot_state`.
//!
//! # Deviations from upstream
//!
//! ## `setFromIK` is replaced by a [`KinematicsSolver`] call
//!
//! This is the largest deviation and the reason this is not a line-by-line
//! transcription. Upstream calls
//! `state.setFromIK(group, pose * inv_offset, link->getName(), 0.0,
//! validCallback, options, cost_function)` at both IK sites
//! (`cartesian_interpolator.cpp:260` in the main loop,
//! `cartesian_interpolator.cpp:94` inside the bisection). `RobotState::
//! setFromIK` (`robot_state/src/robot_state.cpp:1788-2047`) is considerably more than "call
//! the solver", and the pieces it adds are handled here as follows.
//!
//! *Solver lookup.* Upstream reaches the solver through
//! `jmg->getSolverInstance()`, a per-group instance fixed at model-load time
//! from the SRDF's `kinematics.yaml`. Nothing in this workspace's
//! `RobotModel` loads `kinematics.yaml` and `crate::model::JointModelGroup`
//! has no solver field, so [`CartesianInterpolator::to_pose`] takes an
//! already-constructed `&mut dyn KinematicsSolver` — the same deviation, for
//! the same reason, that
//! `cspace_planners::pilz::trajectory_functions::compute_pose_ik` already
//! documents as its item 1.
//!
//! *Frame conversion into the solver's base, tip-frame resolution, attached
//! bodies, the multi-tip fill, and the group-state validity callback.* All
//! five were this module's own problem until round 10, and none of them is
//! any more: [`fn@crate::kinematics::set_from_ik`] is the port of `setFromIK` itself, and
//! `PathRun::solve_link_pose` now calls it with a single
//! [`IkTarget`] naming [`CartesianInterpolator::link_name`]. What that buys
//! this module, concretely: the requested link no longer has to *be* the
//! solver's tip, only rigidly connected to it
//! (`robot_state/src/robot_state.cpp:1922-1945`, ported via
//! `crate::model::RobotModel::rigidly_connected_parent_link`); an attached
//! body or one of its subframes can be the requested frame, through
//! [`IkContext::attached`]; and [`IkContext::validity`] is upstream's real
//! `GroupStateValidityCallbackFn`, taking `(RobotState, JointModelGroup,
//! group-ordered values)` rather than the bare `&[f64]` that
//! [`crate::kinematics::SolveOptions::solution_callback`] takes. That last one is why
//! the three entry points here take an [`IkContext`] instead of a
//! `&mut SolveOptions`: a caller collision-checking a candidate needs the
//! posed state, and threading a scratch state through the solver-level
//! callback was the shape that could not give it one.
//!
//! *`setFromIKSubgroups`.* Ported as [`crate::kinematics::set_from_ik_subgroups`], but
//! not reachable from here, and upstream is the same: `computeCartesianPath`
//! walks *one* link along *one* path, so its `setFromIK` call always carries
//! exactly one pose and can never take the multi-tip branch that diverts to
//! subgroup solvers (`robot_state/src/robot_state.cpp:1836-1866`).
//!
//! *`timeout = 0.0`.* Both upstream IK sites pass `0.0` and the deprecated
//! overload's comment (`cartesian_interpolator.cpp:453-454`) says this means
//! "a single IK attempt only ... random seeding would create large
//! joint-space jumps". It does not — see `doc/upstream-bugs.md`'s
//! `set-from-ik-zero-timeout-is-not-single-attempt`. This port has no
//! timeout parameter at all ([`crate::kinematics::SolverParams::max_restarts`] replaced it
//! crate-wide), so there is no sentinel value to be silently reinterpreted;
//! a caller that wants the single deterministic attempt the comment
//! describes builds its solver with `max_restarts = 0`, and gets it.
//!
//! ## The bisection's dead `percentage` out-parameter is made live
//!
//! Upstream's `validateAndImproveInterval` takes `double& percentage` and
//! writes a partial-progress value into it on the failing path, but every
//! caller discards it — see `doc/upstream-bugs.md`'s
//! `validate-and-improve-interval-percentage-discarded`. This port splits
//! the parameter's two meanings: `percentage` stays a by-value input (the
//! path parameter of the interval's *end*), and the achieved fraction is
//! recorded at the one place a waypoint is actually appended. The invariant
//! that buys is worth stating: **the fraction returned by
//! [`CartesianInterpolator::to_pose`] is the path parameter of the last
//! waypoint in the trajectory it returns**, on the success path and the
//! failure path alike. Upstream holds that invariant only when no bisected
//! sub-interval was accepted before the failure.
//!
//! ## Types
//!
//! `Percentage` is ported ([`Percentage`]) because it carries a real
//! validated invariant — upstream's constructor throws outside `[0, 1]`.
//! `Distance` is not: any `f64` is a valid metre count, so that type carries
//! no invariant and exists only so C++'s `Distance * Percentage` overload
//! can typecheck the one multiplication in `computeCartesianPath`'s
//! translation form. [`CartesianInterpolator::along_translation`] returns
//! that product as a plain `f64` in metres.
//!
//! The per-path arguments upstream repeats on every one of its static
//! `computeCartesianPath` overloads (`group`, `link`, `link_offset`,
//! `max_step`, `precision`, `global_reference_frame`) are fields of
//! [`CartesianInterpolator`] here. That is not only ergonomics: passing them
//! per call would put every entry point past clippy's
//! `too_many_arguments` threshold, and `tools/ci/check-no-lint-suppression.sh`
//! forbids answering that with an `#[allow]`.
//!
//! ## Out of scope, and why
//!
//! - **The three `[[deprecated]]` `computeCartesianPath` overloads**
//!   (`cartesian_interpolator.hpp:250`, `:262`, `:284`, `:299`) — the ones
//!   taking a [`JumpThreshold`] where the current ones take a
//!   [`CartesianPrecision`]. Upstream keeps them for source compatibility
//!   with callers written before the bisection existed; a new port has no
//!   such callers. Their two pieces of behaviour that are *not* just the
//!   current overload plus a jump check go with them: the
//!   `steps = max(steps, MIN_STEPS_FOR_JUMP_THRESH)` floor applied when a
//!   relative threshold is set (`cartesian_interpolator.cpp:415-416`), and
//!   the `consistency_limits` vector built from the absolute thresholds and
//!   passed into the IK call (`cartesian_interpolator.cpp:418-440`). A
//!   caller wanting the latter has [`IkContext::consistency_limits`]
//!   directly. The jump detection those overloads exist to drive is ported
//!   in full and standalone, below — upstream's own
//!   `CartesianInterpolator::checkJointSpaceJump` and free
//!   `hasJointSpaceJump` are *not* deprecated.
//! - **The two deprecated `JumpThreshold` constructors**
//!   (`cartesian_interpolator.hpp:82-83`). They take the same argument list
//!   as each other modulo arity and are exactly what
//!   [`JumpThreshold::relative`]/[`JumpThreshold::absolute`] replaced;
//!   worse, `JumpThreshold(double)` skips the `relative_factor > 1.0` check
//!   its named replacement performs.
//! - **The `direction + distance` overloads** (`.hpp:183`, `:262`). Both
//!   bodies are a single forwarding call with `distance * direction`
//!   substituted for the translation vector, so a caller writes
//!   `along_translation(&(distance * direction))` and reaches the identical
//!   code.
//! - **Every `RCLCPP_*` call.** The workspace-wide rule (D1/D2); none of
//!   them gates control flow. Two carry information worth keeping, so it is
//!   kept here in prose instead: `hasRelativeJointSpaceJump` warns when a
//!   path shorter than [`MIN_STEPS_FOR_JUMP_THRESH`] is measured for
//!   relative jumps, because an average over so few increments is not a
//!   reliable baseline; and `hasAbsoluteJointSpaceJump` warns that it skips
//!   any joint that is neither revolute nor prismatic, which this port does
//!   too (see [`has_joint_space_jump`]).
//! - **`ASSERT_ISOMETRY`** on `target` and `link_offset`
//!   (`cartesian_interpolator.cpp:214-215`, `:374-375`). It guards against
//!   an `Eigen::Isometry3d` that has been written through with a scaling or
//!   shearing matrix — representable in Eigen, since `Isometry3d` is a
//!   storage mode of `Transform` and not a checked type.
//!   [`crate::geometry::Isometry3`] is `nalgebra::Isometry3`, a translation
//!   plus a `UnitQuaternion`, which cannot represent a non-isometry at all.
//! - **The `max_step.translation <= 0 && max_step.rotation <= 0` rejection**
//!   (`cartesian_interpolator.cpp:392-399`). It exists only in the
//!   deprecated overload, and its absence from the current one is coherent
//!   rather than an oversight: with both components disabled the path is one
//!   step to the target, and the bisection the current overload gained then
//!   subdivides it as far as [`CartesianPrecision`] demands. Under the
//!   deprecated overload, which has no bisection, the same input produced a
//!   single unchecked leap, which is what there was to reject.

use crate::error::Result;
use crate::geometry::{Isometry3, Vector3, quaternion};
use crate::model::joint::{JointModel, JointType};
use crate::model::{JointModelGroup, RobotModel};
use crate::state::RobotState;

use crate::kinematics::registry::KinematicsSolver;
use crate::kinematics::set_from_ik::{IkContext, IkTarget, set_from_ik};

/// Minimum number of waypoints for a relative jump threshold's average
/// joint-space increment to be a meaningful baseline.
///
/// Upstream `MIN_STEPS_FOR_JUMP_THRESH` (`cartesian_interpolator.cpp:53`).
/// Upstream uses it in two places: an `RCLCPP_WARN` when
/// `hasRelativeJointSpaceJump` is asked to measure a shorter path (kept here
/// as documentation on [`has_joint_space_jump`], per this module's "Out of
/// scope"), and a floor on the waypoint count inside the deprecated
/// `computeCartesianPath` overloads (not ported, same section). It is public
/// because a caller choosing a [`MaxEefStep`] for a path it intends to
/// jump-check needs the number.
pub const MIN_STEPS_FOR_JUMP_THRESH: usize = 10;

/// How closely the joint path must follow the Cartesian straight line
/// between two consecutive waypoints.
///
/// Upstream `CartesianPrecision`. The deviation is measured at the interval's
/// midpoint: forward kinematics on the joint-space midpoint of the two
/// waypoint states, against the Cartesian midpoint of the two waypoint
/// poses. Exceeding either tolerance bisects the interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianPrecision {
    /// Maximum midpoint deviation in translation, metres.
    pub translational: f64,
    /// Maximum midpoint deviation in rotation, radians.
    pub rotational: f64,
    /// Smallest interval width, as a fraction of the whole path, that will
    /// still be bisected. An interval narrower than this that still fails
    /// the deviation check fails the path.
    pub max_resolution: f64,
}

impl Default for CartesianPrecision {
    /// Upstream's member initializers: `0.001` m, `0.01` rad, `1e-5`.
    fn default() -> Self {
        Self {
            translational: 0.001,
            rotational: 0.01,
            max_resolution: 1e-5,
        }
    }
}

/// The maximum Cartesian distance between two consecutive waypoints, which
/// is what fixes how many waypoints a path gets.
///
/// Upstream `MaxEEFStep` (renamed for clippy's `upper_case_acronyms`). A
/// zero component disables that component's contribution to the waypoint
/// count; see [`CartesianInterpolator::to_pose`] for the exact arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaxEefStep {
    /// Metres.
    pub translation: f64,
    /// Radians.
    pub rotation: f64,
}

impl MaxEefStep {
    /// Both components given explicitly. Upstream `MaxEEFStep(double,
    /// double)`.
    pub fn new(translation: f64, rotation: f64) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Upstream `MaxEEFStep(double step_size)`: the rotation component is
    /// `3.5 * step_size`, upstream's stated "1 cm of allowed translation =
    /// 2 degrees of allowed rotation" (`0.035` rad).
    pub fn from_step_size(step_size: f64) -> Self {
        Self {
            translation: step_size,
            rotation: 3.5 * step_size,
        }
    }
}

/// When two consecutive waypoints of an already-computed path are far enough
/// apart in joint space to count as a discontinuity.
///
/// Upstream `JumpThreshold`. Construct through [`JumpThreshold::disabled`],
/// [`JumpThreshold::relative`] or [`JumpThreshold::absolute`]; the two
/// deprecated upstream constructors are not ported (see this module's "Out
/// of scope"). The two live modes are mutually exclusive in effect, not by
/// type — [`has_joint_space_jump`] tests `relative_factor` first, exactly as
/// upstream does.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JumpThreshold {
    /// Multiple of the path's *average* joint-space increment above which an
    /// individual increment is a jump. `0.0` disables the relative test.
    pub relative_factor: f64,
    /// Absolute per-revolute-joint increment, radians, above which the step
    /// is a jump. `0.0` disables the revolute half of the absolute test.
    pub revolute: f64,
    /// Absolute per-prismatic-joint increment, metres, above which the step
    /// is a jump. `0.0` disables the prismatic half of the absolute test.
    pub prismatic: f64,
}

impl JumpThreshold {
    /// No jump detection. Upstream `JumpThreshold::disabled()`, which is
    /// also its default construction.
    pub fn disabled() -> Self {
        Self::default()
    }

    /// Detect increments larger than `relative_factor` times the path's
    /// average increment. Upstream `JumpThreshold::relative`.
    ///
    /// # Panics
    ///
    /// Unless `relative_factor > 1.0`. Upstream's
    /// `rcpputils::require_true(relative_factor > 1.0)` throws
    /// `std::invalid_argument` for the same input; a factor at or below the
    /// average is not a discontinuity test.
    pub fn relative(relative_factor: f64) -> Self {
        assert!(
            relative_factor > 1.0,
            "JumpThreshold::relative needs relative_factor > 1.0, got {relative_factor}"
        );
        Self {
            relative_factor,
            ..Self::default()
        }
    }

    /// Detect per-joint increments larger than these absolute bounds.
    /// Upstream `JumpThreshold::absolute`.
    ///
    /// # Panics
    ///
    /// Unless both `revolute > 0.0` and `prismatic > 0.0`, matching
    /// upstream's two `rcpputils::require_true` calls. Note that
    /// [`has_joint_space_jump`] itself tolerates one of the two being zero
    /// (it then skips that joint type) — this constructor is stricter than
    /// the check it feeds, upstream included.
    pub fn absolute(revolute: f64, prismatic: f64) -> Self {
        assert!(
            revolute > 0.0,
            "JumpThreshold::absolute needs revolute > 0.0, got {revolute}"
        );
        assert!(
            prismatic > 0.0,
            "JumpThreshold::absolute needs prismatic > 0.0, got {prismatic}"
        );
        Self {
            relative_factor: 0.0,
            revolute,
            prismatic,
        }
    }
}

/// A fraction of a path, in `[0, 1]`.
///
/// Upstream `CartesianInterpolator::Percentage`, whose constructor throws
/// `std::runtime_error` outside that range. Ported as a newtype rather than
/// a bare `f64` for the same reason it exists upstream: the range is a real
/// invariant of every value this module returns, and the type is where it
/// holds.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Percentage(f64);

impl Percentage {
    /// # Panics
    ///
    /// If `value` is outside `[0, 1]` (NaN included), matching upstream's
    /// `throw std::runtime_error("Percentage values must be between 0 and 1,
    /// inclusive")`.
    pub fn new(value: f64) -> Self {
        assert!(
            (0.0..=1.0).contains(&value),
            "Percentage values must be between 0 and 1, inclusive, got {value}"
        );
        Self(value)
    }

    /// The wrapped fraction. Upstream's `operator double()`/`operator*()`,
    /// which exist for the same purpose.
    pub fn value(self) -> f64 {
        self.0
    }
}

impl From<Percentage> for f64 {
    fn from(percentage: Percentage) -> Self {
        percentage.0
    }
}

/// The per-path arguments of upstream's `computeCartesianPath` overloads.
///
/// See this crate's `cartesian_interpolator` module documentation for why
/// these are fields rather than parameters, and for the `setFromIK`
/// substitution every method below rests on.
#[derive(Debug, Clone, Copy)]
pub struct CartesianInterpolator<'a> {
    /// Upstream `group`: the joint group IK moves. Must name a group of the
    /// start state's own [`RobotModel`].
    pub group_name: &'a str,
    /// Upstream `link`: the link whose pose follows the Cartesian path. Must
    /// equal the solver's [`KinematicsSolver::tip_frame`] — see the module
    /// docs' "Tip-frame resolution" paragraph.
    pub link_name: &'a str,
    /// Upstream `link_offset`: a virtual frame rigidly attached to
    /// [`CartesianInterpolator::link_name`]. It is that virtual frame, not
    /// the link itself, that follows the path. Identity means the link's own
    /// frame.
    pub link_offset: Isometry3,
    /// Upstream `max_step`.
    pub max_step: MaxEefStep,
    /// Upstream `precision`.
    pub precision: CartesianPrecision,
    /// Upstream `global_reference_frame`: whether a target pose (or a
    /// translation vector) is given in the model frame, or relative to the
    /// virtual frame's current pose.
    pub global_reference_frame: bool,
}

impl<'a> CartesianInterpolator<'a> {
    /// `group_name` and `link_name` with upstream's defaults for the rest:
    /// identity [`CartesianInterpolator::link_offset`], default
    /// [`CartesianPrecision`], and
    /// [`CartesianInterpolator::global_reference_frame`] `true` (the value
    /// upstream's own `Distance`-returning overload hard-codes when it
    /// forwards to the pose overload). Assign the fields directly to change
    /// any of them.
    pub fn new(group_name: &'a str, link_name: &'a str, max_step: MaxEefStep) -> Self {
        Self {
            group_name,
            link_name,
            link_offset: Isometry3::identity(),
            max_step,
            precision: CartesianPrecision::default(),
            global_reference_frame: true,
        }
    }

    /// Walk [`CartesianInterpolator::link_name`]'s virtual frame in a
    /// straight line from its current pose to `target`.
    ///
    /// Upstream `computeCartesianPath(start_state, group, traj, link,
    /// target, global_reference_frame, max_step, precision, ...)`.
    ///
    /// The waypoint count is upstream's: with `dt` the translation distance
    /// and `dr` the rotation angle to `target`,
    /// `steps = max(floor(dt / max_step.translation),
    /// floor(dr / max_step.rotation)) + 1`, each component contributing `0`
    /// when its `max_step` component is not positive. Waypoint `i` of
    /// `1..=steps` is at path parameter `i / steps`, its pose slerped and
    /// lerped from start to target, and its joint state solved from waypoint
    /// `i - 1`'s solution as the seed. The loop stops at the first waypoint
    /// whose IK fails *or* whose interval cannot be validated down to
    /// [`CartesianPrecision::max_resolution`]; the fraction returned is the
    /// path parameter of the last waypoint in the returned trajectory (see
    /// the module docs' second deviation).
    ///
    /// The returned trajectory's first element is always `start_state`
    /// itself, unmodified, so a fully failed path still returns one waypoint
    /// and fraction `0.0`. Note that this first element is the caller's
    /// state, whereas every later one descends from a copy that has had
    /// [`RobotState::enforce_bounds`]' continuous-joint wrap applied
    /// (upstream `getContinuousJointModels()` + `enforceBounds`, reproduced
    /// here): a continuous joint at `7.0` rad in `start_state` appears as
    /// `7.0` in waypoint 0 and near `0.717` in waypoint 1. Both name the same
    /// configuration and `JointModel::distance` on a continuous joint takes
    /// the short way round, so nothing downstream sees a discontinuity — but
    /// a caller diffing raw variable values would.
    ///
    /// # Errors
    ///
    /// [`crate::error::Error::UnknownName`] if
    /// [`CartesianInterpolator::group_name`] is not a group of
    /// `start_state`'s model, or if [`CartesianInterpolator::link_name`] is
    /// neither a link of it nor a frame `ik`'s
    /// [`AttachedFrames`](crate::kinematics::AttachedFrames) knows, or if the solver's
    /// base frame is not a link of it.
    /// [`crate::error::Error::Other`] if
    /// [`CartesianInterpolator::link_name`] is not rigidly connected to any
    /// tip frame the solver reports — which, since round 10, is the only
    /// way that pairing can fail: the link no longer has to *be* the tip.
    ///
    /// # Panics
    ///
    /// Never, for a fraction outside `[0, 1]`: every value this returns is
    /// `i / steps` for some `i <= steps`, or `0.0`. See [`Percentage::new`].
    pub fn to_pose<'m>(
        &self,
        start_state: &RobotState<'m>,
        solver: &mut dyn KinematicsSolver,
        target: &Isometry3,
        ik: &mut IkContext<'_, 'm>,
    ) -> Result<(Vec<RobotState<'m>>, Percentage)> {
        let model = start_state.model();
        let group = model.joint_model_group(self.group_name)?;

        // Upstream `RobotState state(*start_state)` plus the
        // `getContinuousJointModels()` / `enforceBounds` wrap.
        let mut state = start_state.clone();
        enforce_continuous_joint_bounds(&mut state, group);

        let start_pose = state.update().global_link_transform(self.link_name)? * self.link_offset;
        let rotated_target = if self.global_reference_frame {
            *target
        } else {
            start_pose * target
        };

        let rotation_distance = start_pose.rotation.angle_to(&rotated_target.rotation);
        let translation_distance =
            (rotated_target.translation.vector - start_pose.translation.vector).norm();

        let steps = self
            .step_count(translation_distance)
            .max(self.rotation_step_count(rotation_distance))
            .saturating_add(1);
        let width = 1.0 / steps as f64;

        let mut run = PathRun {
            config: self,
            inv_offset: self.link_offset.inverse(),
            solver,
            ik,
            traj: vec![start_state.clone()],
            achieved: 0.0,
        };

        let mut previous = Waypoint {
            state: state.clone(),
            pose: start_pose,
        };
        for i in 1..=steps {
            let percentage = i as f64 / steps as f64;
            let pose = interpolate_pose(&start_pose, &rotated_target, percentage);

            if !run.solve_link_pose(&mut state, &(pose * run.inv_offset))? {
                break;
            }
            let current = Waypoint {
                state: state.clone(),
                pose,
            };
            if !run.validate_and_improve_interval(&previous, &current, percentage, width)? {
                break;
            }
            previous = current;
        }

        Ok((run.traj, Percentage::new(run.achieved)))
    }

    /// Walk [`CartesianInterpolator::link_name`] along `translation`,
    /// returning the distance in metres actually achieved.
    ///
    /// Upstream `computeCartesianPath(..., const Eigen::Vector3d&
    /// translation, ...)`, which returns `Distance(translation.norm()) *
    /// <the fraction the pose form achieved>`.
    ///
    /// Two of this struct's fields do not apply here, matching upstream's
    /// own forwarding call: [`CartesianInterpolator::link_offset`] is
    /// ignored (upstream targets `getGlobalLinkTransform(link)`, and lets
    /// the pose overload's `link_offset` default to identity), and
    /// [`CartesianInterpolator::global_reference_frame`] selects the frame
    /// *`translation` itself* is expressed in — the target pose it builds is
    /// then always passed on as a global one.
    ///
    /// # Errors
    ///
    /// See [`CartesianInterpolator::to_pose`].
    pub fn along_translation<'m>(
        &self,
        start_state: &RobotState<'m>,
        solver: &mut dyn KinematicsSolver,
        translation: &Vector3,
        ik: &mut IkContext<'_, 'm>,
    ) -> Result<(Vec<RobotState<'m>>, f64)> {
        let distance = translation.norm();

        let mut probe = start_state.clone();
        let mut pose = probe.update().global_link_transform(self.link_name)?;
        pose.translation.vector += if self.global_reference_frame {
            *translation
        } else {
            pose.rotation * translation
        };

        let global = Self {
            link_offset: Isometry3::identity(),
            global_reference_frame: true,
            ..*self
        };
        let (traj, fraction) = global.to_pose(start_state, solver, &pose, ik)?;
        Ok((traj, distance * fraction.value()))
    }

    /// Walk [`CartesianInterpolator::link_name`]'s virtual frame through
    /// `waypoints` in order, in a straight line between each consecutive
    /// pair.
    ///
    /// Upstream `computeCartesianPath(..., const
    /// EigenSTL::vector_Isometry3d& waypoints, ...)`. Each segment is a
    /// [`CartesianInterpolator::to_pose`] call seeded from the previous
    /// segment's final state, and the duplicated joining waypoint is dropped
    /// from every segment after the first. The fraction is over *waypoints*,
    /// not path length: a fully solved segment `i` sets it to
    /// `(i + 1) / waypoints.len()`, and the first segment that is not fully
    /// solved adds its own fraction over `waypoints.len()` and stops the
    /// walk.
    ///
    /// # Errors
    ///
    /// See [`CartesianInterpolator::to_pose`].
    pub fn through_waypoints<'m>(
        &self,
        start_state: &RobotState<'m>,
        solver: &mut dyn KinematicsSolver,
        waypoints: &[Isometry3],
        ik: &mut IkContext<'_, 'm>,
    ) -> Result<(Vec<RobotState<'m>>, Percentage)> {
        let count = waypoints.len() as f64;
        let mut traj: Vec<RobotState<'m>> = Vec::new();
        let mut solved = 0.0;
        let mut segment_start = start_state.clone();

        for (i, waypoint) in waypoints.iter().enumerate() {
            let (mut segment, fraction) = self.to_pose(&segment_start, solver, waypoint, ik)?;

            // Every segment repeats its predecessor's final state as its own
            // first waypoint; keep only the first segment's.
            let skip = usize::from(i > 0 && !segment.is_empty());
            traj.extend(segment.drain(skip..));

            if (fraction.value() - 1.0).abs() < f64::EPSILON {
                solved = (i + 1) as f64 / count;
            } else {
                solved += fraction.value() / count;
                break;
            }
            segment_start = traj
                .last()
                .cloned()
                .expect("a fully solved segment appended at least one waypoint");
        }

        Ok((traj, Percentage::new(solved)))
    }

    /// `floor(translation_distance / max_step.translation)`, `0` when the
    /// translation component is disabled.
    fn step_count(&self, translation_distance: f64) -> usize {
        if self.max_step.translation > 0.0 {
            saturating_floor(translation_distance / self.max_step.translation)
        } else {
            0
        }
    }

    /// `floor(rotation_distance / max_step.rotation)`, `0` when the rotation
    /// component is disabled.
    fn rotation_step_count(&self, rotation_distance: f64) -> usize {
        if self.max_step.rotation > 0.0 {
            saturating_floor(rotation_distance / self.max_step.rotation)
        } else {
            0
        }
    }
}

/// One accepted point of the path: the joint state, and the Cartesian pose
/// of [`CartesianInterpolator::link_name`]'s virtual frame there.
///
/// Upstream carries these as the parallel `(prev_state, prev_pose)` and
/// `(state, pose)` pairs `validateAndImproveInterval` takes as four separate
/// parameters; pairing them keeps that recursion inside clippy's
/// `too_many_arguments` threshold without an `#[allow]`.
struct Waypoint<'m> {
    state: RobotState<'m>,
    pose: Isometry3,
}

/// The mutable working set of one `computeCartesianPath` call: the solver
/// and the IK options every step shares, the trajectory being built, and the
/// fraction achieved so far.
struct PathRun<'a, 'o, 'm> {
    config: &'a CartesianInterpolator<'a>,
    /// `link_offset.inverse()`: the virtual frame's pose times this is the
    /// link pose IK is actually asked for.
    inv_offset: Isometry3,
    solver: &'a mut dyn KinematicsSolver,
    ik: &'a mut IkContext<'o, 'm>,
    traj: Vec<RobotState<'m>>,
    /// The path parameter of `traj`'s last element. See the module docs'
    /// second deviation for why this is a field rather than upstream's
    /// `double& percentage` parameter.
    achieved: f64,
}

impl<'m> PathRun<'_, '_, 'm> {
    /// One IK call: solve for `link_pose_world` (the *link's* pose, offset
    /// already removed) seeded from `state`'s current values, writing the
    /// solution back into `state` on success.
    ///
    /// Upstream's `state.setFromIK(group, pose * inv_offset,
    /// link->getName(), 0.0, validCallback, options, cost_function)`, and
    /// since round 10 that is literally what it is: [`set_from_ik`] is the
    /// port of that method, so the frame resolution, the tip fill and the
    /// validity hook are no longer this module's business. See the module
    /// docs for the two arguments that still have no counterpart.
    fn solve_link_pose(
        &mut self,
        state: &mut RobotState<'m>,
        link_pose_world: &Isometry3,
    ) -> Result<bool> {
        let target = IkTarget {
            pose: *link_pose_world,
            frame: self.config.link_name,
        };
        set_from_ik(state, self.solver, std::slice::from_ref(&target), self.ik)
    }

    /// Accept the interval `start..end` if the joint path across it really
    /// does follow the Cartesian straight line, bisecting it if not.
    ///
    /// Upstream `validateAndImproveInterval`. `percentage` is the path
    /// parameter of `end`; `width` is the interval's width in that same
    /// parameter. On acceptance `end`'s state is appended to the trajectory;
    /// on bisection the two halves are validated in order, so the trajectory
    /// stays sorted by path parameter and keeps whatever the first half
    /// appended even when the second half fails.
    fn validate_and_improve_interval(
        &mut self,
        start: &Waypoint<'m>,
        end: &Waypoint<'m>,
        percentage: f64,
        width: f64,
    ) -> Result<bool> {
        // The pose the joint path actually reaches halfway across the
        // interval ...
        let mut mid_state = interpolate_states(&start.state, &end.state, 0.5);
        let fk_pose = mid_state
            .update()
            .global_link_transform(self.config.link_name)?
            * self.config.link_offset;

        // ... against the pose the Cartesian straight line calls for there.
        let mid_pose = interpolate_pose(&start.pose, &end.pose, 0.5);

        let linear_distance = (mid_pose.translation.vector - fk_pose.translation.vector).norm();
        let angular_distance = mid_pose.rotation.angle_to(&fk_pose.rotation);
        if linear_distance <= self.config.precision.translational
            && angular_distance <= self.config.precision.rotational
        {
            self.traj.push(end.state.clone());
            self.achieved = percentage;
            return Ok(true);
        }

        // Upstream `width < precision.max_resolution` faithfully -- but
        // `width` only ever halves from a finite starting value (see
        // `to_pose`), while `max_resolution` is a caller-supplied
        // `CartesianPrecision` field with no validating constructor (its
        // fields are `pub`, exactly like upstream's own aggregate struct).
        // A non-finite `max_resolution` makes this comparison false for
        // every `width`, so it never stops the recursion below on its own
        // -- confirmed by direct call to reach a stack overflow on ordinary,
        // reachable `panda_arm` geometry (see this crate's tests). Treating
        // "cannot verify the configured resolution floor" as "resolution
        // already exhausted" is the same conservative direction the
        // deviation check above already fails toward: give up on the
        // interval rather than recurse on a bound that cannot be evaluated.
        if !self.config.precision.max_resolution.is_finite()
            || width < self.config.precision.max_resolution
        {
            return Ok(false);
        }

        if !self.solve_link_pose(&mut mid_state, &(mid_pose * self.inv_offset))? {
            return Ok(false);
        }

        let mid = Waypoint {
            state: mid_state,
            pose: mid_pose,
        };
        let half_width = width / 2.0;
        if !self.validate_and_improve_interval(start, &mid, percentage - half_width, half_width)? {
            return Ok(false);
        }
        self.validate_and_improve_interval(&mid, end, percentage, half_width)
    }
}

/// The index of the first waypoint that a jump lands *on*, or [`None`] if
/// the path has no jump under `jump_threshold`.
///
/// Upstream free function `hasJointSpaceJump`. `relative_factor` is tested
/// first and wins outright if set; only then are the absolute thresholds
/// considered. A path of one waypoint or fewer never has a jump.
///
/// The returned index `i` means the step from `waypoints[i - 1]` to
/// `waypoints[i]` is the jump, so `waypoints[..i]` is the jump-free prefix —
/// which is exactly what [`check_joint_space_jump`] truncates to.
///
/// The two modes measure different things, and the difference is not
/// cosmetic. The **relative** mode compares whole-state distances:
/// `RobotState::distance(other, group)`, the sum over the group's active
/// joints of `JointModel::distance_factor() * JointModel::distance(..)`,
/// against `relative_factor` times that sum's average over the path. The
/// **absolute** mode compares one joint at a time:
/// `RobotState::distance(other, joint)` — the same per-joint distance but
/// *without* the distance factor — against
/// [`JumpThreshold::revolute`] or [`JumpThreshold::prismatic`] according to
/// that joint's type. A joint that is neither revolute nor prismatic is
/// skipped by the absolute mode entirely (upstream logs an `RCLCPP_WARN`
/// saying so; see the module docs' "Out of scope"), whereas the relative
/// mode's group distance includes it.
///
/// Upstream also warns when the relative mode is given fewer than
/// [`MIN_STEPS_FOR_JUMP_THRESH`] waypoints, because an average over so few
/// increments is a poor baseline. That is a real caveat on the result, not
/// on the call: it still computes, and this port still computes it.
pub fn has_joint_space_jump(
    waypoints: &[RobotState<'_>],
    group: &JointModelGroup,
    jump_threshold: &JumpThreshold,
) -> Option<usize> {
    if waypoints.len() <= 1 {
        return None;
    }
    if jump_threshold.relative_factor > 0.0 {
        return has_relative_joint_space_jump(waypoints, group, jump_threshold.relative_factor);
    }
    if jump_threshold.revolute > 0.0 || jump_threshold.prismatic > 0.0 {
        return has_absolute_joint_space_jump(
            waypoints,
            group,
            jump_threshold.revolute,
            jump_threshold.prismatic,
        );
    }
    None
}

/// Truncate `waypoints` at its first jump, returning the fraction of it that
/// survived.
///
/// Upstream `CartesianInterpolator::checkJointSpaceJump`. The fraction is
/// the surviving length over the length *before* truncation, so a path with
/// no jump returns `1.0` and is left alone.
///
/// # Panics
///
/// Never, for a fraction outside `[0, 1]`: [`has_joint_space_jump`] only
/// ever returns an index strictly inside `waypoints`. See
/// [`Percentage::new`].
pub fn check_joint_space_jump(
    waypoints: &mut Vec<RobotState<'_>>,
    group: &JointModelGroup,
    jump_threshold: &JumpThreshold,
) -> Percentage {
    match has_joint_space_jump(waypoints, group, jump_threshold) {
        Some(index) => {
            let solved = index as f64 / waypoints.len() as f64;
            waypoints.truncate(index);
            Percentage::new(solved)
        }
        None => Percentage::new(1.0),
    }
}

/// Upstream `hasRelativeJointSpaceJump`.
///
/// `threshold > total_dist / count` was upstream's own comparison, ported
/// faithfully — but a single NaN anywhere in `increments` (any waypoint
/// with a non-finite joint value) makes the sum, and therefore `threshold`,
/// NaN, and every NaN comparison is false: `increment > threshold` would
/// silently clear *every* increment, including the increments that are
/// themselves perfectly finite, treating a data point this function cannot
/// even evaluate as "no jump" rather than as the one thing a jump check
/// exists to catch. `threshold` is the single value the whole decision
/// funnels through, so checking it once here closes the family: a
/// non-finite `threshold` can only come from a non-finite `relative_factor`
/// or a non-finite increment already summed into `total`, and either way
/// the correct, conservative answer is "cannot verify this path is
/// jump-free" — reported at the earliest waypoint, exactly like a genuine
/// jump there would be.
fn has_relative_joint_space_jump(
    waypoints: &[RobotState<'_>],
    group: &JointModelGroup,
    relative_factor: f64,
) -> Option<usize> {
    let increments: Vec<f64> = waypoints
        .windows(2)
        .map(|pair| group_distance(&pair[1], &pair[0], group))
        .collect();
    let total: f64 = increments.iter().sum();
    let threshold = relative_factor * (total / increments.len() as f64);
    increments
        .iter()
        .position(|&increment| !threshold.is_finite() || increment > threshold)
        .map(|index| index + 1)
}

/// Upstream `hasAbsoluteJointSpaceJump`.
///
/// Same anchor as [`has_relative_joint_space_jump`], one level narrower:
/// here it is `distance` — computed fresh per `(waypoint, joint)` pair,
/// from that pair's own joint values — that can be NaN while
/// `revolute_threshold`/`prismatic_threshold` stay perfectly ordinary
/// (caller-supplied constants). `distance > threshold` then silently
/// clears just that one pair rather than the whole list, but the fix is
/// the same rule: a comparison this function cannot evaluate is not
/// evidence of "no jump".
fn has_absolute_joint_space_jump(
    waypoints: &[RobotState<'_>],
    group: &JointModelGroup,
    revolute_threshold: f64,
    prismatic_threshold: f64,
) -> Option<usize> {
    let check_revolute = revolute_threshold > 0.0;
    let check_prismatic = prismatic_threshold > 0.0;
    let model = waypoints[0].model();

    for i in 1..waypoints.len() {
        for &index in group.active_joint_indices() {
            let joint = model.joint_model_at(index);
            let distance = joint_distance(&waypoints[i], &waypoints[i - 1], joint);
            let exceeded = match joint.joint_type() {
                JointType::Revolute => {
                    check_revolute && (!distance.is_finite() || distance > revolute_threshold)
                }
                JointType::Prismatic => {
                    check_prismatic && (!distance.is_finite() || distance > prismatic_threshold)
                }
                // Upstream warns and skips; see this module's "Out of scope".
                _ => false,
            };
            if exceeded {
                return Some(i);
            }
        }
    }
    None
}

/// Upstream `RobotState::distance(const RobotState&, const
/// JointModelGroup*)`: the distance-factor-weighted sum over the group's
/// active joints.
fn group_distance(a: &RobotState<'_>, b: &RobotState<'_>, group: &JointModelGroup) -> f64 {
    let model = a.model();
    group
        .active_joint_indices()
        .iter()
        .map(|&index| {
            let joint = model.joint_model_at(index);
            joint.distance_factor() * joint_distance(a, b, joint)
        })
        .sum()
}

/// Upstream `RobotState::distance(const RobotState&, const JointModel*)`.
/// Note the absent distance factor: upstream's per-joint overload does not
/// apply one, only its per-group overload does.
fn joint_distance(a: &RobotState<'_>, b: &RobotState<'_>, joint: &JointModel) -> f64 {
    if joint.variable_count() == 0 {
        return 0.0;
    }
    let expect = "active joint of this state's own robot model";
    joint.distance(
        a.joint_position(joint.name()).expect(expect),
        b.joint_position(joint.name()).expect(expect),
    )
}

/// Upstream `RobotState::interpolate(to, t, state)`, the group-less overload
/// `validateAndImproveInterval` uses: every active joint of the whole model,
/// not just the group's.
fn interpolate_states<'m>(from: &RobotState<'m>, to: &RobotState<'m>, t: f64) -> RobotState<'m> {
    let model = from.model();
    let mut out = from.clone();
    let expect = "active joint of this state's own robot model";
    for &index in model.active_joint_indices() {
        let joint = model.joint_model_at(index);
        if joint.variable_count() == 0 {
            continue;
        }
        let mut buffer = vec![0.0; joint.variable_count()];
        joint.interpolate(
            from.joint_position(joint.name()).expect(expect),
            to.joint_position(joint.name()).expect(expect),
            t,
            &mut buffer,
        );
        out.set_joint_positions(joint.name(), &buffer)
            .expect(expect);
    }
    out
}

/// The pose `t` of the way along the Cartesian straight line from `from` to
/// `to`: slerp for the rotation, lerp for the translation.
///
/// Upstream writes this out twice, identically — once in the main loop
/// (`cartesian_interpolator.cpp:257-258`) and once for the bisection's
/// midpoint (`:78-79`).
fn interpolate_pose(from: &Isometry3, to: &Isometry3, t: f64) -> Isometry3 {
    // `crate::geometry::quaternion::slerp` rather than nalgebra's, which is a
    // different function in three measured ways — see its doc comment. The
    // `nlerp` fallback that used to stand in for nalgebra's ~180-degree panic
    // is gone with the call that could panic; Eigen has no degenerate case
    // there and neither does the transcription.
    let rotation = quaternion::slerp(&from.rotation, &to.rotation, t);
    // nalgebra's `lerp` *is* upstream's `percentage * b + (1 - percentage) *
    // a`, not the `a + (b - a) * t` the name suggests: it forwards to
    // `axpy(t, rhs, 1 - t)`, which is `t*rhs + (1-t)*self` termwise. The two
    // spellings are the same function over the reals and different f64
    // programs — `a + (b - a)` misses `b` at `t == 1` in 2040 of 8405
    // measured metre-scale pairs, and departs by up to 4.44e-16 in the
    // interior — so which one this is cannot be left to a library name. The
    // tests at the bottom of this file pin it; see PORTING-PLAN.md §239.3.
    let translation = from.translation.vector.lerp(&to.translation.vector, t);
    Isometry3::from_parts(translation.into(), rotation)
}

/// Upstream's `for (const JointModel* joint : group->getContinuousJointModels())
/// state.enforceBounds(joint)`: wrap every continuous revolute joint of the
/// group back into `[-pi, pi]` so IK is seeded from a canonical
/// representation.
///
/// `getContinuousJointModels()` is the group's active revolute joints whose
/// `isContinuous()` holds (`robot_model/src/joint_model_group.cpp:170-172`); `enforceBounds`
/// on one joint enforces its position bounds and, when the state carries
/// velocities, its velocity bounds too
/// (`moveit/robot_state/robot_state.hpp:1400-1405`).
fn enforce_continuous_joint_bounds(state: &mut RobotState<'_>, group: &JointModelGroup) {
    let model: &RobotModel = state.model();
    let expect = "active joint of this state's own robot model";
    let continuous: Vec<&JointModel> = group
        .active_joint_indices()
        .iter()
        .map(|&index| model.joint_model_at(index))
        .filter(|joint| {
            joint
                .as_revolute()
                .is_some_and(crate::model::joint::RevoluteJoint::is_continuous)
        })
        .collect();

    for joint in continuous {
        let mut positions = state.joint_position(joint.name()).expect(expect).to_vec();
        if joint.enforce_position_bounds(&mut positions) {
            state
                .set_joint_positions(joint.name(), &positions)
                .expect(expect);
        }
        if state.has_velocities() {
            // A continuous joint is revolute, so it has exactly one
            // variable and that variable's name is the joint's own
            // (`JointModel::new_single_variable`).
            let mut velocity = [state.variable_velocity(joint.name()).expect(expect)];
            if joint.enforce_velocity_bounds(&mut velocity) {
                state
                    .set_variable_velocity(joint.name(), velocity[0])
                    .expect(expect);
            }
        }
    }
}

/// `floor(value)` as a waypoint count. A negative, NaN or non-finite ratio
/// contributes no steps; `as usize` already saturates rather than wrapping,
/// so an absurdly small `max_step` cannot turn into a small count.
fn saturating_floor(value: f64) -> usize {
    let floored = value.floor();
    if floored.is_nan() || floored <= 0.0 {
        0
    } else {
        floored as usize
    }
}

#[cfg(test)]
/// [`interpolate_pose`]'s translation blend, pinned as an f64 program
/// rather than as a library call.
///
/// `PORTING-PLAN.md` §238.5 read this as a divergence — the port calling
/// `a + (b - a) * t` where upstream computes `percentage * b + (1 -
/// percentage) * a`. It is not: nalgebra's `Vector::lerp` forwards to
/// `axpy(t, rhs, 1 - t)` and is upstream's expression termwise. But the
/// name says otherwise, the two spellings *are* different f64 programs, and
/// nothing in the tree checked which one this was — so the three cases
/// below assert the arithmetic directly and refuse the other spelling.
mod tests {
    use super::*;
    use crate::geometry::UnitQuaternion;

    fn pose(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::from_parts(Vector3::new(x, y, z).into(), UnitQuaternion::identity())
    }

    /// One `(from, to)` pair on each axis for which `from + (to - from)`
    /// does **not** round back to `to`, found by sweeping 401×401 pairs at
    /// metre-scale magnitudes. Every coordinate has to be one of those:
    /// against a pair where the two forms agree, the assertions below hold
    /// under either expression and pin nothing (measured — with an
    /// arbitrarily chosen pair they did exactly that).
    const NEAR: [f64; 3] = [-27.400000000000002, -27.400000000000002, -2.74];
    const FAR: [f64; 3] = [-11.348999999999998, -9.894, 5.819999999999999];

    /// `t == 1` is the last waypoint of every path
    /// [`CartesianInterpolator`] generates, and upstream's `1*to + 0*from`
    /// is exactly `to` there. `from + (to - from)*1` is not: the subtraction
    /// and the addition round separately, so `x` comes back `-11.349`
    /// against a target of `-11.348999999999998`.
    ///
    /// Not a tolerance question — an interpolator whose final waypoint is
    /// not the pose it was asked for has a different contract, whatever the
    /// size of the miss.
    #[test]
    fn the_last_waypoint_is_exactly_the_target_pose() {
        let (from, to) = (
            pose(NEAR[0], NEAR[1], NEAR[2]),
            pose(FAR[0], FAR[1], FAR[2]),
        );
        let end = interpolate_pose(&from, &to, 1.0);
        assert_eq!(end.translation.vector, to.translation.vector);
    }

    /// The other endpoint, which `lerp` gets exactly right and the two-term
    /// form has to be checked for: `0*to + 1*from` must not round `from`.
    #[test]
    fn the_first_waypoint_is_exactly_the_start_pose() {
        let (from, to) = (
            pose(NEAR[0], NEAR[1], NEAR[2]),
            pose(FAR[0], FAR[1], FAR[2]),
        );
        let start = interpolate_pose(&from, &to, 0.0);
        assert_eq!(start.translation.vector, from.translation.vector);
    }

    /// The interior, bit-for-bit against the C++ expression rather than
    /// within a tolerance — a tolerance here would accept exactly the `lerp`
    /// this test exists to exclude, whose largest measured departure over
    /// this grid is 4.44e-16.
    ///
    /// The `differed` count is asserted, not just the agreement: it is what
    /// says the chosen coordinates reach a `t` where the two expressions are
    /// different f64 programs at all.
    #[test]
    fn the_interior_matches_upstreams_two_term_blend_bitwise() {
        let (from, to) = (
            pose(NEAR[0], NEAR[1], NEAR[2]),
            pose(FAR[0], FAR[1], FAR[2]),
        );
        let mut differed = 0usize;
        for i in 1..100u32 {
            let t = f64::from(i) / 100.0;
            let got = interpolate_pose(&from, &to, t).translation.vector;
            for axis in 0..3 {
                let (a, b) = (from.translation.vector[axis], to.translation.vector[axis]);
                assert_eq!(got[axis], t * b + (1.0 - t) * a, "axis {axis} at t={t}");
                if t * b + (1.0 - t) * a != a + (b - a) * t {
                    differed += 1;
                }
            }
        }
        assert!(
            differed > 0,
            "these coordinates never separate the two blends, so the assertion above \
             would hold under either"
        );
    }

    /// Regression coverage for the `max_resolution` fix in
    /// `validate_and_improve_interval`: needs `PathRun`/`Waypoint`
    /// directly, which are private, so this lives here rather than in an
    /// integration test.
    ///
    /// `to_pose`'s own loop always starts an interval at `width = 1.0`, and
    /// `width` only ever halves after that -- it never independently
    /// carries a caller-controlled non-finite value. So this calls
    /// `validate_and_improve_interval` directly with an artificially tiny
    /// starting `width` (`1e-15`, far below any `max_resolution` a caller
    /// would configure) on real, reachable `panda_arm` geometry that does
    /// need genuine bisection to resolve (confirmed separately: with
    /// `width` at its natural `1.0`, this exact `start`/`target` pair
    /// recurses deeply enough to leave the interval unresolved, i.e. the
    /// bisection safety valve is load-bearing here, not decorative).
    ///
    /// Before this fix: on this exact `(start, target, width)`, the `NAN`
    /// case reaches a stack overflow (verified directly against the
    /// pre-fix branch, `width < self.config.precision.max_resolution` with
    /// no finiteness check -- not reproduced here, since a crashing test
    /// would abort the whole binary rather than fail it). The `1e-5` case
    /// beside it is unaffected by the fix either way: `width(1e-15) <
    /// max_resolution(1e-5)` was already true, so it already returned
    /// `Ok(false)` immediately, with no recursion and no IK call. After
    /// this fix, `NAN` takes the same immediate path, for the same
    /// reason `1e-5` does: neither can be trusted to bound the recursion,
    /// so neither is allowed to.
    #[test]
    fn a_non_finite_max_resolution_stops_the_recursion_instead_of_never_stopping_it() {
        use crate::kinematics::set_from_ik::IkContext;
        use crate::kinematics::{NewtonRaphsonSolver, SolverParams};
        use crate::model::{MeshSearchPaths, RobotModel};
        use crate::srdf::SrdfModel;
        use crate::state::RobotState;
        use std::fs;

        fn fixture_path(file_name: &str) -> String {
            format!(
                concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/kinematics/{}"),
                file_name
            )
        }
        let urdf_path = fixture_path("panda.urdf");
        let srdf_path = fixture_path("panda.srdf");
        let urdf_xml = fs::read_to_string(&urdf_path).expect("fixture URDF must be readable");
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        let mut solver = NewtonRaphsonSolver::new(&model, "panda_arm", &SolverParams::default())
            .expect("panda_arm is a chain");
        let joint_names = solver.joint_names().to_vec();
        let tip = solver.tip_frame().to_owned();

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        const START: [f64; 7] = [0.0, -0.4, 0.0, -1.9, 0.0, 1.6, 0.75];
        for (name, value) in joint_names.iter().zip(START) {
            state
                .set_variable_position(name, value)
                .expect("panda_arm joint");
        }
        let start_pose = state
            .clone()
            .update()
            .global_link_transform(&tip)
            .expect("tip link");
        let mut target = start_pose;
        target.translation.vector += Vector3::new(0.10, 0.05, 0.02);

        let mut ik = IkContext::default();
        for max_resolution in [1e-5_f64, f64::NAN] {
            let config = CartesianInterpolator {
                precision: CartesianPrecision {
                    max_resolution,
                    ..CartesianPrecision::default()
                },
                ..CartesianInterpolator::new("panda_arm", &tip, MaxEefStep::from_step_size(0.5))
            };
            let mut run = PathRun {
                config: &config,
                inv_offset: config.link_offset.inverse(),
                solver: &mut solver,
                ik: &mut ik,
                traj: vec![],
                achieved: 0.0,
            };
            let start_wp = Waypoint {
                state: state.clone(),
                pose: start_pose,
            };
            let end_wp = Waypoint {
                state: state.clone(),
                pose: target,
            };
            let result = run.validate_and_improve_interval(&start_wp, &end_wp, 1.0, 1e-15);

            assert!(
                !result.expect("no IK call is reachable past the width check"),
                "max_resolution={max_resolution:?}: an unresolvable-in-time-to-matter width \
                 check must reject the interval outright, not recurse"
            );
            assert!(
                run.traj.is_empty(),
                "max_resolution={max_resolution:?}: rejecting at the width check must not push \
                 any waypoint"
            );
        }
    }
}
