// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_functions.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_functions.cpp

//! IK/FK round-trip, joint-limit verification and Cartesian-to-joint
//! trajectory generation shared by every Pilz trajectory generator
//! (`LIN`/`PTP`/`CIRC`, none of which are in this crate's scope yet).
//!
//! # Reuse, not reimplementation
//!
//! Per this round's task instruction, IK/FK are not reimplemented here —
//! every function below is a thin orchestration layer over crates this
//! workspace already ports:
//!
//! - [`compute_pose_ik`] delegates the actual Newton iteration to a
//!   [`moveit_kinematics::KinematicsSolver`] the caller constructs (e.g.
//!   [`moveit_kinematics::NewtonRaphsonSolver`]). Upstream's self-collision
//!   check (`ik_constraint_function`, upstream's
//!   `GroupStateValidityCallbackFn`, wired into `RobotState::setFromIK`) maps
//!   exactly onto [`moveit_kinematics::SolveOptions::solution_callback`] —
//!   "called on every attempt that converges numerically, reject to retry"
//!   is precisely upstream's `IKCallbackFn` contract, so [`compute_pose_ik`]
//!   passes [`is_state_colliding`] through that hook rather than
//!   re-implementing IK's retry loop around a collision check of its own.
//! - [`compute_link_fk`] delegates to [`moveit_state::Posed::frame_transform`]
//!   (reached through [`moveit_state::RobotState::update`]) — this crate adds
//!   no forward-kinematics math of its own.
//! - [`is_state_colliding`] delegates to
//!   [`moveit_scene::PlanningScene::check_self_collision`], generic over the
//!   same `E: CollisionEnv` the caller already has (a
//!   [`moveit_collision::ParryCollisionEnv`], typically).
//!
//! # Deviations from upstream
//!
//! 1. **No per-group configured solver.** Upstream resolves a `JointModelGroup`
//!    to its one `kinematics::KinematicsBase` instance via `getSolverInstance()`
//!    — a mapping fixed at model-load time from SRDF `kinematics.yaml`. This
//!    port does not carry that mapping (`moveit-model`'s `JointModelGroup` has
//!    no solver field — nothing in this workspace's `RobotModel` port loads
//!    `kinematics.yaml`), so [`compute_pose_ik`] and
//!    [`generate_joint_trajectory`]/[`generate_joint_trajectory_from_cartesian`]
//!    take an already-constructed `&mut dyn KinematicsSolver` instead of
//!    looking one up from `group_name`. [`compute_pose_ik`] still checks
//!    `solver.tip_frame() == link_name`, the one piece of upstream's
//!    `setFromIK`/`canSetStateFromIK` link-matching this port can perform
//!    without that mapping.
//! 2. **No `KDL::Trajectory` equivalent exists yet.** Upstream's first
//!    `generateJointTrajectory` overload samples a concrete `KDL::Trajectory`
//!    (position + orientation as a function of time, built from a path plus a
//!    velocity profile). This port has no KDL dependency (`PORTING-PLAN.md`
//!    D1/D2) and the Cartesian path/velocity-profile machinery that would
//!    produce one is `trajectory_generator_lin`/`_circ`'s job, out of this
//!    round's scope (round 17 already deferred the analogous `KDL::Path_Circle`
//!    interpolation for the same reason — see `path_circle`'s module doc).
//!    [`CartesianPath`] names exactly the two operations
//!    `generateJointTrajectory`'s body actually calls on its `trajectory`
//!    parameter (`Duration()`, `Pos(t)`), so a future Cartesian path type can
//!    implement it without this function's signature changing.
//! 3. **Output is a [`moveit_trajectory::RobotTrajectory`], not a
//!    `trajectory_msgs::msg::JointTrajectory`.** No ROS message types are
//!    ported at all (D1/D2), and upstream's own caller
//!    (`TrajectoryGenerator::setSuccessResponse`) immediately converts the
//!    `JointTrajectory` this function returns into a `RobotTrajectory` via
//!    `setRobotTrajectoryMsg` — itself D1-excluded from `moveit-trajectory`
//!    (see that crate's `robot_trajectory.hpp` symbol audit). Building the
//!    `RobotTrajectory` directly, one `RobotState` waypoint at a time via
//!    [`moveit_trajectory::RobotTrajectory::add_suffix_way_point`], skips a
//!    message-shaped intermediate this port has no reason to reconstruct —
//!    the `moveit_msgs::msg::MoveItErrorCodes` two-value failure classification
//!    (`NO_IK_SOLUTION` / `PLANNING_FAILED`) survives as
//!    [`moveit_error::MoveItErrorCode::NoIkSolution`] /
//!    [`moveit_error::MoveItErrorCode::PlanningFailed`], wrapped in
//!    [`moveit_error::Error::Code`].
//! 4. **`normalizeQuaternion` is not ported.** It exists upstream to
//!    re-normalize a `geometry_msgs::msg::Quaternion` after a `tf2::fromMsg`
//!    round-trip, because that message type carries no normalization
//!    invariant of its own. This port's quaternions are always
//!    [`moveit_geometry::UnitQuaternion`] (`nalgebra::UnitQuaternion`), which
//!    is normalized by construction — there is no un-normalized
//!    representation for this function to ever be called on.
//! 5. **`getConstraintPose`'s message-decoding overload is not ported; its
//!    geometry is, as [`constraint_pose`].** The `moveit_msgs::msg::Constraints`
//!    overload (`getConstraintPose(const Constraints&)`) only extracts fields
//!    from a message this port does not have — dropped. The
//!    `Point`/`Quaternion`/`Vector3` overload's actual computation (goal pose
//!    from position + orientation, offset by `target_point_offset` expressed
//!    in the goal's own rotated frame) is real geometry a future goal-pose
//!    extraction (this round's [`crate::trajectory_generator`] or a later
//!    LIN/PTP/CIRC round) still needs, so it is ported as [`constraint_pose`]
//!    taking [`moveit_geometry::Vector3`]/[`moveit_geometry::UnitQuaternion`]
//!    directly instead of `geometry_msgs` fields.

use std::collections::HashMap;
use std::sync::Arc;

use moveit_collision::{CollisionEnv, CollisionRequest};
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use moveit_kinematics::{KinematicsSolver, SolveOptions};
use moveit_scene::PlanningScene;
use moveit_state::{Posed, RobotState};
use moveit_trajectory::RobotTrajectory;

use crate::cartesian_trajectory::CartesianTrajectory;
use crate::limits::JointLimitsContainer;

/// A Cartesian path sampled by elapsed time. See this module's `# Deviations`,
/// item 2, for why this trait exists instead of a concrete `KDL::Trajectory`
/// port.
pub trait CartesianPath {
    /// Upstream `KDL::Trajectory::Duration()`.
    fn duration(&self) -> f64;
    /// Upstream `KDL::Trajectory::Pos(double)`.
    fn pos(&self, t: f64) -> Isometry3;
}

/// The planning context every IK/trajectory-generation call in this module
/// shares: the scene IK candidates are checked against, the collision
/// backend, and whether a self-colliding candidate should be rejected (and
/// retried, subject to the solver's own restart budget) instead of accepted.
///
/// Grouping these three keeps [`compute_pose_ik`]/[`generate_joint_trajectory`]/
/// [`generate_joint_trajectory_from_cartesian`] at 7 parameters or fewer —
/// `tools/ci/check-no-lint-suppression.sh` forbids reaching for
/// `#[allow(clippy::too_many_arguments)]` instead, and every one of these
/// three genuinely travels together for the life of one `generate` call, so
/// this is not a bag of unrelated parameters shoved into a struct to dodge
/// the lint.
pub struct IkContext<'a, 'm, E> {
    /// The scene IK candidates are checked against. [`compute_pose_ik`]
    /// probes a scratch copy from [`PlanningScene::diff`] rather than this
    /// scene directly, so it is never mutated by a rejected candidate.
    pub scene: &'a Arc<PlanningScene<'m>>,
    /// The collision backend.
    pub env: &'a E,
    /// Whether to reject a self-colliding IK candidate.
    pub check_self_collision: bool,
}

/// Compute the inverse kinematics of `pose`, optionally rejecting a
/// self-colliding solution.
///
/// `solver` must already be built for the group/tip this call targets (see
/// this module's `# Deviations`, item 1); `link_name` is checked against
/// `solver.tip_frame()` rather than used to look one up.
///
/// `seed`'s entries override the corresponding variables of `ctx.scene`'s
/// current state; any variable [`KinematicsSolver::joint_names`] needs that
/// `seed` does not mention keeps that current-state value — matching
/// upstream's `rstate.setVariablePositions(seed)` starting from
/// `scene->getCurrentState()`.
///
/// Upstream `computePoseIK`. Returns `None` where upstream returns `false`
/// (unknown group, `frame_id` mismatch, tip mismatch, or no IK solution) —
/// every one of those is an ordinary negative outcome upstream itself never
/// treats as an exception.
pub fn compute_pose_ik<'m, E>(
    ctx: &IkContext<'_, 'm, E>,
    solver: &mut dyn KinematicsSolver,
    link_name: &str,
    pose: &Isometry3,
    frame_id: &str,
    seed: &HashMap<String, f64>,
) -> Option<HashMap<String, f64>>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    let robot_model = ctx.scene.robot_model();
    if !robot_model.has_joint_model_group(solver.group_name()) {
        return None;
    }
    if frame_id != robot_model.model_frame() {
        return None;
    }
    if solver.tip_frame() != link_name {
        return None;
    }

    let mut probe = ctx.scene.diff();
    probe
        .current_state_mut()
        .set_variable_positions_by_name(seed)
        .ok()?;

    let seed_vec: Vec<f64> = {
        let state = probe.current_state();
        solver
            .joint_names()
            .iter()
            .map(|name| state.variable_position(name).unwrap_or(0.0))
            .collect()
    };

    let joint_names = solver.joint_names().to_vec();
    let group_name = solver.group_name().to_string();
    let mut collision_check = move |candidate: &[f64]| -> bool {
        if !ctx.check_self_collision {
            return true;
        }
        !is_state_colliding(&mut probe, ctx.env, &group_name, &joint_names, candidate)
    };
    let mut options = SolveOptions {
        solution_callback: Some(&mut collision_check),
        ..Default::default()
    };

    let solution = solver.solve_with_options(&seed_vec, pose, &mut options)?;
    Some(solver.joint_names().iter().cloned().zip(solution).collect())
}

/// Compute the pose of `link_name` at `joint_state`.
///
/// Upstream `computeLinkFK(RobotState&, ..., const map<string,double>&, ...)`.
/// A second overload taking `joint_names`/`joint_positions` as two lockstep
/// `Vec`s is declared in `trajectory_functions.hpp` but has no definition
/// anywhere in upstream and is never called (every call site in
/// `trajectory_generator_{circ,lin,polyline}.cpp` passes a
/// `std::map<std::string, double>`, matching the overload ported here) — it
/// is dead upstream code, not a forwarding adapter, so there is nothing to
/// port.
pub fn compute_link_fk<'m>(
    robot_state: &mut RobotState<'m>,
    link_name: &str,
    joint_state: &HashMap<String, f64>,
) -> Option<Isometry3> {
    if !robot_state.knows_frame_transform(link_name) {
        return None;
    }
    robot_state
        .set_variable_positions_by_name(joint_state)
        .ok()?;
    robot_state.update().frame_transform(link_name).ok()
}

/// Verify the current sample's velocity/acceleration against `joint_limits`,
/// via backward difference:
///
/// `v(k) = [x(k) - x(k-1)] / [t(k) - t(k-1)]`
/// `a(k) = [v(k) - v(k-1)] / [t(k) - t(k-2)] * 2`
///
/// Upstream `verifySampleJointLimits`.
///
/// # Panics
///
/// If `position_current` names a joint absent from `position_last` or
/// `velocity_last` — matching upstream's unchecked `.at(...)`, which throws
/// `std::out_of_range` for the same mismatch.
pub fn verify_sample_joint_limits(
    position_last: &HashMap<String, f64>,
    velocity_last: &HashMap<String, f64>,
    position_current: &HashMap<String, f64>,
    duration_last: f64,
    duration_current: f64,
    joint_limits: &JointLimitsContainer,
) -> bool {
    const EPSILON: f64 = 10e-6;
    if duration_current <= EPSILON {
        return false;
    }

    for (name, &pos) in position_current {
        let last_pos = position_last[name];
        let velocity_current = (pos - last_pos) / duration_current;
        if !joint_limits.verify_velocity_limit(name, velocity_current) {
            return false;
        }

        let last_velocity = velocity_last[name];
        let acceleration_current =
            (velocity_current - last_velocity) / (duration_last + duration_current) * 2.0;
        let within_limit = if last_velocity.abs() <= velocity_current.abs() {
            joint_limits.verify_acceleration_limit(name, acceleration_current)
        } else {
            joint_limits.verify_deceleration_limit(name, acceleration_current)
        };
        if !within_limit {
            return false;
        }
    }

    true
}

/// One [`RobotTrajectory`] waypoint's positions/velocities/accelerations, all
/// keyed by joint name — the shape [`generate_joint_trajectory`] and
/// [`generate_joint_trajectory_from_cartesian`] both build per sample before
/// pushing it via [`RobotTrajectory::add_suffix_way_point`].
///
/// `reference_state` seeds every variable this waypoint does not itself set
/// (`positions`/`velocities`/`accelerations` only ever cover the IK group's
/// own joints). Upstream's `RobotTrajectory::setRobotTrajectoryMsg` — the
/// method that would otherwise build these waypoints, see this module's
/// `# Deviations`, item 3 — copy-constructs each waypoint's `RobotState`
/// from a caller-supplied `reference_state` before overwriting the group's
/// variables (`robot_trajectory.cpp`: `auto st =
/// std::make_shared<RobotState>(copy)` where `copy` is that reference). A
/// waypoint built from a bare [`RobotState::new`] instead — every variable
/// raw `0.0` — is wrong for any joint whose zero is not a valid default: a
/// floating joint's quaternion at all-zero is not a unit quaternion, and FK
/// through it is NaN.
pub(crate) fn push_way_point<'m>(
    trajectory: &mut RobotTrajectory<'m>,
    reference_state: &RobotState<'m>,
    positions: &HashMap<String, f64>,
    velocities: &HashMap<String, f64>,
    accelerations: &HashMap<String, f64>,
    dt: f64,
) -> Result<()> {
    let mut state = reference_state.clone();
    state.set_variable_positions_by_name(positions)?;
    for (name, &value) in velocities {
        state.set_variable_velocity(name, value)?;
    }
    for (name, &value) in accelerations {
        state.set_variable_acceleration(name, value)?;
    }
    trajectory.add_suffix_way_point(state, dt)?;
    Ok(())
}

/// Generate a joint trajectory by sampling `trajectory` (a Cartesian path,
/// see [`CartesianPath`]) at `sampling_time` intervals and solving IK at each
/// sample.
///
/// Upstream `generateJointTrajectory(..., const KDL::Trajectory&, ...)`. See
/// this module's `# Deviations`, items 2 and 3.
///
/// # Errors
///
/// [`MoveItErrorCode::NoIkSolution`] if IK fails to converge (optionally,
/// self-collision-free) at any sample; [`MoveItErrorCode::PlanningFailed`] if
/// a sample violates `joint_limits`.
pub fn generate_joint_trajectory<'m, E, P: CartesianPath>(
    ctx: &IkContext<'_, 'm, E>,
    solver: &mut dyn KinematicsSolver,
    joint_limits: &JointLimitsContainer,
    trajectory: &P,
    link_name: &str,
    initial_joint_position: &HashMap<String, f64>,
    sampling_time: f64,
) -> Result<RobotTrajectory<'m>>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    const EPSILON: f64 = 10e-06;
    let duration = trajectory.duration();
    let mut time_samples = Vec::new();
    let mut t = 0.0;
    while t < duration - EPSILON {
        time_samples.push(t);
        t += sampling_time;
    }
    time_samples.push(duration);

    let robot_model = ctx.scene.robot_model();
    let mut out = RobotTrajectory::for_group_name(robot_model, solver.group_name())
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;

    let mut ik_solution_last = initial_joint_position.clone();
    let mut joint_velocity_last: HashMap<String, f64> = ik_solution_last
        .keys()
        .map(|name| (name.clone(), 0.0))
        .collect();

    for (i, &t) in time_samples.iter().enumerate() {
        let pose_sample = trajectory.pos(t);
        let ik_solution = compute_pose_ik(
            ctx,
            solver,
            link_name,
            &pose_sample,
            robot_model.model_frame(),
            &ik_solution_last,
        )
        .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

        let is_last = i == time_samples.len() - 1;
        let duration_current_sample = if is_last && time_samples.len() > 1 {
            t - time_samples[i - 1]
        } else if time_samples.len() == 1 {
            t
        } else {
            sampling_time
        };

        if i != 0
            && !verify_sample_joint_limits(
                &ik_solution_last,
                &joint_velocity_last,
                &ik_solution,
                sampling_time,
                duration_current_sample,
                joint_limits,
            )
        {
            return Err(Error::Code(MoveItErrorCode::PlanningFailed));
        }

        let is_first = i == 0;
        let mut velocities = HashMap::new();
        let mut accelerations = HashMap::new();
        if !is_first && !is_last {
            for (name, &value) in &ik_solution {
                let velocity = (value - ik_solution_last[name]) / duration_current_sample;
                accelerations.insert(
                    name.clone(),
                    (velocity - joint_velocity_last[name])
                        / (duration_current_sample + sampling_time)
                        * 2.0,
                );
                velocities.insert(name.clone(), velocity);
            }
        } else {
            for name in ik_solution.keys() {
                velocities.insert(name.clone(), 0.0);
                accelerations.insert(name.clone(), 0.0);
            }
        }
        joint_velocity_last = velocities.clone();

        let dt = if is_first {
            0.0
        } else {
            duration_current_sample
        };
        push_way_point(
            &mut out,
            ctx.scene.current_state(),
            &ik_solution,
            &velocities,
            &accelerations,
            dt,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::PlanningFailed))?;

        ik_solution_last = ik_solution;
    }

    Ok(out)
}

/// Generate a joint trajectory from a pre-sampled [`CartesianTrajectory`],
/// solving IK at each of its points.
///
/// Upstream `generateJointTrajectory(..., const CartesianTrajectory&, ...)`.
/// See this module's `# Deviations`, item 3.
///
/// # Errors
///
/// [`MoveItErrorCode::NoIkSolution`] if IK fails to converge at any point;
/// [`MoveItErrorCode::PlanningFailed`] if a point violates `joint_limits`.
pub fn generate_joint_trajectory_from_cartesian<'m, E>(
    ctx: &IkContext<'_, 'm, E>,
    solver: &mut dyn KinematicsSolver,
    joint_limits: &JointLimitsContainer,
    trajectory: &CartesianTrajectory,
    link_name: &str,
    initial_joint_position: &HashMap<String, f64>,
    initial_joint_velocity: &HashMap<String, f64>,
) -> Result<RobotTrajectory<'m>>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    let robot_model = ctx.scene.robot_model();
    let mut out = RobotTrajectory::for_group_name(robot_model, solver.group_name())
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;

    let mut ik_solution_last = initial_joint_position.clone();
    let mut joint_velocity_last = initial_joint_velocity.clone();
    let mut duration_last = 0.0;

    for (i, point) in trajectory.points.iter().enumerate() {
        let ik_solution = compute_pose_ik(
            ctx,
            solver,
            link_name,
            &point.pose,
            robot_model.model_frame(),
            &ik_solution_last,
        )
        .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

        let duration_current = if i == 0 {
            duration_last = point.time_from_start;
            point.time_from_start
        } else {
            point.time_from_start - trajectory.points[i - 1].time_from_start
        };

        if !verify_sample_joint_limits(
            &ik_solution_last,
            &joint_velocity_last,
            &ik_solution,
            duration_last,
            duration_current,
            joint_limits,
        ) {
            return Err(Error::Code(MoveItErrorCode::PlanningFailed));
        }

        let mut velocities = HashMap::new();
        let mut accelerations = HashMap::new();
        for (name, &value) in &ik_solution {
            let velocity = (value - ik_solution_last[name]) / duration_current;
            accelerations.insert(
                name.clone(),
                (velocity - joint_velocity_last[name]) / (duration_current + duration_last) * 2.0,
            );
            velocities.insert(name.clone(), velocity);
        }
        joint_velocity_last = velocities.clone();

        let dt = if i == 0 { 0.0 } else { duration_current };
        push_way_point(
            &mut out,
            ctx.scene.current_state(),
            &ik_solution,
            &velocities,
            &accelerations,
            dt,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::PlanningFailed))?;

        ik_solution_last = ik_solution;
        duration_last = duration_current;
    }

    Ok(out)
}

/// Determine the sampling time shared by `first_trajectory`/`second_trajectory`
/// and check that both use it consistently, ignoring each trajectory's last
/// sample (allowed to violate it — the closing partial interval).
///
/// Upstream `determineAndCheckSamplingTime`. Returns `None` where upstream
/// returns `false`, and the resolved sampling time (upstream's `sampling_time`
/// out-parameter) as `Some` on success.
pub fn determine_and_check_sampling_time(
    first_trajectory: &RobotTrajectory,
    second_trajectory: &RobotTrajectory,
    epsilon: f64,
) -> Option<f64> {
    let n1 = first_trajectory.way_point_count() - 1;
    let n2 = second_trajectory.way_point_count() - 1;
    if n1 < 2 && n2 < 2 {
        return None;
    }

    let sampling_time = if n1 >= 2 {
        first_trajectory.way_point_duration_from_previous(1)
    } else {
        second_trajectory.way_point_duration_from_previous(1)
    };

    for i in 1..n1.max(n2) {
        if i < n1
            && (sampling_time - first_trajectory.way_point_duration_from_previous(i)).abs()
                > epsilon
        {
            return None;
        }
        if i < n2
            && (sampling_time - second_trajectory.way_point_duration_from_previous(i)).abs()
                > epsilon
        {
            return None;
        }
    }

    Some(sampling_time)
}

/// Sum of per-variable differences between two states' position, velocity and
/// acceleration, restricted to `group`'s active variables — three checks
/// folded into one boolean, matching upstream's three early-return `if`s.
fn group_state_norm_within<'m>(
    state1: &RobotState<'m>,
    state2: &RobotState<'m>,
    group: &str,
    epsilon: f64,
    extract: impl Fn(&RobotState<'m>, &str) -> Option<f64>,
) -> bool {
    let Ok(names) = state1
        .model()
        .joint_model_group(group)
        .map(|g| g.active_joint_names())
    else {
        return false;
    };
    let mut sum_sq = 0.0;
    for name in names {
        let a = extract(state1, name).unwrap_or(0.0);
        let b = extract(state2, name).unwrap_or(0.0);
        sum_sq += (a - b) * (a - b);
    }
    sum_sq.sqrt() <= epsilon
}

/// Check whether `state1`/`state2` have the same position, velocity and
/// acceleration over `joint_group_name`'s active joints, within `epsilon`
/// (the Euclidean norm of the per-joint difference, matching upstream's
/// `Eigen::VectorXd` `.norm()`).
///
/// Upstream `isRobotStateEqual`. Only single-variable active joints are
/// summed correctly here — see `# Deviations` above this module's IK/FK
/// section: every group this crate's callers use is a serial arm with
/// revolute/prismatic active joints, matching upstream's own assumption that
/// `copyJointGroupPositions` (one `double` per active *variable*, which this
/// port reads via [`RobotState::joint_position`] etc., one `f64` slice per
/// active *joint*) never sees a multi-variable active joint in this crate's
/// use.
pub fn is_robot_state_equal<'m>(
    state1: &RobotState<'m>,
    state2: &RobotState<'m>,
    joint_group_name: &str,
    epsilon: f64,
) -> bool {
    group_state_norm_within(state1, state2, joint_group_name, epsilon, |s, name| {
        s.variable_position(name).ok()
    }) && group_state_norm_within(state1, state2, joint_group_name, epsilon, |s, name| {
        s.variable_velocity(name).ok()
    }) && group_state_norm_within(state1, state2, joint_group_name, epsilon, |s, name| {
        s.variable_acceleration(name).ok()
    })
}

/// Check whether `state` has zero velocity and acceleration over `group`'s
/// active joints, within `epsilon`.
///
/// Upstream `isRobotStateStationary`.
pub fn is_robot_state_stationary<'m>(state: &RobotState<'m>, group: &str, epsilon: f64) -> bool {
    let Ok(names) = state
        .model()
        .joint_model_group(group)
        .map(|g| g.active_joint_names())
    else {
        return false;
    };
    let velocity_sq: f64 = names
        .iter()
        .map(|name| state.variable_velocity(name).unwrap_or(0.0).powi(2))
        .sum();
    if velocity_sq.sqrt() > epsilon {
        return false;
    }
    let acceleration_sq: f64 = names
        .iter()
        .map(|name| state.variable_acceleration(name).unwrap_or(0.0).powi(2))
        .sum();
    acceleration_sq.sqrt() <= epsilon
}

/// Linear search for the waypoint index at which `link_name`'s trajectory
/// crosses the blending sphere centred at `center_position` with radius `r`.
///
/// `inverse_order`: scan from the last waypoint backward (farthest from the
/// sphere's center sits at the smallest index) instead of forward.
///
/// Upstream `linearSearchIntersectionPoint`. Returns `None` where upstream
/// returns `false` (no crossing found).
pub fn linear_search_intersection_point<'m>(
    link_name: &str,
    center_position: &Vector3,
    r: f64,
    traj: &mut RobotTrajectory<'m>,
    inverse_order: bool,
) -> Option<usize> {
    let waypoint_num = traj.way_point_count();
    if waypoint_num == 0 {
        return None;
    }

    let translation_at = |traj: &mut RobotTrajectory<'m>, i: usize| -> Vector3 {
        traj.way_point_mut(i)
            .expect("index within way_point_count")
            .update()
            .frame_transform(link_name)
            .expect("link_name resolves for every waypoint of the same trajectory")
            .translation
            .vector
    };

    if inverse_order {
        for i in (1..waypoint_num).rev() {
            let current = translation_at(traj, i);
            let next = translation_at(traj, i - 1);
            if intersection_found(center_position, &current, &next, r) {
                return Some(i);
            }
        }
    } else {
        for i in 0..waypoint_num - 1 {
            let current = translation_at(traj, i);
            let next = translation_at(traj, i + 1);
            if intersection_found(center_position, &current, &next, r) {
                return Some(i);
            }
        }
    }

    None
}

/// Whether the segment `p_current -> p_next` crosses the sphere centred at
/// `p_center` with radius `r`: `p_current` inside or on it, `p_next` outside
/// or on it.
///
/// Upstream `intersectionFound`.
pub fn intersection_found(
    p_center: &Vector3,
    p_current: &Vector3,
    p_next: &Vector3,
    r: f64,
) -> bool {
    (p_current - p_center).norm() <= r && (p_next - p_center).norm() >= r
}

/// Set `group_name`'s active joints on `scene`'s current state to
/// `joint_values` (in `joint_names` order) and report whether the resulting
/// state self-collides.
///
/// Upstream `isStateColliding`.
///
/// # Deviations from upstream
///
/// 1. **Return polarity matches this function's name, not upstream's.**
///    Upstream's own `isStateColliding` returns `!collision_res.collision`
///    — despite its name, `true` means the state is *not* colliding. That
///    inversion is not a mistake in upstream: `isStateColliding` is used
///    directly, unnegated, as a `GroupStateValidityCallbackFn`
///    (`ik_constraint_function` in [`compute_pose_ik`]'s upstream
///    counterpart), whose contract is "`true` means accept". This port's
///    [`compute_pose_ik`] instead negates at the call site
///    (`!is_state_colliding(...)` feeding
///    [`moveit_kinematics::SolveOptions::solution_callback`], whose contract
///    is the same "`true` accepts") — so [`is_state_colliding`] itself
///    returns the un-inverted, name-matching boolean: `true` iff the state
///    actually collides.
/// 2. Upstream takes an explicit `RobotState*` separate from `scene` and
///    checks *that* state against the scene's world/ACM
///    (`scene->checkSelfCollision(req, res, *rstate)`). This port's
///    [`PlanningScene::check_self_collision`] only ever checks its own
///    current state (see that method's doc comment) — so this function
///    mutates `scene`'s current state directly instead of taking a separate
///    probe state. A caller that must not disturb its real scene (like
///    [`compute_pose_ik`]) passes a scratch scene obtained from
///    [`PlanningScene::diff`].
pub fn is_state_colliding<'m, E>(
    scene: &mut PlanningScene<'m>,
    env: &E,
    group_name: &str,
    joint_names: &[String],
    joint_values: &[f64],
) -> bool
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    let state = scene.current_state_mut();
    for (name, &value) in joint_names.iter().zip(joint_values) {
        let _ = state.set_variable_position(name, value);
    }

    let request = CollisionRequest {
        group_name: Some(group_name.to_string()),
        verbose: true,
        ..Default::default()
    };
    scene.check_self_collision(env, &request).collision
}

/// Adapt a goal pose (`position`/`orientation`) to account for
/// `target_point_offset`, expressed in the goal's own (rotated) frame.
///
/// Upstream `getConstraintPose(const Point&, const Quaternion&, const
/// Vector3&)`. See this module's `# Deviations`, item 5, for why the
/// `Constraints`-message overload is not ported.
pub fn constraint_pose(
    position: &Vector3,
    orientation: &UnitQuaternion,
    offset: &Vector3,
) -> Isometry3 {
    let mut pose = Isometry3::from_parts((*position).into(), *orientation);
    pose.translation.vector -= orientation * offset;
    pose
}

#[cfg(test)]
mod tests {
    use std::fs;

    use approx::assert_relative_eq;
    use moveit_collision::{LinkPaddingScale, ParryCollisionEnv};
    use moveit_kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolverParams};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_scene::PlanningScene;
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;
    use crate::limits::{JointLimit, JointLimitsContainer};

    fn fixture_mesh_search_paths() -> MeshSearchPaths {
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )])
    }

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// panda.srdf's `"ready"` named state for `panda_arm` — moveit's own
    /// non-self-colliding demo pose. See [`all_zero_state`] for the
    /// contrasting self-colliding pose.
    fn ready_positions() -> HashMap<String, f64> {
        [
            ("panda_joint1", 0.0),
            ("panda_joint2", -0.785),
            ("panda_joint3", 0.0),
            ("panda_joint4", -2.356),
            ("panda_joint5", 0.0),
            ("panda_joint6", 1.571),
            ("panda_joint7", 0.785),
        ]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect()
    }

    fn panda_arm_solver(model: &RobotModel) -> NewtonRaphsonSolver {
        NewtonRaphsonSolver::new(model, "panda_arm", &SolverParams::default())
            .expect("panda_arm must build a solver")
    }

    // -- verify_sample_joint_limits: one boundary per limit family --

    fn one_joint_limits(max_velocity: f64, max_acceleration: f64) -> JointLimitsContainer {
        let mut limits = JointLimitsContainer::default();
        limits.add_limit(
            "j1",
            JointLimit {
                has_velocity_limits: true,
                max_velocity,
                has_acceleration_limits: true,
                max_acceleration,
                has_deceleration_limits: true,
                max_deceleration: -max_acceleration,
                ..Default::default()
            },
        );
        limits
    }

    #[test]
    fn verify_sample_joint_limits_accepts_within_every_bound() {
        // velocity = 0.005/0.1 = 0.05 (<= max_velocity 1.0); acceleration =
        // 0.05/(0.1+0.1)*2 = 0.5 (<= max_acceleration 1.0).
        let limits = one_joint_limits(1.0, 1.0);
        let last = HashMap::from([("j1".to_string(), 0.0)]);
        let last_v = HashMap::from([("j1".to_string(), 0.0)]);
        let current = HashMap::from([("j1".to_string(), 0.005)]);
        assert!(verify_sample_joint_limits(
            &last, &last_v, &current, 0.1, 0.1, &limits
        ));
    }

    #[test]
    fn verify_sample_joint_limits_rejects_velocity_over_limit() {
        let limits = one_joint_limits(1.0, 1.0);
        let last = HashMap::from([("j1".to_string(), 0.0)]);
        let last_v = HashMap::from([("j1".to_string(), 0.0)]);
        let current = HashMap::from([("j1".to_string(), 5.0)]);
        assert!(!verify_sample_joint_limits(
            &last, &last_v, &current, 0.1, 0.1, &limits
        ));
    }

    #[test]
    fn verify_sample_joint_limits_rejects_acceleration_when_speeding_up() {
        // last_velocity=0 -> current_velocity large: |last| <= |current|, so
        // the *acceleration* (not deceleration) branch is exercised.
        let limits = one_joint_limits(100.0, 0.5);
        let last = HashMap::from([("j1".to_string(), 0.0)]);
        let last_v = HashMap::from([("j1".to_string(), 0.0)]);
        let current = HashMap::from([("j1".to_string(), 1.0)]);
        assert!(!verify_sample_joint_limits(
            &last, &last_v, &current, 0.1, 0.1, &limits
        ));
    }

    #[test]
    fn verify_sample_joint_limits_rejects_deceleration_when_slowing_down() {
        // last_velocity large, current_velocity smaller: |last| > |current|,
        // so the *deceleration* branch is exercised instead.
        let limits = one_joint_limits(100.0, 0.5);
        let last = HashMap::from([("j1".to_string(), 0.0)]);
        let last_v = HashMap::from([("j1".to_string(), 9.0)]);
        let current = HashMap::from([("j1".to_string(), 0.1)]);
        assert!(!verify_sample_joint_limits(
            &last, &last_v, &current, 0.1, 0.1, &limits
        ));
    }

    #[test]
    fn verify_sample_joint_limits_rejects_duration_at_or_below_epsilon() {
        let limits = one_joint_limits(1.0, 1.0);
        let last = HashMap::from([("j1".to_string(), 0.0)]);
        let last_v = HashMap::from([("j1".to_string(), 0.0)]);
        let current = HashMap::from([("j1".to_string(), 0.0)]);
        assert!(!verify_sample_joint_limits(
            &last, &last_v, &current, 0.1, 10e-6, &limits
        ));
    }

    // -- determine_and_check_sampling_time: n<2 vs consistent vs mismatched --

    fn robot_trajectory_with_durations<'m>(
        model: &'m RobotModel,
        durations: &[f64],
    ) -> RobotTrajectory<'m> {
        let mut traj = RobotTrajectory::new(model);
        for &dt in durations {
            traj.add_suffix_way_point(RobotState::new(model), dt)
                .unwrap();
        }
        traj
    }

    #[test]
    fn determine_and_check_sampling_time_needs_at_least_two_intervals_on_one_side() {
        let (model, _) = load_panda();
        // Two waypoints -> one interval on each side: n1 = n2 = 1, both < 2.
        let a = robot_trajectory_with_durations(&model, &[0.0, 0.1]);
        let b = robot_trajectory_with_durations(&model, &[0.0, 0.1]);
        assert_eq!(determine_and_check_sampling_time(&a, &b, 1e-6), None);
    }

    #[test]
    fn determine_and_check_sampling_time_accepts_consistent_intervals() {
        let (model, _) = load_panda();
        let a = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.1, 0.1]);
        let b = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.1]);
        assert_relative_eq!(
            determine_and_check_sampling_time(&a, &b, 1e-6).unwrap(),
            0.1
        );
    }

    #[test]
    fn determine_and_check_sampling_time_rejects_a_mismatched_interior_interval() {
        let (model, _) = load_panda();
        // Interval 1 (index 1, interior since n1=3) is 0.5, not 0.1.
        let a = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.5, 0.1]);
        let b = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.1]);
        assert_eq!(determine_and_check_sampling_time(&a, &b, 1e-6), None);
    }

    #[test]
    fn determine_and_check_sampling_time_ignores_each_trajectorys_last_interval() {
        let (model, _) = load_panda();
        // Both trajectories' final interval (a closing partial interval) is
        // allowed to violate the shared sampling time.
        let a = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.1, 0.037]);
        let b = robot_trajectory_with_durations(&model, &[0.0, 0.1, 0.1, 0.1, 0.052]);
        assert_relative_eq!(
            determine_and_check_sampling_time(&a, &b, 1e-6).unwrap(),
            0.1
        );
    }

    // -- is_robot_state_equal / is_robot_state_stationary: epsilon boundary --

    #[test]
    fn is_robot_state_equal_true_within_epsilon_false_beyond_it() {
        let (model, _) = load_panda();
        let mut a = RobotState::new(&model);
        a.set_to_default_values();
        let mut b = RobotState::new(&model);
        b.set_to_default_values();
        b.set_variable_position("panda_joint1", 0.05).unwrap();

        assert!(is_robot_state_equal(&a, &b, "panda_arm", 0.1));
        assert!(!is_robot_state_equal(&a, &b, "panda_arm", 0.01));
    }

    #[test]
    fn is_robot_state_stationary_true_at_zero_false_when_moving() {
        let (model, _) = load_panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        assert!(is_robot_state_stationary(&state, "panda_arm", 1e-9));

        state.set_variable_velocity("panda_joint1", 0.2).unwrap();
        assert!(!is_robot_state_stationary(&state, "panda_arm", 1e-9));
    }

    // -- intersection_found: on-sphere boundary is inclusive on both ends --

    #[test]
    fn intersection_found_boundary_and_interior_and_exterior_cases() {
        let center = Vector3::new(0.0, 0.0, 0.0);
        let inside = Vector3::new(0.5, 0.0, 0.0);
        let on_sphere = Vector3::new(1.0, 0.0, 0.0);
        let outside = Vector3::new(2.0, 0.0, 0.0);

        // A crossing segment: current inside, next outside.
        assert!(intersection_found(&center, &inside, &outside, 1.0));
        // Both endpoints exactly on the sphere still counts (`<=`/`>=`).
        assert!(intersection_found(&center, &on_sphere, &on_sphere, 1.0));
        // Entirely inside: next is not outside-or-on, so no crossing.
        assert!(!intersection_found(&center, &inside, &inside, 1.0));
        // Entirely outside: current is not inside-or-on, so no crossing.
        assert!(!intersection_found(&center, &outside, &outside, 1.0));
    }

    // -- constraint_pose: identity vs a rotated frame --

    #[test]
    fn constraint_pose_at_identity_orientation_subtracts_offset_directly() {
        let position = Vector3::new(1.0, 2.0, 3.0);
        let orientation = UnitQuaternion::identity();
        let offset = Vector3::new(0.1, 0.0, 0.0);
        let pose = constraint_pose(&position, &orientation, &offset);
        assert_relative_eq!(pose.translation.vector, Vector3::new(0.9, 2.0, 3.0));
    }

    #[test]
    fn constraint_pose_rotates_offset_into_the_goal_frame() {
        let position = Vector3::new(0.0, 0.0, 0.0);
        // 90 degrees about Z: the frame's local +X axis points along world +Y.
        let orientation = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::z_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let offset = Vector3::new(1.0, 0.0, 0.0);
        let pose = constraint_pose(&position, &orientation, &offset);
        assert_relative_eq!(
            pose.translation.vector,
            Vector3::new(0.0, -1.0, 0.0),
            epsilon = 1e-9
        );
    }

    // -- compute_link_fk: known link resolves, unknown link does not --

    #[test]
    fn compute_link_fk_resolves_a_known_link_and_rejects_an_unknown_one() {
        let (model, _) = load_panda();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let joint_state = ready_positions();

        let pose = compute_link_fk(&mut state, "panda_link8", &joint_state)
            .expect("panda_link8 is a real link in the panda model");
        assert!(pose.translation.vector.norm() > 0.0);

        assert_eq!(
            compute_link_fk(&mut state, "no_such_link", &joint_state),
            None
        );
    }

    // -- compute_pose_ik: round-trip converges; self-collision rejects the
    // known-colliding all-zero panda configuration --

    #[test]
    fn compute_pose_ik_round_trips_a_reachable_pose() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let mut solver = panda_arm_solver(&model);

        let target_positions = ready_positions();
        let mut fk_state = RobotState::new(&model);
        fk_state.set_to_default_values();
        let target_pose = compute_link_fk(&mut fk_state, "panda_link8", &target_positions).unwrap();

        let seed: HashMap<String, f64> = solver
            .joint_names()
            .iter()
            .map(|n| (n.clone(), 0.0))
            .collect();
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };
        let solution = compute_pose_ik(
            &ctx,
            &mut solver,
            "panda_link8",
            &target_pose,
            model.model_frame(),
            &seed,
        )
        .expect("a reachable pose sampled from FK must have an IK solution");

        let mut check_state = RobotState::new(&model);
        check_state.set_to_default_values();
        let solved_pose = compute_link_fk(&mut check_state, "panda_link8", &solution).unwrap();
        assert_relative_eq!(
            solved_pose.translation.vector,
            target_pose.translation.vector,
            epsilon = 1e-4
        );
    }

    #[test]
    fn compute_pose_ik_rejects_tip_frame_mismatch() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let mut solver = panda_arm_solver(&model);
        let target_pose = Isometry3::identity();
        let seed = HashMap::new();

        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };
        assert_eq!(
            compute_pose_ik(
                &ctx,
                &mut solver,
                "panda_link4",
                &target_pose,
                model.model_frame(),
                &seed,
            ),
            None
        );
    }

    #[test]
    fn is_state_colliding_true_at_zero_false_at_ready() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let joint_names: Vec<String> = model
            .joint_model_group("panda_arm")
            .unwrap()
            .active_joint_names()
            .to_vec();

        // All-zero is panda's known self-colliding configuration (see
        // load_panda's own fixture provenance note in
        // `moveit-planners-sbp::planning_scene_validity`'s test module).
        let zero_values = vec![0.0; joint_names.len()];
        assert!(is_state_colliding(
            &mut scene,
            &env,
            "panda_arm",
            &joint_names,
            &zero_values
        ));

        let ready = ready_positions();
        let ready_values: Vec<f64> = joint_names.iter().map(|n| ready[n]).collect();
        assert!(!is_state_colliding(
            &mut scene,
            &env,
            "panda_arm",
            &joint_names,
            &ready_values
        ));
    }
}
