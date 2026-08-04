// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator.cpp

//! Request validation shared by every Pilz trajectory generator
//! (`PTP`/`LIN`/`CIRC`, all three now in this crate's scope — see
//! [`crate::trajectory_generator_ptp`]/[`crate::trajectory_generator_lin`]/
//! [`crate::trajectory_generator_circ`]).
//!
//! Upstream `TrajectoryGenerator`'s body is dominated by the
//! `validateRequest`/`checkXxx` family, validating a
//! `planning_interface::MotionPlanRequest` (itself wrapping
//! `moveit_msgs::msg::MotionPlanRequest`) before a derived class generates a
//! trajectory. This port has no such message type (`PORTING-PLAN.md` D1/D2),
//! so [`MotionPlanRequest`]/[`Goal`]/[`StartState`] below are native replacements
//! — see `# What changed shape, and why` for exactly which upstream checks
//! that shape change removes outright versus which survive as
//! [`TrajectoryGenerator::validate_request`]'s body.
//!
//! # What changed shape, and why
//!
//! Upstream's `moveit_msgs::msg::Constraints` carries `joint_constraints`,
//! `position_constraints` and `orientation_constraints` as three independent
//! `Vec`s, any combination of which could be populated, empty, or
//! contradictory — so `checkGoalConstraints` spends most of its logic
//! disambiguating *which* shape it was actually given
//! (`isJointGoalGiven`/`isCartesianGoalGiven`/`isOnlyOneGoalTypeGiven`,
//! `NotExactlyOneGoalConstraintGiven`, `OnlyOneGoalTypeAllowed`,
//! `PositionOrientationConstraintNameMismatch`, `NoPrimitivePoseGiven`). None
//! of that is geometry or a limit — it exists only because the message shape
//! allows illegal combinations the domain never actually has. [`Goal`] is a
//! two-variant enum instead: exactly one joint-space or Cartesian target,
//! with the Cartesian variant's position/orientation sharing one `link_name`
//! field rather than two lists that could name different links. Every check
//! in the list above is therefore not "not ported" — it is unrepresentable,
//! which is the stronger guarantee (see `CLAUDE.md`'s "Structural fix vs.
//! clever patch": an invariant enforced by construction beats the same
//! invariant re-checked at every call site).
//!
//! What is left after that shape change is genuinely geometry- or
//! limits-driven, and **is** ported below, because skipping it produces a
//! trajectory generator that silently accepts a request it should have
//! rejected before doing any real work:
//!
//! - velocity/acceleration scaling factor range,
//! - the group name resolves in the robot model,
//! - the start state's positions are within their joint limits and its
//!   velocities are (near) zero,
//! - a joint-space goal's joint names belong to the group and are within
//!   limits,
//! - a Cartesian goal names a non-empty link an IK solver can be built for.
//!
//! # Deviation from upstream: how "an IK solver exists for this link" is
//! decided
//!
//! Upstream's `checkCartesianGoalConstraint` asks the *group*'s one
//! SRDF-`kinematics.yaml`-configured solver
//! (`JointModelGroup::canSetStateFromIK`, via `getSolverInstance()`) whether
//! its tip matches the requested link, falling back to
//! `getRigidlyConnectedParentLinkModel` if not (a fixed-transform-chain
//! search). This port's `moveit-model::JointModelGroup` carries no
//! `kinematics.yaml`-derived solver mapping, and `LinkModel` carries no
//! `associated_fixed_transforms_`/rigidly-connected-parent search either
//! (both are documented absences in `moveit-model`/`moveit-state`, not gaps
//! this crate can quietly work around). [`check_cartesian_goal`] instead
//! scans [`static@moveit_kinematics::KINEMATICS_SOLVERS`], attempts to build each
//! registered solver for `(robot_model, group_name)`, and accepts the goal if
//! any constructed solver's [`moveit_kinematics::KinematicsSolver::tip_frame`]
//! equals the requested link exactly. There is no fixed-transform-chain
//! fallback — a link rigidly attached to, but not equal to, a constructible
//! solver's own tip is rejected here where upstream would accept it.
//!
//! # `generate`, `MotionPlanInfo`, `MotionPlanResponse`
//!
//! [`PilzGenerator`] is the base-class half of upstream's orchestration:
//! `generate`'s `try { validateRequest; cmdSpecificRequestValidation;
//! extractMotionPlanInfo; plan } catch {...}` becomes
//! [`PilzGenerator::generate`]'s default method, calling four smaller methods
//! (`self.base().validate_request`, [`PilzGenerator::cmd_specific_request_validation`],
//! [`PilzGenerator::extract_motion_plan_info`], [`PilzGenerator::plan`]) that
//! each concrete generator (`PTP`/`LIN`/`CIRC`, in their own modules) provides
//! instead of C++ virtual dispatch. [`MotionPlanInfo`] is the same
//! diffed-scene-plus-resolved-goal bundle upstream's nested class is,
//! constructed the same way (`scene->diff()`, apply `req.start_state`, read
//! back `start_joint_position` from the group's active joints) via
//! `MotionPlanInfo::new`, called once by [`PilzGenerator::generate`] itself
//! rather than by each concrete generator. [`MotionPlanResponse`] replaces
//! `planning_interface::MotionPlanResponse`, restricted to the two fields any
//! caller here reads (`error_code`, `trajectory`) — `planning_time` is not
//! carried: this port has no `rclcpp::Clock`, and wall-clock timing is not
//! part of what a bit-for-bit oracle comparison could ever check.
//! [`PilzGenerator::generate`] is generic over the same `E: CollisionEnv`
//! [`crate::trajectory_functions::IkContext`] already is, since a Cartesian
//! goal's IK (inside [`PilzGenerator::extract_motion_plan_info`]) needs one.
//!
//! # LIN/CIRC-only machinery not ported as separate functions
//!
//! Three pieces of upstream `TrajectoryGenerator` machinery that only
//! `LIN`/`CIRC` (not `PTP`) need have no like-named port in this crate, but
//! all three are already accounted for, not deferred:
//!
//! - `cartesianTrapVelocityProfile` (`KDL::VelocityProfile_Trap`, a *KDL
//!   library* symmetric trapezoidal profile distinct from this crate's own
//!   [`crate::velocity_profile::VelocityProfileAtrap`]) is ported as
//!   [`crate::velocity_profile_trap::VelocityProfileTrap`] — see that type's
//!   own module doc — and used by both
//!   [`crate::trajectory_generator_lin::TrajectoryGeneratorLin`] and
//!   [`crate::trajectory_generator_circ::TrajectoryGeneratorCirc`].
//! - `setMaxCartesianSpeed` reads an optional per-request Cartesian speed
//!   override (`req.max_cartesian_speed`, a Pilz-specific `moveit_msgs`
//!   extension field) with a fallback to `cartesian_limits.max_trans_vel`.
//!   [`MotionPlanRequest`] carries no such field (this module's `# What
//!   changed shape, and why` message-shape exclusion), so both `LIN` and
//!   `CIRC` always take upstream's fallback branch directly — see
//!   [`crate::trajectory_generator_lin`]'s own "no per-request Cartesian
//!   speed override" deviation note.
//! - `filterGroupValues` (msg-structure-only: parallel-array zipping with no
//!   native counterpart to zip) has no port because [`StartState::velocity`]
//!   is already keyed by name — a joint absent from it reads as `0.0`,
//!   matching `filterGroupValues`'s own "push only if present" behaviour; see
//!   that field's own doc.

use std::collections::HashMap;
use std::sync::Arc;

use moveit_collision::CollisionEnv;
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use moveit_kinematics::{KINEMATICS_SOLVERS, SolverParams};
use moveit_model::RobotModel;
use moveit_scene::PlanningScene;
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::limits::{JointLimitsContainer, LimitsContainer};
use crate::trajectory_functions::IkContext;

/// Lower bound (exclusive) on `max_velocity_scaling_factor`/
/// `max_acceleration_scaling_factor`. Upstream `MIN_SCALING_FACTOR`.
pub const MIN_SCALING_FACTOR: f64 = 0.0001;
/// Upper bound (inclusive) on `max_velocity_scaling_factor`/
/// `max_acceleration_scaling_factor`. Upstream `MAX_SCALING_FACTOR`.
pub const MAX_SCALING_FACTOR: f64 = 1.0;
/// A start-state joint velocity below this magnitude counts as zero.
/// Upstream `VELOCITY_TOLERANCE`.
pub const VELOCITY_TOLERANCE: f64 = 1e-8;

fn is_scaling_factor_valid(scaling_factor: f64) -> bool {
    scaling_factor > MIN_SCALING_FACTOR && scaling_factor <= MAX_SCALING_FACTOR
}

/// A motion goal: exactly one of a joint-space target or a Cartesian target.
///
/// Replaces upstream `moveit_msgs::msg::Constraints`' three independent
/// constraint lists — see this module's `# What changed shape, and why`.
#[derive(Debug, Clone, PartialEq)]
pub enum Goal {
    /// A joint-space target, by joint name. Upstream's `joint_constraints`
    /// list, keyed the same way `JointConstraint::joint_name` already is.
    Joint(HashMap<String, f64>),
    /// A Cartesian target for one link. Upstream's `position_constraints[0]`
    /// combined with `orientation_constraints[0]`, sharing one `link_name`
    /// rather than two that could disagree.
    Cartesian {
        /// The link this pose targets. Upstream's (matching)
        /// `PositionConstraint::link_name`/`OrientationConstraint::link_name`.
        link_name: String,
        /// Target position. Upstream
        /// `position_constraints[0].constraint_region.primitive_poses[0].position`.
        position: Vector3,
        /// Target orientation. Upstream `orientation_constraints[0].orientation`.
        orientation: UnitQuaternion,
        /// Offset from `position`, in `orientation`'s frame. Upstream
        /// `position_constraints[0].target_point_offset`; see
        /// [`crate::trajectory_functions::constraint_pose`] for how this is
        /// applied.
        target_point_offset: Vector3,
    },
}

/// Which of `CIRC`'s two auxiliary-point semantics a [`CircPathConstraint`]
/// carries. Upstream's `moveit_msgs::msg::Constraints::name` string
/// (`"interim"`/`"center"`), typed instead of stringly matched — see
/// [`Goal`]'s own doc for why this crate prefers a closed enum over
/// upstream's open-ended message field wherever the domain only has a fixed
/// set of shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircPathConstraintKind {
    /// The arc passes through the point. Upstream `"interim"`.
    Interim,
    /// The point is the arc's center. Upstream `"center"`.
    Center,
}

/// `CIRC`'s third point, disambiguating which circle a start/goal pair
/// describes. Upstream `req.path_constraints`: a `Constraints` whose `name`
/// is `"interim"`/`"center"` and whose one `PositionConstraint` carries
/// `link_name` and the point itself
/// (`constraint_region.primitive_poses[0].position`). `PTP`/`LIN` never read
/// `path_constraints`, so [`MotionPlanRequest::path_constraints`] is `None`
/// for their requests.
#[derive(Debug, Clone, PartialEq)]
pub struct CircPathConstraint {
    /// Upstream `req.path_constraints.name`.
    pub kind: CircPathConstraintKind,
    /// The link `point` is expressed for. Only read by
    /// [`crate::trajectory_generator_circ::TrajectoryGeneratorCirc`]'s
    /// joint-space goal branch, matching upstream's own
    /// `extractMotionPlanInfo`, which resolves `info.link_name` from here
    /// rather than from the (absent, for a joint goal) Cartesian goal
    /// constraint. Upstream
    /// `req.path_constraints.position_constraints[0].link_name`.
    pub link_name: String,
    /// The interim or center point. Upstream
    /// `req.path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position`.
    pub point: Vector3,
}

/// A request's start state: position (checked against joint limits) and
/// velocity (checked to be near zero — no derived class allows a moving
/// start).
///
/// Replaces upstream `moveit_msgs::msg::RobotState.joint_state`'s parallel
/// `name`/`position`/`velocity` arrays.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StartState {
    /// Per-joint starting position, by joint name.
    pub position: HashMap<String, f64>,
    /// Per-joint starting velocity, by joint name. A joint absent here is
    /// `0.0` — matching upstream's `filterGroupValues`, which only pushes a
    /// velocity entry `if (i < robot_state.velocity.size())`.
    pub velocity: HashMap<String, f64>,
}

/// A validated-before-planning motion request.
///
/// Replaces upstream `planning_interface::MotionPlanRequest`/
/// `moveit_msgs::msg::MotionPlanRequest`, restricted to the fields
/// [`TrajectoryGenerator::validate_request`] or a concrete generator
/// actually reads. `planner_id`, `num_planning_attempts`,
/// `allowed_planning_time`, ... have no reader anywhere in this crate and are
/// not carried here.
#[derive(Debug, Clone, PartialEq)]
pub struct MotionPlanRequest {
    /// The planning group. Upstream `group_name`.
    pub group_name: String,
    /// The state planning starts from. Upstream `start_state`.
    pub start_state: StartState,
    /// The motion goal. Upstream `goal_constraints` (a `Vec` of exactly one
    /// `Constraints`, itself exactly one goal type — see [`Goal`]).
    pub goal: Goal,
    /// Upstream `max_velocity_scaling_factor`.
    pub max_velocity_scaling_factor: f64,
    /// Upstream `max_acceleration_scaling_factor`.
    pub max_acceleration_scaling_factor: f64,
    /// `CIRC`'s auxiliary point. Upstream `req.path_constraints`; `None` for
    /// `PTP`/`LIN` requests, which never read it — see
    /// [`CircPathConstraint`].
    pub path_constraints: Option<CircPathConstraint>,
}

/// Base state every Pilz trajectory generator validates a request against:
/// the robot model and the fused joint/Cartesian limits.
///
/// Upstream `TrajectoryGenerator`. Upstream's `plan`/`extractMotionPlanInfo`/
/// `generate` are virtual methods each concrete generator overrides; this
/// port provides them as the [`PilzGenerator`] trait instead — see this
/// module's `# generate, MotionPlanInfo, MotionPlanResponse` section — with
/// [`TrajectoryGenerator`] itself only holding the state upstream's base
/// class constructor stores (`robot_model_`, `planner_limits_`).
pub struct TrajectoryGenerator<'m> {
    robot_model: &'m RobotModel,
    planner_limits: LimitsContainer,
}

impl<'m> TrajectoryGenerator<'m> {
    /// Upstream `TrajectoryGenerator(robot_model, planner_limits)`.
    pub fn new(robot_model: &'m RobotModel, planner_limits: LimitsContainer) -> Self {
        Self {
            robot_model,
            planner_limits,
        }
    }

    /// The robot model this generator validates against.
    pub fn robot_model(&self) -> &'m RobotModel {
        self.robot_model
    }

    /// The fused joint/Cartesian limits this generator validates against.
    pub fn planner_limits(&self) -> &LimitsContainer {
        &self.planner_limits
    }

    /// Validate `req` against this generator's robot model and limits.
    ///
    /// Upstream `validateRequest`, restricted to the geometry/limits checks
    /// that survive [`Goal`]'s shape — see this module's `# What changed
    /// shape, and why`. Upstream's own `cmdSpecificRequestValidation` (empty
    /// in the base class) is not called here; it belongs to the concrete
    /// generator that overrides it.
    ///
    /// # Errors
    ///
    /// See each `check_*` function's own `# Errors`; the first failing check
    /// short-circuits the rest, matching upstream's `try`/`catch` around one
    /// exception at a time.
    pub fn validate_request(&self, req: &MotionPlanRequest) -> Result<()> {
        check_velocity_scaling(req.max_velocity_scaling_factor)?;
        check_acceleration_scaling(req.max_acceleration_scaling_factor)?;
        check_for_valid_group_name(self.robot_model, &req.group_name)?;
        check_start_state(
            self.robot_model,
            &req.start_state,
            &req.group_name,
            self.planner_limits.joint_limits(),
        )?;
        check_goal(
            self.robot_model,
            &req.goal,
            &req.group_name,
            self.planner_limits.joint_limits(),
        )
    }
}

/// Information extracted from a [`MotionPlanRequest`], needed to plan.
///
/// Upstream `TrajectoryGenerator::MotionPlanInfo`. [`Self::start_scene`] is a
/// [`PlanningScene::diff`] of the scene `Self::new` was built from, with
/// `req.start_state` applied — every concrete generator's `plan` runs against
/// this scene, not the original.
///
/// # Deviations from upstream
///
/// - No `waypoints` field: upstream declares
///   `std::vector<Eigen::Isometry3d> waypoints`, but it has no reader or
///   writer anywhere in `trajectory_generator{,_ptp,_lin,_circ}.cpp` —
///   confirmed by `rg -n waypoints` across all four files, not just `PTP`/
///   `LIN`'s. Carrying a field nothing ever reads or writes is forbidden by
///   this workspace's `deny(warnings)`.
pub struct MotionPlanInfo<'m> {
    /// The planning group. Upstream `group_name`.
    pub group_name: String,
    /// The Cartesian goal's link, empty for a joint-space goal. Upstream
    /// `link_name`.
    pub link_name: String,
    /// The Cartesian goal's link's pose at the start state. Upstream
    /// `start_pose`; left [`Isometry3::identity`] by `Self::new` — only a
    /// Cartesian-goal generator's `extractMotionPlanInfo` fills it in.
    pub start_pose: Isometry3,
    /// The Cartesian goal pose, or [`Isometry3::identity`] for a joint-space
    /// goal. Upstream `goal_pose`.
    pub goal_pose: Isometry3,
    /// Per-joint starting position, over the group's active joints. Upstream
    /// `start_joint_position`.
    pub start_joint_position: HashMap<String, f64>,
    /// Per-joint goal position, over the group's active joints. Upstream
    /// `goal_joint_position`.
    pub goal_joint_position: HashMap<String, f64>,
    /// The scene planning runs against: `Self::new`'s input scene, diffed
    /// and with `req.start_state` applied. Upstream `start_scene`.
    pub start_scene: Arc<PlanningScene<'m>>,
    /// `CIRC`'s resolved auxiliary point (kind plus its final position, after
    /// a Cartesian goal's `target_point_offset` is applied — see
    /// [`crate::trajectory_generator_circ`]'s own doc for that adjustment).
    /// `None` for `PTP`/`LIN`, whose `extract_motion_plan_info` never writes
    /// it. Upstream `circ_path_point`.
    pub circ_aux_point: Option<CircPathConstraint>,
}

impl<'m> MotionPlanInfo<'m> {
    /// Upstream `TrajectoryGenerator::MotionPlanInfo::MotionPlanInfo`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidGroupName`] if `req.group_name` names no
    /// group in `scene`'s robot model — unreachable through
    /// [`PilzGenerator::generate`], which only calls this after
    /// [`TrajectoryGenerator::validate_request`] has already confirmed the
    /// group name.
    pub(crate) fn new(scene: &Arc<PlanningScene<'m>>, req: &MotionPlanRequest) -> Result<Self> {
        let mut diffed = scene.diff();
        diffed
            .current_state_mut()
            .set_variable_positions_by_name(&req.start_state.position)?;
        let start_scene = Arc::new(diffed);

        let group = start_scene
            .robot_model()
            .joint_model_group(&req.group_name)
            .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
        let mut start_joint_position = HashMap::new();
        for name in group.active_joint_names() {
            start_joint_position.insert(
                name.clone(),
                start_scene.current_state().variable_position(name)?,
            );
        }

        Ok(Self {
            group_name: req.group_name.clone(),
            link_name: String::new(),
            start_pose: Isometry3::identity(),
            goal_pose: Isometry3::identity(),
            start_joint_position,
            goal_joint_position: HashMap::new(),
            start_scene,
            circ_aux_point: None,
        })
    }
}

/// The outcome of [`PilzGenerator::generate`].
///
/// Upstream `planning_interface::MotionPlanResponse`, restricted to the two
/// fields any caller here reads — see this module's `# generate,
/// MotionPlanInfo, MotionPlanResponse` section for why `planning_time` is not
/// carried.
pub struct MotionPlanResponse<'m> {
    /// Upstream `error_code.val`.
    pub error_code: MoveItErrorCode,
    /// Upstream `trajectory`. [`None`] on any failure — upstream's
    /// `setFailureResponse` only conditionally clears an already-set
    /// trajectory, but nothing before `plan` succeeds ever sets one, so this
    /// is equivalent for every path [`PilzGenerator::generate`] takes.
    pub trajectory: Option<RobotTrajectory<'m>>,
}

impl<'m> MotionPlanResponse<'m> {
    fn success(trajectory: RobotTrajectory<'m>) -> Self {
        Self {
            error_code: MoveItErrorCode::Success,
            trajectory: Some(trajectory),
        }
    }

    /// Upstream `catch (const MoveItErrorCodeException& ex) { res.error_code.val
    /// = ex.getErrorCode(); ... }`. Every error surfaced by
    /// [`TrajectoryGenerator::validate_request`] or a concrete generator's
    /// own methods is [`Error::Code`] by construction (see each function's
    /// own `# Errors`); [`MoveItErrorCode::Failure`] is a fallback for the
    /// non-`Code` variants those methods never actually return, matching
    /// upstream's own `FAILURE` default for a `TrajectoryGeneratorInvalidLimitsException`-like
    /// construction failure.
    fn failure(error: Error) -> Self {
        let error_code = match error {
            Error::Code(code) => code,
            _ => MoveItErrorCode::Failure,
        };
        Self {
            error_code,
            trajectory: None,
        }
    }
}

/// A concrete Pilz trajectory generator (`PTP`/`LIN`/`CIRC`).
///
/// Upstream's pure-virtual `extractMotionPlanInfo`/`plan`, dispatched through
/// [`PilzGenerator::generate`]'s default method instead of C++ virtual
/// dispatch — see this module's `# generate, MotionPlanInfo,
/// MotionPlanResponse` section.
///
/// `E` is the collision backend [`crate::trajectory_functions::IkContext`]
/// checks a Cartesian goal's IK candidates against (only a Cartesian goal
/// ever performs IK; a joint-space goal never touches `E`).
pub trait PilzGenerator<'m, E>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    /// The base validation state (robot model, fused limits) this generator
    /// was built with.
    fn base(&self) -> &TrajectoryGenerator<'m>;

    /// Command-specific validation beyond [`TrajectoryGenerator::validate_request`].
    /// Upstream `cmdSpecificRequestValidation`, empty in the base class (no
    /// override needs one yet — `PTP`/`LIN` upstream don't override it
    /// either).
    ///
    /// # Errors
    ///
    /// Whatever the concrete generator's own validation reports.
    fn cmd_specific_request_validation(&self, _req: &MotionPlanRequest) -> Result<()> {
        Ok(())
    }

    /// Resolve `req`'s goal into `info`. Upstream `extractMotionPlanInfo`.
    ///
    /// `ctx.scene` is [`PilzGenerator::generate`]'s own `ctx.scene` argument
    /// (the *original*, undiffed scene) — matching upstream's `generate`,
    /// which calls `extractMotionPlanInfo(scene, req, plan_info)` with that
    /// same outer `scene`, not `plan_info.start_scene`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::NoIkSolution`] if `req.goal` is a Cartesian target
    /// with no reachable IK solution. Concrete-generator-specific errors
    /// otherwise.
    fn extract_motion_plan_info(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &mut MotionPlanInfo<'m>,
    ) -> Result<()>;

    /// Plan a trajectory from `info.start_joint_position` to `info.goal_joint_position`
    /// (or `info.goal_pose`, for a Cartesian-space generator). Upstream `plan`.
    ///
    /// `ctx.scene` is `info.start_scene` — matching upstream's `plan(plan_info.start_scene,
    /// ...)`.
    ///
    /// # Errors
    ///
    /// Concrete-generator-specific.
    fn plan(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &MotionPlanInfo<'m>,
        sampling_time: f64,
    ) -> Result<RobotTrajectory<'m>>;

    /// Generate a trajectory for `req` against `ctx.scene`, at `sampling_time`
    /// intervals.
    ///
    /// Upstream `generate`'s `try { validateRequest; cmdSpecificRequestValidation;
    /// extractMotionPlanInfo; plan } catch (const MoveItErrorCodeException& ex)
    /// { ...; setFailureResponse(...); return; }` — each stage's error
    /// short-circuits straight to `MotionPlanResponse::failure`, matching
    /// upstream's one-exception-at-a-time short-circuit.
    fn generate(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        sampling_time: f64,
    ) -> MotionPlanResponse<'m> {
        if let Err(error) = self.base().validate_request(req) {
            return MotionPlanResponse::failure(error);
        }
        if let Err(error) = self.cmd_specific_request_validation(req) {
            return MotionPlanResponse::failure(error);
        }

        let mut info = match MotionPlanInfo::new(ctx.scene, req) {
            Ok(info) => info,
            Err(error) => return MotionPlanResponse::failure(error),
        };
        if let Err(error) = self.extract_motion_plan_info(ctx, req, &mut info) {
            return MotionPlanResponse::failure(error);
        }

        let plan_ctx = IkContext {
            scene: &info.start_scene,
            env: ctx.env,
            check_self_collision: ctx.check_self_collision,
        };
        match self.plan(&plan_ctx, req, &info, sampling_time) {
            Ok(trajectory) => MotionPlanResponse::success(trajectory),
            Err(error) => MotionPlanResponse::failure(error),
        }
    }
}

/// Upstream `checkVelocityScaling`/`TrajectoryGenerator::isScalingFactorValid`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidMotionPlan`] if `scaling_factor` is outside
/// `(MIN_SCALING_FACTOR, MAX_SCALING_FACTOR]`.
pub fn check_velocity_scaling(scaling_factor: f64) -> Result<()> {
    if !is_scaling_factor_valid(scaling_factor) {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }
    Ok(())
}

/// Upstream `checkAccelerationScaling`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidMotionPlan`] if `scaling_factor` is outside
/// `(MIN_SCALING_FACTOR, MAX_SCALING_FACTOR]`.
pub fn check_acceleration_scaling(scaling_factor: f64) -> Result<()> {
    if !is_scaling_factor_valid(scaling_factor) {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }
    Ok(())
}

/// Upstream `checkForValidGroupName`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `group_name` names no group in
/// `robot_model`.
pub fn check_for_valid_group_name(robot_model: &RobotModel, group_name: &str) -> Result<()> {
    if !robot_model.has_joint_model_group(group_name) {
        return Err(Error::Code(MoveItErrorCode::InvalidGroupName));
    }
    Ok(())
}

/// Upstream `checkStartState`. The `joint_state.name.size() !=
/// joint_state.position.size()` check (`SizeMismatchInStartState`) has no
/// counterpart here: [`StartState::position`] is a map, so a name can never
/// disagree in count with its own value.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `group` names no group in
/// `robot_model`. [`MoveItErrorCode::InvalidRobotState`] if any of
/// `group`'s active joints named in [`StartState::position`] is outside its
/// position limit, or any [`StartState::velocity`] entry has magnitude at
/// least [`VELOCITY_TOLERANCE`].
pub fn check_start_state(
    robot_model: &RobotModel,
    start_state: &StartState,
    group: &str,
    joint_limits: &JointLimitsContainer,
) -> Result<()> {
    let group_ref = robot_model
        .joint_model_group(group)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
    for name in group_ref.active_joint_names() {
        if let Some(&position) = start_state.position.get(name)
            && !joint_limits.verify_position_limit(name, position)
        {
            return Err(Error::Code(MoveItErrorCode::InvalidRobotState));
        }
    }
    if !start_state
        .velocity
        .values()
        .all(|v| v.abs() < VELOCITY_TOLERANCE)
    {
        return Err(Error::Code(MoveItErrorCode::InvalidRobotState));
    }
    Ok(())
}

/// Upstream `checkGoalConstraints`, dispatching on [`Goal`]'s variant instead
/// of upstream's `isJointGoalGiven`/`isCartesianGoalGiven` runtime
/// disambiguation — see this module's `# What changed shape, and why`.
///
/// # Errors
///
/// See [`check_joint_goal`]/[`check_cartesian_goal`].
pub fn check_goal(
    robot_model: &RobotModel,
    goal: &Goal,
    group_name: &str,
    joint_limits: &JointLimitsContainer,
) -> Result<()> {
    match goal {
        Goal::Joint(positions) => {
            check_joint_goal(robot_model, positions, group_name, joint_limits)
        }
        Goal::Cartesian { link_name, .. } => {
            check_cartesian_goal(robot_model, group_name, link_name)
        }
    }
}

/// Upstream `checkJointGoalConstraint`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `group_name` names no group in
/// `robot_model`. [`MoveItErrorCode::InvalidGoalConstraints`] if any named
/// joint does not belong to `group_name`, or violates its position limit.
pub fn check_joint_goal(
    robot_model: &RobotModel,
    positions: &HashMap<String, f64>,
    group_name: &str,
    joint_limits: &JointLimitsContainer,
) -> Result<()> {
    let group = robot_model
        .joint_model_group(group_name)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
    for (name, &position) in positions {
        if !group.has_joint_model(name) {
            return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
        }
        if !joint_limits.verify_position_limit(name, position) {
            return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
        }
    }
    Ok(())
}

/// Upstream `checkCartesianGoalConstraint`. `pos_constraint.link_name !=
/// ori_constraint.link_name` (`PositionOrientationConstraintNameMismatch`)
/// and `primitive_poses.empty()` (`NoPrimitivePoseGiven`) have no counterpart
/// here — see this module's `# What changed shape, and why`. See the
/// module's `# Deviation from upstream` section for how "an IK solver exists"
/// is decided here instead of via `canSetStateFromIK`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGoalConstraints`] if `link_name` is empty.
/// [`MoveItErrorCode::NoIkSolution`] if no [`static@KINEMATICS_SOLVERS`] entry can
/// be built for `group_name` with `link_name` as its tip.
pub fn check_cartesian_goal(
    robot_model: &RobotModel,
    group_name: &str,
    link_name: &str,
) -> Result<()> {
    if link_name.is_empty() {
        return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
    }

    let params = SolverParams::default();
    let solver_available = KINEMATICS_SOLVERS.iter().any(|registration| {
        (registration.construct)(robot_model, group_name, &params)
            .map(|solver| solver.tip_frame() == link_name)
            .unwrap_or(false)
    });
    if !solver_available {
        return Err(Error::Code(MoveItErrorCode::NoIkSolution));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::fs;

    use moveit_kinematics::{DEFAULT_SOLVER_NAME, resolve_solver};
    use moveit_model::{MeshSearchPaths, RobotModel};

    use super::*;
    use crate::limits::JointLimit;

    fn load_panda() -> RobotModel {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = moveit_srdf::SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        let mesh_paths = MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )]);
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_paths)
            .expect("fixture model must build")
    }

    fn panda_joint_limits() -> JointLimitsContainer {
        let mut limits = JointLimitsContainer::default();
        for joint in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            limits.add_limit(
                joint,
                JointLimit {
                    has_position_limits: true,
                    min_position: -2.9,
                    max_position: 2.9,
                    ..Default::default()
                },
            );
        }
        limits
    }

    // -- check_velocity_scaling / check_acceleration_scaling: the
    // (MIN_SCALING_FACTOR, MAX_SCALING_FACTOR] boundary is exclusive below,
    // inclusive above --

    #[test]
    fn scaling_factor_boundary_is_exclusive_below_and_inclusive_above() {
        assert!(check_velocity_scaling(MIN_SCALING_FACTOR).is_err());
        assert!(check_velocity_scaling(MIN_SCALING_FACTOR + 1e-9).is_ok());
        assert!(check_velocity_scaling(MAX_SCALING_FACTOR).is_ok());
        assert!(check_velocity_scaling(MAX_SCALING_FACTOR + 1e-9).is_err());

        assert!(check_acceleration_scaling(MIN_SCALING_FACTOR).is_err());
        assert!(check_acceleration_scaling(MAX_SCALING_FACTOR).is_ok());
        assert!(check_acceleration_scaling(MAX_SCALING_FACTOR + 1e-9).is_err());
    }

    #[test]
    fn scaling_factor_rejects_zero_and_negative() {
        assert!(check_velocity_scaling(0.0).is_err());
        assert!(check_velocity_scaling(-0.5).is_err());
    }

    // -- check_for_valid_group_name --

    #[test]
    fn valid_group_name_accepted_unknown_group_name_rejected() {
        let model = load_panda();
        assert!(check_for_valid_group_name(&model, "panda_arm").is_ok());
        assert!(check_for_valid_group_name(&model, "no_such_group").is_err());
    }

    // -- check_start_state: position-limit boundary, velocity-tolerance
    // boundary --

    /// `check_start_state` has three `Error::` sites over two codes (`rg -c
    /// 'Error::' trajectory_generator.rs` scoped to the function body: 3):
    /// group lookup -> `InvalidGroupName`, position and velocity violation
    /// -> `InvalidRobotState` (both the same code -- no caller-visible fact
    /// distinguishes them, so the code is the discrimination this test
    /// needs). A bare `.is_err()` could not tell "bad group" from "bad
    /// state"; checked on the structured code instead.
    #[test]
    fn start_state_position_within_limit_accepted_beyond_limit_rejected() {
        let model = load_panda();
        let limits = panda_joint_limits();

        let within = StartState {
            position: HashMap::from([("panda_joint1".to_string(), 2.9)]),
            velocity: HashMap::new(),
        };
        assert!(check_start_state(&model, &within, "panda_arm", &limits).is_ok());

        let beyond = StartState {
            position: HashMap::from([("panda_joint1".to_string(), 2.9 + 1e-6)]),
            velocity: HashMap::new(),
        };
        let err = check_start_state(&model, &beyond, "panda_arm", &limits).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidRobotState)),
            "expected Error::Code(InvalidRobotState), got {err:?}"
        );
    }

    /// Same three-site function as
    /// `start_state_position_within_limit_accepted_beyond_limit_rejected`;
    /// see that test's doc comment.
    #[test]
    fn start_state_velocity_at_tolerance_accepted_beyond_it_rejected() {
        let model = load_panda();
        let limits = panda_joint_limits();

        let at_tolerance = StartState {
            position: HashMap::new(),
            velocity: HashMap::from([("panda_joint1".to_string(), VELOCITY_TOLERANCE / 2.0)]),
        };
        assert!(check_start_state(&model, &at_tolerance, "panda_arm", &limits).is_ok());

        let beyond_tolerance = StartState {
            position: HashMap::new(),
            velocity: HashMap::from([("panda_joint1".to_string(), VELOCITY_TOLERANCE * 2.0)]),
        };
        let err = check_start_state(&model, &beyond_tolerance, "panda_arm", &limits).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidRobotState)),
            "expected Error::Code(InvalidRobotState), got {err:?}"
        );
    }

    /// Same three-site function; see
    /// `start_state_position_within_limit_accepted_beyond_limit_rejected`'s
    /// doc comment.
    #[test]
    fn start_state_rejects_an_unknown_group() {
        let model = load_panda();
        let limits = panda_joint_limits();
        let state = StartState::default();
        let err = check_start_state(&model, &state, "no_such_group", &limits).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidGroupName)),
            "expected Error::Code(InvalidGroupName), got {err:?}"
        );
    }

    // -- check_joint_goal: joint-in-group vs joint-outside-group, within
    // limit vs beyond it --

    /// `check_joint_goal` has three `Error::` sites over two codes (`rg -c
    /// 'Error::' trajectory_generator.rs` scoped to the function body: 3):
    /// group lookup -> `InvalidGroupName`, joint-not-in-group and
    /// joint-beyond-limit -> `InvalidGoalConstraints` (same code -- no
    /// caller-visible fact distinguishes those two). Checked on the
    /// structured code, which does distinguish from the group-lookup
    /// sibling.
    #[test]
    fn joint_goal_rejects_a_joint_outside_the_group() {
        let model = load_panda();
        let limits = panda_joint_limits();
        let goal = HashMap::from([("no_such_joint".to_string(), 0.0)]);
        let err = check_joint_goal(&model, &goal, "panda_arm", &limits).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidGoalConstraints)),
            "expected Error::Code(InvalidGoalConstraints), got {err:?}"
        );
    }

    /// Same three-site function; see
    /// `joint_goal_rejects_a_joint_outside_the_group`'s doc comment.
    #[test]
    fn joint_goal_within_limit_accepted_beyond_limit_rejected() {
        let model = load_panda();
        let limits = panda_joint_limits();

        let within = HashMap::from([("panda_joint1".to_string(), 1.0)]);
        assert!(check_joint_goal(&model, &within, "panda_arm", &limits).is_ok());

        let beyond = HashMap::from([("panda_joint1".to_string(), 10.0)]);
        let err = check_joint_goal(&model, &beyond, "panda_arm", &limits).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidGoalConstraints)),
            "expected Error::Code(InvalidGoalConstraints), got {err:?}"
        );
    }

    // -- check_cartesian_goal: empty link name, matching tip, non-tip link --

    /// `check_cartesian_goal` has two `Error::` sites with two distinct
    /// codes (`rg -c 'Error::' trajectory_generator.rs` scoped to the
    /// function body: 2): empty link name -> `InvalidGoalConstraints`, no
    /// solver for the link -> `NoIkSolution`. A bare `.is_err()` could not
    /// tell them apart; checked on the structured code.
    #[test]
    fn cartesian_goal_rejects_an_empty_link_name() {
        let model = load_panda();
        let err = check_cartesian_goal(&model, "panda_arm", "").unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidGoalConstraints)),
            "expected Error::Code(InvalidGoalConstraints), got {err:?}"
        );
    }

    #[test]
    fn cartesian_goal_accepts_the_groups_solver_tip() {
        let model = load_panda();
        // panda_arm's SRDF chain is base_link="panda_link0"
        // tip_link="panda_link8" -- every constructible solver's tip_frame()
        // must equal it.
        assert!(check_cartesian_goal(&model, "panda_arm", "panda_link8").is_ok());
    }

    /// Same two-site function as
    /// `cartesian_goal_rejects_an_empty_link_name`; see that test's doc
    /// comment.
    #[test]
    fn cartesian_goal_rejects_a_non_tip_link() {
        let model = load_panda();
        let err = check_cartesian_goal(&model, "panda_arm", "panda_link4").unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::NoIkSolution)),
            "expected Error::Code(NoIkSolution), got {err:?}"
        );
    }

    // -- PORTING-PLAN.md §177: the solver every generator resolves to is a
    // name in the source, never `KINEMATICS_SOLVERS`' linker-decided
    // iteration order --

    #[test]
    fn default_solver_name_is_the_upstream_faithful_port_not_the_ports_own_addition() {
        // `newton_raphson` ports `kdl_kinematics_plugin`'s own
        // `ChainIkSolverVelMimicSVD` as-is (see `NewtonRaphsonSolver`'s own
        // doc comment) -- the solver every oracle fixture (`kinematics.yaml`:
        // `kdl_kinematics_plugin/KDLKinematicsPlugin`) actually used. `lma`
        // is this port's own addition upstream never ships; a `_cached`
        // wrapper deliberately returns a different (still valid) IK solution
        // than its wrapped solver on an empty cache -- see
        // `CachedIkSolver`'s doc comment. Pinning the constant's *value*
        // means a future rename of the `newton_raphson` registration without
        // updating this constant fails here (`UnknownName`, at selection),
        // not as a numeric parity drift three crates away.
        assert_eq!(DEFAULT_SOLVER_NAME, "newton_raphson");
    }

    #[test]
    fn resolve_solver_picks_by_name_not_by_construction_order() {
        let model = load_panda();
        let params = SolverParams::default();

        // The name every pilz call site actually resolves to must exist and
        // build for panda_arm.
        let solver = resolve_solver(&model, "panda_arm", DEFAULT_SOLVER_NAME, &params)
            .expect("DEFAULT_SOLVER_NAME must resolve for panda_arm");
        assert_eq!(solver.tip_frame(), "panda_link8");

        // A name nothing registers must fail closed (`UnknownName`), not
        // silently fall through to whichever registration happens to
        // construct first -- the exact defect this API replaces.
        //
        // `resolve_solver` has exactly one `Error::` constructor in its own
        // body (`rg -c 'Error::' registry.rs` scoped to the function: 1);
        // the sibling is not internal but delegated -- whatever
        // `SolverRegistration::construct` itself returns when a name *is*
        // registered but cannot build. `Error::UnknownName` carries
        // structured fields, so this checks them rather than just the
        // variant, per the doc comment's own stated intent.
        let err = resolve_solver(&model, "panda_arm", "not_a_registered_solver", &params)
            .err()
            .unwrap();
        match err {
            Error::UnknownName { kind, name } => {
                assert_eq!(kind, "kinematics solver");
                assert_eq!(name, "not_a_registered_solver");
            }
            other => panic!("expected Error::UnknownName, got {other:?}"),
        }
    }

    // -- validate_request: a fully valid request passes end to end; an
    // invalid group name fails at the first check --

    #[test]
    fn validate_request_accepts_a_well_formed_joint_goal_request() {
        let model = load_panda();
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        let generator = TrajectoryGenerator::new(&model, limits);

        let request = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: HashMap::from([("panda_joint1".to_string(), 0.0)]),
                velocity: HashMap::new(),
            },
            goal: Goal::Joint(HashMap::from([("panda_joint1".to_string(), 1.0)])),
            max_velocity_scaling_factor: 0.5,
            max_acceleration_scaling_factor: 0.5,
            path_constraints: None,
        };
        assert!(generator.validate_request(&request).is_ok());
    }

    #[test]
    fn validate_request_rejects_an_unknown_group_before_checking_the_goal() {
        let model = load_panda();
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        let generator = TrajectoryGenerator::new(&model, limits);

        let request = MotionPlanRequest {
            group_name: "no_such_group".to_string(),
            start_state: StartState::default(),
            goal: Goal::Joint(HashMap::new()),
            max_velocity_scaling_factor: 0.5,
            max_acceleration_scaling_factor: 0.5,
            path_constraints: None,
        };
        match generator.validate_request(&request) {
            Err(Error::Code(MoveItErrorCode::InvalidGroupName)) => {}
            other => panic!("expected Error::Code(InvalidGroupName), got {other:?}"),
        }
    }
}
