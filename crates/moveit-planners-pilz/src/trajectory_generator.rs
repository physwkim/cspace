// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator.cpp

//! Request validation shared by every Pilz trajectory generator
//! (`LIN`/`PTP`/`CIRC`, none of which are in this crate's scope yet).
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
//! scans [`moveit_kinematics::KINEMATICS_SOLVERS`], attempts to build each
//! registered solver for `(robot_model, group_name)`, and accepts the goal if
//! any constructed solver's [`moveit_kinematics::KinematicsSolver::tip_frame`]
//! equals the requested link exactly. There is no fixed-transform-chain
//! fallback — a link rigidly attached to, but not equal to, a constructible
//! solver's own tip is rejected here where upstream would accept it.
//!
//! # Deferred: everything downstream of validation
//!
//! `generate` (the `try { validateRequest; cmdSpecificRequestValidation;
//! extractMotionPlanInfo; plan } catch {...}` orchestration),
//! `cmdSpecificRequestValidation` (empty in the base class; each of
//! `LIN`/`PTP`/`CIRC` overrides it), `extractMotionPlanInfo`/`plan` (pure
//! virtual — no body to port), `MotionPlanInfo` (needs a diffed
//! [`moveit_scene::PlanningScene`] plus a resolved goal pose — the same
//! extraction only a concrete generator can do), `cartesianTrapVelocityProfile`
//! (needs `KDL::VelocityProfile_Trap`, a *KDL library* symmetric trapezoidal
//! profile distinct from this crate's own [`crate::velocity_profile::VelocityProfileAtrap`]
//! and not yet ported anywhere in this crate), `setMaxCartesianSpeed`,
//! `setSuccessResponse`/`setFailureResponse` (build a
//! `planning_interface::MotionPlanResponse`, a type this port does not have),
//! and `filterGroupValues` (msg-structure-only: parallel-array zipping with no
//! native counterpart to zip, since [`StartState`] is already keyed by name)
//! all belong to the concrete `LIN`/`PTP`/`CIRC` generators, a later round's
//! scope — none of them are "not ported", they are not reachable without that
//! round's own types.

use std::collections::HashMap;

use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::{UnitQuaternion, Vector3};
use moveit_kinematics::{KINEMATICS_SOLVERS, SolverParams};
use moveit_model::RobotModel;

use crate::limits::{JointLimitsContainer, LimitsContainer};

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
/// [`TrajectoryGenerator::validate_request`] actually reads. `planner_id`,
/// `num_planning_attempts`, `allowed_planning_time`, `path_constraints`, ...
/// have no reader in the upstream methods this round ports and are not
/// carried here.
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
}

/// Base state every Pilz trajectory generator validates a request against:
/// the robot model and the fused joint/Cartesian limits.
///
/// Upstream `TrajectoryGenerator`. See this module's `# Deferred` section for
/// what upstream carries that this port does not yet — `plan`/
/// `extractMotionPlanInfo`/`generate` are a later round's scope.
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
/// [`MoveItErrorCode::NoIkSolution`] if no [`KINEMATICS_SOLVERS`] entry can
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
        assert!(check_start_state(&model, &beyond, "panda_arm", &limits).is_err());
    }

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
        assert!(check_start_state(&model, &beyond_tolerance, "panda_arm", &limits).is_err());
    }

    #[test]
    fn start_state_rejects_an_unknown_group() {
        let model = load_panda();
        let limits = panda_joint_limits();
        let state = StartState::default();
        assert!(check_start_state(&model, &state, "no_such_group", &limits).is_err());
    }

    // -- check_joint_goal: joint-in-group vs joint-outside-group, within
    // limit vs beyond it --

    #[test]
    fn joint_goal_rejects_a_joint_outside_the_group() {
        let model = load_panda();
        let limits = panda_joint_limits();
        let goal = HashMap::from([("no_such_joint".to_string(), 0.0)]);
        assert!(check_joint_goal(&model, &goal, "panda_arm", &limits).is_err());
    }

    #[test]
    fn joint_goal_within_limit_accepted_beyond_limit_rejected() {
        let model = load_panda();
        let limits = panda_joint_limits();

        let within = HashMap::from([("panda_joint1".to_string(), 1.0)]);
        assert!(check_joint_goal(&model, &within, "panda_arm", &limits).is_ok());

        let beyond = HashMap::from([("panda_joint1".to_string(), 10.0)]);
        assert!(check_joint_goal(&model, &beyond, "panda_arm", &limits).is_err());
    }

    // -- check_cartesian_goal: empty link name, matching tip, non-tip link --

    #[test]
    fn cartesian_goal_rejects_an_empty_link_name() {
        let model = load_panda();
        assert!(check_cartesian_goal(&model, "panda_arm", "").is_err());
    }

    #[test]
    fn cartesian_goal_accepts_the_groups_solver_tip() {
        let model = load_panda();
        // panda_arm's SRDF chain is base_link="panda_link0"
        // tip_link="panda_link8" -- every constructible solver's tip_frame()
        // must equal it.
        assert!(check_cartesian_goal(&model, "panda_arm", "panda_link8").is_ok());
    }

    #[test]
    fn cartesian_goal_rejects_a_non_tip_link() {
        let model = load_panda();
        assert!(check_cartesian_goal(&model, "panda_arm", "panda_link4").is_err());
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
        };
        match generator.validate_request(&request) {
            Err(Error::Code(MoveItErrorCode::InvalidGroupName)) => {}
            other => panic!("expected Error::Code(InvalidGroupName), got {other:?}"),
        }
    }
}
