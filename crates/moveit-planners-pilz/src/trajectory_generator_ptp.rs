// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_ptp.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_ptp.cpp

//! Point-to-point trajectory generation ([`TrajectoryGeneratorPtp`]): one
//! [`crate::velocity_profile::VelocityProfileAtrap`] per active joint,
//! synchronized to the slowest joint (the "leading axis").
//!
//! Ported from upstream `TrajectoryGeneratorPTP`.
//!
//! # Deviations from upstream
//!
//! - **No `joint_limits_` field.** Upstream stores the full per-joint
//!   `JointLimitsContainer` as a member (`joint_limits_`) but
//!   [`TrajectoryGeneratorPtp`] (like upstream's own `TrajectoryGeneratorPTP`)
//!   never reads it outside the constructor — only the fused
//!   `most_strict_limit_` drives `planPTP`. A field this port would never
//!   read is dead code under this workspace's `deny(warnings)`, so only the
//!   fused [`crate::limits::JointLimit`] is kept.
//! - **The goal-already-reached single waypoint's `time_from_start` is `0`,
//!   not `sampling_time`.** Upstream pushes one `JointTrajectoryPoint` with
//!   `time_from_start = sampling_time` directly (a ROS trajectory message
//!   point carries an absolute `time_from_start`, independent of any
//!   "previous waypoint" arithmetic). This port's
//!   [`moveit_trajectory::RobotTrajectory::add_suffix_way_point`] structurally
//!   forbids a nonzero duration for waypoint 0 (`Err` if the trajectory is
//!   empty and `dt != 0.0` — see that method's own doc comment), so the
//!   single point this degenerate case produces can only be at `dt = 0`.
//!   This is the one case where a bit-for-bit oracle comparison of this
//!   specific edge case cannot pass; every non-degenerate case (goal not
//!   already reached) is unaffected.

use std::collections::HashMap;

use moveit_collision::CollisionEnv;
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use moveit_scene::PlanningScene;
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::limits::JointLimit;
use crate::trajectory_functions::{IkContext, compute_pose_ik, constraint_pose, push_way_point};
use crate::trajectory_generator::{
    Goal, MotionPlanInfo, MotionPlanRequest, PilzGenerator, TrajectoryGenerator,
};
use crate::velocity_profile::VelocityProfileAtrap;

/// Lower bound on per-joint movement below which the goal counts as already
/// reached. Upstream `TrajectoryGeneratorPTP::MIN_MOVEMENT`.
pub const MIN_MOVEMENT: f64 = 0.001;

/// Point-to-point trajectory generator: one [`VelocityProfileAtrap`] per
/// active joint of the constructor's `group_name`, synchronized to the
/// slowest ("leading") axis.
///
/// Upstream `TrajectoryGeneratorPTP`. See the [module docs](self) for
/// deviations.
pub struct TrajectoryGeneratorPtp<'m> {
    base: TrajectoryGenerator<'m>,
    most_strict_limit: JointLimit,
}

impl<'m> TrajectoryGeneratorPtp<'m> {
    /// Upstream `TrajectoryGeneratorPTP(robot_model, planner_limits,
    /// group_name)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `base`'s planner limits have no joint limits
    /// set, `group_name` names no group in `base`'s robot model, or (when
    /// `group_name` has at least one active joint) the fused limit over those
    /// joints is missing a velocity, acceleration or deceleration limit.
    pub fn new(base: TrajectoryGenerator<'m>, group_name: &str) -> Result<Self> {
        if !base.planner_limits().has_joint_limits() {
            return Err(Error::construct("joint limit not set"));
        }
        let joint_limits = base.planner_limits().joint_limits();

        let group = base
            .robot_model()
            .joint_model_group(group_name)
            .map_err(|_| Error::construct(format!("invalid group: {group_name}")))?;
        let active_joints = group.active_joint_names();

        let most_strict_limit = if active_joints.is_empty() {
            JointLimit::default()
        } else {
            let limit = joint_limits
                .common_limit_for(active_joints)
                .map_err(|_| Error::construct("failed to compute common limit"))?;
            if !limit.has_velocity_limits {
                return Err(Error::construct(format!(
                    "velocity limit not set for group {group_name}"
                )));
            }
            if !limit.has_acceleration_limits {
                return Err(Error::construct(format!(
                    "acceleration limit not set for group {group_name}"
                )));
            }
            if !limit.has_deceleration_limits {
                return Err(Error::construct(format!(
                    "deceleration limit not set for group {group_name}"
                )));
            }
            limit
        };

        Ok(Self {
            base,
            most_strict_limit,
        })
    }
}

impl<'m, E> PilzGenerator<'m, E> for TrajectoryGeneratorPtp<'m>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    fn base(&self) -> &TrajectoryGenerator<'m> {
        &self.base
    }

    /// Upstream `TrajectoryGeneratorPTP::extractMotionPlanInfo`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::NoIkSolution`] if `req.goal` is a Cartesian target
    /// and either no [`static@moveit_kinematics::KINEMATICS_SOLVERS`] entry can be
    /// built for `req.group_name` with `link_name` as its tip, or IK does not
    /// converge for the resolved goal pose.
    fn extract_motion_plan_info(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &mut MotionPlanInfo<'m>,
    ) -> Result<()> {
        info.group_name = req.group_name.clone();

        match &req.goal {
            Goal::Joint(positions) => {
                info.goal_joint_position = positions.clone();
            }
            Goal::Cartesian {
                link_name,
                position,
                orientation,
                target_point_offset,
            } => {
                info.link_name = link_name.clone();
                let robot_model = self.base.robot_model();
                info.goal_pose = constraint_pose(position, orientation, target_point_offset);

                let params = SolverParams::default();
                let mut solver =
                    resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
                        .ok()
                        .filter(|solver| solver.tip_frame() == link_name.as_str())
                        .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

                let solution = compute_pose_ik(
                    ctx,
                    solver.as_mut(),
                    link_name,
                    &info.goal_pose,
                    robot_model.model_frame(),
                    &info.start_joint_position,
                )
                .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;
                info.goal_joint_position = solution;
            }
        }
        Ok(())
    }

    /// Upstream `TrajectoryGeneratorPTP::plan`, delegating to `planPTP`.
    ///
    /// # Errors
    ///
    /// See [`plan_ptp`].
    fn plan(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &MotionPlanInfo<'m>,
        sampling_time: f64,
    ) -> Result<RobotTrajectory<'m>> {
        plan_ptp(
            &self.most_strict_limit,
            &info.start_joint_position,
            &info.goal_joint_position,
            req,
            sampling_time,
            ctx.scene,
        )
    }
}

/// Plan a point-to-point trajectory from `start_pos` to `goal_pos`,
/// synchronizing every joint named in `goal_pos` to the slowest ("leading")
/// axis's [`VelocityProfileAtrap`].
///
/// Upstream `TrajectoryGeneratorPTP::planPTP`. Iterates `goal_pos`'s joint
/// names in sorted order, matching upstream's `std::map<std::string, double>`
/// iteration order — the leading-axis tie-break (`Duration() > max_duration`,
/// strict) depends on it.
///
/// `req` supplies the velocity/acceleration scaling factors and the group
/// name (its goal is ignored — `start_pos`/`goal_pos` are already resolved).
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `req.group_name` names no group
/// in `scene`'s robot model. [`Error::Construct`] if a non-leading joint's
/// profile cannot be synchronized to the leading axis's phase durations
/// (upstream: `PtpVelocityProfileSyncFailed`, marked `LCOV_EXCL_START` —
/// unreachable given `most_strict_limit`'s own limits, kept here as a
/// defensive error rather than a panic since this port has no exception
/// hierarchy to distinguish "should never happen" from an ordinary failure).
pub fn plan_ptp<'m>(
    most_strict_limit: &JointLimit,
    start_pos: &HashMap<String, f64>,
    goal_pos: &HashMap<String, f64>,
    req: &MotionPlanRequest,
    sampling_time: f64,
    scene: &PlanningScene<'m>,
) -> Result<RobotTrajectory<'m>> {
    let velocity_scaling_factor = req.max_velocity_scaling_factor;
    let acceleration_scaling_factor = req.max_acceleration_scaling_factor;

    let mut joint_names: Vec<String> = goal_pos.keys().cloned().collect();
    joint_names.sort();

    let robot_model = scene.robot_model();
    let mut out = RobotTrajectory::for_group_name(robot_model, &req.group_name)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;

    let goal_reached = joint_names
        .iter()
        .all(|name| (start_pos[name] - goal_pos[name]).abs() < MIN_MOVEMENT);
    if goal_reached {
        let mut positions = HashMap::new();
        let mut velocities = HashMap::new();
        let mut accelerations = HashMap::new();
        for name in &joint_names {
            positions.insert(name.clone(), start_pos[name]);
            velocities.insert(name.clone(), 0.0);
            accelerations.insert(name.clone(), 0.0);
        }
        // See the module docs' "# Deviations from upstream" note: this dt is
        // `0`, not upstream's `sampling_time`.
        push_way_point(
            &mut out,
            scene.current_state(),
            &positions,
            &velocities,
            &accelerations,
            0.0,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::PlanningFailed))?;
        return Ok(out);
    }

    let mut profiles: HashMap<String, VelocityProfileAtrap> = HashMap::new();
    let mut leading_axis = joint_names[0].clone();
    let mut max_duration = -1.0;
    for name in &joint_names {
        let mut profile = VelocityProfileAtrap::new(
            velocity_scaling_factor * most_strict_limit.max_velocity,
            acceleration_scaling_factor * most_strict_limit.max_acceleration,
            acceleration_scaling_factor * most_strict_limit.max_deceleration,
        );
        profile.set_profile(start_pos[name], goal_pos[name]);
        if profile.duration() > max_duration {
            max_duration = profile.duration();
            leading_axis = name.clone();
        }
        profiles.insert(name.clone(), profile);
    }

    let acc_time = profiles[&leading_axis].first_phase_duration();
    let const_time = profiles[&leading_axis].second_phase_duration();
    let dec_time = profiles[&leading_axis].third_phase_duration();
    for name in &joint_names {
        if *name == leading_axis {
            continue;
        }
        let profile = profiles.get_mut(name).expect("just inserted above");
        if !profile.set_profile_all_durations(
            start_pos[name],
            goal_pos[name],
            acc_time,
            const_time,
            dec_time,
        ) {
            return Err(Error::construct(format!(
                "TrajectoryGeneratorPTP::planPTP(): Can not synchronize velocity \
                 profile of axis {name} with leading axis {leading_axis}"
            )));
        }
    }

    let mut time_samples = Vec::new();
    let mut t = 0.0;
    while t < max_duration {
        time_samples.push(t);
        t += sampling_time;
    }
    time_samples.push(max_duration);

    for (i, &t) in time_samples.iter().enumerate() {
        let is_last = i == time_samples.len() - 1;
        let mut positions = HashMap::new();
        let mut velocities = HashMap::new();
        let mut accelerations = HashMap::new();
        for name in &joint_names {
            let profile = &profiles[name];
            positions.insert(name.clone(), profile.pos(t));
            velocities.insert(name.clone(), if is_last { 0.0 } else { profile.vel(t) });
            accelerations.insert(name.clone(), if is_last { 0.0 } else { profile.acc(t) });
        }
        let dt = if i == 0 { 0.0 } else { t - time_samples[i - 1] };
        push_way_point(
            &mut out,
            scene.current_state(),
            &positions,
            &velocities,
            &accelerations,
            dt,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::PlanningFailed))?;
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_collision::{LinkPaddingScale, ParryCollisionEnv};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_scene::PlanningScene;
    use moveit_srdf::SrdfModel;

    use super::*;
    use crate::limits::{JointLimitsContainer, LimitsContainer};
    use crate::trajectory_generator::StartState;

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

    const PANDA_ARM_JOINTS: [&str; 7] = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ];

    fn panda_joint_limits() -> JointLimitsContainer {
        let mut limits = JointLimitsContainer::default();
        for joint in PANDA_ARM_JOINTS {
            limits.add_limit(
                joint,
                JointLimit {
                    has_position_limits: true,
                    min_position: -2.9,
                    max_position: 2.9,
                    has_velocity_limits: true,
                    max_velocity: 2.0,
                    has_acceleration_limits: true,
                    max_acceleration: 3.0,
                    has_deceleration_limits: true,
                    max_deceleration: -3.0,
                    ..Default::default()
                },
            );
        }
        limits
    }

    fn panda_generator(model: &RobotModel) -> TrajectoryGeneratorPtp<'_> {
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        let base = TrajectoryGenerator::new(model, limits);
        TrajectoryGeneratorPtp::new(base, "panda_arm").expect("panda_arm has full limits")
    }

    fn zero_positions() -> HashMap<String, f64> {
        PANDA_ARM_JOINTS
            .iter()
            .map(|&n| (n.to_string(), 0.0))
            .collect()
    }

    fn ptp_request(group_name: &str, scaling: f64) -> MotionPlanRequest {
        MotionPlanRequest {
            group_name: group_name.to_string(),
            start_state: StartState::default(),
            goal: Goal::Joint(HashMap::new()),
            max_velocity_scaling_factor: scaling,
            max_acceleration_scaling_factor: scaling,
            path_constraints: None,
        }
    }

    // -- constructor: missing limit dimensions are each rejected --

    /// Boundary: no joint limits set at all, checked before `group_name` is
    /// even looked up.
    ///
    /// `TrajectoryGeneratorPtp::new` has six `Error::construct` sites (`rg
    /// -c 'Error::' trajectory_generator_ptp.rs` restricted to the function
    /// body: 6), so a bare `.is_err()` cannot say which fired. Checked on
    /// the message against its sibling guards below (message-swap bite).
    ///
    /// Live bite: neutralizing this guard (`if false && !has_joint_limits`)
    /// on this fixture (empty `LimitsContainer`) falls through to
    /// `common_limit_for` on an empty joint-limits container, which fails
    /// its own way -- the assertion here correctly FAILS with "construction
    /// failed: failed to compute common limit" while
    /// `constructor_rejects_unknown_group` and
    /// `constructor_rejects_a_group_missing_an_acceleration_limit` (whose
    /// fixtures never reach this guard) stay GREEN. Mutation reverted.
    #[test]
    fn constructor_rejects_missing_joint_limits() {
        let (model, _) = load_panda();
        let base = TrajectoryGenerator::new(&model, LimitsContainer::new());
        let err = TrajectoryGeneratorPtp::new(base, "panda_arm")
            .err()
            .unwrap();
        assert!(
            err.to_string().contains("joint limit not set"),
            "expected the no-joint-limits message, got {err}"
        );
    }

    /// Boundary: joint limits are set, but `group_name` names no group.
    /// Same six-site function as above; see that test's doc comment.
    #[test]
    fn constructor_rejects_unknown_group() {
        let (model, _) = load_panda();
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        let base = TrajectoryGenerator::new(&model, limits);
        let err = TrajectoryGeneratorPtp::new(base, "no_such_group")
            .err()
            .unwrap();
        assert!(
            err.to_string().contains("invalid group"),
            "expected the invalid-group message, got {err}"
        );
    }

    /// Boundary: the group's fused limit has a velocity limit but no
    /// acceleration limit. Same six-site function; see
    /// `constructor_rejects_missing_joint_limits`'s doc comment.
    #[test]
    fn constructor_rejects_a_group_missing_an_acceleration_limit() {
        let (model, _) = load_panda();
        let mut limits_container = JointLimitsContainer::default();
        for joint in PANDA_ARM_JOINTS {
            limits_container.add_limit(
                joint,
                JointLimit {
                    has_velocity_limits: true,
                    max_velocity: 2.0,
                    ..Default::default()
                },
            );
        }
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(limits_container);
        let base = TrajectoryGenerator::new(&model, limits);
        let err = TrajectoryGeneratorPtp::new(base, "panda_arm")
            .err()
            .unwrap();
        assert!(
            err.to_string().contains("acceleration limit not set"),
            "expected the missing-acceleration-limit message, got {err}"
        );
    }

    #[test]
    fn constructor_accepts_a_fully_limited_group() {
        let (model, _) = load_panda();
        let _ = panda_generator(&model);
    }

    // -- plan_ptp: goal-already-reached boundary --

    #[test]
    fn plan_ptp_goal_already_reached_returns_one_zero_dt_waypoint() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let limit = JointLimit {
            has_velocity_limits: true,
            max_velocity: 2.0,
            has_acceleration_limits: true,
            max_acceleration: 3.0,
            has_deceleration_limits: true,
            max_deceleration: -3.0,
            ..Default::default()
        };
        let start = zero_positions();
        // Every joint moves by less than MIN_MOVEMENT.
        let goal: HashMap<String, f64> = start
            .iter()
            .map(|(k, &v)| (k.clone(), v + MIN_MOVEMENT / 10.0))
            .collect();

        let req = ptp_request("panda_arm", 0.5);
        let traj = plan_ptp(&limit, &start, &goal, &req, 0.1, &scene)
            .expect("goal already reached must not error");
        assert_eq!(traj.way_point_count(), 1);
        assert_relative_eq!(traj.way_point_duration_from_start(0), 0.0);
        for name in PANDA_ARM_JOINTS {
            assert_relative_eq!(
                traj.way_point(0).unwrap().variable_position(name).unwrap(),
                start[name]
            );
            assert_relative_eq!(
                traj.way_point(0).unwrap().variable_velocity(name).unwrap(),
                0.0
            );
        }
    }

    // -- plan_ptp: multi-joint synchronization, leading-axis selection,
    // last-point zeroing --

    #[test]
    fn plan_ptp_synchronizes_every_joint_to_the_leading_axis_duration() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let limit = JointLimit {
            has_velocity_limits: true,
            max_velocity: 2.0,
            has_acceleration_limits: true,
            max_acceleration: 3.0,
            has_deceleration_limits: true,
            max_deceleration: -3.0,
            ..Default::default()
        };
        let start = zero_positions();
        let mut goal = zero_positions();
        // panda_joint2 travels farthest -> becomes the leading axis.
        *goal.get_mut("panda_joint2").unwrap() = 2.0;
        *goal.get_mut("panda_joint4").unwrap() = 0.3;

        let req = ptp_request("panda_arm", 1.0);
        let traj = plan_ptp(&limit, &start, &goal, &req, 0.1, &scene).unwrap();
        assert!(traj.way_point_count() > 1);

        // Every joint's own trapezoid finishes exactly at the trajectory's
        // total duration: the last waypoint's position equals its goal.
        let last = traj.way_point(traj.way_point_count() - 1).unwrap();
        for name in PANDA_ARM_JOINTS {
            assert_relative_eq!(
                last.variable_position(name).unwrap(),
                goal[name],
                epsilon = 1e-9
            );
            // Last-point velocity/acceleration are zeroed for every joint,
            // not only the leading axis.
            assert_relative_eq!(last.variable_velocity(name).unwrap(), 0.0);
            assert_relative_eq!(last.variable_acceleration(name).unwrap(), 0.0);
        }

        // A non-leading, non-moving joint (panda_joint1) still gets the same
        // total duration and zero velocity/acceleration throughout.
        let total_duration = traj.way_point_duration_from_start(traj.way_point_count() - 1);
        assert!(total_duration > 0.0);
    }

    /// Boundary: `req.group_name` names no group in `scene`'s robot model.
    ///
    /// `plan_ptp` has four fallible sites, each with its own
    /// [`MoveItErrorCode`] (`InvalidGroupName` at the `RobotTrajectory`
    /// construction, two `PlanningFailed`, one `Error::Construct` for the
    /// sync-failed case -- `rg -n 'Error::' trajectory_generator_ptp.rs`
    /// scoped to `plan_ptp`'s body: 4). A bare `.is_err()` cannot say which
    /// fired; checked on the structured [`Error::Code`] variant instead,
    /// which the other three sites in this function cannot produce.
    #[test]
    fn plan_ptp_rejects_an_unknown_group() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let limit = JointLimit {
            has_velocity_limits: true,
            max_velocity: 2.0,
            has_acceleration_limits: true,
            max_acceleration: 3.0,
            has_deceleration_limits: true,
            max_deceleration: -3.0,
            ..Default::default()
        };
        let start = HashMap::from([("panda_joint1".to_string(), 0.0)]);
        let goal = HashMap::from([("panda_joint1".to_string(), 1.0)]);
        let req = ptp_request("no_such_group", 0.5);
        let err = plan_ptp(&limit, &start, &goal, &req, 0.1, &scene).unwrap_err();
        assert!(
            matches!(err, Error::Code(MoveItErrorCode::InvalidGroupName)),
            "expected Error::Code(InvalidGroupName), got {err:?}"
        );
    }

    // -- PilzGenerator::generate: end-to-end joint-space goal, and a
    // Cartesian-goal IK path --

    fn env() -> ParryCollisionEnv {
        ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default())
    }

    #[test]
    fn generate_end_to_end_joint_goal_succeeds() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let generator = panda_generator(&model);
        let env = env();

        let mut goal = HashMap::new();
        for name in PANDA_ARM_JOINTS {
            goal.insert(name.to_string(), 0.5);
        }
        let request = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: zero_positions(),
                velocity: HashMap::new(),
            },
            goal: Goal::Joint(goal.clone()),
            max_velocity_scaling_factor: 0.5,
            max_acceleration_scaling_factor: 0.5,
            path_constraints: None,
        };

        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: true,
        };
        let response = generator.generate(&ctx, &request, 0.1);
        assert_eq!(response.error_code, MoveItErrorCode::Success);
        let trajectory = response.trajectory.expect("success carries a trajectory");
        let last = trajectory
            .way_point(trajectory.way_point_count() - 1)
            .unwrap();
        for name in PANDA_ARM_JOINTS {
            assert_relative_eq!(
                last.variable_position(name).unwrap(),
                goal[name],
                epsilon = 1e-9
            );
        }
    }

    #[test]
    fn generate_rejects_an_invalid_group_before_planning() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let generator = panda_generator(&model);
        let env = env();

        let request = MotionPlanRequest {
            group_name: "no_such_group".to_string(),
            start_state: StartState::default(),
            goal: Goal::Joint(HashMap::new()),
            max_velocity_scaling_factor: 0.5,
            max_acceleration_scaling_factor: 0.5,
            path_constraints: None,
        };
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: true,
        };
        let response = generator.generate(&ctx, &request, 0.1);
        assert_eq!(response.error_code, MoveItErrorCode::InvalidGroupName);
        // ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` --
        // `MotionPlanResponse::failure` is the only place in this crate that
        // writes `trajectory: None` (`rg -n 'trajectory: None'`
        // crate-wide: 1 hit), a single unconditional field literal reached
        // by every one of `generate`'s five failure short-circuits. Which
        // of those five fired is what `error_code` above already names;
        // this line only re-checks the type's own None-on-any-failure
        // invariant, which has one cause regardless.
        assert!(response.trajectory.is_none());
    }
}
