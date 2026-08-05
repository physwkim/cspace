// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2025, Aiman Haidar
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_polyline.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_polyline.cpp

//! Multi-waypoint Cartesian trajectory generation
//! ([`TrajectoryGeneratorPolyline`]): a
//! [`crate::path_rounded_composite::PathRoundedComposite`] built by
//! [`polyline_from_waypoints`], time-parametrized by one
//! [`VelocityProfileTrap`], sampled and IK-solved by
//! [`generate_joint_trajectory`].
//!
//! Ported from upstream `TrajectoryGeneratorPOLYLINE`.
//!
//! `POLYLINE` differs from `LIN` only in the path: one profile still spans
//! the whole motion, so the arm accelerates once at the start and decelerates
//! once at the end regardless of how many corners the path rounds.
//!
//! # Deviations from upstream
//!
//! - **No per-request Cartesian speed override, `max_trans_dec` unused.**
//!   Same two deviations as
//!   [`crate::trajectory_generator_lin::TrajectoryGeneratorLin`] — see that
//!   module's own doc. `plan` below takes the identical fallback branch.
//! - **Waypoints arrive already in the planning frame.** Upstream's
//!   `extractMotionPlanInfo` composes each `path_constraints` position
//!   constraint into a pose and left-multiplies it by
//!   `scene->getFrameTransform(frame_id)`; the frame resolution belongs to
//!   the message layer, which this crate does not have
//!   ([`crate::trajectory_generator`]'s own `# What changed shape, and why`).
//!   [`crate::trajectory_generator::PolylinePathConstraint::waypoints`] is
//!   therefore already frame-resolved.
//! - **The goal pose is the last waypoint's, not a separate goal
//!   constraint.** Upstream reads `req.goal_constraints` for the final pose
//!   *and* `req.path_constraints` for the vias, so a request can name a goal
//!   the waypoint list does not end at — upstream then plans to the vias and
//!   silently never reaches the stated goal, because `polylineFromWaypoints`
//!   is given only the vias. This port keeps upstream's *path* exactly
//!   (`start_pose` then the waypoint list) and additionally requires the
//!   Cartesian goal to be reachable, the same `computePoseIK` check upstream
//!   runs on it — see [`TrajectoryGeneratorPolyline::extract_motion_plan_info`].
//! - **Upstream's KDL-error-code-to-message mapping is not reproduced.**
//!   `plan` catches `KDL::Error_MotionPlanning` and rewrites codes
//!   `3102`/`3103`/`3104`/`3105`/`3106`/`3001`/`3002` into six English
//!   sentences, all thrown as the same `ConsicutiveColinearWaypoints`
//!   exception carrying [`MoveItErrorCode::InvalidMotionPlan`]. This port's
//!   [`crate::path_rounded_composite`]/[`crate::path_polyline_generator`]
//!   already produce a message naming the same code, so `plan` narrows them
//!   to [`MoveItErrorCode::InvalidMotionPlan`] without rewriting the text.
//!   The one behavioural difference is that upstream's `catch` also swallows
//!   code `3001`/`3002` from a `Path_RoundedComposite` constructed with a
//!   non-positive `eqradius`; that is unreachable here, since `eqradius` is
//!   computed from the Cartesian limits, which
//!   [`crate::limits::CartesianLimits`] already requires to be positive.

use moveit_collision::CollisionEnv;
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::Isometry3;
use moveit_kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::path_polyline_generator::polyline_from_waypoints;
use crate::path_rounded_composite::PathRoundedComposite;
use crate::trajectory_functions::{
    CartesianPath, IkContext, compute_link_fk, compute_pose_ik, constraint_pose,
    generate_joint_trajectory,
};
use crate::trajectory_generator::{
    Goal, MotionPlanInfo, MotionPlanRequest, PilzGenerator, PolylinePathConstraint,
    TrajectoryGenerator,
};
use crate::velocity_profile_trap::VelocityProfileTrap;

/// Multi-waypoint Cartesian trajectory generator.
///
/// Upstream `TrajectoryGeneratorPOLYLINE`. See the [module docs](self) for
/// deviations.
pub struct TrajectoryGeneratorPolyline<'m> {
    base: TrajectoryGenerator<'m>,
}

impl<'m> TrajectoryGeneratorPolyline<'m> {
    /// Upstream
    /// `TrajectoryGeneratorPOLYLINE(robot_model, planner_limits, group_name)`.
    /// `group_name` is accepted (matching upstream's constructor signature)
    /// but unused, exactly as in
    /// [`crate::trajectory_generator_circ::TrajectoryGeneratorCirc::new`].
    pub fn new(base: TrajectoryGenerator<'m>, _group_name: &str) -> Self {
        Self { base }
    }
}

/// `req`'s `POLYLINE` waypoints.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidMotionPlan`] if `req.path_constraints` is absent
/// or carries another command's constraint.
fn polyline_path_constraint(req: &MotionPlanRequest) -> Result<&PolylinePathConstraint> {
    req.path_constraints
        .as_ref()
        .and_then(|pc| pc.as_polyline())
        .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))
}

impl<'m, E> PilzGenerator<'m, E> for TrajectoryGeneratorPolyline<'m>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    fn base(&self) -> &TrajectoryGenerator<'m> {
        &self.base
    }

    /// Upstream `TrajectoryGeneratorPOLYLINE::cmdSpecificRequestValidation`:
    /// fewer than two waypoints is `NoWaypointsSpecified`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidMotionPlan`] if `req.path_constraints` is
    /// absent, carries another command's constraint, or holds fewer than two
    /// waypoints.
    fn cmd_specific_request_validation(&self, req: &MotionPlanRequest) -> Result<()> {
        let constraint = polyline_path_constraint(req)?;
        if constraint.waypoints.len() < 2 {
            return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
        }
        Ok(())
    }

    /// Upstream `TrajectoryGeneratorPOLYLINE::extractMotionPlanInfo`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidMotionPlan`] if `req.goal` is not Cartesian:
    /// upstream reads `goal_constraints.front().position_constraints.front()`
    /// unconditionally, which is a `POLYLINE`-only precondition this port
    /// states as an error rather than an out-of-bounds read.
    /// [`MoveItErrorCode::NoIkSolution`] if the goal pose has no reachable IK
    /// solution (upstream `LinInverseForGoalIncalculable`).
    fn extract_motion_plan_info(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &mut MotionPlanInfo<'m>,
    ) -> Result<()> {
        info.group_name = req.group_name.clone();
        let robot_model = self.base.robot_model();
        let mut scratch_state = ctx.scene.current_state().clone();

        let Goal::Cartesian {
            link_name,
            position,
            orientation,
            target_point_offset,
        } = &req.goal
        else {
            return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
        };

        info.link_name = link_name.clone();
        info.goal_pose = constraint_pose(position, orientation, target_point_offset);

        let params = SolverParams::default();
        let mut solver = resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
            .ok()
            .filter(|solver| solver.tip_frame() == link_name.as_str())
            .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

        compute_pose_ik(
            ctx,
            solver.as_mut(),
            link_name,
            &info.goal_pose,
            robot_model.model_frame(),
            &info.start_joint_position,
        )
        .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

        // Upstream discards this call's `bool` too -- same deviation note as
        // `TrajectoryGeneratorLin`'s.
        if let Some(pose) = compute_link_fk(
            &mut scratch_state,
            &info.link_name,
            &info.start_joint_position,
        ) {
            info.start_pose = pose;
        }
        Ok(())
    }

    /// Upstream `TrajectoryGeneratorPOLYLINE::plan`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidMotionPlan`] for every path-construction
    /// failure (upstream's `ConsicutiveColinearWaypoints`, which carries the
    /// same code for all six KDL error codes it rewrites).
    /// [`MoveItErrorCode::NoIkSolution`] if no
    /// [`static@moveit_kinematics::KINEMATICS_SOLVERS`] entry can be built
    /// for `req.group_name` with `info.link_name` as its tip. Otherwise, see
    /// [`generate_joint_trajectory`].
    fn plan(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &MotionPlanInfo<'m>,
        sampling_time: f64,
    ) -> Result<RobotTrajectory<'m>> {
        let constraint = polyline_path_constraint(req)?;
        let cartesian_limits = self.base.planner_limits().cartesian_limits();
        // Upstream setMaxCartesianSpeed's fallback branch -- see this module's
        // "no per-request Cartesian speed override" deviation note.
        let max_cartesian_speed = cartesian_limits.max_trans_vel;

        let path = polyline_from_waypoints(
            &info.start_pose,
            &constraint.waypoints,
            constraint.smoothness_level,
            max_cartesian_speed / cartesian_limits.max_rot_vel,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

        let mut velocity_profile = VelocityProfileTrap::new(
            req.max_velocity_scaling_factor * max_cartesian_speed,
            req.max_acceleration_scaling_factor * cartesian_limits.max_trans_acc,
        );
        let path_length = path.path_length();
        if path_length > f64::EPSILON {
            velocity_profile.set_profile(0.0, path_length);
        } else {
            velocity_profile.set_profile(0.0, f64::EPSILON);
        }
        let segment = PolylineSegment {
            path,
            velocity_profile,
        };

        let robot_model = self.base.robot_model();
        let params = SolverParams::default();
        let mut solver = resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
            .ok()
            .filter(|solver| solver.tip_frame() == info.link_name.as_str())
            .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

        generate_joint_trajectory(
            ctx,
            solver.as_mut(),
            self.base.planner_limits().joint_limits(),
            &segment,
            &info.link_name,
            &info.start_joint_position,
            sampling_time,
        )
    }
}

/// A [`PathRoundedComposite`] time-parametrized by a [`VelocityProfileTrap`]
/// over its own arc length — the `POLYLINE` counterpart of
/// `trajectory_generator_lin`'s `LinSegment`, and upstream's
/// `KDL::Trajectory_Segment(path, vp, false)`.
struct PolylineSegment {
    path: PathRoundedComposite,
    velocity_profile: VelocityProfileTrap,
}

impl CartesianPath for PolylineSegment {
    fn duration(&self) -> f64 {
        self.velocity_profile.duration()
    }

    fn pos(&self, t: f64) -> Isometry3 {
        self.path.pos(self.velocity_profile.pos(t))
    }
}
