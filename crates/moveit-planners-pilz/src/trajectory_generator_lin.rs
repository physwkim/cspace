// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_lin.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_lin.cpp

//! Straight-line Cartesian trajectory generation ([`TrajectoryGeneratorLin`]):
//! a [`crate::path_line::PathLine`] time-parametrized by one
//! [`crate::velocity_profile_trap::VelocityProfileTrap`], sampled and
//! IK-solved by [`crate::trajectory_functions::generate_joint_trajectory`].
//!
//! Ported from upstream `TrajectoryGeneratorLIN`.
//!
//! # Deviations from upstream
//!
//! - **No per-request Cartesian speed override.** Upstream's `setMaxCartesianSpeed`
//!   reads an optional speed from `req.max_cartesian_speed` (a Pilz-specific
//!   `moveit_msgs` extension field) and falls back to
//!   `cartesian_limits.max_trans_vel` only if that field is unset. This port's
//!   [`crate::trajectory_generator::MotionPlanRequest`] carries no such field
//!   (`PORTING-PLAN.md` D1/D2's message-shape exclusion — see
//!   `trajectory_generator`'s own module doc for other fields dropped the same
//!   way), so [`TrajectoryGeneratorLin::plan`] always takes upstream's
//!   fallback branch.
//! - **`max_trans_dec` is read from [`crate::limits::CartesianLimits`] but
//!   never used.** Confirmed by reading upstream `cartesianTrapVelocityProfile`
//!   directly: `KDL::VelocityProfile_Trap` accepts only one acceleration
//!   magnitude for both ramps, so only `max_trans_acc` is ever passed to
//!   [`crate::velocity_profile_trap::VelocityProfileTrap::new`]. This is an
//!   upstream quirk (the field exists in the Cartesian limits schema for
//!   `CIRC`'s own use elsewhere, not because LIN secretly needs it), not a bug
//!   this port invents an asymmetric use to "fix".
//! - **A failed FK for the goal or start pose is silently ignored, matching
//!   upstream exactly.** Upstream's `computeLinkFK` call inside
//!   `extractMotionPlanInfo` has its `bool` return value discarded for both
//!   the goal-pose and start-pose computations; on failure `info.goal_pose`/
//!   `info.start_pose` keep whatever they already held (`Isometry3::identity`,
//!   from `crate::trajectory_generator::MotionPlanInfo::new`). This is
//!   reproduced verbatim rather than turned into a hard error.
//! - **A Cartesian goal's IK solution is discarded.** Unlike
//!   [`crate::trajectory_generator_ptp::TrajectoryGeneratorPtp`], where a
//!   Cartesian goal's IK solution directly becomes `goal_joint_position`,
//!   upstream's LIN only calls `computePoseIK` to confirm the goal pose is
//!   reachable at all -- the local `ik_solution` output parameter has no
//!   further reader. `info.goal_joint_position` is left empty for a Cartesian
//!   goal here, matching upstream: [`TrajectoryGeneratorLin::plan`] never
//!   reads it, only `info.goal_pose`.
//! - **No separate `getSolverTipFrame` port.** Upstream resolves a joint
//!   goal's `link_name` via `JointModelGroup::getSolverInstance()->getTipFrame()`,
//!   throwing `NoSolverException`/`MoreThanOneTipFrameException` (mapped to
//!   [`MoveItErrorCode::Failure`]) if that fails. This port's
//!   [`moveit_kinematics::KinematicsSolver::tip_frame`] is already singular
//!   (see that trait's own `# Deviations`), so "more than one tip frame" is
//!   unrepresentable here; `solver_tip_frame` only has upstream's
//!   "no solver" case left to handle, via the same scan-`KINEMATICS_SOLVERS`
//!   pattern [`crate::trajectory_generator::check_cartesian_goal`] already
//!   uses.

use moveit_collision::CollisionEnv;
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::Isometry3;
use moveit_kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use moveit_model::RobotModel;
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::path_line::PathLine;
use crate::trajectory_functions::{
    CartesianPath, IkContext, compute_link_fk, compute_pose_ik, constraint_pose,
    generate_joint_trajectory,
};
use crate::trajectory_generator::{
    Goal, MotionPlanInfo, MotionPlanRequest, PilzGenerator, TrajectoryGenerator,
};
use crate::velocity_profile_trap::VelocityProfileTrap;

/// Straight-line Cartesian trajectory generator.
///
/// Upstream `TrajectoryGeneratorLIN`. See the [module docs](self) for
/// deviations.
pub struct TrajectoryGeneratorLin<'m> {
    base: TrajectoryGenerator<'m>,
}

impl<'m> TrajectoryGeneratorLin<'m> {
    /// Upstream `TrajectoryGeneratorLIN(robot_model, planner_limits, group_name)`.
    /// `group_name` is accepted (matching upstream's constructor signature) but
    /// unused: upstream's own constructor body only logs it, doing nothing
    /// else with it either.
    pub fn new(base: TrajectoryGenerator<'m>, _group_name: &str) -> Self {
        Self { base }
    }
}

impl<'m, E> PilzGenerator<'m, E> for TrajectoryGeneratorLin<'m>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    fn base(&self) -> &TrajectoryGenerator<'m> {
        &self.base
    }

    /// Upstream `TrajectoryGeneratorLIN::extractMotionPlanInfo`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::Failure`] if `req.goal` is a joint-space target and
    /// no [`static@moveit_kinematics::KINEMATICS_SOLVERS`] entry can be built for
    /// `req.group_name` (upstream's `getSolverTipFrame` failure). [`MoveItErrorCode::NoIkSolution`]
    /// if `req.goal` is a Cartesian target with no reachable IK solution.
    fn extract_motion_plan_info(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &mut MotionPlanInfo<'m>,
    ) -> Result<()> {
        info.group_name = req.group_name.clone();
        let robot_model = self.base.robot_model();
        let mut scratch_state = ctx.scene.current_state().clone();

        match &req.goal {
            Goal::Joint(positions) => {
                info.link_name = solver_tip_frame(robot_model, &req.group_name)?;
                info.goal_joint_position = positions.clone();
                if let Some(pose) = compute_link_fk(
                    &mut scratch_state,
                    &info.link_name,
                    &info.goal_joint_position,
                ) {
                    info.goal_pose = pose;
                }
            }
            Goal::Cartesian {
                link_name,
                position,
                orientation,
                target_point_offset,
            } => {
                info.link_name = link_name.clone();
                info.goal_pose = constraint_pose(position, orientation, target_point_offset);

                let params = SolverParams::default();
                let mut solver =
                    resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
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
            }
        }

        if let Some(pose) = compute_link_fk(
            &mut scratch_state,
            &info.link_name,
            &info.start_joint_position,
        ) {
            info.start_pose = pose;
        }
        Ok(())
    }

    /// Upstream `TrajectoryGeneratorLIN::plan`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::NoIkSolution`] if no [`static@moveit_kinematics::KINEMATICS_SOLVERS`]
    /// entry can be built for `req.group_name` with `info.link_name` as its tip.
    /// Otherwise, see [`generate_joint_trajectory`].
    fn plan(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &MotionPlanInfo<'m>,
        sampling_time: f64,
    ) -> Result<RobotTrajectory<'m>> {
        let cartesian_limits = self.base.planner_limits().cartesian_limits();
        // Upstream setMaxCartesianSpeed's fallback branch -- see this module's
        // "no per-request Cartesian speed override" deviation note.
        let max_cartesian_speed = cartesian_limits.max_trans_vel;

        let path = PathLine::new(
            &info.start_pose,
            &info.goal_pose,
            max_cartesian_speed / cartesian_limits.max_rot_vel,
        );
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
        let segment = LinSegment {
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

/// A [`PathLine`] time-parametrized by a [`VelocityProfileTrap`] over its own
/// arc length. Upstream builds the equivalent `KDL::Trajectory_Segment(path,
/// vel_profile)` inline inside `plan`; this port names the composition as a
/// type instead, satisfying [`CartesianPath`] for
/// [`generate_joint_trajectory`].
struct LinSegment {
    path: PathLine,
    velocity_profile: VelocityProfileTrap,
}

impl CartesianPath for LinSegment {
    fn duration(&self) -> f64 {
        self.velocity_profile.duration()
    }

    fn pos(&self, t: f64) -> Isometry3 {
        self.path.pos(self.velocity_profile.pos(t))
    }
}

/// Resolve `group_name`'s solver tip frame. Upstream `getSolverTipFrame`,
/// minus the "more than one tip frame" case -- see this module's `#
/// Deviations` for why that case is unrepresentable here.
///
/// # Errors
///
/// [`MoveItErrorCode::Failure`] if no [`static@moveit_kinematics::KINEMATICS_SOLVERS`]
/// entry can be built for `group_name` (upstream's `NoSolverException`).
fn solver_tip_frame(robot_model: &RobotModel, group_name: &str) -> Result<String> {
    let params = SolverParams::default();
    resolve_solver(robot_model, group_name, DEFAULT_SOLVER_NAME, &params)
        .ok()
        .map(|solver| solver.tip_frame().to_string())
        .ok_or(Error::Code(MoveItErrorCode::Failure))
}
