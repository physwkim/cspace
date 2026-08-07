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
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::path_line::PathLine;
use crate::trajectory_functions::{
    CartesianPath, IkContext, compute_link_fk, compute_pose_ik, constraint_pose,
    generate_joint_trajectory, resolve_goal_frame, solver_tip_frame,
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
    /// `req.group_name` (upstream's `getSolverTipFrame` failure).
    /// [`MoveItErrorCode::InvalidGroupName`] if `req.group_name` names no
    /// joint model group. [`MoveItErrorCode::InvalidGoalConstraints`] if
    /// `req.goal` is a joint-space target whose position count does not match
    /// the group's active joint count (upstream's `JointNumberMismatch`).
    /// [`MoveItErrorCode::NoIkSolution`]
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

                let group = robot_model
                    .joint_model_group(&req.group_name)
                    .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
                if positions.len() != group.active_joint_names().len() {
                    return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
                }

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
                frame,
                position,
                orientation,
                target_point_offset,
            } => {
                info.link_name = link_name.clone();
                let local_pose = constraint_pose(position, orientation, target_point_offset);
                info.goal_pose = resolve_goal_frame(ctx, frame.as_deref())? * local_pose;

                let params = SolverParams::default();
                let mut solver =
                    resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
                        .map_err(|_| Error::Code(MoveItErrorCode::NoIkSolution))?;

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
            .map_err(|_| Error::Code(MoveItErrorCode::NoIkSolution))?;

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

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::fs;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_collision::{LinkPaddingScale, ParryCollisionEnv};
    use moveit_geometry::{UnitQuaternion, Vector3};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_scene::PlanningScene;
    use moveit_srdf::SrdfModel;

    use super::*;
    use crate::limits::{CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer};
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

    /// `panda_moveit_config/config/pilz_cartesian_limits.yaml`'s values --
    /// the same figures `command_list_manager.rs`'s own test fixture and the
    /// `tests/pilz_trajectory_lin_parity.rs` fixtures use. `plan`'s
    /// [`VelocityProfileTrap`] needs a nonzero `max_trans_vel`/`max_trans_acc`
    /// to produce a finite-duration profile at all --
    /// [`LimitsContainer::new`] alone leaves `cartesian_limits` all-zero (see
    /// [`crate::limits::LimitsContainer::cartesian_limits`]'s doc), which
    /// makes an unreachable (zero-velocity, zero-acceleration) profile for
    /// any nonzero path length.
    fn panda_generator(model: &RobotModel) -> TrajectoryGeneratorLin<'_> {
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        limits.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.25,
            max_trans_dec: -5.0,
            max_rot_vel: 1.57,
        });
        let base = TrajectoryGenerator::new(model, limits);
        TrajectoryGeneratorLin::new(base, "panda_arm")
    }

    /// panda.srdf's `"ready"` named state for `panda_arm`.
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

    /// Upstream: `PositionConstraint::header.frame_id = "panda_link8"`,
    /// `position = (0, 0, 0.1)` -- 10cm along local Z off the flange, not off
    /// the world origin. `extractMotionPlanInfo` resolves this via
    /// `scene->getFrameTransform("panda_link8") * getConstraintPose(...)`
    /// before any IK is attempted -- see `trajectory_generator.rs`'s own
    /// module doc for the full upstream citation.
    ///
    /// [`Goal::Cartesian`] had no field to carry `"panda_link8"` at all
    /// before this fix: this exact test (`goal: Goal::Cartesian { frame:
    /// Some("panda_link8".to_string()), .. }`) does not compile against the
    /// pre-fix type -- "cannot be represented", not merely "computed wrong".
    #[test]
    fn cartesian_goal_in_a_named_frame_resolves_relative_to_that_frames_current_pose() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_variable_positions_by_name(&ready_positions())
            .unwrap();
        let scene = Arc::new(scene);
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };

        // panda_link8's own pose at "ready", computed independently of the
        // code under test.
        let mut fk_state = scene.current_state().clone();
        let link8_pose = compute_link_fk(&mut fk_state, "panda_link8", &ready_positions()).unwrap();

        let base = TrajectoryGenerator::new(&model, LimitsContainer::new());
        let generator = TrajectoryGeneratorLin::new(base, "panda_arm");
        let req = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: ready_positions(),
                velocity: HashMap::new(),
            },
            goal: Goal::Cartesian {
                link_name: "panda_link8".to_string(),
                frame: Some("panda_link8".to_string()),
                position: Vector3::new(0.0, 0.0, 0.1),
                orientation: UnitQuaternion::identity(),
                target_point_offset: Vector3::zeros(),
            },
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            path_constraints: None,
        };
        let mut info = MotionPlanInfo::new(&scene, &req).unwrap();
        generator
            .extract_motion_plan_info(&ctx, &req, &mut info)
            .expect("panda_link8 is reachable from itself with a pure-Z offset");

        let expected = link8_pose * nalgebra::Translation3::new(0.0, 0.0, 0.1);
        assert_relative_eq!(
            info.goal_pose.translation.vector,
            expected.translation.vector,
            epsilon = 1e-9
        );
        // Not 10cm off the world origin, which is what today's un-transformed
        // `constraint_pose(position, orientation, target_point_offset)` alone
        // would compute.
        assert!(info.goal_pose.translation.vector.z - 0.1 > 1e-3);
    }

    /// The full `generate` pipeline (`validate_request` ->
    /// `cmd_specific_request_validation` -> `extract_motion_plan_info` ->
    /// `plan`) for a Cartesian goal naming `"panda_hand"` -- rigidly
    /// connected to `panda_arm`'s solver tip (`"panda_link8"`) by fixed
    /// joints only, never equal to it (`fixtures/panda.urdf`:
    /// `panda_joint8`/`panda_hand_joint`).
    ///
    /// This is deliberately the whole pipeline, not `check_cartesian_goal`
    /// or `compute_pose_ik` in isolation: before this round's fix,
    /// `check_cartesian_goal` rejected `"panda_hand"` at `validate_request`
    /// (`NoIkSolution`), and even had it not, `plan`'s own
    /// `resolve_solver(..).filter(|solver| solver.tip_frame() ==
    /// info.link_name)` would have rejected it again immediately after --
    /// two gates on the one path, either sufficient alone to fail this test.
    /// Fixing only one and leaving the other is exactly what this test would
    /// catch.
    #[test]
    fn generate_end_to_end_reaches_a_link_rigidly_connected_to_the_solver_tip() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .current_state_mut()
            .set_variable_positions_by_name(&ready_positions())
            .unwrap();
        let scene = Arc::new(scene);
        let env =
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };

        // `panda_hand`'s own pose at "ready", offset 2cm along world Z --
        // computed independently of the code under test, and far enough from
        // "ready" that a converging IK solve proves the fixed-joint offset
        // was actually applied, not merely that start already equalled goal.
        let mut fk_state = scene.current_state().clone();
        let hand_pose = compute_link_fk(&mut fk_state, "panda_hand", &ready_positions()).unwrap();
        let target_position = hand_pose.translation.vector + Vector3::new(0.0, 0.0, 0.02);

        let generator = panda_generator(&model);
        let req = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: ready_positions(),
                velocity: HashMap::new(),
            },
            goal: Goal::Cartesian {
                link_name: "panda_hand".to_string(),
                frame: None,
                position: target_position,
                orientation: hand_pose.rotation,
                target_point_offset: Vector3::zeros(),
            },
            // Matches `tests/fixtures/panda_lin_request.json`'s scaling --
            // 1.0 on both factors drives the real
            // `fixtures/panda.urdf`/panda_moveit_config joint acceleration
            // limits for this short 2cm move past their limit on the very
            // first sample, which is a request-level `PlanningFailed`, not
            // anything to do with the rigid-offset fix under test.
            max_velocity_scaling_factor: 0.1,
            max_acceleration_scaling_factor: 0.1,
            path_constraints: None,
        };

        let response = generator.generate(&ctx, &req, 0.1);
        assert_eq!(response.error_code, MoveItErrorCode::Success);
        let trajectory = response.trajectory.expect("success carries a trajectory");

        let final_state = trajectory.last_way_point().unwrap();
        let final_positions: HashMap<String, f64> = ready_positions()
            .keys()
            .map(|name| (name.clone(), final_state.variable_position(name).unwrap()))
            .collect();
        let mut check_state = scene.current_state().clone();
        let final_hand_pose =
            compute_link_fk(&mut check_state, "panda_hand", &final_positions).unwrap();
        assert_relative_eq!(
            final_hand_pose.translation.vector,
            target_position,
            epsilon = 1e-4
        );
    }
}
