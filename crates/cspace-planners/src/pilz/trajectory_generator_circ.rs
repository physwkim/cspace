// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_generator_circ.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_circ.cpp

//! Circular-arc Cartesian trajectory generation
//! ([`TrajectoryGeneratorCirc`]): a [`crate::pilz::path_circle::PathCircle`]
//! time-parametrized by one [`VelocityProfileTrap`], sampled and IK-solved by
//! [`generate_joint_trajectory`].
//!
//! Ported from upstream `TrajectoryGeneratorCIRC`.
//!
//! # Deviations from upstream
//!
//! - **No per-request Cartesian speed override, `max_trans_dec` unused.**
//!   Same two deviations as
//!   [`crate::pilz::trajectory_generator_lin::TrajectoryGeneratorLin`] — see that
//!   module's own doc; `plan` below takes the identical fallback branch and
//!   builds [`VelocityProfileTrap`] with only `max_trans_acc`.
//! - **`cmdSpecificRequestValidation`'s three checks collapse to one
//!   presence check.** Upstream separately verifies `req.path_constraints.name`
//!   is `"interim"`/`"center"` (else `UnknownPathConstraintName`), that
//!   exactly one `PositionConstraint` is present (else `NoPositionConstraints`),
//!   and that constraint carries exactly one primitive pose (else
//!   `NoPrimitivePose`) — three independent ways the same message-shaped
//!   field could be malformed, all mapped to
//!   [`MoveItErrorCode::InvalidMotionPlan`]. [`MotionPlanRequest::path_constraints`]
//!   is an `Option<PathConstraints>` whose `Circ` variant carries exactly one
//!   point and a two-variant `kind`, so all three malformations are
//!   unrepresentable by construction
//!   (the same "unrepresentable, not merely not-ported" pattern
//!   [`crate::pilz::trajectory_generator`]'s own `# What changed shape, and why`
//!   documents for [`Goal`]) — [`TrajectoryGeneratorCirc::cmd_specific_request_validation`]
//!   only has the `None` case left to check -- plus the one the variant split
//!   adds, a request carrying another command's path constraint, which
//!   upstream's single `Constraints` field could not distinguish at all.
//! - **A `check_cartesian_limits` call is added; upstream's `cmdSpecificRequestValidation`
//!   has none.** `plan` divides by `cartesian_limits.max_rot_vel` unguarded,
//!   same as upstream — but upstream's own reachability of a zero there is
//!   gated by `cartesian_limits_parameters.yaml`'s mandatory (no-default)
//!   parameter declaration, a boundary this port has no equivalent of. See
//!   [`crate::pilz::trajectory_generator::check_cartesian_limits`]'s own doc.
//! - **A joint-space goal's constraint-count check reads the group's active
//!   joint count from [`cspace_core::model::JointModelGroup::active_joint_names`],
//!   not a `size()` comparison against a message list**, since
//!   [`Goal::Joint`] is already a `HashMap` rather than a `Vec` that could
//!   under/over-populate independently of the map's own key set.
//! - **A failed FK for the goal or start pose is silently ignored, matching
//!   upstream exactly** — same as
//!   [`crate::pilz::trajectory_generator_lin::TrajectoryGeneratorLin`]'s own
//!   identical deviation note.
//! - **A Cartesian goal's IK solution is discarded**, same as
//!   `TrajectoryGeneratorLin` — upstream's CIRC only calls `computePoseIK` to
//!   confirm the goal pose is reachable, matching
//!   `CircInverseForGoalIncalculable`'s role as a reachability check, not a
//!   seed for `info.goal_joint_position`.
//! - **`Error::Construct` failures from [`circle_from_center`]/
//!   [`circle_from_interim`]/[`PathCircle::new`] are narrowed to
//!   [`MoveItErrorCode::InvalidMotionPlan`] explicitly**, rather than left as
//!   [`Error::Construct`] for `MotionPlanResponse::failure`'s own error-code
//!   narrowing to collapse to the generic [`MoveItErrorCode::Failure`]. Upstream's three
//!   circle-construction exceptions (`CircleNoPlane`/`CircleToSmall`/
//!   `CenterPointDifferentRadius`) all carry `INVALID_MOTION_PLAN`
//!   specifically, so this port's `.map_err` at each call site preserves that
//!   for oracle parity rather than falling through to the generic code.

use cspace_collision::CollisionEnv;
use cspace_core::error::{Error, MoveItErrorCode, Result};
use cspace_core::geometry::{Isometry3, Vector3};
use cspace_core::kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use cspace_core::state::Posed;
use cspace_core::trajectory::RobotTrajectory;

use crate::pilz::path_circle::{
    CircleGeometry, MAX_COLINEAR_NORM, PathCircle, circle_from_center, circle_from_interim,
};
use crate::pilz::trajectory_functions::{
    CartesianPath, IkContext, compute_link_fk, compute_pose_ik, constraint_pose,
    generate_joint_trajectory, resolve_goal_frame,
};
use crate::pilz::trajectory_generator::{
    CircPathConstraint, CircPathConstraintKind, Goal, MotionPlanInfo, MotionPlanRequest,
    PilzGenerator, TrajectoryGenerator, check_cartesian_limits,
};
use crate::pilz::velocity_profile::KDL_EPSILON;
use crate::pilz::velocity_profile_trap::VelocityProfileTrap;

/// Circular-arc Cartesian trajectory generator.
///
/// Upstream `TrajectoryGeneratorCIRC`. See the [module docs](self) for
/// deviations.
pub struct TrajectoryGeneratorCirc<'m> {
    base: TrajectoryGenerator<'m>,
}

impl<'m> TrajectoryGeneratorCirc<'m> {
    /// Upstream `TrajectoryGeneratorCIRC(robot_model, planner_limits, group_name)`.
    /// `group_name` is accepted (matching upstream's constructor signature) but
    /// unused: upstream's own constructor body only logs it, doing nothing
    /// else with it either.
    pub fn new(base: TrajectoryGenerator<'m>, _group_name: &str) -> Self {
        Self { base }
    }
}

impl<'m, E> PilzGenerator<'m, E> for TrajectoryGeneratorCirc<'m>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    fn base(&self) -> &TrajectoryGenerator<'m> {
        &self.base
    }

    /// Upstream `TrajectoryGeneratorCIRC::cmdSpecificRequestValidation`,
    /// minus its three now-unrepresentable malformations — see the [module
    /// docs](self).
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidMotionPlan`] if `req.path_constraints` is
    /// `None`, or carries another command's constraint, or the base's
    /// planner limits fail [`crate::pilz::trajectory_generator::check_cartesian_limits`]
    /// (added beyond upstream — see that function's own doc; `plan` below
    /// divides by `cartesian_limits.max_rot_vel` unguarded).
    fn cmd_specific_request_validation(&self, req: &MotionPlanRequest) -> Result<()> {
        check_cartesian_limits(self.base.planner_limits())?;
        if circ_path_constraint(req).is_err() {
            return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
        }
        Ok(())
    }

    /// Upstream `TrajectoryGeneratorCIRC::extractMotionPlanInfo`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidLinkName`] if a joint-space goal's
    /// `req.path_constraints`' `link_name` names no link in the robot model
    /// (upstream `UnknownLinkNameOfAuxiliaryPoint`).
    /// [`MoveItErrorCode::InvalidGoalConstraints`] if a joint-space goal's
    /// variable count does not match the group's active joint count
    /// (upstream `NumberOfConstraintsMismatch`).
    /// [`MoveItErrorCode::NoIkSolution`] if `req.goal` is a Cartesian target
    /// with no reachable IK solution (upstream
    /// `CircInverseForGoalIncalculable`).
    fn extract_motion_plan_info(
        &self,
        ctx: &IkContext<'_, 'm, E>,
        req: &MotionPlanRequest,
        info: &mut MotionPlanInfo<'m>,
    ) -> Result<()> {
        info.group_name = req.group_name.clone();
        let robot_model = self.base.robot_model();
        let mut scratch_state = ctx.scene.current_state().clone();

        // Both branches below need it; extracted once since `Option::take`
        // isn't available on a shared `&MotionPlanRequest`.
        let path_constraint = circ_path_constraint(req)?;

        match &req.goal {
            Goal::Joint(positions) => {
                info.link_name = path_constraint.link_name.clone();
                if !robot_model.has_link_model(&info.link_name) {
                    return Err(Error::Code(MoveItErrorCode::InvalidLinkName));
                }

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

        // The frame transform applies unconditionally to the raw point,
        // before the `target_point_offset` re-adjustment below -- upstream
        // applies `center_point_frame_id`'s transform first, then only
        // re-folds through the goal's own orientation/offset for a Cartesian
        // goal (`goal_constraints.front()`'s position constraint is only
        // populated then).
        let center_transform = resolve_goal_frame(ctx, path_constraint.frame.as_deref())?;
        let center_point =
            center_transform.rotation * path_constraint.point + center_transform.translation.vector;
        let point = match &req.goal {
            Goal::Cartesian {
                orientation,
                target_point_offset,
                ..
            } => {
                constraint_pose(&center_point, orientation, target_point_offset)
                    .translation
                    .vector
            }
            Goal::Joint(_) => center_point,
        };
        info.circ_aux_point = Some(crate::pilz::trajectory_generator::CircPathConstraint {
            kind: path_constraint.kind,
            link_name: path_constraint.link_name.clone(),
            // `point` above is already resolved into the planning frame --
            // `frame: None` here means exactly that, not "no frame was
            // given". Carrying `path_constraint.frame` forward unchanged
            // would leave one field meaning two different things depending
            // on which `CircPathConstraint` (the request's raw one, or this
            // resolved one) holds it.
            frame: None,
            point,
        });

        Ok(())
    }

    /// Upstream `TrajectoryGeneratorCIRC::plan`.
    ///
    /// # Errors
    ///
    /// [`MoveItErrorCode::InvalidMotionPlan`] if the circle described by
    /// `info.circ_aux_point` cannot be constructed (upstream
    /// `CircleNoPlane`/`CircleToSmall`/`CenterPointDifferentRadius` — see the
    /// [module docs](self)'s `Error::Construct` narrowing note).
    /// [`MoveItErrorCode::NoIkSolution`] if no [`static@cspace_core::kinematics::KINEMATICS_SOLVERS`]
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
        // Upstream setMaxCartesianSpeed's fallback branch -- see this
        // module's "no per-request Cartesian speed override" deviation note.
        let max_cartesian_speed = cartesian_limits.max_trans_vel;
        let eqradius = max_cartesian_speed / cartesian_limits.max_rot_vel;

        let aux_point = info
            .circ_aux_point
            .as_ref()
            .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

        let path = build_path(
            aux_point.kind,
            aux_point.point,
            &info.start_pose,
            &info.goal_pose,
            eqradius,
        )?;

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
        let segment = CircSegment {
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

/// Upstream `TrajectoryGeneratorCIRC::setPathCIRC`: solve the circle geometry
/// from `kind`/`aux_point`, then build the interpolated
/// [`PathCircle`] from it. See [`crate::pilz::path_circle`]'s own `eps`
/// convention note for why [`circle_from_center`]'s result is paired with
/// [`MAX_COLINEAR_NORM`] but [`circle_from_interim`]'s is paired with
/// [`KDL_EPSILON`].
fn build_path(
    kind: CircPathConstraintKind,
    aux_point: Vector3,
    start: &Isometry3,
    goal: &Isometry3,
    eqradius: f64,
) -> Result<PathCircle> {
    let (geometry, eps): (CircleGeometry, f64) = match kind {
        CircPathConstraintKind::Center => (
            circle_from_center(start.translation.vector, goal.translation.vector, aux_point)
                .map_err(|_| Error::Code(MoveItErrorCode::InvalidMotionPlan))?,
            MAX_COLINEAR_NORM,
        ),
        CircPathConstraintKind::Interim => (
            circle_from_interim(start.translation.vector, goal.translation.vector, aux_point)
                .map_err(|_| Error::Code(MoveItErrorCode::InvalidMotionPlan))?,
            KDL_EPSILON,
        ),
    };
    PathCircle::new(start, goal, &geometry, eqradius, eps)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidMotionPlan))
}

/// A [`PathCircle`] time-parametrized by a [`VelocityProfileTrap`] over its
/// own arc length. Same composition [`crate::pilz::trajectory_generator_lin`]'s
/// `LinSegment` names for [`crate::pilz::path_line::PathLine`].
struct CircSegment {
    path: PathCircle,
    velocity_profile: VelocityProfileTrap,
}

impl CartesianPath for CircSegment {
    fn duration(&self) -> f64 {
        self.velocity_profile.duration()
    }

    fn pos(&self, t: f64) -> Isometry3 {
        self.path.pos(self.velocity_profile.pos(t))
    }
}

/// `req`'s `CIRC` auxiliary point.
///
/// Upstream reads `req.path_constraints` directly and only checks that it is
/// present; [`crate::pilz::trajectory_generator::PathConstraints`] can also hold
/// another command's constraint, so "absent" and "some other command's" are
/// both rejected here, with the same error upstream gives for "absent".
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidMotionPlan`] in either case.
fn circ_path_constraint(req: &MotionPlanRequest) -> Result<&CircPathConstraint> {
    req.path_constraints
        .as_ref()
        .and_then(|pc| pc.as_circ())
        .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::fs;

    use cspace_collision::ParryCollisionEnv;
    use cspace_core::model::{MeshSearchPaths, RobotModel};

    use super::*;
    use crate::pilz::limits::{JointLimit, JointLimitsContainer, LimitsContainer};
    use crate::pilz::trajectory_generator::{PathConstraints, StartState};

    fn load_panda() -> RobotModel {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = cspace_core::srdf::SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
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

    /// [`TrajectoryGeneratorCirc::cmd_specific_request_validation`] rejects a
    /// `LimitsContainer` that never had `set_cartesian_limits` called, even
    /// when `req.path_constraints` is otherwise well-formed -- isolating this
    /// from the sibling "no path constraint" rejection the same function
    /// also performs, so a regression that removed only the path-constraint
    /// check could not pass this test by accident.
    #[test]
    fn cmd_specific_request_validation_rejects_missing_cartesian_limits() {
        let model = load_panda();
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        // Deliberately never call `set_cartesian_limits`.
        let base = TrajectoryGenerator::new(&model, limits);
        let generator = TrajectoryGeneratorCirc::new(base, "panda_arm");

        let req = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState::default(),
            goal: Goal::Joint(HashMap::new()),
            max_velocity_scaling_factor: 0.5,
            max_acceleration_scaling_factor: 0.5,
            path_constraints: Some(PathConstraints::Circ(CircPathConstraint {
                kind: CircPathConstraintKind::Center,
                link_name: "panda_link8".to_string(),
                frame: None,
                point: Vector3::zeros(),
            })),
        };

        // `PilzGenerator` is generic over the collision backend `E`, which
        // nothing below actually needs -- `ParryCollisionEnv` pins it so
        // `.cmd_specific_request_validation()` resolves.
        type G<'m> = TrajectoryGeneratorCirc<'m>;
        match <G as PilzGenerator<ParryCollisionEnv>>::cmd_specific_request_validation(
            &generator, &req,
        ) {
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan)) => {}
            other => panic!("expected Error::Code(InvalidMotionPlan), got {other:?}"),
        }
    }
}
