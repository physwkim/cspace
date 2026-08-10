// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blender_transition_window.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blend_request.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/trajectory_blend_response.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_blender_transition_window.cpp

//! Blends two consecutive joint trajectories into one continuous motion,
//! replacing their shared stop with a smoothed transition inside a sphere
//! of `blend_radius` centred on the junction. Upstream
//! `TrajectoryBlenderTransitionWindow`.
//!
//! # ROS dependencies found, and how each was resolved
//!
//! Checked individually, matching this crate's `lib.rs` "ROS dependencies
//! found" section:
//!
//! - `rclcpp::Logger`/`RCLCPP_INFO`/`RCLCPP_ERROR`/`RCLCPP_DEBUG`/
//!   `RCLCPP_ERROR_STREAM`/`RCLCPP_INFO_STREAM` (`<moveit/utils/logger.hpp>`)
//!   are used exclusively for logging; every one is replaced by a `Result`
//!   return instead of a log-and-continue, the same pattern the three files
//!   before this one already established.
//! - `<tf2_eigen/tf2_eigen.hpp>` is used for exactly one call,
//!   `tf2::toMsg(blend_sample_pose)`, converting `Eigen::Isometry3d` to
//!   `geometry_msgs::msg::Pose` for [`CartesianTrajectoryPoint::pose`].
//!   That field is already [`cspace_geometry::Isometry3`] here (see
//!   [`crate::cartesian_trajectory`]'s own `# Deviations`), so the
//!   conversion has nothing left to do — dropped as a message-shape
//!   exclusion (`PORTING-PLAN.md` D1/D2), not a ROS dependency this port
//!   had to work around.
//! - `moveit_msgs::msg::MoveItErrorCodes` (in `TrajectoryBlendResponse`
//!   and `validateRequest`'s out-parameter) is replaced by
//!   [`cspace_error::MoveItErrorCode`], via [`blend`]'s `Result` — see
//!   [`TrajectoryBlendResponse`]'s own doc for why the field itself is
//!   dropped rather than carried.
//! - `trajectory_msgs::msg::JointTrajectory` (the `blend_joint_trajectory`
//!   intermediate `blend()` builds before converting it via
//!   `setRobotTrajectoryMsg`) has no counterpart: this port's
//!   [`generate_joint_trajectory_from_cartesian`] already returns a
//!   [`RobotTrajectory`] directly (see `trajectory_functions`'s own
//!   `# Deviations`, item 3), so [`blend`] uses that return value as
//!   `blend_trajectory` with no message-shaped intermediate at all.
//! - `rclcpp::Duration::from_seconds(...)` has no counterpart:
//!   [`CartesianTrajectoryPoint::time_from_start`] is already a plain
//!   `f64` in seconds (see `cartesian_trajectory`'s own `# Deviations`).
//!
//! No computation in this file depends on ROS; every dependency above is
//! either logging (dropped) or a message-shape conversion this crate
//! already excludes everywhere else it appears.
//!
//! # Licence
//!
//! `trajectory_blender_transition_window.{hpp,cpp}`,
//! `trajectory_blend_request.hpp` and `trajectory_blend_response.hpp` all
//! carry the identical "Software License Agreement (BSD License) Copyright
//! (c) 2018 Pilz GmbH & Co. KG" header every other file in this crate's
//! `pilz_industrial_motion_planner` citation carries — read directly from
//! each file before porting, per this round's instruction. No LGPL or
//! Apache-2.0 surprise: `tools/ci/verify-upstream-license-provenance.sh`
//! needs no new exemption for this module.
//!
//! # `TrajectoryBlender`, the abstract base class, is not ported
//!
//! Upstream declares `TrajectoryBlender` (`trajectory_blender.hpp`) as a
//! pure-virtual base with one method, `blend`, and
//! `TrajectoryBlenderTransitionWindow` as its only override —
//! `rg -rl "public TrajectoryBlender" moveit_planners/pilz_industrial_motion_planner/`
//! against the upstream tree finds exactly this one subclass anywhere in
//! the whole package. With a single implementation and no second caller
//! needing to select between blend algorithms at runtime, there is nothing
//! for a Rust trait to dispatch over — [`blend`] below is a plain function
//! taking `planner_limits` as a parameter, matching this crate's
//! established pattern of collapsing a single concrete override into a
//! function rather than porting virtual dispatch that has exactly one
//! implementor (see e.g. [`crate::trajectory_generator::check_cartesian_goal`],
//! which replaces upstream's `getSolverInstance()` virtual lookup the same
//! way).
//!
//! # Reuse, not reimplementation
//!
//! Every helper upstream's `blend`/`validateRequest`/
//! `searchIntersectionPoints` call was already ported by
//! [`crate::trajectory_functions`] for `LIN`/`PTP`/`CIRC`'s own use, before
//! this module existed:
//! [`determine_and_check_sampling_time`], [`is_robot_state_equal`],
//! [`is_robot_state_stationary`], [`linear_search_intersection_point`] and
//! [`generate_joint_trajectory_from_cartesian`]. None of the five is
//! re-transcribed here — this module only adds the two pieces of geometry
//! genuinely unique to the transition-window algorithm,
//! `determine_trajectory_alignment` and `blend_trajectory_cartesian`,
//! plus the orchestration in [`blend`] itself.
//!
//! # Deviations from upstream
//!
//! - **`link_name` naming an attached body resolves through `scene`, not
//!   through `first_trajectory`'s own waypoints.** Upstream's
//!   `hasAttachedBody`/`getFrameTransform` reach an attached body through
//!   `RobotState`'s own `attached_body_map_`; this port's states carry no
//!   attached bodies at all (see `cspace-scene`'s `attached_body` module
//!   doc — they live on [`cspace_scene::PlanningScene`] instead), so
//!   `validate_request`, `search_intersection_points` and
//!   `blend_trajectory_cartesian` each take an explicit
//!   `scene: &PlanningScene` parameter (matching [`blend`]'s own
//!   `ctx.scene`) and resolve `link_name` through
//!   `crate::trajectory_functions::resolve_link_or_attached_body_transform`
//!   rather than a bare [`cspace_state::Posed::frame_transform`] on a
//!   trajectory waypoint alone.
//! - **`TrajectoryBlendRequest`/`TrajectoryBlendResponse` own their
//!   trajectories instead of sharing them.** Upstream's
//!   `robot_trajectory::RobotTrajectoryPtr` is a `std::shared_ptr`;
//!   [`TrajectoryBlendRequest::first_trajectory`]/`second_trajectory` are
//!   plain owned [`RobotTrajectory`]s. Nothing in this crate's ported
//!   scope needs shared ownership after [`blend`] returns —
//!   `command_list_manager`, upstream's only caller, is excluded
//!   (`PORTING-PLAN.md` D1/D2).
//! - **`TrajectoryBlendResponse` carries no `error_code`.** [`blend`]
//!   reports failure through its own `Result`, the idiom every other
//!   function in this crate uses, rather than upstream's always-returned
//!   response-plus-error-code pair. Contrast
//!   [`crate::trajectory_generator::MotionPlanResponse`], which keeps its
//!   `error_code` field: that type is [`crate::trajectory_generator::PilzGenerator::generate`]'s
//!   own outward boundary, with no `Result` to report through instead (see
//!   that module's `# generate, MotionPlanInfo, MotionPlanResponse`
//!   section).
//! - **A dead reassignment is not reproduced.** Upstream's
//!   `blendTrajectoryCartesian` computes
//!   `blend_sample_pose2 = req.second_trajectory->getWayPoint(second_interse_index)...`
//!   and then, before the value is ever read, immediately overwrites it
//!   with `req.second_trajectory->getFirstWayPoint()...` on the very next
//!   line. `blend_trajectory_cartesian` computes the surviving value
//!   directly and skips the dead first assignment.
//! - **`response.second_trajectory`'s waypoint-0 duration is always `0.0`,
//!   not `sampling_time`.** Upstream's `second_trajectory` copy loop inserts
//!   waypoint 0 with `getWayPointDurationFromPrevious(second_intersection_index
//!   + 1)` (in general nonzero, an arbitrary carried-over gap from the
//!   original trajectory) and then unconditionally overwrites it with
//!   `setWayPointDurationFromPrevious(0, sampling_time)` right after the
//!   loop. Neither value survives here: [`RobotTrajectory`]'s own
//!   pre-existing "Deviations from upstream" note already normalizes
//!   waypoint 0's duration to always be `0.0` (there is no previous waypoint
//!   within the trajectory for it to measure a gap from — the same
//!   observation upstream's own `getAverageSegmentDuration` comment makes in
//!   passing), so [`RobotTrajectory::set_way_point_duration_from_previous`]
//!   rejects any nonzero value at index 0 unconditionally, not only at
//!   insertion. [`blend`] does not attempt to route around that — it drops
//!   the value upstream would have stored there, matching every other
//!   `RobotTrajectory` in this port.

use std::collections::HashMap;

use cspace_collision::CollisionEnv;
use cspace_error::{Error, MoveItErrorCode, Result};
use cspace_geometry::{Isometry3, quaternion};
use cspace_kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use cspace_scene::PlanningScene;
use cspace_state::Posed;
use cspace_trajectory::RobotTrajectory;

use crate::cartesian_trajectory::{CartesianTrajectory, CartesianTrajectoryPoint, Twist};
use crate::limits::LimitsContainer;
use crate::trajectory_functions::{
    IkContext, determine_and_check_sampling_time, generate_joint_trajectory_from_cartesian,
    is_robot_state_equal, is_robot_state_stationary, linear_search_intersection_point,
    resolve_link_or_attached_body_transform,
};

/// Constant to check for equality of values. Upstream
/// `TrajectoryBlenderTransitionWindow::EPSILON`.
const EPSILON: f64 = 1e-4;

/// A request to blend `first_trajectory` and `second_trajectory` around
/// their shared boundary. Upstream `TrajectoryBlendRequest`
/// (`trajectory_blend_request.hpp`). See this module's `# Deviations` for
/// why the two trajectories are owned rather than shared.
pub struct TrajectoryBlendRequest<'m> {
    /// The planning group. Upstream `group_name`.
    pub group_name: String,
    /// The target link. Upstream `link_name`.
    pub link_name: String,
    /// The trajectory blending starts from; its last waypoint must equal
    /// `second_trajectory`'s first (within `EPSILON`), both stationary.
    /// Upstream `first_trajectory`.
    pub first_trajectory: RobotTrajectory<'m>,
    /// The trajectory blending continues into. Upstream `second_trajectory`.
    pub second_trajectory: RobotTrajectory<'m>,
    /// Blend radius, in metres, of the sphere centred on the shared
    /// boundary. Upstream `blend_radius`.
    pub blend_radius: f64,
}

/// The outcome of [`blend`]: `first_trajectory`'s portion outside the
/// blend sphere, the blend itself, and `second_trajectory`'s portion
/// outside the blend sphere. Upstream `TrajectoryBlendResponse`
/// (`trajectory_blend_response.hpp`). See this module's `# Deviations` for
/// why `error_code` is not carried.
pub struct TrajectoryBlendResponse<'m> {
    /// Upstream `group_name`.
    pub group_name: String,
    /// The part of `first_trajectory` outside the blend sphere. Upstream
    /// `first_trajectory`.
    pub first_trajectory: RobotTrajectory<'m>,
    /// The smoothed transition inside the blend sphere. Upstream
    /// `blend_trajectory`.
    pub blend_trajectory: RobotTrajectory<'m>,
    /// The part of `second_trajectory` outside the blend sphere. Upstream
    /// `second_trajectory`.
    pub second_trajectory: RobotTrajectory<'m>,
}

/// Blend `req.first_trajectory` and `req.second_trajectory` using the
/// transition-window algorithm. Upstream
/// `TrajectoryBlenderTransitionWindow::blend`.
///
/// # Errors
///
/// See `validate_request`'s `# Errors` for a malformed `req`.
/// [`MoveItErrorCode::InvalidMotionPlan`] if `req.blend_radius` is too
/// large for either trajectory to have a crossing point (upstream's
/// "Blend radius too large"). [`MoveItErrorCode::NoIkSolution`] if no
/// [`static@cspace_kinematics::KINEMATICS_SOLVERS`] entry can be built for
/// `req.group_name` with `req.link_name` as its tip, or the blended
/// Cartesian path is not reachable from it.
/// [`MoveItErrorCode::PlanningFailed`] if a blended sample violates a
/// joint limit.
pub fn blend<'m, E>(
    ctx: &IkContext<'_, 'm, E>,
    planner_limits: &LimitsContainer,
    req: &mut TrajectoryBlendRequest<'m>,
) -> Result<TrajectoryBlendResponse<'m>>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    let sampling_time = validate_request(ctx.scene, req)?;

    let (first_intersection_index, second_intersection_index) =
        search_intersection_points(ctx.scene, req)?;

    let blend_align_index =
        determine_trajectory_alignment(req, first_intersection_index, second_intersection_index);

    let blend_trajectory_cartesian = blend_trajectory_cartesian(
        ctx.scene,
        req,
        first_intersection_index,
        second_intersection_index,
        blend_align_index,
        sampling_time,
    );

    let robot_model = req.first_trajectory.robot_model();
    let group = robot_model
        .joint_model_group(&req.group_name)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;

    let mut initial_joint_position = HashMap::new();
    let mut initial_joint_velocity = HashMap::new();
    for name in group.active_joint_names() {
        let source = req
            .first_trajectory
            .way_point(first_intersection_index - 1)?;
        initial_joint_position.insert(name.clone(), source.variable_position(name)?);
        initial_joint_velocity.insert(name.clone(), source.variable_velocity(name)?);
    }

    let params = SolverParams::default();
    let mut solver = resolve_solver(robot_model, &req.group_name, DEFAULT_SOLVER_NAME, &params)
        .map_err(|_| Error::Code(MoveItErrorCode::NoIkSolution))?;

    let blend_joint_trajectory = generate_joint_trajectory_from_cartesian(
        ctx,
        solver.as_mut(),
        planner_limits.joint_limits(),
        &blend_trajectory_cartesian,
        &req.link_name,
        &initial_joint_position,
        &initial_joint_velocity,
    )?;

    let mut first_trajectory = RobotTrajectory::for_group_name(robot_model, &req.group_name)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
    for i in 0..first_intersection_index {
        first_trajectory.insert_way_point(
            i,
            req.first_trajectory.way_point(i)?.clone(),
            req.first_trajectory.way_point_duration_from_previous(i),
        )?;
    }

    let mut second_trajectory = RobotTrajectory::for_group_name(robot_model, &req.group_name)
        .map_err(|_| Error::Code(MoveItErrorCode::InvalidGroupName))?;
    let second_count = req.second_trajectory.way_point_count();
    for i in (second_intersection_index + 1)..second_count {
        let index = i - (second_intersection_index + 1);
        // See this module's `# Deviations`: index 0's duration is always
        // `0.0` in this port, so upstream's `setWayPointDurationFromPrevious(0,
        // sampling_time)` afterward has no equivalent here — the interval
        // upstream would have stored there is dropped, not corrected.
        let dt = if index == 0 {
            0.0
        } else {
            req.second_trajectory.way_point_duration_from_previous(i)
        };
        second_trajectory.insert_way_point(
            index,
            req.second_trajectory.way_point(i)?.clone(),
            dt,
        )?;
    }

    Ok(TrajectoryBlendResponse {
        group_name: req.group_name.clone(),
        first_trajectory,
        blend_trajectory: blend_joint_trajectory,
        second_trajectory,
    })
}

/// Validate `req` before blending, returning the sampling time shared by
/// both trajectories on success. Upstream `validateRequest`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `req.group_name` names no group.
/// [`MoveItErrorCode::InvalidLinkName`] if `req.link_name` names neither a
/// link nor an attached body in `scene` (upstream's `hasAttachedBody`
/// fallback — see [`PlanningScene::has_attached_body`]).
/// [`MoveItErrorCode::InvalidMotionPlan`] if `req.blend_radius` is not
/// positive, the trajectories' shared boundary state does not match within
/// `EPSILON` (see [`is_robot_state_equal`]), no consistent sampling time
/// can be determined (see [`determine_and_check_sampling_time`]), or that
/// boundary state has nonzero velocity/acceleration (see
/// [`is_robot_state_stationary`]).
fn validate_request(scene: &PlanningScene<'_>, req: &TrajectoryBlendRequest<'_>) -> Result<f64> {
    let robot_model = req.first_trajectory.robot_model();

    if !robot_model.has_joint_model_group(&req.group_name) {
        return Err(Error::Code(MoveItErrorCode::InvalidGroupName));
    }

    if !robot_model.has_link_model(&req.link_name) && !scene.has_attached_body(&req.link_name) {
        return Err(Error::Code(MoveItErrorCode::InvalidLinkName));
    }

    if req.blend_radius <= 0.0 {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }

    if !is_robot_state_equal(
        req.first_trajectory.last_way_point()?,
        req.second_trajectory.first_way_point()?,
        &req.group_name,
        EPSILON,
    ) {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }

    let sampling_time =
        determine_and_check_sampling_time(&req.first_trajectory, &req.second_trajectory, EPSILON)
            .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

    if !is_robot_state_stationary(
        req.first_trajectory.last_way_point()?,
        &req.group_name,
        EPSILON,
    ) || !is_robot_state_stationary(
        req.second_trajectory.first_way_point()?,
        &req.group_name,
        EPSILON,
    ) {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }

    Ok(sampling_time)
}

/// Find the waypoint indices at which `req.first_trajectory`/
/// `req.second_trajectory` cross the blend sphere centred on
/// `req.first_trajectory`'s last waypoint. Upstream
/// `searchIntersectionPoints`.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidMotionPlan`] if `req.blend_radius` is too
/// large for either trajectory to have a crossing point (upstream's
/// "Blend radius too large" — both linear searches share this one error
/// code, matching upstream: `searchIntersectionPoints` itself returns only
/// a `bool`, and `blend` assigns `INVALID_MOTION_PLAN` for either cause).
fn search_intersection_points<'m>(
    scene: &PlanningScene<'m>,
    req: &mut TrajectoryBlendRequest<'m>,
) -> Result<(usize, usize)> {
    let circ_pose = {
        let state = req
            .first_trajectory
            .last_way_point_mut()
            .expect("validate_request already confirmed first_trajectory is non-empty");
        let posed = state.update();
        resolve_link_or_attached_body_transform(scene, &posed, &req.link_name)
            .expect("validate_request already confirmed link_name resolves")
    };

    let first_index = linear_search_intersection_point(
        scene,
        &req.link_name,
        &circ_pose.translation.vector,
        req.blend_radius,
        &mut req.first_trajectory,
        true,
    )
    .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

    let second_index = linear_search_intersection_point(
        scene,
        &req.link_name,
        &circ_pose.translation.vector,
        req.blend_radius,
        &mut req.second_trajectory,
        false,
    )
    .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

    Ok((first_index, second_index))
}

/// Determine how `req.second_trajectory` should be aligned with
/// `req.first_trajectory` for the blend, returning the alignment index.
/// Upstream `determineTrajectoryAlignment`; see that method's own
/// (upstream) doc comment for the two-branch diagram this mirrors exactly.
fn determine_trajectory_alignment(
    req: &TrajectoryBlendRequest<'_>,
    first_intersection_index: usize,
    second_intersection_index: usize,
) -> usize {
    let way_point_count_1 = req.first_trajectory.way_point_count() - first_intersection_index;
    let way_point_count_2 = second_intersection_index + 1;

    if way_point_count_1 > way_point_count_2 {
        req.first_trajectory.way_point_count() - second_intersection_index - 1
    } else {
        first_intersection_index
    }
}

/// Blend `req.first_trajectory` and `req.second_trajectory` in Cartesian
/// space between the two intersection indices, using a quintic
/// (`6s^5 - 15s^4 + 10s^3`) smoothstep of `req.link_name`'s pose — zero
/// velocity and acceleration at both ends, matching the stationary
/// boundary `validate_request` already confirmed. Upstream
/// `blendTrajectoryCartesian`. See this module's `# Deviations` for the
/// upstream dead reassignment this skips.
fn blend_trajectory_cartesian<'m>(
    scene: &PlanningScene<'m>,
    req: &mut TrajectoryBlendRequest<'m>,
    first_intersection_index: usize,
    second_intersection_index: usize,
    blend_align_index: usize,
    sampling_time: f64,
) -> CartesianTrajectory {
    let frame_transform_at =
        |traj: &mut RobotTrajectory<'m>, index: usize, link_name: &str| -> Isometry3 {
            let state = traj
                .way_point_mut(index)
                .expect("index within way_point_count");
            let posed = state.update();
            resolve_link_or_attached_body_transform(scene, &posed, link_name)
                .expect("link_name resolves for every waypoint of the same trajectory")
        };

    let mut blend_sample_pose1 = frame_transform_at(
        &mut req.first_trajectory,
        first_intersection_index,
        &req.link_name,
    );
    let mut blend_sample_pose2 = frame_transform_at(&mut req.second_trajectory, 0, &req.link_name);

    let blend_sample_num =
        (second_intersection_index + blend_align_index + 1) - first_intersection_index;
    let mut points = Vec::with_capacity(blend_sample_num);

    for i in 0..blend_sample_num {
        if (first_intersection_index + i) < req.first_trajectory.way_point_count() {
            blend_sample_pose1 = frame_transform_at(
                &mut req.first_trajectory,
                first_intersection_index + i,
                &req.link_name,
            );
        }

        if (first_intersection_index + i) > blend_align_index {
            blend_sample_pose2 = frame_transform_at(
                &mut req.second_trajectory,
                first_intersection_index + i - blend_align_index,
                &req.link_name,
            );
        }

        let s = (i as f64 + 1.0) / blend_sample_num as f64;
        let alpha = 6.0 * s.powi(5) - 15.0 * s.powi(4) + 10.0 * s.powi(3);

        let translation = blend_sample_pose1.translation.vector
            + alpha
                * (blend_sample_pose2.translation.vector - blend_sample_pose1.translation.vector);
        // Upstream is `start_quat.slerp(alpha, end_quat).toRotationMatrix()`
        // (`trajectory_blender_transition_window.cpp:259`), i.e.
        // `Eigen::Quaterniond::slerp` — not nalgebra's, which differs from it
        // in three measured ways and additionally panics at 180 degrees where
        // Eigen returns `start_quat`.
        // Upstream is `start_quat.slerp(alpha, end_quat).toRotationMatrix()`
        // (`trajectory_blender_transition_window.cpp:259`), i.e.
        // `Eigen::Quaterniond::slerp` — not nalgebra's, which differs from it
        // in three measured ways and additionally panics at 180 degrees where
        // Eigen returns `start_quat`.
        let rotation = quaternion::slerp(
            &blend_sample_pose1.rotation,
            &blend_sample_pose2.rotation,
            alpha,
        );

        points.push(CartesianTrajectoryPoint {
            pose: Isometry3::from_parts(translation.into(), rotation),
            velocity: Twist::default(),
            acceleration: Twist::default(),
            time_from_start: (i as f64 + 1.0) * sampling_time,
        });
    }

    CartesianTrajectory {
        group_name: req.group_name.clone(),
        link_name: req.link_name.clone(),
        points,
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet};
    use std::fs;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use cspace_collision::{LinkPaddingScale, ParryCollisionEnv};
    use cspace_model::{MeshSearchPaths, RobotModel};
    use cspace_scene::PlanningScene;
    use cspace_srdf::SrdfModel;
    use cspace_state::RobotState;

    use cspace_geometry::{Cuboid, Isometry3, Shape, UnitQuaternion, Vector3};

    use super::*;
    use crate::limits::{CartesianLimits, JointLimit, JointLimitsContainer};
    use crate::trajectory_generator::{
        Goal, MotionPlanRequest, PilzGenerator, StartState, TrajectoryGenerator,
    };
    use crate::trajectory_generator_lin::TrajectoryGeneratorLin;

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        let mesh_paths = MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )]);
        let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_paths)
            .expect("fixture model must build");
        (model, srdf)
    }

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

    /// A stationary waypoint at `positions`, settled via [`RobotState::update`]
    /// (through [`RobotTrajectory::add_suffix_way_point`]) so its frame
    /// transforms are ready to read.
    fn stationary_state<'m>(
        model: &'m RobotModel,
        positions: &HashMap<String, f64>,
    ) -> RobotState<'m> {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, &value) in positions {
            state.set_variable_position(name, value).unwrap();
        }
        state
    }

    /// A single-joint (`panda_joint1`) sweep trajectory from `ready_positions`,
    /// rotating `panda_joint1` from `start_angle_offset` to `end_angle_offset`
    /// (both added to `ready_positions`'s own `panda_joint1` value) over
    /// `steps` uniform `sampling_time`-spaced waypoints, stationary at both
    /// ends. Two calls sharing an angle offset at one call's end and the
    /// other's start produce trajectories whose shared boundary state
    /// matches exactly, e.g.
    /// `panda_joint1_sweep(m, 0.0, 0.2, n, dt)`/`panda_joint1_sweep(m, 0.2, 0.4, n, dt)`.
    fn panda_joint1_sweep<'m>(
        model: &'m RobotModel,
        start_angle_offset: f64,
        end_angle_offset: f64,
        steps: usize,
        sampling_time: f64,
    ) -> RobotTrajectory<'m> {
        let base = ready_positions();
        let mut traj = RobotTrajectory::for_group_name(model, "panda_arm").unwrap();
        for i in 0..=steps {
            let mut positions = base.clone();
            let angle = base["panda_joint1"]
                + start_angle_offset
                + (end_angle_offset - start_angle_offset) * (i as f64) / (steps as f64);
            positions.insert("panda_joint1".to_string(), angle);
            let mut state = stationary_state(model, &positions);
            if i == 0 || i == steps {
                // Both ends are stationary -- validate_request/is_robot_state_stationary
                // require zero velocity, which `set_to_default_values` already
                // gives every waypoint (no velocity is ever set here).
            } else {
                let velocity =
                    (end_angle_offset - start_angle_offset) / (steps as f64 * sampling_time);
                state
                    .set_variable_velocity("panda_joint1", velocity)
                    .unwrap();
            }
            let dt = if i == 0 { 0.0 } else { sampling_time };
            traj.add_suffix_way_point(state, dt).unwrap();
        }
        traj
    }

    /// One Cartesian-space LIN segment from `start` to `goal_pos`, fixed
    /// orientation (the SRDF `"ready"` pose's own orientation -- the same
    /// value `doc/oracle-request-pilz-blend.md`'s case A/B and
    /// `doc/oracle-request-pilz-blend-geometry.md`'s case C/D use), used by
    /// the `search_intersection_points` geometry tests below to build real
    /// Cartesian corners without going through `blend()`/end-to-end
    /// generator machinery each time.
    fn gen_lin_segment<'m>(
        model: &'m RobotModel,
        limits: &LimitsContainer,
        ctx: &IkContext<'_, 'm, ParryCollisionEnv>,
        start: &HashMap<String, f64>,
        goal_pos: [f64; 3],
        scaling: f64,
        sampling_time: f64,
    ) -> RobotTrajectory<'m> {
        let base = TrajectoryGenerator::new(model, limits.clone());
        let generator = TrajectoryGeneratorLin::new(base, "panda_arm");
        let goal = Goal::Cartesian {
            link_name: "panda_link8".to_string(),
            frame: None,
            position: Vector3::new(goal_pos[0], goal_pos[1], goal_pos[2]),
            orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                3.2004117663522442e-12,
                0.9239556994689483,
                -0.38249949727920757,
                1.324932583900579e-12,
            )),
            target_point_offset: Vector3::new(0.0, 0.0, 0.0),
        };
        let req = MotionPlanRequest {
            group_name: "panda_arm".to_string(),
            start_state: StartState {
                position: start.clone(),
                velocity: HashMap::new(),
            },
            goal,
            max_velocity_scaling_factor: scaling,
            max_acceleration_scaling_factor: scaling,
            path_constraints: None,
        };
        let response = generator.generate(ctx, &req, sampling_time);
        response
            .trajectory
            .unwrap_or_else(|| panic!("LIN segment must succeed, got {:?}", response.error_code))
    }

    /// `blend_radius`/Cartesian limits shared by every
    /// `search_intersection_points` geometry test below -- the exact values
    /// `doc/oracle-request-pilz-blend.md`'s case A/B request.
    fn blend_geometry_cartesian_limits() -> CartesianLimits {
        CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.25,
            max_trans_dec: -5.0,
            max_rot_vel: 1.57,
        }
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
                    has_velocity_limits: true,
                    max_velocity: 10.0,
                    has_acceleration_limits: true,
                    max_acceleration: 100.0,
                    has_deceleration_limits: true,
                    max_deceleration: -100.0,
                    ..Default::default()
                },
            );
        }
        limits
    }

    // -- validate_request: group name, link name, blend radius, boundary
    // mismatch, sampling time, stationarity -- one case per check --

    #[test]
    fn validate_request_rejects_an_unknown_group_name() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let traj = panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1);
        let req = TrajectoryBlendRequest {
            group_name: "no_such_group".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: traj,
            blend_radius: 0.05,
        };
        match validate_request(&scene, &req) {
            Err(Error::Code(MoveItErrorCode::InvalidGroupName)) => {}
            other => panic!("expected InvalidGroupName, got {other:?}"),
        }
    }

    #[test]
    fn validate_request_rejects_an_unknown_link_name() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "no_such_link".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            blend_radius: 0.05,
        };
        match validate_request(&scene, &req) {
            Err(Error::Code(MoveItErrorCode::InvalidLinkName)) => {}
            other => panic!("expected InvalidLinkName, got {other:?}"),
        }
    }

    #[test]
    fn validate_request_rejects_blend_radius_at_or_below_zero() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        // A chained pair (unlike an earlier version of this test, which
        // paired a sweep with an independent, non-chained copy of itself):
        // first_trajectory's last waypoint must equal second_trajectory's
        // first, or the boundary-mismatch check a few lines above the
        // blend_radius check would also fire and reject the request for a
        // second, unrelated reason -- masking whether the blend_radius
        // check on its own actually did anything. Mutation testing this
        // round caught exactly that: with the blend_radius check deleted,
        // this test still passed, because the earlier, non-chained version
        // of this fixture failed the boundary check instead.
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1),
            blend_radius: 0.0,
        };
        assert!(matches!(
            validate_request(&scene, &req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
        req.blend_radius = -0.01;
        assert!(matches!(
            validate_request(&scene, &req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_rejects_a_boundary_state_mismatch() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut second = panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1);
        // Perturb second_trajectory's first waypoint so it no longer matches
        // first_trajectory's last waypoint.
        second
            .way_point_mut(0)
            .unwrap()
            .set_variable_position("panda_joint2", -0.5)
            .unwrap();
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: second,
            blend_radius: 0.05,
        };
        assert!(matches!(
            validate_request(&scene, &req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_rejects_a_mismatched_sampling_time() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.05),
            blend_radius: 0.05,
        };
        assert!(matches!(
            validate_request(&scene, &req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_rejects_non_stationary_boundary_waypoints() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut first = panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1);
        let mut second = panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1);
        // Give both boundary waypoints the SAME nonzero velocity, not just
        // one: is_robot_state_equal (:352) compares velocity too, so a
        // one-sided perturbation would trip the boundary-mismatch guard
        // instead of the stationarity guard this test targets. Matching
        // velocities keep :352 satisfied while still failing
        // is_robot_state_stationary on each end individually.
        let last = first.way_point_count() - 1;
        first
            .way_point_mut(last)
            .unwrap()
            .set_variable_velocity("panda_joint1", 0.05)
            .unwrap();
        second
            .way_point_mut(0)
            .unwrap()
            .set_variable_velocity("panda_joint1", 0.05)
            .unwrap();
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: first,
            second_trajectory: second,
            blend_radius: 0.05,
        };
        assert!(matches!(
            validate_request(&scene, &req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_accepts_a_well_formed_request() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1),
            blend_radius: 0.05,
        };
        assert_relative_eq!(validate_request(&scene, &req).unwrap(), 0.1);
    }

    // -- determine_trajectory_alignment: way_point_count_1 > way_point_count_2
    // vs the else branch -- boundary is the strict `>` itself --

    #[test]
    fn determine_trajectory_alignment_picks_first_trajectory_tail_when_it_is_longer() {
        let (model, _) = load_panda();
        // first_trajectory: 11 waypoints (indices 0..=10); first_intersection
        // at 2 -> way_point_count_1 = 11 - 2 = 9.
        // second_intersection at 3 -> way_point_count_2 = 3 + 1 = 4.
        // 9 > 4, so the first branch: blend_align_index = 11 - 3 - 1 = 7.
        let first = panda_joint1_sweep(&model, 0.0, 0.2, 10, 0.1);
        let second = panda_joint1_sweep(&model, 0.2, 0.4, 10, 0.1);
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: first,
            second_trajectory: second,
            blend_radius: 0.05,
        };
        assert_eq!(determine_trajectory_alignment(&req, 2, 3), 7);
    }

    #[test]
    fn determine_trajectory_alignment_picks_first_intersection_index_otherwise() {
        let (model, _) = load_panda();
        // first_trajectory: 5 waypoints (indices 0..=4); first_intersection
        // at 3 -> way_point_count_1 = 5 - 3 = 2.
        // second_intersection at 3 -> way_point_count_2 = 3 + 1 = 4.
        // 2 is not > 4, so the else branch: blend_align_index =
        // first_intersection_index = 3.
        let first = panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1);
        let second = panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1);
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: first,
            second_trajectory: second,
            blend_radius: 0.05,
        };
        assert_eq!(determine_trajectory_alignment(&req, 3, 3), 3);
    }

    // -- search_intersection_points: a blend radius small enough to be
    // crossed by both trajectories vs one too large for either --

    #[test]
    fn search_intersection_points_finds_both_crossings_within_radius() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let (first_index, second_index) = search_intersection_points(&scene, &mut req).unwrap();
        // The center is first_trajectory's own last waypoint, so the first
        // trajectory's crossing must be strictly before its own end.
        assert!(first_index < req.first_trajectory.way_point_count() - 1);
        assert!(second_index < req.second_trajectory.way_point_count());
    }

    // The two calls in search_intersection_points are independently
    // `?`-chained (first_trajectory's inverse-order search, then
    // second_trajectory's forward search against the same center). A test
    // that makes both fail to cross at once cannot tell which call actually
    // produced the Err -- forcing either one to succeed still leaves the
    // other failing, so the overall outcome never changes and the mutation
    // that broke only one of the two calls survives. These two tests each
    // keep the OTHER trajectory a known crosser (the geometry from
    // search_intersection_points_finds_both_crossings_within_radius above)
    // so only the trajectory under test can be the cause of the Err.

    #[test]
    fn search_intersection_points_rejects_when_first_trajectory_never_reaches_the_blend_radius() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.005, 0.0, 10, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        assert!(matches!(
            search_intersection_points(&scene, &mut req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn search_intersection_points_rejects_when_second_trajectory_never_reaches_the_blend_radius() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.005, 10, 0.05),
            blend_radius: 0.05,
        };
        assert!(matches!(
            search_intersection_points(&scene, &mut req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    // -- search_intersection_points: real Cartesian-corner geometry, not
    // just the joint-space sweeps above -- doc/oracle-request-pilz-blend-geometry.md's
    // case C (blend_radius) and case D (corner angle) predictions, pinned as
    // permanent tests rather than left in that document's prose. See
    // doc/mutation-audit-trajectory-blender.md's "Rows 13-15" for the
    // mutation that confirms the angle-invariance test below actually
    // detects a direction-dependent regression.

    #[test]
    fn search_intersection_points_indices_are_invariant_to_the_corners_angle() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: true,
        };
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        limits.set_cartesian_limits(blend_geometry_cartesian_limits());

        // Segment 1: identical to doc/oracle-request-pilz-blend.md's case
        // A/B (ready pose, +0.1m/+x).
        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let seg1 = gen_lin_segment(&model, &limits, &ctx, &ready_positions(), corner, 0.1, 0.1);
        let group = model.joint_model_group("panda_arm").unwrap();
        let boundary = seg1.last_way_point().unwrap();
        let mut chained = HashMap::new();
        for name in group.active_joint_names() {
            chained.insert(name.clone(), boundary.variable_position(name).unwrap());
        }

        // Baseline: case A's own known indices (8, 7), corner at +0.1m/+y
        // (a 90 degree turn from segment 1's +x direction).
        let mut baseline_req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: seg1.clone(),
            second_trajectory: gen_lin_segment(
                &model,
                &limits,
                &ctx,
                &chained,
                [corner[0], 0.1, corner[2]],
                0.1,
                0.1,
            ),
            blend_radius: 0.05,
        };
        let baseline = search_intersection_points(&scene, &mut baseline_req).unwrap();
        assert_eq!(
            baseline,
            (8, 7),
            "case A's own already-oracle-verified indices"
        );

        // Same blend_radius, same segment-2 travel distance and speed, only
        // the corner's included angle changes (45/120/150 degrees from
        // segment 1's +x direction, vs case A's 90 degrees). Per
        // doc/oracle-request-pilz-blend-geometry.md's case D argument:
        // first_intersection_index depends only on first_trajectory's own
        // waypoints' distance to circ_pose (:387-393, computed from
        // first_trajectory's own last waypoint before segment 2 exists at
        // all); second_intersection_index depends only on
        // second_trajectory's own waypoints' distance to that same fixed
        // point (:404-411). Neither ever reads the other trajectory's
        // direction, so a pure angle change with distance/speed held fixed
        // must not move either index.
        for angle_deg in [45.0_f64, 120.0, 150.0] {
            let goal = [
                corner[0] + 0.1 * angle_deg.to_radians().cos(),
                0.1 * angle_deg.to_radians().sin(),
                corner[2],
            ];
            let mut req = TrajectoryBlendRequest {
                group_name: "panda_arm".to_string(),
                link_name: "panda_link8".to_string(),
                first_trajectory: seg1.clone(),
                second_trajectory: gen_lin_segment(&model, &limits, &ctx, &chained, goal, 0.1, 0.1),
                blend_radius: 0.05,
            };
            let indices = search_intersection_points(&scene, &mut req).unwrap();
            assert_eq!(
                indices, baseline,
                "corner angle {angle_deg} degrees must not move the indices \
                 (radius and speed held fixed at case A's values)"
            );
        }
    }

    #[test]
    fn search_intersection_points_radius_sweep_moves_the_indices_but_not_the_branch() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: true,
        };
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        limits.set_cartesian_limits(blend_geometry_cartesian_limits());

        // Case A's exact corner and (symmetric) speed -- only blend_radius
        // varies below, per doc/oracle-request-pilz-blend-geometry.md's case
        // C.
        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let seg1 = gen_lin_segment(&model, &limits, &ctx, &ready_positions(), corner, 0.1, 0.1);
        let group = model.joint_model_group("panda_arm").unwrap();
        let boundary = seg1.last_way_point().unwrap();
        let mut chained = HashMap::new();
        for name in group.active_joint_names() {
            chained.insert(name.clone(), boundary.variable_position(name).unwrap());
        }
        let seg2 = gen_lin_segment(
            &model,
            &limits,
            &ctx,
            &chained,
            [corner[0], 0.1, corner[2]],
            0.1,
            0.1,
        );

        // Measured locally (probe written, run, reverted) against this exact
        // fixture. Symmetric-speed segments have identical waypoint density
        // by construction, so the alignment branch stays `else`
        // (way_point_count_1 == way_point_count_2) at every radius here --
        // case B already covers the other branch -- but the index values
        // themselves move across a real range, exercising
        // linear_search_intersection_point's walk arithmetic at values other
        // than case A/B's pinned 8/7.
        for (radius, expected) in [
            (0.02, (11, 4)),
            (0.03, (10, 5)),
            (0.08, (5, 10)),
            (0.1, (1, 14)),
        ] {
            let mut req = TrajectoryBlendRequest {
                group_name: "panda_arm".to_string(),
                link_name: "panda_link8".to_string(),
                first_trajectory: seg1.clone(),
                second_trajectory: seg2.clone(),
                blend_radius: radius,
            };
            let indices = search_intersection_points(&scene, &mut req).unwrap();
            assert_eq!(indices, expected, "blend_radius {radius}");

            let (first_index, second_index) = indices;
            let way_point_count_1 = req.first_trajectory.way_point_count() - first_index;
            let way_point_count_2 = second_index + 1;
            assert_eq!(
                way_point_count_1, way_point_count_2,
                "blend_radius {radius} must stay in the else branch on this symmetric geometry"
            );
        }
    }

    // Unlike the two tests above, this one does not discriminate between
    // search_intersection_points's two `Error::Code(InvalidMotionPlan)`
    // sites (the first_trajectory search's ok_or at :402 vs the
    // second_trajectory search's at :411) -- verified by isolating
    // mutation: neutralizing either ok_or alone still leaves this test
    // green, because at this radius/corner neither trajectory has enough
    // length to cross, so the other search's guard still fires. Only
    // neutralizing both makes it fail. This is a genuine joint failure,
    // not a masked single branch: the function's own doc comment records
    // that both searches deliberately share one error code, matching
    // upstream (`searchIntersectionPoints` itself returns only a `bool`),
    // and `blend` (this module's only caller, :234) never distinguishes
    // the two causes either. The assertion below verifies overall
    // rejection at this pinned oracle geometry, not which branch fired.
    #[test]
    fn search_intersection_points_rejects_a_radius_that_exceeds_this_corners_reach() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: true,
        };
        let mut limits = LimitsContainer::new();
        limits.set_joint_limits(panda_joint_limits());
        limits.set_cartesian_limits(blend_geometry_cartesian_limits());

        // Same corner/speed as the radius sweep above. 0.12 is the smallest
        // value found (locally, by 0.01 steps from the sweep's largest
        // accepted 0.1) at which neither trajectory has enough length left
        // to cross -- pinning where this specific corner's blend_radius
        // stops fitting, not an arbitrary large number.
        let corner = [
            0.40701957005161055,
            -5.221329615610066e-12,
            0.5902695582766445,
        ];
        let seg1 = gen_lin_segment(&model, &limits, &ctx, &ready_positions(), corner, 0.1, 0.1);
        let group = model.joint_model_group("panda_arm").unwrap();
        let boundary = seg1.last_way_point().unwrap();
        let mut chained = HashMap::new();
        for name in group.active_joint_names() {
            chained.insert(name.clone(), boundary.variable_position(name).unwrap());
        }
        let seg2 = gen_lin_segment(
            &model,
            &limits,
            &ctx,
            &chained,
            [corner[0], 0.1, corner[2]],
            0.1,
            0.1,
        );

        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: seg1,
            second_trajectory: seg2,
            blend_radius: 0.12,
        };
        assert!(matches!(
            search_intersection_points(&scene, &mut req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    // -- blend_trajectory_cartesian: the quintic smoothstep's own boundary
    // values, alpha(s=1/n) close to 0 and alpha(s=1) exactly 1 --

    #[test]
    fn blend_trajectory_cartesian_first_sample_stays_near_pose1_last_sample_reaches_pose2() {
        let (model, srdf) = load_panda();
        let scene = PlanningScene::new(&model, &srdf);
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let (first_index, second_index) = search_intersection_points(&scene, &mut req).unwrap();
        let blend_align_index = determine_trajectory_alignment(&req, first_index, second_index);

        let pose1 = req
            .first_trajectory
            .way_point_mut(first_index)
            .unwrap()
            .update()
            .frame_transform("panda_link8")
            .unwrap();
        // The pose the loop's last iteration actually blends toward: with
        // second_index > 0 (asserted below), the `(first_interse_index + i) >
        // blend_align_index` branch fires on the final sample and rebinds
        // `blend_sample_pose2` to `second_trajectory`'s waypoint at
        // `second_index` -- not its first waypoint. See this test's own
        // assertion below and upstream `blendTrajectoryCartesian`'s update
        // branch.
        assert!(second_index > 0, "test assumes a nonzero second_index");
        let pose2_at_end = req
            .second_trajectory
            .way_point_mut(second_index)
            .unwrap()
            .update()
            .frame_transform("panda_link8")
            .unwrap();
        let pose2_at_start = req
            .second_trajectory
            .way_point_mut(0)
            .unwrap()
            .update()
            .frame_transform("panda_link8")
            .unwrap();

        let cartesian = blend_trajectory_cartesian(
            &scene,
            &mut req,
            first_index,
            second_index,
            blend_align_index,
            0.05,
        );

        assert!(!cartesian.points.is_empty());
        let first_point = cartesian.points.first().unwrap();
        let last_point = cartesian.points.last().unwrap();

        // s = 1/n on the first sample -> alpha is small but not zero; the
        // sample must sit strictly between pose1 and the loop's initial
        // pose2 (second_trajectory's first waypoint, still in effect this
        // early), closer to pose1.
        let d_first_to_pose1 =
            (first_point.pose.translation.vector - pose1.translation.vector).norm();
        let d_first_to_pose2 =
            (first_point.pose.translation.vector - pose2_at_start.translation.vector).norm();
        assert!(d_first_to_pose1 < d_first_to_pose2);

        // s = 1 on the last sample -> alpha == 1 exactly (6-15+10 == 1), so
        // the last sample must equal the loop's final pose2 (second_index's
        // waypoint) closely.
        assert_relative_eq!(
            last_point.pose.translation.vector,
            pose2_at_end.translation.vector,
            epsilon = 1e-9
        );

        // time_from_start is strictly increasing and starts at one sampling
        // interval, not zero.
        assert!(first_point.time_from_start > 0.0);
        for pair in cartesian.points.windows(2) {
            assert!(pair[1].time_from_start > pair[0].time_from_start);
        }
    }

    // -- blend: end-to-end, using two real kinematically-consistent
    // trajectories sharing a boundary --

    #[test]
    fn blend_produces_a_continuous_trajectory_through_the_shared_boundary() {
        let (model, srdf) = load_panda();
        let scene = Arc::new(PlanningScene::new(&model, &srdf));
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };

        let mut planner_limits = LimitsContainer::new();
        planner_limits.set_joint_limits(panda_joint_limits());
        planner_limits.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.0,
            max_trans_dec: -2.0,
            max_rot_vel: 1.0,
        });

        // Ground truth for the response segment lengths: an independent
        // search_intersection_points call over the same geometry, so the
        // assertions below can check the copy loops in `blend` produce
        // exactly `first_intersection_index` and
        // `second_count - (second_intersection_index + 1)` waypoints, not
        // just "no more than the original" -- a bound loose enough to miss
        // an off-by-one in either loop, which mutation testing confirmed by
        // extending `blend`'s first-segment copy loop by one waypoint
        // without failing this test.
        let mut probe_req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let (first_intersection_index, second_intersection_index) =
            search_intersection_points(&scene, &mut probe_req).expect("same geometry as req below");

        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let second_count = req.second_trajectory.way_point_count();

        let response =
            blend(&ctx, &planner_limits, &mut req).expect("well-formed request must blend");

        assert!(!response.blend_trajectory.is_empty());
        assert_eq!(
            response.first_trajectory.way_point_count(),
            first_intersection_index
        );
        assert_eq!(
            response.second_trajectory.way_point_count(),
            second_count - (second_intersection_index + 1)
        );
    }

    /// Upstream `validateRequest` accepts `req.link_name` if it names either
    /// a robot model link or an attached body on `first_trajectory`'s last
    /// waypoint (`hasAttachedBody`, `trajectory_blender_transition_window.cpp:160-161`).
    /// `"grasped_box"` here is neither `panda_arm`'s solver tip nor any
    /// other model link -- only [`PlanningScene::has_attached_body`] can
    /// accept it, matching this module's (now-closed) `# Deviations` entry.
    ///
    /// `"grasped_box"` is attached to `"panda_link8"` with an identity
    /// local pose (matching every other attached body in this crate: see
    /// `cspace_scene::AttachedBody`'s module doc on why the local pose is
    /// always identity here), so its world pose equals `"panda_link8"`'s at
    /// every waypoint exactly. The control request below blends the same
    /// two trajectories by `"panda_link8"` directly and asserts the two
    /// responses match waypoint-for-waypoint -- proving the attached-body
    /// path resolves to the *same* geometry its underlying link would, not
    /// merely that it fails to error.
    #[test]
    fn blend_reaches_a_link_name_naming_an_attached_body() {
        let (model, srdf) = load_panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        scene
            .attach_new(
                "grasped_box",
                "panda_link8",
                vec![Arc::new(Shape::Cuboid(
                    Cuboid::new(0.02, 0.02, 0.02).expect("extents are non-negative"),
                ))],
                vec![Isometry3::identity()],
                BTreeSet::new(),
                BTreeMap::new(),
            )
            .expect("panda_link8 is a real link");
        let scene = Arc::new(scene);
        let env =
            ParryCollisionEnv::new(cspace_collision::World::new(), LinkPaddingScale::default());
        let ctx = IkContext {
            scene: &scene,
            env: &env,
            check_self_collision: false,
        };

        let mut planner_limits = LimitsContainer::new();
        planner_limits.set_joint_limits(panda_joint_limits());
        planner_limits.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.0,
            max_trans_dec: -2.0,
            max_rot_vel: 1.0,
        });

        let mut attached_req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "grasped_box".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let attached_response = blend(&ctx, &planner_limits, &mut attached_req).expect(
            "upstream accepts link_name naming an attached body (validateRequest's \
             hasAttachedBody fallback)",
        );

        let mut link_req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let link_response = blend(&ctx, &planner_limits, &mut link_req)
            .expect("the control request, naming the attached body's own link, must also blend");

        assert_eq!(
            attached_response.blend_trajectory.way_point_count(),
            link_response.blend_trajectory.way_point_count(),
            "an identity-offset attached body must blend identically to its own link"
        );
        for i in 0..attached_response.blend_trajectory.way_point_count() {
            let attached_point = attached_response.blend_trajectory.way_point(i).unwrap();
            let link_point = link_response.blend_trajectory.way_point(i).unwrap();
            for name in model
                .joint_model_group("panda_arm")
                .unwrap()
                .active_joint_names()
            {
                assert_relative_eq!(
                    attached_point.variable_position(name).unwrap(),
                    link_point.variable_position(name).unwrap(),
                    epsilon = 1e-9
                );
            }
        }
    }
}
