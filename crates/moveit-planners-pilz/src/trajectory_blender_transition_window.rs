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
//!   That field is already [`moveit_geometry::Isometry3`] here (see
//!   [`crate::cartesian_trajectory`]'s own `# Deviations`), so the
//!   conversion has nothing left to do — dropped as a message-shape
//!   exclusion (`PORTING-PLAN.md` D1/D2), not a ROS dependency this port
//!   had to work around.
//! - `moveit_msgs::msg::MoveItErrorCodes` (in `TrajectoryBlendResponse`
//!   and `validateRequest`'s out-parameter) is replaced by
//!   [`moveit_error::MoveItErrorCode`], via [`blend`]'s `Result` — see
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
//! - **`validateRequest`'s link-name check drops the attached-body
//!   fallback.** Upstream accepts `link_name` if it names either a robot
//!   model link or an attached body on `first_trajectory`'s last waypoint
//!   (`hasAttachedBody`). This port keeps attached bodies on
//!   [`moveit_scene::PlanningScene`], not on [`moveit_state::RobotState`]
//!   (see `moveit-scene`'s `attached_body` module doc) — a bare
//!   [`RobotTrajectory`] here carries no scene to check against, matching
//!   [`moveit_state::Posed::frame_transform`]'s own documented
//!   inability to see attached bodies from a state alone. A link name that
//!   only resolves via an attached body is rejected here where upstream
//!   would accept it.
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

use moveit_collision::CollisionEnv;
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::Isometry3;
use moveit_kinematics::{DEFAULT_SOLVER_NAME, SolverParams, resolve_solver};
use moveit_state::Posed;
use moveit_trajectory::RobotTrajectory;

use crate::cartesian_trajectory::{CartesianTrajectory, CartesianTrajectoryPoint, Twist};
use crate::limits::LimitsContainer;
use crate::trajectory_functions::{
    IkContext, determine_and_check_sampling_time, generate_joint_trajectory_from_cartesian,
    is_robot_state_equal, is_robot_state_stationary, linear_search_intersection_point,
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
/// [`moveit_kinematics::KINEMATICS_SOLVERS`] entry can be built for
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
    let sampling_time = validate_request(req)?;

    let (first_intersection_index, second_intersection_index) = search_intersection_points(req)?;

    let blend_align_index =
        determine_trajectory_alignment(req, first_intersection_index, second_intersection_index);

    let blend_trajectory_cartesian = blend_trajectory_cartesian(
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
        .ok()
        .filter(|solver| solver.tip_frame() == req.link_name)
        .ok_or(Error::Code(MoveItErrorCode::NoIkSolution))?;

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
/// both trajectories on success. Upstream `validateRequest`. See this
/// module's `# Deviations` for the dropped attached-body fallback.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidGroupName`] if `req.group_name` names no group.
/// [`MoveItErrorCode::InvalidLinkName`] if `req.link_name` names no link.
/// [`MoveItErrorCode::InvalidMotionPlan`] if `req.blend_radius` is not
/// positive, the trajectories' shared boundary state does not match within
/// `EPSILON` (see [`is_robot_state_equal`]), no consistent sampling time
/// can be determined (see [`determine_and_check_sampling_time`]), or that
/// boundary state has nonzero velocity/acceleration (see
/// [`is_robot_state_stationary`]).
fn validate_request(req: &TrajectoryBlendRequest<'_>) -> Result<f64> {
    let robot_model = req.first_trajectory.robot_model();

    if !robot_model.has_joint_model_group(&req.group_name) {
        return Err(Error::Code(MoveItErrorCode::InvalidGroupName));
    }

    if !robot_model.has_link_model(&req.link_name) {
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
fn search_intersection_points<'m>(req: &mut TrajectoryBlendRequest<'m>) -> Result<(usize, usize)> {
    let circ_pose = req
        .first_trajectory
        .last_way_point_mut()
        .expect("validate_request already confirmed first_trajectory is non-empty")
        .update()
        .frame_transform(&req.link_name)
        .expect("validate_request already confirmed link_name resolves");

    let first_index = linear_search_intersection_point(
        &req.link_name,
        &circ_pose.translation.vector,
        req.blend_radius,
        &mut req.first_trajectory,
        true,
    )
    .ok_or(Error::Code(MoveItErrorCode::InvalidMotionPlan))?;

    let second_index = linear_search_intersection_point(
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
    req: &mut TrajectoryBlendRequest<'m>,
    first_intersection_index: usize,
    second_intersection_index: usize,
    blend_align_index: usize,
    sampling_time: f64,
) -> CartesianTrajectory {
    let frame_transform_at =
        |traj: &mut RobotTrajectory<'m>, index: usize, link_name: &str| -> Isometry3 {
            traj.way_point_mut(index)
                .expect("index within way_point_count")
                .update()
                .frame_transform(link_name)
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
        let rotation = blend_sample_pose1
            .rotation
            .slerp(&blend_sample_pose2.rotation, alpha);

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
    use std::fs;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use moveit_collision::{LinkPaddingScale, ParryCollisionEnv};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_scene::PlanningScene;
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;
    use crate::limits::{CartesianLimits, JointLimit, JointLimitsContainer};

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
        let (model, _) = load_panda();
        let traj = panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1);
        let req = TrajectoryBlendRequest {
            group_name: "no_such_group".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: traj,
            blend_radius: 0.05,
        };
        match validate_request(&req) {
            Err(Error::Code(MoveItErrorCode::InvalidGroupName)) => {}
            other => panic!("expected InvalidGroupName, got {other:?}"),
        }
    }

    #[test]
    fn validate_request_rejects_an_unknown_link_name() {
        let (model, _) = load_panda();
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "no_such_link".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            blend_radius: 0.05,
        };
        match validate_request(&req) {
            Err(Error::Code(MoveItErrorCode::InvalidLinkName)) => {}
            other => panic!("expected InvalidLinkName, got {other:?}"),
        }
    }

    #[test]
    fn validate_request_rejects_blend_radius_at_or_below_zero() {
        let (model, _) = load_panda();
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            blend_radius: 0.0,
        };
        assert!(matches!(
            validate_request(&req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
        req.blend_radius = -0.01;
        assert!(matches!(
            validate_request(&req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_rejects_a_boundary_state_mismatch() {
        let (model, _) = load_panda();
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
            validate_request(&req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_rejects_a_mismatched_sampling_time() {
        let (model, _) = load_panda();
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.05),
            blend_radius: 0.05,
        };
        assert!(matches!(
            validate_request(&req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    #[test]
    fn validate_request_accepts_a_well_formed_request() {
        let (model, _) = load_panda();
        let req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, 0.0, 0.2, 4, 0.1),
            second_trajectory: panda_joint1_sweep(&model, 0.2, 0.4, 4, 0.1),
            blend_radius: 0.05,
        };
        assert_relative_eq!(validate_request(&req).unwrap(), 0.1);
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
        let (model, _) = load_panda();
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let (first_index, second_index) = search_intersection_points(&mut req).unwrap();
        // The center is first_trajectory's own last waypoint, so the first
        // trajectory's crossing must be strictly before its own end.
        assert!(first_index < req.first_trajectory.way_point_count() - 1);
        assert!(second_index < req.second_trajectory.way_point_count());
    }

    #[test]
    fn search_intersection_points_rejects_a_radius_larger_than_either_trajectory_reaches() {
        let (model, _) = load_panda();
        // A tiny joint sweep keeps panda_link8 within a small Cartesian
        // radius of the boundary pose -- a blend_radius far larger than any
        // sample's distance from the center is never crossed.
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.005, 0.0, 10, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.005, 10, 0.05),
            blend_radius: 10.0,
        };
        assert!(matches!(
            search_intersection_points(&mut req),
            Err(Error::Code(MoveItErrorCode::InvalidMotionPlan))
        ));
    }

    // -- blend_trajectory_cartesian: the quintic smoothstep's own boundary
    // values, alpha(s=1/n) close to 0 and alpha(s=1) exactly 1 --

    #[test]
    fn blend_trajectory_cartesian_first_sample_stays_near_pose1_last_sample_reaches_pose2() {
        let (model, _) = load_panda();
        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };
        let (first_index, second_index) = search_intersection_points(&mut req).unwrap();
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
            ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
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

        let mut req = TrajectoryBlendRequest {
            group_name: "panda_arm".to_string(),
            link_name: "panda_link8".to_string(),
            first_trajectory: panda_joint1_sweep(&model, -0.3, 0.0, 20, 0.05),
            second_trajectory: panda_joint1_sweep(&model, 0.0, 0.3, 20, 0.05),
            blend_radius: 0.05,
        };

        let response =
            blend(&ctx, &planner_limits, &mut req).expect("well-formed request must blend");

        assert!(!response.blend_trajectory.is_empty());
        // The three segments together must cover strictly less than the two
        // original trajectories' combined waypoint count (some waypoints
        // near the boundary are replaced by the blend), but each of the
        // three non-blend segments is non-empty-or-absent consistently with
        // where the crossing indices landed.
        assert!(
            response.first_trajectory.way_point_count() <= req.first_trajectory.way_point_count()
        );
        assert!(
            response.second_trajectory.way_point_count() <= req.second_trajectory.way_point_count()
        );
    }
}
