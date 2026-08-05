// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! End-to-end tests for
//! [`moveit_planners_pilz::trajectory_generator_polyline::TrajectoryGeneratorPolyline`]
//! on `panda_arm`.
//!
//! # This is not the parity test
//!
//! Parity against the moveit2 C++ oracle lives in
//! `pilz_trajectory_polyline_parity.rs`, on the same corner geometry and the
//! same `panda_lin_request.json` numbers. This file is what it cannot be:
//! the oracle compares *joint values at 34 sampled instants*, which pins the
//! two implementations to each other but says nothing about the path either
//! of them is on. What follows asserts *properties of the produced
//! trajectory that only a correct rounded-polyline path can satisfy* --
//! that the tip tracks the very path
//! [`polyline_from_waypoints`](moveit_planners_pilz::path_polyline_generator::polyline_from_waypoints)
//! builds, and that it rounds the corner rather than driving through it (the
//! test a `LIN`-per-segment or a straight-to-goal implementation fails).
//! Both would still pass a joint-value comparison against an oracle that
//! made the same mistake.
//!
//! The request's limits, `sampling_time`, scaling factors and start state
//! are reused verbatim from `panda_lin_request.json` so this file introduces
//! no second, independently drifting set of panda numbers.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use moveit_error::MoveItErrorCode;
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_pilz::limits::{
    CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer,
};
use moveit_planners_pilz::path_polyline_generator::polyline_from_waypoints;
use moveit_planners_pilz::trajectory_functions::{IkContext, compute_link_fk};
use moveit_planners_pilz::trajectory_generator::{
    CircPathConstraint, CircPathConstraintKind, Goal, MotionPlanRequest, PathConstraints,
    PilzGenerator, PolylinePathConstraint, StartState, TrajectoryGenerator,
};
use moveit_planners_pilz::trajectory_generator_polyline::TrajectoryGeneratorPolyline;
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

#[derive(Deserialize)]
struct FixtureJointLimit {
    #[serde(default)]
    has_position_limits: bool,
    #[serde(default)]
    min_position: f64,
    #[serde(default)]
    max_position: f64,
    #[serde(default)]
    has_velocity_limits: bool,
    #[serde(default)]
    max_velocity: f64,
    #[serde(default)]
    has_acceleration_limits: bool,
    #[serde(default)]
    max_acceleration: f64,
    #[serde(default)]
    has_deceleration_limits: bool,
    #[serde(default)]
    max_deceleration: f64,
}

impl From<&FixtureJointLimit> for JointLimit {
    fn from(f: &FixtureJointLimit) -> Self {
        JointLimit {
            has_position_limits: f.has_position_limits,
            min_position: f.min_position,
            max_position: f.max_position,
            has_velocity_limits: f.has_velocity_limits,
            max_velocity: f.max_velocity,
            has_acceleration_limits: f.has_acceleration_limits,
            max_acceleration: f.max_acceleration,
            has_deceleration_limits: f.has_deceleration_limits,
            max_deceleration: f.max_deceleration,
            ..Default::default()
        }
    }
}

#[derive(Deserialize)]
struct FixtureCartesianLimits {
    max_trans_vel: f64,
    max_trans_acc: f64,
    max_trans_dec: f64,
    max_rot_vel: f64,
}

impl From<&FixtureCartesianLimits> for CartesianLimits {
    fn from(f: &FixtureCartesianLimits) -> Self {
        CartesianLimits {
            max_trans_vel: f.max_trans_vel,
            max_trans_acc: f.max_trans_acc,
            max_trans_dec: f.max_trans_dec,
            max_rot_vel: f.max_rot_vel,
        }
    }
}

#[derive(Deserialize)]
struct GoalFixture {
    link_name: String,
}

#[derive(Deserialize)]
struct RequestFixture {
    group_name: String,
    sampling_time: f64,
    joint_limits: HashMap<String, FixtureJointLimit>,
    cartesian_limits: FixtureCartesianLimits,
    start_state: HashMap<String, f64>,
    max_velocity_scaling_factor: f64,
    max_acceleration_scaling_factor: f64,
    goal: GoalFixture,
}

fn load_json<T: serde::de::DeserializeOwned>(file_name: &str) -> T {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

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

/// See `pilz_trajectory_lin_parity.rs`'s own doc for why this fixture's pose
/// makes the value inconsequential -- the same start state is reused here,
/// and every waypoint below is a small translation from it in free space.
const CHECK_SELF_COLLISION: bool = true;

/// The corner leg length. `0.15 m` is far above
/// `path_polyline_generator::MIN_SEGMENT_LENGTH` (`0.2 mm`, so no waypoint is
/// filtered) and still inside `panda_arm`'s reach from the SRDF `"ready"`
/// pose, so every sample has an IK solution.
const LEG: f64 = 0.15;

/// `smoothness_level` for the corner. `0.5` sits in the middle of
/// `path_polyline_generator`'s accepted `(MIN_SMOOTHNESS, MAX_SMOOTHNESS)`
/// band, and at a right-angle corner with equal `LEG`-long legs it puts the
/// blend radius at half of the largest radius the corner admits -- large
/// enough that the rounding is unambiguous against IK noise, small enough
/// that the arc stays well inside both legs.
const SMOOTHNESS: f64 = 0.5;

/// How far a trajectory sample's tip may sit from the reference path. Every
/// sample is an independent IK solve followed by FK, so this budget covers
/// the same solver-convergence spread `pilz_trajectory_lin_parity.rs`
/// measures in joint space (`1.26e-5` there, carried through the Jacobian).
/// Measured maximum over this test's own 34-sample trajectory: `9.35e-6 m`;
/// set with roughly a 10x margin. It is *not* sized to hide the rounding
/// itself, which is `1.55e-2 m` at this corner -- three orders larger, and
/// asserted separately by
/// [`polyline_panda_arm_rounds_the_corner_instead_of_reaching_the_vertex`].
const PATH_TRACKING_TOLERANCE: f64 = 1e-4;

/// Arc-length step of the reference-path sampling that
/// [`worst_path_deviation`] searches for a nearest point. The bound that
/// matters is *along* the path, not across it: a tip sitting exactly between
/// two grid points reports up to `step / 2` of spurious deviation, so the
/// grid -- not IK -- sets the floor unless the step is far below the
/// tolerance. At `1e-4` it was: the measured worst deviation came out
/// `4.02e-5`, i.e. `step / 2`, and the number said nothing about the port.
/// At `1e-6` the grid contributes at most `5e-7`, and the same measurement
/// drops to `9.35e-6` -- which is then really the IK spread
/// [`PATH_TRACKING_TOLERANCE`] is sized from.
const REFERENCE_SAMPLE_STEP: f64 = 1e-6;

/// The panda `"ready"` tip pose, and the two waypoints of an L-shaped corner
/// from it: `+LEG` along `x`, then `+LEG` along `y`, orientation held.
fn corner_waypoints(
    model: &RobotModel,
    link_name: &str,
    start_state: &HashMap<String, f64>,
) -> (Isometry3, Vec<Isometry3>) {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    let start_pose = compute_link_fk(&mut state, link_name, start_state)
        .expect("the fixture's own link must resolve on the fixture's own model");

    let mut via = start_pose;
    via.translation.vector += Vector3::new(LEG, 0.0, 0.0);
    let mut end = start_pose;
    end.translation.vector += Vector3::new(LEG, LEG, 0.0);
    (start_pose, vec![via, end])
}

fn panda_polyline_request(
    request: &RequestFixture,
    waypoints: Vec<Isometry3>,
    goal_pose: &Isometry3,
) -> MotionPlanRequest {
    MotionPlanRequest {
        group_name: request.group_name.clone(),
        start_state: StartState {
            position: request.start_state.clone(),
            velocity: HashMap::new(),
        },
        goal: Goal::Cartesian {
            link_name: request.goal.link_name.clone(),
            position: goal_pose.translation.vector,
            orientation: UnitQuaternion::from_rotation_matrix(
                &goal_pose.rotation.to_rotation_matrix(),
            ),
            target_point_offset: Vector3::new(0.0, 0.0, 0.0),
        },
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: Some(PathConstraints::Polyline(PolylinePathConstraint {
            waypoints,
            smoothness_level: SMOOTHNESS,
        })),
    }
}

fn panda_limits(request: &RequestFixture) -> LimitsContainer {
    let mut joint_limits = JointLimitsContainer::default();
    for (name, limit) in &request.joint_limits {
        assert!(
            joint_limits.add_limit(name.clone(), limit.into()),
            "duplicate or invalid joint limit for {name} in fixture"
        );
    }
    let mut limits = LimitsContainer::new();
    limits.set_joint_limits(joint_limits);
    limits.set_cartesian_limits((&request.cartesian_limits).into());
    limits
}

/// The largest distance from any sample of the produced trajectory's tip to
/// the nearest point of `reference`.
fn worst_path_deviation(tip_positions: &[Vector3], reference: &[Vector3]) -> (f64, usize) {
    let mut worst = 0.0;
    let mut worst_index = 0;
    for (i, tip) in tip_positions.iter().enumerate() {
        let nearest = reference
            .iter()
            .map(|p| (p - tip).norm())
            .fold(f64::INFINITY, f64::min);
        if nearest > worst {
            worst = nearest;
            worst_index = i;
        }
    }
    (worst, worst_index)
}

/// The smallest distance from any sample of the produced trajectory's tip to
/// the sharp corner vertex `via`.
fn corner_clearance(tip_positions: &[Vector3], via: &Vector3) -> f64 {
    tip_positions
        .iter()
        .map(|p| (p - via).norm())
        .fold(f64::INFINITY, f64::min)
}

struct Planned {
    tip_positions: Vec<Vector3>,
    times: Vec<f64>,
}

fn plan_corner(request: &RequestFixture) -> (Planned, Isometry3, Vec<Isometry3>) {
    let (model, srdf) = load_panda();
    let (start_pose, waypoints) =
        corner_waypoints(&model, &request.goal.link_name, &request.start_state);

    let base = TrajectoryGenerator::new(&model, panda_limits(request));
    let generator = TrajectoryGeneratorPolyline::new(base, &request.group_name);
    let plan_request =
        panda_polyline_request(request, waypoints.clone(), waypoints.last().unwrap());

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let response = generator.generate(&ctx, &plan_request, request.sampling_time);
    let trajectory = response.trajectory.unwrap_or_else(|| {
        panic!(
            "an L corner in free space from the fixture's own start state must plan, got {:?}",
            response.error_code
        )
    });

    let mut tip_positions = Vec::with_capacity(trajectory.way_point_count());
    let mut times = Vec::with_capacity(trajectory.way_point_count());
    for i in 0..trajectory.way_point_count() {
        let state = trajectory.way_point(i).unwrap();
        let pose = state
            .clone()
            .update()
            .frame_transform(&request.goal.link_name)
            .expect("the fixture's own link must resolve on every produced waypoint");
        tip_positions.push(pose.translation.vector);
        times.push(trajectory.way_point_duration_from_start(i));
    }

    (
        Planned {
            tip_positions,
            times,
        },
        start_pose,
        waypoints,
    )
}

/// The produced trajectory's tip must stay on the very
/// [`PathRoundedComposite`](moveit_planners_pilz::path_rounded_composite::PathRoundedComposite)
/// that `polyline_from_waypoints` builds from the same request -- start pose,
/// waypoints, smoothness and `eqradius` all identical to what
/// `TrajectoryGeneratorPolyline::plan` passes.
#[test]
fn polyline_panda_arm_tip_tracks_the_rounded_path() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let (planned, start_pose, waypoints) = plan_corner(&request);

    let path = polyline_from_waypoints(
        &start_pose,
        &waypoints,
        SMOOTHNESS,
        request.cartesian_limits.max_trans_vel / request.cartesian_limits.max_rot_vel,
    )
    .expect("the same inputs the generator itself used must build a path");

    let mut reference = Vec::new();
    let mut s = 0.0;
    while s < path.path_length() {
        reference.push(path.pos(s).translation.vector);
        s += REFERENCE_SAMPLE_STEP;
    }
    reference.push(path.pos(path.path_length()).translation.vector);

    let (worst, worst_index) = worst_path_deviation(&planned.tip_positions, &reference);
    assert!(
        worst < PATH_TRACKING_TOLERANCE,
        "waypoint {worst_index} of {} sits {worst} m off the rounded path \
         (tolerance {PATH_TRACKING_TOLERANCE})",
        planned.tip_positions.len()
    );
}

/// The discriminating assertion: a `POLYLINE` motion must *round* the corner,
/// so its tip never reaches the sharp vertex. A straight-to-goal `LIN`, or a
/// `LIN` per segment, drives through it -- both would pass the tracking test
/// above only if the reference path were also wrong, and neither passes this
/// one.
#[test]
fn polyline_panda_arm_rounds_the_corner_instead_of_reaching_the_vertex() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let (planned, start_pose, waypoints) = plan_corner(&request);

    let path = polyline_from_waypoints(
        &start_pose,
        &waypoints,
        SMOOTHNESS,
        request.cartesian_limits.max_trans_vel / request.cartesian_limits.max_rot_vel,
    )
    .expect("the same inputs the generator itself used must build a path");

    // The rounded path's own shortest distance to the vertex, i.e. what the
    // blend radius geometrically implies. The trajectory must clear the
    // vertex by that much, minus the per-sample IK spread the sampling can
    // add -- not by some tolerance chosen here independently of the radius.
    let mut reference_clearance = f64::INFINITY;
    let mut s = 0.0;
    while s < path.path_length() {
        let d = (path.pos(s).translation.vector - waypoints[0].translation.vector).norm();
        reference_clearance = reference_clearance.min(d);
        s += REFERENCE_SAMPLE_STEP;
    }

    // Measured at this corner: `1.553e-2 m`. The floor below is an order
    // down from that -- it is a vacuity guard, not a tolerance.
    assert!(
        reference_clearance > 1e-3,
        "this test is vacuous unless the reference path itself clears the \
         vertex; got {reference_clearance} m -- the corner is not being rounded \
         at all"
    );

    let clearance = corner_clearance(&planned.tip_positions, &waypoints[0].translation.vector);
    assert!(
        clearance > reference_clearance - PATH_TRACKING_TOLERANCE,
        "trajectory passes {clearance} m from the sharp vertex, closer than \
         the rounded path's own {reference_clearance} m -- the corner is being \
         cut through, not rounded"
    );
}

/// `time_from_start` comes from one `VelocityProfileTrap` spanning the whole
/// composite (this port's, and upstream's, single-profile deviation from a
/// per-segment retiming), so it must be strictly increasing across every
/// segment boundary, not merely per segment.
#[test]
fn polyline_panda_arm_times_increase_across_the_segment_boundaries() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let (planned, _, _) = plan_corner(&request);

    assert!(
        planned.times.len() > 2,
        "a {LEG} m two-leg corner at sampling_time {} must produce more than \
         two samples, got {}",
        request.sampling_time,
        planned.times.len()
    );
    assert_eq!(planned.times[0], 0.0);
    for i in 1..planned.times.len() {
        assert!(
            planned.times[i] > planned.times[i - 1],
            "time_from_start[{i}] = {} is not after [{}] = {}",
            planned.times[i],
            i - 1,
            planned.times[i - 1]
        );
    }
}

/// Upstream `cmdSpecificRequestValidation`'s `NoWaypointsSpecified`.
#[test]
fn polyline_rejects_a_request_with_fewer_than_two_waypoints() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let (model, srdf) = load_panda();
    let (_, waypoints) = corner_waypoints(&model, &request.goal.link_name, &request.start_state);

    let base = TrajectoryGenerator::new(&model, panda_limits(&request));
    let generator = TrajectoryGeneratorPolyline::new(base, &request.group_name);
    let one = vec![waypoints[0]];
    let plan_request = panda_polyline_request(&request, one, &waypoints[0]);

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let response = generator.generate(&ctx, &plan_request, request.sampling_time);
    assert_eq!(response.error_code, MoveItErrorCode::InvalidMotionPlan);
    assert!(response.trajectory.is_none());
}

/// The `PathConstraints` enum split's own regression: swapping *only* the
/// constraint variant, on a request that is otherwise byte-identical to one
/// that plans, must turn `SUCCESS` into a rejection.
///
/// # What this test can and cannot distinguish
///
/// It cannot tell "rejected because the constraint is `CIRC`" from "rejected
/// because no `POLYLINE` waypoints were found" -- both paths of
/// `polyline_path_constraint` return the same
/// [`MoveItErrorCode::InvalidMotionPlan`], so no assertion on the code alone
/// can name the branch. What it *does* pin is that the two variants are not
/// interchangeable: the accepted request below is the failing one with the
/// variant put back, so the variant is the only difference between a plan and
/// a rejection. That is exactly the property the split bought -- a `CIRC`
/// constraint carries no waypoint list, so no reinterpretation of it as a
/// `POLYLINE` can be constructed, where before the split
/// `path_constraints` was a bare `CircPathConstraint` and neither request
/// shape here was expressible at all.
#[test]
fn polyline_plans_a_polyline_constraint_and_rejects_the_same_request_carrying_a_circ_one() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let (model, srdf) = load_panda();
    let (_, waypoints) = corner_waypoints(&model, &request.goal.link_name, &request.start_state);

    let base = TrajectoryGenerator::new(&model, panda_limits(&request));
    let generator = TrajectoryGeneratorPolyline::new(base, &request.group_name);

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let accepted = panda_polyline_request(&request, waypoints.clone(), &waypoints[1]);
    let response = generator.generate(&ctx, &accepted, request.sampling_time);
    assert_eq!(
        response.error_code,
        MoveItErrorCode::Success,
        "the request this test swaps the variant on must itself plan, or the \
         rejection below proves nothing"
    );

    let mut rejected = accepted;
    rejected.path_constraints = Some(PathConstraints::Circ(CircPathConstraint {
        kind: CircPathConstraintKind::Center,
        link_name: request.goal.link_name.clone(),
        point: waypoints[0].translation.vector,
    }));
    let response = generator.generate(&ctx, &rejected, request.sampling_time);
    assert_eq!(response.error_code, MoveItErrorCode::InvalidMotionPlan);
    assert!(response.trajectory.is_none());
}
