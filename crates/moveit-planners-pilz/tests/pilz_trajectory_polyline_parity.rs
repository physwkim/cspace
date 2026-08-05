// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `TrajectoryGeneratorPolyline` parity test against the moveit2 C++ oracle's
//! `pilz_trajectory` op (`generator: "polyline"`).
//!
//! Ground truth is captured verbatim into
//! `tests/fixtures/panda_polyline_{request,response}.json`. The request is
//! `panda_lin_request.json` with the generator swapped and a two-waypoint
//! `path_waypoints` list added: the same `panda_arm`/`panda_link8` SRDF
//! `"ready"` start, then `+0.15 m` along `x` and `+0.15 m` along `y` from the
//! start tip pose, orientation held at the start's — the right-angle corner
//! `pilz_trajectory_polyline.rs`'s property tests already use, so the two
//! files describe one motion from two directions rather than two motions.
//! `sampling_time`/scaling factors/`cartesian_limits`/`joint_limits` are the
//! LIN fixture's, unmodified.
//!
//! The waypoint poses are absolute numbers in the fixture, not FK-derived at
//! test time, because the oracle reads them straight out of the request as
//! `position_constraints[i].constraint_region.primitive_poses[0]`. Only
//! `start_pose` is each side's own FK, which is what
//! `pilz_trajectory_lin_parity.rs` already compares.
//!
//! Every waypoint's `positions`/`velocities`/`accelerations`/`time_from_start`
//! is compared, not positions alone — see `pilz_trajectory_parity.rs`'s own
//! module doc for why.
//!
//! `panda_polyline_onewaypoint_rejected_{request,response}.json` is the same
//! request with the second waypoint removed and the goal moved onto the
//! remaining one, so the *only* reason left to reject is the count. The
//! oracle's own log for that fixture (captured alongside it, not reproduced
//! here) names it: `waypoints specified in path constraints is less than 2
//! for POLYLINE motion`, i.e. upstream `NoWaypointsSpecified`,
//! `INVALID_MOTION_PLAN` (`-2`).
//!
//! # The same IK-parity debt as LIN, inherited here
//!
//! `TrajectoryGeneratorPolyline::extract_motion_plan_info`'s Cartesian-goal
//! branch and `plan`'s per-waypoint sampling route through the same
//! `compute_pose_ik`/`moveit-kinematics` machinery
//! `pilz_trajectory_lin_parity.rs`'s own "known IkContext-level
//! self-collision deviation" doc section documents. This fixture's start pose
//! is that module's, and every waypoint is a small free-space translation
//! from it, so `CHECK_SELF_COLLISION`'s value is inconsequential here for the
//! same reason.
//!
//! What is *not* inherited is the size. LIN's fixture measures `1.26e-5 rad`
//! of joint-space divergence and CIRC's `4.05e-6`; this one measures
//! `1.60e-9` — four orders tighter, on the same solver pair and the same
//! start pose. Why the two fixtures separate by that much is not established
//! here, and this file does not assert a reason for it. The tolerances below
//! are therefore sized from *this* fixture's own measurement rather than
//! inherited from its siblings', which is what keeps a future four-order
//! regression visible instead of absorbed.
//!
//! # What the `polyline-filter-waypoints-stale-index` deviation means here
//!
//! This fixture's two waypoints are `0.15 m` apart, far above
//! `path_polyline_generator::MIN_SEGMENT_LENGTH` (`0.2 mm`), so
//! `filter_waypoints` drops nothing and the upstream index bug
//! `doc/upstream-bugs.md` records as `reproduced-deliberately` never fires on
//! either side. This test therefore neither confirms nor refutes that entry;
//! it establishes the parity surface that entry says did not exist yet.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{Isometry3, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_pilz::limits::{
    CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer,
};
use moveit_planners_pilz::trajectory_functions::IkContext;
use moveit_planners_pilz::trajectory_generator::{
    Goal, MotionPlanRequest, PathConstraints, PilzGenerator, PolylinePathConstraint, StartState,
    TrajectoryGenerator,
};
use moveit_planners_pilz::trajectory_generator_polyline::TrajectoryGeneratorPolyline;
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;

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

/// One pose, in the wire order the oracle reads it back in:
/// `orientation` is `[x, y, z, w]`, matching
/// `geometry_msgs::msg::Pose::orientation`'s own field order.
#[derive(Deserialize)]
struct PoseFixture {
    position: [f64; 3],
    orientation: [f64; 4],
}

impl From<&PoseFixture> for Isometry3 {
    fn from(f: &PoseFixture) -> Self {
        let [x, y, z, w] = f.orientation;
        Isometry3::from_parts(
            nalgebra::Translation3::new(f.position[0], f.position[1], f.position[2]),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        )
    }
}

#[derive(Deserialize)]
struct GoalFixture {
    kind: String,
    link_name: String,
    position: [f64; 3],
    orientation: [f64; 4],
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
    path_waypoints: Vec<PoseFixture>,
    smoothness_level: f64,
}

#[derive(Deserialize)]
struct WaypointFixture {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
    time_from_start: f64,
}

#[derive(Deserialize)]
struct ResponseFixture {
    error_code: i32,
    waypoints: Option<Vec<WaypointFixture>>,
}

/// See `pilz_trajectory_circ_parity.rs`'s own comment on this type: the
/// committed files are full oracle wire responses so
/// `verify-fixture-replay.sh` can diff them byte-for-byte.
#[derive(Deserialize)]
struct OracleResponseEnvelope<T> {
    result: T,
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

/// Same chain as LIN's and CIRC's fixtures (`VelocityProfileTrap::duration`
/// plus `sampling_time` accumulation, no IK involved). Measured maximum over
/// this fixture's own 34 waypoints: `1.00e-9 s`; set with a 10x margin.
const TIME_TOLERANCE: f64 = 1e-8;

/// See this module's `# The same IK-parity debt as LIN, inherited here` doc
/// section for where the divergence comes from, and its measured size on
/// this fixture. Measured maximum: `1.60e-9 rad`; set with a roughly 6x
/// margin. Deliberately *not* copied from CIRC's `2e-5` or LIN's `5e-5`,
/// which would be four orders too loose here — see `CLAUDE.md`'s "Size test
/// tolerances from measurement".
const POSITION_TOLERANCE: f64 = 1e-8;

/// Backward-difference velocity amplifies [`POSITION_TOLERANCE`] by roughly
/// `1 / sampling_time` (`0.1` here), same chain as LIN's and CIRC's. Measured
/// maximum: `7.64e-9 rad/s`; set with a roughly 6x margin.
const VELOCITY_TOLERANCE: f64 = 5e-8;

/// The acceleration term divides by `sampling_time` again, amplifying
/// [`VELOCITY_TOLERANCE`] by roughly another `1 / sampling_time`. Measured
/// maximum: `7.60e-8 rad/s^2`; set with a roughly 6x margin.
const ACCELERATION_TOLERANCE: f64 = 5e-7;

/// See `pilz_trajectory_lin_parity.rs`'s own `# A known IkContext-level
/// self-collision deviation` doc section — this fixture reuses that module's
/// start pose specifically so this value is inconsequential.
const CHECK_SELF_COLLISION: bool = true;

fn cartesian_goal(f: &GoalFixture) -> Goal {
    assert_eq!(f.kind, "cartesian");
    let [x, y, z, w] = f.orientation;
    Goal::Cartesian {
        link_name: f.link_name.clone(),
        position: Vector3::new(f.position[0], f.position[1], f.position[2]),
        orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        target_point_offset: Vector3::new(0.0, 0.0, 0.0),
    }
}

fn limits_of(request: &RequestFixture) -> LimitsContainer {
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

fn plan_request(request: &RequestFixture) -> MotionPlanRequest {
    MotionPlanRequest {
        group_name: request.group_name.clone(),
        start_state: StartState {
            position: request.start_state.clone(),
            velocity: HashMap::new(),
        },
        goal: cartesian_goal(&request.goal),
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: Some(PathConstraints::Polyline(PolylinePathConstraint {
            waypoints: request.path_waypoints.iter().map(Isometry3::from).collect(),
            smoothness_level: request.smoothness_level,
        })),
    }
}

#[test]
fn polyline_panda_arm_matches_the_oracle() {
    let request: RequestFixture = load_json("panda_polyline_request.json");
    let response: ResponseFixture =
        load_json::<OracleResponseEnvelope<ResponseFixture>>("panda_polyline_response.json").result;
    assert_eq!(
        response.error_code, 1,
        "fixture's own oracle run must have succeeded"
    );
    let expected_waypoints = response
        .waypoints
        .expect("SUCCESS response fixture must carry waypoints");

    let (model, srdf) = load_panda();
    let base = TrajectoryGenerator::new(&model, limits_of(&request));
    let generator = TrajectoryGeneratorPolyline::new(base, &request.group_name);

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let response = generator.generate(&ctx, &plan_request(&request), request.sampling_time);
    let trajectory = response.trajectory.unwrap_or_else(|| {
        panic!(
            "this port must also reach SUCCESS on the fixture's own accepted request, got {:?}",
            response.error_code
        )
    });

    assert_eq!(
        trajectory.way_point_count(),
        expected_waypoints.len(),
        "waypoint count must match the oracle exactly"
    );

    for (i, expected) in expected_waypoints.iter().enumerate() {
        let actual_dt = trajectory.way_point_duration_from_start(i);
        assert!(
            (actual_dt - expected.time_from_start).abs() < TIME_TOLERANCE,
            "waypoint {i} time_from_start: {actual_dt} != {} (oracle)",
            expected.time_from_start
        );

        let state = trajectory.way_point(i).unwrap();
        for (name, &expected_pos) in &expected.positions {
            let actual_pos = state.variable_position(name).unwrap();
            assert!(
                (actual_pos - expected_pos).abs() < POSITION_TOLERANCE,
                "waypoint {i} position[{name}]: {actual_pos} != {expected_pos} (oracle)"
            );
        }
        for (name, &expected_vel) in &expected.velocities {
            let actual_vel = state.variable_velocity(name).unwrap();
            assert!(
                (actual_vel - expected_vel).abs() < VELOCITY_TOLERANCE,
                "waypoint {i} velocity[{name}]: {actual_vel} != {expected_vel} (oracle)"
            );
        }
        for (name, &expected_acc) in &expected.accelerations {
            let actual_acc = state.variable_acceleration(name).unwrap();
            assert!(
                (actual_acc - expected_acc).abs() < ACCELERATION_TOLERANCE,
                "waypoint {i} acceleration[{name}]: {actual_acc} != {expected_acc} (oracle)"
            );
        }
    }
}

/// Upstream `TrajectoryGeneratorPolyline::cmdSpecificRequestValidation`'s
/// `NoWaypointsSpecified`, compared against the oracle rather than asserted
/// from this port alone — see this module's doc for how the fixture isolates
/// the count as the only remaining reason to reject.
#[test]
fn polyline_panda_arm_rejects_the_same_request_the_oracle_rejects() {
    let request: RequestFixture = load_json("panda_polyline_onewaypoint_rejected_request.json");
    let response: ResponseFixture = load_json::<OracleResponseEnvelope<ResponseFixture>>(
        "panda_polyline_onewaypoint_rejected_response.json",
    )
    .result;
    assert_eq!(
        response.error_code, -2,
        "fixture's own oracle run must have failed with INVALID_MOTION_PLAN"
    );
    assert!(
        response.waypoints.is_none(),
        "an INVALID_MOTION_PLAN response fixture must carry no waypoints"
    );
    assert_eq!(
        request.path_waypoints.len(),
        1,
        "this fixture rejects on the waypoint count, so the count is what must \
         be under two"
    );

    let (model, srdf) = load_panda();
    let base = TrajectoryGenerator::new(&model, limits_of(&request));
    let generator = TrajectoryGeneratorPolyline::new(base, &request.group_name);

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let mut plan = plan_request(&request);
    let response = generator.generate(&ctx, &plan, request.sampling_time);
    assert!(
        response.trajectory.is_none(),
        "this port must also reject the fixture's own rejected request"
    );
    assert_eq!(
        response.error_code,
        moveit_error::MoveItErrorCode::InvalidMotionPlan,
        "rejection reason must match the oracle's INVALID_MOTION_PLAN"
    );

    // The count is what rejected it, not the fixture's goal or start state:
    // putting the dropped waypoint back — the only edit — must plan. Without
    // this, the assertion above passes for any request this port happens to
    // dislike, including one the oracle rejected for a different reason.
    let source: RequestFixture = load_json("panda_polyline_request.json");
    plan.path_constraints = Some(PathConstraints::Polyline(PolylinePathConstraint {
        waypoints: source.path_waypoints.iter().map(Isometry3::from).collect(),
        smoothness_level: request.smoothness_level,
    }));
    plan.goal = cartesian_goal(&source.goal);
    let response = generator.generate(&ctx, &plan, request.sampling_time);
    assert_eq!(
        response.error_code,
        moveit_error::MoveItErrorCode::Success,
        "restoring the second waypoint must plan, or the rejection above is \
         not attributable to the count"
    );
}
