// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `TrajectoryGeneratorPtp` parity test against the moveit2 C++ oracle's
//! `pilz_trajectory` op.
//!
//! Ground truth is captured verbatim into
//! `tests/fixtures/panda_ptp_{request,response}.json` — a `panda_arm` joint
//! goal at `max_velocity_scaling_factor`/`max_acceleration_scaling_factor`
//! `0.5`, `sampling_time` `0.1`, `panda_joint4`'s goal negative (its URDF
//! range is `[-3.1416, 0.0873]`, so a `+0.5` goal like the other six joints
//! would be an out-of-range request the oracle rejects before planning ever
//! starts). `joint_limits` mirrors `crate::limits::JointLimit` field-for-field
//! (see `trajectory_generator.rs`'s oracle-op doc comment) — position limits
//! from `fixtures/panda.urdf`, velocity/acceleration from
//! `third_party/moveit_resources/panda_moveit_config/config/joint_limits.yaml`,
//! deceleration defaulted to `-`acceleration (upstream
//! `JointLimitsAggregator::getAggregatedLimits`'s own rule when a YAML entry
//! sets acceleration but not deceleration).
//!
//! Every waypoint's `positions`/`velocities`/`accelerations`/`time_from_start`
//! is compared, not positions alone — round 18 found two real bugs
//! (`is_state_colliding`'s inverted return, `push_way_point`'s missing
//! reference-state seed) that a positions-only trace cannot see: the second
//! corrupts a floating joint's *state* rather than its interpolated position,
//! and the backward-difference velocity/acceleration chain is arithmetic no
//! positions-only fixture exercises at all.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_planners_pilz::limits::{JointLimit, JointLimitsContainer, LimitsContainer};
use cspace_planners_pilz::trajectory_functions::IkContext;
use cspace_planners_pilz::trajectory_generator::{
    Goal, MotionPlanRequest, PilzGenerator, StartState, TrajectoryGenerator,
};
use cspace_planners_pilz::trajectory_generator_ptp::TrajectoryGeneratorPtp;
use cspace_scene::PlanningScene;
use cspace_srdf::SrdfModel;

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

impl From<FixtureJointLimit> for JointLimit {
    fn from(f: FixtureJointLimit) -> Self {
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
struct GoalFixture {
    kind: String,
    joints: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct RequestFixture {
    group_name: String,
    sampling_time: f64,
    joint_limits: HashMap<String, FixtureJointLimit>,
    start_state: HashMap<String, f64>,
    max_velocity_scaling_factor: f64,
    max_acceleration_scaling_factor: f64,
    goal: GoalFixture,
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

/// The committed fixture files are full oracle wire responses
/// (`{"id":.., "ok":.., "result": {..}}`, verbatim from `oracle.cpp`'s
/// stdout) so `verify-fixture-replay.sh` can replay the committed
/// `*_request.json` and diff byte-for-byte against this file -- see that
/// script's own module doc for why replay needs the exact wire shape, not a
/// curated subset. `ResponseFixture` above only cares about `result`.
#[derive(Deserialize)]
struct OracleResponseEnvelope<T> {
    result: T,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load_json<T: serde::de::DeserializeOwned>(file_name: &str) -> T {
    let path = fixture_path(file_name);
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

/// The oracle (Eigen) and this port (nalgebra) run different trapezoid-profile
/// and IK-free joint-space arithmetic, so a `1e-6` budget -- not bit-for-bit
/// -- is what a genuine agreement claim can make. See the module doc's
/// "positions alone" note for why every field, not only position, is checked
/// at this tolerance.
const TOLERANCE: f64 = 1e-6;

#[test]
fn ptp_panda_arm_matches_the_oracle() {
    let request: RequestFixture = load_json("panda_ptp_request.json");
    let response: ResponseFixture =
        load_json::<OracleResponseEnvelope<ResponseFixture>>("panda_ptp_response.json").result;
    assert_eq!(
        response.error_code, 1,
        "fixture's own oracle run must have succeeded"
    );
    let expected_waypoints = response
        .waypoints
        .expect("SUCCESS response fixture must carry waypoints");

    let (model, _srdf) = load_panda();

    let mut joint_limits = JointLimitsContainer::default();
    for (name, limit) in request.joint_limits {
        assert!(
            joint_limits.add_limit(name.clone(), limit.into()),
            "duplicate or invalid joint limit for {name} in fixture"
        );
    }
    let mut limits = LimitsContainer::new();
    limits.set_joint_limits(joint_limits);

    let base = TrajectoryGenerator::new(&model, limits);
    let generator = TrajectoryGeneratorPtp::new(base, &request.group_name)
        .expect("fixture's own joint limits cover every panda_arm joint");

    let goal = match request.goal.kind.as_str() {
        "joint" => Goal::Joint(request.goal.joints),
        other => panic!("fixture goal kind {other} not handled by this test"),
    };
    let plan_request = MotionPlanRequest {
        group_name: request.group_name,
        start_state: StartState {
            position: request.start_state,
            velocity: HashMap::new(),
        },
        goal,
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: None,
    };

    let scene = Arc::new(PlanningScene::new(&model, &_srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: true,
    };

    let response = generator.generate(&ctx, &plan_request, request.sampling_time);
    let trajectory = response
        .trajectory
        .expect("this port must also reach SUCCESS on the fixture's own accepted request");

    assert_eq!(
        trajectory.way_point_count(),
        expected_waypoints.len(),
        "waypoint count must match the oracle exactly"
    );

    for (i, expected) in expected_waypoints.iter().enumerate() {
        let actual_dt = trajectory.way_point_duration_from_start(i);
        assert!(
            (actual_dt - expected.time_from_start).abs() < TOLERANCE,
            "waypoint {i} time_from_start: {actual_dt} != {} (oracle)",
            expected.time_from_start
        );

        let state = trajectory.way_point(i).unwrap();
        for (name, &expected_pos) in &expected.positions {
            let actual_pos = state.variable_position(name).unwrap();
            assert!(
                (actual_pos - expected_pos).abs() < TOLERANCE,
                "waypoint {i} position[{name}]: {actual_pos} != {expected_pos} (oracle)"
            );
        }
        for (name, &expected_vel) in &expected.velocities {
            let actual_vel = state.variable_velocity(name).unwrap();
            assert!(
                (actual_vel - expected_vel).abs() < TOLERANCE,
                "waypoint {i} velocity[{name}]: {actual_vel} != {expected_vel} (oracle)"
            );
        }
        for (name, &expected_acc) in &expected.accelerations {
            let actual_acc = state.variable_acceleration(name).unwrap();
            assert!(
                (actual_acc - expected_acc).abs() < TOLERANCE,
                "waypoint {i} acceleration[{name}]: {actual_acc} != {expected_acc} (oracle)"
            );
        }
    }
}
