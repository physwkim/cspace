// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `TrajectoryGeneratorCirc` parity test against the moveit2 C++ oracle's
//! `pilz_trajectory` op (`generator: "circ"`).
//!
//! Ground truth is captured verbatim into
//! `tests/fixtures/panda_circ_{request,response}.json` — the same
//! `panda_arm`/`panda_link8` start and Cartesian goal as
//! `pilz_trajectory_lin_parity.rs`'s own fixture (SRDF `"ready"` pose to
//! `+0.1m` along `x`, same orientation both ends), but routed through a
//! `"center"` path constraint at `(0.357, 0.05, z_ready)` instead of a
//! straight line -- equidistant from both endpoints (`x = 0.357` is their
//! midpoint), giving an exact quarter-circle sweep (`alpha = pi/2`) so
//! `circle_from_center`'s `cosines` call lands on a non-singular angle.
//! `sampling_time`/scaling factors/`cartesian_limits`/`joint_limits` are
//! identical to the LIN fixture's.
//!
//! Every waypoint's `positions`/`velocities`/`accelerations`/`time_from_start`
//! is compared, not positions alone — see `pilz_trajectory_parity.rs`'s own
//! module doc for why.
//!
//! `panda_circ_noplane_rejected_{request,response}.json` moves the same
//! `"center"` point onto the `x = 0.357` line through the start/goal
//! midpoint (`y = 0` instead of `y = 0.05`) — `circle_from_center` still
//! succeeds (start/goal remain equidistant from a center on their
//! perpendicular bisector), but the resulting geometry is exactly
//! `path_circle.rs`'s own `half_circle_from_center_has_no_determinable_plane`
//! unit test: `aux_point = goal` is colinear with the start-to-center radius
//! vector, so `PathCircle::new` rejects it. The oracle's own log for this
//! fixture (captured alongside it, not reproduced here) names the same
//! cause: `Circle : Plane for motion is not properly defined`, i.e. upstream
//! `CircleNoPlane`, `INVALID_MOTION_PLAN` (`-2`).
//!
//! # The same IK-parity debt as LIN, inherited here
//!
//! `TrajectoryGeneratorCirc::extract_motion_plan_info`'s Cartesian-goal
//! branch and `plan`'s per-waypoint sampling both route through the same
//! `compute_pose_ik`/`cspace-kinematics` machinery
//! `pilz_trajectory_lin_parity.rs`'s own "known IkContext-level
//! self-collision deviation" doc section already documents — this fixture's
//! start/goal pose is the identical one, chosen there specifically so
//! `CHECK_SELF_COLLISION`'s value is inconsequential, and that reasoning
//! carries over unchanged. Any position/velocity/acceleration divergence
//! measured below was attributed to the same "different IK solver, same
//! redundant manifold" origin as LIN's rather than to a CIRC-specific
//! trajectory-generation bug. That attribution no longer holds as measured:
//! LIN's residual is now `2.09e-14` (a few ULP), so there is no joint-space
//! null-space divergence there to inherit, and CIRC's own `1.81e-9` is
//! whatever this generator's chain produces rather than a shared
//! solver-choice effect. Both constants below are set from their own
//! measurement.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_planners_pilz::limits::{
    CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer,
};
use cspace_planners_pilz::trajectory_functions::IkContext;
use cspace_planners_pilz::trajectory_generator::{
    CircPathConstraint, CircPathConstraintKind, Goal, MotionPlanRequest, PathConstraints,
    PilzGenerator, StartState, TrajectoryGenerator,
};
use cspace_planners_pilz::trajectory_generator_circ::TrajectoryGeneratorCirc;
use cspace_scene::PlanningScene;

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
struct FixtureCartesianLimits {
    max_trans_vel: f64,
    max_trans_acc: f64,
    max_trans_dec: f64,
    max_rot_vel: f64,
}

impl From<FixtureCartesianLimits> for CartesianLimits {
    fn from(f: FixtureCartesianLimits) -> Self {
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
    kind: String,
    link_name: String,
    position: [f64; 3],
    orientation: [f64; 4],
}

#[derive(Deserialize)]
struct PathConstraintFixture {
    name: String,
    link_name: String,
    position: [f64; 3],
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
    path_constraint: PathConstraintFixture,
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

/// Same chain as LIN's fixture (`VelocityProfileTrap::duration` plus
/// `sampling_time` accumulation, no IK involved). Measured max divergence on
/// this fixture: `1e-9`; set with a large margin since this quantity has no
/// IK-solver noise to absorb.
const TIME_TOLERANCE: f64 = 1e-6;

/// See this module's `# The same IK-parity debt as LIN, inherited here` doc
/// section. Measured per-fixture maximum, 2026-08-05, over every waypoint and
/// joint of the comparison loop below: **`1.81e-9`**; set with a roughly 4x
/// margin, not copied from LIN's own constant — see `CLAUDE.md`'s "Size test
/// tolerances from measurement".
///
/// This replaces `2e-5`, set from a measured `4.05e-6` under the same
/// null-space reasoning LIN's constant used. LIN now measures `2.09e-14`, so
/// that reasoning has stopped describing either fixture; CIRC's own residual
/// is four orders larger than LIN's but still four orders under the old
/// budget, which is why it gets its own measurement rather than LIN's.
const POSITION_TOLERANCE: f64 = 8e-9;

/// Backward-difference velocity amplifies [`POSITION_TOLERANCE`] by roughly
/// `1 / sampling_time` (`0.1` here), same chain as LIN's. Measured
/// per-fixture maximum: `5.52e-9`; set with a roughly 4x margin. As in LIN,
/// the amplification names the mechanism and not the size — the measured
/// velocity maximum here is ~3x the position one, not 10x.
const VELOCITY_TOLERANCE: f64 = 2.5e-8;

/// The acceleration term divides by `sampling_time` again. Measured
/// per-fixture maximum: `3.32e-8`; set with a roughly 4x margin.
const ACCELERATION_TOLERANCE: f64 = 1.4e-7;

/// See `pilz_trajectory_lin_parity.rs`'s own `# A known IkContext-level
/// self-collision deviation` doc section — this fixture reuses that
/// module's start/goal pose specifically so this value is inconsequential.
const CHECK_SELF_COLLISION: bool = true;

fn path_constraint(f: PathConstraintFixture) -> PathConstraints {
    let kind = match f.name.as_str() {
        "center" => CircPathConstraintKind::Center,
        "interim" => CircPathConstraintKind::Interim,
        other => panic!("fixture path constraint kind {other} not handled by this test"),
    };
    PathConstraints::Circ(CircPathConstraint {
        kind,
        link_name: f.link_name,
        // The oracle's fixture has no `frame_id` field for the path
        // constraint either -- always the model frame.
        frame: None,
        point: Vector3::new(f.position[0], f.position[1], f.position[2]),
    })
}

fn cartesian_goal(f: GoalFixture) -> Goal {
    assert_eq!(f.kind, "cartesian");
    let [x, y, z, w] = f.orientation;
    Goal::Cartesian {
        link_name: f.link_name,
        frame: None,
        position: Vector3::new(f.position[0], f.position[1], f.position[2]),
        orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        target_point_offset: Vector3::new(0.0, 0.0, 0.0),
    }
}

#[test]
fn circ_panda_arm_matches_the_oracle() {
    let request: RequestFixture = load_json("panda_circ_request.json");
    let response: ResponseFixture =
        load_json::<OracleResponseEnvelope<ResponseFixture>>("panda_circ_response.json").result;
    assert_eq!(
        response.error_code, 1,
        "fixture's own oracle run must have succeeded"
    );
    let expected_waypoints = response
        .waypoints
        .expect("SUCCESS response fixture must carry waypoints");

    let (model, srdf) = load_panda();

    let mut joint_limits = JointLimitsContainer::default();
    for (name, limit) in request.joint_limits {
        assert!(
            joint_limits.add_limit(name.clone(), limit.into()),
            "duplicate or invalid joint limit for {name} in fixture"
        );
    }
    let mut limits = LimitsContainer::new();
    limits.set_joint_limits(joint_limits);
    limits.set_cartesian_limits(request.cartesian_limits.into());

    let base = TrajectoryGenerator::new(&model, limits);
    let generator = TrajectoryGeneratorCirc::new(base, &request.group_name);

    let plan_request = MotionPlanRequest {
        group_name: request.group_name,
        start_state: StartState {
            position: request.start_state,
            velocity: HashMap::new(),
        },
        goal: cartesian_goal(request.goal),
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: Some(path_constraint(request.path_constraint)),
    };

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

/// A genuine Pilz rejection, not a capture failure — see this module's own
/// doc for how `panda_circ_noplane_rejected_request.json`'s center point was
/// chosen to trigger it.
#[test]
fn circ_panda_arm_rejects_the_same_request_the_oracle_rejects() {
    let request: RequestFixture = load_json("panda_circ_noplane_rejected_request.json");
    let response: ResponseFixture = load_json::<OracleResponseEnvelope<ResponseFixture>>(
        "panda_circ_noplane_rejected_response.json",
    )
    .result;
    assert_eq!(
        response.error_code, -2,
        "fixture's own oracle run must have failed with INVALID_MOTION_PLAN"
    );
    // ASSERTION-DISCRIMINATION AUDIT (round 2): not applicable -- `response`
    // here is the oracle's own `ResponseFixture` deserialized from the
    // recorded JSON, not a value this port produced. There is no guard or
    // branch under test to discriminate; this only checks that the fixture
    // itself is internally consistent (an error-code response with no
    // waypoints) before it is used as the expected result below.
    assert!(
        response.waypoints.is_none(),
        "an INVALID_MOTION_PLAN response fixture must carry no waypoints"
    );

    let (model, srdf) = load_panda();

    let mut joint_limits = JointLimitsContainer::default();
    for (name, limit) in request.joint_limits {
        assert!(
            joint_limits.add_limit(name.clone(), limit.into()),
            "duplicate or invalid joint limit for {name} in fixture"
        );
    }
    let mut limits = LimitsContainer::new();
    limits.set_joint_limits(joint_limits);
    limits.set_cartesian_limits(request.cartesian_limits.into());

    let base = TrajectoryGenerator::new(&model, limits);
    let generator = TrajectoryGeneratorCirc::new(base, &request.group_name);

    let plan_request = MotionPlanRequest {
        group_name: request.group_name,
        start_state: StartState {
            position: request.start_state,
            velocity: HashMap::new(),
        },
        goal: cartesian_goal(request.goal),
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: Some(path_constraint(request.path_constraint)),
    };

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let response = generator.generate(&ctx, &plan_request, request.sampling_time);
    // ASSERTION-DISCRIMINATION AUDIT (round 2): `discriminating` (§3a bite
    // done) -- `MotionPlanResponse::failure` is the only place in this crate
    // that writes `trajectory: None` (`rg -n 'trajectory: None'`
    // crate-wide: 1 hit), so this line's own cause is what `error_code`
    // below already names. But `error_code == InvalidMotionPlan` is itself
    // a collapsed shape: `TrajectoryGeneratorCirc::plan`/`build_path` have
    // four `InvalidMotionPlan` sites sharing this exact call path (no
    // `circ_aux_point`, `circle_from_center` failure, `circle_from_interim`
    // failure, `PathCircle::new` failure). Traced with eprintln! at each of
    // the four sites (reverted after): only `PathCircle::new`'s site fired
    // for this fixture, matching this module's own doc comment's claim
    // that `circle_from_center` succeeds and `PathCircle::new`'s
    // no-determinable-plane check is what actually rejects it.
    assert!(
        response.trajectory.is_none(),
        "this port must also reject the fixture's own rejected request"
    );
    assert_eq!(
        response.error_code,
        cspace_core::error::MoveItErrorCode::InvalidMotionPlan,
        "rejection reason must match the oracle's INVALID_MOTION_PLAN"
    );
}
