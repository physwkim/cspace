// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `TrajectoryGeneratorLin` parity test against the moveit2 C++ oracle's
//! `pilz_trajectory` op (`generator: "lin"`).
//!
//! Ground truth is captured verbatim into
//! `tests/fixtures/panda_lin_{request,response}.json` — a `panda_arm`
//! Cartesian goal for `panda_link8`: start at the SRDF `"ready"` pose (a
//! known non-self-colliding panda configuration -- see
//! `crate::trajectory_functions`'s own test fixtures), goal 0.1m further
//! along `+x` at the identical orientation (a pure-translation LIN motion,
//! so `PathLine`'s `angle == 0` branch is what this fixture exercises; the
//! `angle == PI` singularity in `path_line`'s `get_rot_angle` is covered by
//! that module's own unit tests instead, since a same-orientation-both-ends
//! goal can never reach it here).
//! `sampling_time` `0.1`, `max_velocity_scaling_factor`/
//! `max_acceleration_scaling_factor` `0.1`, `cartesian_limits` transcribed
//! from `third_party/moveit_resources/panda_moveit_config/config/pilz_cartesian_limits.yaml`.
//!
//! Every waypoint's `positions`/`velocities`/`accelerations`/`time_from_start`
//! is compared, not positions alone -- see `pilz_trajectory_parity.rs`'s own
//! module doc for why (round 18's two real bugs were invisible to a
//! positions-only trace).
//!
//! # A known IkContext-level self-collision deviation, sidestepped by this
//! fixture's own choice of pose, not fixed
//!
//! Upstream's `TrajectoryGeneratorLIN::extractMotionPlanInfo` calls
//! `computePoseIK` for its Cartesian-goal reachability check with
//! `check_self_collision`'s *default* (`true`), while its `plan` calls
//! `generateJointTrajectory` with *that* overload's default (`false`) --
//! two different defaults for two different calls in the same generator.
//! This port's `IkContext` carries one `check_self_collision` flag shared by
//! both call sites (`PilzGenerator::generate` threads the same
//! `ctx.check_self_collision` value through `extract_motion_plan_info` and
//! into `plan`'s `plan_ctx`), so no single flag value reproduces upstream's
//! per-call split exactly. This fixture's start/goal pose is not
//! self-colliding at any point along the path, so `CHECK_SELF_COLLISION`'s
//! value is inconsequential for this comparison -- chosen so the fixture
//! cannot silently depend on which of upstream's two defaults it happens to
//! match. Restructuring `IkContext` (or `PilzGenerator::generate`) to carry
//! two independent flags is a separate, larger change than "port LIN" and is
//! not made here.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_pilz::limits::{
    CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer,
};
use moveit_planners_pilz::trajectory_functions::IkContext;
use moveit_planners_pilz::trajectory_generator::{
    Goal, MotionPlanRequest, PilzGenerator, StartState, TrajectoryGenerator,
};
use moveit_planners_pilz::trajectory_generator_lin::TrajectoryGeneratorLin;
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

/// `time_from_start` comes only from `VelocityProfileTrap::duration` plus
/// `sampling_time` accumulation -- no IK anywhere in that chain -- so it gets
/// the same tight budget PTP's fixture uses. Measured max divergence on this
/// fixture: `1e-9`.
const TIME_TOLERANCE: f64 = 1e-6;

/// Every LIN waypoint is an independent IK solve -- this port's
/// `compute_pose_ik` (`moveit-kinematics`) against the oracle's
/// `kdl_kinematics_plugin` -- so unlike PTP (closed-form joint interpolation,
/// no IK in the loop) this comparison is between two solvers, not two
/// evaluations of one formula.
///
/// Measured per-fixture maximum, 2026-08-05, by collecting the maximum of
/// `(actual - expected).abs()` over every waypoint and joint of this
/// fixture's comparison loop below: **`2.09e-14`**. Set with a roughly 4x
/// margin -- see `CLAUDE.md`'s "Size test tolerances from measurement".
///
/// This replaces `5e-5`, which was set from a measured `1.26e-5` and the
/// reasoning that `panda_arm`'s 7-DOF redundancy lets two solvers land on
/// two different points of the same one-parameter null-space manifold. That
/// is not this tree's behaviour: the two solvers agree here to within a few
/// ULP, so the joint-space divergence the old number budgeted for is not
/// present and a `5e-5` budget left this comparison blind across nine orders
/// of magnitude. What changed is not established here -- only that the
/// divergence is absent now, and that `resolve_solver` selects by name
/// (`DEFAULT_SOLVER_NAME`, `registry.rs:203`) rather than by `linkme` order,
/// so which solver runs is no longer a function of the dependency graph.
const POSITION_TOLERANCE: f64 = 1e-13;

/// [`generate_joint_trajectory`]'s backward-difference velocity
/// (`Δposition / sampling_time`) amplifies [`POSITION_TOLERANCE`] by roughly
/// `1 / sampling_time` (`0.1` here). Measured per-fixture maximum:
/// `3.17e-14`; set with a roughly 4x margin. The amplification is the
/// mechanism, not a prediction of the size: at this magnitude the measured
/// velocity maximum is ~1.5x the position one, not 10x, so the number below
/// is the measurement rather than `POSITION_TOLERANCE / sampling_time`.
const VELOCITY_TOLERANCE: f64 = 1.5e-13;

/// The same backward-difference chain's acceleration term divides by
/// `sampling_time` again. Measured per-fixture maximum: `2.66e-13`; set with
/// a roughly 4x margin.
const ACCELERATION_TOLERANCE: f64 = 1.2e-12;

/// See this module's `# A known IkContext-level self-collision deviation`
/// doc section for why this fixture's own choice of pose makes the value
/// here inconsequential.
const CHECK_SELF_COLLISION: bool = true;

#[test]
fn lin_panda_arm_matches_the_oracle() {
    let request: RequestFixture = load_json("panda_lin_request.json");
    let response: ResponseFixture =
        load_json::<OracleResponseEnvelope<ResponseFixture>>("panda_lin_response.json").result;
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
    let generator = TrajectoryGeneratorLin::new(base, &request.group_name);

    assert_eq!(request.goal.kind, "cartesian");
    let [x, y, z, w] = request.goal.orientation;
    let goal = Goal::Cartesian {
        link_name: request.goal.link_name,
        // The oracle's request fixture has no `frame_id` field -- always the
        // model frame.
        frame: None,
        position: Vector3::new(
            request.goal.position[0],
            request.goal.position[1],
            request.goal.position[2],
        ),
        orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        target_point_offset: Vector3::new(0.0, 0.0, 0.0),
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

/// A genuine Pilz rejection, not a capture failure: the same start/goal as
/// [`lin_panda_arm_matches_the_oracle`], but `max_velocity_scaling_factor`/
/// `max_acceleration_scaling_factor` raised from `0.1` to `0.5`. The oracle's
/// own log for this fixture (captured alongside it, not reproduced here)
/// names the cause directly: `panda_joint2`'s acceleration reaches `3.52375`
/// against a `1.875` limit at `t=0.2s`. Upstream's `generateJointTrajectory`
/// throws `LinTrajectoryConversionFailure` wrapping `PLANNING_FAILED` (`-1`)
/// for this; this port's [`moveit_planners_pilz::trajectory_functions::generate_joint_trajectory`]
/// returns [`moveit_error::Error::Code`]`(`[`moveit_error::MoveItErrorCode::PlanningFailed`]`)`
/// for the identical reason -- `crate::trajectory_functions::verify_sample_joint_limits`
/// rejecting a backward-difference acceleration sample.
#[test]
fn lin_panda_arm_rejects_the_same_request_the_oracle_rejects() {
    let request: RequestFixture = load_json("panda_lin_scaling05_rejected_request.json");
    let response: ResponseFixture = load_json::<OracleResponseEnvelope<ResponseFixture>>(
        "panda_lin_scaling05_rejected_response.json",
    )
    .result;
    assert_eq!(
        response.error_code, -1,
        "fixture's own oracle run must have failed with PLANNING_FAILED"
    );
    // ASSERTION-DISCRIMINATION AUDIT (round 2): not applicable -- `response`
    // here is the oracle's own `ResponseFixture` deserialized from the
    // recorded JSON, not a value this port produced. There is no guard or
    // branch under test to discriminate; this only checks that the fixture
    // itself is internally consistent (an error-code response with no
    // waypoints) before it is used as the expected result below.
    assert!(
        response.waypoints.is_none(),
        "a PLANNING_FAILED response fixture must carry no waypoints"
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
    let generator = TrajectoryGeneratorLin::new(base, &request.group_name);

    assert_eq!(request.goal.kind, "cartesian");
    let [x, y, z, w] = request.goal.orientation;
    let goal = Goal::Cartesian {
        link_name: request.goal.link_name,
        // The oracle's request fixture has no `frame_id` field -- always the
        // model frame.
        frame: None,
        position: Vector3::new(
            request.goal.position[0],
            request.goal.position[1],
            request.goal.position[2],
        ),
        orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        target_point_offset: Vector3::new(0.0, 0.0, 0.0),
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
    // below already names. But `error_code == PlanningFailed` is itself a
    // collapsed shape: `generate_joint_trajectory` has two `PlanningFailed`
    // sites sharing this exact call path (`verify_sample_joint_limits`
    // rejection at trajectory_functions.rs:400, and `push_way_point`
    // failure at :438). Isolating mutation: neutralizing the :400 guard
    // (`if false && i != 0 && !verify_sample_joint_limits(..)`) flipped
    // this test from reject to accept, proving :400 -- not :438 -- is this
    // fixture's actual cause, matching the doc comment above.
    assert!(
        response.trajectory.is_none(),
        "this port must also reject the fixture's own rejected request"
    );
    assert_eq!(
        response.error_code,
        moveit_error::MoveItErrorCode::PlanningFailed,
        "rejection reason must match the oracle's PLANNING_FAILED"
    );
}

/// Regression for the gap `PORTING-PLAN.md` §227.6 records as #41
/// `JointNumberMismatch`, found while auditing residual claims:
/// `TrajectoryGeneratorLin`'s `Goal::Joint`
/// arm built `info.goal_joint_position` directly from the request without
/// ever checking its size against the planning group, unlike
/// `TrajectoryGeneratorCirc`'s identical arm (`trajectory_generator_circ.rs`)
/// and upstream `TrajectoryGeneratorLIN::extractMotionPlanInfo`, which throws
/// `JointNumberMismatch` for exactly this case right after resolving the tip
/// frame. No oracle fixture exercises this path (both `panda_lin_*.json`
/// fixtures are Cartesian goals), so this is a plain no-oracle construction
/// rather than a captured-response comparison; before the fix this request
/// planned successfully against whatever 6 joints were named, silently
/// leaving `panda_joint7` at its start-state value instead of being
/// rejected.
#[test]
fn lin_panda_arm_rejects_a_joint_goal_with_the_wrong_position_count() {
    let request: RequestFixture = load_json("panda_lin_request.json");
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
    let generator = TrajectoryGeneratorLin::new(base, &request.group_name);

    // panda_arm has 7 active joints; this omits panda_joint7.
    let positions: HashMap<String, f64> = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
    ]
    .into_iter()
    .map(|name| (name.to_string(), 0.0))
    .collect();
    let plan_request = MotionPlanRequest {
        group_name: request.group_name,
        start_state: StartState {
            position: request.start_state,
            velocity: HashMap::new(),
        },
        goal: Goal::Joint(positions),
        max_velocity_scaling_factor: request.max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.max_acceleration_scaling_factor,
        path_constraints: None,
    };

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let response = generator.generate(&ctx, &plan_request, request.sampling_time);
    assert!(
        response.trajectory.is_none(),
        "a joint-space goal with the wrong position count must be rejected, not silently planned"
    );
    assert_eq!(
        response.error_code,
        moveit_error::MoveItErrorCode::InvalidGoalConstraints,
        "must match TrajectoryGeneratorCirc's identical check and upstream's JointNumberMismatch"
    );
}
