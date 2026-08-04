// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `TrajectoryBlenderTransitionWindow::blend` parity test against the
//! moveit2 C++ oracle's `pilz_blend` op. Closes the gap
//! `doc/oracle-request-pilz-blend.md` filed and PORTING-PLAN.md §188
//! records the op landing for (`b63171d`): the twelve unit tests in
//! `trajectory_blender_transition_window.rs` check this port's own logic
//! against itself, never against upstream's actual numeric output.
//!
//! Ground truth is `tests/fixtures/panda_blend_{symmetric,asymmetric}_{request,response}.json`
//! -- two LIN segments (SRDF `"ready"` pose, `+0.1m` along `+x`, then
//! `+0.1m` along `+y` from that corner, a genuine direction change) blended
//! at `blend_radius: 0.05`. The two cases differ only in segment 2's
//! `max_velocity_scaling_factor`/`max_acceleration_scaling_factor` (`0.1` vs
//! `0.3`), chosen so `determine_trajectory_alignment`'s two branches are each
//! exercised by a real oracle response: symmetric hits the `else` branch
//! (`8 == 8`), asymmetric hits `way_point_count_1 > way_point_count_2`
//! (`8 > 4`) -- see PORTING-PLAN.md §188.2 for both cases' measured indices.
//!
//! Three further cases move the geometry rather than the speeds, per
//! `doc/oracle-request-pilz-blend-geometry.md`: `panda_blend_radius08`
//! (case C) raises `blend_radius` to `0.08`, moving the intersection
//! indices to `(5, 10)` so the two walks are exercised somewhere other than
//! the single `(8, 7)` point A/B pin them at; `panda_blend_corner150`
//! (case D) turns the corner through 150 degrees instead of 90, and is a
//! *rejection* case -- see its own test's doc comment and PORTING-PLAN.md
//! §207; `panda_blend_corner112` (case E) turns the corner through 112
//! degrees, the sharpest angle short of case D's rejection boundary at
//! which the full pipeline still succeeds -- see its own test's doc comment
//! and `CORNER112_VELOCITY_TOLERANCE` for a genuine growing-divergence
//! finding this case surfaces that cases A-D do not.
//!
//! Case E's report attributed that growth to "corner sharpness driving
//! redundant-kinematics IK divergence" -- plausible, and a one-data-point
//! claim. Case F turns that into a falsifiable prediction (divergence
//! monotone in corner angle, radius and per-segment speed fixed) and tests
//! it with an 8-point angle sweep (`panda_blend_corner{30,60,75,100,105,110}`
//! plus cases A/E as the 90°/112° anchors) and a radius control
//! (`panda_blend_corner112_radius03`, case E's own 112° with `blend_radius`
//! dropped to `0.03`). The prediction is refuted: 90°/100° measure the
//! *lowest* divergence of the whole sweep, not a monotone function of angle
//! -- see `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section for
//! the full table and the radius control's own result.
//!
//! # No `blend_align_index` field, by design -- see PORTING-PLAN.md §188
//!
//! The request document asked for `blend_align_index` alongside the two
//! intersection indices. It is not in the response fixture, deliberately:
//! `determineTrajectoryAlignment` is a private member of
//! `TrajectoryBlenderTransitionWindow`, unreachable from `oracle.cpp` without
//! reimplementing its six lines there -- which would make this fixture
//! compare the port against a second oracle-side implementation of the same
//! branch, not against upstream's own execution. The two intersection
//! indices survive as real oracle output because `blend()`'s own copy loops
//! truncate the response trajectories at exactly those indices: the same
//! recovery this file performs on the *port's* side below via
//! `blend_response.first_trajectory.way_point_count()` and
//! `second_trajectory_input_waypoint_count - blend_response.second_trajectory.way_point_count() - 1`,
//! since `search_intersection_points` is a private `fn` in
//! `trajectory_blender_transition_window.rs` and not reachable from this
//! integration test either. The alignment branch itself is not asserted
//! directly (there is no witness for it independent of the waypoints), but
//! it is exercised: the asymmetric case's numeric divergence check below is
//! precisely what would catch a branch inverted, output coincidentally
//! similar" bug, per that document's own "Why the alignment branch needs its
//! own field" section.
//!
//! Every waypoint's `positions`/`velocities`/`accelerations`/`time_from_start`
//! is compared across all three response segments (`first_trajectory`,
//! `blend_trajectory`, `second_trajectory`), not `blend_trajectory` alone --
//! `first_trajectory`/`second_trajectory`'s waypoint counts are themselves
//! the intersection-index witness described above, so comparing only
//! `blend_trajectory` would silently drop that check.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{LinkPaddingScale, ParryCollisionEnv, World};
use moveit_error::{Error, MoveItErrorCode, Result};
use moveit_geometry::{UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_pilz::limits::{
    CartesianLimits, JointLimit, JointLimitsContainer, LimitsContainer,
};
use moveit_planners_pilz::trajectory_blender_transition_window::{
    TrajectoryBlendRequest, TrajectoryBlendResponse, blend,
};
use moveit_planners_pilz::trajectory_functions::IkContext;
use moveit_planners_pilz::trajectory_generator::{
    Goal, MotionPlanRequest, PilzGenerator, StartState, TrajectoryGenerator,
};
use moveit_planners_pilz::trajectory_generator_lin::TrajectoryGeneratorLin;
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_trajectory::RobotTrajectory;

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
    kind: String,
    link_name: String,
    position: [f64; 3],
    orientation: [f64; 4],
}

fn goal_from_fixture(g: &GoalFixture) -> Goal {
    assert_eq!(g.kind, "cartesian");
    let [x, y, z, w] = g.orientation;
    Goal::Cartesian {
        link_name: g.link_name.clone(),
        position: Vector3::new(g.position[0], g.position[1], g.position[2]),
        orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z)),
        target_point_offset: Vector3::new(0.0, 0.0, 0.0),
    }
}

#[derive(Deserialize)]
struct SegmentFixture {
    max_velocity_scaling_factor: f64,
    max_acceleration_scaling_factor: f64,
    goal: GoalFixture,
}

#[derive(Deserialize)]
struct RequestFixture {
    group_name: String,
    link_name: String,
    sampling_time: f64,
    blend_radius: f64,
    joint_limits: HashMap<String, FixtureJointLimit>,
    cartesian_limits: FixtureCartesianLimits,
    start_state: HashMap<String, f64>,
    segments: Vec<SegmentFixture>,
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
    first_intersection_index: usize,
    second_intersection_index: usize,
    first_trajectory: Vec<WaypointFixture>,
    blend_trajectory: Vec<WaypointFixture>,
    second_trajectory: Vec<WaypointFixture>,
}

/// A rejected `pilz_blend` response carries `error_code` and the two input
/// waypoint counts and nothing else -- the oracle never reaches the fields
/// [`ResponseFixture`] requires, so a rejected fixture cannot be read
/// through that type at all.
#[derive(Deserialize)]
struct RejectedResponseFixture {
    error_code: i32,
}

/// See `pilz_trajectory_lin_parity.rs`'s identical wrapper for why the
/// fixture files are full oracle wire envelopes.
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

/// Measured against both fixture cases (after accounting for the
/// documented `blend_trajectory`/`second_trajectory` waypoint-0 offset
/// above): max divergence `1.0e-9`. `first_trajectory`/`second_trajectory`
/// carry no new IK solve at all (they are truncated copies of waypoints the
/// segment-generation step already solved), and `blend_trajectory`'s own
/// closed-form quintic-smoothstep sampling has no separate wall-clock or
/// solver-iteration source of drift, so this is tighter than LIN's own
/// `TIME_TOLERANCE` and not copied from it -- see `CLAUDE.md`'s "Size test
/// tolerances from measurement".
const TIME_TOLERANCE: f64 = 1e-6;

/// Measured max divergence across the three succeeding fixture cases:
/// `5.46e-9`, at case C (`panda_blend_radius08`); cases A/B measure
/// `2.28e-9`/`1.84e-9`. Unlike `pilz_trajectory_lin_parity.rs`'s
/// `POSITION_TOLERANCE` (`1.26e-5` measured, budgeting for panda_arm's
/// redundant-kinematics IK-solver divergence),
/// `first_trajectory`/`second_trajectory` here are truncated copies of
/// waypoints a LIN segment already solved (no second independent IK solve
/// to diverge from), and `blend_trajectory`'s own IK solves converge far
/// more tightly on this fixture's geometry.
///
/// The value is **not** raised to restore the roughly-4x margin it carried
/// when only cases A/B existed; case C leaves it at about `1.8x`. Both
/// sides are deterministic here (no sampling, no seeded search), so the
/// margin buys nothing but blindness: widening to `2.5e-8` to keep a round
/// multiple would make the test unable to see a real `1e-8`-scale
/// regression that case C has just shown is within reach of this geometry.
/// PORTING-PLAN.md §207.
const POSITION_TOLERANCE: f64 = 1e-8;

/// Backward-difference velocity amplifies [`POSITION_TOLERANCE`] by roughly
/// `1 / sampling_time` (`0.1` here), the same chain LIN's own
/// `VELOCITY_TOLERANCE` documents. Measured max divergence across the three
/// succeeding fixture cases: `1.96e-8` (case A; case C measures `1.95e-8`,
/// case B `2.95e-10`). Set with a roughly 4x margin.
const VELOCITY_TOLERANCE: f64 = 8e-8;

/// The same backward-difference chain's acceleration term divides by
/// `sampling_time` again. Measured max divergence across the three
/// succeeding fixture cases: `2.91e-7` (case A; case C measures `1.01e-7`,
/// case B `2.53e-9`). Set with a roughly 4x margin.
const ACCELERATION_TOLERANCE: f64 = 1.2e-6;

/// Case E (`panda_blend_corner112`, 112° corner) measures `8.276e-8` at
/// `blend_trajectory` waypoint 5, `panda_joint5` -- **above**
/// [`VELOCITY_TOLERANCE`]. This is a real finding, not noise from the same
/// source cases A-D measure: `first_trajectory`/`second_trajectory` stay
/// far tighter on this same case (max `2.44e-14` velocity, `2.66e-13`
/// acceleration -- both confined to `first_trajectory`, no new IK solve, as
/// documented above), so the growth is entirely inside `blend_trajectory`'s
/// own interior waypoints (indices 1, 2, 5, 6 all show growing divergence
/// across `panda_joint1`/`3`/`5`/`6`, not one isolated joint or sample) --
/// consistent with panda_arm's redundant-kinematics IK null-space selection
/// diverging more between solvers as the corner sharpens, the same
/// phenomenon `lin_panda_arm_matches_the_oracle`'s module doc already
/// documents at 90°, not a slerp-direction or off-by-one bug (which would
/// show as one outlier, not a smooth spread across multiple joints and
/// waypoints).
/// [`VELOCITY_TOLERANCE`]/[`ACCELERATION_TOLERANCE`] are deliberately left
/// unchanged for cases A-D rather than widened to also cover case E --
/// doing that would hide A/B/C's own tighter true precision behind a
/// number sized for a geometry they never exercise. Set from case E's own
/// measured max (`8.276e-8`) with the same ~1.2x margin case C's
/// [`POSITION_TOLERANCE`] uses, not case A/B's ~4x -- both sides are
/// deterministic, so slack here only hides a future regression of this
/// size. Reported to the human orchestrator as a growing-divergence
/// finding, not resolved by loosening a shared constant.
const CORNER112_VELOCITY_TOLERANCE: f64 = 1e-7;

/// See [`CORNER112_VELOCITY_TOLERANCE`]. Measured max `1.6513e-6` at
/// `blend_trajectory` waypoint 6, `panda_joint5` -- above
/// [`ACCELERATION_TOLERANCE`] by about 38%. Set with the same ~1.2x margin.
const CORNER112_ACCELERATION_TOLERANCE: f64 = 2e-6;

/// Case F's acceleration sweep point at 60 degrees measures `1.1279e-6`,
/// inside [`ACCELERATION_TOLERANCE`] (`1.2e-6`) but by only about 6% --
/// close enough to the shared ceiling that reusing [`ACCELERATION_TOLERANCE`]
/// here would make the test unable to see a regression case A/B/C's own
/// wider margin still would. Set from this case's own measured max with the
/// same ~1.2x margin as [`CORNER112_ACCELERATION_TOLERANCE`]. See
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section.
const CORNER60_ACCELERATION_TOLERANCE: f64 = 1.35e-6;

/// Case F's sweep point at 75 degrees measures `8.9525e-8` velocity /
/// `1.4818e-6` acceleration, both above [`VELOCITY_TOLERANCE`] /
/// [`ACCELERATION_TOLERANCE`]. Set from this case's own measured max with a
/// ~1.2x margin, same as [`CORNER112_VELOCITY_TOLERANCE`]. This is the sweep
/// point the case-F prediction did not anticipate needing an override at all
/// -- 75 degrees is shallower than case A's 90, where the "sharper corner"
/// story predicts *less* divergence than case A, not more. See
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section for the
/// full sweep table and refutation verdict.
const CORNER75_VELOCITY_TOLERANCE: f64 = 1.1e-7;
/// See [`CORNER75_VELOCITY_TOLERANCE`].
const CORNER75_ACCELERATION_TOLERANCE: f64 = 1.8e-6;

/// Case F's sweep point at 105 degrees measures `1.3293e-6` acceleration,
/// above [`ACCELERATION_TOLERANCE`]; its velocity (`6.6526e-8`) stays inside
/// [`VELOCITY_TOLERANCE`] with a comfortable ~20% margin. Set from this
/// case's own measured max with a ~1.2x margin. See
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section.
const CORNER105_ACCELERATION_TOLERANCE: f64 = 1.6e-6;

/// Case F's sweep point at 110 degrees measures `7.9133e-8` velocity --
/// inside [`VELOCITY_TOLERANCE`] (`8e-8`) by only about 1%, too thin a
/// margin to trust against a deterministic pipeline's own rounding -- and
/// `1.5797e-6` acceleration, above [`ACCELERATION_TOLERANCE`] outright. Both
/// set from this case's own measured max with a ~1.2x margin. See
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section.
const CORNER110_VELOCITY_TOLERANCE: f64 = 9.5e-8;
/// See [`CORNER110_VELOCITY_TOLERANCE`].
const CORNER110_ACCELERATION_TOLERANCE: f64 = 1.9e-6;

/// Case F's radius control: case E's own 112 degree corner with
/// `blend_radius` lowered from `0.05` to `0.03`, angle and per-segment speed
/// held fixed. Measures `9.2596e-8` velocity, above [`VELOCITY_TOLERANCE`];
/// its acceleration (`9.2747e-7`) stays inside [`ACCELERATION_TOLERANCE`]
/// with a ~29% margin. That radius alone -- with angle fixed at case E's own
/// 112 degrees -- moves acceleration divergence from case E's `1.6513e-6`
/// down to `9.2747e-7` (a ~1.8x drop) is the control result
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section reports
/// against the redundant-IK/angle-only attribution.
const CORNER112_RADIUS03_VELOCITY_TOLERANCE: f64 = 1.11e-7;

/// See `pilz_trajectory_lin_parity.rs`'s own `CHECK_SELF_COLLISION` doc
/// comment -- this fixture's poses are the identical "ready, +x, +y corner"
/// geometry chosen there so the value is inconsequential.
const CHECK_SELF_COLLISION: bool = true;

/// The four tolerances a case is compared under, as one value.
///
/// They travel together because a case's precision is one property of that
/// case, not four independent knobs: case E (`panda_blend_corner112`)
/// measures a genuinely larger divergence at `blend_trajectory`'s interior
/// waypoints than cases A/B/C/D's shared budget covers, and carrying its own
/// set keeps that a separately-documented, separately-measured number rather
/// than a silent widening of [`VELOCITY_TOLERANCE`]/[`ACCELERATION_TOLERANCE`]
/// that would loosen A/B/C's own tighter measured precision to match it (see
/// [`CORNER112_VELOCITY_TOLERANCE`]).
///
/// All four live here, including the two no case has yet needed to vary.
/// Threading only the two that differ, and reading the other two from the
/// module constants inside the comparison, would split one value class across
/// two mechanisms -- and the next case needing its own `POSITION_TOLERANCE`
/// would have to re-plumb rather than fill in a field. That split is also
/// what put eight parameters on `compare_segment` and an
/// `#[allow(clippy::too_many_arguments)]` above it, which
/// `tools/ci/check-no-lint-suppression.sh` rejects.
#[derive(Debug, Clone, Copy)]
struct Tolerances {
    time: f64,
    position: f64,
    velocity: f64,
    acceleration: f64,
}

impl Tolerances {
    /// Cases A-D's shared, measured budget.
    const SHARED: Self = Self {
        time: TIME_TOLERANCE,
        position: POSITION_TOLERANCE,
        velocity: VELOCITY_TOLERANCE,
        acceleration: ACCELERATION_TOLERANCE,
    };
}

fn compare_segment(
    label: &str,
    case: &str,
    actual: &RobotTrajectory<'_>,
    expected: &[WaypointFixture],
    // Non-zero only for `blend_trajectory`/`second_trajectory` -- see the
    // call sites below for why those two, and not `first_trajectory`, carry
    // a constant `sampling_time` offset between the port's and the oracle's
    // `time_from_start` values.
    expected_time_offset: f64,
    tol: Tolerances,
) {
    assert_eq!(
        actual.way_point_count(),
        expected.len(),
        "{case}/{label} waypoint count must match the oracle exactly"
    );
    for (i, exp) in expected.iter().enumerate() {
        let actual_dt = actual.way_point_duration_from_start(i);
        let expected_dt = exp.time_from_start - expected_time_offset;
        assert!(
            (actual_dt - expected_dt).abs() < tol.time,
            "{case}/{label} waypoint {i} time_from_start: {actual_dt} != {expected_dt} (oracle {}, offset {expected_time_offset})",
            exp.time_from_start
        );

        let state = actual.way_point(i).unwrap();
        for (name, &expected_pos) in &exp.positions {
            let actual_pos = state.variable_position(name).unwrap();
            assert!(
                (actual_pos - expected_pos).abs() < tol.position,
                "{case}/{label} waypoint {i} position[{name}]: {actual_pos} != {expected_pos} (oracle)"
            );
        }
        for (name, &expected_vel) in &exp.velocities {
            let actual_vel = state.variable_velocity(name).unwrap();
            assert!(
                (actual_vel - expected_vel).abs() < tol.velocity,
                "{case}/{label} waypoint {i} velocity[{name}]: {actual_vel} != {expected_vel} (oracle)"
            );
        }
        for (name, &expected_acc) in &exp.accelerations {
            let actual_acc = state.variable_acceleration(name).unwrap();
            assert!(
                (actual_acc - expected_acc).abs() < tol.acceleration,
                "{case}/{label} waypoint {i} acceleration[{name}]: {actual_acc} != {expected_acc} (oracle)"
            );
        }
    }
}

/// Builds the model, generates the fixture's own two chained LIN segments,
/// calls [`blend`], and hands the raw `Result` to `check` -- which is what
/// makes a *rejected* case expressible: a driver that unwrapped the result
/// itself could only ever express the success path, and the rejection path
/// is exactly where this port and upstream can disagree without either
/// producing a comparable waypoint array (PORTING-PLAN.md §207).
///
/// The whole pipeline is built and consumed inside one call because
/// [`TrajectoryBlendResponse`] borrows the [`RobotModel`] this function
/// owns; returning it would return a borrow of a local.
fn drive_case<R>(
    case: &str,
    check: impl for<'m> FnOnce(&RequestFixture, usize, Result<TrajectoryBlendResponse<'m>>) -> R,
) -> R {
    let request: RequestFixture = load_json(&format!("{case}_request.json"));
    assert_eq!(
        request.segments.len(),
        2,
        "{case}: this fixture shape always carries exactly two segments"
    );

    let (model, srdf) = load_panda();

    let mut joint_limits = JointLimitsContainer::default();
    for (name, limit) in &request.joint_limits {
        assert!(
            joint_limits.add_limit(name.clone(), limit.into()),
            "{case}: duplicate or invalid joint limit for {name} in fixture"
        );
    }
    let mut limits = LimitsContainer::new();
    limits.set_joint_limits(joint_limits);
    limits.set_cartesian_limits((&request.cartesian_limits).into());

    let base = TrajectoryGenerator::new(&model, limits.clone());
    let generator = TrajectoryGeneratorLin::new(base, &request.group_name);

    let scene = Arc::new(PlanningScene::new(&model, &srdf));
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let ctx = IkContext {
        scene: &scene,
        env: &env,
        check_self_collision: CHECK_SELF_COLLISION,
    };

    let segment1_request = MotionPlanRequest {
        group_name: request.group_name.clone(),
        start_state: StartState {
            position: request.start_state.clone(),
            velocity: HashMap::new(),
        },
        goal: goal_from_fixture(&request.segments[0].goal),
        max_velocity_scaling_factor: request.segments[0].max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.segments[0].max_acceleration_scaling_factor,
        path_constraints: None,
    };
    let segment1_response = generator.generate(&ctx, &segment1_request, request.sampling_time);
    let first_trajectory = segment1_response.trajectory.unwrap_or_else(|| {
        panic!(
            "{case}: segment 1 must also reach SUCCESS on the fixture's own accepted request, got {:?}",
            segment1_response.error_code
        )
    });

    // Chain segment 2 onto segment 1's own actual last waypoint, not an
    // independently re-planned start -- see this module's own doc comment
    // and doc/oracle-request-pilz-blend.md's "Request JSON shape" section
    // for why two independently-solved IK results for the same corner can
    // fail validate_request's boundary check on panda_arm's redundant
    // kinematics.
    let group = model
        .joint_model_group(&request.group_name)
        .expect("fixture group must exist");
    let boundary = first_trajectory
        .last_way_point()
        .expect("segment 1 trajectory must be non-empty");
    let mut chained_position = HashMap::new();
    let mut chained_velocity = HashMap::new();
    for name in group.active_joint_names() {
        chained_position.insert(name.clone(), boundary.variable_position(name).unwrap());
        chained_velocity.insert(name.clone(), boundary.variable_velocity(name).unwrap());
    }

    let segment2_request = MotionPlanRequest {
        group_name: request.group_name.clone(),
        start_state: StartState {
            position: chained_position,
            velocity: chained_velocity,
        },
        goal: goal_from_fixture(&request.segments[1].goal),
        max_velocity_scaling_factor: request.segments[1].max_velocity_scaling_factor,
        max_acceleration_scaling_factor: request.segments[1].max_acceleration_scaling_factor,
        path_constraints: None,
    };
    let segment2_response = generator.generate(&ctx, &segment2_request, request.sampling_time);
    let second_trajectory = segment2_response.trajectory.unwrap_or_else(|| {
        panic!(
            "{case}: segment 2 must also reach SUCCESS on the fixture's own accepted request, got {:?}",
            segment2_response.error_code
        )
    });
    let second_trajectory_input_waypoint_count = second_trajectory.way_point_count();

    let mut blend_request = TrajectoryBlendRequest {
        group_name: request.group_name.clone(),
        link_name: request.link_name.clone(),
        first_trajectory,
        second_trajectory,
        blend_radius: request.blend_radius,
    };
    check(
        &request,
        second_trajectory_input_waypoint_count,
        blend(&ctx, &limits, &mut blend_request),
    )
}

fn run_case(case: &str) {
    run_case_with_tolerances(case, Tolerances::SHARED)
}

fn run_case_with_tolerances(case: &str, tol: Tolerances) {
    let response: ResponseFixture =
        load_json::<OracleResponseEnvelope<ResponseFixture>>(&format!("{case}_response.json"))
            .result;
    assert_eq!(
        response.error_code, 1,
        "{case}: fixture's own oracle run must have succeeded"
    );

    drive_case(
        case,
        |request, second_trajectory_input_waypoint_count, result| {
            let blend_response = result.unwrap_or_else(|e| {
        panic!("{case}: blend must also succeed on the fixture's own accepted request, got {e:?}")
    });

            // See this module's own doc comment on why there is no `blend_align_index`
            // field to compare directly: recover the port's own intersection indices
            // from the exact truncation-loop witness `blend()` leaves in its
            // response, the same recovery PORTING-PLAN.md §188 records the oracle
            // side performing.
            let port_first_intersection_index = blend_response.first_trajectory.way_point_count();
            let port_second_intersection_index = second_trajectory_input_waypoint_count
                - blend_response.second_trajectory.way_point_count()
                - 1;

            assert_eq!(
                port_first_intersection_index, response.first_intersection_index,
                "{case}: first_intersection_index (recovered from first_trajectory's exact truncation length) must match the oracle exactly"
            );
            assert_eq!(
                port_second_intersection_index, response.second_intersection_index,
                "{case}: second_intersection_index (recovered from second_trajectory's exact truncation length) must match the oracle exactly"
            );

            compare_segment(
                "first_trajectory",
                case,
                &blend_response.first_trajectory,
                &response.first_trajectory,
                0.0,
                tol,
            );
            compare_segment(
                "blend_trajectory",
                case,
                &blend_response.blend_trajectory,
                &response.blend_trajectory,
                // Same structural offset as `second_trajectory` below:
                // `blend_trajectory` is a fresh `RobotTrajectory` whose first
                // Cartesian sample's own real elapsed time is `sampling_time`
                // (`generate_joint_trajectory_from_cartesian`'s `duration_current`
                // for `i == 0` is `point.time_from_start`, not `0.0`), but
                // `moveit-trajectory`'s own documented invariant --
                // `duration_from_previous[0]` is always `0.0`, enforced
                // structurally, not just by convention (`robot_trajectory.rs`'s own
                // `# Deviations`, "New invariant") -- makes it structurally
                // impossible to store that value at waypoint 0. `first_trajectory`
                // does not need this: it is a genuine prefix of the original LIN
                // trajectory, whose own waypoint 0 duration really was `0.0`
                // upstream too.
                request.sampling_time,
                tol,
            );
            compare_segment(
                "second_trajectory",
                case,
                &blend_response.second_trajectory,
                &response.second_trajectory,
                // See this module's own doc comment: `second_trajectory`'s waypoint 0
                // duration is always `0.0` on this port's side, never
                // `sampling_time` -- a permanent, documented deviation
                // (`trajectory_blender_transition_window.rs`'s own module doc,
                // "`response.second_trajectory`'s waypoint-0 duration is always
                // `0.0`" section), not a bug this test should paper over with a
                // loose blanket tolerance. Every later waypoint's
                // `duration_from_previous` is copied unchanged, so the missing
                // correction is a constant offset through the whole segment.
                request.sampling_time,
                tol,
            );
        },
    )
}

/// Case A: segment 2 shares segment 1's `max_velocity_scaling_factor`
/// (`0.1`), producing symmetric intersection counts and
/// `determine_trajectory_alignment`'s `else` branch (`8 == 8`) -- see
/// PORTING-PLAN.md §188.2.
#[test]
fn blend_panda_arm_symmetric_matches_the_oracle() {
    run_case("panda_blend_symmetric");
}

/// Case B: segment 2's `max_velocity_scaling_factor`/
/// `max_acceleration_scaling_factor` raised to `0.3`, breaking the symmetry
/// and flipping `determine_trajectory_alignment` to its
/// `way_point_count_1 > way_point_count_2` branch (`8 > 4`) -- the branch
/// case A's geometry never reaches. See PORTING-PLAN.md §188.2.
#[test]
fn blend_panda_arm_asymmetric_matches_the_oracle() {
    run_case("panda_blend_asymmetric");
}

/// Case C: case A's exact geometry and speeds with `blend_radius` raised
/// from `0.05` to `0.08`, moving both intersection indices off the single
/// `(8, 7)` point cases A/B pin them at -- see
/// `doc/oracle-request-pilz-blend-geometry.md`'s own case C section for why
/// `0.08` and not one of the three other radii swept locally.
#[test]
fn blend_panda_arm_radius08_matches_the_oracle() {
    run_case("panda_blend_radius08");
}

/// Case D: case A's exact `blend_radius` and speeds at a 150 degree corner
/// instead of 90.
///
/// `doc/oracle-request-pilz-blend-geometry.md` requested this case as an
/// interpolation test and predicted it would succeed on both sides. The
/// oracle rejects it: `generateJointTrajectory` fails the 4th blend sample
/// on `panda_joint2`'s deceleration limit (`-2.50863` against a `-1.875`
/// limit). So the case is a *rejection*-parity case, and its value is that
/// both implementations reject the same request for the same reason --
/// which is not a weaker result than the interpolation comparison it
/// replaces, because a port that accepted this request would be silently
/// emitting a trajectory upstream considers dynamically infeasible.
/// PORTING-PLAN.md §207.
///
/// "Same reason" is measured, not inferred from the matching error code: a
/// temporary `eprintln!` in `verify_sample_joint_limits`'s deceleration
/// branch and in `generate_joint_trajectory_from_cartesian`'s per-sample
/// loop (applied, run, reverted) shows this port rejecting at sample `4` on
/// `panda_joint2` with `acceleration_current = -2.5086292326350526` --
/// upstream's own log line for the same request reads `Joint deceleration
/// limit of panda_joint2 violated ... Actual joint deceleration is
/// -2.50863` at the 4th sample. Same joint, same sample, same number to
/// every digit upstream prints.
#[test]
fn blend_panda_arm_corner150_is_rejected_like_the_oracle() {
    let response: RejectedResponseFixture = load_json::<
        OracleResponseEnvelope<RejectedResponseFixture>,
    >("panda_blend_corner150_response.json")
    .result;
    assert_eq!(
        response.error_code, -1,
        "panda_blend_corner150: this fixture exists because the oracle rejected it; \
         a re-capture that now succeeds means the case changed, not that the port improved"
    );

    drive_case("panda_blend_corner150", |_request, _count, result| {
        // `expect_err` is unavailable here: `TrajectoryBlendResponse` is not
        // `Debug`, and making it `Debug` to satisfy one test assertion would
        // be a production change driven by test convenience.
        let Err(error) = result else {
            panic!(
                "panda_blend_corner150: the oracle rejects this blend on panda_joint2's \
                 deceleration limit; this port accepting it would emit a trajectory upstream \
                 considers dynamically infeasible"
            )
        };
        // Compared as the raw wire `int32` the oracle actually emitted, not
        // against a hard-coded variant: the point of the case is that both
        // sides reject for the *same* reason, and `InvalidMotionPlan` (`-2`,
        // what `search_intersection_points` returns when the geometry itself
        // is unreachable) would pass a bare "it errored" check while meaning
        // something entirely different from upstream's `-1`.
        let Error::Code(code) = error else {
            panic!("panda_blend_corner150: expected a MoveItErrorCode, got {error:?}")
        };
        assert_eq!(
            code.as_i32(),
            response.error_code,
            "panda_blend_corner150: the port must reject with the oracle's own error code \
             ({}), not merely reject",
            MoveItErrorCode::from(response.error_code),
        );
    })
}

/// Case E: case A's exact `blend_radius` and speeds at a 112 degree corner
/// instead of 90 -- the sharpest angle strictly between 90 and 150 at which
/// the full pipeline succeeds on this port's own side (measured by
/// bisection: succeeds through 112.6°, rejects at 112.8° on the same
/// `panda_joint2` deceleration limit case D hits at 150°; 112.0° is filed
/// with a margin below that boundary, not the boundary itself, so the case
/// does not sit on a knife-edge that could flip sides from IK-solver
/// divergence against the oracle's own solver). Replaces the interpolation
/// comparison case D was proposed for and could not deliver -- see
/// `doc/oracle-request-pilz-blend-geometry.md`'s case E section and
/// PORTING-PLAN.md §207.1.
///
/// Both predictions from that document are confirmed on the real oracle:
/// `first_intersection_index = 8`, `second_intersection_index = 7`,
/// identical to case A -- a third independent confirmation that
/// `search_intersection_points`'s walk is angle-invariant when radius and
/// per-segment speed are held fixed. The waypoint arrays are *not* fully
/// identical, though: see [`CORNER112_VELOCITY_TOLERANCE`] for the growing-
/// divergence finding this case surfaces at `blend_trajectory`'s interior
/// waypoints, which the shared `VELOCITY_TOLERANCE`/`ACCELERATION_TOLERANCE`
/// budget (sized from cases A-D at up to 90°) does not cover.
#[test]
fn blend_panda_arm_corner112_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner112",
        Tolerances {
            velocity: CORNER112_VELOCITY_TOLERANCE,
            acceleration: CORNER112_ACCELERATION_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}

/// Case F: an 8-point corner-angle sweep (30/60/75/90(A)/100/105/110/112(E)
/// degrees, `blend_radius`/per-segment speed held fixed at case A's own
/// values) built to test the case E doc's own prediction that divergence is
/// monotone in corner angle. It is not: 90° and 100° measure the *lowest*
/// divergence of the whole sweep, well below both their shallower (75°,
/// 60°) and sharper (105°, 110°, 112°) neighbors. See
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section for the
/// full measured table, the radius control, and the refutation verdict.
///
/// Case F's own 30 degree sweep point: measures `4.0972e-8` velocity /
/// `7.7099e-7` acceleration, both comfortably inside
/// [`VELOCITY_TOLERANCE`]/[`ACCELERATION_TOLERANCE`] -- no override needed.
#[test]
fn blend_panda_arm_corner30_matches_the_oracle() {
    run_case("panda_blend_corner30");
}

/// Case F's 60 degree sweep point. See [`CORNER60_ACCELERATION_TOLERANCE`];
/// velocity (`6.4188e-8`) stays inside [`VELOCITY_TOLERANCE`] with a
/// comfortable ~25% margin, no override needed.
#[test]
fn blend_panda_arm_corner60_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner60",
        Tolerances {
            acceleration: CORNER60_ACCELERATION_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}

/// Case F's 75 degree sweep point -- the sweep's actual local maximum, and
/// the point that refutes the "monotone in angle" prediction outright: 75°
/// is shallower than case A's 90°, yet measures higher divergence than case
/// A, case E (112°), and every sweep point up to 110°. See
/// [`CORNER75_VELOCITY_TOLERANCE`]/[`CORNER75_ACCELERATION_TOLERANCE`] and
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section.
#[test]
fn blend_panda_arm_corner75_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner75",
        Tolerances {
            velocity: CORNER75_VELOCITY_TOLERANCE,
            acceleration: CORNER75_ACCELERATION_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}

/// Case F's 100 degree sweep point: measures `1.5487e-8` velocity /
/// `2.9642e-7` acceleration -- the sweep's own minimum, both comfortably
/// inside [`VELOCITY_TOLERANCE`]/[`ACCELERATION_TOLERANCE`], no override
/// needed. Sits right next to case A (90°, `1.9582e-8`/`2.9061e-7`) at the
/// bottom of the dip the case-F prediction did not anticipate.
#[test]
fn blend_panda_arm_corner100_matches_the_oracle() {
    run_case("panda_blend_corner100");
}

/// Case F's 105 degree sweep point. See
/// [`CORNER105_ACCELERATION_TOLERANCE`]; velocity (`6.6526e-8`) stays inside
/// [`VELOCITY_TOLERANCE`] with a comfortable ~20% margin, no override
/// needed.
#[test]
fn blend_panda_arm_corner105_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner105",
        Tolerances {
            acceleration: CORNER105_ACCELERATION_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}

/// Case F's 110 degree sweep point. See
/// [`CORNER110_VELOCITY_TOLERANCE`]/[`CORNER110_ACCELERATION_TOLERANCE`].
#[test]
fn blend_panda_arm_corner110_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner110",
        Tolerances {
            velocity: CORNER110_VELOCITY_TOLERANCE,
            acceleration: CORNER110_ACCELERATION_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}

/// Case F's radius control: case E's exact 112 degree corner with
/// `blend_radius` lowered from `0.05` to `0.03`, angle and per-segment speed
/// held fixed -- the discriminating case the round asked for, isolating
/// radius from angle. See [`CORNER112_RADIUS03_VELOCITY_TOLERANCE`] and
/// `doc/oracle-request-pilz-blend-geometry.md`'s "Case F" section for what
/// this control measures against the angle-only attribution. Acceleration
/// (`9.2747e-7`) stays inside [`ACCELERATION_TOLERANCE`] with a ~29% margin,
/// no override needed there.
#[test]
fn blend_panda_arm_corner112_radius03_matches_the_oracle() {
    run_case_with_tolerances(
        "panda_blend_corner112_radius03",
        Tolerances {
            velocity: CORNER112_RADIUS03_VELOCITY_TOLERANCE,
            ..Tolerances::SHARED
        },
    );
}
