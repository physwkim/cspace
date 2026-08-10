// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is Phase 8 benchmark infrastructure (PORTING-PLAN.md
// §5's "CHOMP/STOMP는 Phase 7과 같은 속성 기반 검증", §263), not a port.
// Upstream drives STOMP only through `StompPlanningContext`'s pluginlib
// entry point, never through a benchmark runner.

//! Runs this crate's [`plan`](cspace_planners::stomp::plan) over a `plan`-op
//! request JSON -- the exact format `cspace_planners::sbp`'s
//! `examples/plan_benchmark_problem_set` emits and
//! `cspace-planners/benches/sweep_baseline.sh` feeds to the oracle -- so
//! Phase 8's STOMP measurement runs the *identical* 500 problems the Phase 7
//! C++ OMPL RRTConnect baseline was measured on, not a re-sample.
//!
//! # Which baseline this measures against, and why
//!
//! Same reading, and the same reason, as
//! `cspace-planners/examples/chomp_benchmark_port.rs`'s own
//! "# Which baseline this measures against": §5's Phase 8 clause forwards to
//! Phase 7's three properties (PORTING-PLAN.md lines 705-708), whose
//! comparison side is **C++ OMPL RRTConnect**. The "no other option even in
//! principle" this file used to claim for STOMP no longer holds: the oracle
//! builds `stomp_moveit_planning_context.cpp` straight out of the pinned
//! moveit2 tree (`tools/moveit-oracle/CMakeLists.txt`'s `STOMP_MOVEIT_SRC`)
//! and answers a `stomp_plan` op, which
//! `tools/ci/measure-phase8-cpp-baseline.sh` drives. This binary keeps the
//! RRTConnect reading because §5 names that baseline;
//! `tools/ci/measure-phase8-optimizer-properties.sh` measures the
//! planner-against-its-own-upstream one. See PORTING-PLAN.md §263 for the
//! original assumption and the Phase 8 property section for what replaced it.
//!
//! # Usage
//!
//! `cargo run --release --example stomp_benchmark_port -p
//! cspace_planners::stomp -- <seed_base> [allowed_planning_time_secs]`, with a
//! `plan`-op request JSON on stdin.
//!
//! `seed_base` seeds the per-problem `ChaCha8Rng` STOMP's noisy rollouts are
//! drawn from; each problem uses `seed_base.wrapping_add(problem.id)`.
//! `allowed_planning_time_secs` defaults to `5.0` -- upstream's own
//! `MoveGroupInterface::allowed_planning_time_` initial value
//! (`move_group_interface.cpp:165`), which is what
//! `StompPlanningContext::solve` waits on before cancelling
//! (`stomp_moveit_planning_context.cpp:247-257`).
//!
//! Prints one NDJSON line per problem to stdout: `plan_benchmark_port`'s
//! own shape plus one attribution field,
//! `{"id", "solved", "length"?, "condition2_valid"?,
//! "invalid_waypoint_count"?, "condition2_valid_at_returned_waypoints"?,
//! "failure"?}`, plus `condition2_by_resolution` when the request carries
//! `condition2_resolutions` (see [`condition2_by_resolution`]).
//! See [`returned_waypoints`] for what the attribution field is
//! for and why it is not the condition-2 number.
//!
//! # Every loop here is bounded
//!
//! STOMP's own iteration loop is bounded by `num_iterations` (upstream
//! default 1000). That alone is not a *wall-clock* bound, so this binary
//! reproduces upstream's timeout watcher exactly: a second thread waits
//! `allowed_planning_time` on a channel and calls
//! [`CancelHandle::cancel`](cspace_stomp_core::CancelHandle::cancel) if the
//! plan has not finished by then, and the main thread signals that channel
//! (closing it) the instant [`plan`] returns, so the watcher never outlives
//! its problem. `failure` distinguishes the two ways STOMP can come back
//! empty, the same split upstream makes into `TIMED_OUT` vs
//! `PLANNING_FAILED`.
//!
//! # Where every non-obvious constant comes from
//!
//! Every [`StompConfiguration`] field below is upstream's declared default
//! from `moveit_planners/stomp/res/stomp_moveit.yaml`, not a value tuned for
//! this benchmark: `num_iterations = 1000`, `num_iterations_after_valid = 0`,
//! `num_rollouts = 15`, `max_rollouts = 25`, `num_timesteps = 40`,
//! `exponentiated_cost_sensitivity = 0.5`, `control_cost_weight = 0.1`,
//! `delta_t = 0.1`. `num_dimensions` and `initialization_method` are not set
//! here at all -- [`plan`] overwrites both (from the group's active joint
//! count, and to `LinearInterpolation`) exactly as
//! `StompPlanningContext::solve` does. The collision penalty is `1.0`,
//! upstream's own hardcoded argument at
//! `stomp_moveit_planning_context.cpp`'s `createStompTask`.
//!
//! There are no path constraints in any Phase 7/8 benchmark request, so this
//! binary uses `costs::getCollisionCostFunction` alone -- the same branch
//! `createStompTask` takes when `constraints.empty()`.
//!
//! # Condition 2's collision-check resolution, and the asymmetry it exposes
//!
//! Identical rule to `plan_benchmark_port`'s: the returned path is
//! re-interpolated at the request's own `motion_resolution` (0.01 for every
//! request `plan_benchmark_problem_set` emits) before
//! [`PlanningScene::is_path_valid`] sees it. One uniform rule for all three
//! planners, deliberately -- a per-planner resolution would make the three
//! condition-2 numbers incomparable.
//!
//! Note what that uniform rule means *here*, because it is not the same
//! relationship RRT-Connect had: STOMP's own validity check interpolates at
//! [`COL_CHECK_DISTANCE`](cspace_planners::stomp::cost_functions::COL_CHECK_DISTANCE)
//! `= 0.05` (upstream `cost_functions.hpp:59`), five times *coarser* than
//! `motion_resolution`. For RRT-Connect the two resolutions were equal, so
//! condition 2 was a cross-check between two independent code paths at one
//! resolution; for STOMP it is additionally a finer-resolution search, and a
//! condition-2 failure here can mean "STOMP's own 0.05 sampling stepped over
//! a collision", not necessarily "the port disagrees with itself". That is a
//! property of upstream's constant, and it is reported rather than papered
//! over by loosening the check.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::env;
use std::io::{self, Read};
use std::sync::Arc;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::thread;
use std::time::Duration;

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::test_support::isometry_from_row_major;
use cspace_planners::sbp::{CompoundValue, JointModelGroupSpace, StateSpace};
use cspace_planners::stomp::cost_functions::get_collision_cost_function;
use cspace_planners::stomp::planner::{PlanRequest, plan};
use cspace_planning::scene::PlanningScene;
use cspace_stomp_core::{CancelHandle, StompConfiguration, TrajectoryInitialization};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Upstream `createStompTask`'s own hardcoded collision penalty
/// (`stomp_moveit_planning_context.cpp`, the `1.0 /* collision penalty */`
/// argument).
const COLLISION_PENALTY: f64 = 1.0;

/// Upstream `MoveGroupInterface`'s initial `allowed_planning_time_`
/// (`move_group_interface.cpp:165`), the value
/// `StompPlanningContext::solve`'s timeout watcher waits on.
const DEFAULT_ALLOWED_PLANNING_TIME: f64 = 5.0;

/// Every field's value is `res/stomp_moveit.yaml`'s declared
/// `default_value`. `num_dimensions`/`initialization_method` are placeholders
/// [`plan`] overwrites -- see this file's own doc comment.
fn upstream_default_config() -> StompConfiguration {
    StompConfiguration {
        num_iterations: 1000,
        num_iterations_after_valid: 0,
        num_timesteps: 40,
        num_dimensions: 0,
        delta_t: 0.1,
        initialization_method: TrajectoryInitialization::LinearInterpolation,
        exponentiated_cost_sensitivity: 0.5,
        num_rollouts: 15,
        max_rollouts: 25,
        control_cost_weight: 0.1,
    }
}

/// The `moveit_resources_panda_description` package committed under
/// `fixtures/meshes/` -- same pattern as
/// `cspace-planners/examples/plan_benchmark_port.rs`, duplicated rather
/// than shared because a cargo example cannot import another crate's
/// example.
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn load_panda() -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let urdf_xml = std::fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
    let model =
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
            .expect("fixture model must build");
    (model, srdf)
}

/// Reads a joint-name -> value map (the request JSON's
/// `problems[].start`/`.goal` shape) into a fresh [`RobotState`].
fn joint_map_to_robot_state<'m>(
    model: &'m RobotModel,
    map: &BTreeMap<String, f64>,
) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in map {
        state
            .set_variable_position(name, *value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    state
}

/// Densifies `path` at `resolution` spacing -- byte-identical rule to
/// `plan_benchmark_port`'s own `densify`, see this file's `# Condition 2's
/// collision-check resolution`.
fn densify<'m>(
    space: &JointModelGroupSpace,
    model: &'m RobotModel,
    path: &[Vec<CompoundValue>],
    resolution: f64,
) -> Vec<RobotState<'m>> {
    let mut template = RobotState::new(model);
    template.set_to_default_values();
    let to_robot_state = |state: &Vec<CompoundValue>| {
        let mut rs = template.clone();
        space.write_robot_state(state, &mut rs);
        rs
    };

    let mut out = vec![to_robot_state(&path[0])];
    for pair in path.windows(2) {
        let (from, to) = (&pair[0], &pair[1]);
        let dist = space.distance(from, to);
        let steps = ((dist / resolution).ceil() as u64).max(1);
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            out.push(to_robot_state(&space.interpolate(from, to, t)));
        }
    }
    out
}

/// Condition 2 re-evaluated at each of `resolutions`, as
/// `[{"resolution", "invalid_count", "densified_waypoint_count", "valid"}]`.
///
/// The plan itself does not depend on the densification resolution -- it is
/// read only by [`densify`], after `plan` has returned -- so this list is
/// several verdicts about one path, not several runs. On this side that is
/// the difference between a measurement and none at all: one 500-problem
/// STOMP sweep costs about three hours, so a per-resolution re-run of the
/// planner is not affordable.
///
/// Emitted only when the request carries `condition2_resolutions`; without
/// that field this function is never called and the record shape is
/// unchanged.
fn condition2_by_resolution<'m>(
    space: &JointModelGroupSpace,
    model: &'m RobotModel,
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    path: &[Vec<CompoundValue>],
    resolutions: &[f64],
) -> Vec<serde_json::Value> {
    resolutions
        .iter()
        .map(|resolution| {
            let dense = densify(space, model, path, *resolution);
            let validity =
                scene.is_path_valid(env, &CollisionRequest::default(), &dense, None, &[]);
            serde_json::json!({
                "resolution": resolution,
                "invalid_count": validity.invalid_waypoints.len(),
                "densified_waypoint_count": dense.len(),
                "valid": validity.valid,
            })
        })
        .collect()
}

/// The request's optional `condition2_resolutions`, the operating-point grid
/// [`condition2_by_resolution`] walks.
///
/// Rejected rather than defaulted when malformed: a silently dropped grid
/// would make a sweep report one resolution's verdict under every
/// resolution's name.
fn parse_condition2_resolutions(request: &serde_json::Value) -> Vec<f64> {
    match request.get("condition2_resolutions") {
        None | Some(serde_json::Value::Null) => Vec::new(),
        Some(value) => value
            .as_array()
            .expect("request.condition2_resolutions must be an array")
            .iter()
            .map(|entry| {
                let resolution = entry
                    .as_f64()
                    .expect("request.condition2_resolutions entries must be numbers");
                assert!(
                    resolution > 0.0,
                    "request.condition2_resolutions entries must be positive, got {resolution}"
                );
                resolution
            })
            .collect(),
    }
}

/// `path` as [`RobotState`]s with no interpolation at all -- exactly the
/// waypoints the planner returned.
///
/// Feeding these to [`PlanningScene::is_path_valid`] reproduces what
/// upstream's own `isPathValid(trajectory, group)` would report for this
/// planner's output, and nothing finer. Reported alongside the densified
/// verdict as `condition2_valid_at_returned_waypoints`, purely to attribute
/// a condition-2 failure: a path that is valid here and invalid after
/// densification failed *between* the planner's own waypoints, at a
/// resolution neither the planner nor upstream ever looks at. The official
/// condition-2 number stays the densified one.
fn returned_waypoints<'m>(
    space: &JointModelGroupSpace,
    model: &'m RobotModel,
    path: &[Vec<CompoundValue>],
) -> Vec<RobotState<'m>> {
    let mut template = RobotState::new(model);
    template.set_to_default_values();
    path.iter()
        .map(|state| {
            let mut rs = template.clone();
            space.write_robot_state(state, &mut rs);
            rs
        })
        .collect()
}

/// What one [`plan`] call came back with, already lifted out of every borrow
/// the call held -- see [`run_stomp`]'s own doc for why the waypoints leave
/// as plain `Vec<f64>` rows.
enum Outcome {
    Solved(Vec<Vec<f64>>),
    TimedOut,
    Failed,
}

/// Everything one sweep holds fixed across all 250 problems of a config: the
/// robot, the obstacle world, the group being planned for, and the timeout
/// the watcher arms. Split out from [`Sweep::run_stomp`]'s per-problem
/// arguments (start, goal, seed) because those are the only three that vary.
struct Sweep<'a> {
    model: &'a RobotModel,
    srdf: &'a SrdfModel,
    env: &'a ParryCollisionEnv,
    group_name: &'a str,
    allowed_planning_time: Duration,
}

impl Sweep<'_> {
    /// One STOMP planning attempt, with upstream's timeout watcher around it.
    ///
    /// The scene the cost function writes into is borrowed mutably for the
    /// whole [`plan`] call, so this returns the trajectory as plain
    /// joint-value rows rather than a `RobotTrajectory`: that ends every
    /// borrow at the function boundary, and lets the caller reuse its own
    /// scene for the condition-2 check without a second `RefCell` dance.
    fn run_stomp(
        &self,
        start_state: &RobotState<'_>,
        goal_state: &RobotState<'_>,
        seed: u64,
    ) -> Outcome {
        let Sweep {
            model,
            srdf,
            env,
            group_name,
            allowed_planning_time,
        } = *self;
        let group = model
            .joint_model_group(group_name)
            .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));

        let mut cost_scene = PlanningScene::new(model, srdf);
        cost_scene.set_current_state(start_state.clone());

        let cell = RefCell::new(&mut cost_scene);
        let cost_fn = get_collision_cost_function(&cell, env, group, COLLISION_PENALTY)
            .unwrap_or_else(|e| panic!("get_collision_cost_function: {e}"));

        // Upstream's `std::async` + `condition_variable` timeout watcher
        // (`stomp_moveit_planning_context.cpp:247-257`), as a thread waiting on a
        // channel: `Timeout` means the watcher fired, a closed channel means
        // `plan` returned first and the watcher must *not* cancel anything.
        let cancel_handle = CancelHandle::new();
        let watcher_handle = cancel_handle.clone();
        let (finished_tx, finished_rx) = mpsc::channel::<()>();
        let watcher =
            thread::spawn(
                move || match finished_rx.recv_timeout(allowed_planning_time) {
                    Err(RecvTimeoutError::Timeout) => {
                        watcher_handle.cancel();
                        true
                    }
                    Ok(()) | Err(RecvTimeoutError::Disconnected) => false,
                },
            );

        let result = plan(
            upstream_default_config(),
            cost_fn,
            PlanRequest {
                start_state,
                goal_state,
                group,
                input_trajectory: None,
            },
            ChaCha8Rng::seed_from_u64(seed),
            cancel_handle,
        );

        drop(finished_tx);
        let timed_out = watcher.join().expect("timeout watcher must not panic");

        match result {
            Ok(Some(trajectory)) => {
                // `UnparameterizedTrajectory` exposes only its waypoint count and
                // this conversion; `delta_t` is irrelevant to the positions read
                // out below, and matching the config's own value keeps the one
                // number that reaches a `RobotTrajectory` here consistent with
                // what `StompPlanningContext` would produce.
                let trajectory = trajectory
                    .into_uniformly_timed(upstream_default_config().delta_t)
                    .unwrap_or_else(|e| panic!("into_uniformly_timed: {e}"));
                let rows = (0..trajectory.way_point_count())
                    .map(|i| {
                        let state = trajectory
                            .way_point(i)
                            .expect("index below way_point_count");
                        group
                            .active_joint_names()
                            .iter()
                            .map(|name| {
                                state
                                    .variable_position(name)
                                    .unwrap_or_else(|e| panic!("variable_position({name}): {e}"))
                            })
                            .collect()
                    })
                    .collect();
                Outcome::Solved(rows)
            }
            Ok(None) if timed_out => Outcome::TimedOut,
            Ok(None) => Outcome::Failed,
            Err(e) => panic!("stomp::plan returned a hard error: {e}"),
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let seed_base: u64 = args
        .get(1)
        .unwrap_or_else(|| panic!("usage: <seed_base> [allowed_planning_time_secs]; got {args:?}"))
        .parse()
        .expect("seed_base must be a u64");
    let allowed_planning_time = Duration::from_secs_f64(
        args.get(2)
            .map(|s| {
                s.parse::<f64>()
                    .expect("allowed_planning_time_secs must be a number")
            })
            .unwrap_or(DEFAULT_ALLOWED_PLANNING_TIME),
    );

    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .expect("stdin must contain a plan-op request JSON");
    let request: serde_json::Value =
        serde_json::from_str(&input).expect("stdin must be valid JSON");

    let group_name = request["group"]
        .as_str()
        .expect("request.group must be a string")
        .to_string();
    let resolution = request["motion_resolution"]
        .as_f64()
        .expect("request.motion_resolution must be a number");
    let condition2_resolutions = parse_condition2_resolutions(&request);

    let (model, srdf) = load_panda();
    let space = JointModelGroupSpace::new(&model, &group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));
    let group = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));
    let active_joint_names: Vec<String> = group.active_joint_names().to_vec();

    let mut world = World::new();
    for object in request["objects"]
        .as_array()
        .expect("request.objects must be an array")
    {
        let id = object["id"].as_str().expect("object.id must be a string");
        let size = object["shape"]["size"]
            .as_array()
            .expect("object.shape.size must be an array");
        let (sx, sy, sz) = (
            size[0].as_f64().unwrap(),
            size[1].as_f64().unwrap(),
            size[2].as_f64().unwrap(),
        );
        let pose_flat: [f64; 16] = object["pose"]
            .as_array()
            .expect("object.pose must be an array")
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect::<Vec<f64>>()
            .try_into()
            .unwrap_or_else(|v: Vec<f64>| {
                panic!("object.pose must have 16 elements, got {}", v.len())
            });
        world.add_shape(
            id,
            Arc::new(Shape::Cuboid(
                Cuboid::new(sx, sy, sz).unwrap_or_else(|e| panic!("Cuboid::new: {e}")),
            )),
            isometry_from_row_major(&pose_flat),
        );
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let sweep = Sweep {
        model: &model,
        srdf: &srdf,
        env: &env,
        group_name: &group_name,
        allowed_planning_time,
    };

    let mut check_scene = PlanningScene::new(&model, &srdf);
    let mut solved_count = 0usize;
    let mut total = 0usize;

    for problem in request["problems"]
        .as_array()
        .expect("request.problems must be an array")
    {
        total += 1;
        let id = problem["id"].as_u64().expect("problem.id must be a number");
        let start_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["start"].clone()).expect("problem.start");
        let goal_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["goal"].clone()).expect("problem.goal");

        let start_state = joint_map_to_robot_state(&model, &start_map);
        let goal_state = joint_map_to_robot_state(&model, &goal_map);

        let outcome = sweep.run_stomp(&start_state, &goal_state, seed_base.wrapping_add(id));

        let line = match outcome {
            Outcome::Solved(rows) => {
                solved_count += 1;
                let path: Vec<Vec<CompoundValue>> = rows
                    .iter()
                    .map(|row| {
                        let mut state = start_state.clone();
                        for (name, value) in active_joint_names.iter().zip(row) {
                            state
                                .set_variable_position(name, *value)
                                .expect("group joint names come from this model");
                        }
                        space.read_robot_state(&state)
                    })
                    .collect();
                let length: f64 = path
                    .windows(2)
                    .map(|pair| space.distance(&pair[0], &pair[1]))
                    .sum();

                let raw = returned_waypoints(&space, &model, &path);
                let raw_validity =
                    check_scene.is_path_valid(&env, &CollisionRequest::default(), &raw, None, &[]);
                let dense = densify(&space, &model, &path, resolution);
                let validity = check_scene.is_path_valid(
                    &env,
                    &CollisionRequest::default(),
                    &dense,
                    None,
                    &[],
                );

                let mut record = serde_json::json!({
                    "id": id,
                    "solved": true,
                    "length": length,
                    "condition2_valid": validity.valid,
                    "invalid_waypoint_count": validity.invalid_waypoints.len(),
                    "condition2_valid_at_returned_waypoints": raw_validity.valid,
                });
                if !condition2_resolutions.is_empty() {
                    record["condition2_by_resolution"] =
                        serde_json::Value::Array(condition2_by_resolution(
                            &space,
                            &model,
                            &mut check_scene,
                            &env,
                            &path,
                            &condition2_resolutions,
                        ));
                }
                record
            }
            Outcome::TimedOut => serde_json::json!({
                "id": id,
                "solved": false,
                "failure": "TIMED_OUT",
            }),
            Outcome::Failed => serde_json::json!({
                "id": id,
                "solved": false,
                "failure": "PLANNING_FAILED",
            }),
        };
        println!("{line}");
    }

    eprintln!("solved={solved_count}/{total}");
}
