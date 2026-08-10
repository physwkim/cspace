// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ad hoc verification, not benchmark infrastructure and not a port: answers
// one question a decision is resting on, by calling the port's own
// collision predicate a second time in a place nothing else calls it.

//! A claim under review says CHOMP's FINAL trajectory on two `cage`
//! benchmark problems (Phase 8's `panda cage 250 900002` set, ids 33 and
//! 83 -- the config/count/seed `tools/ci/verify-phase8-benchmark.sh` pins,
//! `PORT_SEED_BASE=700001`, non-binding clock) is collision-free, evidenced
//! by a *derived* per-waypoint minimum clearance distance. This binary does
//! not recompute that derivation. It calls the *predicate* CHOMP's own
//! optimizer loop uses to decide the question for itself
//! (`ChompOptimizer::optimize`, `optimizer.rs:1934-1935`:
//! `mesh_to_mesh_collision_free(&self.start_state, &self.best_group_trajectory)`)
//! a second time, directly, on
//! [`ChompSolution::trajectory`](cspace_planners_chomp::ChompSolution::trajectory)
//! -- the value [`solve_with_trace`] actually hands back to a caller once
//! the loop exits.
//!
//! # Why this is the same predicate and not a second implementation
//!
//! The closure passed to `solve_with_trace` below is byte-for-byte
//! `chomp_benchmark_port.rs`'s own `mesh_to_mesh` (same
//! `PlanningScene::is_path_valid` call, same `ParryCollisionEnv`, same
//! `CollisionRequest::default()`, same `None`/`&[]` constraint arguments) --
//! duplicated rather than imported because a cargo example cannot import
//! another crate's example (`chomp_benchmark_port.rs`'s own precedent for
//! `fixture_mesh_search_paths`). `solve_with_trace` is called completely
//! unmodified; nothing about the optimizer, its parameters, or its RNG seed
//! is touched. The one addition is a second call to the exact same
//! predicate logic, made *after* `solve_with_trace` returns, against the
//! trajectory it returned -- which is the one check neither
//! `ChompOptimizer::optimize` (only checks `best_group_trajectory`
//! periodically, every 10th iteration, and never again after the loop
//! exits -- see `optimizer.rs:1934-1935` and its post-loop copy-back at
//! `:1985-1993`) nor `chomp_benchmark_port.rs` (reads `condition2_valid`,
//! a *different*, coarser-resolution mesh check with a *different*
//! collision request, not this one) ever performs.
//!
//! `solution.trajectory`'s waypoints are, by construction, the physical
//! joint configurations of `best_group_trajectory` at the moment the loop
//! exited: `ChompOptimizer::optimize`'s last three statements copy
//! `best_group_trajectory` row-by-row into `self.group_trajectory`, then
//! `full_trajectory.update_from_group_trajectory(&self.group_trajectory)`
//! writes that into the outer, unpadded `ChompTrajectory` -- and
//! `solve_inner` (`planner.rs`) builds `ChompSolution::trajectory` directly
//! from that same outer trajectory's points, with no further mutation.
//! `best_group_trajectory` carries `DIFF_RULE_LENGTH` extra padding rows on
//! each end that `update_from_group_trajectory` never writes back (see
//! `build_seed_trajectory`'s doc: rows outside `[start_index, end_index]`
//! are untouched), but those padding rows are copies of the fixed start/
//! goal rows already present in `solution.trajectory`'s own first/last
//! waypoint -- so checking `solution.trajectory`'s waypoints checks every
//! physically distinct configuration `best_group_trajectory` held.
//!
//! # Usage
//!
//! `cargo run --example verify_final_trajectory_predicate -p
//! cspace-planners-chomp`, with the `panda cage 250 900002` request JSON
//! (`plan_benchmark_problem_set cage 250 900002`) on stdin. Reports one line
//! per target id; ids not present in the input are reported missing rather
//! than silently skipped.

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::sync::Arc;

use cspace_collision::distance_field::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GridGeometry,
    PropagationDistanceField, add_link_body_decompositions,
};
use cspace_collision::{AllowedCollisionMatrix, CollisionRequest, LinkPaddingScale};
use cspace_collision::{ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::test_support::isometry_from_row_major;
use cspace_planners_chomp::optimizer::ChompCollisionContext;
use cspace_planners_chomp::{
    ChompExit, ChompGoal, ChompParameters, ChompRequest, GoalJointConstraint, solve_with_trace,
};
use cspace_scene::PlanningScene;
use nalgebra::DMatrix;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// The two problems under review. Positional, not configurable from the
/// command line: this binary answers a specific question about specific
/// ids, not a general-purpose subset runner.
const TARGET_IDS: [u64; 2] = [33, 83];

/// `tools/ci/verify-phase8-benchmark.sh`'s `PORT_SEED_BASE`. Each problem's
/// RNG is seeded from `seed_base.wrapping_add(id)`, matching
/// `chomp_benchmark_port.rs`.
const PORT_SEED_BASE: u64 = 700_001;

/// `tools/ci/verify-phase8-benchmark.sh`'s `NO_CLOCK_BOUND`: large enough
/// that `ChompParameters::planning_time_limit` never binds, so the run
/// depends only on `ChompParameters::max_iterations` (upstream's default,
/// 50) and is reproducible regardless of machine load.
const NO_CLOCK_BOUND: f64 = 1e9;

// `DISTANCE_FIELD_RESOLUTION`, `distance_field_config`,
// `fixture_mesh_search_paths`, `load_panda`, `Obstacle`, `parse_obstacles`,
// `joint_map_to_robot_state`, `build_collision_world` and
// `mesh_to_mesh_collision_free_check` all live in `support/chomp_bench_world.rs`
// now -- shared with `chomp_benchmark_port.rs` so this file's duplicate of
// its construction cannot silently diverge from the binary it is verifying.
// See that file's own header for why `include!` and not a `src/` module or
// a second copy.
include!("support/chomp_bench_world.rs");

/// Everything one request's problems share, built once by [`main`]. Named
/// lifetime for the same reason `chomp_benchmark_port.rs`'s own `Bench` has
/// one -- see that file's doc comment on its `Bench` struct: a closure
/// written inline in `main` gets a higher-ranked `&RobotState<'_>` and
/// cannot hand those states to a `&mut PlanningScene<'_>` borrowed from an
/// outer local, since `&mut` is invariant in its type parameter.
struct Bench<'m> {
    model: &'m RobotModel,
    srdf: &'m SrdfModel,
    env: ParryCollisionEnv,
    env_distance_field: PropagationDistanceField,
    cache: DistanceFieldCollisionCache<'m>,
    mesh_scene: PlanningScene<'m>,
    acm: AllowedCollisionMatrix,
    params: ChompParameters,
    active_joint_names: Vec<String>,
    group_name: String,
}

impl<'m> Bench<'m> {
    fn solve_problem(
        &mut self,
        id: u64,
        start_map: &BTreeMap<String, f64>,
        goal_map: &BTreeMap<String, f64>,
    ) -> serde_json::Value {
        let start_state = joint_map_to_robot_state(self.model, start_map);
        let goal = ChompGoal {
            joint_constraints: self
                .active_joint_names
                .iter()
                .map(|name| GoalJointConstraint {
                    joint_name: name.clone(),
                    position: *goal_map
                        .get(name)
                        .unwrap_or_else(|| panic!("problem.goal has no entry for {name}")),
                    tolerance_above: f64::EPSILON,
                    tolerance_below: f64::EPSILON,
                    weight: 1.0,
                })
                .collect(),
        };

        let mut collision = ChompCollisionContext {
            cache: &mut self.cache,
            env_distance_field: &self.env_distance_field,
        };

        let env = &self.env;
        let active_joint_names = &self.active_joint_names;
        let mesh_scene = &mut self.mesh_scene;
        // The same `mesh_to_mesh_collision_free_check` (`support/chomp_bench_world.rs`)
        // `chomp_benchmark_port.rs` wires up -- one definition, so this file's
        // in-loop check and its post-loop re-check below are provably the
        // same predicate applied twice, not two implementations that happen
        // to agree today. See this file's header for why the re-check is not
        // a second implementation of the predicate.
        let mut mesh_to_mesh = move |start: &RobotState<'m>, best: &DMatrix<f64>| -> bool {
            mesh_to_mesh_collision_free_check(mesh_scene, env, active_joint_names, start, best)
        };

        let mut rng = ChaCha8Rng::seed_from_u64(PORT_SEED_BASE.wrapping_add(id));
        let chomp_request = ChompRequest {
            start_state: &start_state,
            group_name: &self.group_name,
            goal_constraints: std::slice::from_ref(&goal),
            params: &self.params,
            seed_trajectory: None,
        };

        let (outcome, trace) = solve_with_trace(
            &chomp_request,
            &mut collision,
            Some(&self.acm),
            &mut mesh_to_mesh,
            &mut rng,
        );

        match outcome {
            Ok(solution) => {
                let waypoints: Vec<RobotState<'m>> = (0..solution.trajectory.way_point_count())
                    .map(|i| {
                        solution
                            .trajectory
                            .way_point(i)
                            .unwrap_or_else(|e| panic!("way_point({i}): {e}"))
                            .clone()
                    })
                    .collect();

                // The predicate under review, called a second time, after
                // the loop, on the trajectory the loop actually returned --
                // same `PlanningScene::is_path_valid` call, same `env`,
                // same `CollisionRequest::default()`, same `None`/`&[]`, on
                // a fresh scene so this check cannot be affected by
                // whatever `current_state` the in-loop closure's
                // `mesh_scene` was left holding.
                let mut post_hoc_scene = PlanningScene::new(self.model, self.srdf);
                let validity = post_hoc_scene.is_path_valid(
                    &self.env,
                    &CollisionRequest::default(),
                    &waypoints,
                    None,
                    &[],
                );

                let exit = trace.as_ref().map(|t| match t.exit {
                    ChompExit::IterationBound => "iteration_bound",
                    ChompExit::ClockLimit => "clock_limit",
                    ChompExit::BreakOut => "break_out",
                });

                let mut line = serde_json::json!({
                    "id": id,
                    "solved": true,
                    "waypoint_count": waypoints.len(),
                    "loop_exit": exit,
                    "mesh_free_passes": trace.as_ref().map(|t| t.mesh_free_passes),
                    "below_threshold_passes": trace.as_ref().map(|t| t.below_threshold_passes),
                    "post_hoc_mesh_to_mesh_collision_free": validity.valid,
                });
                if !validity.valid {
                    let mut detail = Vec::new();
                    for &wp in &validity.invalid_waypoints {
                        post_hoc_scene.set_current_state(waypoints[wp].clone());
                        let result = post_hoc_scene.check_collision(
                            &self.env,
                            &CollisionRequest {
                                contacts: true,
                                max_contacts: usize::MAX,
                                ..CollisionRequest::default()
                            },
                        );
                        let pairs: Vec<String> = result
                            .contacts
                            .map(|c| c.by_pair.keys().map(|(a, b)| format!("{a}/{b}")).collect())
                            .unwrap_or_default();
                        detail
                            .push(serde_json::json!({ "waypoint": wp, "colliding_pairs": pairs }));
                    }
                    line["invalid_waypoints"] = serde_json::Value::Array(detail);
                }
                line
            }
            Err(e) => {
                let mut line = serde_json::json!({
                    "id": id,
                    "solved": false,
                    "error": e.to_string(),
                });
                if let Some(t) = trace {
                    line["loop_exit"] = serde_json::json!(match t.exit {
                        ChompExit::IterationBound => "iteration_bound",
                        ChompExit::ClockLimit => "clock_limit",
                        ChompExit::BreakOut => "break_out",
                    });
                }
                line
            }
        }
    }
}

fn main() {
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

    let (model, srdf) = load_panda();
    let group = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));
    let active_joint_names: Vec<String> = group
        .active_joint_indices()
        .iter()
        .map(|&i| model.joint_model_at(i).name().to_string())
        .collect();

    let obstacles = parse_obstacles(
        request["objects"]
            .as_array()
            .expect("request.objects must be an array"),
    );

    let (env, env_distance_field, cache) = build_collision_world(&model, &obstacles);

    let acm = AllowedCollisionMatrix::from_srdf(&srdf);
    let params = ChompParameters {
        planning_time_limit: NO_CLOCK_BOUND,
        ..ChompParameters::default()
    };

    let mut bench = Bench {
        model: &model,
        srdf: &srdf,
        env,
        env_distance_field,
        cache,
        mesh_scene: PlanningScene::new(&model, &srdf),
        acm,
        params,
        active_joint_names,
        group_name,
    };

    let problems = request["problems"]
        .as_array()
        .expect("request.problems must be an array");

    for &target in &TARGET_IDS {
        let Some(problem) = problems.iter().find(|p| p["id"].as_u64() == Some(target)) else {
            println!("{{\"id\": {target}, \"error\": \"not found in input problem set\"}}");
            continue;
        };
        let start_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["start"].clone()).expect("problem.start");
        let goal_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["goal"].clone()).expect("problem.goal");

        let line = bench.solve_problem(target, &start_map, &goal_map);
        println!("{line}");
    }
}
