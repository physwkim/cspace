// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is Phase 8 benchmark infrastructure (PORTING-PLAN.md
// §5's "CHOMP/STOMP는 Phase 7과 같은 속성 기반 검증", §263), not a port. The
// C++ side of MoveIt has no equivalent binary: upstream drives CHOMP only
// through `chomp_interface`'s pluginlib `PlanningContext` (excluded from this
// port by D1/D2, PORTING-PLAN.md §121.3), never through a benchmark runner.

//! Runs this crate's [`chomp::solve`](cspace_planners_chomp::solve) over a
//! `plan`-op request JSON -- the exact format
//! `cspace-planners-sbp`'s `examples/plan_benchmark_problem_set` emits and
//! `cspace-planners-sbp/benches/sweep_baseline.sh` feeds to the oracle -- so
//! Phase 8's CHOMP measurement runs the *identical* 500 problems the Phase 7
//! C++ OMPL RRTConnect baseline was measured on, not a re-sample.
//!
//! # Which baseline this measures against, and why
//!
//! §5's Phase 8 completion condition for CHOMP/STOMP is "Phase 7과 같은 속성
//! 기반 검증" -- Phase 7's three properties, whose lines 705-708 name **C++
//! OMPL RRTConnect** as the comparison side of conditions 1 and 3. That is
//! the baseline this binary's output is compared against. The other
//! available reading ("the same *shape* of check, but against C++ CHOMP")
//! was not measurable in this workspace when this file was written, and now
//! is: the oracle answers `chomp_plan` (`oracle.cpp`'s `chompPlan`, upstream
//! `ChompPlanner::solve`) and `stomp_plan` (`stompPlan`, upstream
//! `StompPlanningContext::solve`), and
//! `tools/ci/measure-phase8-cpp-baseline.sh` drives both. So the reading is
//! now a choice rather than the only option, and this binary keeps the
//! RRTConnect one because §5 names that baseline; the planner-against-its-own-
//! upstream reading is measured by
//! `tools/ci/measure-phase8-optimizer-properties.sh` instead. See
//! PORTING-PLAN.md §263 for the original assumption and the Phase 8 property
//! section for what replaced it.
//!
//! Comparing a trajectory optimizer against a sampling-based planner's
//! success rate is a comparison between different algorithm classes, and
//! the numbers should be read that way -- CHOMP seeded from a quintic-spline
//! interpolation has no mechanism for escaping a local minimum the way
//! RRT-Connect's random restarts do. This binary measures and reports;
//! it does not adjust either side to make them look alike.
//!
//! # Why this is docker-free (and needs no oracle at all)
//!
//! Like `plan_benchmark_port`, this binary consumes the request JSON
//! directly (`objects` + `problems`), reconstructs the world
//! `plan_benchmark_problem_set` built when it sampled the pairs, and runs
//! this crate's own planner. There is nothing here for a C++ process to do.
//!
//! # Usage
//!
//! `cargo run --release --example chomp_benchmark_port -p
//! cspace-planners-chomp -- <seed_base> [planning_time_limit_secs]`, with a
//! `plan`-op request JSON on stdin.
//!
//! `planning_time_limit_secs` defaults to upstream's own
//! `ChompParameters::planning_time_limit` default of `6.0`. That default
//! makes each problem's outcome depend on how fast the machine ran it:
//! `ChompOptimizer::optimize` breaks out of its iteration loop on elapsed
//! wall clock, so a loaded machine completes fewer iterations and reports
//! more failures. Measured on this tree: the same 500 problems at the same
//! `seed_base` solved 359 on an otherwise-idle machine and 349 while a
//! second sweep ran alongside, differing on 12 problems. Pass a value large
//! enough never to bind (the gate `tools/ci/verify-phase8-benchmark.sh` uses
//! `1e9`) to make the run depend only on
//! [`ChompParameters::max_iterations`] and therefore be reproducible.
//!
//! `seed_base` seeds the per-problem `ChaCha8Rng` CHOMP's
//! [`use_stochastic_descent`](cspace_planners_chomp::ChompParameters::use_stochastic_descent)
//! draws from; each problem uses `seed_base.wrapping_add(problem.id)`, so a
//! rerun with the same `seed_base` over the same request file is
//! reproducible. It is unrelated to the request's own `seed` field (that
//! one is the oracle's OMPL seed).
//!
//! Prints one NDJSON line per problem to stdout: `plan_benchmark_port`'s
//! own shape plus one attribution field,
//! `{"id", "solved", "length"?, "condition2_valid"?,
//! "invalid_waypoint_count"?, "condition2_valid_at_returned_waypoints"?,
//! "objective"?, "loop"?, "failure"?}`. See [`returned_waypoints`] for what the extra
//! field is for and why it is not the condition-2 number.
//!
//! # `objective`, and why it is three pairs rather than one number
//!
//! `length` is a *path-length* proxy, not CHOMP's own objective; a CHOMP run
//! that lowers its smoothness+obstacle cost can raise its joint-space path
//! length and vice versa. `objective` is the optimizer's own cost function,
//! read off
//! [`ChompSolution::objective`](cspace_planners_chomp::ChompSolution::objective),
//! and it is present on every solved line and absent on every failed one
//! (`solve` returns `Err`, so there is no solution to read it from).
//!
//! ```text
//! "objective": {"seed": {"smoothness": s, "collision": c, "total": t},
//!               "best": {...}, "last": {...},
//!               "improvement": seed.total - best.total,
//!               "descent":     seed.total - last.total}
//! ```
//!
//! `best` is the objective of the trajectory this line's `length` and
//! `condition2_valid` were measured on -- the one
//! `ChompOptimizer::optimize` copied back out of `best_group_trajectory_`.
//! `improvement` is therefore the paired improvement claim PORTING-PLAN.md
//! §264.12 asked for, but it cannot be negative: `best` starts at `seed` and
//! the optimizer only ever replaces it with something smaller
//! (`chomp_optimizer.cpp:338`). A sweep that reported "0 problems made worse"
//! from `improvement` alone would be reporting the min-tracking, not the
//! optimizer. `descent` is the number whose sign is open -- `last` is the
//! objective of the final iterate the loop actually evaluated, which
//! upstream computes and discards, so `descent < 0` says gradient descent
//! ended above where it started and only the best-snapshot kept the answer
//! from being worse than its own input.
//!
//! # `loop`, and why `improvement == 0` needs it
//!
//! `improvement == 0` has more than one cause and `objective` cannot tell
//! them apart: a seed at a local minimum, a collision term with no support, an
//! update computed and rejected, and an update scaled to nothing all produce
//! the same zero. `loop` is
//! [`ChompSolution::loop_trace`](cspace_planners_chomp::ChompSolution::loop_trace)
//! -- the loop's own account of what it did -- and it separates them.
//!
//! Unlike `objective`, `loop` is not absent on a failed line: this binary
//! reads it off
//! [`solve_with_trace`](cspace_planners_chomp::solve_with_trace) rather than
//! [`solve`](cspace_planners_chomp::solve), so a failed line carries the
//! trace of whichever attempt's optimizer last completed a loop -- the
//! *why* behind `INVALID_MOTION_PLAN` and `GOAL_CONSTRAINTS_VIOLATED`, both
//! of which can only fire after an optimizer has run. `loop` is still
//! absent on the failures that precede any optimizer attempt (bounds and
//! goal-constraint validation, trajectory initialization, optimizer
//! construction).
//!
//! ```text
//! "loop": {"evaluations": n, "exit": "iteration_bound"|"clock_limit"|"break_out",
//!          "accepted": k, "mesh_free_passes": m, "below_threshold_passes": b,
//!          "seed_points_within_clearance": w, "seed_points_in_collision": c,
//!          "first_pass_max_update": u}
//! ```
//!
//! `evaluations` is the one to read first: the objective is evaluated at the
//! *top* of each pass and the increments are applied after it, so
//! `evaluations == 1` means the loop left before any updated iterate was ever
//! costed, and no cause that needs a second evaluation can be the reason.
//! `accepted` counts the passes that beat the running best, so `accepted == 0`
//! with `evaluations > 1` is the "computed and rejected" case. `u` is the
//! largest change the first pass actually applied after
//! `joint_update_limit`'s rescale, so a `u` at the limit rules out a
//! collapsed update.
//!
//! A request carrying `condition2_resolutions: [r, ...]` additionally gets
//! `condition2_by_resolution`, one condition-2 verdict per `r` over the same
//! path -- see [`condition2_by_resolution`]. Without that field nothing about
//! this binary's output or cost changes.
//!
//! # Where every non-obvious constant comes from
//!
//! - CHOMP's own tuning knobs are [`ChompParameters::default`] unmodified --
//!   upstream's `ChompParameters()` constructor values (`max_iterations = 50`,
//!   `planning_time_limit = 6.0`, `enable_failure_recovery = false`), not a
//!   set tuned to make this benchmark look better. Both of those are also
//!   what bounds this binary's runtime per problem: the optimizer loop runs
//!   at most `max_iterations` times and additionally breaks once
//!   `planning_time_limit` seconds have elapsed
//!   (`ChompOptimizer::optimize`), and with `enable_failure_recovery` unset
//!   the replan loop in `solve` runs exactly once.
//! - The environment distance field is upstream `CollisionEnvDistanceField`'s
//!   own defaults (`collision_env_distance_field.hpp:49-55`): size 3x3x4 m
//!   centred on the robot origin, resolution 0.02 m, max propagation
//!   distance 0.25 m, unsigned, collision tolerance 0.0. CHOMP upstream gets
//!   its distance field from exactly that class via `CollisionEnvHybrid`, so
//!   these are the values a real upstream CHOMP run uses.
//! - Goal joint tolerances are `f64::EPSILON`, upstream's own default for
//!   `kinematic_constraints::constructGoalConstraints(state, jmg, tolerance =
//!   std::numeric_limits<double>::epsilon())`
//!   (`moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/utils.hpp:99-101`),
//!   the call
//!   `move_group` makes to turn a goal *state* into the
//!   `goal_constraints[0].joint_constraints` CHOMP's `solve` reads.
//!
//! # `mesh_to_mesh_collision_free` is really wired here
//!
//! [`chomp::solve`](cspace_planners_chomp::solve) takes upstream's
//! `ChompOptimizer::isCurrentTrajectoryMeshToMeshCollisionFree` as an
//! injected closure rather than a method, so this crate need not depend on
//! `cspace-scene`/`cspace-collision`'s `ParryCollisionEnv` (see
//! `optimizer.rs`'s own "closed API gap" doc). Every caller in the tree
//! before this file passed `|_, _| false` -- i.e. no test ever exercised
//! upstream's every-10th-iteration mesh check doing anything. This binary
//! passes a real implementation: `PlanningScene::is_path_valid` over the
//! best group trajectory's rows, which is what upstream's method does
//! (`chomp_optimizer.cpp:520-537`). The dependency that makes it possible is
//! a **dev**-dependency, so the library's own dependency graph is unchanged.
//!
//! # Condition 2's collision-check resolution
//!
//! Identical rule to `plan_benchmark_port`'s, deliberately: the returned
//! path is re-interpolated at the request's own `motion_resolution` before
//! [`PlanningScene::is_path_valid`] sees it. CHOMP returns a fixed 101-point
//! trajectory (upstream's `ChompTrajectory(model, 3.0, 0.03, group)`), whose
//! consecutive points can be much further apart than `motion_resolution`, so
//! checking the raw 101 points would be a *weaker* check than the one Phase
//! 7's port measurement passed. Using the same rule for both keeps the two
//! condition-2 numbers comparable.

use std::collections::BTreeMap;
use std::env;
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
    ChompExit, ChompGoal, ChompLoopTrace, ChompObjective, ChompObjectiveProgress, ChompParameters,
    ChompRequest, GoalJointConstraint, solve_with_trace,
};
use cspace_planners_sbp::{CompoundValue, JointModelGroupSpace, StateSpace};
use cspace_scene::PlanningScene;
use nalgebra::DMatrix;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

// `DISTANCE_FIELD_RESOLUTION`, `distance_field_config`,
// `fixture_mesh_search_paths`, `load_panda`, `Obstacle`, `parse_obstacles`,
// `joint_map_to_robot_state`, `build_collision_world` and
// `mesh_to_mesh_collision_free_check` all live in `support/chomp_bench_world.rs`
// now -- shared with `verify_final_trajectory_predicate.rs` so the two
// cannot silently diverge on the world this binary measures against. See
// that file's own header for why `include!` and not a `src/` module or a
// second copy.
include!("support/chomp_bench_world.rs");

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
/// read only by [`densify`], after `solve` has returned -- so this list is
/// several verdicts about one path, not several runs. That is what makes an
/// operating-point sweep affordable on the STOMP side, where one 500-problem
/// sweep costs about three hours and re-running it per resolution would not
/// be measurable at all.
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

/// Serialises one [`ChompObjectiveProgress`] into the `objective` object this
/// file's header documents.
///
/// `total` is emitted alongside its two components rather than left for the
/// consumer to add: the sum is the quantity CHOMP's accept test compares
/// (`chomp_optimizer.cpp:338`), and a consumer that re-derived it would be
/// free to derive a different one. `improvement` and `descent` are emitted
/// for the same reason -- see the header for why the two are not the same
/// claim.
fn objective_json(progress: &ChompObjectiveProgress) -> serde_json::Value {
    let term = |o: &ChompObjective| {
        serde_json::json!({
            "smoothness": o.smoothness,
            "collision": o.collision,
            "total": o.total(),
        })
    };
    serde_json::json!({
        "seed": term(&progress.seed),
        "best": term(&progress.best),
        "last": term(&progress.last),
        "improvement": progress.improvement(),
        "descent": progress.descent(),
    })
}

/// Serialises one [`ChompLoopTrace`] into the `loop` object this file's
/// header documents.
///
/// `exit` is a string rather than an index so a consumer cannot silently
/// re-map it when a variant is added; the three names are
/// `ChompExit`'s own.
fn loop_json(trace: &ChompLoopTrace) -> serde_json::Value {
    serde_json::json!({
        "evaluations": trace.evaluations,
        "exit": match trace.exit {
            ChompExit::IterationBound => "iteration_bound",
            ChompExit::ClockLimit => "clock_limit",
            ChompExit::BreakOut => "break_out",
        },
        "accepted": trace.accepted,
        "mesh_free_passes": trace.mesh_free_passes,
        "below_threshold_passes": trace.below_threshold_passes,
        "seed_points_within_clearance": trace.seed_points_within_clearance,
        "seed_points_in_collision": trace.seed_points_in_collision,
        "first_pass_max_update": trace.first_pass_max_update,
    })
}

/// Everything one request's problems share, built once by [`main`] and read
/// by [`Bench::solve_problem`] for each problem.
///
/// This exists to give the robot model's lifetime a *name*.
/// [`solve`]'s `mesh_to_mesh_collision_free` parameter is
/// `&mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool` with `'m` fixed
/// by the call, and the [`PlanningScene`] that closure checks against is
/// itself `PlanningScene<'m>`; a closure written inside `main` gets a
/// higher-ranked `&RobotState<'_>` instead and cannot hand those states to a
/// scene borrowed from an outer local (`&mut PlanningScene<'_>` is invariant
/// in `'_`). Naming `'m` on this struct is what makes the two line up.
struct Bench<'m> {
    model: &'m RobotModel,
    /// The same metric the C++ OMPL baseline's `length` is measured in --
    /// see this file's own doc comment.
    space: JointModelGroupSpace,
    env: ParryCollisionEnv,
    /// The environment distance field CHOMP's obstacle gradients read.
    /// Built once for the whole request: every problem in one request shares
    /// a single obstacle configuration.
    env_distance_field: PropagationDistanceField,
    /// Reused across problems, matching upstream, where one
    /// `CollisionEnvDistanceField` lives for the planning scene's whole
    /// lifetime and is re-consulted per request.
    /// `generate_collision_checking_structures` re-validates its cached
    /// entry against the group/state/ACM it is handed and rebuilds when they
    /// do not match, so reuse cannot leak one problem's geometry into the
    /// next.
    cache: DistanceFieldCollisionCache<'m>,
    /// Upstream's `isCurrentTrajectoryMeshToMeshCollisionFree` scene, and
    /// the condition-2 scene. Two separate scenes because the first is
    /// mutably borrowed by the closure for the whole `solve` call, while the
    /// second has to be usable after it returns.
    mesh_scene: PlanningScene<'m>,
    check_scene: PlanningScene<'m>,
    acm: AllowedCollisionMatrix,
    params: ChompParameters,
    active_joint_names: Vec<String>,
    group_name: String,
    resolution: f64,
    /// The request's `condition2_resolutions`, empty when it has none.
    condition2_resolutions: Vec<f64>,
}

impl<'m> Bench<'m> {
    /// Runs one problem and returns its NDJSON verdict line, plus whether it
    /// counted as solved.
    fn solve_problem(
        &mut self,
        id: u64,
        start_map: &BTreeMap<String, f64>,
        goal_map: &BTreeMap<String, f64>,
        seed: u64,
    ) -> (bool, serde_json::Value) {
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
        let mut mesh_to_mesh = move |start: &RobotState<'m>, best: &DMatrix<f64>| -> bool {
            mesh_to_mesh_collision_free_check(mesh_scene, env, active_joint_names, start, best)
        };

        let mut rng = ChaCha8Rng::seed_from_u64(seed);
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
                let path: Vec<Vec<CompoundValue>> = (0..solution.trajectory.way_point_count())
                    .map(|i| {
                        self.space.read_robot_state(
                            solution
                                .trajectory
                                .way_point(i)
                                .expect("index below way_point_count"),
                        )
                    })
                    .collect();
                let length: f64 = path
                    .windows(2)
                    .map(|pair| self.space.distance(&pair[0], &pair[1]))
                    .sum();

                let raw = returned_waypoints(&self.space, self.model, &path);
                let raw_validity = self.check_scene.is_path_valid(
                    &self.env,
                    &CollisionRequest::default(),
                    &raw,
                    None,
                    &[],
                );
                let dense = densify(&self.space, self.model, &path, self.resolution);
                let validity = self.check_scene.is_path_valid(
                    &self.env,
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
                // Absent rather than null when the optimizer never evaluated
                // its objective, which under these parameters
                // (`max_iterations` = 50) cannot happen -- so a reader that
                // finds a solved line without this key has found a real
                // change, not a configuration.
                if let Some(progress) = solution.objective {
                    record["objective"] = objective_json(&progress);
                }
                if let Some(trace) = solution.loop_trace {
                    record["loop"] = loop_json(&trace);
                }
                if !self.condition2_resolutions.is_empty() {
                    record["condition2_by_resolution"] =
                        serde_json::Value::Array(condition2_by_resolution(
                            &self.space,
                            self.model,
                            &mut self.check_scene,
                            &self.env,
                            &path,
                            &self.condition2_resolutions,
                        ));
                }
                (true, record)
            }
            Err(e) => {
                let mut record = serde_json::json!({
                    "id": id,
                    "solved": false,
                    "failure": e.to_string(),
                });
                // Same field, same reason as the `Ok` arm above: `loop_trace`
                // exists on exactly the runs `solve_with_trace` reports it
                // for, so a failed run with a completed optimizer attempt
                // gets its `loop` object too -- see this file's header,
                // "loop, and why improvement == 0 needs it", and
                // `solve_with_trace`'s own doc for why this was previously
                // unreachable on this branch.
                if let Some(trace) = trace {
                    record["loop"] = loop_json(&trace);
                }
                (false, record)
            }
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let seed_base: u64 = args
        .get(1)
        .unwrap_or_else(|| panic!("usage: <seed_base> [planning_time_limit_secs]; got {args:?}"))
        .parse()
        .expect("seed_base must be a u64");
    let planning_time_limit: f64 = args
        .get(2)
        .map(|s| {
            s.parse::<f64>()
                .expect("planning_time_limit_secs must be a number")
        })
        .unwrap_or(ChompParameters::default().planning_time_limit);

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

    let mut bench = Bench {
        model: &model,
        space,
        env,
        env_distance_field,
        cache,
        mesh_scene: PlanningScene::new(&model, &srdf),
        check_scene: PlanningScene::new(&model, &srdf),
        acm: AllowedCollisionMatrix::from_srdf(&srdf),
        params: ChompParameters {
            planning_time_limit,
            ..ChompParameters::default()
        },
        active_joint_names,
        group_name,
        resolution,
        condition2_resolutions,
    };

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

        let (solved, line) =
            bench.solve_problem(id, &start_map, &goal_map, seed_base.wrapping_add(id));
        if solved {
            solved_count += 1;
        }
        println!("{line}");
    }

    eprintln!("solved={solved_count}/{total}");
}
