// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is Phase 7 benchmark infrastructure (PORTING-PLAN.md
// §5, §118), not a port -- see `lib.rs`'s own top-of-file comment for why
// this crate has no OMPL C++ counterpart to transcribe.

//! Runs this crate's own `rrt_connect` (via [`RrtConnectManager`]) over a
//! `plan`-op request JSON -- the exact format `plan_benchmark_problem_set`
//! emits and `benches/sweep_baseline.sh` feeds to the oracle -- so the port
//! side of Phase 7's benchmark measures the identical problems the C++
//! baseline already measured, not a re-sample.
//!
//! # Why this is docker-free (and needs no oracle at all)
//!
//! Unlike `plan_benchmark_problem_set`, this binary never needs the oracle:
//! it consumes the request JSON directly (`objects` + `problems`),
//! reconstructs the same world `plan_benchmark_problem_set` built when it
//! sampled the pairs, and runs this crate's own planner. There is nothing
//! here for a C++ process to do.
//!
//! # Usage
//!
//! `cargo run --release --example plan_benchmark_port -p moveit-planners-sbp
//! -- <seed_base> [timeout_seconds] [inject] [dense]`, with a `plan`-op
//! request JSON on stdin (see `examples/plan_benchmark_problem_set.rs`'s own
//! doc comment for the exact shape -- the same file
//! `benches/sweep_baseline.sh` writes to
//! `$WORKDIR/$config.json` before piping it to the oracle is valid input
//! here too). `seed_base` is this run's own RNG seed -- independent of the
//! request's own `seed` field, which is the *oracle's* OMPL seed and has no
//! meaning to this crate's `ChaCha8Rng`-driven planner. Each problem's own
//! seed is `seed_base.wrapping_add(problem.id)`, so two runs over the same
//! request file with the same `seed_base` are reproducibly identical, subject
//! to the timeout caveat in `# The timeout` below.
//!
//! Prints one NDJSON line per problem to stdout, then one `{"summary": ...}`
//! line. `length` is this crate's own [`StateSpace::distance`] summed along
//! the returned path -- directly comparable to the oracle response's own
//! `length` field, since `tests/plan_space_parity.rs` already establishes
//! bit-exact parity between this crate's `JointModelGroupSpace` and the
//! oracle's OMPL space.
//!
//! # The timeout
//!
//! Every planner call is bounded by an explicit wall-clock deadline
//! ([`Termination::Both`]), defaulting to [`DEFAULT_TIMEOUT_SECONDS`]. A
//! call that hits it is reported as `outcome: "timeout"` and counted as a
//! **failure**, never as a skip and never left to hang -- an unbounded
//! planner call that never returns is a failed benchmark, not a passing one.
//!
//! Two properties of this bound are stated rather than assumed:
//!
//! 1. **It is checked at iteration granularity.** `rrt_connect` tests the
//!    deadline once at the top of each grow-and-connect iteration
//!    (`rrt_connect.rs`'s own loop), so a call can overshoot by at most the
//!    duration of one in-flight iteration. This binary therefore measures
//!    and reports each problem's *actual* elapsed wall clock and the summary
//!    reports `slowest_seconds`; the bound is verified by that measurement,
//!    not by the deadline's existence.
//! 2. **It costs determinism only if it fires.** [`Termination::Both`]
//!    carries a `Duration`, so unlike [`Termination::Iterations`] it has no
//!    machine-speed-independence guarantee. But the deadline changes the
//!    search only on a call that actually reaches it: on every problem that
//!    finishes inside the budget, the iteration sequence is identical to the
//!    one `Termination::Iterations(max_iterations)` alone would have
//!    produced. The summary's `timeouts` count is therefore also the count
//!    of problems whose result is not reproducible; `timeouts: 0` means the
//!    whole run was.
//!
//! # Condition 2's collision-check resolution
//!
//! Phase 7 condition 2 requires "100% of produced port paths pass
//! `moveit-scene`'s collision check and constraints".
//! [`PlanningScene::is_path_valid`] checks exactly the waypoints it is
//! given -- it does not itself interpolate between them -- and this crate's
//! own `rrt_connect` only *returns* the RRT tree's vertices (roughly
//! [`RrtConnectParams::step_size`] apart), not the interior points
//! [`DiscreteMotionValidator`](moveit_planners_sbp::DiscreteMotionValidator)
//! already checked and discarded while building each edge. Calling
//! `is_path_valid` on the raw returned vertices alone would therefore not
//! independently re-verify anything: those vertices were already
//! known-valid the moment `PlanningSceneValidityChecker::is_valid` accepted
//! them during planning.
//!
//! This binary instead re-interpolates every consecutive waypoint pair via
//! [`StateSpace::interpolate`] at the *same* resolution
//! (`request.motion_resolution`) `DiscreteMotionValidator` used during
//! planning, then calls `is_path_valid` on that dense list -- **every**
//! waypoint, not only the endpoints. This is a deliberate choice, not a
//! default: it re-derives no new information `DiscreteMotionValidator`'s own
//! bisection did not already establish by construction (that type checks
//! every sample index down to `resolution` spacing, not a subsample of them
//! -- see its own doc comment), so what condition 2 actually verifies here
//! is that `is_path_valid`'s independent code path
//! (`PlanningScene::is_state_valid` on each dense waypoint) agrees with
//! `DiscreteMotionValidator`'s (`PlanningSceneValidityChecker::is_valid`
//! during planning) -- an independent-implementation-path cross-check
//! against a planner-side plumbing bug, not a search for
//! finer-than-planning collision gaps. A resolution finer than
//! `motion_resolution` would also find genuine sub-resolution collision gaps
//! neither the planner nor this check has any way to see at their shared
//! resolution -- a real limitation of resolution-discretized collision
//! checking in general (shared with upstream's own analogous discrete motion
//! validators), not something this binary's choice of resolution introduces.
//!
//! # Proving the condition-2 check can fail
//!
//! A validator that passes because it silently checks nothing reports 100%
//! exactly as a working one does. The `inject` argument exists to tell those
//! two apart, and `tools/ci/verify-phase7-benchmark.sh` runs it as a gate
//! rather than as a one-off spot check:
//!
//! - `inject=collision` searches (uniformly, from this run's RNG) for a
//!   state the scene's collision check actually rejects, then splices that
//!   state into the middle of every solved path before `is_path_valid` runs.
//! - `inject=constraint` takes a valid waypoint and drives the constrained
//!   joint outside its tolerance band, then splices that in the same way.
//!   Requires the request to carry a `joint_constraint`.
//!
//! Under either mode the summary's `condition2_pass` must come back **0**
//! *out of a non-zero `condition2_checked`*; this binary exits non-zero if
//! either half fails, so "the checker rejects an injected bad waypoint" is a
//! build failure when untrue rather than a sentence in a report. Both halves
//! are load-bearing: `condition2_pass == 0` on its own is what a run that
//! solved nothing also reports, since it never called the checker at all.
//! The injected state is verified to be genuinely bad by direct query before
//! it is spliced, so the mode cannot silently degrade into injecting a
//! *valid* state and concluding the checker is broken.
//!
//! What that argument does **not** establish is that the collision model is
//! right: `build_injected_state` finds its bad state by asking
//! [`PlanningScene::is_state_valid`], and the rejection it then requires
//! comes from [`PlanningScene::is_path_valid`]. Those are two entry points
//! to the same [`ParryCollisionEnv`], so a backend permissive in one place
//! is permissive in both, and a path that really does collide is produced
//! *and* approved with both gates green. Only an independent implementation
//! can see that, which is what `dense` below exists to feed.
//!
//! # `dense` -- handing the checked waypoints to an independent checker
//!
//! With `dense` as the fourth argument, every solved problem's NDJSON line
//! carries the full densified waypoint list under `"dense"`, as
//! joint-name -> value maps -- the shape the oracle's `is_state_valid` op
//! takes. `tools/ci/verify-phase7-benchmark.sh` turns those into one
//! `is_state_valid` request per path and requires upstream MoveIt's own
//! `PlanningScene::isPathValid` to agree that every waypoint is valid.
//!
//! Handing over *the same waypoint list this binary checked* is the point:
//! a re-derivation on the C++ side would be a different path and could only
//! ever be compared statistically. Upstream's `isPathValid` is a per-
//! waypoint loop with no interpolation of its own
//! (`moveit_core/planning_scene/src/planning_scene.cpp:2365-2424`), the same
//! shape as [`PlanningScene::is_path_valid`] here, so the two are asked
//! exactly the same question about exactly the same states.
//!
//! # Endpoint fidelity -- `start_gap` and `goal_gap`
//!
//! `outcome: "solved"` means `solve()` returned `Ok`; on its own it says
//! nothing about *which* problem was solved. A path that stops short of the
//! goal, or starts somewhere other than the requested start, would be
//! counted as a success by condition 1, pass condition 2 (each of its
//! waypoints is collision-free), and *lower* the median length condition 3
//! compares -- one defect reading as a pass in all three conditions at once.
//! So every solved problem reports `start_gap`/`goal_gap`: this crate's own
//! [`StateSpace::distance`] from the returned path's first waypoint to the
//! requested start, and from its last to the requested goal. Both are
//! expected to be exactly `0.0` (`rrt_connect` returns the endpoint states
//! it was handed, not a resampling of them), and the verify script gates on
//! that rather than on their being "small".

use std::collections::BTreeMap;
use std::env;
use std::io::{self, Read};
use std::sync::Arc;
use std::time::{Duration, Instant};

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_constraints::utils::construct_goal_joint_constraints;
use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_sbp::{
    CompoundValue, JointModelGroupSpace, PlanError, PlanningFailure, RrtConnectManager,
    RrtConnectParams, StateSpace, Termination,
};
use moveit_planning::{PlannerManager, PlanningRequest};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Wall-clock bound on a single planner call, in seconds, when the caller
/// does not pass one.
///
/// Sized from a measured pilot rather than picked as a round number. On
/// `floor_wall`/`cage` pilots of 10-20 problems per robot, mean `solve()`
/// time was 2.4 s on panda and 8.7-9.1 s on fanuc, with the slowest single
/// call 9.6 s on panda and 40.4 s on fanuc -- the 6-DoF robot's larger
/// scaled workspace, not a pathology.
///
/// That pilot is no longer the largest observation, so the multiple it was
/// chosen for has shrunk and the number is restated here rather than left to
/// read as a stale 3x. Over the 500-problem sets the slowest single call is
/// 57.46 s (fanuc `cage`, problem 117, `doc/phase7-benchmark-results.json`),
/// and across three port RNG streams (`seed_base` 424242 / 20260806 / 999983)
/// the slowest fanuc call was 58.12 / 56.06 / 54.33 s. So 120 s is 2.1x the
/// largest call measured to date, not 3x, and every one of those runs
/// reported `timeouts: 0`.
///
/// The direction of the error matters, which is why the bound is a multiple
/// of the worst case and not a hair above it: a deadline that fires on a
/// problem the planner *would* have solved converts a success into a recorded
/// failure and understates the port against its own completion condition.
/// The bound exists to stop an unbounded call from hanging the benchmark, not
/// to shape the result, so it is set well clear of the legitimate hard tail.
/// `timeouts` is reported separately from `failures` in the summary precisely
/// so a run where this choice started to bite is visible rather than folded
/// into the failure count.
const DEFAULT_TIMEOUT_SECONDS: f64 = 120.0;

/// Attempts allowed when searching for a genuinely colliding state for
/// `inject=collision`. Finite so a scene with no reachable colliding state
/// fails loudly rather than spinning.
const MAX_INJECT_SEARCH_ATTEMPTS: usize = 100_000;

/// One benchmark robot's fixture wiring. Mirrors
/// `plan_benchmark_problem_set.rs`'s own `ROBOTS` table -- the request JSON's
/// `robot` field is what ties the two together, so this side rebuilds the
/// robot the problems were actually sampled against rather than assuming
/// panda.
fn mesh_package_for(robot: &str) -> (&'static str, &'static str) {
    match robot {
        "panda" => ("moveit_resources_panda_description", "panda_description"),
        "fanuc" => ("moveit_resources_fanuc_description", "fanuc_description"),
        other => panic!("unknown robot {other:?} in request.robot"),
    }
}

fn load_robot(robot: &str) -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    let (package, dir) = mesh_package_for(robot);
    let paths = MeshSearchPaths::new([(package, format!("{meshes_root}/{dir}"))]);
    let urdf_xml = std::fs::read_to_string(format!("{root}/{robot}.urdf")).unwrap();
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/{robot}.srdf")).unwrap();
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &paths)
        .expect("fixture model must build");
    (model, srdf)
}

/// Rebuilds the request's `joint_constraint` (the
/// `<joint>:<position>:<tolerance>` string `plan_benchmark_problem_set`
/// emitted) into the same one-member set that generator filtered endpoints
/// with. Returns the set and the parsed parts, the latter for
/// `inject=constraint`.
fn parse_joint_constraint(
    model: &RobotModel,
    spec: &str,
) -> (KinematicConstraintSet, String, f64, f64) {
    let parts: Vec<&str> = spec.split(':').collect();
    assert_eq!(
        parts.len(),
        3,
        "joint_constraint must be <joint_name>:<position>:<tolerance>, got {spec:?}"
    );
    let position: f64 = parts[1].parse().expect("joint_constraint position");
    let tolerance: f64 = parts[2].parse().expect("joint_constraint tolerance");
    let constraint = JointConstraint::new(model, parts[0], position, tolerance, tolerance, 1.0)
        .unwrap_or_else(|e| panic!("JointConstraint::new({:?}): {e}", parts[0]));
    let mut set = KinematicConstraintSet::new();
    set.push(Constraint::Joint(constraint));
    (set, parts[0].to_string(), position, tolerance)
}

/// The translation column of a row-major 4x4 (`fromRowMajor4x4`'s own
/// encoding, `tools/moveit-oracle/src/oracle.cpp`; indices 3/7/11 for
/// x/y/z). Every obstacle `plan_benchmark_problem_set.rs` emits is an
/// axis-aligned, unrotated box (`Isometry3::translation`, no rotation
/// component), so recovering only the translation is exact for every
/// object this binary is ever fed -- it does not attempt to reconstruct a
/// general rotation, because nothing in this workspace's Phase 7 benchmark
/// set produces one.
fn translation_from_row_major_4x4(flat: &[f64]) -> Isometry3 {
    assert_eq!(flat.len(), 16, "expected a flat 4x4 matrix, got {flat:?}");
    Isometry3::translation(flat[3], flat[7], flat[11])
}

/// The tolerance the concrete-state goal is expressed with.
///
/// Zero, because the question this benchmark asks the port is the question
/// it asks the oracle, and the oracle's is
/// `pdef->setStartAndGoalStates(start, goal)` (`oracle.cpp:5824`) — the goal
/// state itself, not a region around it. A zero-width window is what
/// survives the trip through `goal_constraints` unchanged: see
/// `construct_goal_joint_constraints`' own "`0.0` is how a caller says
/// exactly this state". Any positive tolerance makes `goal_gap` report the
/// width of the window instead of the port's error, and conditions 1 and 3
/// then compare two answers to two different questions.
const GOAL_TOLERANCE: f64 = 0.0;

/// `constructGoalConstraints(state, jmg, tolerance)` over a state-space
/// value: one [`JointConstraint`] per variable of `group_name` at the
/// position `value` puts it in.
fn goal_constraints_for(
    space: &JointModelGroupSpace,
    model: &RobotModel,
    group_name: &str,
    value: &[CompoundValue],
) -> KinematicConstraintSet {
    let mut robot_state = RobotState::new(model);
    robot_state.set_to_default_values();
    space.write_robot_state(&value.to_vec(), &mut robot_state);
    let posed = robot_state.update();
    construct_goal_joint_constraints(model, &posed, group_name, GOAL_TOLERANCE, GOAL_TOLERANCE)
        .unwrap_or_else(|e| panic!("construct_goal_joint_constraints({group_name}): {e}"))
}

/// Inverse of `plan_benchmark_problem_set.rs`'s own `state_to_joint_map`:
/// reads a joint-name -> value map (the request JSON's
/// `problems[].start`/`.goal` shape) back into this group's
/// `StateSpace::State`.
fn joint_map_to_state(
    space: &JointModelGroupSpace,
    model: &RobotModel,
    map: &BTreeMap<String, f64>,
) -> Vec<CompoundValue> {
    let mut robot_state = RobotState::new(model);
    robot_state.set_to_default_values();
    for (name, value) in map {
        robot_state
            .set_variable_position(name, *value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    space.read_robot_state(&robot_state)
}

/// Densifies `path` by interpolating every consecutive pair at `resolution`
/// spacing -- see this file's own `# Condition 2's collision-check
/// resolution` doc section for why.
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

/// Which kind of deliberately-bad waypoint an injection run splices into
/// every solved path, to prove condition 2's check can fail.
///
/// A typed mode rather than the argument string carried around: with a
/// `&str` the set of valid modes was decided twice -- once by an
/// `assert!(matches!(..))` on the command line and again by
/// `build_injected_state`'s own `match`, whose final arm was unreachable
/// only as long as those two agreed. Parsing once here makes the two
/// consistent by construction and leaves `build_injected_state`'s `match`
/// exhaustive with no catch-all to keep in sync.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InjectMode {
    /// Splice a state the collision check rejects.
    Collision,
    /// Splice a state that violates the request's `joint_constraint`.
    Constraint,
}

impl InjectMode {
    fn parse(raw: &str) -> Self {
        match raw {
            "collision" => Self::Collision,
            "constraint" => Self::Constraint,
            other => panic!("inject must be 'collision' or 'constraint', got {other:?}"),
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Collision => "collision",
            Self::Constraint => "constraint",
        }
    }
}

/// Builds the deliberately-bad waypoint `inject` splices into every solved
/// path, and **verifies it is bad by direct query before returning it**.
///
/// The verification is the point. Constructing a state that "should" collide
/// and trusting that it does would make an injection run that reports
/// `condition2_pass == 0` ambiguous between "the checker works" and "the
/// checker rejects everything, including valid states". Both modes here
/// therefore assert the state's badness through the same
/// `PlanningScene::is_state_valid` surface, and additionally assert that a
/// state which is *not* bad in the injected way passes -- so the returned
/// state is known to be rejected for the injected reason specifically.
fn build_injected_state(
    mode: InjectMode,
    space: &JointModelGroupSpace,
    model: &RobotModel,
    srdf: &SrdfModel,
    env: &ParryCollisionEnv,
    parsed_constraint: Option<&(KinematicConstraintSet, String, f64, f64)>,
    rng: &mut ChaCha8Rng,
) -> Vec<CompoundValue> {
    let mut scene = PlanningScene::new(model, srdf);
    let request = CollisionRequest::default();

    let install = |state: &Vec<CompoundValue>, scene: &mut PlanningScene<'_>| {
        let mut rs = scene.current_state().clone();
        space.write_robot_state(state, &mut rs);
        scene.set_current_state(rs);
    };

    match mode {
        InjectMode::Collision => {
            for _ in 0..MAX_INJECT_SEARCH_ATTEMPTS {
                let candidate = space.sample_uniform(rng);
                install(&candidate, &mut scene);
                // Constraints deliberately `None`: this mode must produce a
                // state rejected for *collision*, not one that merely
                // violates a constraint.
                if !scene.is_state_valid(env, &request, None) {
                    return candidate;
                }
            }
            panic!(
                "inject=collision: no colliding state found in {MAX_INJECT_SEARCH_ATTEMPTS} \
                 uniform samples -- this scene may have no reachable collision"
            );
        }
        InjectMode::Constraint => {
            let (set, joint, position, tolerance) = parsed_constraint
                .expect("inject=constraint requires the request to carry a joint_constraint");
            for _ in 0..MAX_INJECT_SEARCH_ATTEMPTS {
                let candidate = space.sample_uniform(rng);
                install(&candidate, &mut scene);
                // Only a collision-free, constraint-satisfying state is a
                // usable base: starting from a colliding one would make the
                // injected rejection attributable to collision instead.
                if !scene.is_state_valid(env, &request, Some(set)) {
                    continue;
                }
                // Drive the constrained joint well outside its band. 4x the
                // tolerance, not 1.01x, so the violation cannot be mistaken
                // for a boundary rounding effect.
                let mut rs = scene.current_state().clone();
                let outside = position + 4.0 * tolerance;
                if rs.set_variable_position(joint, outside).is_err() {
                    continue;
                }
                let violating = space.read_robot_state(&rs);
                install(&violating, &mut scene);
                // Bad *for the constraint*, and clean on collision -- so the
                // rejection this produces is attributable to the constraint.
                if !scene.is_state_valid(env, &request, Some(set))
                    && scene.is_state_valid(env, &request, None)
                {
                    return violating;
                }
            }
            panic!(
                "inject=constraint: no constraint-violating, collision-free state found in \
                 {MAX_INJECT_SEARCH_ATTEMPTS} attempts"
            );
        }
    }
}

/// How one problem ended. `Solved` is the only success; every other variant
/// counts against Phase 7 condition 1's success rate, `Timeout` included.
fn outcome_name(result: &Result<(), &PlanError>) -> &'static str {
    match result {
        Ok(()) => "solved",
        Err(PlanError::Failed(PlanningFailure::DeadlineExhausted)) => "timeout",
        Err(PlanError::Failed(PlanningFailure::IterationsExhausted)) => "iterations_exhausted",
        Err(PlanError::Failed(PlanningFailure::InvalidEndpoint)) => "invalid_endpoint",
        Err(PlanError::NoGoalSample) => "no_goal_sample",
        Err(PlanError::Sbp(_)) => "error",
        Err(_) => "error",
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let usage = "usage: <seed_base> [timeout_seconds] [inject] [dense]";
    let seed_base: u64 = args
        .get(1)
        .unwrap_or_else(|| panic!("{usage}; got {args:?}"))
        .parse()
        .expect("seed_base must be a u64");
    let timeout_seconds: f64 = args
        .get(2)
        .filter(|s| !s.is_empty())
        .map_or(DEFAULT_TIMEOUT_SECONDS, |s| {
            s.parse().expect("timeout_seconds must be a number")
        });
    let inject = args
        .get(3)
        .filter(|s| !s.is_empty())
        .map(|s| InjectMode::parse(s));
    // Spelled out rather than "any non-empty fourth argument": a typo would
    // otherwise silently turn the cross-check's input off, and the verify
    // script would then compare an empty waypoint set and pass.
    let emit_dense = match args.get(4).map(String::as_str) {
        None | Some("") => false,
        Some("dense") => true,
        Some(other) => panic!("fourth argument must be 'dense' or absent, got {other:?}"),
    };
    let timeout = Duration::from_secs_f64(timeout_seconds);

    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .expect("stdin must contain a plan-op request JSON");
    let request: serde_json::Value =
        serde_json::from_str(&input).expect("stdin must be valid JSON");

    let robot = request["robot"].as_str().unwrap_or("panda").to_string();
    let group_name = request["group"]
        .as_str()
        .expect("request.group must be a string")
        .to_string();
    let resolution = request["motion_resolution"]
        .as_f64()
        .expect("request.motion_resolution must be a number");
    let step_size = request["range"]
        .as_f64()
        .expect("request.range must be a number");
    let max_iterations = request["max_iterations"]
        .as_u64()
        .expect("request.max_iterations must be a number") as usize;

    let (model, srdf) = load_robot(&robot);
    let space = JointModelGroupSpace::new(&model, &group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));
    // The same list `plan_benchmark_problem_set`'s `state_to_joint_map` writes
    // `start`/`goal` with, and the same one the oracle's `plan` op emits its
    // own paths with (`group->getVariableNames()`), so a `dense` waypoint map
    // names its joints exactly the way both other sides of this benchmark do.
    let variable_names = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"))
        .variable_names()
        .to_vec();

    let constraint_spec = request["joint_constraint"].as_str().map(str::to_string);
    let parsed_constraint = constraint_spec
        .as_deref()
        .map(|spec| parse_joint_constraint(&model, spec));
    let constraints = parsed_constraint.as_ref().map(|(set, ..)| set.clone());

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
        let pose_flat: Vec<f64> = object["pose"]
            .as_array()
            .expect("object.pose must be an array")
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        let pose = translation_from_row_major_4x4(&pose_flat);
        world.add_shape(
            id,
            Arc::new(Shape::Cuboid(
                Cuboid::new(sx, sy, sz).unwrap_or_else(|e| panic!("Cuboid::new: {e}")),
            )),
            pose,
        );
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    // Built once, before any planning: `inject` splices this state into every
    // solved path, and it is *verified bad by direct query here* rather than
    // assumed bad from how it was constructed -- see this file's `# Proving
    // the condition-2 check can fail`.
    let injected: Option<Vec<CompoundValue>> = inject.map(|mode| {
        // A distinct stream from any problem's planner seed, so the search
        // for a bad state cannot alias a planning RNG.
        let mut rng = ChaCha8Rng::seed_from_u64(seed_base ^ 0xBAD_5EED);
        build_injected_state(
            mode,
            &space,
            &model,
            &srdf,
            &env,
            parsed_constraint.as_ref(),
            &mut rng,
        )
    });

    let mut solved_count = 0usize;
    let mut timeout_count = 0usize;
    let mut failure_count = 0usize;
    let mut total = 0usize;
    let mut condition2_checked = 0usize;
    let mut condition2_pass = 0usize;
    let mut waypoints_checked = 0usize;
    let mut raw_waypoints_total = 0usize;
    let mut max_endpoint_gap = 0f64;
    let mut lengths: Vec<f64> = Vec::new();
    let mut slowest = (0f64, u64::MAX);
    let run_start = Instant::now();

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

        let mut scene = PlanningScene::new(&model, &srdf);
        let start_state = joint_map_to_state(&space, &model, &start_map);
        let goal_state = joint_map_to_state(&space, &model, &goal_map);
        let mut start_robot_state = scene.current_state().clone();
        space.write_robot_state(&start_state, &mut start_robot_state);
        scene.set_current_state(start_robot_state);

        let planning_request = PlanningRequest {
            group_name: group_name.clone(),
            // A concrete-state goal expressed the way upstream expresses one:
            // `constructGoalConstraints(state, jmg, tolerance)`
            // (`kinematic_constraints/utils.hpp:99`), one JointConstraint per
            // group variable. `goal_state` itself is kept because `goal_gap`
            // compares the returned path's last waypoint against it.
            goal_constraints: vec![goal_constraints_for(
                &space,
                &model,
                &group_name,
                &goal_state,
            )],
            path_constraints: constraints.clone(),
            ..PlanningRequest::default()
        };
        let manager = RrtConnectManager {
            resolution,
            seed: seed_base.wrapping_add(id),
            params: RrtConnectParams {
                step_size,
                goal_bias: 0.05,
                // The explicit timeout. See `# The timeout`.
                termination: Termination::Both {
                    max_iterations,
                    deadline: timeout,
                },
                nn_degree: 8,
            },
            solver: None,
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, &planning_request)
            .unwrap_or_else(|e| panic!("get_planning_context: {e}"));
        let t0 = Instant::now();
        let result = context.solve();
        let elapsed = t0.elapsed().as_secs_f64();
        if elapsed > slowest.0 {
            slowest = (elapsed, id);
        }

        match result {
            Ok(response) => {
                drop(context);
                solved_count += 1;
                let mut path: Vec<Vec<CompoundValue>> = response
                    .trajectory
                    .iter()
                    .map(|(rs, _)| space.read_robot_state(rs))
                    .collect();
                let length: f64 = path
                    .windows(2)
                    .map(|pair| space.distance(&pair[0], &pair[1]))
                    .sum();
                lengths.push(length);

                // Measured before the injection splice, and against the two
                // states this problem actually asked for. See
                // `# Endpoint fidelity`.
                let start_gap = space.distance(&path[0], &start_state);
                let goal_gap = space.distance(&path[path.len() - 1], &goal_state);
                let raw_waypoints = path.len();

                // Splice the known-bad state into the middle of the path, if
                // this is an injection run. Done *after* `length` so the
                // reported length still describes the planner's real output.
                if let Some(bad) = &injected {
                    let mid = path.len() / 2;
                    path.insert(mid, bad.clone());
                }

                let dense = densify(&space, &model, &path, resolution);
                let validity = scene.is_path_valid(
                    &env,
                    &CollisionRequest::default(),
                    &dense,
                    constraints.as_ref(),
                    &[],
                );
                condition2_checked += 1;
                waypoints_checked += dense.len();
                raw_waypoints_total += raw_waypoints;
                max_endpoint_gap = max_endpoint_gap.max(start_gap).max(goal_gap);
                if validity.valid {
                    condition2_pass += 1;
                }

                let mut line = serde_json::json!({
                    "id": id,
                    "solved": true,
                    "outcome": "solved",
                    "length": length,
                    "plan_seconds": elapsed,
                    "condition2_valid": validity.valid,
                    "waypoints_checked": dense.len(),
                    "raw_waypoints": raw_waypoints,
                    "start_gap": start_gap,
                    "goal_gap": goal_gap,
                    "invalid_waypoint_count": validity.invalid_waypoints.len(),
                    // The indices themselves, not just how many: with `dense`
                    // the verify script compares this set against the
                    // oracle's own `invalid_waypoints`, and two checkers that
                    // reject the same *count* of different waypoints are not
                    // the same answer.
                    "invalid_waypoints": validity.invalid_waypoints,
                });
                if emit_dense {
                    // The waypoints as checked, in order, named the way the
                    // oracle's `is_state_valid` op reads them.
                    let waypoints: Vec<serde_json::Value> = dense
                        .iter()
                        .map(|rs| {
                            let map: BTreeMap<&str, f64> = variable_names
                                .iter()
                                .map(|name| {
                                    let value = rs.variable_position(name).unwrap_or_else(|e| {
                                        panic!("variable_position({name}): {e}")
                                    });
                                    (name.as_str(), value)
                                })
                                .collect();
                            serde_json::to_value(map).expect("waypoint map must serialize")
                        })
                        .collect();
                    line["dense"] = serde_json::Value::Array(waypoints);
                }
                println!("{line}");
            }
            Err(e) => {
                // `PlanningContext::solve` hands back
                // `moveit_planning::PlanError` (a boxed `dyn Error`), while
                // this benchmark classifies outcomes by sbp's own variants.
                // `RrtConnectManager` is the only planner it constructs, so
                // any other error type here is a wiring bug, not an outcome.
                let sbp_error = e.downcast_ref::<PlanError>().unwrap_or_else(|| {
                    panic!("RrtConnectContext::solve must fail with an sbp PlanError: {e}")
                });
                let outcome = outcome_name(&Err(sbp_error));
                if outcome == "timeout" {
                    timeout_count += 1;
                } else {
                    failure_count += 1;
                }
                drop(context);
                println!(
                    "{}",
                    serde_json::json!({
                        "id": id,
                        "solved": false,
                        "outcome": outcome,
                        "plan_seconds": elapsed,
                        "failure": e.to_string(),
                    })
                );
            }
        }
    }

    let wall_clock = run_start.elapsed().as_secs_f64();
    lengths.sort_by(f64::total_cmp);
    let median_length = if lengths.is_empty() {
        None
    } else if lengths.len() % 2 == 1 {
        Some(lengths[lengths.len() / 2])
    } else {
        Some((lengths[lengths.len() / 2 - 1] + lengths[lengths.len() / 2]) / 2.0)
    };

    println!(
        "{}",
        serde_json::json!({
            "summary": {
                "robot": robot,
                "group": group_name,
                "config": request["config"],
                "seed_base": seed_base,
                "request_seed": request["seed"],
                "timeout_seconds": timeout_seconds,
                "joint_constraint": constraint_spec,
                "inject": inject.map(InjectMode::as_str),
                "problems": total,
                "solved": solved_count,
                "timeouts": timeout_count,
                "failures": failure_count,
                "median_length": median_length,
                "condition2_checked": condition2_checked,
                "condition2_pass": condition2_pass,
                "waypoints_checked": waypoints_checked,
                "raw_waypoints": raw_waypoints_total,
                "max_endpoint_gap": max_endpoint_gap,
                "wall_clock_seconds": wall_clock,
                "slowest_seconds": slowest.0,
                "slowest_problem_id": if slowest.1 == u64::MAX { None } else { Some(slowest.1) },
            }
        })
    );

    eprintln!(
        "robot={robot} problems={total} solved={solved_count} timeouts={timeout_count} \
         failures={failure_count} cond2={condition2_pass}/{condition2_checked} \
         max_endpoint_gap={max_endpoint_gap} \
         wall_clock={wall_clock:.1}s slowest={:.1}s",
        slowest.0
    );

    // An injection run that still reports every path valid means the
    // condition-2 checker did not actually check -- the exact failure mode
    // `# Proving the condition-2 check can fail` exists to rule out. Exit
    // non-zero so the verify script gates on it.
    if let Some(mode) = inject {
        let mode = mode.as_str();
        // Two assertions because `condition2_pass == 0` alone is satisfied
        // vacuously: a run that solves nothing never calls `is_path_valid`,
        // so the count it is compared against is zero too. Before this first
        // assertion existed, an injection run whose planner deadline was cut
        // to 1ns printed "inject=collision rejected all 0 paths, as required"
        // and exited 0 -- the gate that exists to prove the condition-2
        // checker can fail, vouching for a checker it never called.
        assert!(
            condition2_checked > 0,
            "inject={mode} solved no problem, so `is_path_valid` was never \
             called -- an injection run that checks nothing cannot show that \
             the condition-2 checker rejects a bad waypoint"
        );
        assert_eq!(
            condition2_pass, 0,
            "inject={mode} spliced a state verified invalid by direct query into every \
             solved path, but is_path_valid still passed {condition2_pass}/{condition2_checked} \
             of them -- the condition-2 check is not checking what it reports on"
        );
        eprintln!("inject={mode} rejected all {condition2_checked} paths, as required");
    }
}
