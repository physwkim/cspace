// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: like its sibling `stomp_benchmark_port`, this is Phase 8
// benchmark infrastructure, not a port. Upstream reaches STOMP only through
// `StompPlanningContext`'s pluginlib entry point and ships no binary that runs
// it over a problem set.

//! STOMP half of the Phase 8 property instrument: runs
//! [`cspace_planners_stomp::planner::plan`] over the *same* problem set
//! `cspace-planners-sbp`'s `plan_benchmark_problem_set` emits for Phase 7,
//! and reports, per problem, exactly the quantities
//! `tools/ci/measure-phase8-optimizer-properties.sh` gates on.
//!
//! # Why this is not `plan_benchmark_port` with a different planner
//!
//! Phase 7's three conditions are written for a *sampler* measured against a
//! second sampler: a success rate relative to C++ OMPL RRTConnect, and a
//! median path length relative to the same baseline. STOMP starts from a seed
//! trajectory (`TrajectoryInitialization::LinearInterpolation`, hardcoded by
//! `plan` itself, matching upstream's own hardcoding at
//! `stomp_moveit_planning_context.cpp:196`) and returns a *locally optimized*
//! version of it. Two of Phase 7's conditions therefore do not transfer as
//! written, and the ones that do transfer need a different reference:
//!
//! * **Path validity (Phase 7 condition 2) transfers verbatim.** A returned
//!   trajectory whose waypoints collide is wrong for any planner. This binary
//!   reports it the same way `plan_benchmark_port` does -- densified, over
//!   every solved problem, with the invalid-waypoint *indices* so an
//!   independent checker can be compared index by index rather than count by
//!   count. It also reports [`returned_waypoints`]'s undensified verdict
//!   alongside the densified one, as `condition2_valid_at_returned_waypoints`
//!   -- the same attribution field `stomp_benchmark_port`/`chomp_benchmark_port`
//!   already carry, missing here until now. This adds an attribution field;
//!   the densified verdict stays the official condition-2 number.
//! * **The quality condition does NOT become "output cost <= seed cost".**
//!   That was this binary's first design, and it is unfalsifiable. `seed_cost`
//!   and `output_cost` below are the same `CostFn` evaluated on the
//!   linear-interpolation seed STOMP starts from and on the trajectory it
//!   returns -- but `Stomp::solve` returns `parameters_valid`
//!   (`crates/cspace-stomp-core/src/stomp.rs:601`), and
//!   `cost_function_from_state_validator` sets that false for any column with
//!   `costs(t) > 0.0`
//!   (`crates/cspace-planners-stomp/src/cost_functions.rs:174,199`).
//!   Every column is a sum of non-negative validator penalties, so
//!   `solved == true` forces `output_cost == 0`, and `seed_cost >= 0` makes
//!   `output_cost <= seed_cost` hold for every run this instrument can
//!   produce. Measured, over `panda floor_wall` problems 0-3 at
//!   `SEED_BASE=525252`: `output_cost` was `0.0` on all 3 solved problems,
//!   with `seed_cost` `0.0`, `19.0`, `0.0`.
//!
//!   What is left of the pairing, and what the gate holds instead, is
//!   `cost_fn_missed_seed_collision`: over solved problems, a seed the
//!   independent checker calls colliding must score `seed_cost > 0`. Nothing
//!   in the solver forces that, so it can fail -- it is a cross-check of the
//!   ported distance-field cost against the collision checker on the same
//!   configuration. `cost_fn_margin_only` counts the opposite direction and is
//!   reported rather than gated: the collision cost is a clearance potential,
//!   so a seed that touches nothing but passes inside the margin legitimately
//!   scores above zero.
//! * **The success rate has no OMPL-relative bar.** `0.9 x cpp_rate` against
//!   RRTConnect would compare a local optimizer with a global search on
//!   problems that need global search; the number that falls out of it says
//!   nothing about whether this port matches upstream STOMP. What this binary
//!   reports instead is the raw rate, for the gate to hold against a *pinned
//!   floor measured from this same tree* -- a regression bar, not a
//!   cross-implementation one. See `PORTING-PLAN.md`'s Phase 8 property
//!   section for the full item-by-item transfer table.
//!
//! # The timeout
//!
//! `plan` has no deadline of its own: upstream bounds it from outside, with an
//! `std::async` watcher that calls `stomp->cancel()` after
//! `req.allowed_planning_time` (`stomp_moveit_planning_context.cpp:247-257`)
//! and then reports the run as timed out. This binary does the same thing with
//! a watchdog thread over a [`CancelHandle`], and -- like upstream, which sets
//! `MoveItErrorCodes::TIMED_OUT` on that path -- counts a fired watchdog as a
//! **failure**, never as a solve. A cancelled STOMP returns its seed, so
//! without that rule a timeout would be indistinguishable from a solve whose
//! optimizer did nothing.
//!
//! # Proving the validity check can fail
//!
//! `inject` splices one state, verified bad by direct query before any
//! planning starts, into the middle of every solved path. The run then asserts
//! that *no* path passed -- and, first, that at least one path was actually
//! checked, since "rejected all 0 paths" is a vacuous pass. Same shape, and
//! the same three assertions, as `plan_benchmark_port`'s own injection mode.
//!
//! The report carries the denominator with the count, because only solved
//! paths can be checked and this planner times out on a real fraction of
//! them: over one 125-problem `panda floor_wall` set at the 120s budget the
//! run checked 105 and 20 timed out, so "rejected all 105 paths" on its own
//! would state a numerator as if it were the injected population.
//!
//! Usage: `optimize_benchmark_stomp <seed_base> [timeout_seconds] [inject]
//! [dense]`, with the problem-set JSON on stdin.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::env;
use std::io::{self, Read};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier};
use std::time::{Duration, Instant};

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{JointModelGroup, MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planners_sbp::{CompoundValue, JointModelGroupSpace, StateSpace};
use cspace_planners_stomp::conversion_functions::{positions, robot_trajectory_to_matrix};
use cspace_planners_stomp::cost_functions;
use cspace_planners_stomp::planner::{PlanRequest, plan};
use cspace_planning::constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use cspace_planning::scene::PlanningScene;
use cspace_stomp_core::{CancelHandle, StompConfiguration, TrajectoryInitialization};
use nalgebra::{DMatrix, DVector};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// Upstream's `collision penalty` argument to `getCollisionCostFunction`,
/// spelled as a literal `1.0` at both of its call sites
/// (`stomp_moveit_planning_context.cpp:165,171`), and the same value it passes
/// as the constraint penalty (cpp:167).
const PENALTY: f64 = 1.0;

/// The wall-clock bound handed to the watchdog when no third argument is
/// given. Matches `plan_benchmark_port`'s own default so a STOMP run and an
/// RRTConnect run over the same problem set are bounded identically; the
/// measured slowest call is reported in every summary so this constant can be
/// checked against what it actually had to absorb.
const DEFAULT_TIMEOUT_SECONDS: f64 = 120.0;

/// How many rejection-sampling attempts `build_injected_state` gets before it
/// gives up. Same bound, for the same reason, as `plan_benchmark_port`'s.
const MAX_INJECT_SEARCH_ATTEMPTS: usize = 100_000;

/// Fixture mesh package/directory per benchmark robot -- the same table
/// `plan_benchmark_port` carries, since both read the same fixtures.
fn mesh_package_for(robot: &str) -> (&'static str, &'static str) {
    match robot {
        "panda" => ("moveit_resources_panda_description", "panda_description"),
        "fanuc" => ("moveit_resources_fanuc_description", "fanuc_description"),
        other => panic!("unknown benchmark robot {other:?}"),
    }
}

fn load_robot(robot: &str) -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    let (package, directory) = mesh_package_for(robot);
    let paths = MeshSearchPaths::new([(package, format!("{meshes_root}/{directory}"))]);
    let urdf_xml = std::fs::read_to_string(format!("{root}/{robot}.urdf"))
        .unwrap_or_else(|e| panic!("read {root}/{robot}.urdf: {e}"));
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/{robot}.srdf"))
        .unwrap_or_else(|e| panic!("parse {root}/{robot}.srdf: {e}"));
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &paths)
        .expect("fixture model must build");
    (model, srdf)
}

/// `joint:position:tolerance`, the same spelling `plan_benchmark_problem_set`
/// writes into the request and `plan_benchmark_port` parses back out.
fn parse_joint_constraint(model: &RobotModel, spec: &str) -> KinematicConstraintSet {
    let parts: Vec<&str> = spec.split(':').collect();
    assert_eq!(
        parts.len(),
        3,
        "joint_constraint must be joint:position:tolerance, got {spec:?}"
    );
    let position: f64 = parts[1].parse().expect("constraint position");
    let tolerance: f64 = parts[2].parse().expect("constraint tolerance");
    let constraint = JointConstraint::new(model, parts[0], position, tolerance, tolerance, 1.0)
        .unwrap_or_else(|e| panic!("JointConstraint::new({:?}): {e}", parts[0]));
    let mut set = KinematicConstraintSet::new();
    set.push(Constraint::Joint(constraint));
    set
}

fn translation_from_row_major_4x4(flat: &[f64]) -> Isometry3 {
    Isometry3::translation(flat[3], flat[7], flat[11])
}

/// A state with every group joint at `map`'s value and every other joint at
/// its default -- the same construction `plan_benchmark_port` uses, so a
/// problem's `start`/`goal` mean the same configuration on both sides.
fn state_from_map<'m>(model: &'m RobotModel, map: &BTreeMap<String, f64>) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in map {
        state
            .set_variable_position(name, *value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    state.update();
    state
}

/// Writes `values` (one entry per active joint of `group`, in
/// `active_joint_names` order) into a copy of `template`.
fn state_from_column<'m>(
    template: &RobotState<'m>,
    group: &JointModelGroup,
    values: &DVector<f64>,
) -> RobotState<'m> {
    let mut state = template.clone();
    for (name, value) in group.active_joint_names().iter().zip(values.iter()) {
        state
            .set_variable_position(name, *value)
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    state.update();
    state
}

/// Upstream `computeLinearInterpolation` (`stomp.cpp`, and this port's private
/// `compute_linear_interpolation`): the exact seed `Stomp::solve` starts from
/// under `TrajectoryInitialization::LinearInterpolation`, which `plan` sets
/// unconditionally. Replicated here rather than called because it is not
/// `pub` -- the same replication, with the same reason, that
/// `cspace-planners-stomp`'s own `plan_finds_a_lower_cost_trajectory...` test
/// already carries.
fn linear_interpolation_seed(
    start: &DVector<f64>,
    goal: &DVector<f64>,
    num_timesteps: usize,
) -> DMatrix<f64> {
    let n = start.len();
    let mut matrix = DMatrix::zeros(n, num_timesteps);
    for i in 0..n {
        let dtheta = (goal[i] - start[i]) / (num_timesteps as f64 - 1.0);
        for t in 0..num_timesteps {
            matrix[(i, t)] = start[i] + t as f64 * dtheta;
        }
    }
    matrix
}

/// Interpolates `path` so that no active joint moves by more than
/// `resolution` between two consecutive checked states.
///
/// Not the space-metric densification `plan_benchmark_port` performs (this
/// crate has no `JointModelGroupSpace`): the per-joint bound is a stronger and
/// metric-free guarantee -- a sum-of-joints distance can hide one joint
/// sweeping most of the step -- and it is the guarantee the gate's
/// `condition2-densified` check reads. Every benchmark group joint is
/// single-variable revolute, asserted by the caller, so a component-wise
/// linear interpolation is the joint's own interpolation.
fn densify<'m>(
    template: &RobotState<'m>,
    group: &JointModelGroup,
    path: &[DVector<f64>],
    resolution: f64,
) -> Vec<RobotState<'m>> {
    let mut out = vec![state_from_column(template, group, &path[0])];
    for pair in path.windows(2) {
        let (from, to) = (&pair[0], &pair[1]);
        let widest = (to - from).abs().max();
        let steps = ((widest / resolution).ceil() as u64).max(1);
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            out.push(state_from_column(
                template,
                group,
                &(from + (to - from) * t),
            ));
        }
    }
    out
}

/// `path` as [`RobotState`]s with no interpolation at all -- exactly the
/// waypoints STOMP returned.
///
/// Feeding these to [`PlanningScene::is_path_valid`] reproduces what
/// upstream's own `isPathValid(trajectory, group)` would report for this
/// planner's output, and nothing finer. Reported alongside the densified
/// verdict as `condition2_valid_at_returned_waypoints`, purely to attribute a
/// condition-2 failure: a path that is valid here and invalid after
/// [`densify`] failed *between* the planner's own waypoints, at a resolution
/// neither the planner nor upstream ever evaluates. The official condition-2
/// number stays the densified one -- this adds an attribution field, it does
/// not move the bar. Same construction, same reason, as
/// `stomp_benchmark_port`'s and `chomp_benchmark_port`'s own
/// `returned_waypoints` (both benchmark infrastructure, not a port -- see
/// this file's header -- so there is no upstream citation for either).
fn returned_waypoints<'m>(
    template: &RobotState<'m>,
    group: &JointModelGroup,
    path: &[DVector<f64>],
) -> Vec<RobotState<'m>> {
    path.iter()
        .map(|column| state_from_column(template, group, column))
        .collect()
}

/// Which kind of deliberately-bad waypoint an injection run splices into every
/// solved path. Typed rather than a `&str` carried around, for the reason
/// `plan_benchmark_port::InjectMode` gives: with a string the valid set is
/// decided twice and the two copies can drift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InjectMode {
    /// A state the collision check rejects.
    Collision,
    /// A state that violates the request's `joint_constraint`.
    Constraint,
}

impl InjectMode {
    fn parse(s: &str) -> Self {
        match s {
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

/// Finds a state that the requested check *actually rejects*, by asking the
/// check -- not by constructing something assumed bad. Returns the active-joint
/// column so the caller can splice it straight into a path.
fn build_injected_state(
    mode: InjectMode,
    model: &RobotModel,
    srdf: &SrdfModel,
    group: &JointModelGroup,
    env: &ParryCollisionEnv,
    constraints: Option<&KinematicConstraintSet>,
    rng: &mut ChaCha8Rng,
) -> DVector<f64> {
    let mut scene = PlanningScene::new(model, srdf);
    let names = group.active_joint_names();
    let bounds: Vec<(f64, f64)> = names
        .iter()
        .map(|name| {
            let joint = model
                .joint_model(name)
                .unwrap_or_else(|e| panic!("joint_model({name}): {e}"));
            let bound = joint.variable_bounds()[0];
            let (lo, hi) = (bound.min_position, bound.max_position);
            assert!(
                lo.is_finite() && hi.is_finite(),
                "{name} has unbounded position, so a uniform sample over it is not defined"
            );
            (lo, hi)
        })
        .collect();
    let template = {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        state
    };

    for _ in 0..MAX_INJECT_SEARCH_ATTEMPTS {
        let column = DVector::from_iterator(
            names.len(),
            bounds.iter().map(|(lo, hi)| rng.random_range(*lo..*hi)),
        );
        let candidate = state_from_column(&template, group, &column);
        scene.set_current_state(candidate);
        let rejected = match mode {
            InjectMode::Collision => !scene.is_state_valid(env, &CollisionRequest::default(), None),
            InjectMode::Constraint => {
                let set = constraints.expect(
                    "inject=constraint needs the request to carry a joint_constraint, else \
                     there is no constraint for the spliced state to violate",
                );
                !scene.is_state_constrained(set)
            }
        };
        if rejected {
            return column;
        }
    }
    panic!(
        "no state rejected by inject={} found in {MAX_INJECT_SEARCH_ATTEMPTS} attempts -- \
         an injection run cannot prove the check fires without one",
        mode.as_str()
    );
}

/// Largest absolute deviation of `column` from `wanted`, joint by joint.
fn endpoint_gap(column: &DVector<f64>, wanted: &DVector<f64>) -> f64 {
    (column - wanted).abs().max()
}

/// The length of `columns` in the plan-space metric --
/// [`JointModelGroupSpace::distance`] summed along it, which is what the
/// oracle's `stomp_plan` reports as its own `length` (`oracle.cpp`'s
/// `planSpacePathLength`, summing OMPL `CompoundStateSpace::distance`).
///
/// Not the Euclidean `.norm()` over raw joint deltas this instrument reported
/// before: both spaces weight each bounded axis by `1/(max - min)` and add the
/// weighted absolute differences, so the two are different quantities rather
/// than two estimates of one. Phase 7's `condition3-*` is a ratio between the
/// port's length and the C++ side's, and it means nothing until both are in
/// this metric. See `optimize_benchmark_chomp`'s twin for the same note.
fn plan_space_length(
    space: &JointModelGroupSpace,
    scratch: &mut RobotState<'_>,
    group: &JointModelGroup,
    columns: &[DVector<f64>],
) -> f64 {
    let states: Vec<Vec<CompoundValue>> = columns
        .iter()
        .map(|column| {
            for (name, value) in group.active_joint_names().iter().zip(column.iter()) {
                scratch
                    .set_variable_position(name, *value)
                    .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
            }
            space.read_robot_state(scratch)
        })
        .collect();
    states
        .windows(2)
        .map(|pair| space.distance(&pair[0], &pair[1]))
        .sum()
}

fn median(mut values: Vec<f64>) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let n = values.len();
    Some(if n % 2 == 1 {
        values[n / 2]
    } else {
        (values[n / 2 - 1] + values[n / 2]) / 2.0
    })
}

/// One problem's emitted JSON record.
///
/// Every emission site in this file's main loop builds one of these and
/// prints [`ProblemRecord::to_json`] rather than assembling its own
/// `serde_json::json!` object, so a field cannot exist on one exit path and
/// not another. That was exactly the bug this replaces: `seed_valid`,
/// `seed_length`, `seed_cost` and `seed_cost_fn_valid` are all functions of
/// `start_column`/`goal_column`/`config` alone -- available before `plan` is
/// even called -- but were only added to the record inside the
/// `solved_count += 1` arm, so a failed or timed-out problem's line carried
/// none of them, though none of the four depended on the outcome at all.
///
/// Fields that genuinely cannot exist on a given path -- everything
/// downstream of the returned trajectory -- are `Option` here and reach the
/// line as an explicit JSON `null`, never an absent key: an absent key on
/// one branch of a two-way `jq` select silently drops that record from
/// *both* buckets a partition builds from it, where a keyed `null` does
/// not.
struct ProblemRecord {
    id: u64,
    solved: bool,
    outcome: &'static str,
    plan_seconds: f64,
    /// `Some` only on a failed or timed-out outcome.
    failure: Option<String>,
    /// `Some` only when `plan` returned `Ok(Some(_))` and the run did not
    /// time out -- everything below this point needs the returned
    /// trajectory.
    condition2_valid: Option<bool>,
    condition2_valid_at_returned_waypoints: Option<bool>,
    waypoints_checked: Option<usize>,
    raw_waypoints: Option<usize>,
    start_gap: Option<f64>,
    goal_gap: Option<f64>,
    invalid_waypoint_count: Option<usize>,
    invalid_waypoints: Option<Vec<usize>>,
    length: Option<f64>,
    output_cost: Option<f64>,
    output_cost_fn_valid: Option<bool>,
    /// Computed from `start_column`/`goal_column`/`config` alone; present on
    /// every outcome, `plan` is never consulted for any of these four.
    seed_length: f64,
    seed_valid: bool,
    seed_cost: f64,
    seed_cost_fn_valid: bool,
}

impl ProblemRecord {
    fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "id": self.id,
            "solved": self.solved,
            "outcome": self.outcome,
            "plan_seconds": self.plan_seconds,
            "failure": self.failure,
            "condition2_valid": self.condition2_valid,
            "condition2_valid_at_returned_waypoints": self.condition2_valid_at_returned_waypoints,
            "waypoints_checked": self.waypoints_checked,
            "raw_waypoints": self.raw_waypoints,
            "start_gap": self.start_gap,
            "goal_gap": self.goal_gap,
            "invalid_waypoint_count": self.invalid_waypoint_count,
            "invalid_waypoints": self.invalid_waypoints,
            "length": self.length,
            "seed_cost": self.seed_cost,
            "output_cost": self.output_cost,
            "seed_valid": self.seed_valid,
            "seed_cost_fn_valid": self.seed_cost_fn_valid,
            "output_cost_fn_valid": self.output_cost_fn_valid,
            "seed_length": self.seed_length,
        })
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

    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .expect("stdin must contain a problem-set request JSON");
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

    let (model, srdf) = load_robot(&robot);
    let group = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));
    // Every quantity below -- the seed matrix, the cost columns, the
    // per-joint densification -- indexes active joints one value at a time.
    // `get_collision_cost_function` enforces the same thing internally; this
    // states it where the assumption is first used instead of surfacing it as
    // a mismatched matrix row much later.
    for name in group.active_joint_names() {
        let count = model
            .joint_model(name)
            .unwrap_or_else(|e| panic!("joint_model({name}): {e}"))
            .variable_count();
        assert_eq!(
            count, 1,
            "{name} has {count} variables; this instrument reports one value per active joint"
        );
    }
    let variable_names = group.variable_names().to_vec();

    let constraint_spec = request["joint_constraint"].as_str().map(str::to_string);
    let constraints = constraint_spec
        .as_deref()
        .map(|spec| parse_joint_constraint(&model, spec));

    // The constraint the *checker* uses, which is not always the constraint the
    // planner was given. Upstream's `getConstraintsCostFunction` costs a state by
    // `constraints.decide(state).distance`, and `JointConstraint::decide` returns
    // `constraint_weight_ * fabs(dif)` -- the distance to the target, not the
    // amount by which the tolerance is exceeded (`kinematic_constraint.cpp:326`).
    // `cost_function_from_state_validator` then marks any timestep with
    // `cost > 0.0` invalid, so a group that moves the constrained joint at all
    // has no valid timestep, and STOMP reports no valid trajectory however long
    // it is given. Measured: 0 solved / 16 timeouts over three constrained sets,
    // and 8/8 solved on the same 8 problems once the constraint moved to this
    // field.
    // A path-constrained *problem* therefore cannot produce a solved STOMP path
    // for the validity check to reject an injected waypoint from -- so the two
    // roles are separate fields, and the injection stage plans unconstrained and
    // checks against `check_joint_constraint`.
    let check_constraint_spec = request["check_joint_constraint"]
        .as_str()
        .map(str::to_string)
        .or_else(|| constraint_spec.clone());
    let check_constraints = check_constraint_spec
        .as_deref()
        .map(|spec| parse_joint_constraint(&model, spec));

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
        world.add_shape(
            id,
            Arc::new(Shape::Cuboid(
                Cuboid::new(sx, sy, sz).unwrap_or_else(|e| panic!("Cuboid::new: {e}")),
            )),
            translation_from_row_major_4x4(&pose_flat),
        );
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    // Built before any planning, and verified bad by direct query rather than
    // assumed bad from how it was constructed.
    let injected: Option<DVector<f64>> = inject.map(|mode| {
        let mut rng = ChaCha8Rng::seed_from_u64(seed_base ^ 0xBAD_5EED);
        build_injected_state(
            mode,
            &model,
            &srdf,
            group,
            &env,
            check_constraints.as_ref(),
            &mut rng,
        )
    });

    let mut template = RobotState::new(&model);
    template.set_to_default_values();

    // The metric the C++ baseline's `length` is measured in; see
    // `plan_space_length`. `length_scratch` is the one state it writes group
    // columns into, reused rather than allocated per waypoint.
    let space = JointModelGroupSpace::new(&model, &group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));
    let mut length_scratch = template.clone();

    let mut total = 0usize;
    let mut solved_count = 0usize;
    let mut timeout_count = 0usize;
    let mut failure_count = 0usize;
    let mut condition2_checked = 0usize;
    let mut condition2_pass = 0usize;
    let mut waypoints_checked = 0usize;
    let mut raw_waypoints_total = 0usize;
    let mut max_endpoint_gap = 0f64;
    let mut cost_improved = 0usize;
    let mut cost_worsened = 0usize;
    let mut seed_costs: Vec<f64> = Vec::new();
    let mut output_costs: Vec<f64> = Vec::new();
    let mut seed_invalid = 0usize;
    let mut seed_cost_fn_invalid = 0usize;
    let mut cost_fn_missed_seed_collision = 0usize;
    let mut cost_fn_margin_only = 0usize;
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
        let start_state = state_from_map(&model, &start_map);
        let goal_state = state_from_map(&model, &goal_map);
        let start_column = positions(&start_state, group).expect("start positions");
        let goal_column = positions(&goal_state, group).expect("goal positions");

        // Upstream `getStompConfig` (`stomp_moveit_planning_context.cpp:191-207`)
        // over `stomp_moveit.yaml`'s declared defaults: num_iterations 1000,
        // num_iterations_after_valid 0, num_rollouts 15, max_rollouts 25,
        // num_timesteps 40, exponentiated_cost_sensitivity 0.5,
        // control_cost_weight 0.1, delta_t 0.1. `num_dimensions` and
        // `initialization_method` are overwritten by `plan` itself, so the
        // values given here are the ones upstream's own two overrides land on.
        let config = StompConfiguration {
            num_iterations: 1000,
            num_iterations_after_valid: 0,
            num_timesteps: 40,
            num_dimensions: group.active_joint_names().len(),
            delta_t: 0.1,
            initialization_method: TrajectoryInitialization::LinearInterpolation,
            exponentiated_cost_sensitivity: 0.5,
            num_rollouts: 15,
            max_rollouts: 25,
            control_cost_weight: 0.1,
        };

        let mut cost_scene = PlanningScene::new(&model, &srdf);
        cost_scene.set_current_state(start_state.clone());
        let cell = RefCell::new(&mut cost_scene);
        let build_cost_fn = || {
            let collision =
                cost_functions::get_collision_cost_function(&cell, &env, group, PENALTY)
                    .expect("collision cost function");
            match constraints.as_ref() {
                // Upstream `costs::sum({collision, constraints})` when the
                // request carries path constraints, collision alone otherwise
                // (`stomp_moveit_planning_context.cpp:163-172`).
                Some(set) => cost_functions::sum(vec![
                    collision,
                    cost_functions::get_constraints_cost_function(&cell, group, set, PENALTY)
                        .expect("constraints cost function"),
                ]),
                None => collision,
            }
        };

        // Both pure functions of start_column/goal_column (plus `config`,
        // fixed above and never touched by `plan`), computed here -- before
        // `plan` runs at all -- rather than after a successful outcome: see
        // `ProblemRecord`'s own doc. The same functional is called again
        // below on `output_matrix` once `plan` has returned: a `CostFn` is
        // `FnMut`, so the seed and the output are each evaluated through
        // their own instance rather than one shared across both.
        let seed_matrix =
            linear_interpolation_seed(&start_column, &goal_column, config.num_timesteps);
        let (seed_cost_columns, seed_cost_fn_valid) =
            build_cost_fn()(&seed_matrix).expect("cost of the seed trajectory");
        let seed_cost: f64 = seed_cost_columns.sum();
        let seed_columns: Vec<DVector<f64>> = (0..seed_matrix.ncols())
            .map(|t| seed_matrix.column(t).clone_owned())
            .collect();
        let seed_length = plan_space_length(&space, &mut length_scratch, group, &seed_columns);

        // `seed_valid` must mean the SAME thing here as in
        // `optimize_benchmark_chomp.rs`, because one gate check
        // (`nontrivial-population`) aggregates the field across both
        // instruments. It is the independent collision checker on the
        // densified seed -- not `seed_cost_fn_valid`, which is STOMP's own
        // cost functional and disagrees with the checker in both directions:
        // the cost carries a clearance margin the checker does not, and a
        // ported distance field could miss a collision the checker sees.
        // Fresh scene so the seed check cannot leave state in the one the cost
        // function closes over.
        let seed_dense = densify(
            &template,
            group,
            &[start_column.clone(), goal_column.clone()],
            resolution,
        );
        let mut seed_scene = PlanningScene::new(&model, &srdf);
        let seed_valid = seed_scene
            .is_path_valid(
                &env,
                &CollisionRequest::default(),
                &seed_dense,
                check_constraints.as_ref(),
                &[],
            )
            .valid;

        // Upstream's timeout watcher (`stomp_moveit_planning_context.cpp:247-257`):
        // cancel from outside after the allowed time, then classify the run as
        // timed out if the watcher is what ended it. The barrier makes the
        // watchdog's clock start when `plan` does, not when the thread was
        // spawned, so a slow scene build cannot eat the budget.
        let handle = CancelHandle::new();
        let watch_handle = handle.clone();
        let fired = Arc::new(AtomicBool::new(false));
        let watch_fired = Arc::clone(&fired);
        let finished = Arc::new(AtomicBool::new(false));
        let watch_finished = Arc::clone(&finished);
        let barrier = Arc::new(Barrier::new(2));
        let watch_barrier = Arc::clone(&barrier);
        let watchdog = std::thread::spawn(move || {
            watch_barrier.wait();
            let deadline = Instant::now() + Duration::from_secs_f64(timeout_seconds);
            while Instant::now() < deadline {
                if watch_finished.load(Ordering::SeqCst) {
                    return;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            if !watch_finished.load(Ordering::SeqCst) {
                watch_fired.store(true, Ordering::SeqCst);
                watch_handle.cancel();
            }
        });

        barrier.wait();
        let t0 = Instant::now();
        let outcome = plan(
            config,
            build_cost_fn(),
            PlanRequest {
                start_state: &start_state,
                goal_state: &goal_state,
                group,
                input_trajectory: None,
            },
            ChaCha8Rng::seed_from_u64(seed_base.wrapping_add(id)),
            handle,
        );
        let elapsed = t0.elapsed().as_secs_f64();
        finished.store(true, Ordering::SeqCst);
        watchdog.join().expect("watchdog thread must not panic");
        let timed_out = fired.load(Ordering::SeqCst);
        if elapsed > slowest.0 {
            slowest = (elapsed, id);
        }

        // A fired watchdog is a failure even when a trajectory came back:
        // a cancelled `Stomp::solve` returns its seed, so counting that as a
        // solve would report the linear interpolation as STOMP's output.
        let trajectory = match outcome {
            Ok(Some(trajectory)) if !timed_out => trajectory,
            other => {
                let outcome_name = if timed_out {
                    timeout_count += 1;
                    "timeout"
                } else if matches!(other, Ok(None)) {
                    failure_count += 1;
                    "not_optimized"
                } else {
                    failure_count += 1;
                    "error"
                };
                let detail = match other {
                    Err(e) => e.to_string(),
                    Ok(_) => String::from("plan returned no trajectory"),
                };
                println!(
                    "{}",
                    ProblemRecord {
                        id,
                        solved: false,
                        outcome: outcome_name,
                        plan_seconds: elapsed,
                        failure: Some(detail),
                        condition2_valid: None,
                        condition2_valid_at_returned_waypoints: None,
                        waypoints_checked: None,
                        raw_waypoints: None,
                        start_gap: None,
                        goal_gap: None,
                        invalid_waypoint_count: None,
                        invalid_waypoints: None,
                        length: None,
                        output_cost: None,
                        output_cost_fn_valid: None,
                        seed_cost,
                        seed_length,
                        seed_valid,
                        seed_cost_fn_valid,
                    }
                    .to_json()
                );
                continue;
            }
        };

        solved_count += 1;
        let timed = trajectory
            .into_uniformly_timed(config.delta_t)
            .expect("uniform timing of a returned trajectory");
        let output_matrix =
            robot_trajectory_to_matrix(&timed, group).expect("returned trajectory to matrix");

        // The same functional the seed was evaluated through above, built
        // fresh because a `CostFn` is `FnMut`: evaluating the output through
        // the seed's own instance is not possible, and reusing one instance
        // across the two evaluations would let any internal state carry from
        // one into the other.
        let (output_cost_columns, output_cost_fn_valid) =
            build_cost_fn()(&output_matrix).expect("cost of the returned trajectory");
        let output_cost: f64 = output_cost_columns.sum();
        if output_cost < seed_cost {
            cost_improved += 1;
        } else if output_cost > seed_cost {
            cost_worsened += 1;
        }
        if !seed_cost_fn_valid {
            seed_cost_fn_invalid += 1;
        }
        seed_costs.push(seed_cost);
        output_costs.push(output_cost);

        if !seed_valid {
            seed_invalid += 1;
            // The falsifiable half of the seed-versus-output pairing: a
            // collision the checker sees on the seed MUST make STOMP's own
            // cost positive there. Nothing in the solver forces this -- unlike
            // `output_cost`, which `solve`'s return value pins to 0 (it returns
            // `parameters_valid`, and `cost_functions.rs:174` sets that false
            // for any positive column) -- so this is the one paired quantity
            // whose passing is not a restatement of the termination rule.
            if seed_cost <= 0.0 {
                cost_fn_missed_seed_collision += 1;
            }
        } else if seed_cost > 0.0 {
            // Legitimate, hence counted and reported rather than gated: the
            // collision cost is a clearance potential, so a seed that clears
            // every mesh but passes inside the margin scores above zero.
            cost_fn_margin_only += 1;
        }

        let mut path: Vec<DVector<f64>> = (0..output_matrix.ncols())
            .map(|t| output_matrix.column(t).clone_owned())
            .collect();
        let raw_waypoints = path.len();
        let start_gap = endpoint_gap(&path[0], &start_column);
        let goal_gap = endpoint_gap(&path[raw_waypoints - 1], &goal_column);
        max_endpoint_gap = max_endpoint_gap.max(start_gap).max(goal_gap);
        let length = plan_space_length(&space, &mut length_scratch, group, &path);

        // After the length and the endpoint gaps, so both still describe what
        // the planner actually returned.
        if let Some(bad) = &injected {
            let mid = path.len() / 2;
            path.insert(mid, bad.clone());
        }

        let raw = returned_waypoints(&template, group, &path);
        let raw_validity = cell.borrow_mut().is_path_valid(
            &env,
            &CollisionRequest::default(),
            &raw,
            check_constraints.as_ref(),
            &[],
        );
        let dense = densify(&template, group, &path, resolution);
        let validity = cell.borrow_mut().is_path_valid(
            &env,
            &CollisionRequest::default(),
            &dense,
            check_constraints.as_ref(),
            &[],
        );
        condition2_checked += 1;
        waypoints_checked += dense.len();
        raw_waypoints_total += raw_waypoints;
        if validity.valid {
            condition2_pass += 1;
        }

        let mut line = ProblemRecord {
            id,
            solved: true,
            outcome: "solved",
            plan_seconds: elapsed,
            failure: None,
            condition2_valid: Some(validity.valid),
            condition2_valid_at_returned_waypoints: Some(raw_validity.valid),
            waypoints_checked: Some(dense.len()),
            raw_waypoints: Some(raw_waypoints),
            start_gap: Some(start_gap),
            goal_gap: Some(goal_gap),
            invalid_waypoint_count: Some(validity.invalid_waypoints.len()),
            invalid_waypoints: Some(validity.invalid_waypoints.clone()),
            length: Some(length),
            output_cost: Some(output_cost),
            output_cost_fn_valid: Some(output_cost_fn_valid),
            seed_cost,
            seed_length,
            seed_valid,
            seed_cost_fn_valid,
        }
        .to_json();
        if emit_dense {
            let waypoints: Vec<serde_json::Value> = dense
                .iter()
                .map(|rs| {
                    let map: BTreeMap<&str, f64> = variable_names
                        .iter()
                        .map(|name| {
                            let value = rs
                                .variable_position(name)
                                .unwrap_or_else(|e| panic!("variable_position({name}): {e}"));
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

    let wall_clock = run_start.elapsed().as_secs_f64();
    println!(
        "{}",
        serde_json::json!({
            "summary": {
                "planner": "stomp",
                "robot": robot,
                "group": group_name,
                "config": request["config"],
                "seed_base": seed_base,
                "request_seed": request["seed"],
                "timeout_seconds": timeout_seconds,
                "joint_constraint": constraint_spec,
                "check_joint_constraint": check_constraint_spec,
                "inject": inject.map(InjectMode::as_str),
                "problems": total,
                "solved": solved_count,
                "timeouts": timeout_count,
                "failures": failure_count,
                "condition2_checked": condition2_checked,
                "condition2_pass": condition2_pass,
                "waypoints_checked": waypoints_checked,
                "raw_waypoints": raw_waypoints_total,
                "max_endpoint_gap": max_endpoint_gap,
                "cost_improved": cost_improved,
                "cost_worsened": cost_worsened,
                "seed_invalid": seed_invalid,
                "seed_cost_fn_invalid": seed_cost_fn_invalid,
                "cost_fn_missed_seed_collision": cost_fn_missed_seed_collision,
                "cost_fn_margin_only": cost_fn_margin_only,
                "paired_seed_cost_median": median(seed_costs.clone()),
                "paired_output_cost_median": median(output_costs.clone()),
                "wall_clock_seconds": wall_clock,
                "slowest_seconds": slowest.0,
                "slowest_problem_id": if slowest.1 == u64::MAX { None } else { Some(slowest.1) },
            }
        })
    );

    eprintln!(
        "planner=stomp robot={robot} problems={total} solved={solved_count} \
         timeouts={timeout_count} failures={failure_count} \
         cond2={condition2_pass}/{condition2_checked} \
         cost_improved={cost_improved} cost_worsened={cost_worsened} \
         seed_invalid={seed_invalid} seed_cost_fn_invalid={seed_cost_fn_invalid} \
         cost_fn_missed_seed_collision={cost_fn_missed_seed_collision} \
         cost_fn_margin_only={cost_fn_margin_only} \
         max_endpoint_gap={max_endpoint_gap} \
         wall_clock={wall_clock:.1}s slowest={:.1}s",
        slowest.0
    );

    if let Some(mode) = inject {
        let mode = mode.as_str();
        // Three assertions, for the reason `plan_benchmark_port`'s own
        // injection mode carries: `condition2_pass == 0` is satisfied
        // vacuously by a run that solved nothing, so a run whose planner never
        // produced a path would otherwise vouch for a checker it never called,
        // and the third one keeps the checked set from drifting away from the
        // injected population without saying so.
        assert!(
            condition2_checked > 0,
            "inject={mode} solved no problem, so `is_path_valid` was never called -- an \
             injection run that checks nothing cannot show that the validity check rejects \
             a bad waypoint"
        );

        // The rejection assertion below runs over the paths the checker saw,
        // and that set is not the injected population -- a problem that times
        // out or fails never reaches `is_path_valid`. Closing the accounting
        // keeps the narrowing visible: every injected problem is checked,
        // timed out, or failed, so a later edit that lets a solved path skip
        // the checker fails here instead of quietly shrinking the set the
        // "rejected all" line reports on.
        assert_eq!(
            condition2_checked + timeout_count + failure_count,
            total,
            "inject={mode} accounts for {condition2_checked} checked + {timeout_count} timeout \
             + {failure_count} failure, which is not the {total} injected -- a problem in no \
             bucket left the population the rejection assertion reports on"
        );
        assert_eq!(
            condition2_pass, 0,
            "inject={mode} spliced a state verified invalid by direct query into every solved \
             path, but is_path_valid still passed {condition2_pass}/{condition2_checked} of \
             them -- the validity check is not checking what it reports on"
        );
        // `condition2_checked` is the numerator, and printing it alone reads as
        // the population: "rejected all 105 paths" says nothing about the 125
        // that were injected. A problem that times out or errors never has its
        // spliced waypoint checked, so it silently leaves the set the assertion
        // runs over -- measured at 105 of 125 for STOMP and 85 of 125 for CHOMP
        // on one 125-problem set. The denominator goes in the line so the gate
        // cannot narrow without saying by how much.
        let not_checked = total - condition2_checked;
        eprintln!(
            "inject={mode} rejected all {condition2_checked} checked paths of {total} injected, \
             as required; {not_checked} not checked ({timeout_count} timeout, \
             {failure_count} failure)"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returned_waypoints_emits_exactly_one_state_per_column_with_no_interpolation() {
        // The property that distinguishes `returned_waypoints` from `densify`:
        // same input, but `densify` inserts extra points between every pair
        // (`out.push` runs `steps` times per `windows(2)` pair, `steps >= 1`),
        // while `returned_waypoints` must produce exactly `path.len()` states,
        // each one a direct round-trip of its input column -- otherwise the
        // "no interpolation at all" claim in its own doc is untested.
        let (model, _srdf) = load_robot("panda");
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut template = RobotState::new(&model);
        template.set_to_default_values();
        let path = vec![
            DVector::from_vec(vec![0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]),
            DVector::from_vec(vec![0.3, -0.4, 0.1, -1.4, 0.05, 1.05, 0.55]),
            DVector::from_vec(vec![0.6, -0.3, 0.2, -1.3, 0.1, 1.1, 0.6]),
        ];

        let raw = returned_waypoints(&template, group, &path);

        assert_eq!(raw.len(), path.len());
        for (state, column) in raw.iter().zip(path.iter()) {
            let got = positions(state, group).expect("panda_arm positions");
            assert_eq!(got, *column);
        }

        // Contrast: the same path through `densify` at a resolution finer
        // than any consecutive-column delta produces MORE than `path.len()`
        // states -- confirming the two functions actually differ, not just
        // that this test only exercises the trivial resolution=infinity case.
        let dense = densify(&template, group, &path, 0.05);
        assert!(
            dense.len() > path.len(),
            "densify at resolution 0.05 produced {} states from {} raw waypoints -- expected \
             interpolation to add some",
            dense.len(),
            path.len()
        );
    }

    #[test]
    fn condition2_valid_at_returned_waypoints_is_true_when_densified_is_false() {
        // The scenario `condition2_valid_at_returned_waypoints` exists to
        // attribute: two raw waypoints that do not themselves collide, but
        // whose straight-line interpolation sweeps the arm through an
        // obstacle strictly between them. `is_path_valid` on
        // `returned_waypoints`'s output must stay `true` while the same call
        // on `densify`'s output goes `false` -- if both moved together, the
        // new field would just restate `condition2_valid` under a second
        // name.
        let (model, srdf) = load_robot("panda");
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut template = RobotState::new(&model);
        template.set_to_default_values();

        // A real problem's start/goal (`doc/phase8-baseline-500/
        // floor_wall.250.900001.set.json`, problem id 1), not a synthetic
        // all-other-joints-at-default pose: panda's zero configuration is
        // itself self-colliding (measured -- every waypoint of an all-default
        // sweep was invalid even against an empty world), so a fabricated
        // two-point path risks testing self-collision instead of the wall.
        // Problem id 1 specifically, not id 0: this instrument's own
        // `seed_valid` field (computed the same way, over the same set)
        // already measured id 0's straight-line seed as collision-free and
        // id 1's as colliding, so id 1 is the one whose densified
        // interpolation is known to cross an obstacle somewhere between two
        // otherwise-valid endpoints.
        let start = state_from_map(
            &model,
            &BTreeMap::from([
                ("panda_joint1".to_string(), -0.12569973212653407),
                ("panda_joint2".to_string(), 0.8539681979683693),
                ("panda_joint3".to_string(), 2.784858631146275),
                ("panda_joint4".to_string(), -2.829531705878982),
                ("panda_joint5".to_string(), 0.7209874782302768),
                ("panda_joint6".to_string(), 0.9767144543011761),
                ("panda_joint7".to_string(), -2.63052329696014),
            ]),
        );
        let goal = state_from_map(
            &model,
            &BTreeMap::from([
                ("panda_joint1".to_string(), -1.0206841738437522),
                ("panda_joint2".to_string(), -1.507401901709096),
                ("panda_joint3".to_string(), -1.3901007965446428),
                ("panda_joint4".to_string(), -0.8806967682840048),
                ("panda_joint5".to_string(), -1.2343998073850808),
                ("panda_joint6".to_string(), 2.111189573531795),
                ("panda_joint7".to_string(), 2.0759920810853454),
            ]),
        );
        let start_column = positions(&start, group).expect("start positions");
        let goal_column = positions(&goal, group).expect("goal positions");
        let path = vec![start_column, goal_column];

        // The same floor + wall obstacles `doc/phase8-baseline-500/
        // floor_wall.250.900001.set.json` places for this problem.
        let mut world = World::new();
        world.add_shape(
            "floor",
            Arc::new(Shape::Cuboid(Cuboid::new(2.0, 2.0, 0.5).unwrap())),
            Isometry3::translation(0.0, 0.0, -0.28),
        );
        world.add_shape(
            "wall",
            Arc::new(Shape::Cuboid(Cuboid::new(0.05, 1.6, 1.6).unwrap())),
            Isometry3::translation(0.45, 0.0, 0.8),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let mut scene = PlanningScene::new(&model, &srdf);

        let raw = returned_waypoints(&template, group, &path);
        let raw_validity = scene.is_path_valid(&env, &CollisionRequest::default(), &raw, None, &[]);
        let dense = densify(&template, group, &path, 0.05);
        let dense_validity =
            scene.is_path_valid(&env, &CollisionRequest::default(), &dense, None, &[]);

        assert!(
            raw_validity.valid,
            "problem id 1's own start and goal must not collide with floor_wall on their own -- \
             got invalid_waypoints={:?}",
            raw_validity.invalid_waypoints
        );
        assert!(
            !dense_validity.valid,
            "problem id 1's straight-line interpolation is already known invalid (this \
             instrument's own seed_valid measured it colliding), so the densified path must \
             disagree with the raw one -- got both valid"
        );
    }
}
