// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: like its sibling `chomp_benchmark_port`, this is Phase 8
// benchmark infrastructure, not a port. Upstream reaches `ChompPlanner::solve`
// only through `CHOMPPlanningContext`'s pluginlib entry point and ships no
// binary that runs it over a problem set.

//! CHOMP half of the Phase 8 property instrument: runs
//! [`moveit_planners_chomp::solve`] over the *same* problem set
//! `moveit-planners-sbp`'s `plan_benchmark_problem_set` emits for Phase 7, and
//! reports the quantities `tools/ci/measure-phase8-optimizer-properties.sh`
//! gates on.
//!
//! See `optimize_benchmark_stomp`'s module doc (and `PORTING-PLAN.md`'s Phase 8
//! property section) for which of Phase 7's checks transfer to an optimizing
//! planner and which do not; the reasoning is the same for both planners and is
//! not repeated here. What is specific to CHOMP:
//!
//! # This binary supplies `mesh_to_mesh_collision_free` for the first time
//!
//! [`moveit_planners_chomp::solve`] takes upstream's
//! `isCurrentTrajectoryMeshToMeshCollisionFree` (`chomp_optimizer.cpp:520-537`)
//! as an injected closure rather than a method, because wiring it inside the
//! crate would make `moveit-planners-chomp` depend on `moveit-scene` and
//! `moveit-collision` (`optimizer`'s own doc, "`isCurrentTrajectoryMeshToMesh
//! CollisionFree` becomes an injected closure"). Every caller in the tree so far
//! passes `|_, _| false`, which is upstream's *never-collision-free* branch: it
//! makes `is_collision_free_` reachable only through the sphere-approximated
//! `collision_cost < collision_threshold_` comparison. This binary is a caller
//! that does have both crates, so it passes the real check --
//! `PlanningScene::is_path_valid` over the group trajectory matrix, exactly
//! what upstream's method calls. A run therefore exercises the early-break
//! path upstream has and no in-crate test can reach.
//!
//! # The environment distance field is centered, not origin-cornered
//!
//! Upstream's CHOMP plugin allocates `CollisionDetectorAllocatorHybrid`
//! (`chomp_plugin.cpp:95`) with `CollisionEnvHybrid`'s defaults: size
//! `3 x 3 x 4`, resolution `0.02`, `max_propagation_distance 0.25`, and origin
//! `(0, 0, 0)` (`collision_env_distance_field.hpp:49-55`,
//! `collision_env_hybrid.hpp:53-61`). This binary keeps the size, resolution
//! and propagation distance and *moves the origin* so the grid is centered on
//! the robot base. With upstream's own origin the grid spans `[0,3] x [0,3] x
//! [0,4]`, so every obstacle at negative `x` or `y` -- the cage's west and
//! south walls, and three quarters of the floor -- falls outside it, where
//! [`DistanceField::distance`] returns `uninitialized_distance`, i.e. "no
//! obstacle". Replicating that origin would produce a measurement whose
//! obstacle set is silently a quarter of the one the problem set declares.
//! The grid is scaled by the request's own `scale` for the same reason the
//! problem set scales its obstacles by it.
//!
//! Usage: `optimize_benchmark_chomp <seed_base> [timeout_seconds] [inject]
//! [dense]`, with the problem-set JSON on stdin.

use std::collections::BTreeMap;
use std::env;
use std::io::{self, Read};
use std::sync::Arc;
use std::time::Instant;

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use moveit_distance_field::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GridGeometry,
    PropagationDistanceField, add_link_body_decompositions, collision_object_point_decomposition,
};
use moveit_geometry::{Cuboid, Isometry3, Shape, Vector3};
use moveit_model::{JointModelGroup, MeshSearchPaths, RobotModel};
use moveit_planners_chomp::optimizer::ChompCollisionContext;
use moveit_planners_chomp::{ChompGoal, ChompParameters, ChompRequest, GoalJointConstraint, solve};
use moveit_planners_sbp::{CompoundValue, JointModelGroupSpace, StateSpace};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::DMatrix;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// `CollisionEnvDistanceField`'s own `DEFAULT_RESOLUTION`
/// (`collision_env_distance_field.hpp:53`), the value upstream's CHOMP plugin
/// gets by not overriding it.
const DF_RESOLUTION: f64 = 0.02;

/// `DEFAULT_MAX_PROPOGATION_DISTANCE` (`collision_env_distance_field.hpp:55`),
/// upstream's spelling included.
const DF_MAX_PROPAGATION: f64 = 0.25;

/// `DEFAULT_SIZE_X`/`_Y`/`_Z` (`collision_env_distance_field.hpp:49-51`).
const DF_SIZE: (f64, f64, f64) = (3.0, 3.0, 4.0);

/// Tolerance on every goal joint constraint this binary builds. The problem set
/// carries a bare goal *configuration*, not a tolerance, and CHOMP needs a
/// tolerance because `solve` returns `GoalConstraintsViolated` when the final
/// trajectory point misses it. `0.01` rad matches the value
/// `moveit-planners-chomp`'s own `planner` tests use; every run reports its
/// measured `goal_gap` so this constant can be checked against what the
/// optimizer actually leaves at the goal end.
const GOAL_TOLERANCE: f64 = 0.01;

/// The wall-clock bound per `solve` call when no third argument is given.
/// `ChompParameters::planning_time_limit` bounds CHOMP's own iteration loop, so
/// unlike STOMP this is a *second*, outer bound that should never fire; it is
/// here so that a call which does hang is a counted failure rather than a
/// stalled run. Matching `plan_benchmark_port`'s default keeps the two
/// benchmarks bounded identically.
const DEFAULT_TIMEOUT_SECONDS: f64 = 120.0;

/// How many rejection-sampling attempts `build_injected_state` gets.
const MAX_INJECT_SEARCH_ATTEMPTS: usize = 100_000;

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
    assert_eq!(flat.len(), 16, "expected a flat 4x4 matrix, got {flat:?}");
    Isometry3::translation(flat[3], flat[7], flat[11])
}

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

/// Writes row `point` of a CHOMP group-trajectory matrix (`num_points` rows by
/// `num_joints` columns, columns in `active_joint_names` order -- `trajectory`'s
/// own module doc and `ChompOptimizer::new`'s `joint_names`) into a copy of
/// `template`, with every *non*-group variable taken from `other`.
///
/// `other` is copied by value rather than cloned into the result because its
/// lifetime is the caller's, not `template`'s: the closure
/// `mesh_to_mesh_collision_free` receives its `RobotState` under an anonymous
/// lifetime, while `PlanningScene::is_path_valid` needs its waypoints under the
/// scene's own model lifetime. Copying the positions keeps upstream's
/// `start_state_` contribution (`chomp_optimizer.cpp:534`) without tying the two
/// lifetimes together.
fn state_from_row<'m>(
    template: &RobotState<'m>,
    other: &RobotState<'_>,
    group: &JointModelGroup,
    matrix: &DMatrix<f64>,
    point: usize,
) -> RobotState<'m> {
    let mut state = template.clone();
    state.set_variable_positions(other.positions());
    for (j, name) in group.active_joint_names().iter().enumerate() {
        state
            .set_variable_position(name, matrix[(point, j)])
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    state.update();
    state
}

/// Reads every active-joint position of `state` in `active_joint_names` order.
fn columns_of(state: &RobotState<'_>, group: &JointModelGroup) -> Vec<f64> {
    group
        .active_joint_names()
        .iter()
        .map(|name| {
            state
                .variable_position(name)
                .unwrap_or_else(|e| panic!("variable_position({name}): {e}"))
        })
        .collect()
}

/// Interpolates `path` so that no active joint moves by more than `resolution`
/// between two consecutive checked states -- the same per-joint rule, for the
/// same reason, as `optimize_benchmark_stomp`'s `densify`.
fn densify<'m>(
    template: &RobotState<'m>,
    group: &JointModelGroup,
    path: &[Vec<f64>],
    resolution: f64,
) -> Vec<RobotState<'m>> {
    let write = |values: &[f64]| {
        let mut state = template.clone();
        for (name, value) in group.active_joint_names().iter().zip(values) {
            state
                .set_variable_position(name, *value)
                .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
        }
        state.update();
        state
    };
    let mut out = vec![write(&path[0])];
    for pair in path.windows(2) {
        let (from, to) = (&pair[0], &pair[1]);
        let widest = from
            .iter()
            .zip(to)
            .map(|(a, b)| (b - a).abs())
            .fold(0.0f64, f64::max);
        let steps = ((widest / resolution).ceil() as u64).max(1);
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mixed: Vec<f64> = from.iter().zip(to).map(|(a, b)| a + (b - a) * t).collect();
            out.push(write(&mixed));
        }
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InjectMode {
    Collision,
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

/// Finds a state the requested check *actually rejects*, by asking the check.
fn build_injected_state(
    mode: InjectMode,
    model: &RobotModel,
    srdf: &SrdfModel,
    group: &JointModelGroup,
    env: &ParryCollisionEnv,
    constraints: Option<&KinematicConstraintSet>,
    rng: &mut ChaCha8Rng,
) -> Vec<f64> {
    let mut scene = PlanningScene::new(model, srdf);
    let mut template = RobotState::new(model);
    template.set_to_default_values();
    let bounds: Vec<(f64, f64)> = group
        .active_joint_names()
        .iter()
        .map(|name| {
            let joint = model
                .joint_model(name)
                .unwrap_or_else(|e| panic!("joint_model({name}): {e}"));
            let bound = joint.variable_bounds()[0];
            assert!(
                bound.min_position.is_finite() && bound.max_position.is_finite(),
                "{name} has unbounded position, so a uniform sample over it is not defined"
            );
            (bound.min_position, bound.max_position)
        })
        .collect();

    for _ in 0..MAX_INJECT_SEARCH_ATTEMPTS {
        let column: Vec<f64> = bounds
            .iter()
            .map(|(lo, hi)| rng.random_range(*lo..*hi))
            .collect();
        let mut candidate = template.clone();
        for (name, value) in group.active_joint_names().iter().zip(&column) {
            candidate
                .set_variable_position(name, *value)
                .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
        }
        candidate.update();
        scene.set_current_state(candidate);
        let rejected = match mode {
            InjectMode::Collision => !scene.is_state_valid(env, &CollisionRequest::default(), None),
            InjectMode::Constraint => {
                let set = constraints.expect(
                    "inject=constraint needs the request to carry a joint_constraint, else there \
                     is no constraint for the spliced state to violate",
                );
                !scene.is_state_constrained(set)
            }
        };
        if rejected {
            return column;
        }
    }
    panic!(
        "no state rejected by inject={} found in {MAX_INJECT_SEARCH_ATTEMPTS} attempts -- an \
         injection run cannot prove the check fires without one",
        mode.as_str()
    );
}

fn max_abs_gap(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0f64, f64::max)
}

/// The length of `path` in the plan-space metric --
/// [`JointModelGroupSpace::distance`] summed along it, which is what the
/// oracle's `chomp_plan`/`stomp_plan` report as their own `length`
/// (`oracle.cpp`'s `planSpacePathLength`, summing OMPL
/// `CompoundStateSpace::distance`).
///
/// Not Euclidean L2 over raw joint values, which this instrument reported
/// before and which is a *different quantity*: both spaces weight each bounded
/// axis by `1/(max - min)` and add the weighted absolute differences, so an L2
/// length and a plan-space length are not two estimates of one number. Phase
/// 7's `condition3-*` divides the port's length by the C++ side's, and that
/// ratio means nothing unless both sides measure in the same metric.
/// `tests/plan_space_parity.rs` is what makes this one metric rather than two
/// that agree by argument.
fn plan_space_length(
    space: &JointModelGroupSpace,
    scratch: &mut RobotState<'_>,
    group: &JointModelGroup,
    path: &[Vec<f64>],
) -> f64 {
    let states: Vec<Vec<CompoundValue>> = path
        .iter()
        .map(|row| {
            for (name, value) in group.active_joint_names().iter().zip(row) {
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
    let scale = request["scale"].as_f64().unwrap_or(1.0);

    let (model, srdf) = load_robot(&robot);
    let group = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));
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

    // `ChompRequest` carries goal constraints and no path constraints, so for
    // CHOMP the request's `joint_constraint` was only ever the *checker's*
    // constraint -- there is no planner-side constraint set to build here at
    // all. It gets the same second field the STOMP half needs anyway, so the
    // gate has one rule it can apply to either planner rather than a field that
    // exists on one side. STOMP's own comment says why the split is forced
    // there: a planned joint constraint makes every STOMP timestep invalid.
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

    // See this file's "# The environment distance field is centered" for why
    // the origin is not upstream's `(0, 0, 0)`, and why the extent scales.
    let df_size = Vector3::new(DF_SIZE.0 * scale, DF_SIZE.1 * scale, DF_SIZE.2 * scale);
    let df_origin = Vector3::new(-0.5 * df_size.x, -0.5 * df_size.y, -0.5 * scale);
    let df_config = DistanceFieldConfig {
        geometry: GridGeometry::new(df_size, df_origin, DF_RESOLUTION)
            .unwrap_or_else(|e| panic!("GridGeometry::new: {e}")),
        max_propagation_distance: DF_MAX_PROPAGATION,
        use_signed_distance_field: false,
    };

    let field_start = Instant::now();
    let mut env_field = PropagationDistanceField::new(
        df_config.geometry,
        df_config.max_propagation_distance,
        df_config.use_signed_distance_field,
    )
    .unwrap_or_else(|e| panic!("PropagationDistanceField::new: {e}"));
    let mut obstacle_points = 0usize;
    for (_, object) in world.iter() {
        let decomposition = collision_object_point_decomposition(object, DF_RESOLUTION)
            .unwrap_or_else(|e| panic!("collision_object_point_decomposition: {e}"));
        let points = decomposition.collision_points();
        obstacle_points += points.len();
        env_field.add_points_to_field(&points);
    }
    let field_seconds = field_start.elapsed().as_secs_f64();

    let decompositions =
        add_link_body_decompositions(&model, DF_RESOLUTION, &LinkPaddingScale::new(), None)
            .unwrap_or_else(|e| panic!("add_link_body_decompositions: {e}"));
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let injected: Option<Vec<f64>> = inject.map(|mode| {
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

    let params = ChompParameters::default();
    let mut total = 0usize;
    let mut solved_count = 0usize;
    let mut timeout_count = 0usize;
    let mut failure_count = 0usize;
    let mut condition2_checked = 0usize;
    let mut condition2_pass = 0usize;
    let mut waypoints_checked = 0usize;
    let mut raw_waypoints_total = 0usize;
    let mut max_endpoint_gap = 0f64;
    let mut mesh_check_calls = 0usize;
    let mut mesh_check_true = 0usize;
    let mut seed_invalid_count = 0usize;
    let mut seed_lengths: Vec<f64> = Vec::new();
    let mut output_lengths: Vec<f64> = Vec::new();
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
        let start_column = columns_of(&start_state, group);
        let goal_column = columns_of(&goal_state, group);

        let goal = ChompGoal {
            joint_constraints: group
                .active_joint_names()
                .iter()
                .zip(&goal_column)
                .map(|(name, position)| GoalJointConstraint {
                    joint_name: name.clone(),
                    position: *position,
                    tolerance_above: GOAL_TOLERANCE,
                    tolerance_below: GOAL_TOLERANCE,
                    weight: 1.0,
                })
                .collect(),
        };

        let mut cache = DistanceFieldCollisionCache::new(
            decompositions.clone(),
            df_config,
            /* collision_tolerance = */ 0.0,
        );
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &env_field,
        };
        let mut mesh_scene = PlanningScene::new(&model, &srdf);
        let mesh_template = template.clone();
        let mut calls = 0usize;
        let mut trues = 0usize;
        // Scoped so the closure's `&mut calls`/`&mut trues` borrows end before
        // the counters are read; the block yields what the run produced.
        let (outcome, elapsed) = {
            // Upstream's `isCurrentTrajectoryMeshToMeshCollisionFree`, supplied for
            // real: `isPathValid` over the matrix rows, in the same
            // `(point, joint)` order upstream reads `best_group_trajectory_(i, j)`.
            let mut mesh_to_mesh = |state: &RobotState<'_>, matrix: &DMatrix<f64>| {
                calls += 1;
                let waypoints: Vec<RobotState<'_>> = (0..matrix.nrows())
                    .map(|point| state_from_row(&mesh_template, state, group, matrix, point))
                    .collect();
                // No constraints, and not an oversight: upstream's method calls
                // `planning_scene_->isPathValid(trajectory, planning_group_)`, the
                // overload that takes a group name and no constraint set. Passing
                // the checker's constraints here would make CHOMP's own early-break
                // criterion stricter than upstream's and change how often the
                // optimizer stops.
                let valid = mesh_scene
                    .is_path_valid(&env, &CollisionRequest::default(), &waypoints, None, &[])
                    .valid;
                if valid {
                    trues += 1;
                }
                valid
            };

            let chomp_request = ChompRequest {
                start_state: &start_state,
                group_name: &group_name,
                goal_constraints: std::slice::from_ref(&goal),
                params: &params,
                seed_trajectory: None,
            };
            let mut rng = ChaCha8Rng::seed_from_u64(seed_base.wrapping_add(id));
            let t0 = Instant::now();
            let outcome = solve(
                &chomp_request,
                &mut collision,
                None,
                &mut mesh_to_mesh,
                &mut rng,
            );
            (outcome, t0.elapsed().as_secs_f64())
        };
        mesh_check_calls += calls;
        mesh_check_true += trues;
        if elapsed > slowest.0 {
            slowest = (elapsed, id);
        }
        // `ChompParameters::planning_time_limit` bounds CHOMP's own loop, so
        // this outer bound firing means the call did not return within a
        // budget it was supposed to respect. Counted as a failure, never as a
        // solve -- see `DEFAULT_TIMEOUT_SECONDS`. It is checked after the fact
        // rather than enforced by cancellation because `solve` exposes no
        // cancel handle: a call that hangs forever would hang this binary, and
        // that is a failure the gate reports as a stalled run, not a pass.
        let timed_out = elapsed > timeout_seconds;

        let solution = match outcome {
            Ok(solution) if !timed_out => solution,
            other => {
                let outcome_name = if timed_out {
                    timeout_count += 1;
                    "timeout"
                } else {
                    failure_count += 1;
                    "error"
                };
                let detail = match other {
                    Err(e) => e.to_string(),
                    Ok(_) => format!("solved in {elapsed}s, over the {timeout_seconds}s bound"),
                };
                println!(
                    "{}",
                    serde_json::json!({
                        "id": id, "solved": false, "outcome": outcome_name,
                        "plan_seconds": elapsed, "failure": detail,
                        "mesh_check_calls": calls,
                    })
                );
                continue;
            }
        };

        solved_count += 1;
        let trajectory = solution.trajectory;
        let mut path: Vec<Vec<f64>> = (0..trajectory.way_point_count())
            .map(|i| {
                columns_of(
                    trajectory
                        .way_point(i)
                        .unwrap_or_else(|e| panic!("way_point({i}): {e}")),
                    group,
                )
            })
            .collect();
        let raw_waypoints = path.len();
        let start_gap = max_abs_gap(&path[0], &start_column);
        let goal_gap = max_abs_gap(&path[raw_waypoints - 1], &goal_column);
        max_endpoint_gap = max_endpoint_gap.max(start_gap).max(goal_gap);
        let length = plan_space_length(&space, &mut length_scratch, group, &path);
        // CHOMP's own seed under the default `quintic-spline` initialization is
        // not a straight line, so the seed length is measured as the straight
        // line between the two endpoints: the shortest path any trajectory
        // between them can have in this metric, i.e. the strongest baseline the
        // output can be held against without re-deriving `fill_in_min_jerk`.
        let seed_length = plan_space_length(
            &space,
            &mut length_scratch,
            group,
            &[start_column.clone(), goal_column.clone()],
        );
        // No improved/worsened counters here, unlike the STOMP instrument:
        // CHOMP's own objective (smoothness + obstacle cost) is not returned by
        // `solve`, and path length is not it. On a problem CHOMP breaks out of
        // immediately the returned path IS the straight line, so a
        // `length > seed_length` counter reports a 1-ULP difference as a
        // regression. The ratio of the two medians is what the gate bands.
        seed_lengths.push(seed_length);
        output_lengths.push(length);

        // Is the straight line between the two endpoints ALREADY collision-free?
        // Without this the gate cannot tell an optimizer that moved a colliding
        // seed out of collision from one that returned its seed untouched
        // because the seed was already free -- and on this population the median
        // solved problem is the second kind, so the difference decides whether
        // any of the numbers above measure the optimizer at all. Densified with
        // the same rule as the output so the two answers are comparable.
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
        if !seed_valid {
            seed_invalid_count += 1;
        }

        if let Some(bad) = &injected {
            let mid = path.len() / 2;
            path.insert(mid, bad.clone());
        }

        let dense = densify(&template, group, &path, resolution);
        let mut check_scene = PlanningScene::new(&model, &srdf);
        let validity = check_scene.is_path_valid(
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

        let mut line = serde_json::json!({
            "id": id,
            "solved": true,
            "outcome": "solved",
            "plan_seconds": elapsed,
            "condition2_valid": validity.valid,
            "waypoints_checked": dense.len(),
            "raw_waypoints": raw_waypoints,
            "start_gap": start_gap,
            "goal_gap": goal_gap,
            "invalid_waypoint_count": validity.invalid_waypoints.len(),
            "invalid_waypoints": validity.invalid_waypoints,
            "length": length,
            "seed_length": seed_length,
            "seed_valid": seed_valid,
            "mesh_check_calls": calls,
            "mesh_check_true": trues,
        });
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
                "planner": "chomp",
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
                "mesh_check_calls": mesh_check_calls,
                "mesh_check_true": mesh_check_true,
                "seed_invalid": seed_invalid_count,
                "paired_seed_length_median": median(seed_lengths.clone()),
                "paired_output_length_median": median(output_lengths.clone()),
                "distance_field_seconds": field_seconds,
                "distance_field_points": obstacle_points,
                "distance_field_resolution": DF_RESOLUTION,
                "wall_clock_seconds": wall_clock,
                "slowest_seconds": slowest.0,
                "slowest_problem_id": if slowest.1 == u64::MAX { None } else { Some(slowest.1) },
            }
        })
    );

    eprintln!(
        "planner=chomp robot={robot} problems={total} solved={solved_count} \
         timeouts={timeout_count} failures={failure_count} \
         cond2={condition2_pass}/{condition2_checked} \
         mesh_check={mesh_check_true}/{mesh_check_calls} \
         seed_invalid={seed_invalid_count} \
         max_endpoint_gap={max_endpoint_gap} field={field_seconds:.1}s/{obstacle_points}pts \
         wall_clock={wall_clock:.1}s slowest={:.1}s",
        slowest.0
    );

    if let Some(mode) = inject {
        let mode = mode.as_str();
        assert!(
            condition2_checked > 0,
            "inject={mode} solved no problem, so `is_path_valid` was never called -- an \
             injection run that checks nothing cannot show that the validity check rejects a \
             bad waypoint"
        );
        assert_eq!(
            condition2_pass, 0,
            "inject={mode} spliced a state verified invalid by direct query into every solved \
             path, but is_path_valid still passed {condition2_pass}/{condition2_checked} of \
             them -- the validity check is not checking what it reports on"
        );
        eprintln!("inject={mode} rejected all {condition2_checked} paths, as required");
    }
}
