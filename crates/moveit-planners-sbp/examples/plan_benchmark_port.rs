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
//! -- <seed_base>`, with a `plan`-op request JSON on stdin (see
//! `examples/plan_benchmark_problem_set.rs`'s own doc comment for the exact
//! shape -- the same file `benches/sweep_baseline.sh` writes to
//! `$WORKDIR/$config.json` before piping it to the oracle is valid input
//! here too). `seed_base` is this run's own RNG seed -- independent of the
//! request's own `seed` field, which is the *oracle's* OMPL seed and has no
//! meaning to this crate's `ChaCha8Rng`-driven planner. Each problem's own
//! seed is `seed_base.wrapping_add(problem.id)`, so two runs over the same
//! request file with the same `seed_base` are reproducibly identical (this
//! binary always uses [`Termination::Iterations`], which is what makes
//! [`rrt_connect`](moveit_planners_sbp::rrt_connect)'s own determinism
//! guarantee apply).
//!
//! Prints one NDJSON line per problem to stdout:
//! `{"id", "solved", "length"?, "condition2_valid"?, "invalid_waypoint_count"?, "failure"?}`.
//! `length` is this crate's own [`StateSpace::distance`] summed along the
//! returned path -- directly comparable to the oracle response's own
//! `length` field, since `tests/plan_space_parity.rs` already establishes
//! bit-exact parity between this crate's `JointModelGroupSpace` and the
//! oracle's OMPL space. `condition2_valid`/`invalid_waypoint_count` are
//! Phase 7 condition 2's per-problem verdict -- see `# Condition 2's
//! collision-check resolution` below.
//!
//! # Condition 2's collision-check resolution
//!
//! Phase 7 condition 2 requires "100% of produced port paths pass
//! `moveit-scene`'s collision/constraint checks".
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
//! planning, then calls `is_path_valid` on that dense list. This is a
//! deliberate choice, not a default: it re-derives no new information
//! `DiscreteMotionValidator`'s own bisection did not already establish by
//! construction (that type checks every sample index down to `resolution`
//! spacing, not a subsample of them -- see its own doc comment), so what
//! condition 2 actually verifies here is that `is_path_valid`'s independent
//! code path (`PlanningScene::is_state_valid` on each dense waypoint)
//! agrees with `DiscreteMotionValidator`'s
//! (`PlanningSceneValidityChecker::is_valid` during planning) -- an
//! independent-implementation-path cross-check against a planner-side
//! plumbing bug, not a search for finer-than-planning collision gaps. A
//! resolution finer than `motion_resolution` would also find genuine
//! sub-resolution collision gaps neither the planner nor this check has any
//! way to see at their shared resolution -- a real limitation of
//! resolution-discretized collision checking in general (shared with
//! upstream's own analogous discrete motion validators), not something this
//! binary's choice of resolution introduces.

use std::collections::BTreeMap;
use std::env;
use std::io::{self, Read};
use std::sync::Arc;

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_sbp::{
    CompoundValue, JointModelGroupSpace, PlannerManager, PlanningRequest, RrtConnectManager,
    RrtConnectParams, StateSpace, Termination,
};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

/// The `moveit_resources_panda_description` package committed under
/// `fixtures/meshes/` -- same pattern as `plan_benchmark_problem_set.rs`.
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

fn main() {
    let args: Vec<String> = env::args().collect();
    let seed_base: u64 = args
        .get(1)
        .unwrap_or_else(|| panic!("usage: <seed_base>; got {args:?}"))
        .parse()
        .expect("seed_base must be a u64");

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
    let step_size = request["range"]
        .as_f64()
        .expect("request.range must be a number");
    let max_iterations = request["max_iterations"]
        .as_u64()
        .expect("request.max_iterations must be a number") as usize;

    let (model, srdf) = load_panda();
    let space = JointModelGroupSpace::new(&model, &group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));

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

    let manager = RrtConnectManager;
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

        let mut scene = PlanningScene::new(&model, &srdf);
        let start_state = joint_map_to_state(&space, &model, &start_map);
        let goal_state = joint_map_to_state(&space, &model, &goal_map);
        let mut start_robot_state = scene.current_state().clone();
        space.write_robot_state(&start_state, &mut start_robot_state);
        scene.set_current_state(start_robot_state);

        let planning_request = PlanningRequest {
            group_name: group_name.clone(),
            goal: goal_state,
            path_constraints: None,
            resolution,
            seed: seed_base.wrapping_add(id),
            params: RrtConnectParams {
                step_size,
                goal_bias: 0.05,
                termination: Termination::Iterations(max_iterations),
                nn_degree: 8,
            },
        };

        let mut context = manager
            .get_planning_context(&mut scene, &env, planning_request)
            .unwrap_or_else(|e| panic!("get_planning_context: {e}"));
        let result = context.solve();

        match result {
            Ok(response) => {
                drop(context);
                solved_count += 1;
                let path: Vec<Vec<CompoundValue>> = response
                    .trajectory
                    .iter()
                    .map(|rs| space.read_robot_state(rs))
                    .collect();
                let length: f64 = path
                    .windows(2)
                    .map(|pair| space.distance(&pair[0], &pair[1]))
                    .sum();

                let dense = densify(&space, &model, &path, resolution);
                let validity =
                    scene.is_path_valid(&env, &CollisionRequest::default(), &dense, None, &[]);

                println!(
                    "{}",
                    serde_json::json!({
                        "id": id,
                        "solved": true,
                        "length": length,
                        "condition2_valid": validity.valid,
                        "invalid_waypoint_count": validity.invalid_waypoints.len(),
                    })
                );
            }
            Err(e) => {
                drop(context);
                println!(
                    "{}",
                    serde_json::json!({
                        "id": id,
                        "solved": false,
                        "failure": e.to_string(),
                    })
                );
            }
        }
    }

    eprintln!("solved={solved_count}/{total}");
}
