// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is Phase 8 benchmark infrastructure (PORTING-PLAN.md
// §5's CHOMP/STOMP row, §264.7's population finding), not a port. Upstream has
// no equivalent binary -- it never asks whether a benchmark problem needed an
// optimizer, because it ships no benchmark problem set.

//! Answers one question about each problem in a `plan`-op request, for every
//! resolution in a grid: **is the straight segment between the problem's two
//! endpoints already collision-free?**
//!
//! # Why this is a property of the problem, not of a planner
//!
//! Both Phase 8 optimizers start from that segment.
//!
//! - STOMP's initialization is `fillInLinearInterpolation`
//!   (`cspace_planners::chomp`'s ported twin is `ChompTrajectory::
//!   fill_in_linear_interpolation`), which is the segment itself.
//! - CHOMP's default `trajectory_initialization_method` is `quintic-spline`,
//!   i.e. `ChompTrajectory::fill_in_min_jerk`. That is *not* the same time
//!   parameterization, but it is the same **path**: the method sets
//!   `coeff[joint][0] = x0`, `coeff[joint][1] = coeff[joint][2] = 0`, and the
//!   three remaining coefficients each proportional to `x1 - x0` with a
//!   per-joint-independent factor, so every joint runs the *same* scalar
//!   profile `s(t)` and the trajectory traces `x0 + s(t) * (x1 - x0)` -- the
//!   straight segment in joint space, sampled at different times.
//!
//! So one answer per problem serves both planners and both implementations,
//! and no planner has to run to obtain it.
//!
//! # What it is for
//!
//! PORTING-PLAN.md §264.7 measured, on 8-problem strata, that the problems
//! this uniform-random-endpoint population yields to an optimizer are largely
//! the problems whose seed was already valid -- where CHOMP breaks out on its
//! first iteration and STOMP returns its seed. On those problems "the returned
//! path is collision-free" is a statement about the problem generator, not
//! about the optimizer. This binary is what carries that measurement from 8
//! problems to the full 500 the Phase 8 row is judged on.
//!
//! # Usage
//!
//! `cargo run --release --example seed_validity_problem_set -p
//! cspace_planners::sbp` with a `plan`-op request JSON on stdin, optionally
//! carrying `condition2_resolutions` (the same field the Phase 8 harnesses
//! read). One NDJSON line per problem:
//! `{"id", "seed_valid", "seed_length", "seed_invalid_count",
//! "densified_waypoint_count", "seed_by_resolution"?}`, where the top-level
//! fields are at the request's own `motion_resolution`.

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::sync::Arc;

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::test_support::isometry_from_row_major;
use cspace_planners::sbp::{CompoundValue, JointModelGroupSpace, StateSpace};
use cspace_planning::scene::PlanningScene;

/// `plan_benchmark_port.rs`'s own `mesh_package_for`/`load_robot`, unchanged:
/// the request's `robot` field is what ties this binary to the set the
/// problems were sampled against, so it rebuilds that robot rather than
/// assuming panda.
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

fn parse_obstacles(objects: &[serde_json::Value]) -> Vec<(String, Arc<Shape>, Isometry3)> {
    objects
        .iter()
        .map(|object| {
            let id = object["id"]
                .as_str()
                .expect("object.id must be a string")
                .to_string();
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
            let shape = Arc::new(Shape::Cuboid(
                Cuboid::new(sx, sy, sz).unwrap_or_else(|e| panic!("Cuboid::new: {e}")),
            ));
            (id, shape, isometry_from_row_major(&pose_flat))
        })
        .collect()
}

/// Densifies `path` at `resolution` -- byte-identical rule to
/// `plan_benchmark_port`'s own `densify`, so a seed's verdict and an output's
/// verdict are answers to the same question.
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
    let resolution = request["motion_resolution"]
        .as_f64()
        .expect("request.motion_resolution must be a number");
    let grid: Vec<f64> = match request.get("condition2_resolutions") {
        None | Some(serde_json::Value::Null) => Vec::new(),
        Some(value) => value
            .as_array()
            .expect("request.condition2_resolutions must be an array")
            .iter()
            .map(|entry| {
                let r = entry
                    .as_f64()
                    .expect("request.condition2_resolutions entries must be numbers");
                assert!(r > 0.0, "resolutions must be positive, got {r}");
                r
            })
            .collect(),
    };

    let robot = request["robot"]
        .as_str()
        .expect("request.robot must be a string")
        .to_string();
    let (model, srdf) = load_robot(&robot);
    let space = JointModelGroupSpace::new(&model, &group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));

    let mut world = World::new();
    for (id, shape, pose) in parse_obstacles(
        request["objects"]
            .as_array()
            .expect("request.objects must be an array"),
    ) {
        world.add_shape(&id, shape, pose);
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    let mut scene = PlanningScene::new(&model, &srdf);

    let mut invalid = 0usize;
    let mut total = 0usize;
    for problem in request["problems"]
        .as_array()
        .expect("request.problems must be an array")
    {
        total += 1;
        let id = problem["id"].as_u64().expect("problem.id must be a u64");
        let start_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["start"].clone()).expect("problem.start");
        let goal_map: BTreeMap<String, f64> =
            serde_json::from_value(problem["goal"].clone()).expect("problem.goal");
        let start = space.read_robot_state(&joint_map_to_robot_state(&model, &start_map));
        let goal = space.read_robot_state(&joint_map_to_robot_state(&model, &goal_map));
        let segment = [start, goal];

        let dense = densify(&space, &model, &segment, resolution);
        let validity = scene.is_path_valid(&env, &CollisionRequest::default(), &dense, None, &[]);
        if !validity.valid {
            invalid += 1;
        }

        let mut record = serde_json::json!({
            "id": id,
            "seed_valid": validity.valid,
            "seed_invalid_count": validity.invalid_waypoints.len(),
            "seed_length": space.distance(&segment[0], &segment[1]),
            "densified_waypoint_count": dense.len(),
        });
        if !grid.is_empty() {
            let by_resolution: Vec<serde_json::Value> = grid
                .iter()
                .map(|r| {
                    let at = densify(&space, &model, &segment, *r);
                    let at_validity =
                        scene.is_path_valid(&env, &CollisionRequest::default(), &at, None, &[]);
                    serde_json::json!({
                        "resolution": r,
                        "valid": at_validity.valid,
                        "invalid_count": at_validity.invalid_waypoints.len(),
                        "densified_waypoint_count": at.len(),
                    })
                })
                .collect();
            record["seed_by_resolution"] = serde_json::Value::Array(by_resolution);
        }
        println!("{record}");
    }

    // A zero population is the failure this line exists to make loud: an empty
    // `problems` array and a set every one of whose seeds is valid print the
    // same nothing otherwise.
    eprintln!("seed_invalid={invalid}/{total} at motion_resolution={resolution}");
}
