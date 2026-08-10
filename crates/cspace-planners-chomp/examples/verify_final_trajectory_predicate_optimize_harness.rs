// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ad hoc verification, not benchmark infrastructure and not a port -- see
// `verify_final_trajectory_predicate.rs`'s module doc for the question both
// binaries answer. This one exists because there are two candidate CHOMP
// Phase 8 harnesses with different construction (`chomp_benchmark_port.rs`,
// checked by the other binary, and `optimize_benchmark_chomp.rs`, checked
// here), and the claim under review did not say which one produced its
// measurement. Both are checked rather than guessed.

//! Same question as [`verify_final_trajectory_predicate`], reusing
//! `optimize_benchmark_chomp.rs`'s construction instead of
//! `chomp_benchmark_port.rs`'s: `tools/ci/measure-phase8-optimizer-properties.sh`'s
//! `panda cage 125 810002` set, `SEED_BASE=525252`, `TIMEOUT_SECONDS=120`
//! (which *does* bind here, unlike the other binary's `1e9`),
//! `GOAL_TOLERANCE=0.01`, no ACM (`optimize_benchmark_chomp.rs` passes
//! `None`, not `Some(&acm)`), and that file's own centered-but-not-quite
//! distance field origin (`z` origin is `-0.5 * scale`, not `-0.5 *
//! df_size.z` -- reproduced exactly, not fixed, since the point is to match
//! what that binary actually computes).
//!
//! Usage: `cargo run --example
//! verify_final_trajectory_predicate_optimize_harness -p
//! cspace-planners-chomp`, with `plan_benchmark_problem_set cage 125
//! 810002`'s output on stdin.

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::sync::Arc;
use std::time::Instant;

use cspace_collision::distance_field::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GridGeometry,
    PropagationDistanceField, add_link_body_decompositions, collision_object_point_decomposition,
};
use cspace_collision::{CollisionRequest, LinkPaddingScale};
use cspace_collision::{ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planners_chomp::optimizer::ChompCollisionContext;
use cspace_planners_chomp::{ChompGoal, ChompParameters, ChompRequest, GoalJointConstraint, solve};
use cspace_scene::PlanningScene;
use nalgebra::DMatrix;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

const TARGET_IDS: [u64; 2] = [33, 83];
const SEED_BASE: u64 = 525_252;
const TIMEOUT_SECONDS: f64 = 120.0;
const GOAL_TOLERANCE: f64 = 0.01;
const DF_RESOLUTION: f64 = 0.02;
const DF_MAX_PROPAGATION: f64 = 0.25;
const DF_SIZE: (f64, f64, f64) = (3.0, 3.0, 4.0);

fn load_panda() -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    let paths = MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )]);
    let urdf_xml = std::fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &paths)
        .expect("fixture model must build");
    (model, srdf)
}

fn translation_from_row_major_4x4(flat: &[f64]) -> Isometry3 {
    assert_eq!(flat.len(), 16, "expected a flat 4x4 matrix, got {flat:?}");
    Isometry3::translation(flat[3], flat[7], flat[11])
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
    state.update();
    state
}

fn state_from_row<'m>(
    template: &RobotState<'m>,
    other: &RobotState<'_>,
    names: &[String],
    matrix: &DMatrix<f64>,
    point: usize,
) -> RobotState<'m> {
    let mut state = template.clone();
    state.set_variable_positions(other.positions());
    for (j, name) in names.iter().enumerate() {
        state
            .set_variable_position(name, matrix[(point, j)])
            .unwrap_or_else(|e| panic!("set_variable_position({name}): {e}"));
    }
    state.update();
    state
}

/// Interpolates `path` (one `Vec<f64>` of active-joint columns per waypoint)
/// so no active joint moves by more than `resolution` between two
/// consecutive states -- same per-joint rule as `optimize_benchmark_chomp.rs`'s
/// own `densify`, duplicated for the same reason as everything else in this
/// file. Used only for the informational densified clearance number below,
/// not for the predicate itself: the predicate matches upstream's own
/// undensified, per-raw-waypoint check exactly.
fn densify_columns(names: &[String], path: &[Vec<f64>], resolution: f64) -> Vec<Vec<f64>> {
    let mut out = vec![path[0].clone()];
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
            out.push(from.iter().zip(to).map(|(a, b)| a + (b - a) * t).collect());
        }
    }
    debug_assert_eq!(names.len(), path[0].len());
    out
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
    let scale = request["scale"].as_f64().unwrap_or(1.0);
    let motion_resolution = request["motion_resolution"]
        .as_f64()
        .expect("request.motion_resolution must be a number");

    let (model, srdf) = load_panda();
    let group = model
        .joint_model_group(&group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"));
    let active_joint_names: Vec<String> = group
        .active_joint_names()
        .iter()
        .map(|s| s.to_string())
        .collect();

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

    // Reproduced exactly from `optimize_benchmark_chomp.rs`, including its
    // z-origin formula -- see this file's header.
    let df_size = Vector3::new(DF_SIZE.0 * scale, DF_SIZE.1 * scale, DF_SIZE.2 * scale);
    let df_origin = Vector3::new(-0.5 * df_size.x, -0.5 * df_size.y, -0.5 * scale);
    let df_config = DistanceFieldConfig {
        geometry: GridGeometry::new(df_size, df_origin, DF_RESOLUTION)
            .unwrap_or_else(|e| panic!("GridGeometry::new: {e}")),
        max_propagation_distance: DF_MAX_PROPAGATION,
        use_signed_distance_field: false,
    };
    let mut env_field = PropagationDistanceField::new(
        df_config.geometry,
        df_config.max_propagation_distance,
        df_config.use_signed_distance_field,
    )
    .unwrap_or_else(|e| panic!("PropagationDistanceField::new: {e}"));
    for (_, object) in world.iter() {
        let decomposition = collision_object_point_decomposition(object, DF_RESOLUTION)
            .unwrap_or_else(|e| panic!("collision_object_point_decomposition: {e}"));
        env_field.add_points_to_field(&decomposition.collision_points());
    }

    let decompositions =
        add_link_body_decompositions(&model, DF_RESOLUTION, &LinkPaddingScale::new(), None)
            .unwrap_or_else(|e| panic!("add_link_body_decompositions: {e}"));
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let mut template = RobotState::new(&model);
    template.set_to_default_values();

    let params = ChompParameters {
        planning_time_limit: TIMEOUT_SECONDS,
        ..ChompParameters::default()
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
        let start_state = joint_map_to_robot_state(&model, &start_map);

        let goal = ChompGoal {
            joint_constraints: active_joint_names
                .iter()
                .zip(active_joint_names.iter().map(|name| {
                    goal_map
                        .get(name)
                        .unwrap_or_else(|| panic!("problem.goal has no entry for {name}"))
                }))
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
        let names = active_joint_names.clone();
        // Byte-for-byte `optimize_benchmark_chomp.rs`'s own `mesh_to_mesh` --
        // same `is_path_valid` call, same `env`, same `None` constraints,
        // same `&[]`. `None` for the ACM: this harness never passes one to
        // `solve` either (see below).
        let mut mesh_to_mesh = |state: &RobotState<'_>, matrix: &DMatrix<f64>| {
            let waypoints: Vec<RobotState<'_>> = (0..matrix.nrows())
                .map(|point| state_from_row(&mesh_template, state, &names, matrix, point))
                .collect();
            mesh_scene
                .is_path_valid(&env, &CollisionRequest::default(), &waypoints, None, &[])
                .valid
        };

        let chomp_request = ChompRequest {
            start_state: &start_state,
            group_name: &group_name,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(SEED_BASE.wrapping_add(target));
        let t0 = Instant::now();
        // `optimize_benchmark_chomp.rs` passes `None` for the ACM (its
        // `solve(&chomp_request, &mut collision, None, &mut mesh_to_mesh,
        // &mut rng)`), unmodified here.
        let outcome = solve(
            &chomp_request,
            &mut collision,
            None,
            &mut mesh_to_mesh,
            &mut rng,
        );
        let elapsed = t0.elapsed().as_secs_f64();
        let timed_out = elapsed > TIMEOUT_SECONDS;

        match outcome {
            Ok(solution) if !timed_out => {
                let waypoints: Vec<RobotState<'_>> = (0..solution.trajectory.way_point_count())
                    .map(|i| {
                        solution
                            .trajectory
                            .way_point(i)
                            .unwrap_or_else(|e| panic!("way_point({i}): {e}"))
                            .clone()
                    })
                    .collect();

                let mut post_hoc_scene = PlanningScene::new(&model, &srdf);
                let validity = post_hoc_scene.is_path_valid(
                    &env,
                    &CollisionRequest::default(),
                    &waypoints,
                    None,
                    &[],
                );

                // Informational only, not the predicate under review: the
                // DERIVED per-waypoint minimum the claim under review cites
                // (9.13e-4 m / 3.80e-3 m), recomputed independently via
                // `PlanningScene::distance_to_collision` for comparison.
                // This is a *narrower* quantity than the predicate above --
                // it is robot-vs-environment only ("ignoring self-collisions",
                // its own doc comment) -- so agreement with the predicate is
                // expected but not guaranteed; a self-collision the predicate
                // catches would not lower this number at all.
                let mut clearance_scene = PlanningScene::new(&model, &srdf);
                let min_distance_to_collision = waypoints
                    .iter()
                    .map(|wp| {
                        clearance_scene.set_current_state(wp.clone());
                        clearance_scene.distance_to_collision(&env)
                    })
                    .fold(f64::INFINITY, f64::min);

                // Same number, but over the path densified at the request's
                // own `motion_resolution` (0.01 rad here) instead of the 101
                // raw waypoints -- tests whether the raw-waypoint minimum
                // above is hiding a closer approach strictly *between* two
                // consecutive optimizer waypoints.
                let raw_columns: Vec<Vec<f64>> = waypoints
                    .iter()
                    .map(|wp| {
                        active_joint_names
                            .iter()
                            .map(|name| wp.variable_position(name).unwrap())
                            .collect()
                    })
                    .collect();
                let dense_columns =
                    densify_columns(&active_joint_names, &raw_columns, motion_resolution);
                let min_distance_to_collision_densified = dense_columns
                    .iter()
                    .map(|cols| {
                        let mut state = template.clone();
                        for (name, value) in active_joint_names.iter().zip(cols) {
                            state.set_variable_position(name, *value).unwrap();
                        }
                        state.update();
                        clearance_scene.set_current_state(state);
                        clearance_scene.distance_to_collision(&env)
                    })
                    .fold(f64::INFINITY, f64::min);

                let mut line = serde_json::json!({
                    "id": target,
                    "solved": true,
                    "plan_seconds": elapsed,
                    "waypoint_count": waypoints.len(),
                    "post_hoc_mesh_to_mesh_collision_free": validity.valid,
                    "min_distance_to_collision_robot_vs_env_only": min_distance_to_collision,
                    "min_distance_to_collision_densified_robot_vs_env_only": min_distance_to_collision_densified,
                    "densified_waypoint_count": dense_columns.len(),
                });
                if !validity.valid {
                    let mut detail = Vec::new();
                    for &wp in &validity.invalid_waypoints {
                        post_hoc_scene.set_current_state(waypoints[wp].clone());
                        let result = post_hoc_scene.check_collision(
                            &env,
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
                println!("{line}");
            }
            other => {
                let detail = match other {
                    Err(e) => e.to_string(),
                    Ok(_) => format!("solved in {elapsed}s, over the {TIMEOUT_SECONDS}s bound"),
                };
                println!(
                    "{}",
                    serde_json::json!({
                        "id": target, "solved": false, "plan_seconds": elapsed,
                        "failure": detail,
                    })
                );
            }
        }
    }
}
