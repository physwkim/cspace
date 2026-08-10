// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Shared by every `examples/*.rs` binary in this crate that stands up a
// panda-fixture `RobotModel` plus one obstacle set's `ParryCollisionEnv` and
// distance field, and calls the real
// `isCurrentTrajectoryMeshToMeshCollisionFree` predicate
// (`ChompOptimizer::optimize`'s injected `mesh_to_mesh_collision_free`
// closure, `optimizer.rs:1934-1935`).
//
// `include!`d, not a `src/` module: this crate's library half deliberately
// cannot depend on `cspace_planning::scene` (see `Cargo.toml`'s `[dev-dependencies]`
// comment), and `include!`d text is spliced into the including file's own
// compilation unit, so it can use this crate's dev-only dependencies the way
// a `src/` module could not.
//
// `include!`d, not copy-pasted: a byte-for-byte duplicate between two
// example files has no link back to its original and cannot go stale
// loudly -- `verify_final_trajectory_predicate.rs`'s own header names the
// concrete case this closes: a benchmark-construction fix landing in
// `chomp_benchmark_port.rs` and silently orphaning a second, un-updated
// copy. There is now exactly one definition of each item below; every
// caller sees the same text because it *is* the same text.
//
// Not a `[[example]]` itself: it lives in a subdirectory (`support/`) with
// no sibling `main.rs`, so cargo's default example auto-discovery
// (`examples/*.rs`, `examples/*/main.rs`) never treats it as its own binary
// target.
//
// Relies on the including file's own `use` block for every external name
// used below (`RobotModel`, `PlanningScene`, `ParryCollisionEnv`, etc.) --
// both current callers already import the same set for their own use, and a
// caller that stops doing so gets a compile error naming the missing
// import, not a silent miscompile.

/// Upstream `CollisionEnvDistanceField::DEFAULT_RESOLUTION`
/// (`collision_env_distance_field.hpp:53`). Named once because it is read
/// twice: by [`distance_field_config`] for the grid, and by
/// `add_link_body_decompositions` for the per-link sphere decomposition --
/// upstream passes its own `resolution_` to both, and the two disagreeing
/// would silently mismatch the collision spheres against the grid they are
/// looked up in.
const DISTANCE_FIELD_RESOLUTION: f64 = 0.02;

/// Upstream `CollisionEnvDistanceField`'s `DEFAULT_SIZE_X/Y/Z`
/// (`collision_env_distance_field.hpp:49-51`), as a corner-origin
/// [`GridGeometry`] centred on the robot origin -- upstream performs that
/// centre-to-corner shift itself at its own `PropagationDistanceField`
/// construction site, which this port makes the caller's job (see
/// [`DistanceFieldConfig`]'s own doc).
fn distance_field_config() -> DistanceFieldConfig {
    let size = Vector3::new(3.0, 3.0, 4.0);
    let origin_center = Vector3::new(0.0, 0.0, 0.0);
    DistanceFieldConfig {
        geometry: GridGeometry::new(size, origin_center - 0.5 * size, DISTANCE_FIELD_RESOLUTION)
            .expect("upstream's own default grid geometry must be constructible"),
        max_propagation_distance: 0.25,
        use_signed_distance_field: false,
    }
}

/// The `moveit_resources_panda_description` package committed under
/// `fixtures/meshes/`. Cross-crate duplicates of this same helper still
/// exist (e.g. `cspace-planners/examples/plan_benchmark_port.rs`)
/// because a cargo example cannot import another *crate's* example; this
/// `include!` only closes the duplication possible within one crate.
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

/// One obstacle as both a world object (for `cspace_planning::scene`'s mesh-level
/// checks) and a shape/pose pair (for the distance field CHOMP's gradients
/// read).
struct Obstacle {
    id: String,
    shape: Arc<Shape>,
    pose: Isometry3,
}

fn parse_obstacles(objects: &[serde_json::Value]) -> Vec<Obstacle> {
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
            Obstacle {
                id,
                shape: Arc::new(Shape::Cuboid(
                    Cuboid::new(sx, sy, sz).unwrap_or_else(|e| panic!("Cuboid::new: {e}")),
                )),
                pose: isometry_from_row_major(&pose_flat),
            }
        })
        .collect()
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

/// `obstacles` as a [`ParryCollisionEnv`] (the mesh-level world every
/// `PlanningScene::is_path_valid` call in this crate's examples checks
/// against) plus the matching [`PropagationDistanceField`] and
/// [`DistanceFieldCollisionCache`] CHOMP's own gradients read -- the same
/// upstream `CollisionEnvDistanceField` defaults
/// (`collision_env_distance_field.hpp:49-55`) both current callers built
/// inline before this was extracted.
fn build_collision_world<'m>(
    model: &'m RobotModel,
    obstacles: &[Obstacle],
) -> (
    ParryCollisionEnv,
    PropagationDistanceField,
    DistanceFieldCollisionCache<'m>,
) {
    let mut world = World::new();
    for obstacle in obstacles {
        world.add_shape(&obstacle.id, Arc::clone(&obstacle.shape), obstacle.pose);
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let field_config = distance_field_config();
    let mut env_distance_field = PropagationDistanceField::new(
        field_config.geometry,
        field_config.max_propagation_distance,
        field_config.use_signed_distance_field,
    )
    .expect("PropagationDistanceField::new with upstream's own defaults");
    for obstacle in obstacles {
        env_distance_field
            .add_shape_to_field(&obstacle.shape, &obstacle.pose)
            .unwrap_or_else(|e| panic!("add_shape_to_field({}): {e}", obstacle.id));
    }

    let decompositions = add_link_body_decompositions(
        model,
        DISTANCE_FIELD_RESOLUTION,
        &LinkPaddingScale::new(),
        None,
    )
    .expect("add_link_body_decompositions");
    let cache = DistanceFieldCollisionCache::new(
        decompositions,
        distance_field_config(),
        /* collision_tolerance, upstream DEFAULT_COLLISION_TOLERANCE */ 0.0,
    );

    (env, env_distance_field, cache)
}

/// Upstream `ChompOptimizer::isCurrentTrajectoryMeshToMeshCollisionFree`,
/// wired to a real [`PlanningScene`] -- the predicate
/// `ChompOptimizer::optimize` calls as its injected
/// `mesh_to_mesh_collision_free` closure (`optimizer.rs:1934-1935`) and the
/// same one `verify_final_trajectory_predicate.rs` calls a second time,
/// post-loop. One definition so both call sites are provably the same
/// check, not two implementations that happen to agree today.
fn mesh_to_mesh_collision_free_check<'m>(
    mesh_scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    active_joint_names: &[String],
    start: &RobotState<'m>,
    best: &DMatrix<f64>,
) -> bool {
    let mut waypoints = Vec::with_capacity(best.nrows());
    for row in 0..best.nrows() {
        let mut state = start.clone();
        for (column, name) in active_joint_names.iter().enumerate() {
            state
                .set_variable_position(name, best[(row, column)])
                .expect("group joint names come from this model");
        }
        waypoints.push(state);
    }
    mesh_scene
        .is_path_valid(env, &CollisionRequest::default(), &waypoints, None, &[])
        .valid
}
