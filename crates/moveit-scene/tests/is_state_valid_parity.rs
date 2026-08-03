// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `is_state_valid` op, ground
//! truth for [`PlanningScene::is_state_valid`]/[`PlanningScene::is_state_constrained`]/
//! [`PlanningScene::is_path_valid`].
//!
//! Ground truth was captured against a freshly built oracle image (source
//! digest `bad01c75cf8584b5`) by sending each case in
//! `panda_is_state_valid.json` as an independent `is_state_valid` request
//! and recording the response verbatim; `waypoints`/`objects`/
//! `path_constraints`/`goal_constraints` are the exact request fields, and
//! `valid`/`invalid_waypoints` are the oracle's own unedited answer.
//!
//! # Every case is a genuine two-sided comparison
//!
//! `moveit_model::LinkModel` loads real `<mesh>` collision geometry (see
//! `collision_parity.rs`'s module doc), so panda builds here with real STL
//! shapes on both sides, not none at all. Every case below is deliberately
//! either genuinely invalid for a real reason or genuinely valid *despite*
//! something present that could have made it invalid, never valid because
//! nothing was there to disagree about:
//!
//! - `default_self_collision`/`clean_arm_no_objects`: the same real
//!   self-collision fact `collision_parity.rs`'s `panda_collision.json`
//!   cases 0/1 already establish (defaults self-collide; this arm
//!   configuration does not) — collision-only, no constraints involved.
//! - `joint_constraint_violated`/`joint_constraint_satisfied`: the same
//!   clean (collision-free) arm configuration both times, differing only in
//!   whether the `JointConstraint`'s `position` matches
//!   `panda_joint1`'s actual value — isolates constraint evaluation from
//!   collision entirely.
//! - `floor_collision`/`floor_far_away`: the same clean arm and the same
//!   `4x4x0.1` floor object (`collision_parity.rs`'s established pose
//!   convention) both times, differing only in the floor's `z` — the world
//!   object is present in *both* cases, so `floor_far_away`'s `valid: true`
//!   is not "nothing to disagree about", it is "the real mesh and the real
//!   box, at a pose where they provably do not intersect".
//! - `path_first_waypoint_colliding`/`path_goal_unsatisfied`: two-waypoint
//!   paths exercising [`PlanningScene::is_path_valid`] specifically —
//!   respectively a colliding first waypoint (state-validity failure,
//!   goal never even relevant since it is not the last waypoint) and a
//!   goal constraint the otherwise-valid last waypoint does not satisfy
//!   (goal failure, not a state-validity failure) — the two ways
//!   `invalid_waypoints` can be populated, kept distinct on purpose.
//!
//! Three cases report `valid: true`, five report `valid: false`: both
//! branches are exercised, not just the satisfied one.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::{Matrix3, Translation3, UnitQuaternion};

#[derive(Deserialize)]
struct ShapeSpec {
    #[serde(rename = "type")]
    kind: String,
    #[serde(default)]
    size: [f64; 3],
}

#[derive(Deserialize)]
struct ObjectSpec {
    id: String,
    pose: [f64; 16],
    shape: ShapeSpec,
}

#[derive(Deserialize, Default)]
struct JointConstraintSpec {
    joint_name: String,
    position: f64,
    tolerance_above: f64,
    tolerance_below: f64,
    weight: f64,
}

#[derive(Deserialize, Default)]
struct ConstraintsSpec {
    #[serde(default)]
    joint_constraints: Vec<JointConstraintSpec>,
}

#[derive(Deserialize)]
struct Case {
    name: String,
    waypoints: Vec<BTreeMap<String, f64>>,
    objects: Vec<ObjectSpec>,
    #[serde(default)]
    path_constraints: ConstraintsSpec,
    #[serde(default)]
    goal_constraints: Vec<ConstraintsSpec>,
    valid: bool,
    invalid_waypoints: Vec<usize>,
}

#[derive(Deserialize)]
struct Fixture {
    cases: Vec<Case>,
}

fn load_fixture() -> Fixture {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/panda_is_state_valid.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

/// The two `moveit_resources_*_description` packages committed under
/// `fixtures/meshes/`, same mapping `collision_parity.rs`'s
/// `fixture_mesh_search_paths` uses.
fn mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn build_model() -> RobotModel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_search_paths())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn world_from_objects(objects: &[ObjectSpec]) -> World {
    let mut world = World::new();
    for object in objects {
        assert_eq!(
            object.shape.kind, "box",
            "this fixture only ever uses box shapes"
        );
        let cuboid = Cuboid::new(
            object.shape.size[0],
            object.shape.size[1],
            object.shape.size[2],
        )
        .expect("fixture box dimensions must be positive");
        world.add_shape(
            &object.id,
            Arc::new(Shape::Cuboid(cuboid)),
            isometry_from_row_major(&object.pose),
        );
    }
    world
}

fn constraint_set(model: &RobotModel, spec: &ConstraintsSpec) -> KinematicConstraintSet {
    let mut set = KinematicConstraintSet::new();
    for jc in &spec.joint_constraints {
        let constraint = JointConstraint::new(
            model,
            &jc.joint_name,
            jc.position,
            jc.tolerance_above,
            jc.tolerance_below,
            jc.weight,
        )
        .unwrap_or_else(|e| panic!("building JointConstraint for {}: {e}", jc.joint_name));
        set.push(Constraint::Joint(constraint));
    }
    set
}

fn waypoint_state<'m>(
    model: &'m RobotModel,
    joint_values: &BTreeMap<String, f64>,
) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

#[test]
fn panda_is_state_valid_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let fixture = load_fixture();

    let mut valid_count = 0;
    let mut invalid_count = 0;

    for case in &fixture.cases {
        let mut scene = PlanningScene::new(&model, &srdf);
        let world = world_from_objects(&case.objects);
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let waypoints: Vec<RobotState> = case
            .waypoints
            .iter()
            .map(|wp| waypoint_state(&model, wp))
            .collect();

        let path_constraints = constraint_set(&model, &case.path_constraints);
        let path_constraints_arg = if case.path_constraints.joint_constraints.is_empty() {
            None
        } else {
            Some(&path_constraints)
        };

        let goal_constraint_sets: Vec<KinematicConstraintSet> = case
            .goal_constraints
            .iter()
            .map(|spec| constraint_set(&model, spec))
            .collect();

        let result = scene.is_path_valid(
            &env,
            &CollisionRequest::default(),
            &waypoints,
            path_constraints_arg,
            &goal_constraint_sets,
        );

        assert_eq!(result.valid, case.valid, "case {}: valid", case.name);
        assert_eq!(
            result.invalid_waypoints, case.invalid_waypoints,
            "case {}: invalid_waypoints",
            case.name
        );

        if case.valid {
            valid_count += 1;
        } else {
            invalid_count += 1;
        }
    }

    assert_eq!(
        valid_count, 3,
        "expected exactly 3 valid cases in the fixture"
    );
    assert_eq!(
        invalid_count, 5,
        "expected exactly 5 invalid cases in the fixture"
    );
}
