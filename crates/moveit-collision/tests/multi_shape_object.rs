// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Boundary cases for a world object's *shape count* and *shape kind*.
//!
//! `PosedBody` holds one globally-posed part per shape, matching upstream's
//! `FCLObject::collision_objects_`. Before that, a body's shapes were folded
//! into a single `parry` shape — a lone shape as itself, two or more wrapped
//! in a `Compound` — and `Compound::new` panics
//! (`"Nested composite shapes are not allowed"`) the moment one part is a
//! `TriMesh`, which `parry` classifies as composite.
//!
//! So the boundaries are shape count (1 vs 2+) crossed with whether any shape
//! is a mesh, not a narrative about scenes. Each case below is one cell of
//! that grid; the mesh-bearing multi-shape cells are the ones that panicked.

use std::sync::Arc;

use moveit_collision::{
    CollisionEnv, CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_geometry::Isometry3;
use moveit_geometry::shapes::{Mesh, Shape, Sphere};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

/// A unit tetrahedron at the origin — the smallest closed `Shape::Mesh` that
/// converts to a `parry` `TriMesh`, which is what makes a part composite.
fn tetrahedron() -> Arc<Shape> {
    Arc::new(Shape::Mesh(Mesh {
        vertices: vec![
            [0.0, 0.0, 0.0].into(),
            [0.2, 0.0, 0.0].into(),
            [0.0, 0.2, 0.0].into(),
            [0.0, 0.0, 0.2].into(),
        ],
        triangles: vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        triangle_normals: None,
        vertex_normals: None,
    }))
}

fn sphere() -> Arc<Shape> {
    Arc::new(Shape::Sphere(
        Sphere::new(0.1).expect("0.1 is a valid sphere radius"),
    ))
}

/// pr2, not panda: panda/fanuc/dual_arm_panda carry only `<mesh>` collision
/// geometry, which the URDF loader does not yet retain (PORTING-PLAN.md
/// §13.4), so every one of their links is invisible to this backend and no
/// "collides" assertion against them could mean anything. pr2 has primitive
/// collision shapes -- `base_footprint`'s 1mm box sits at `z = 0.071`, within
/// reach of a 0.1-radius sphere at the origin.
fn robot() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("fixture URDF must read");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// `true` if the robot at its default state collides with a world object built
/// from `shapes`, all placed at `translation` along x.
fn collides_with_object_at(shapes: &[Arc<Shape>], translation: f64) -> bool {
    let poses = vec![Isometry3::translation(translation, 0.0, 0.0); shapes.len()];
    let mut world = World::new();
    world
        .add_shapes_to_object("obj", shapes, &poses)
        .expect("a non-empty shape list must create the object");
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let model = robot();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None)
        .collision
}

/// Far enough that no pr2 link reaches it in any configuration, so every
/// case below has a known-negative twin and a "collides" assertion cannot
/// pass by accident.
const OUT_OF_REACH: f64 = 50.0;

#[test]
fn one_primitive() {
    assert!(collides_with_object_at(&[sphere()], 0.0));
    assert!(!collides_with_object_at(&[sphere()], OUT_OF_REACH));
}

#[test]
fn one_mesh() {
    assert!(collides_with_object_at(&[tetrahedron()], 0.0));
    assert!(!collides_with_object_at(&[tetrahedron()], OUT_OF_REACH));
}

#[test]
fn two_primitives() {
    assert!(collides_with_object_at(&[sphere(), sphere()], 0.0));
    assert!(!collides_with_object_at(
        &[sphere(), sphere()],
        OUT_OF_REACH
    ));
}

/// Panicked before `PosedBody` became a part list: `Compound::new` rejects a
/// `TriMesh` part.
#[test]
fn a_mesh_beside_a_primitive() {
    assert!(collides_with_object_at(&[tetrahedron(), sphere()], 0.0));
    assert!(!collides_with_object_at(
        &[tetrahedron(), sphere()],
        OUT_OF_REACH
    ));
}

/// The case the old `combine_parts` doc named as "remains unsupported (would
/// still panic)".
#[test]
fn two_meshes() {
    assert!(collides_with_object_at(
        &[tetrahedron(), tetrahedron()],
        0.0
    ));
    assert!(!collides_with_object_at(
        &[tetrahedron(), tetrahedron()],
        OUT_OF_REACH
    ));
}

/// One part of a multi-shape object colliding is enough, even when the other
/// is out of reach — the cross product must not be short-circuited by the
/// first miss.
#[test]
fn only_the_second_part_collides() {
    let shapes = [tetrahedron(), sphere()];
    let poses = [
        Isometry3::translation(OUT_OF_REACH, 0.0, 0.0),
        Isometry3::identity(),
    ];
    let mut world = World::new();
    world
        .add_shapes_to_object("obj", &shapes, &poses)
        .expect("a non-empty shape list must create the object");
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let model = robot();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    assert!(
        env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None)
            .collision
    );
}
