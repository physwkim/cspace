// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins one measured case from `examples/mesh_orientation_probe.rs`'s own
//! sweep, the same way `mesh_orientation_tangency_can_miss.rs` pins its own
//! `box` reproducer -- except this one has a known, stable, unambiguous fcl
//! target (`tools/fcl-mesh-orientation-probe` measures `mesh x sphere` as
//! `true` at every one of 497 tilted orientations, zero exceptions, matching
//! the closed-form `Sphere`-triangle specialisation's orientation-independent
//! boundary padding), and this crate now has a table that knows it:
//! `crate::mesh_tangency_table::MESH_TANGENCY[MeshOtherKind::Sphere]` is
//! `MeshVerdict::AlwaysTouching`, and the deleted `is_mesh_pair`'s single
//! blanket boolean is gone.
//!
//! That is not the same as this test passing. `MeshVerdict::AlwaysTouching`'s
//! own module doc has the measurement: `accumulate_collision`'s existing
//! rescue branch for a `query::contact` miss confirms touching via
//! `query::intersection_test`, and for this exact construction that call
//! answers `false` too -- one geometric query deeper than the `contact` path
//! (`Ball`-vs-`Triangle`'s `PointQuery::project_local_point`, not GJK), the
//! same near-degenerate rounding `mesh_orientation_probe.rs` measures,
//! surviving a second, independent computation. Closing this needs a
//! widened-prediction second `query::contact` call inside
//! `accumulate_collision`'s branch body -- a change outside
//! `crate::mesh_tangency_table`'s own confinement, not made here.
//!
//! This test pins the *current* behaviour, exactly as
//! `mesh_orientation_tangency_can_miss.rs` does for its own `box` case: it
//! still misses, and the two controls bracket it as a genuine near-tie
//! rather than a construction error.

use std::collections::BTreeSet;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Isometry3, Mesh, Shape, Sphere, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::{Translation3, Unit};

/// Half the extent of both shapes, along every axis -- matches
/// `mesh_orientation_tangency_can_miss.rs::HALF`.
const HALF: f64 = 0.5;
/// The point the sphere's own centre-minus-radius and the tilted mesh's own
/// lowest rotated vertex are both translated onto.
const TOUCH: (f64, f64, f64) = (5.0, 0.0, 0.0);

/// The same 8 vertices `mesh_orientation_tangency_can_miss.rs::cube_vertices`
/// builds.
fn cube_vertices() -> [Vector3; 8] {
    let mut vertices = [Vector3::zeros(); 8];
    let mut i = 0;
    for &z in &[-HALF, HALF] {
        for &y in &[-HALF, HALF] {
            for &x in &[-HALF, HALF] {
                vertices[i] = Vector3::new(x, y, z);
                i += 1;
            }
        }
    }
    vertices
}

fn unit_cube_mesh() -> Mesh {
    let triangles = vec![
        [0u32, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    Mesh::new(cube_vertices().to_vec(), triangles).expect("cube mesh indices are in range")
}

/// Rotates the mesh 5 degrees about world `x` and translates it so its own
/// lowest rotated vertex sits exactly on [`TOUCH`], offset by `delta` along
/// z (negative moves it toward more overlap) -- one of the ten
/// `mesh x sphere` miss poses `mesh_orientation_probe.rs`'s own sweep found,
/// sampled and confirmed unrescued during this round's own investigation.
fn tilted_mesh_pose(delta: f64) -> Isometry3 {
    let axis = Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0));
    let rot = UnitQuaternion::from_axis_angle(&axis, 5.0_f64.to_radians());
    let vertices = cube_vertices();
    let mut lowest = rot * vertices[0];
    for &v in &vertices[1..] {
        let r = rot * v;
        if r.z < lowest.z {
            lowest = r;
        }
    }
    let translation = Translation3::new(
        TOUCH.0 - lowest.x,
        TOUCH.1 - lowest.y,
        TOUCH.2 - lowest.z + delta,
    );
    Isometry3::from_parts(translation, rot)
}

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

fn collides(delta: f64) -> bool {
    let model = build_prbt();
    let acm = build_acm();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    let touch_links = BTreeSet::new();

    let mesh_shapes = [Arc::new(Shape::Mesh(unit_cube_mesh()))];
    let mesh_poses = [tilted_mesh_pose(delta)];
    let attached = AttachedBodyGeometry {
        id: "tilted_mesh",
        link_name: "prbt_base_link",
        shapes: &mesh_shapes,
        shape_poses: &mesh_poses,
        touch_links: &touch_links,
    };
    let mut world = World::new();
    world.add_shape(
        "sphere",
        Arc::new(Shape::Sphere(Sphere::new(HALF).expect("positive sphere"))),
        Isometry3::translation(TOUCH.0, TOUCH.1, TOUCH.2 - HALF),
    );
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    env.check_robot_collision(
        &CollisionRequest::default(),
        &posed,
        &[attached],
        Some(&acm),
    )
    .collision
}

/// Measured by `mesh_orientation_probe`, and confirmed unrescued by this
/// round's own investigation (`MeshVerdict::AlwaysTouching`'s own doc):
/// `check_robot_collision` reports no collision even though `mesh x sphere`
/// has a stable, unambiguous fcl target of `true` at this exact tangency.
/// Pins the *current*, unfixed behaviour -- update this test (and the doc
/// above) when `accumulate_collision`'s branch body gets the widened-
/// prediction second `query::contact` call this pair needs.
#[test]
fn a_5_degree_tilted_mesh_on_a_sphere_misses_at_exact_tangency_despite_a_known_target() {
    assert!(
        !collides(0.0),
        "if this now collides, the miss this test pins has already been fixed -- \
         update this test to assert the fix instead of deleting it silently"
    );
}

/// The same pose with `1e-14` of added overlap does collide, confirming the
/// pose above is a genuine near-tie and not a construction error.
#[test]
fn a_hair_more_overlap_on_the_same_pose_does_collide() {
    assert!(collides(-1e-14));
}
