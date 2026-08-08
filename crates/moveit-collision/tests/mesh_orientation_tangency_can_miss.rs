// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins one measured case from `examples/mesh_orientation_probe.rs`'s own
//! sweep: that probe found `check_robot_collision` reporting `false` for
//! 6,083 of 24,970 rotated-mesh-at-exact-tangency configurations (mesh
//! against box/sphere/cylinder/cone/mesh, mesh as both the attached and the
//! world object, 497 systematic orientations at 5-degree resolution over 7
//! axes plus 2,000 random orientations), every one of the 6,083 flipping to
//! `true` with between `1e-16` and `1e-14` of added overlap -- the same
//! order of magnitude `exact_tangency_is_decided_per_shape_pair.rs`'s own
//! module doc already measures as GJK's own floating-point rounding at an
//! axis-aligned tie (its `-1.1102230246251565e-16` for `cylinder x box`, for
//! one).
//!
//! The difference is the rescue: that rounding is absorbed for every
//! non-mesh pair by `touches_at_tie`'s `fcl_tangency_table` lookup, and
//! cannot be for mesh by construction -- `fcl_tangency_verdict`
//! (`crates/moveit-collision/src/parry.rs:2151-2158`) returns `None` for
//! `TriMesh` regardless of the paired shape, which
//! `triage-2429-enumeration.md` (`residual-triage`, this repository)
//! established makes `accumulate_collision`'s rescue branch
//! (`parry.rs:2429`) structurally inert for every mesh pair. So the same
//! rounding that every other pair already gets forgiven for surfaces here as
//! `collision: false` in production.
//!
//! This is the smallest reproducer the probe's own sweep found: a unit cube
//! mesh rotated 5 degrees about world `z`, placed at exact tangency (by
//! construction -- see the probe's own module doc for why the construction
//! is exact for any rotation) above an axis-aligned unit box, with the mesh
//! as the attached/upper body. The probe's systematic sweep found the same
//! shape pair (`box x mesh`, tilted about `z`) failing at three separate
//! angles (5, 285, 355 degrees) in both argument orders, so this is
//! representative of a class the probe measured, not a singular fluke this
//! file picked out.
//!
//! This test pins the *current* behaviour rather than asserting a fix --
//! whether to change `accumulate_collision` to cover mesh pairs the way it
//! already covers every other pair is a decision this file does not make.
//!
//! `tools/fcl-mesh-orientation-probe` measured this exact pose (`box`, mesh
//! attached, `axis=z, angle=5deg`) against `fcl::BVHModel<fcl::OBBRSSd>`:
//! `true` in both argument orders. Unlike `mesh x cone`
//! (`exact_tangency_is_decided_per_shape_pair.rs`'s module doc), this is not
//! a case where fcl disagrees with itself -- there is a single stable
//! upstream answer this pose diverges from, not merely a `false` this side
//! of the wire.

use std::collections::BTreeSet;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Isometry3, Mesh, Shape, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::{Translation3, Unit};

/// Half the extent of both shapes, along every axis -- matches
/// `exact_tangency_is_decided_per_shape_pair.rs::HALF`.
const HALF: f64 = 0.5;
/// The point the box's own top face centre and the tilted mesh's own lowest
/// rotated vertex are both translated onto.
const TOUCH: (f64, f64, f64) = (5.0, 0.0, 0.0);

/// The same 8 vertices `exact_tangency_is_decided_per_shape_pair.rs::
/// unit_cube_mesh` and `mesh_orientation_probe.rs::cube_geometry` build.
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

/// Rotates the mesh 5 degrees about world `z` and translates it so its own
/// lowest rotated vertex sits exactly on [`TOUCH`], offset by `delta` along
/// z (negative moves it toward more overlap) -- the same construction
/// `mesh_orientation_probe.rs::rotated_mesh_pose` uses for `Role::Upper`,
/// specialised to this one reproducer.
fn tilted_mesh_pose(delta: f64) -> Isometry3 {
    let axis = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
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
        "box",
        Arc::new(Shape::Cuboid(
            Cuboid::new(2.0 * HALF, 2.0 * HALF, 2.0 * HALF).expect("positive cuboid"),
        )),
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

/// Measured by `mesh_orientation_probe`: at this exact construction,
/// `check_robot_collision` reports no collision even though the mesh and box
/// are tangent by construction -- pins the *current*, unfixed behaviour.
#[test]
fn a_5_degree_tilted_mesh_on_a_box_misses_at_exact_tangency() {
    assert!(
        !collides(0.0),
        "if this now collides, the miss this test pins has already been fixed -- \
         update this test to assert the fix instead of deleting it silently"
    );
}

/// The same pose with `1e-14` of added overlap -- two orders of magnitude
/// past the `1e-16` depth `mesh_orientation_probe` measured for this exact
/// case -- does collide, confirming the pose above is a genuine near-tie and
/// not a construction error that happens to read as `false` for an
/// unrelated reason.
#[test]
fn a_hair_more_overlap_on_the_same_pose_does_collide() {
    assert!(collides(-1e-14));
}
