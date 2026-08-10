// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins one measured case from `examples/mesh_orientation_probe.rs`'s own
//! sweep: a unit cube mesh rotated 5 degrees about world `z`, placed at
//! exact tangency (by construction -- see the probe's own module doc for why
//! the construction is exact for any rotation) above an axis-aligned unit
//! box, with the mesh as the attached/upper body.
//!
//! `tools/fcl-mesh-orientation-probe` measured this exact pose (`box`, mesh
//! attached, `axis=z, angle=5deg`) against `fcl::BVHModel<fcl::OBBRSSd>`:
//! `true` in both argument orders. Unlike `mesh x cone`
//! (`exact_tangency_is_decided_per_shape_pair.rs`'s module doc), this is not
//! a case where fcl disagrees with itself -- there is a single stable
//! upstream answer, and this file asserts the port now reaches it.
//!
//! # What this file used to pin, and what closed it
//!
//! It was written as `mesh_orientation_tangency_can_miss.rs`, asserting the
//! opposite: `check_robot_collision` answered `false` here. The probe found
//! that on 6,083 of 24,970 configurations at the time, every one flipping to
//! `true` with between `1e-16` and `1e-14` of added overlap -- the same
//! order of magnitude `exact_tangency_is_decided_per_shape_pair.rs`'s own
//! module doc measures as GJK's floating-point rounding at an axis-aligned
//! tie.
//!
//! Two rounds of collision work later the population had moved rather than
//! shrunk: re-run on the tree at `cc9ec185`, the probe reported 2,758
//! misses, all of them `mesh x box` -- the pair this file pins. What it had
//! become was not the unrescuable rounding the original text describes but
//! an ordinary too-tight gate. `mesh_shape_contact`'s descent admitted a
//! triangle only if the primitive reached the node box grown by the query's
//! own `prediction`, while `query::contact` at that same `prediction` still
//! answers `Some` across roughly `5e-8 m` of clear air. At an exact tangency
//! the triangle holding the contact sits inside that gap, so the descent
//! dropped it before the narrow phase could round it into a touch.
//!
//! `parry::rejection_slack` closes it by giving every conservative rejection
//! in that file the same `BROADPHASE_MARGIN`-wider bound to prove, which the
//! two body-level gates already had and the two descents did not. Re-running
//! the probe after that: 0 misses in 24,970 configurations, all five paired
//! kinds and both argument orders.
//!
//! The rescue path the original text describes is untouched and still inert
//! for `TriMesh` pairs other than `Sphere` -- `fcl_tangency_verdict` returns
//! `None` for them, so nothing here rides on it. This pose is caught in the
//! narrow phase now, not forgiven after it.

use std::collections::BTreeSet;
use std::sync::Arc;

use cspace_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_core::geometry::{Cuboid, Isometry3, Mesh, Shape, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
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

/// The mesh and the box are tangent by construction, and fcl answers `true`
/// here in both argument orders -- so this port has to as well.
#[test]
fn a_5_degree_tilted_mesh_on_a_box_collides_at_exact_tangency() {
    assert!(
        collides(0.0),
        "the descent has stopped reaching the tangent triangle -- see this file's \
         module doc for the gate that used to drop it"
    );
}

/// The same pose with `1e-14` of added overlap -- two orders of magnitude
/// past the `1e-16` depth `mesh_orientation_probe` measured for this exact
/// case -- collides too, confirming the pose above is a genuine near-tie and
/// not a construction error that happens to read as `true` for an unrelated
/// reason.
#[test]
fn a_hair_more_overlap_on_the_same_pose_does_collide() {
    assert!(collides(-1e-14));
}

/// The control the assertion above needs to not be vacuous: real clear air
/// still reports no collision.
///
/// `1e-6` is the tightest gap that has to answer `false` for the widened
/// rejection to have cost nothing -- it is `parry::BROADPHASE_MARGIN` itself,
/// so the descent does admit this triangle, and it is `query::contact` at a
/// prediction of `0.0` that has to reject it. A gate that answered `true`
/// here would have bought its tangency by calling clear air a touch.
#[test]
fn a_real_gap_at_the_descent_margin_still_reports_no_collision() {
    assert!(!collides(1e-6));
}
