// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins one measured case from `examples/mesh_orientation_probe.rs`'s own
//! sweep, the same way `mesh_orientation_tangency_is_caught_at_exact_tangency.rs` pins its own
//! `box` reproducer -- except this one has a known, stable, unambiguous fcl
//! target (`tools/fcl-mesh-orientation-probe` measures `mesh x sphere` as
//! `true` at every one of 497 tilted orientations, zero exceptions, matching
//! the closed-form `Sphere`-triangle specialisation's orientation-independent
//! boundary padding), and this crate's `crate::mesh_tangency_table::
//! MESH_TANGENCY[MeshOtherKind::Sphere]` (`MeshVerdict::AlwaysTouching`)
//! knows it.
//!
//! This pose used to be a genuine miss: `MeshVerdict::AlwaysTouching`'s own
//! module doc measured that `accumulate_collision`'s rescue confirmation,
//! `query::intersection_test`, answers `false` here too -- one geometric
//! query deeper than the `contact` path (`Ball`-vs-`Triangle`'s `PointQuery::
//! project_local_point`, not GJK), the same near-degenerate rounding
//! `mesh_orientation_probe.rs` measures, surviving a second, independent
//! computation. `crate::parry::tangent_pair_touches` closes that: a `TriMesh`
//! pair whose plain `intersection_test` fails gets one more chance, a second
//! `query::contact` call with prediction widened by `TIE_ROUNDING_MARGIN`'s
//! own tie-band margin (doubled) -- which does find a real contact here, and
//! `accumulate_collision`'s branch now reports a collision. Re-running
//! `mesh_orientation_probe` confirms the whole `mesh x sphere` row, not just
//! this one pose: 2594 misses (497 systematic + 2000 random orientations x 2
//! roles) before this fix, 0 after; every other kind's miss count is
//! unchanged (`box` 6, `cylinder` 13, `cone` 1521, `mesh x mesh` 1949) --
//! `fcl_tangency_verdict` only reaches `Some(true)` for a mesh pair here, so
//! nothing else could have moved.
//!
//! `mesh_sphere_tangency_has_a_target_but_is_not_yet_rescued.rs` was this
//! file's own former name and content -- it pinned the miss as *current,
//! unfixed* behaviour. Renamed and rewritten here because that stopped being
//! true for `sphere`.
//!
//! The `box`/`cylinder`/`cone`/`mesh x mesh` counts quoted above are the ones
//! that stood when `sphere` closed, and none of them stands now:
//! `parry::rejection_slack` later took the whole sweep to 0 misses, `sphere`
//! included. See `mesh_orientation_tangency_is_caught_at_exact_tangency.rs`,
//! which was the `box` pin and now asserts the fix. This file's own subject
//! is untouched by that -- it pins the *rescue path* for `sphere`, which is
//! reached before any of it and is what the control below still measures.

use std::collections::BTreeSet;
use std::sync::Arc;

use cspace_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_geometry::{Isometry3, Mesh, Shape, Sphere, UnitQuaternion, Vector3};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_srdf::SrdfModel;
use cspace_state::RobotState;
use nalgebra::{Translation3, Unit};

/// Half the extent of both shapes, along every axis -- matches
/// `mesh_orientation_tangency_is_caught_at_exact_tangency.rs::HALF`.
const HALF: f64 = 0.5;
/// The point the sphere's own centre-minus-radius and the tilted mesh's own
/// lowest rotated vertex are both translated onto.
const TOUCH: (f64, f64, f64) = (5.0, 0.0, 0.0);

/// The same 8 vertices `mesh_orientation_tangency_is_caught_at_exact_tangency.rs::cube_vertices`
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
/// z (negative moves it toward more overlap, positive opens a real gap) --
/// one of the 2594 `mesh x sphere` miss poses `mesh_orientation_probe.rs`'s
/// own sweep found before this fix.
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

/// Measured by `mesh_orientation_probe` as a miss before this fix and a hit
/// after: `check_robot_collision` now reports a collision for `mesh x
/// sphere` at exact tangency, matching fcl's own stable target.
#[test]
fn a_5_degree_tilted_mesh_on_a_sphere_collides_at_exact_tangency() {
    assert!(collides(0.0));
}

/// The same pose with `1e-14` of added overlap also collides -- unsurprising
/// on its own, but confirms the pose is a real near-tie rather than a
/// construction artefact the widened prediction happens to catch regardless
/// of geometry.
#[test]
fn a_hair_more_overlap_on_the_same_pose_does_collide() {
    assert!(collides(-1e-14));
}

/// The same pose opened by `1e-9` -- roughly five orders of magnitude past
/// `tangent_pair_touches`'s own widened margin (`2 * TIE_ROUNDING_MARGIN *
/// f64::EPSILON * tie_scale`, on the order of `1e-14` at this pose's `TOUCH`
/// magnitude) -- does not collide. The widened second `query::contact` call
/// only reaches as far as that margin; it does not swallow a real clearance.
#[test]
fn a_real_gap_past_the_widened_margin_still_reports_no_collision() {
    assert!(!collides(1e-9));
}
