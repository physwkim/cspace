// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The reproducer for `doc/upstream-bugs.md`'s
//! `distance-callback-max-contact-depth`, and the measurement that closes
//! PORTING-PLAN.md §5 Phase 3's `distance: f64` clause.
//!
//! # What this pins
//!
//! A pair's penetration depth is the length of the shortest translation that
//! separates the two bodies. It therefore cannot depend on how *wide* the
//! other body is: widening a floor slab sideways adds no material between the
//! robot and the surface it is resting in, so the shortest way out is
//! unchanged. Every test below is that one invariant, applied to a floor box
//! grown from `0.4 m` to `20 m` across while its top face stays put.
//!
//! Upstream fails this invariant, and that -- not numerical disagreement -- is
//! what the Phase 3 `distance` clause was measuring. `distanceCallback`
//! (`collision_detection_fcl/collision_common.cpp:648-663` at `e017c91ee`) runs `fcl::collide` with
//! `num_max_contacts = 200` and keeps the contact with the **largest**
//! `penetration_depth`. For a mesh link that is `fcl::collide` per *triangle*,
//! and a triangle lying entirely inside a large box has no separating axis, so
//! FCL reports its escape along a lateral direction -- a distance that grows
//! with the box. Taking the maximum promotes that artifact over the ~200 sane
//! contacts beside it. Measured against the oracle at `e017c91ee`, for
//! `panda_link0` resting `0.05 m` into a floor whose top face is at `z =
//! +0.05`:
//!
//! | floor | oracle `robot_distance` | oracle *median* contact depth | this backend |
//! |---|---|---|---|
//! | `0.4 x 0.4 x 0.1` | `-0.225255` | -- | `-0.05003249277506257` |
//! | `1 x 1 x 0.1`     | `-0.644284` | -- | `-0.05003249277506257` |
//! | `4 x 4 x 0.1`     | `-2.763224` | `0.049999832` | `-0.05003249277506257` |
//! | `20 x 20 x 0.1`   | `-9.999483` | `0.049999698` | `-0.05003249277506257` |
//!
//! The oracle's own median contact is the correct `~0.05` at both widths where
//! the contact set was dumped; only its `max` selection moves. This backend
//! agrees with that median to `3.3e-5` -- inside Phase 3's `1e-4` clause --
//! and its spread across the four widths is exactly `0.000000e0`. At `20 m`
//! the oracle reports `9.999483 m` of penetration for a link whose entire
//! collision mesh fits in a sphere of radius `0.154636 m`: 32x the deepest
//! overlap the link could have at any pose, in any orientation. That is why
//! widening the `1e-4` tolerance was never available -- the gap is not an
//! error of measurement, it is a different quantity, and it is unbounded in
//! the floor's width.
//!
//! # Scope
//!
//! These tests need no oracle and no docker: the invariant is checkable
//! against this backend alone, which is the point -- an invariant that only
//! holds relative to the oracle could not have caught the oracle. The oracle
//! numbers quoted above were measured with
//! `tools/moveit-oracle/run-oracle.sh` and are recorded in the upstream-bugs
//! entry, not re-measured here.

use std::fs;
use std::sync::Arc;

use cspace_collision::{
    AllowedCollisionMatrix, CollisionEnv, DistanceRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

/// The floor widths swept. `4.0` is the width `tools/moveit-diff`'s own
/// `collision_scene` uses, so the middle row is the configuration Phase 3's
/// sweep actually measured; the others bracket it by a factor of ten each way.
const FLOOR_WIDTHS: [f64; 4] = [0.4, 1.0, 4.0, 20.0];

/// Top face of the floor slab. Held fixed across [`FLOOR_WIDTHS`] so that the
/// only thing changing between cases is the width -- with `panda_link0`
/// spanning `z = [-0.000032, 0.140003]`, this puts `0.05 m` of it inside the
/// slab in every case.
const FLOOR_TOP_Z: f64 = 0.05;

/// Slab thickness, matching `collision_scene`'s.
const FLOOR_THICKNESS: f64 = 0.1;

/// `panda_link0`'s bounding radius, measured from
/// `fixtures/meshes/panda_description/meshes/collision/link0.stl`: the largest
/// `|vertex|` over its 200 triangles is `0.154636 m`. Asserted rather than
/// trusted -- [`link0_bounding_radius_is_what_the_depth_bound_assumes`] rebuilds
/// it from the loaded model so this constant cannot silently drift from the
/// mesh it describes.
const LINK0_BOUNDING_RADIUS: f64 = 0.154_636;

fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn build_panda() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// A floor slab `width x width x FLOOR_THICKNESS`, positioned so its top face
/// is at [`FLOOR_TOP_Z`] whatever the width.
fn floor_env(width: f64) -> ParryCollisionEnv {
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(width, width, FLOOR_THICKNESS).expect("positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, FLOOR_TOP_Z - FLOOR_THICKNESS / 2.0),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

/// This backend's `robot_distance` for the default panda state against a floor
/// of the given width, paired with the link names it attributes that distance
/// to.
fn robot_distance_at(width: f64) -> (f64, String, String) {
    let model = build_panda();
    let acm = build_acm();
    let env = floor_env(width);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    let request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let result = env.distance_robot(&request, &posed, &[]);
    let data = result.minimum_distance;
    (
        data.distance,
        data.link_names[0].clone(),
        data.link_names[1].clone(),
    )
}

/// The invariant itself: widening the floor by a factor of fifty must not
/// change the reported penetration depth.
///
/// Deliberately *not* expressed as "equals the oracle" or "equals `-0.05`".
/// Pinning the literal would make this a regression test for one number;
/// pinning the *spread across widths* makes it a test of the property that
/// upstream violates, and it fails for any backend whose depth tracks the
/// other body's size regardless of what that depth is.
#[test]
fn depth_is_invariant_to_floor_width() {
    let measured: Vec<(f64, f64)> = FLOOR_WIDTHS
        .iter()
        .map(|&w| (w, robot_distance_at(w).0))
        .collect();

    let depths: Vec<f64> = measured.iter().map(|&(_, d)| d).collect();
    let min = depths.iter().copied().fold(f64::INFINITY, f64::min);
    let max = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    // 1e-9, not the 1e-4 of Phase 3's clause: this is one backend compared
    // against itself on inputs that differ only in an irrelevant dimension, so
    // any spread at all is a defect. The looser Phase 3 figure is a
    // cross-backend budget and would pass a real width dependence of 1e-5.
    assert!(
        max - min < 1e-9,
        "penetration depth must not depend on the floor's width, but it spread by {:.6e} across \
         widths {:?}: {:?}",
        max - min,
        FLOOR_WIDTHS,
        measured
    );
}

/// The same invariant stated as the physical bound it follows from, and the
/// one that makes upstream's answer impossible rather than merely different.
///
/// Separated from [`depth_is_invariant_to_floor_width`] because the two fail
/// for different reasons: a backend could be perfectly width-*invariant* and
/// still report a depth larger than the link, and a backend could stay under
/// the bound while drifting with width. Upstream fails both, but only at
/// widths large enough -- at `0.4 m` its `0.225255` is under `0.309272` and
/// this bound alone would clear it. That is why the width sweep is the
/// primary test and this is the companion.
#[test]
fn depth_never_exceeds_the_links_own_diameter() {
    for &width in &FLOOR_WIDTHS {
        let (distance, a, b) = robot_distance_at(width);
        let bound = 2.0 * LINK0_BOUNDING_RADIUS;
        assert!(
            distance.abs() <= bound,
            "floor width {width}: reported |{distance}| for {a}/{b} exceeds {bound} -- a rigid \
             body cannot overlap anything by more than its own diameter, so this is not a \
             penetration depth"
        );
    }
}

/// [`LINK0_BOUNDING_RADIUS`] is a constant copied from a mesh; this rebuilds it
/// from the model actually loaded, so the bound in
/// [`depth_never_exceeds_the_links_own_diameter`] cannot go stale against a
/// re-vendored `link0.stl` and silently become a weaker (or vacuous) check.
#[test]
fn link0_bounding_radius_is_what_the_depth_bound_assumes() {
    let model = build_panda();
    let link = model
        .link_model("panda_link0")
        .expect("panda_link0 must resolve");
    let measured = link
        .shapes()
        .iter()
        .map(|link_shape| {
            let local = match &link_shape.shape {
                Shape::Mesh(mesh) => mesh
                    .vertices
                    .iter()
                    .map(|v| nalgebra::Point3::from(*v).coords.norm())
                    .fold(0.0_f64, f64::max),
                other => panic!("panda_link0's collision geometry is a mesh, found {other:?}"),
            };
            link_shape.origin_transform.translation.vector.norm() + local
        })
        .fold(0.0_f64, f64::max);

    assert!(
        (measured - LINK0_BOUNDING_RADIUS).abs() < 1e-6,
        "LINK0_BOUNDING_RADIUS is {LINK0_BOUNDING_RADIUS} but the loaded mesh measures {measured}"
    );
}

/// The pair itself has to stay put, or the width sweep above could hold for the
/// trivial reason that a *different* pair wins at each width.
#[test]
fn the_same_pair_wins_at_every_floor_width() {
    let pairs: Vec<(f64, String, String)> = FLOOR_WIDTHS
        .iter()
        .map(|&w| {
            let (_, a, b) = robot_distance_at(w);
            (w, a, b)
        })
        .collect();
    let (_, ref first_a, ref first_b) = pairs[0];
    for (width, a, b) in &pairs {
        assert!(
            a == first_a && b == first_b,
            "floor width {width} ranked {a}/{b} deepest, but width {} ranked {first_a}/{first_b} \
             -- the width sweep only means something if the pair is held fixed",
            FLOOR_WIDTHS[0]
        );
    }
}

/// A guard on the fixture rather than the backend: if `panda_link0` ever stops
/// actually penetrating the slab, every assertion above passes vacuously on a
/// non-colliding pair whose distance is a plain positive gap.
#[test]
fn the_default_state_really_does_penetrate_the_floor() {
    for &width in &FLOOR_WIDTHS {
        let (distance, a, b) = robot_distance_at(width);
        assert!(
            distance < 0.0,
            "floor width {width}: expected panda_link0 to penetrate the slab, but {a}/{b} \
             reports a positive gap of {distance}"
        );
    }
}

/// The pose every measurement above is stated against. The module doc's
/// arithmetic -- `panda_link0` spanning `z = [-0.000032, 0.140003]`, so
/// `0.05 m` of it inside a slab whose top face is at `z = +0.05` -- holds only
/// while `set_to_default_values` leaves `panda_link0` at the world origin.
///
/// Checks the resulting transform rather than the joint variables: panda's
/// default state is not the all-zero vector (its virtual joint carries the
/// identity quaternion, whose `rot_w` is `1.0`), so a variables-are-zero
/// assertion would fail on a correct model while still saying nothing about
/// where the link ended up.
#[test]
fn the_default_state_leaves_link0_at_the_world_origin() {
    let model = build_panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    let transform = posed
        .global_link_transform("panda_link0")
        .expect("panda_link0 must resolve");
    let translation = transform.translation.vector.norm();
    let rotation = transform.rotation.angle();
    assert!(
        translation < 1e-12 && rotation < 1e-12,
        "panda_link0 must sit at the world origin in the default state, but it is translated by \
         {translation} and rotated by {rotation} rad"
    );
}
