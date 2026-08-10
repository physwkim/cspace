// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: upstream's equivalent test
// (`moveit_core/collision_detection/test/test_all_valid.cpp`) only
// default-constructs a `CollisionEnvAllValid` and calls nothing on it, so
// there is nothing there to port. This file tests what that one does not:
// that the null backend is *reachable through the selection path* this port
// uses in place of `CollisionDetectorAllocatorAllValid`.

//! [`AllValidCollisionEnv`] is selected, not merely returning `false`.
//!
//! A null collision detector is the one backend whose correct answer is also
//! the answer you get from never calling it, so "it reported no collision"
//! proves nothing on its own. Every test here pins the difference instead:
//! the same [`PlanningScene`], the same current state, the same ACM and the
//! same request, answered twice, differing only in which type the caller
//! named for the `E: CollisionEnv<..>` parameter. A backend that was never
//! reached cannot produce two different answers to one question.
//!
//! That parameter *is* the selection path. Upstream selects this backend by
//! registering `CollisionDetectorAllocatorAllValid` (whose `NAME` is
//! `"ALL_VALID"`) and looking the name up in `PlanningScene`'s
//! `collision_detector_` map; this port has neither the allocator nor the
//! map (`PORTING-PLAN.md` §225.4, and `cspace_collision`'s `env` module
//! doc), and a caller chooses a backend by naming its type at the call
//! site. So these are the production selection sites, not a test-local
//! stand-in for one.

use std::sync::Arc;

use cspace_collision::{
    AllValidCollisionEnv, CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World,
};
use cspace_core::geometry::{Isometry3, Shape, Sphere};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_scene::PlanningScene;

/// pr2 with no mesh search paths, the same fixture recipe
/// `cspace-collision`'s `tests/multi_shape_object.rs` uses and for the same
/// reason: pr2 carries a *primitive* collision shape on `base_footprint`
/// near the origin, so a small primitive world object placed there collides
/// without any mesh having to resolve. panda and fanuc carry only `<mesh>`
/// collision geometry, which would make the "collides" half of every
/// assertion below depend on mesh loading rather than on backend selection.
fn pr2() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("fixture URDF must read");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf(), &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

/// A 0.1-radius sphere at the origin, inside pr2's `base_footprint` box.
fn colliding_world() -> World {
    let mut world = World::new();
    world.add_shape(
        "obstacle",
        Arc::new(Shape::Sphere(
            Sphere::new(0.1).expect("0.1 is a valid sphere radius"),
        )),
        Isometry3::identity(),
    );
    world
}

/// The control: this fixture really does collide, so `false` from the null
/// backend below is a changed answer rather than the only answer available.
/// If this assertion ever fails, every other assertion in this file becomes
/// vacuous, which is why it is its own named case.
#[test]
fn the_fixture_collides_under_a_real_backend() {
    let model = pr2();
    let mut scene = PlanningScene::new(&model, &srdf());
    let parry = ParryCollisionEnv::new(colliding_world(), LinkPaddingScale::default());
    assert!(
        scene
            .check_collision(&parry, &CollisionRequest::default())
            .collision,
        "pr2 at its default state must collide with a 0.1 sphere at the origin"
    );
}

#[test]
fn check_collision_answers_by_the_backend_the_caller_named() {
    let model = pr2();
    let mut scene = PlanningScene::new(&model, &srdf());
    let request = CollisionRequest::default();
    let parry = ParryCollisionEnv::new(colliding_world(), LinkPaddingScale::default());

    let with_parry = scene.check_collision(&parry, &request).collision;
    let with_all_valid = scene
        .check_collision(&AllValidCollisionEnv, &request)
        .collision;

    assert!(with_parry, "the real backend must find the obstacle");
    assert!(
        !with_all_valid,
        "the null backend must report the same scene clear"
    );
}

/// `is_state_colliding` is the entry point a planner uses to reject a
/// sample, so this is the shape "run the pipeline with collision checking
/// disabled" actually takes here.
#[test]
fn is_state_colliding_answers_by_the_backend_the_caller_named() {
    let model = pr2();
    let mut scene = PlanningScene::new(&model, &srdf());
    let request = CollisionRequest::default();
    let parry = ParryCollisionEnv::new(colliding_world(), LinkPaddingScale::default());

    assert!(scene.is_state_colliding(&parry, &request));
    assert!(!scene.is_state_colliding(&AllValidCollisionEnv, &request));
}

/// The discriminator that does not rest on `false` at all: `f64::MAX` is a
/// value only [`AllValidCollisionEnv::distance_robot`] produces here, so this
/// case would fail against a scene that skipped the backend and reported a
/// default. It is also the query upstream answers two different ways
/// depending on the static type of the expression — see
/// `doc/upstream-bugs.md`'s `all-valid-distance-robot-hides-base-overload`.
#[test]
fn distance_to_collision_through_the_null_backend_is_maximum_clearance() {
    let model = pr2();
    let mut scene = PlanningScene::new(&model, &srdf());
    let parry = ParryCollisionEnv::new(colliding_world(), LinkPaddingScale::default());

    let with_parry = scene.distance_to_collision(&parry);
    let with_all_valid = scene.distance_to_collision(&AllValidCollisionEnv);

    assert!(
        with_parry < f64::MAX,
        "the real backend must report a finite clearance, got {with_parry}"
    );
    assert_eq!(
        with_all_valid,
        f64::MAX,
        "the null backend reports maximum clearance, not upstream's hidden 0.0"
    );
}
