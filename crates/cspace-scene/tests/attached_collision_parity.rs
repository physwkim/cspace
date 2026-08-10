// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision` op's
//! `attached_bodies` handling, ground truth for
//! [`PlanningScene::attach_new`] feeding
//! [`cspace_collision::AttachedBodyGeometry`] into
//! [`PlanningScene::check_robot_collision`].
//!
//! `robot_distance` is checked via a direct, signed-distance
//! [`CollisionEnv::distance_robot`] call built from the scene's own posed
//! state and attached-body snapshot, not via
//! [`PlanningScene::distance_to_collision`]: that convenience method
//! reproduces upstream `PlanningScene::distanceToCollision`'s own
//! `enable_signed_distance: false` default (confirmed by reading
//! `collision_env.hpp`'s convenience `distanceRobot` overloads and
//! `collision_common.hpp`'s `DistanceRequest` default), so a real
//! penetration clamps to `0.0` there by design -- the oracle's `robot_distance`
//! field, by contrast, is captured with `enable_signed_distance: true` (see
//! `oracle.cpp`'s `collision()` doc), so comparing it against
//! `distance_to_collision`'s output would be comparing two deliberately
//! different quantities, not a same-quantity parity check.
//!
//! Built on pr2, not panda/fanuc: `crates/cspace-collision/tests/collision_parity.rs`'s
//! module doc records that panda/fanuc build with zero collision geometry on
//! this port (no mesh loader), so an attached-body effect there would be
//! invisible on both sides for the wrong reason. pr2's `base_footprint` has
//! real primitive collision geometry (a small box) and sits at the model's
//! root, so its global pose is identity at `joint_values: {}` regardless of
//! the rest of the kinematic tree -- confirmed live against the oracle's `fk`
//! op.
//!
//! Only `robot_collision`/`robot_distance` are asserted here, matching
//! `pr2_robot_collision_matches_the_oracle`'s reasoning in
//! `collision_parity.rs`: pr2's `self_collision` disagrees with the oracle
//! almost everywhere on this port because most of pr2's real self-collision
//! surface is mesh geometry this port does not load, which would swamp any
//! attached-body-specific signal.
//!
//! Ground truth was captured by hand against the oracle's `collision` op
//! (`--urdf fixtures/pr2.urdf --srdf fixtures/pr2.srdf`, `attached_bodies`
//! request field), verified reproducible across two independent runs of a
//! freshly built oracle image. Case 0 (no attachment) reproduces the
//! existing `pr2_collision.json` case 0 exactly. Case 1 attaches a
//! `radius: 0.1` sphere to `base_footprint` translated `+0.5m` in z -- well
//! clear of the `4x4x0.1` floor at `z <= 0` -- and reproduces case 0's
//! numbers exactly, showing a distant attached body does not perturb the
//! result. Case 2 attaches the same sphere translated `-0.1m` in z, driving
//! it `0.1m` into the floor's top face at `z = 0`; the oracle reports
//! `robot_collision: true, robot_distance: -0.1` bit-exact with the offset.
//!
//! `pr2_attached_collision.json` is a hand-built "cases" shape (one summary
//! object per case, not the literal oracle wire request), so it is not
//! covered by `tools/ci/verify-fixture-replay.sh`'s `oracle-models.json`
//! manifest: replaying it would mean reconstructing a `collision` wire
//! request from these summary fields for each case, a different job from
//! diffing an already-captured request/response pair. Intentionally absent,
//! not an oversight.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{
    AttachedBodyGeometry, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_geometry::{Cuboid, Isometry3, Shape, Sphere};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_scene::{AttachedBody, PlanningScene};
use cspace_srdf::SrdfModel;

#[derive(Deserialize)]
struct AttachedBodyCase {
    id: String,
    link_name: String,
    sphere_radius: f64,
    shape_translation: [f64; 3],
    touch_links: Vec<String>,
}

#[derive(Deserialize)]
struct CollisionCase {
    joint_values: BTreeMap<String, f64>,
    attached_bodies: Vec<AttachedBodyCase>,
    robot_collision: bool,
    robot_distance: f64,
}

#[derive(Deserialize)]
struct CollisionFixture {
    cases: Vec<CollisionCase>,
}

fn load_fixture() -> CollisionFixture {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/pr2_attached_collision.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn build_model() -> RobotModel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    let urdf_xml = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

/// The same `4x4x0.1` floor box, at the same pose, `collision_parity.rs`'s
/// `floor_env` and the oracle fixtures were captured against.
fn floor_env(world: World) -> ParryCollisionEnv {
    let mut world = world;
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, 0.1).expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -0.05),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

/// `1e-4`, per PORTING-PLAN.md §5's distance tolerance for Phase 3's
/// completion condition.
const TOLERANCE: f64 = 1e-4;

#[test]
fn pr2_attached_body_robot_collision_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let fixture = load_fixture();

    for (case_index, case) in fixture.cases.iter().enumerate() {
        let mut scene = PlanningScene::new(&model, &srdf);
        for (name, &value) in &case.joint_values {
            scene
                .current_state_mut()
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("setting {name}: {e}"));
        }
        for attached in &case.attached_bodies {
            scene
                .attach_new(
                    &attached.id,
                    &attached.link_name,
                    vec![Arc::new(Shape::Sphere(
                        Sphere::new(attached.sphere_radius)
                            .expect("fixture sphere_radius must be positive"),
                    ))],
                    vec![Isometry3::translation(
                        attached.shape_translation[0],
                        attached.shape_translation[1],
                        attached.shape_translation[2],
                    )],
                    attached.touch_links.iter().cloned().collect(),
                    BTreeMap::new(),
                )
                .unwrap_or_else(|e| panic!("attach {}: {e}", attached.id));
        }

        let env = floor_env(scene.world().clone());

        let robot_result = scene.check_robot_collision(&env, &CollisionRequest::default());
        assert_eq!(
            robot_result.collision, case.robot_collision,
            "case {case_index}: robot_collision"
        );

        let acm = scene.allowed_collision_matrix().clone();
        let attached_bodies: Vec<AttachedBody> = scene.attached_bodies().cloned().collect();
        let posed = scene.current_state_mut().update();
        let attached: Vec<AttachedBodyGeometry<'_>> = attached_bodies
            .iter()
            .map(AttachedBody::as_geometry)
            .collect();
        let distance_request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let robot_distance = env
            .distance_robot(&distance_request, &posed, &attached)
            .minimum_distance
            .distance;
        assert!(
            (robot_distance - case.robot_distance).abs() < TOLERANCE,
            "case {case_index}: robot_distance {robot_distance} != {} (oracle)",
            case.robot_distance
        );
    }
}
