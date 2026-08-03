// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision` op.
//!
//! Ground truth is the oracle's own response, captured verbatim into
//! `tests/fixtures/pr2_collision.json`: one default case
//! (`joint_values: {}`) plus three cases sampled by the oracle's own
//! `random_states` op, exactly `fk_parity.rs`'s pattern. Every case checks the
//! robot against one fixed world object -- a `4x4x0.1` floor box centered at
//! `(0, 0, -0.05)`, so its top face sits at `z = 0` -- built via
//! [`ParryCollisionEnv::check_self_collision`]/[`check_robot_collision`]/
//! [`distance_self`]/[`distance_robot`] with `enable_signed_distance: true`.
//!
//! Contact-point/nearest-point coordinates are never compared here.
//! PORTING-PLAN.md §4.5 records that exclusion as Phase 3's recorded
//! verification limit, not an oversight: `parry.rs`'s own module doc
//! (deviations 4 and 6) establishes that this backend's contact geometry
//! differs from FCL's by construction (at most one contact per pair, taken
//! from a single `parry3d_f64::query::contact` call, versus FCL's up to 200
//! contacts per pair) in ways that cannot converge under any tolerance --
//! only `collision`/`distance` are architecturally comparable at all.
//!
//! # No panda or fanuc case, and no `self_collision` case for pr2
//!
//! Every panda/fanuc link's collision geometry is exactly one `<mesh>`
//! element, and `moveit_model::LinkModel` does not load `<mesh>` collision
//! geometry at all (see its own doc comment, deviation 4: no mesh-file
//! loader exists, and the mesh files live under the gitignored
//! `third_party/` anyway). Panda and fanuc therefore build with **zero**
//! collision geometry on this port, regardless of backend -- confirmed by a
//! live 10,000-state sweep against the oracle (`tools/ci/run-oracle-sweep.sh`
//! style, `--collision`): panda disagreed with the oracle on `collision:
//! bool` in all 10,000 cases (1,266 on `self_collision`, 8,734 on
//! `robot_collision`, every rust-side distance exactly `f64::MAX` --
//! "no pair was ever evaluated", not a near-boundary numerical disagreement).
//!
//! Fanuc used to have a fixture here that agreed with the oracle on every
//! field. That agreement was an artifact of the oracle, not of this port:
//! the image built `--packages-up-to moveit_core`, which does not pull in
//! `moveit_resources_fanuc_description`, so the C++ side could not resolve
//! fanuc's `package://` mesh URIs either and answered every query about a
//! robot with no collision geometry at all -- `robot_collision: false`,
//! `self_collision: false`, both distances `DBL_MAX`, in all four cases.
//! With that package built in, the same four cases come back
//! `robot_collision: true` in 4 of 4 (`robot_distance` ~ -1e-15, the base
//! resting exactly on the floor) and `self_collision: true` in 2 of 4. Every
//! field disagrees with this port, for the same reason panda's does, so
//! fanuc is now in panda's position: no fixture, no test, until the mesh
//! loader lands.
//!
//! PR2 mixes `<mesh>` with a handful of small `<box>`/`<cylinder>`/`<sphere>`
//! links (gripper fingertips, a laser mount), so it is not geometry-free, but
//! that leftover set is not what pr2's own real self-collision surface (torso
//! against arms) is made of: the same live sweep found `self_collision`
//! disagreed in 9,999 of 10,000 cases, with rust's `self_distance` always
//! landing near a single, nearly pose-invariant value (~2.9 cm -- the
//! leftover primitive pair's own separation, essentially independent of arm
//! configuration) while the oracle's real mesh-driven self-collision varies
//! per state. `robot_collision`, by contrast, agreed in 9,999 of 10,000: pr2's
//! base has real box collision geometry covering it, so the floor-vs-base
//! check genuinely exercises the same geometry on both sides. This file
//! therefore asserts `robot_collision`/`robot_distance` parity for pr2 but not
//! `self_collision`/`self_distance`.
//!
//! None of this is a defect in [`ParryCollisionEnv`] itself: every
//! disagreement in the underlying sweep traced to rust-side geometry being
//! either absent (panda, fanuc) or incidental (pr2's self-collision leftover
//! primitives), and the one case this file *can* assert (pr2's
//! `robot_collision`) passes at bit-for-bit distance agreement.
//! `moveit-model`'s mesh loading is out of this crate's scope and owned by
//! another worker.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

/// `self_collision`/`self_distance` are present in the fixture but not
/// deserialized: pr2 is the only robot left here and its self-collision is
/// excluded on purpose (see the module doc). Serde ignores the extra keys, so
/// the fixture stays a verbatim capture of the oracle's response rather than a
/// trimmed one -- the day the mesh loader lands, asserting them again is a
/// matter of adding the fields back, not re-capturing.
#[derive(Deserialize)]
struct CollisionCase {
    joint_values: BTreeMap<String, f64>,
    robot_collision: bool,
    robot_distance: f64,
}

#[derive(Deserialize)]
struct CollisionFixture {
    cases: Vec<CollisionCase>,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load_fixture(file_name: &str) -> CollisionFixture {
    let path = fixture_path(file_name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        urdf_file
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let urdf_xml = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(&path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// The oracle's `collision` op checks against
/// `AllowedCollisionMatrix(*model_->getSRDF())` (`buildAcm` in `oracle.cpp`),
/// so every case in the fixture was captured with pr2's
/// `disable_collisions` entries suppressing the pairs they name. Without
/// applying the same matrix here, this test would disagree with the oracle
/// on exactly those suppressed pairs -- not a `ParryCollisionEnv` defect, a
/// missing input.
fn build_acm(srdf_file: &str) -> AllowedCollisionMatrix {
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// The same `4x4x0.1` floor box, at the same pose, the oracle fixtures were
/// captured against (see the module doc). Built once per test the same way
/// `tools/moveit-diff`'s own `collision_scene` is, so both crates' tests bear
/// out the identical geometry that comparison relies on.
fn floor_env() -> ParryCollisionEnv {
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, 0.1).expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -0.05),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

fn build_state<'m>(model: &'m RobotModel, joint_values: &BTreeMap<String, f64>) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

/// `1e-4`, per PORTING-PLAN.md §5's distance tolerance for Phase 3's
/// completion condition.
const TOLERANCE: f64 = 1e-4;

/// Only `robot_collision`/`robot_distance` -- see the module doc for why
/// `self_collision` is excluded for pr2.
fn assert_robot_collision_matches_oracle(model: &RobotModel, fixture_name: &str, srdf_file: &str) {
    let env = floor_env();
    let acm = build_acm(srdf_file);
    let fixture = load_fixture(fixture_name);
    for (case_index, case) in fixture.cases.iter().enumerate() {
        let mut state = build_state(model, &case.joint_values);
        let posed = state.update();

        let robot_result =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
        assert_eq!(
            robot_result.collision, case.robot_collision,
            "{fixture_name} case {case_index}: robot_collision"
        );

        let distance_request = DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        };
        let robot_distance = env.distance_robot(&distance_request, &posed, &[]);
        assert!(
            (robot_distance.minimum_distance.distance - case.robot_distance).abs() < TOLERANCE,
            "{fixture_name} case {case_index}: robot_distance {} != {} (oracle)",
            robot_distance.minimum_distance.distance,
            case.robot_distance
        );
    }
}

#[test]
fn pr2_robot_collision_matches_the_oracle() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    assert_robot_collision_matches_oracle(&model, "pr2_collision.json", "pr2.srdf");
}
