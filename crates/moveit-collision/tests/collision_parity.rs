// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision` op.
//!
//! Ground truth is the oracle's own response, captured verbatim into
//! `tests/fixtures/{panda,fanuc,pr2}_collision.json`: one default case
//! (`joint_values: {}`) plus three cases sampled by the oracle's own
//! `random_states` op, exactly `fk_parity.rs`'s pattern. Every case checks the
//! robot against one fixed world object -- a `4x4x0.1` floor box centered at
//! `(0, 0, -0.05)`, so its top face sits at `z = 0` -- built via
//! [`ParryCollisionEnv::check_self_collision`]/[`check_robot_collision`]/
//! [`distance_self`]/[`distance_robot`] with `enable_signed_distance: true`.
//!
//! `moveit_model::LinkModel` now loads `<mesh>` collision geometry (STL,
//! resolved through [`MeshSearchPaths`] -- see that type and
//! `moveit-geometry`'s `stl` module), so panda and fanuc, whose collision
//! geometry is exactly one `<mesh>` element per link, build with real
//! collision shapes here rather than none at all. [`fixture_mesh_search_paths`]
//! points at the two packages committed under `fixtures/meshes/`.
//!
//! Contact-point/nearest-point coordinates are never compared here.
//! PORTING-PLAN.md §4.5 records that exclusion as Phase 3's recorded
//! verification limit, not an oversight: `parry.rs`'s own module doc
//! (deviations 4 and 6) establishes that this backend's contact geometry
//! differs from FCL's by construction (at most one contact per pair, taken
//! from a single `parry3d_f64::query::contact` call, versus FCL's up to 200
//! contacts per pair) in ways that cannot converge under any tolerance.
//!
//! # No exact penetration-depth parity for interpenetrating meshes
//!
//! With real mesh geometry loaded, this is the first ground truth exercising
//! `parry.rs`'s deviation 6 in the actually-penetrating regime it describes:
//! upstream's `distanceCallback`, once a pair is confirmed touching or
//! penetrating, re-runs `fcl::collide` (up to 200 contacts) and takes the
//! *maximum* penetration depth found; this backend's single
//! `parry3d_f64::query::contact` call returns exactly one (not necessarily
//! the deepest) contact for the whole pair. For two convex primitives that
//! never differ -- there is only one contact to find -- but for a mesh
//! overlapping another shape across many triangles, FCL's 200-sample search
//! routinely finds a deeper local penetration than this backend's single EPA
//! result, and the two numbers do not converge under any tolerance.
//!
//! Confirmed at two scales: `fanuc_collision.json`'s two self-colliding
//! cases (mesh vs. mesh) -- case 1 (`link_1`/`link_4`): oracle `self_distance
//! = -0.01624`, this backend `-0.00561`; case 2 (`link_1`/`link_5`): oracle
//! `-0.07129`, this backend `-0.02322` -- and, larger still, a live
//! `tools/moveit-diff --collision` sweep of `panda_arm` (mesh vs. the
//! floor's `<box>`): `robot_distance` disagreed by roughly `1.7`-`1.9` on
//! every case where a link actually penetrated the floor (e.g. oracle
//! `-1.896`, this backend `-0.149`), while every *non*-penetrating case (the
//! two fanuc cases above, and every panda case with `robot_collision:
//! false`) agrees to `~1e-9`-`~1e-16`. The divergence is confined exactly to
//! the interpenetrating regime deviation 6 predicts, not a general
//! mesh-distance defect. [`assert_full_parity_matches_oracle`] therefore
//! asserts full distance-magnitude parity only when the oracle reports no
//! collision on that side; when it reports a collision, only the sign (`<=
//! TOLERANCE`) is asserted, matching what the boolean
//! `self_collision`/`robot_collision` check already independently confirms.
//!
//! # No `self_collision` case for pr2
//!
//! PR2 mixes `<mesh>` with a handful of small `<box>`/`<cylinder>`/`<sphere>`
//! links (gripper fingertips, a laser mount) that this port does load, but
//! that leftover primitive set is not what pr2's own real self-collision
//! surface (torso against arms) is made of: a live 10,000-state sweep found
//! `self_collision` disagreed in 9,999 of 10,000 cases, with rust's
//! `self_distance` always landing near a single, nearly pose-invariant value
//! (~2.9 cm -- the leftover primitive pair's own separation, essentially
//! independent of arm configuration) while the oracle's real mesh-driven
//! self-collision varies per state. `robot_collision`, by contrast, agreed in
//! 9,999 of 10,000: pr2's base has real box collision geometry covering it,
//! so the floor-vs-base check genuinely exercises the same geometry on both
//! sides. This file therefore asserts `robot_collision`/`robot_distance`
//! parity for pr2 but not `self_collision`/`self_distance`.

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

#[derive(Deserialize)]
struct CollisionCase {
    joint_values: BTreeMap<String, f64>,
    self_collision: bool,
    self_distance: f64,
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

/// The two `moveit_resources_*_description` packages committed under
/// `fixtures/meshes/` (see `tools/ci/verify-fixture-provenance.sh`) -- pr2's
/// meshes are not committed there, so its `<mesh>` collision elements stay
/// unresolved and skipped, exactly as before mesh loading existed. That is
/// fine: this file's pr2 test asserts only `robot_collision`, which pr2's
/// real `<box>`/`<cylinder>`/`<sphere>` links already cover without any mesh.
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([
        (
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        ),
        (
            "moveit_resources_fanuc_description",
            format!("{meshes_root}/fanuc_description"),
        ),
    ])
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
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

/// The oracle's `collision` op checks against
/// `AllowedCollisionMatrix(*model_->getSRDF())` (`buildAcm` in `oracle.cpp`),
/// so every case in the fixtures was captured with fanuc/pr2's
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

fn assert_full_parity_matches_oracle(model: &RobotModel, fixture_name: &str, srdf_file: &str) {
    let env = floor_env();
    let acm = build_acm(srdf_file);
    let fixture = load_fixture(fixture_name);
    for (case_index, case) in fixture.cases.iter().enumerate() {
        let mut state = build_state(model, &case.joint_values);
        let posed = state.update();

        let self_result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
        assert_eq!(
            self_result.collision, case.self_collision,
            "{fixture_name} case {case_index}: self_collision"
        );
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
        // Penetration depth for two overlapping general (non-convex) meshes
        // is only asserted for its sign, not its magnitude -- see the module
        // doc's "no exact penetration-depth parity for interpenetrating
        // meshes" section. Separating distances (both sides report
        // `collision: false`) are architecturally comparable and asserted at
        // full tolerance.
        let self_distance = env.distance_self(&distance_request, &posed, &[]);
        if case.self_collision {
            assert!(
                self_distance.minimum_distance.distance <= TOLERANCE,
                "{fixture_name} case {case_index}: self_distance {} should be <= 0 \
                 (oracle reports self_collision)",
                self_distance.minimum_distance.distance
            );
        } else {
            assert!(
                (self_distance.minimum_distance.distance - case.self_distance).abs() < TOLERANCE,
                "{fixture_name} case {case_index}: self_distance {} != {} (oracle)",
                self_distance.minimum_distance.distance,
                case.self_distance
            );
        }
        let robot_distance = env.distance_robot(&distance_request, &posed, &[]);
        if case.robot_collision {
            assert!(
                robot_distance.minimum_distance.distance <= TOLERANCE,
                "{fixture_name} case {case_index}: robot_distance {} should be <= 0 \
                 (oracle reports robot_collision)",
                robot_distance.minimum_distance.distance
            );
        } else {
            assert!(
                (robot_distance.minimum_distance.distance - case.robot_distance).abs() < TOLERANCE,
                "{fixture_name} case {case_index}: robot_distance {} != {} (oracle)",
                robot_distance.minimum_distance.distance,
                case.robot_distance
            );
        }
    }
}

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
fn panda_collision_matches_the_oracle() {
    let model = build_model("panda.urdf", "panda.srdf");
    assert_full_parity_matches_oracle(&model, "panda_collision.json", "panda.srdf");
}

#[test]
fn fanuc_collision_matches_the_oracle() {
    let model = build_model("fanuc.urdf", "fanuc.srdf");
    assert_full_parity_matches_oracle(&model, "fanuc_collision.json", "fanuc.srdf");
}

#[test]
fn pr2_robot_collision_matches_the_oracle() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    assert_robot_collision_matches_oracle(&model, "pr2_collision.json", "pr2.srdf");
}
