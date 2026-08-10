// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity tests against the moveit2 C++ oracle for
//! `collision_env_distance_field.rs`'s `add_link_body_decompositions` and
//! `generate_distance_field_cache_entry`.
//!
//! `add_link_body_decompositions`: two properties, two ops:
//!
//! - **Link *set* selection.** `add_link_body_decompositions` iterates
//!   `robot_model.link_models()` filtered to `!shapes().is_empty()`, in
//!   place of the missing `RobotModel::getLinkModelsWithCollisionGeometry()`
//!   (see that function's own doc comment). `link_models_with_collision_geometry_matches_the_oracle`
//!   checks that filter picks the exact same, exactly ordered link set as
//!   upstream's real method, via the new `link_models_with_collision_geometry`
//!   oracle op.
//! - **Per-link geometry.** `add_link_body_decompositions` builds each
//!   link's `BodyDecomposition` with `BodyDecomposition::from_shapes(link.shapes(),
//!   ..., resolution, link_padding.link_padding(name))` -- for an untracked
//!   link (this test's `LinkPaddingScale::new()`, tracking nothing) that
//!   padding is `0.0`. `collision_common_distance_field_parity.rs`'s
//!   `link_body_decomposition_matches_the_oracle` already oracle-verifies
//!   `BodyDecomposition::from_shapes` on `base_bellow_link`'s real shapes at
//!   `resolution: 0.05, padding: 0.0` -- the exact same inputs
//!   `add_link_body_decompositions` uses for that link at those settings, so
//!   `add_link_body_decompositions_matches_the_per_link_oracle_fixture`
//!   reuses that existing, already-committed fixture as ground truth for
//!   `base_bellow_link`'s entry rather than re-deriving the same numbers
//!   through a second oracle op.
//!
//! `test_collision_distance_field.cpp` gives no ground truth for either
//! function -- see `collision_env_distance_field.rs`'s own module doc for
//! why (every `TEST_F` case calls `checkSelfCollision`/`checkRobotCollision`,
//! none reach `addLinkBodyDecompositions`/`generateDistanceFieldCacheEntry`
//! directly). `generate_distance_field_cache_entry_matches_the_oracle` below
//! drives `generateDistanceFieldCacheEntry` indirectly instead, through the
//! `distance_field_cache_entry` op's `checkSelfCollision` ->
//! `getLastDistanceFieldEntry()` path (see that op's own doc comment in
//! `oracle.cpp`), across three cases: `right_arm` with an ACM (exercises the
//! self/intra-group exclusion logic against `pr2.srdf`'s real
//! `disable_collisions` entries), `right_arm` with no ACM (the
//! "enable everything" branch), and `l_end_effector` (a group with one
//! active joint and several mimics, exercising `state_check_indices`'
//! active-joint-variable exclusion against a group shape very different from
//! a chain).
//!
//! Every PR2 arm/gripper link's collision geometry is mesh-only, and
//! `cspace-model` can load STL `<mesh>` geometry now (`RobotModel::from_urdf_and_srdf`
//! takes a [`MeshSearchPaths`]). pr2's 18 `<collision>` mesh files are
//! committed under `fixtures/meshes/pr2_description/` (landed by p3-acm; see
//! `tools/ci/verify-fixture-provenance.sh`), the same way panda's and
//! fanuc's are, so [`fixture_mesh_search_paths`] resolves every one of them
//! and `build_pr2_model` builds with real collision shapes throughout --
//! `model.diagnostics()` is empty for pr2 now (confirmed directly: no
//! `UnsupportedLinkGeometry` entries), so every field this file compares
//! against the oracle -- `link_models_with_collision_geometry`'s link set,
//! `link_has_geometry`, `link_body_indices`, `self_collision_enabled`,
//! `intra_group_collision_enabled`, and the distance field's own
//! `distance_queries` -- is asserted by plain equality, no per-link
//! mesh-gap narrowing.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use nalgebra::Vector3;
use serde::Deserialize;

use cspace_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, BodyType, CollisionRequest, ContactData,
    LinkPaddingScale, World,
};
use cspace_core::geometry::{Isometry3, Shape, Sphere};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::test_support::isometry_from_row_major;
use cspace_distance_field::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GridGeometry,
    PropagationDistanceField, add_link_body_decompositions, collision_object_point_decomposition,
    generate_distance_field_cache_entry, group_state_representation,
};

/// Measured-margin tolerance, not policy: this constant used to pin `1e-4`
/// with no doc comment at all -- inherited from the other parity files in
/// this crate, never checked against what this file's own assertions
/// actually see. Bisected directly against every `assert_relative_eq!` call
/// in this file (with `max_relative` already pinned explicitly, so no
/// implicit `approx` default can hide the true floor).
///
/// **Re-bisected under `float_roundtrip` (PORTING-PLAN.md §115) -- part of
/// the original floor was contaminated.** The original `bounding_sphere_center.z`
/// binding site (`1e-16` fails, `left = 0.9616420264076277`, `right =
/// 0.9616420264076276`) no longer differs at all -- that was a fixture
/// parsing artifact (see the `sphere_radii` comparison below for the same
/// effect on a bit-exact scale). A different, genuine `bounding_sphere_center.z`
/// site now binds at the same order: `1e-16` fails
/// (`left = 1.1670184264413426`, `right = 1.1670184264413423`), `3.2e-16`
/// still fails. The `field_pose` quaternion rotation check's `TOL * TOL`
/// quadratic gate is unaffected by the parsing fix (identical values before
/// and after): `right_arm`'s `r_wrist_roll_link` gives `expected
/// [0.46547879812158294, 0.6080129633451752, -0.5383633189200836,
/// 0.35187307618636826]` up to sign, `got [0.46547879812158266,
/// 0.6080129633451752, -0.5383633189200835, 0.3518730761863682]`, and binds
/// the combined floor at `3.2e-16` fails / `3.5e-16` passes. `TOL = 5e-13`
/// keeps roughly three orders of margin above that `3.5e-16` boundary --
/// tightened from the old `1e-12`, `5e-16` boundary, which had rounded up
/// to `5e-16` partly on the strength of the now-fixed parsing artifact.
///
/// `max_relative = TOL` is passed explicitly alongside `epsilon` at every
/// `assert_relative_eq!` call below (the `TOL * TOL` quaternion check is a
/// plain `assert!`, not `assert_relative_eq!`, so it has no implicit
/// `max_relative` to worry about). Without the explicit `max_relative`,
/// `approx` falls back to `max_relative = f64::EPSILON` (~2.22e-16)
/// whenever none is given, silently becoming the binding term for any
/// `epsilon` below `largest_operand * f64::EPSILON`.
const TOL: f64 = 5e-13;

// `sphere_radii` used to need its own `RADIUS_TOL` here: measured (not
// assumed) at `3.469e-18` absolute / `1.436e-16` relative across the 24
// radii that differed at all -- one ulp at these magnitudes, real float
// non-associativity in the mesh-decomposition arithmetic, not a fixture
// literal. Re-bisected under `float_roundtrip` (PORTING-PLAN.md §115):
// `RADIUS_TOL = 0.0` (`max_relative` and `epsilon` both zero, effectively
// `assert_eq!`) now passes. The one ULP was `serde_json`'s fixture-parsing
// error, not a real difference between the two implementations'
// arithmetic -- with correct rounding, every radius matches bit-for-bit.
// Compares with plain `assert_eq!` now, per this crate's own convention
// (`collision_sphere_free_functions_parity.rs`'s doc: "a constant nothing
// can violate is not a gate").

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn read_fixture(file_name: &str) -> String {
    let path = fixture_path(file_name);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// `pr2_description`'s 18 collision meshes, committed under
/// `fixtures/meshes/pr2_description/` (see
/// `tools/ci/verify-fixture-provenance.sh`) -- same mapping
/// `cspace-collision`'s `collision_parity.rs` uses for panda/fanuc/pr2.
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_pr2_description",
        format!("{meshes_root}/pr2_description"),
    )])
}

fn build_pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("pr2 model must build")
}

// --- link_models_with_collision_geometry ---

#[derive(Deserialize)]
struct LmwcgRequest {
    op: String,
}

#[derive(Deserialize)]
struct LmwcgResult {
    links: Vec<String>,
}

#[derive(Deserialize)]
struct LmwcgResponseEntry {
    result: LmwcgResult,
}

/// `link_models_with_collision_geometry`'s link set, in order, against the
/// oracle's real `getLinkModelsWithCollisionGeometry()`. pr2's meshes are
/// vendored under `fixtures/meshes/pr2_description/` (see this module's own
/// doc comment), so `build_pr2_model` resolves every `<mesh>` collision
/// element and this is plain equality, no mesh-gap narrowing.
#[test]
fn link_models_with_collision_geometry_matches_the_oracle() {
    let model = build_pr2_model();

    // `linkModelsWithCollisionGeometry()` (oracle.cpp) takes no request
    // fields at all -- `op` is the only thing this request fixture could
    // ever carry -- so reading it and confirming it actually names this op
    // is what "replaying the committed request" means here; there is no
    // parameter to route into the call below.
    let requests: Vec<LmwcgRequest> = serde_json::from_str(&read_fixture(
        "link_models_with_collision_geometry_request.json",
    ))
    .expect("parse link_models_with_collision_geometry_request.json");
    assert_eq!(requests.len(), 1);
    assert_eq!(
        requests[0].op, "link_models_with_collision_geometry",
        "request fixture must actually be a link_models_with_collision_geometry op"
    );

    let responses: Vec<LmwcgResponseEntry> = serde_json::from_str(&read_fixture(
        "link_models_with_collision_geometry_response.json",
    ))
    .expect("parse link_models_with_collision_geometry_response.json");
    assert_eq!(responses.len(), 1);
    let expected_links = &responses[0].result.links;

    let padding = LinkPaddingScale::new();
    let (_decompositions, index_map) = add_link_body_decompositions(&model, 0.05, &padding, None)
        .expect("add_link_body_decompositions");

    // Order matters: this must match `RobotModel::link_models()` order
    // filtered to `!shapes().is_empty()`, the same order upstream's real
    // `getLinkModelsWithCollisionGeometry()` reports (see that method's own
    // construction-time filter, quoted in this crate's module doc).
    let actual_links: Vec<String> = model
        .link_models()
        .iter()
        .filter(|link| !link.shapes().is_empty())
        .map(|link| link.name().to_string())
        .collect();

    assert!(
        model.diagnostics().is_empty(),
        "pr2's meshes are all vendored under fixtures/meshes/pr2_description/; \
         a non-empty diagnostics list means a mesh failed to resolve, which \
         would silently narrow the comparison below rather than fail loudly"
    );

    assert_eq!(
        actual_links, *expected_links,
        "link set/order mismatch against getLinkModelsWithCollisionGeometry()"
    );

    assert_eq!(
        index_map.len(),
        actual_links.len(),
        "add_link_body_decompositions' index map size mismatch"
    );
    for name in &actual_links {
        assert!(
            index_map.contains_key(name),
            "add_link_body_decompositions has no entry for {name}"
        );
    }
}

// --- add_link_body_decompositions vs. the existing link_body_decomposition fixture ---

#[derive(Deserialize)]
struct LbdRequest {
    link: String,
    resolution: f64,
    padding: f64,
}

#[derive(Deserialize)]
struct LbdSphereDump {
    radius: f64,
}

#[derive(Deserialize)]
struct LbdResult {
    collision_spheres: Vec<LbdSphereDump>,
}

#[derive(Deserialize)]
struct LbdResponseEntry {
    result: LbdResult,
}

#[test]
fn add_link_body_decompositions_matches_the_per_link_oracle_fixture() {
    let model = build_pr2_model();

    let requests: Vec<LbdRequest> =
        serde_json::from_str(&read_fixture("link_body_decomposition_request.json"))
            .expect("parse link_body_decomposition_request.json");
    let responses: Vec<LbdResponseEntry> =
        serde_json::from_str(&read_fixture("link_body_decomposition_response.json"))
            .expect("parse link_body_decomposition_response.json");
    let request = &requests[0];
    let response = &responses[0];

    // `add_link_body_decompositions` looks up padding through
    // `LinkPaddingScale`, whose untracked-link default is `0.0` -- confirm
    // that is genuinely what the reused fixture was generated with, so this
    // test does not silently stop meaning what its doc comment claims.
    assert_eq!(
        request.padding, 0.0,
        "link_body_decomposition fixture must use padding 0.0 to double as \
         ground truth for add_link_body_decompositions' untracked-link default"
    );

    let padding = LinkPaddingScale::new();
    let (decompositions, index_map) =
        add_link_body_decompositions(&model, request.resolution, &padding, None)
            .expect("add_link_body_decompositions");

    let index = *index_map
        .get(&request.link)
        .unwrap_or_else(|| panic!("no decomposition for {}", request.link));
    let decomposition = &decompositions[index];

    assert_eq!(
        decomposition.collision_spheres().len(),
        response.result.collision_spheres.len(),
        "unposed collision sphere count mismatch"
    );
    for (actual, expected) in decomposition
        .sphere_radii()
        .iter()
        .zip(&response.result.collision_spheres)
    {
        assert_relative_eq!(*actual, expected.radius, epsilon = TOL, max_relative = TOL);
    }
}

// --- distance_field_cache_entry ---

fn build_pr2_srdf() -> SrdfModel {
    SrdfModel::parse_file(fixture_path("pr2.srdf")).expect("pr2.srdf must parse")
}

#[derive(Deserialize)]
struct DfceRequest {
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    distance_queries: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct DfceResult {
    group_name: String,
    link_names: Vec<String>,
    link_has_geometry: Vec<bool>,
    link_body_indices: Vec<usize>,
    link_state_indices: Vec<usize>,
    self_collision_enabled: Vec<bool>,
    intra_group_collision_enabled: Vec<Vec<bool>>,
    attached_body_names: Vec<String>,
    attached_body_link_state_indices: Vec<usize>,
    state_check_indices: Vec<usize>,
    state_values: Vec<f64>,
    has_field: bool,
    distance_queries: Vec<f64>,
}

#[derive(Deserialize)]
struct DfceResponseEntry {
    result: DfceResult,
}

/// The oracle's `distance_field_cache_entry` op drives
/// `CollisionEnvDistanceField env(model_)` with every constructor argument
/// defaulted; this reproduces those defaults exactly (`size_x = size_y =
/// 3.0`, `size_z = 4.0`, `origin = (0, 0, 0)`, `resolution = .02`,
/// `max_propogation_distance = .25`, `use_signed_distance_field = false` --
/// `collision_env_distance_field.hpp:49-55`) so the distance field this test
/// builds is cell-for-cell the same field the oracle queried, not merely a
/// field with the same obstacle points at a different resolution.
fn oracle_default_distance_field_config() -> DistanceFieldConfig {
    let size = Vector3::new(3.0, 3.0, 4.0);
    let origin_center = Vector3::new(0.0, 0.0, 0.0);
    let resolution = 0.02;
    DistanceFieldConfig {
        // Upstream shifts its own center-origin members to a corner inline
        // at the `PropagationDistanceField` construction site this function
        // ports; see `DistanceFieldConfig::geometry`'s own doc comment.
        geometry: GridGeometry::new(size, origin_center - 0.5 * size, resolution)
            .expect("grid geometry"),
        max_propagation_distance: 0.25,
        use_signed_distance_field: false,
    }
}

/// [`RobotState::set_to_default_values`] is called before the fixture's
/// `joint_values` are applied, mirroring the oracle's `applyJointValues`,
/// which runs `setToDefaultValues()` and then overlays. It is not optional
/// on pr2: `torso_lift_joint`, `l/r_elbow_flex_joint` and
/// `l/r_wrist_flex_joint` carry a `<safety_controller>` whose soft bounds
/// exclude zero, so upstream defaults them to the soft-bound midpoint
/// (`torso_lift_joint`: `(0.0115, 0.325)` -> `0.16825`), and
/// [`RobotState::new`] alone leaves every variable at raw `0.0`. This port
/// computes the same five defaults --
/// `cspace_core::model`'s URDF joint-bounds builder prefers `<safety_controller>`
/// soft limits and narrows them by `<limit>` only where the hard bound is
/// tighter, exactly as `robot_model.cpp`'s `jointBoundsFromURDF` does -- so
/// the two sides agree here and nothing needs pinning to make them.
#[test]
fn generate_distance_field_cache_entry_matches_the_oracle() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<DfceRequest> =
        serde_json::from_str(&read_fixture("distance_field_cache_entry_request.json"))
            .expect("parse distance_field_cache_entry_request.json");
    let responses: Vec<DfceResponseEntry> =
        serde_json::from_str(&read_fixture("distance_field_cache_entry_response.json"))
            .expect("parse distance_field_cache_entry_response.json");
    assert_eq!(requests.len(), responses.len());
    assert!(!requests.is_empty(), "fixture must carry at least one case");

    // pr2's meshes are all vendored under fixtures/meshes/pr2_description/
    // (see this module's own doc comment); a non-empty diagnostics list
    // means a mesh failed to resolve, which would silently narrow the
    // comparisons below rather than fail loudly.
    assert!(model.diagnostics().is_empty());

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values for {}: {e}", request.group));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);

        let entry = generate_distance_field_cache_entry(
            &request.group,
            &posed,
            acm_arg,
            &link_body_decompositions,
            Some(oracle_default_distance_field_config()),
            &[],
        )
        .unwrap_or_else(|e| {
            panic!(
                "generate_distance_field_cache_entry({}): {e}",
                request.group
            )
        });

        assert_eq!(entry.group_name, expected.group_name, "group_name");
        assert_eq!(entry.link_names, expected.link_names, "link_names");
        assert_eq!(
            entry.link_state_indices, expected.link_state_indices,
            "link_state_indices"
        );
        assert_eq!(
            entry.attached_body_names, expected.attached_body_names,
            "attached_body_names"
        );
        assert_eq!(
            entry.attached_body_link_state_indices, expected.attached_body_link_state_indices,
            "attached_body_link_state_indices"
        );
        assert_eq!(
            entry.state_check_indices, expected.state_check_indices,
            "state_check_indices"
        );
        assert_eq!(
            entry.state_values.len(),
            expected.state_values.len(),
            "state_values length"
        );

        for (i, link_name) in entry.link_names.iter().enumerate() {
            assert_eq!(
                entry.link_has_geometry[i], expected.link_has_geometry[i],
                "link_has_geometry[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.link_body_indices[i], expected.link_body_indices[i],
                "link_body_indices[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.self_collision_enabled[i], expected.self_collision_enabled[i],
                "self_collision_enabled[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.intra_group_collision_enabled[i], expected.intra_group_collision_enabled[i],
                "intra_group_collision_enabled[{i}] ({link_name}, group {})",
                request.group
            );
        }

        assert_eq!(
            entry.distance_field.is_some(),
            expected.has_field,
            "has_field"
        );
        let field = entry
            .distance_field
            .as_ref()
            .expect("every fixture case requests generate_distance_field");
        assert_eq!(
            request.distance_queries.len(),
            expected.distance_queries.len(),
            "fixture's own request/response distance_queries length mismatch"
        );
        for (point, expected_distance) in request
            .distance_queries
            .iter()
            .zip(&expected.distance_queries)
        {
            let actual_distance = field.distance(point[0], point[1], point[2]);
            assert_relative_eq!(
                actual_distance,
                *expected_distance,
                epsilon = TOL,
                max_relative = TOL
            );
        }
    }
}

/// Round 21 asks whether the `distance_field_cache_entry` fixture above is
/// evidence for anything on the [`DistanceFieldCollisionCache::check_self_collision`]
/// code path this round ports, or only for [`generate_distance_field_cache_entry`]
/// called directly. It is: the oracle's `distance_field_cache_entry` op is
/// itself driven by `CollisionEnvDistanceField::checkSelfCollision(req, res,
/// state)` followed by `getLastDistanceFieldEntry()` (see this test module's
/// own doc comment), and `check_self_collision` is this port's entry point
/// for exactly that upstream call -- `generate_distance_field_cache_entry`
/// is a step *inside* it, reached through
/// [`DistanceFieldCollisionCache::generate_collision_checking_structures`],
/// not an alternate path the oracle happens to resemble. This test drives
/// the same three request/response fixture cases through
/// `check_self_collision` instead of calling `generate_distance_field_cache_entry`
/// directly, and checks the resulting [`cspace_distance_field::GroupStateRepresentation::dfce`]
/// against the same expected fields -- proving the fixture is evidence for
/// the newly ported method, not merely for the free function it happens to
/// share a name with.
///
/// Every fixture case has `has_field: true` (checked below), matching
/// `check_self_collision`'s unconditional `generate_distance_field = true`
/// (see that method's own doc comment) -- so this fixture cannot exercise
/// the `generate_distance_field = false` path `check_robot_collision`'s
/// no-acm overload uses; that path has no oracle fixture and is covered
/// only by this crate's own unit tests.
#[test]
fn check_self_collision_reuses_the_distance_field_cache_entry_fixture() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<DfceRequest> =
        serde_json::from_str(&read_fixture("distance_field_cache_entry_request.json"))
            .expect("parse distance_field_cache_entry_request.json");
    let responses: Vec<DfceResponseEntry> =
        serde_json::from_str(&read_fixture("distance_field_cache_entry_response.json"))
            .expect("parse distance_field_cache_entry_response.json");
    assert_eq!(requests.len(), responses.len());
    assert!(!requests.is_empty(), "fixture must carry at least one case");
    assert!(model.diagnostics().is_empty());

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;
        assert!(
            expected.has_field,
            "check_self_collision always requests a distance field; a \
             fixture case without one would not be evidence for this path"
        );

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values for {}: {e}", request.group));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);

        // A fresh cache per case: `check_self_collision` caches its
        // `DistanceFieldCacheEntry` across calls, and this test's point is
        // "does one `check_self_collision` call reproduce the oracle
        // entry", not "does the cache-reuse path also reproduce it" (that
        // is `generate_collision_checking_structures_agrees_with_a_fresh_generate_and_represent_call`'s
        // job, in `collision_env_distance_field.rs`).
        let mut cache = DistanceFieldCollisionCache::new(
            link_body_decompositions.clone(),
            oracle_default_distance_field_config(),
            0.0,
        );
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            ..CollisionRequest::default()
        };
        let (_res, gsr) = cache
            .check_self_collision(&req, &posed, acm_arg, &[])
            .unwrap_or_else(|e| panic!("check_self_collision({}): {e}", request.group));
        let entry = gsr.dfce;

        assert_eq!(entry.group_name, expected.group_name, "group_name");
        assert_eq!(entry.link_names, expected.link_names, "link_names");
        assert_eq!(
            entry.link_state_indices, expected.link_state_indices,
            "link_state_indices"
        );
        assert_eq!(
            entry.attached_body_names, expected.attached_body_names,
            "attached_body_names"
        );
        assert_eq!(
            entry.attached_body_link_state_indices, expected.attached_body_link_state_indices,
            "attached_body_link_state_indices"
        );
        assert_eq!(
            entry.state_check_indices, expected.state_check_indices,
            "state_check_indices"
        );
        assert_eq!(
            entry.state_values.len(),
            expected.state_values.len(),
            "state_values length"
        );

        for (i, link_name) in entry.link_names.iter().enumerate() {
            assert_eq!(
                entry.link_has_geometry[i], expected.link_has_geometry[i],
                "link_has_geometry[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.link_body_indices[i], expected.link_body_indices[i],
                "link_body_indices[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.self_collision_enabled[i], expected.self_collision_enabled[i],
                "self_collision_enabled[{i}] ({link_name}, group {})",
                request.group
            );
            assert_eq!(
                entry.intra_group_collision_enabled[i], expected.intra_group_collision_enabled[i],
                "intra_group_collision_enabled[{i}] ({link_name}, group {})",
                request.group
            );
        }

        assert_eq!(
            entry.distance_field.is_some(),
            expected.has_field,
            "has_field"
        );
        let field = entry
            .distance_field
            .as_ref()
            .expect("every fixture case requests generate_distance_field");
        for (point, expected_distance) in request
            .distance_queries
            .iter()
            .zip(&expected.distance_queries)
        {
            let actual_distance = field.distance(point[0], point[1], point[2]);
            assert_relative_eq!(
                actual_distance,
                *expected_distance,
                epsilon = TOL,
                max_relative = TOL
            );
        }
    }
}

// --- group_state_representation ---

#[derive(Deserialize)]
struct GsrRequest {
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
}

#[derive(Deserialize)]
struct GsrGradient {
    closest_distance: f64,
    collision: bool,
    types: Vec<i32>,
    distances: Vec<f64>,
    sphere_radii: Vec<f64>,
    joint_name: String,
    sphere_locations_count: usize,
}

#[derive(Deserialize)]
struct GsrLink {
    link_name: String,
    has_link_decomposition: bool,
    #[serde(default)]
    bounding_sphere_center: Vec<f64>,
    #[serde(default)]
    bounding_sphere_radius: f64,
    #[serde(default)]
    collision_points_count: usize,
    #[serde(default)]
    field_pose: Vec<f64>,
    gradient: Option<GsrGradient>,
}

#[derive(Deserialize)]
struct GsrResult {
    links: Vec<GsrLink>,
}

#[derive(Deserialize)]
struct GsrResponseEntry {
    result: GsrResult,
}

/// See `oracle.cpp`'s own doc comment on its `groupStateRepresentation` op
/// (`tools/moveit-oracle/src/oracle.cpp`) for the full explanation this test
/// relies on: that op does not isolate `getGroupStateRepresentation`'s
/// *fresh* branch the way [`group_state_representation`] ports it --
/// `CollisionEnvDistanceField(model_)`'s constructor eagerly pre-builds a
/// `GroupStateRepresentation` per group at construction time, so every
/// query the oracle answers actually takes upstream's **pregenerated**
/// reuse branch instead. `sphere_locations_count` used to be excluded here
/// for exactly that reason -- the pregenerated branch fills
/// `sphere_locations` for links, the fresh branch this port implements
/// previously never did -- until PORTING-PLAN.md §154 (round 25) measured
/// that gap to be this port's own, not upstream's: `sphere_locations` is
/// value-identical between the two branches (see
/// [`group_state_representation`]'s own doc comment), so this port now sets
/// it too, and this test compares its length against the oracle's count
/// below rather than skipping it. One field remains precondition-checked
/// rather than compared outright:
///
/// - `closest_distance`/`collision`/`types`/`distances` are only meaningful
///   to compare when the oracle's own `checkCollision` pipeline found
///   nothing for that link (`collision: false`): construction alone (this
///   port's scope) can never set them to anything but their fresh defaults,
///   while the oracle's full pipeline can mutate them in place after
///   construction. This test asserts `collision: false` as an explicit
///   precondition per link before trusting the comparison, rather than
///   assume every fixture case happens to avoid it silently.
#[test]
fn group_state_representation_matches_the_oracle() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");
    let (link_body_decomposition_vector, _) = &link_body_decompositions;

    let requests: Vec<GsrRequest> =
        serde_json::from_str(&read_fixture("group_state_representation_request.json"))
            .expect("parse group_state_representation_request.json");
    let responses: Vec<GsrResponseEntry> =
        serde_json::from_str(&read_fixture("group_state_representation_response.json"))
            .expect("parse group_state_representation_response.json");
    assert_eq!(requests.len(), responses.len());
    assert!(!requests.is_empty(), "fixture must carry at least one case");

    // pr2's meshes are all vendored under fixtures/meshes/pr2_description/
    // (see this module's own doc comment); a non-empty diagnostics list
    // means a mesh failed to resolve, which would silently narrow the
    // comparisons below rather than fail loudly.
    assert!(model.diagnostics().is_empty());

    for (request, response) in requests.iter().zip(&responses) {
        let expected_links = &response.result.links;

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values for {}: {e}", request.group));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);
        let dfce = generate_distance_field_cache_entry(
            &request.group,
            &posed,
            acm_arg,
            &link_body_decompositions,
            None,
            &[],
        )
        .unwrap_or_else(|e| {
            panic!(
                "generate_distance_field_cache_entry({}): {e}",
                request.group
            )
        });

        let gsr = group_state_representation(
            &dfce,
            &posed,
            link_body_decomposition_vector,
            0.02,
            0.25,
            false,
            &[],
        )
        .unwrap_or_else(|e| panic!("group_state_representation({}): {e}", request.group));

        assert_eq!(
            gsr.link_body_decompositions.len(),
            expected_links.len(),
            "link count (group {})",
            request.group
        );

        for (i, expected_link) in expected_links.iter().enumerate() {
            let actual_has_geometry = dfce.link_has_geometry[i];

            assert_eq!(
                actual_has_geometry, expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] ({}, group {})",
                expected_link.link_name, request.group
            );
            if !actual_has_geometry {
                // ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch`
                // -- same one cause as `collision_env_distance_field.rs`'s
                // `update_group_state_representation_state_skips_links_without_geometry`:
                // `group_state_representation` has exactly one
                // `link_body_decompositions.push(None)` site, gated by this
                // same `!link_has_geometry[i]` condition.
                assert!(gsr.link_body_decompositions[i].is_none());
                continue;
            }

            let link_bd = gsr.link_body_decompositions[i]
                .as_ref()
                .expect("has_link_decomposition implies Some");
            let field = gsr.link_distance_fields[i]
                .as_ref()
                .expect("has_link_decomposition implies Some");

            let center = link_bd.bounding_sphere_center();
            assert_relative_eq!(
                center.x,
                expected_link.bounding_sphere_center[0],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                center.y,
                expected_link.bounding_sphere_center[1],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                center.z,
                expected_link.bounding_sphere_center[2],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                link_bd.bounding_sphere_radius(),
                expected_link.bounding_sphere_radius,
                epsilon = TOL,
                max_relative = TOL
            );
            assert_eq!(
                link_bd.collision_points().len(),
                expected_link.collision_points_count,
                "collision_points_count ({}, group {})",
                expected_link.link_name,
                request.group
            );

            let pose = field.pose();
            assert_relative_eq!(
                pose.translation.x,
                expected_link.field_pose[0],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                pose.translation.y,
                expected_link.field_pose[1],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                pose.translation.z,
                expected_link.field_pose[2],
                epsilon = TOL,
                max_relative = TOL
            );
            // `q` and `-q` represent the identical rotation (the unit
            // quaternion double cover), and each side's FK computation is
            // free to land on either sign -- compare whichever sign of the
            // oracle's quaternion is closer, not the raw components.
            let expected_quat = [
                expected_link.field_pose[3],
                expected_link.field_pose[4],
                expected_link.field_pose[5],
                expected_link.field_pose[6],
            ];
            let actual_quat = [
                pose.rotation.w,
                pose.rotation.i,
                pose.rotation.j,
                pose.rotation.k,
            ];
            let same_sign_error: f64 = actual_quat
                .iter()
                .zip(expected_quat)
                .map(|(a, e)| (a - e).powi(2))
                .sum();
            let flipped_sign_error: f64 = actual_quat
                .iter()
                .zip(expected_quat)
                .map(|(a, e)| (a + e).powi(2))
                .sum();
            assert!(
                same_sign_error.min(flipped_sign_error) < TOL * TOL,
                "field_pose rotation ({}, group {}): expected {expected_quat:?} (up to sign), \
                 got {actual_quat:?}",
                expected_link.link_name,
                request.group
            );

            let expected_gradient = expected_link
                .gradient
                .as_ref()
                .expect("has_link_decomposition implies a gradient entry");
            assert_eq!(
                gsr.gradients[i].sphere_radii.len(),
                expected_gradient.sphere_radii.len(),
                "sphere_radii length ({}, group {})",
                expected_link.link_name,
                request.group
            );
            for (actual_radius, expected_radius) in gsr.gradients[i]
                .sphere_radii
                .iter()
                .zip(&expected_gradient.sphere_radii)
            {
                assert_eq!(*actual_radius, *expected_radius);
            }
            assert_eq!(
                gsr.gradients[i].joint_name, expected_gradient.joint_name,
                "joint_name ({}, group {})",
                expected_link.link_name, request.group
            );
            assert_eq!(
                gsr.gradients[i].sphere_locations.len(),
                expected_gradient.sphere_locations_count,
                "sphere_locations_count ({}, group {})",
                expected_link.link_name,
                request.group
            );

            // See this test's own doc comment: only comparable when the
            // oracle's post-construction collision pipeline found nothing
            // for this link.
            assert!(
                !expected_gradient.collision,
                "{} (group {}): fixture must be a no-collision case for this \
                 test's construction-only comparison to be valid -- if this \
                 now fails, either re-derive the fixture at a joint \
                 configuration with no detected collision, or extend this \
                 test to only compare closest_distance/types/distances when \
                 collision is false per-link",
                expected_link.link_name, request.group
            );
            assert_eq!(
                gsr.gradients[i].closest_distance, expected_gradient.closest_distance,
                "closest_distance ({}, group {})",
                expected_link.link_name, request.group
            );
            assert!(!gsr.gradients[i].collision);
            let expected_types: Vec<i32> =
                gsr.gradients[i].types.iter().map(|t| *t as i32).collect();
            assert_eq!(
                expected_types, expected_gradient.types,
                "types ({}, group {})",
                expected_link.link_name, request.group
            );
            assert_eq!(
                gsr.gradients[i].distances, expected_gradient.distances,
                "distances ({}, group {})",
                expected_link.link_name, request.group
            );
        }
    }
}

// --- distance_field_cache_entry / group_state_representation: attached
// bodies + contacts (round 24) ---
//
// Both ops gained `attached_bodies`/`contacts`/`max_contacts`/
// `max_contacts_per_pair` request fields in `tools/moveit-oracle/src/
// oracle.cpp` commit `5be5f72` (`applyAttachedBodies`, shared with
// `collision`/`frame_transform`; see `oracle.cpp:1491-1548`). Round 23's
// `ebd7ebc` closed a real gap in `get_self_collisions`/
// `get_intra_group_collisions`/`get_intra_group_proximity_gradients`/
// `get_environment_collisions`: each had stopped its loop bound at
// `link_names_.size()` instead of `link_names_.size() +
// attached_body_names_.size()`, so an attached body's own collision
// spheres were silently skipped. The fixtures above never exercised that
// widened bound (none carry `attached_bodies`) and never exercised
// `contacts`/`max_contacts`/`max_contacts_per_pair` either (every case
// omits them, so the oracle's default -- `contacts: false` -- applies
// throughout) -- the two fixtures below are the first oracle ground truth
// for both.

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ContactsShapeSpec {
    Sphere { radius: f64 },
}

impl ContactsShapeSpec {
    fn to_shape(&self) -> Shape {
        match self {
            Self::Sphere { radius } => Shape::Sphere(Sphere::new(*radius).unwrap()),
        }
    }
}

#[derive(Deserialize)]
struct AttachedBodySpec {
    id: String,
    link_name: String,
    shapes: Vec<ContactsShapeSpec>,
    shape_poses: Vec<[f64; 16]>,
    #[serde(default)]
    touch_links: Vec<String>,
}

/// Owned storage for one fixture case's attached bodies, so the borrowed
/// [`AttachedBodyGeometry`] handed to `check_self_collision`/
/// `check_collision` can outlive the per-case loop body that builds it.
struct OwnedAttachedBody {
    id: String,
    link_name: String,
    shapes: Vec<Arc<Shape>>,
    shape_poses: Vec<Isometry3>,
    touch_links: BTreeSet<String>,
}

impl AttachedBodySpec {
    fn to_owned_body(&self) -> OwnedAttachedBody {
        OwnedAttachedBody {
            id: self.id.clone(),
            link_name: self.link_name.clone(),
            shapes: self.shapes.iter().map(|s| Arc::new(s.to_shape())).collect(),
            shape_poses: self
                .shape_poses
                .iter()
                .map(isometry_from_row_major)
                .collect(),
            touch_links: self.touch_links.iter().cloned().collect(),
        }
    }
}

impl OwnedAttachedBody {
    fn geometry(&self) -> AttachedBodyGeometry<'_> {
        AttachedBodyGeometry {
            id: &self.id,
            link_name: &self.link_name,
            shapes: &self.shapes,
            shape_poses: &self.shape_poses,
            touch_links: &self.touch_links,
        }
    }
}

fn default_max_contacts() -> usize {
    100
}

fn default_max_contacts_per_pair() -> usize {
    1
}

fn parse_body_type(s: &str) -> BodyType {
    match s {
        "robot_link" => BodyType::RobotLink,
        "robot_attached" => BodyType::RobotAttached,
        "world_object" => BodyType::WorldObject,
        other => panic!("unknown body_type {other}"),
    }
}

/// One `contacts` array entry, `allContactsToJson`'s 7-field shape
/// (`oracle.cpp:2662-2693`). `shape_kinds_1`/`shape_kinds_2` are not
/// deserialized: [`cspace_collision::Contact`] carries no shape-kind field
/// at all (this port's `Contact` is a synthesized "collision found" record
/// derived from a sphere-vs-field query, not a real per-shape FCL contact),
/// so there is nothing on the port side to compare them against.
#[derive(Deserialize)]
struct ContactJson {
    body_name_1: String,
    body_type_1: String,
    body_name_2: String,
    body_type_2: String,
    depth: f64,
}

/// Reduces a `contacts` array to (pair -> (body types, contact count)),
/// the level this module's contact fixtures are actually comparable at:
/// every contact in a given pair carries the same two body types and
/// `depth: 0.0` (see [`ContactJson`]'s own doc and this round's report --
/// `depth` is a default member initializer, not a real penetration
/// measurement, per `collision_detection/collision_common.hpp:84`), so distinguishing
/// individual same-pair entries beyond their count would compare noise.
fn contacts_by_pair_from_json(
    contacts: &[ContactJson],
) -> BTreeMap<(String, String), (BodyType, BodyType, usize)> {
    let mut map: BTreeMap<(String, String), (BodyType, BodyType, usize)> = BTreeMap::new();
    for c in contacts {
        assert_eq!(c.depth, 0.0, "depth ({}, {})", c.body_name_1, c.body_name_2);
        let entry = map
            .entry((c.body_name_1.clone(), c.body_name_2.clone()))
            .or_insert((
                parse_body_type(&c.body_type_1),
                parse_body_type(&c.body_type_2),
                0,
            ));
        entry.2 += 1;
    }
    map
}

fn contacts_by_pair_from_result(
    contacts: &ContactData,
) -> BTreeMap<(String, String), (BodyType, BodyType, usize)> {
    contacts
        .by_pair
        .iter()
        .map(|(pair, cs)| {
            let first = cs.first().expect("by_pair never stores an empty Vec");
            assert!(
                cs.iter().all(|c| c.depth == 0.0
                    && c.body_type_1 == first.body_type_1
                    && c.body_type_2 == first.body_type_2),
                "pair {pair:?}: every Contact for one pair must share body types and depth 0.0"
            );
            (
                pair.clone(),
                (first.body_type_1, first.body_type_2, cs.len()),
            )
        })
        .collect()
}

#[derive(Deserialize)]
struct DfceContactsRequest {
    id: u64,
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    #[serde(default)]
    attached_bodies: Vec<AttachedBodySpec>,
    #[serde(default)]
    contacts: bool,
    #[serde(default = "default_max_contacts")]
    max_contacts: usize,
    #[serde(default = "default_max_contacts_per_pair")]
    max_contacts_per_pair: usize,
}

#[derive(Deserialize)]
struct DfceContactsResult {
    attached_body_names: Vec<String>,
    #[serde(default)]
    collision: Option<bool>,
    #[serde(default)]
    contacts: Option<Vec<ContactJson>>,
}

#[derive(Deserialize)]
struct DfceContactsResponseEntry {
    result: DfceContactsResult,
}

/// External, oracle-backed verification for the four functions round 23's
/// `ebd7ebc` closed a real gap in
/// (`get_self_collisions`/`get_intra_group_collisions`/
/// `get_intra_group_proximity_gradients`/`get_environment_collisions`) --
/// specifically the two of those four this op can reach,
/// `get_self_collisions`/`get_intra_group_collisions`, via
/// [`DistanceFieldCollisionCache::check_self_collision`]. Round 23's own 12
/// boundary tests only proved this port's pieces agree with each other;
/// this is the first fixture where an independent implementation (upstream
/// C++, via the oracle) checks the same attached-body-widened loop and the
/// same `contacts`/`max_contacts`/`max_contacts_per_pair` branches.
///
/// All five cases share `right_arm`, `joint_values: {}` (upstream's own
/// safety-controller-midpoint defaults --
/// [`generate_distance_field_cache_entry_matches_the_oracle`]'s own doc
/// comment explains why `set_to_default_values` alone reproduces them) and
/// `use_acm: true`. That combination is already self-colliding at 25
/// distinct link/attached pairs (measured directly against the oracle,
/// `max_contacts: 100, max_contacts_per_pair: 5`), including both the
/// `"self"` sentinel path (`get_self_collisions`) and real link-vs-link
/// intra-group pairs (`get_intra_group_collisions`) in the same response --
/// so no case needs a hand-tuned joint configuration to exercise both
/// collision paths at once.
///
/// - id 1: no attached body, `contacts: true, max_contacts: 100,
///   max_contacts_per_pair: 5` -- baseline. 25 pairs, several already at
///   the 5-contact-per-pair cap, total 100 (this id also happens to hit
///   `max_contacts: 100`, but id 4 below is the case that isolates that
///   boundary on purpose).
/// - id 2: same, plus a 0.5m sphere attached to `r_wrist_roll_link`
///   (`touch_links: [r_wrist_roll_link, r_gripper_palm_link]`). The
///   `use_acm: true` gate on `attached_body_names_` population
///   (`collision_env_distance_field.cpp:775`) is already satisfied by every
///   case in this fixture, so id 1 vs id 2 isolates attached-body-absent
///   vs -present at an otherwise identical joint configuration and request.
/// - id 3: `contacts: false`, same joint configuration as id 1. The oracle
///   omits `collision`/`contacts` from the response entirely when
///   `contacts` is false (measured directly; see this test's `None` match
///   arm below), so there is no oracle boolean to compare here -- what this
///   case checks is that the port's own `res.collision` is still `true`
///   under `contacts: false` even though no `Contact` gets recorded
///   (`get_self_collisions`'s `else` branch at
///   `collision_env_distance_field.rs:1653-1656` sets `res.collision = true`
///   before returning, same as the `if` branch, just without touching
///   `gsr.gradients` or `res.contacts`).
/// - id 4: `max_contacts: 50` -- the `contacts.count() >= req.max_contacts`
///   early-return boundary (`collision_env_distance_field.rs:1641-1643`,
///   `:1933-1935`); measured directly against the oracle: exactly 50
///   contacts back, not 49 or 51.
/// - id 5: `max_contacts_per_pair: 1` -- every pair capped at exactly one
///   contact; 32 distinct pairs at this joint configuration (more than at
///   `max_contacts_per_pair: 5`, since fewer contacts per pair are consumed
///   before the scan naturally exhausts every pair), all under the
///   `max_contacts: 100` cap -- so id 5's total (32) is driven by pair
///   count, not by hitting the cap the way id 1 and id 4 do.
#[test]
fn check_self_collision_matches_the_oracle_with_contacts_and_attached_bodies() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<DfceContactsRequest> = serde_json::from_str(&read_fixture(
        "distance_field_cache_entry_contacts_request.json",
    ))
    .expect("parse distance_field_cache_entry_contacts_request.json");
    let responses: Vec<DfceContactsResponseEntry> = serde_json::from_str(&read_fixture(
        "distance_field_cache_entry_contacts_response.json",
    ))
    .expect("parse distance_field_cache_entry_contacts_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(
        requests.len(),
        5,
        "this fixture's five ids are each a distinct boundary -- see this test's own doc comment"
    );
    assert!(model.diagnostics().is_empty());

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values (id {}): {e}", request.id));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);

        let owned_bodies: Vec<OwnedAttachedBody> = request
            .attached_bodies
            .iter()
            .map(AttachedBodySpec::to_owned_body)
            .collect();
        let attached: Vec<AttachedBodyGeometry<'_>> = owned_bodies
            .iter()
            .map(OwnedAttachedBody::geometry)
            .collect();

        let mut cache = DistanceFieldCollisionCache::new(
            link_body_decompositions.clone(),
            oracle_default_distance_field_config(),
            0.0,
        );
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            contacts: request.contacts,
            max_contacts: request.max_contacts,
            max_contacts_per_pair: request.max_contacts_per_pair,
            ..CollisionRequest::default()
        };

        let (res, gsr) = cache
            .check_self_collision(&req, &posed, acm_arg, &attached)
            .unwrap_or_else(|e| panic!("check_self_collision (id {}): {e}", request.id));

        assert_eq!(
            gsr.dfce.attached_body_names, expected.attached_body_names,
            "attached_body_names (id {})",
            request.id
        );

        match expected.collision {
            Some(expected_collision) => assert_eq!(
                res.collision, expected_collision,
                "collision (id {})",
                request.id
            ),
            None => assert!(
                res.collision,
                "id {}: contacts:false, so the oracle omits `collision` from its response -- \
                 but this fixture's id 3 shares id 1's known-colliding joint configuration, and \
                 get_self_collisions' contacts:false branch still sets res.collision before \
                 returning early, so this must still be true (see this test's own doc comment)",
                request.id
            ),
        }

        match (&expected.contacts, &res.contacts) {
            (None, None) => {}
            (None, Some(_)) => panic!(
                "id {}: oracle reported no contacts field but the port returned one",
                request.id
            ),
            (Some(_), None) => panic!(
                "id {}: oracle reported a contacts field but the port returned none",
                request.id
            ),
            (Some(expected_contacts), Some(actual_contacts)) => {
                let expected_pairs = contacts_by_pair_from_json(expected_contacts);
                let actual_pairs = contacts_by_pair_from_result(actual_contacts);
                assert_eq!(
                    actual_pairs, expected_pairs,
                    "contact pairs (id {})",
                    request.id
                );
                assert_eq!(
                    actual_contacts.count(),
                    expected_contacts.len(),
                    "total contact count (id {})",
                    request.id
                );
                for (pair, (_, _, count)) in &actual_pairs {
                    assert!(
                        *count <= request.max_contacts_per_pair,
                        "pair {pair:?} (id {}) exceeds max_contacts_per_pair={}",
                        request.id,
                        request.max_contacts_per_pair
                    );
                }
            }
        }
    }
}

#[derive(Deserialize)]
struct GsrContactsRequest {
    id: u64,
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    #[serde(default)]
    attached_bodies: Vec<AttachedBodySpec>,
    #[serde(default)]
    contacts: bool,
    #[serde(default = "default_max_contacts")]
    max_contacts: usize,
    #[serde(default = "default_max_contacts_per_pair")]
    max_contacts_per_pair: usize,
}

#[derive(Deserialize)]
struct GsrContactsGradient {
    closest_distance: f64,
    collision: bool,
    types: Vec<i32>,
    distances: Vec<f64>,
}

#[derive(Deserialize)]
struct GsrContactsLink {
    has_link_decomposition: bool,
    gradient: Option<GsrContactsGradient>,
}

#[derive(Deserialize)]
struct GsrContactsResult {
    links: Vec<GsrContactsLink>,
    #[serde(default)]
    collision: Option<bool>,
    #[serde(default)]
    contacts: Option<Vec<ContactJson>>,
}

#[derive(Deserialize)]
struct GsrContactsResponseEntry {
    result: GsrContactsResult,
}

/// `group_state_representation`'s op (`oracle.cpp:3985-4295`) drives
/// `CollisionEnvDistanceField::checkCollision`, not `checkSelfCollision` --
/// unlike [`check_self_collision_matches_the_oracle_with_contacts_and_attached_bodies`]
/// above, it also runs the environment phase ([`cspace_distance_field::get_environment_collisions`],
/// round 23's fourth closed gap), even though neither op builds a `World`
/// so that phase can never actually *find* an environment collision through
/// either op (both construct `CollisionEnvDistanceField(model_)` with the
/// single-argument, no-world constructor -- see this round's report). This
/// test drives the port's [`DistanceFieldCollisionCache::check_collision`]
/// against an empty [`PropagationDistanceField`] to match that, so it
/// verifies `get_environment_collisions` runs without corrupting the
/// self/intra-group result, not that it can find a real environment
/// collision -- that remains unverified against the oracle (see this
/// round's UNFIXED).
///
/// Unlike [`group_state_representation_matches_the_oracle`] above (which
/// drives *construction-only* `group_state_representation`, so its
/// `collision`/`types`/`distances` per-link fields never reflect a real
/// query and that test enforces `collision: false` as a precondition before
/// trusting them), this test's `gsr` comes from `check_collision`, which
/// genuinely runs `get_self_collisions`/`get_intra_group_collisions`/
/// `get_environment_collisions` -- so every link's `gradient.collision`/
/// `types`/`distances`/`closest_distance` is real collision-detection
/// output on both sides, and this test compares every link regardless of
/// whether it collided.
///
/// Three ids, the same `right_arm`/`joint_values: {}`/`use_acm: true`
/// colliding configuration as `distance_field_cache_entry_contacts` above:
///
/// - id 1: no attached body, `contacts: true` -- 13/22 links show
///   `gradient.collision: true` (measured directly).
/// - id 2: `payload` sphere attached to `r_wrist_roll_link`, `contacts:
///   true` -- same 13/22 link split, plus the attached body itself
///   participates in `result.contacts`. It never appears in `links[]`:
///   that array is link-indexed only (`oracle.cpp:4237`, `for (i = 0; i <
///   gsr->dfce_->link_names_.size(); ++i)`), so an attached body's own
///   gradient slot is unreachable from this op's per-link dump regardless
///   of `contacts` -- a distinct, narrower gap from the one this round's
///   UNFIXED reports for `get_intra_group_proximity_gradients` (that one is
///   about neither op ever calling `getCollisionGradients` at all; this one
///   is about the dump loop even if it did).
/// - id 3: `contacts: false`, same joint configuration as id 1 -- 0/22
///   links show `gradient.collision: true`, matching
///   `get_self_collisions`/`get_intra_group_collisions`'s `contacts: false`
///   branches never touching `gsr.gradients` before returning
///   (`collision_env_distance_field.rs:1653-1656`, `:1936-1938`). This is
///   the oracle ground truth for the "`contacts` is not an output-only
///   switch" fact this round's brief measured: id 1 vs id 3 is the same
///   joint configuration with only `contacts` toggled.
#[test]
fn check_collision_matches_the_oracle_with_contacts_and_attached_bodies() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<GsrContactsRequest> = serde_json::from_str(&read_fixture(
        "group_state_representation_contacts_request.json",
    ))
    .expect("parse group_state_representation_contacts_request.json");
    let responses: Vec<GsrContactsResponseEntry> = serde_json::from_str(&read_fixture(
        "group_state_representation_contacts_response.json",
    ))
    .expect("parse group_state_representation_contacts_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(
        requests.len(),
        3,
        "this fixture's three ids are each a distinct boundary -- see this test's own doc comment"
    );
    assert!(model.diagnostics().is_empty());

    let config = oracle_default_distance_field_config();
    let empty_env = PropagationDistanceField::new(
        config.geometry,
        config.max_propagation_distance,
        config.use_signed_distance_field,
    )
    .expect("empty environment field");

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values (id {}): {e}", request.id));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);

        let owned_bodies: Vec<OwnedAttachedBody> = request
            .attached_bodies
            .iter()
            .map(AttachedBodySpec::to_owned_body)
            .collect();
        let attached: Vec<AttachedBodyGeometry<'_>> = owned_bodies
            .iter()
            .map(OwnedAttachedBody::geometry)
            .collect();

        let mut cache =
            DistanceFieldCollisionCache::new(link_body_decompositions.clone(), config, 0.0);
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            contacts: request.contacts,
            max_contacts: request.max_contacts,
            max_contacts_per_pair: request.max_contacts_per_pair,
            ..CollisionRequest::default()
        };

        let (res, gsr) = cache
            .check_collision(&req, &posed, acm_arg, &attached, &empty_env)
            .unwrap_or_else(|e| panic!("check_collision (id {}): {e}", request.id));

        assert_eq!(
            gsr.dfce.link_names.len(),
            expected.links.len(),
            "link count (id {})",
            request.id
        );

        for (i, expected_link) in expected.links.iter().enumerate() {
            assert_eq!(
                gsr.dfce.link_has_geometry[i], expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] (id {})",
                request.id
            );
            if !gsr.dfce.link_has_geometry[i] {
                continue;
            }
            let expected_gradient = expected_link
                .gradient
                .as_ref()
                .expect("has_link_decomposition implies a gradient entry");
            assert_eq!(
                gsr.gradients[i].collision, expected_gradient.collision,
                "gradient.collision[{i}] (id {})",
                request.id
            );
            let actual_types: Vec<i32> = gsr.gradients[i].types.iter().map(|t| *t as i32).collect();
            assert_eq!(
                actual_types, expected_gradient.types,
                "gradient.types[{i}] (id {})",
                request.id
            );
            assert_eq!(
                gsr.gradients[i].distances, expected_gradient.distances,
                "gradient.distances[{i}] (id {})",
                request.id
            );
            assert_eq!(
                gsr.gradients[i].closest_distance, expected_gradient.closest_distance,
                "gradient.closest_distance[{i}] (id {})",
                request.id
            );
        }

        match expected.collision {
            Some(expected_collision) => assert_eq!(
                res.collision, expected_collision,
                "collision (id {})",
                request.id
            ),
            None => assert!(
                res.collision,
                "id {}: contacts:false, so the oracle omits `collision` from its response -- \
                 but this fixture's id 3 shares id 1's known-colliding joint configuration (see \
                 this test's own doc comment)",
                request.id
            ),
        }

        match (&expected.contacts, &res.contacts) {
            (None, None) => {}
            (None, Some(_)) => panic!(
                "id {}: oracle reported no contacts field but the port returned one",
                request.id
            ),
            (Some(_), None) => panic!(
                "id {}: oracle reported a contacts field but the port returned none",
                request.id
            ),
            (Some(expected_contacts), Some(actual_contacts)) => {
                let expected_pairs = contacts_by_pair_from_json(expected_contacts);
                let actual_pairs = contacts_by_pair_from_result(actual_contacts);
                assert_eq!(
                    actual_pairs, expected_pairs,
                    "contact pairs (id {})",
                    request.id
                );
                assert_eq!(
                    actual_contacts.count(),
                    expected_contacts.len(),
                    "total contact count (id {})",
                    request.id
                );
            }
        }
    }
}

// --- group_state_representation: `gradients` (round 25) ---
//
// `tools/moveit-oracle/src/oracle.cpp` gained `request["gradients"]`
// (PORTING-PLAN.md §155, commit `84f5565`): when set, `groupStateRepresentation`
// drives `CollisionEnvDistanceField::getCollisionGradients` instead of
// `checkCollision`, which is the first ground truth for the three functions
// round 23 closed with no external verification at all --
// [`get_self_proximity_gradients`]/[`get_intra_group_proximity_gradients`]/
// [`get_environment_proximity_gradients`], reached here only through
// [`DistanceFieldCollisionCache::get_collision_gradients`] (they are
// module-private otherwise). `request["objects"]` (§155, commit `bc14b80`)
// is what makes the environment branch reachable at all -- see this test's
// own doc comment for why a real, populated environment field is built here
// rather than reusing the empty one
// [`check_collision_matches_the_oracle_with_contacts_and_attached_bodies`]
// above uses.

#[derive(Deserialize)]
struct GsrGradientsObjectSpec {
    pose: [f64; 16],
    shape: ContactsShapeSpec,
}

#[derive(Deserialize)]
struct GsrGradientsRequest {
    id: u64,
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    #[serde(default)]
    objects: Vec<GsrGradientsObjectSpec>,
    #[serde(default)]
    attached_bodies: Vec<AttachedBodySpec>,
}

#[derive(Deserialize)]
struct GsrGradientsLink {
    has_link_decomposition: bool,
    gradient: Option<GsrGradient>,
}

#[derive(Deserialize)]
struct AttachedBodyGradientEntry {
    name: String,
    gradient: GsrGradient,
}

#[derive(Deserialize)]
struct GsrGradientsResult {
    links: Vec<GsrGradientsLink>,
    #[serde(default)]
    attached_body_gradients: Vec<AttachedBodyGradientEntry>,
}

#[derive(Deserialize)]
struct GsrGradientsResponseEntry {
    result: GsrGradientsResult,
}

/// External, oracle-backed verification for
/// [`get_self_proximity_gradients`]/[`get_intra_group_proximity_gradients`]/
/// [`get_environment_proximity_gradients`], all three reached through
/// [`DistanceFieldCollisionCache::get_collision_gradients`]. Four ids, each a
/// distinct boundary rather than a narrative scenario:
///
/// - id 1: `right_arm`, `use_acm: true`, no `objects`/`attached_bodies` --
///   self/intra-group only, no environment field populated. Measured type
///   histogram `{SELF: 6, INTRA: 48}` (PORTING-PLAN.md §155's own reference
///   value for this exact case).
/// - id 2: same, plus one `objects` sphere placed on `r_wrist_roll_link`'s
///   own first collision-sphere center (from
///   `group_state_representation_response.json` id 1) -- close enough to
///   guarantee an environment hit, so [`get_environment_proximity_gradients`]
///   actually reaches its `ENVIRONMENT`-type branch, not just runs over an
///   empty field. Unlike every other test in this file, the environment
///   [`PropagationDistanceField`] built here is *not* empty: it is populated
///   from `request.objects` via [`collision_object_point_decomposition`], the
///   same free function [`check_collision_matches_the_oracle_with_contacts_and_attached_bodies`]'s
///   own doc comment reports as the missing piece for exercising this branch
///   against the oracle. The resulting histogram is measured directly against
///   the oracle below, not hand-derived -- the object placement only needs to
///   land *some* sphere in collision, not reproduce any specific split.
/// - id 3: same as id 1, plus a `payload` sphere attached to
///   `r_wrist_roll_link` -- exercises `attached_body_gradients`, the output
///   array [`GroupStateRepresentation::gradients`]'s
///   `dfce.link_names.len()..` tail that the link-indexed `links[]` dump can
///   never reach (see [`check_collision_matches_the_oracle_with_contacts_and_attached_bodies`]'s
///   own doc comment on that same structural gap for the `contacts` path).
/// - id 4: same as id 1, `use_acm: false` -- the null-ACM path through
///   [`get_self_proximity_gradients`]'s own ACM check.
///
/// `gradients: true` + `contacts: true` is deliberately *not* a fixture case
/// here: `oracle.cpp`'s `groupStateRepresentation` throws for that
/// combination before producing any `result` at all (`getCollisionGradients`
/// discards its own `CollisionResult&` parameter,
/// `collision_env_distance_field.cpp:1517`), so there is no successful
/// response shape to commit -- unlike `totg_parity.rs`'s per-case `ok`/`stage`
/// pattern, this would be a *whole-request* failure, not one case among
/// several successful ones in the same array, and this op's other response
/// structs above all assume `result` is always present. Confirmed live
/// against the oracle instead (`sg docker -c 'tools/moveit-oracle/run-oracle.sh
/// --urdf .../pr2.urdf --srdf .../pr2.srdf'`, request
/// `{"op":"group_state_representation","group":"right_arm","gradients":true,"contacts":true}`):
/// `{"id":5,"ok":false,"error":"gradients and contacts are mutually exclusive:
/// getCollisionGradients discards its CollisionResult
/// (collision_env_distance_field.cpp:1517)"}`. This port has nothing
/// structurally comparable to reject: [`DistanceFieldCollisionCache::get_collision_gradients`]
/// takes a `req: &CollisionRequest` for `group_name` alone (see its own "no
/// `res` parameter" deviation doc) and never reads `req.contacts` at all, so
/// there is no shared mutable `CollisionResult` for the two modes to race
/// over the way `oracle.cpp`'s single combined op has to guard against --
/// the oracle's request-schema-level exclusivity is a fact about that one op
/// flattening two separate upstream entry points into one JSON request, not
/// a constraint this port's typed, two-separate-methods API needs to
/// reproduce. `get_collision_gradients_ignores_the_contacts_request_field`
/// below asserts that directly: a `CollisionRequest { contacts: true, .. }`
/// passed to `get_collision_gradients` succeeds normally rather than
/// erroring, which is the structural reason no error string is depended on
/// anywhere in this file.
#[test]
fn group_state_representation_gradients_matches_the_oracle() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<GsrGradientsRequest> = serde_json::from_str(&read_fixture(
        "group_state_representation_gradients_request.json",
    ))
    .expect("parse group_state_representation_gradients_request.json");
    let responses: Vec<GsrGradientsResponseEntry> = serde_json::from_str(&read_fixture(
        "group_state_representation_gradients_response.json",
    ))
    .expect("parse group_state_representation_gradients_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(
        requests.len(),
        4,
        "this fixture's four ids are each a distinct boundary -- see this test's own doc comment"
    );
    assert!(model.diagnostics().is_empty());

    let config = oracle_default_distance_field_config();

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values (id {}): {e}", request.id));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);

        let owned_bodies: Vec<OwnedAttachedBody> = request
            .attached_bodies
            .iter()
            .map(AttachedBodySpec::to_owned_body)
            .collect();
        let attached: Vec<AttachedBodyGeometry<'_>> = owned_bodies
            .iter()
            .map(OwnedAttachedBody::geometry)
            .collect();

        let mut env_field = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .expect("environment field");
        if !request.objects.is_empty() {
            let mut world = World::new();
            for object in &request.objects {
                let shape = Arc::new(object.shape.to_shape());
                world
                    .add_shape("env_sphere", shape, isometry_from_row_major(&object.pose))
                    .expect("add_shape (non-empty shapes, matching poses)");
            }
            let world_object = world.get_object("env_sphere").expect("just added above");
            let points = collision_object_point_decomposition(&world_object, 0.02)
                .unwrap_or_else(|e| {
                    panic!(
                        "collision_object_point_decomposition (id {}): {e}",
                        request.id
                    )
                })
                .collision_points();
            env_field.add_points_to_field(&points);
        }

        let mut cache =
            DistanceFieldCollisionCache::new(link_body_decompositions.clone(), config, 0.0);
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            ..CollisionRequest::default()
        };

        let gsr = cache
            .get_collision_gradients(&req, &posed, acm_arg, &attached, &env_field)
            .unwrap_or_else(|e| panic!("get_collision_gradients (id {}): {e}", request.id));

        assert_eq!(
            gsr.dfce.link_names.len(),
            expected.links.len(),
            "link count (id {})",
            request.id
        );

        for (i, expected_link) in expected.links.iter().enumerate() {
            assert_eq!(
                gsr.dfce.link_has_geometry[i], expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] (id {})",
                request.id
            );
            if !gsr.dfce.link_has_geometry[i] {
                continue;
            }
            let expected_gradient = expected_link
                .gradient
                .as_ref()
                .expect("has_link_decomposition implies a gradient entry");
            assert_gradient_matches_oracle(
                &gsr.gradients[i],
                expected_gradient,
                &format!("link[{i}] (id {})", request.id),
            );
        }

        assert_eq!(
            gsr.dfce.attached_body_names.len(),
            expected.attached_body_gradients.len(),
            "attached_body_gradients count (id {})",
            request.id
        );
        for (j, expected_attached) in expected.attached_body_gradients.iter().enumerate() {
            assert_eq!(
                gsr.dfce.attached_body_names[j], expected_attached.name,
                "attached_body_gradients[{j}].name (id {})",
                request.id
            );
            let index = gsr.dfce.link_names.len() + j;
            assert_gradient_matches_oracle(
                &gsr.gradients[index],
                &expected_attached.gradient,
                &format!("attached_body_gradients[{j}] (id {})", request.id),
            );
        }
    }
}

/// Shared by every link/attached-body slot in
/// [`group_state_representation_gradients_matches_the_oracle`]: compares
/// every [`GradientInfo`] field [`GsrGradient`] carries, `sphere_locations`
/// included (round 25 closed that gap -- see
/// [`group_state_representation`]'s own "Deviations from upstream").
///
/// `distances`/`closest_distance` use this file's own `TOL` rather than
/// `assert_eq!`: unlike the construction-only fields
/// [`group_state_representation_matches_the_oracle`] compares (fresh
/// `DBL_MAX`/`0.0` fills, bit-exact by construction), these are real
/// per-sphere distance-field query results computed independently on each
/// side, and differ by a few ULP the same way this file's other genuinely
/// computed floats do (measured directly: worst observed case here is
/// `1.6e-14` relative, comfortably inside `TOL`'s existing margin -- see
/// `TOL`'s own doc comment for how that margin was set).
fn assert_gradient_matches_oracle(
    actual: &cspace_distance_field::GradientInfo,
    expected: &GsrGradient,
    ctx: &str,
) {
    assert_eq!(actual.collision, expected.collision, "collision {ctx}");
    let actual_types: Vec<i32> = actual.types.iter().map(|t| *t as i32).collect();
    assert_eq!(actual_types, expected.types, "types {ctx}");
    assert_eq!(
        actual.distances.len(),
        expected.distances.len(),
        "distances length {ctx}"
    );
    for (a, e) in actual.distances.iter().zip(&expected.distances) {
        assert_relative_eq!(*a, *e, epsilon = TOL, max_relative = TOL);
    }
    assert_relative_eq!(
        actual.closest_distance,
        expected.closest_distance,
        epsilon = TOL,
        max_relative = TOL
    );
    assert_eq!(
        actual.sphere_radii, expected.sphere_radii,
        "sphere_radii {ctx}"
    );
    assert_eq!(actual.joint_name, expected.joint_name, "joint_name {ctx}");
    assert_eq!(
        actual.sphere_locations.len(),
        expected.sphere_locations_count,
        "sphere_locations_count {ctx}"
    );
}

/// See [`group_state_representation_gradients_matches_the_oracle`]'s own doc
/// comment for why `gradients: true` + `contacts: true`'s oracle-side
/// rejection has nothing on this port's side to reproduce: this port's
/// [`DistanceFieldCollisionCache::get_collision_gradients`] takes
/// `req: &CollisionRequest` for `group_name` alone and never reads
/// `req.contacts`, unlike `oracle.cpp`'s single op, which has to guard the
/// two modes sharing one `CollisionResult`. A request with `contacts: true`
/// set succeeds exactly like one without it.
#[test]
fn get_collision_gradients_ignores_the_contacts_request_field() {
    let model = build_pr2_model();
    assert!(model.diagnostics().is_empty());

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");
    let config = oracle_default_distance_field_config();
    let empty_env = PropagationDistanceField::new(
        config.geometry,
        config.max_propagation_distance,
        config.use_signed_distance_field,
    )
    .expect("empty environment field");

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let mut cache = DistanceFieldCollisionCache::new(link_body_decompositions, config, 0.0);
    let req = CollisionRequest {
        group_name: Some("right_arm".to_string()),
        contacts: true,
        max_contacts: 100,
        max_contacts_per_pair: 1,
        ..CollisionRequest::default()
    };

    cache
        .get_collision_gradients(&req, &posed, None, &[], &empty_env)
        .expect("contacts:true is not read by get_collision_gradients, so this must succeed");
}
