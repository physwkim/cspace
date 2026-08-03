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
//! `moveit-model` can load STL `<mesh>` geometry now (`RobotModel::from_urdf_and_srdf`
//! takes a [`MeshSearchPaths`]). pr2's 18 `<collision>` mesh files *are*
//! vendored -- all present under the gitignored
//! `third_party/moveit_resources/pr2_description/urdf/meshes/` checkout --
//! but not yet copied into this workspace's committed `fixtures/meshes/` the
//! way panda's and fanuc's are (p3-acm's task this round; see
//! `link_models_with_collision_geometry_matches_the_oracle_modulo_the_documented_mesh_deviation`'s
//! own doc comment for what was checked). `build_pr2_model` below still
//! builds with [`MeshSearchPaths::none`] accordingly, since the committed
//! fixture path does not exist yet.
//!
//! This is a missing *fixture copy*, not an open correctness question: this
//! round, pointing `build_pr2_model` at `third_party/` directly (a probe,
//! not committed -- that path is gitignored, not guaranteed present in CI)
//! reproduced every mesh, and every assertion this file narrows --
//! `link_models_with_collision_geometry`'s link set, `link_has_geometry`,
//! `self_collision_enabled`, `intra_group_collision_enabled`, and the
//! distance field's own `distance_queries` -- came back byte-exact against
//! the oracle for all three `generate_distance_field_cache_entry` cases and
//! the `link_models_with_collision_geometry` case, no discrepancies. Until
//! the `fixtures/meshes/pr2_description/` copy lands, `build_pr2_model`
//! cannot point there, so `generate_distance_field_cache_entry_matches_the_oracle`
//! still hits `link_has_geometry` and every field derived from it
//! (`link_body_indices`, `self_collision_enabled`,
//! `intra_group_collision_enabled`, and the distance field's own obstacle
//! set) on essentially every case; each of those assertions is narrowed
//! per-link accordingly, see the loop body's own comments for the exact
//! rule per field. Once the fixture copy lands, swap `MeshSearchPaths::none()`
//! for the real map and tighten every narrowed assertion here to plain
//! equality -- this round's probe is the evidence that tightening will pass.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use nalgebra::Vector3;
use serde::Deserialize;

use moveit_collision::{AllowedCollisionMatrix, LinkPaddingScale};
use moveit_distance_field::{
    DistanceField, DistanceFieldConfig, GridGeometry, add_link_body_decompositions,
    generate_distance_field_cache_entry, group_state_representation,
};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

const TOL: f64 = 1e-4;

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

fn build_pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("pr2 model must build")
}

// --- link_models_with_collision_geometry ---

#[derive(Deserialize)]
struct LmwcgResult {
    links: Vec<String>,
}

#[derive(Deserialize)]
struct LmwcgResponseEntry {
    result: LmwcgResult,
}

/// `link_models_with_collision_geometry` cannot expect byte-exact parity
/// with the oracle here: the oracle links against real mesh files and so
/// counts mesh-only-collision links (`pr2.urdf`'s `base_link`, the caster
/// rotation links, `torso_lift_link`, every arm link, ...) as having
/// collision geometry, while `build_pr2_model` below builds with
/// [`MeshSearchPaths::none`] -- **not** because this port's `RobotModel`
/// cannot load `<mesh>` geometry (it can, given an explicit
/// [`MeshSearchPaths`]; see `moveit-model`'s `LinkModel` doc comment,
/// deviation 4, and its own `mesh_collision_is_skipped_with_a_diagnostic_and_leaves_no_shape`
/// test), and **not** because pr2's meshes are missing -- all 18 of
/// `pr2.urdf`'s `<collision>` mesh files are vendored under the gitignored
/// `third_party/moveit_resources/pr2_description/urdf/meshes/` (checked
/// directly: every reference resolves to a file that exists there) -- but
/// because they are not yet copied into this workspace's committed
/// `fixtures/meshes/` the way panda's and fanuc's are (checked directly:
/// `fixtures/meshes/` holds only `panda_description/` and
/// `fanuc_description/`; `moveit-collision`'s `collision_parity.rs` documents
/// the identical gap for its own pr2 case; p3-acm owns landing that copy).
/// Every unresolved `<mesh>` is recorded as a `Diagnostic::UnsupportedLinkGeometry`.
///
/// So the real property to check here, until that fixture copy lands, is
/// not set-equality but: our computed set is *exactly* the oracle's set
/// minus the links whose divergence is explained by a recorded mesh
/// diagnostic -- i.e. every disagreement is accounted for, none is silent.
/// Pointing `build_pr2_model` at `third_party/` directly this round (a
/// probe, not committed) confirmed the stronger property already holds:
/// with real meshes loaded, `actual_links` matches `expected_links`
/// byte-for-byte, no diagnostics, no narrowing needed.
#[test]
fn link_models_with_collision_geometry_matches_the_oracle_modulo_the_documented_mesh_deviation() {
    let model = build_pr2_model();

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

    let unsupported_mesh_links: std::collections::HashSet<&str> = model
        .diagnostics()
        .iter()
        .filter_map(|d| match d {
            moveit_model::Diagnostic::UnsupportedLinkGeometry {
                link, kind: "mesh", ..
            } => Some(link.as_str()),
            _ => None,
        })
        .collect();
    assert!(
        !unsupported_mesh_links.is_empty(),
        "pr2.urdf is expected to exercise the mesh-skip path; if this is now \
         empty, either the fixture changed or moveit-model started loading \
         meshes -- re-derive this test rather than deleting the assertion"
    );

    let expected_links_our_model_can_represent: Vec<String> = expected_links
        .iter()
        .filter(|name| !unsupported_mesh_links.contains(name.as_str()))
        .cloned()
        .collect();

    assert_eq!(
        actual_links, expected_links_our_model_can_represent,
        "link set/order mismatch against getLinkModelsWithCollisionGeometry(), \
         after excluding links whose only collision geometry is an \
         (intentionally unsupported) mesh"
    );

    // Every link the oracle reports but we don't must be explained by a
    // recorded diagnostic -- not merely "absent from our shapes() filter"
    // for some other, unrecorded reason.
    for name in expected_links {
        if !actual_links.contains(name) {
            assert!(
                unsupported_mesh_links.contains(name.as_str()),
                "{name} is in the oracle's collision-geometry list but missing \
                 from ours with no UnsupportedLinkGeometry diagnostic to explain it"
            );
        }
    }

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
        assert_relative_eq!(*actual, expected.radius, epsilon = TOL);
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
    // `link_body_indices` deliberately not deserialized: not oracle-comparable
    // under the mesh gap, see the test's own comment at its use site.
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
/// `moveit_model`'s URDF joint-bounds builder prefers `<safety_controller>`
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

    // Same mesh-collision gap as `link_models_with_collision_geometry_matches_the_oracle_modulo_the_documented_mesh_deviation`,
    // reused here because it cascades into every geometry-derived field this
    // test checks -- see the loop body below for exactly how each field's
    // assertion is narrowed per mesh-affected link.
    let unsupported_mesh_links: std::collections::HashSet<&str> = model
        .diagnostics()
        .iter()
        .filter_map(|d| match d {
            moveit_model::Diagnostic::UnsupportedLinkGeometry {
                link, kind: "mesh", ..
            } => Some(link.as_str()),
            _ => None,
        })
        .collect();
    assert!(
        !unsupported_mesh_links.is_empty(),
        "pr2.urdf is expected to exercise the mesh-skip path; if this is now \
         empty, either the fixture changed or moveit-model started loading \
         meshes -- re-derive this test rather than deleting the assertion"
    );
    let (_, link_body_decomposition_index_map) = &link_body_decompositions;

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

        // `link_has_geometry` and everything it feeds (`link_body_indices`,
        // `self_collision_enabled`, `intra_group_collision_enabled`) inherit
        // the mesh-collision gap: for a mesh-affected link this port always
        // computes `link_has_geometry = false` (`add_link_body_decompositions`
        // never built it a `BodyDecomposition` either), while the oracle,
        // linked against the real mesh files, computes `true`. That single
        // divergence is asserted explicitly per link below; every downstream
        // field is then checked either against the oracle (where the link's
        // geometry status agrees) or for internal self-consistency with this
        // port's own `link_has_geometry` value (where it does not) -- never
        // against an oracle value this port has no way to reproduce.
        for (i, link_name) in entry.link_names.iter().enumerate() {
            let mesh_affected = unsupported_mesh_links.contains(link_name.as_str());
            if mesh_affected {
                assert!(
                    !entry.link_has_geometry[i] && expected.link_has_geometry[i],
                    "{link_name} (group {}): expected the documented mesh-gap divergence \
                     (ours=false, oracle=true), got ours={}, oracle={}",
                    request.group,
                    entry.link_has_geometry[i],
                    expected.link_has_geometry[i]
                );
            } else {
                assert_eq!(
                    entry.link_has_geometry[i], expected.link_has_geometry[i],
                    "link_has_geometry[{i}] ({link_name}, group {})",
                    request.group
                );
            }

            // `link_body_indices`: not oracle-comparable at all under the
            // mesh gap, even for a non-mesh-affected link -- both ports
            // assign indices sequentially over their own filtered
            // collision-geometry link set, and that set's length differs
            // (ours omits every mesh-only link), so the same link can land
            // at different sequential positions in each. Check wiring
            // against this port's own index map instead.
            let expected_body_index = if entry.link_has_geometry[i] {
                link_body_decomposition_index_map[link_name]
            } else {
                0
            };
            assert_eq!(
                entry.link_body_indices[i], expected_body_index,
                "link_body_indices[{i}] ({link_name}, group {}) disagrees with this port's own index map",
                request.group
            );

            if mesh_affected {
                assert!(
                    !entry.self_collision_enabled[i],
                    "self_collision_enabled[{i}] ({link_name}, group {}): mesh-affected link must \
                     take this port's no-geometry branch (false)",
                    request.group
                );
                assert!(
                    entry.intra_group_collision_enabled[i].iter().all(|&b| !b),
                    "intra_group_collision_enabled[{i}] ({link_name}, group {}): mesh-affected \
                     link's row must be all-false, matching this port's no-geometry branch",
                    request.group
                );
            } else {
                assert_eq!(
                    entry.self_collision_enabled[i], expected.self_collision_enabled[i],
                    "self_collision_enabled[{i}] ({link_name}, group {})",
                    request.group
                );
                assert_eq!(
                    entry.intra_group_collision_enabled[i],
                    expected.intra_group_collision_enabled[i],
                    "intra_group_collision_enabled[{i}] ({link_name}, group {})",
                    request.group
                );
            }
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
        // Distance, not equality: this port's field is seeded from a strict
        // subset of the oracle's obstacle points (every mesh-only link's
        // points are simply never generated, see above), and removing
        // obstacle points from a propagation distance transform can only
        // raise or preserve the distance reported at any query point, never
        // lower it, up to the same `max_propagation_distance` cap on both
        // sides. So `actual >= expected` is the real, checkable property; a
        // tighter equality would only hold by coincidence.
        for (point, expected_distance) in request
            .distance_queries
            .iter()
            .zip(&expected.distance_queries)
        {
            let actual_distance = field.distance(point[0], point[1], point[2]);
            assert!(
                actual_distance >= *expected_distance - TOL,
                "distance at {point:?} (group {}): expected >= {expected_distance} \
                 (subset-obstacle-set property), got {actual_distance}",
                request.group
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
    // `sphere_locations_count` deliberately not deserialized: not
    // oracle-comparable at all, see this test's own doc comment on
    // `group_state_representation_matches_the_oracle`.
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
/// reuse branch instead. Two fields are consequently excluded or
/// precondition-checked rather than compared outright:
///
/// - `sphere_locations_count` is not deserialized at all: the pregenerated
///   branch always populates `sphere_locations`, the fresh branch this port
///   implements never does (see [`group_state_representation`]'s own doc
///   comment) -- there is no value on this port's side to compare it
///   against.
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

    let unsupported_mesh_links: std::collections::HashSet<&str> = model
        .diagnostics()
        .iter()
        .filter_map(|d| match d {
            moveit_model::Diagnostic::UnsupportedLinkGeometry {
                link, kind: "mesh", ..
            } => Some(link.as_str()),
            _ => None,
        })
        .collect();

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
        )
        .unwrap_or_else(|e| panic!("group_state_representation({}): {e}", request.group));

        assert_eq!(
            gsr.link_body_decompositions.len(),
            expected_links.len(),
            "link count (group {})",
            request.group
        );

        for (i, expected_link) in expected_links.iter().enumerate() {
            let mesh_affected = unsupported_mesh_links.contains(expected_link.link_name.as_str());
            let actual_has_geometry = dfce.link_has_geometry[i];

            if mesh_affected {
                assert!(
                    !actual_has_geometry && expected_link.has_link_decomposition,
                    "{} (group {}): expected the documented mesh-gap divergence \
                     (ours=false, oracle=true)",
                    expected_link.link_name,
                    request.group
                );
                assert!(gsr.link_body_decompositions[i].is_none());
                assert!(gsr.link_distance_fields[i].is_none());
                continue;
            }

            assert_eq!(
                actual_has_geometry, expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] ({}, group {})",
                expected_link.link_name, request.group
            );
            if !actual_has_geometry {
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
                epsilon = TOL
            );
            assert_relative_eq!(
                center.y,
                expected_link.bounding_sphere_center[1],
                epsilon = TOL
            );
            assert_relative_eq!(
                center.z,
                expected_link.bounding_sphere_center[2],
                epsilon = TOL
            );
            assert_relative_eq!(
                link_bd.bounding_sphere_radius(),
                expected_link.bounding_sphere_radius,
                epsilon = TOL
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
                epsilon = TOL
            );
            assert_relative_eq!(
                pose.translation.y,
                expected_link.field_pose[1],
                epsilon = TOL
            );
            assert_relative_eq!(
                pose.translation.z,
                expected_link.field_pose[2],
                epsilon = TOL
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
                gsr.gradients[i].sphere_radii, expected_gradient.sphere_radii,
                "sphere_radii ({}, group {})",
                expected_link.link_name, request.group
            );
            assert_eq!(
                gsr.gradients[i].joint_name, expected_gradient.joint_name,
                "joint_name ({}, group {})",
                expected_link.link_name, request.group
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
