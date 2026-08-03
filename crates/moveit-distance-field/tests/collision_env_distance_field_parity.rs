// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity tests against the moveit2 C++ oracle for
//! `collision_env_distance_field.rs`'s `add_link_body_decompositions`.
//!
//! Two properties, two ops:
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
//! `test_collision_distance_field.cpp` gives no ground truth here either --
//! see `collision_env_distance_field.rs`'s own module doc for why (every
//! `TEST_F` case calls `checkSelfCollision`/`checkRobotCollision`, none
//! reach `addLinkBodyDecompositions` directly).

use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_collision::LinkPaddingScale;
use moveit_distance_field::add_link_body_decompositions;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;

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
/// with the oracle: the oracle links against real mesh files and so counts
/// mesh-only-collision links (`pr2.urdf`'s `base_link`, the caster rotation
/// links, `torso_lift_link`, every arm link, ...) as having collision
/// geometry, while this port's `RobotModel` deliberately never loads
/// `<mesh>` geometry at all -- see `moveit-model`'s `LinkModel` doc comment,
/// deviation 4, and its own `mesh_collision_is_skipped_with_a_diagnostic_and_leaves_no_shape`
/// test. Every such skip is recorded as a `Diagnostic::UnsupportedLinkGeometry`.
///
/// So the real property to check is not set-equality but: our computed set
/// is *exactly* the oracle's set minus the links whose divergence is
/// explained by a recorded mesh diagnostic -- i.e. every disagreement is
/// accounted for, none is silent.
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
