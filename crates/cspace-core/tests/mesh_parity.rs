// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `mesh` op
//! (`tools/moveit-oracle/src/oracle.cpp`'s `meshOp`), which calls
//! `shapes::createMeshFromResource` -- real Assimp, run inside the same
//! container image `RobotModel::constructShape` itself uses for a URDF
//! `<mesh>` element -- so this is a direct check of [`stl`]'s central,
//! evidence-based-but-unverifiable-from-source claim: that merging by exact
//! vertex-position equality (no tolerance, no degenerate-triangle removal)
//! reproduces what Assimp's `aiProcess_JoinIdenticalVertices` actually does.
//! See `crates/cspace-core/src/geometry/stl.rs`'s module doc for the full
//! evidence trail and why Assimp's own source could not be read instead.
//!
//! `tests/fixtures/mesh_parity.json` is the oracle's own response, captured
//! verbatim (one entry per request/response pair, not hand-transcribed) for
//! every panda, fanuc, and pr2 collision STL, plus one case (`link0.stl`
//! again, non-uniform `scale`) that exercises `createMeshFromResource`'s
//! scale argument specifically. The STL bytes each case parses come from
//! `fixtures/meshes/<package>/...` -- the repo-root fixture tree copied from
//! `third_party/moveit_resources` (see `tools/ci/verify-fixture-provenance.sh`),
//! not from `third_party/` directly, so this test runs under a plain
//! `cargo nextest run --workspace` with no vendored checkout required.
//!
//! pr2's 18 collision meshes were added a round after panda's and fanuc's,
//! specifically because nothing had ever checked them: `Shape::extents`/
//! `bounding_sphere` conclusions this crate's own doc cites for pr2 (and the
//! self-collision/penetration-depth arguments `cspace-collision` builds on
//! top of them) all rest on the vertex positions this port parses out of
//! those 18 files, and panda/fanuc passing here says nothing about whether
//! pr2's STLs -- which carry a different STL-writer header (`VCG`, i.e.
//! VCGLib, versus panda/fanuc's `Export`) -- exercise the same merge/order/
//! degenerate-triangle behavior. They do not diverge: all 18 pass the same
//! ordered vertex/triangle check below, captured via the oracle's `mesh` op
//! exactly as panda's and fanuc's were (see this change's commit body for
//! the exact resource list and how the fixture was captured).
//!
//! Not in `tests/fixtures/oracle-models.json`, deliberately:
//! `tools/ci/verify-fixture-replay.sh` replays a `<stem>_request.json`/
//! `<stem>_response.json` pair through the oracle's wire protocol verbatim
//! (each line an `{"id", "op", ...}` object in, an `{"id", "ok", "result"}`
//! object out). This file is real oracle output but already reshaped into
//! this test's own per-case schema (`resource`/`scale`/`vertex_count`/
//! `triangle_count`/`vertices`/`triangles`, no `id`/`op` at all) before being
//! committed, so it cannot be replayed as-is -- there is no wire-format
//! request to send back through `run-oracle.sh`. Re-capturing it in wire
//! format to make it replayable is a separate job from this round's audit.
//!
//! # Vertex order and triangle indices are both asserted
//!
//! This mesh-construction path is one of the remaining candidates for
//! `cspace-collision`'s deviation 6(b) audit: upstream builds
//! `fcl::BVHModel<OBBRSSd>` from *both* arrays together
//! (`collision_detection_fcl/collision_common.cpp:902-920` -- `points` in [`Mesh::vertices`] order,
//! `tri_indices` indexing that same order), so a port whose vertex set
//! matches but whose order or triangle indices do not would build a
//! different BVH, traverse it differently, and report a different
//! deepest-penetration point despite passing a set-only check. An earlier
//! version of this test compared vertex *sets*
//! (`shape_points_parity.rs`'s grid-bucketed-`HashSet` pattern) and did not
//! compare triangle indices at all, reasoning that
//! [`Mesh::merge_vertices`]'s first-occurrence dedup order was an
//! unverifiable implementation detail. That reasoning is now refuted: this
//! fixture stores vertices in the oracle's own emission order (it was never
//! reordered), so comparing index-for-index against [`mesh_from_bytes`]'s
//! output needed no new capture -- all 36 meshes agree element-wise, order
//! included. The oracle's `mesh` op now also emits `triangles`
//! (`meshOp` in `oracle.cpp`, oracle stamp `552427488cc040a2`), so the
//! second half -- triangle indices, which are only meaningful relative to a
//! specific vertex order -- is compared here too. Both matching closes the
//! mesh-construction candidate for deviation 6(b); either mismatching would
//! have been the root cause.

use std::fs;

use nalgebra::Vector3;
use serde::Deserialize;

use cspace_core::geometry::mesh_from_bytes;

/// Well below STL's `f32` vertex precision (~1e-7 relative for these models'
/// centimeter-to-meter-scale coordinates) and well above nothing. Unlike the
/// bucketing this constant used before, comparison here is ordered and
/// element-wise (see `body_query_parity.rs`'s `LINEAR_EPS`/`assert_vec_close`
/// for the same pattern), so this is a plain absolute tolerance, not a
/// quantization bucket.
const VERTEX_EPS: f64 = 1e-6;

#[derive(Deserialize)]
struct Case {
    resource: String,
    scale: [f64; 3],
    vertex_count: usize,
    triangle_count: usize,
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
}

fn load_cases() -> Vec<Case> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/geometry/mesh_parity.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

/// `package://moveit_resources_panda_description/meshes/collision/link0.stl`
/// -> `fixtures/meshes/panda_description/meshes/collision/link0.stl`.
/// Deliberately a small fixed map local to this test, not a general
/// `package://` resolver -- that resolver (mapping arbitrary package names
/// to arbitrary search paths) is `cspace-model`'s concern once `<mesh>`
/// loading is wired into `LinkModel`; this test only needs the three packages
/// its own fixture actually references.
fn resolve_resource(resource: &str) -> std::path::PathBuf {
    const PREFIX: &str = "package://";
    let rest = resource
        .strip_prefix(PREFIX)
        .unwrap_or_else(|| panic!("resource {resource:?} is not a package:// URI"));
    let (package, relative_path) = rest
        .split_once('/')
        .unwrap_or_else(|| panic!("resource {resource:?} has no path after the package name"));
    let package_dir = match package {
        "moveit_resources_panda_description" => "panda_description",
        "moveit_resources_fanuc_description" => "fanuc_description",
        "moveit_resources_pr2_description" => "pr2_description",
        other => panic!("resource {resource:?}: no fixture mapping for package {other:?}"),
    };
    [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "..",
        "fixtures",
        "meshes",
        package_dir,
        relative_path,
    ]
    .iter()
    .collect()
}

fn assert_vertex_close(actual: Vector3<f64>, expected: [f64; 3], ctx: &str) {
    assert!(
        (actual.x - expected[0]).abs() < VERTEX_EPS
            && (actual.y - expected[1]).abs() < VERTEX_EPS
            && (actual.z - expected[2]).abs() < VERTEX_EPS,
        "{ctx}: {actual:?} vs oracle {expected:?}"
    );
}

#[test]
fn mesh_from_bytes_matches_the_oracle_for_every_panda_fanuc_and_pr2_collision_stl() {
    let cases = load_cases();
    assert!(!cases.is_empty(), "mesh_parity.json fixture is empty");

    for case in &cases {
        let path = resolve_resource(&case.resource);
        let bytes = fs::read(&path)
            .unwrap_or_else(|e| panic!("{}: read {}: {e}", case.resource, path.display()));

        let scale = Vector3::new(case.scale[0], case.scale[1], case.scale[2]);
        let mesh = mesh_from_bytes(&bytes, scale)
            .unwrap_or_else(|e| panic!("{}: mesh_from_bytes: {e}", case.resource));

        assert_eq!(
            mesh.vertices.len(),
            case.vertex_count,
            "{}: vertex count disagrees with the oracle",
            case.resource
        );
        assert_eq!(
            mesh.triangles.len(),
            case.triangle_count,
            "{}: triangle count disagrees with the oracle",
            case.resource
        );

        for (i, (&actual, &expected)) in mesh.vertices.iter().zip(&case.vertices).enumerate() {
            assert_vertex_close(actual, expected, &format!("{}: vertex {i}", case.resource));
        }

        assert_eq!(
            mesh.triangles, case.triangles,
            "{}: triangle indices disagree with the oracle",
            case.resource
        );
    }
}
