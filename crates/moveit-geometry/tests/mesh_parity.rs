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
//! See `crates/moveit-geometry/src/stl.rs`'s module doc for the full
//! evidence trail and why Assimp's own source could not be read instead.
//!
//! `tests/fixtures/mesh_parity.json` is the oracle's own response, captured
//! verbatim (one entry per request/response pair, not hand-transcribed) for
//! every panda and fanuc collision STL, plus one case (`link0.stl` again,
//! non-uniform `scale`) that exercises `createMeshFromResource`'s scale
//! argument specifically. The STL bytes each case parses come from
//! `fixtures/meshes/<package>/...` -- the repo-root fixture tree copied from
//! `third_party/moveit_resources` (see `tools/ci/verify-fixture-provenance.sh`),
//! not from `third_party/` directly, so this test runs under a plain
//! `cargo nextest run --workspace` with no vendored checkout required.
//!
//! Not in `tests/fixtures/oracle-models.json`, deliberately:
//! `tools/ci/verify-fixture-replay.sh` replays a `<stem>_request.json`/
//! `<stem>_response.json` pair through the oracle's wire protocol verbatim
//! (each line an `{"id", "op", ...}` object in, an `{"id", "ok", "result"}`
//! object out). This file is real oracle output but already reshaped into
//! this test's own per-case schema (`resource`/`scale`/`vertex_count`/
//! `triangle_count`/`vertices`, no `id`/`op` at all) before being committed,
//! so it cannot be replayed as-is -- there is no wire-format request to send
//! back through `run-oracle.sh`. Re-capturing it in wire format to make it
//! replayable is a separate job from this round's audit.
//!
//! # Vertex order is not asserted
//!
//! [`Mesh::merge_vertices`]'s dedup keeps each vertex at its first-occurrence
//! index; there is no evidence (Assimp's source is unavailable, see the
//! module doc) that `aiProcess_JoinIdenticalVertices` preserves the same
//! order internally. Comparing vertex arrays index-for-index would be
//! asserting an implementation detail neither side documents, so this
//! compares vertex *sets* (`shape_points_parity.rs`'s
//! grid-bucketed-`HashSet` pattern) instead of ordered arrays. `triangle`
//! index arrays are consequently not compared either: they are only
//! meaningful relative to a specific vertex order.

use std::collections::HashSet;
use std::fs;

use nalgebra::Vector3;
use serde::Deserialize;

use moveit_geometry::mesh_from_bytes;

/// Well below STL's `f32` vertex precision (~1e-7 relative for these models'
/// centimeter-to-meter-scale coordinates) and well above nothing -- this
/// buckets rather than tolerates, so two vertices a few ULPs apart that
/// straddle a bucket edge read as a mismatch rather than silently passing.
/// See `shape_points_parity.rs`'s `POINT_EPS` for the same reasoning.
const VERTEX_EPS: f64 = 1e-6;

#[derive(Deserialize)]
struct Case {
    resource: String,
    scale: [f64; 3],
    vertex_count: usize,
    triangle_count: usize,
    vertices: Vec<[f64; 3]>,
}

fn load_cases() -> Vec<Case> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/mesh_parity.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

/// `package://moveit_resources_panda_description/meshes/collision/link0.stl`
/// -> `fixtures/meshes/panda_description/meshes/collision/link0.stl`.
/// Deliberately a small fixed map local to this test, not a general
/// `package://` resolver -- that resolver (mapping arbitrary package names
/// to arbitrary search paths) is `moveit-model`'s concern once `<mesh>`
/// loading is wired into `LinkModel`; this test only needs the two packages
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

fn vertex_key(v: Vector3<f64>) -> [i64; 3] {
    [
        (v.x / VERTEX_EPS).round() as i64,
        (v.y / VERTEX_EPS).round() as i64,
        (v.z / VERTEX_EPS).round() as i64,
    ]
}

#[test]
fn mesh_from_bytes_matches_the_oracle_for_every_panda_and_fanuc_collision_stl() {
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

        let actual: HashSet<[i64; 3]> = mesh.vertices.iter().copied().map(vertex_key).collect();
        let expected: HashSet<[i64; 3]> = case
            .vertices
            .iter()
            .map(|&v| Vector3::new(v[0], v[1], v[2]))
            .map(vertex_key)
            .collect();

        let missing: Vec<_> = expected.difference(&actual).collect();
        let extra: Vec<_> = actual.difference(&expected).collect();
        assert!(
            missing.is_empty() && extra.is_empty(),
            "{}: vertex sets disagree -- {} the oracle has that this port does not: {:?}; \
             {} this port has that the oracle does not: {:?}",
            case.resource,
            missing.len(),
            missing,
            extra.len(),
            extra
        );
    }
}
