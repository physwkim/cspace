// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/robot_model.cpp (RobotModel::constructShape's
//   MESH case: `shapes::createMeshFromResource(mesh->filename, scale)`)
// and from geometric_shapes 2.3.3's src/mesh_operations.cpp
// (createMeshFromResource/createMeshFromBinary/createMeshFromAsset/
// extractMeshData) — see [`crate::shapes`]'s provenance comment for how that
// source was obtained and verified.

//! STL parsing for `<mesh>` collision/visual geometry: the on-disk half of
//! `RobotModel::constructShape`'s `MESH` case. `package://` resolution and
//! wiring this into `LinkModel` live in `moveit-model`, which has the URDF
//! and package-search-path context this crate deliberately does not.
//!
//! # What upstream's call chain actually does
//!
//! `constructShape` calls `shapes::createMeshFromResource(filename, scale)`,
//! which (`mesh_operations.cpp`) retrieves the file, then calls
//! `createMeshFromBinary`:
//!
//! ```text
//! Assimp::Importer importer;
//! importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, /* strip normals, uv, etc. */);
//! importer.ReadFileFromMemory(buffer, size,
//!     aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
//!     aiProcess_SortByPType | aiProcess_RemoveComponent, hint);
//! scene->mRootNode->mTransformation = aiMatrix4x4();  // reset before...
//! importer.ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
//! ```
//!
//! then `extractMeshData` walks the scene graph applying each node's
//! transform and `scale`, and `createMeshFromVertices(vertices, triangles)` —
//! the two-`Vec` overload — copies that data into a `shapes::Mesh` verbatim
//! (**no further deduplication**) and then unconditionally calls
//! `mesh->computeTriangleNormals(); mesh->computeVertexNormals();` before
//! returning it (geometric_shapes 2.3.3's `mesh_operations.cpp`, read from a
//! source checkout, since the ROS `rolling` install this port's oracle uses
//! ships only the compiled `.so` and headers — see below). [`mesh_from_bytes`]
//! does the same: every mesh it returns already has both normals populated,
//! matching that every downstream consumer (this port's collision backend
//! calls [`crate::shapes::Mesh::scale_and_padd`] on a robot link's own
//! shape) can rely on them being present without computing them itself.
//!
//! So the per-vertex merge a `<mesh>` collision file goes through is
//! `aiProcess_JoinIdenticalVertices`, run *inside* Assimp during
//! `ReadFileFromMemory` — not `Mesh::mergeVertices(threshold)`
//! ([`crate::shapes::Mesh::merge_vertices`]'s own upstream) and not
//! `mesh_operations.cpp`'s other `createMeshFromVertices(source)` overload
//! (the one with an internal `std::set`-based dedup). Both of those are
//! separate, real upstream functions — but grepping geometric_shapes 2.3.3
//! and moveit2 @ the pinned SHA for callers finds neither one anywhere in the
//! `RobotModel::constructShape` call chain: `Mesh::mergeVertices`'s only
//! moveit2 caller is `planning_scene_monitor.cpp:906`, which is octomap/
//! perception world-mesh handling, not robot link collision geometry.
//!
//! Assimp itself is not vendored anywhere reachable — checked both the host
//! and the oracle image (`moveit-rs/oracle:1717af5da743934c`): only the
//! compiled `libassimp5` 5.3.1+ds-2build1 `.so` and its public C API headers
//! are installed (`dpkg -l`/`apt-cache policy` show no source package, and
//! there is no `apt source` cache entry), so `STLLoader.cpp` and
//! `JoinVerticesProcess.cpp` cannot be read. The public `postprocess.h`
//! header (read from the oracle image) is the closest available source of
//! truth on Assimp's own documented behavior here, and it settles two things
//! this module's design depends on:
//!
//! - `aiProcess_JoinIdenticalVertices` is documented only as "identifies and
//!   joins identical vertex data sets" — no tolerance is mentioned.
//! - Degenerate-triangle removal is `aiProcess_FindDegenerates`, a *separate*
//!   opt-in flag (gated further behind the `AI_CONFIG_PP_FD_REMOVE`/
//!   `AI_CONFIG_PP_SBP_REMOVE` importer properties, both off by default) that
//!   `createMeshFromBinary`'s flag list — `Triangulate | JoinIdenticalVertices
//!   | SortByPType | RemoveComponent` — does not request at all. Neither
//!   `OptimizeMeshes` nor `OptimizeGraph` (applied afterward) touches
//!   per-mesh geometry either; both only affect the scene graph / draw-call
//!   count.
//!
//! [`mesh_from_bytes`] therefore merges by **exact** vertex-position equality
//! ([`crate::shapes::Mesh::merge_vertices`] with a `0.0` threshold — matching
//! "identical", not "close") and performs no degenerate-triangle filtering.
//! This is a reconstruction from the closest available primary source, not a
//! transcription of Assimp's actual implementation, so it is independently
//! verified rather than assumed: `tests/mesh_parity.rs` (in this crate)
//! compares vertex count, triangle count and the full vertex array against
//! the oracle's own `shapes::createMeshFromResource` output (real Assimp, via
//! a real `ros-rolling-geometric-shapes`/`libassimp5` install) for every
//! panda and fanuc collision STL. A wrong guess about Assimp's merge
//! tolerance or degenerate handling shows up there as a fixture mismatch, not
//! a silently-passing test.
//!
//! # Binary vs. ASCII detection
//!
//! **Not** the `solid` prefix: plenty of binary STLs begin with the ASCII
//! word `solid` in their free-form 80-byte header — every panda collision
//! mesh here begins `Exported from Bl...`, which happens to not start with
//! `solid`, but nothing in the format guarantees that; a `solid`-prefixed
//! binary file is a real, common case (`tests` covers one built in the test
//! itself). The reliable test is the exact byte-length identity a binary STL
//! must satisfy: `84 + 50 * triangle_count`, with `triangle_count` read as a
//! little-endian `u32` at offset 80. [`mesh_from_bytes`] checks that
//! identity and only falls back to ASCII when it fails.
//!
//! Binary vertex components are IEEE 754 `f32` (widened to `f64` after
//! reading, matching Assimp's `aiVector3D` being single-precision
//! internally); every collision mesh in the four fixture robots is binary,
//! confirmed by exact byte-length check against each file on disk, so this
//! path is what `tests/mesh_parity.rs` exercises. The ASCII path parses each
//! `vertex x y z` coordinate directly as `f64` text; no ASCII fixture exists
//! to verify precision-matching against Assimp's own ASCII importer, so that
//! part is unverified (recorded here rather than silently assumed correct).
//! Multi-solid ASCII files also are not exercised: this module always merges
//! vertices across the whole file as one pass, whereas Assimp runs
//! `JoinIdenticalVertices` per `aiMesh` (one per `solid` block in a
//! multi-solid ASCII file) before `extractMeshData` concatenates them — a
//! difference invisible on every real fixture here, since binary STL has no
//! concept of multiple solids at all.
//!
//! # `mesh_operations.h` symbol audit (round 8)
//!
//! Every public declaration in the oracle container's installed
//! `geometric_shapes/include/geometric_shapes/mesh_operations.h`, classified
//! against `moveit2`'s pinned-tree callers so the next audit can re-run this
//! rather than re-derive it:
//!
//! - `createMeshFromVertices(vertices, triangles)` (the two-`Vec` overload) —
//!   **ported as [`crate::shapes::Mesh::new`]**, called from this module's own
//!   [`mesh_from_bytes`] the same way upstream's `extractMeshData` calls it.
//! - `createMeshFromVertices(source)` (the single-`Vec`, `std::set`-dedup
//!   overload) — **unported.** `rg -n 'createMeshFromVertices\('
//!   /home/stevek/work/moveit2` finds exactly one call, `semantic_world.cpp:154`,
//!   using the two-`Vec` overload above, not this one; this overload has zero
//!   callers anywhere in the pinned tree. Its constituents
//!   ([`crate::shapes::Mesh::new`], [`crate::shapes::Mesh::merge_vertices`],
//!   [`crate::shapes::Mesh::compute_triangle_normals`]) already exist, so it
//!   is trivially composable if a caller ever needs it — unlike
//!   `body_operations.cpp`'s `computeBoundingSphere(vector)`, this is not
//!   asserted as equivalent to any already-shipped port, since no caller
//!   requires it be built.
//! - `createMeshFromResource(resource)`, `createMeshFromResource(resource,
//!   scale)`, `createMeshFromBinary(buffer, size, hint)`,
//!   `createMeshFromBinary(buffer, size, scale, hint)` — **D-decision
//!   excludes the general case** (assimp is not a workspace dependency;
//!   PORTING-PLAN.md records no Rust assimp binding pulled in, and this
//!   module's own doc above records that even the reference source for
//!   Assimp's behavior had to come from a public header, since no assimp
//!   source/-dbgsym package exists on this machine either). The one in-scope
//!   caller of the `(resource, scale)` overload — `robot_model.cpp:1280`,
//!   `RobotModel::constructShape`'s `MESH` case — is real and ported, but
//!   narrowed to the STL-only subset this crate's fixtures actually need:
//!   [`mesh_from_bytes`] is that port, verified against real Assimp output in
//!   `tests/mesh_parity.rs` rather than merely assumed.
//! - `createMeshFromAsset(scene, scale, hint)`, `createMeshFromAsset(scene,
//!   hint)` — **D-decision excludes.** Both take `const aiScene*`, an assimp
//!   type, as a parameter — there is no STL-only narrowing possible here the
//!   way there is for `createMeshFromResource`, since these operate on an
//!   already-parsed assimp scene graph, not a byte buffer this port could
//!   intercept before assimp sees it.
//! - `createMeshFromShape(const Shape*)` and its four single-type overloads
//!   (`Box`, `Sphere`, `Cylinder`, `Cone`) — **unported.** `rg -n
//!   'createMeshFromShape\(' /home/stevek/work/moveit2` finds two callers,
//!   `render_shapes.cpp:86` (the `Cone` overload) and
//!   `depth_image_octomap_updater.cpp:213` (the polymorphic `Shape*`
//!   overload) — both `moveit_ros` (out of scope per PORTING-PLAN.md §1),
//!   zero `moveit_core` callers.
//! - `writeSTLBinary(const Mesh*, buffer)` — **unported.** `rg -n
//!   'writeSTLBinary' /home/stevek/work/moveit2` returns no hits at all —
//!   zero callers anywhere in the pinned tree, in or out of scope.

use crate::Vector3;
use crate::shapes::Mesh;
use moveit_error::{Error, Result};

const BINARY_HEADER_LEN: usize = 84;
const BINARY_TRIANGLE_RECORD_LEN: usize = 50;

/// `shapes::createMeshFromBinary` + `extractMeshData` +
/// `createMeshFromVertices`, for STL specifically: parse `bytes` (binary or
/// ASCII, auto-detected), scale each vertex by `scale`, and merge exactly
/// coincident vertices — see the module doc for why `0.0` is the right
/// threshold here rather than [`Mesh::merge_vertices`]'s general tolerance
/// parameter.
///
/// # Errors
///
/// [`Error::Construct`] if `bytes` is neither a well-formed binary STL nor
/// parseable as ASCII STL.
pub fn mesh_from_bytes(bytes: &[u8], scale: Vector3) -> Result<Mesh> {
    let triangles = parse_triangles(bytes)?;
    let mut vertices = Vec::with_capacity(triangles.len() * 3);
    let mut triangle_indices = Vec::with_capacity(triangles.len());
    for [a, b, c] in triangles {
        let base = vertices.len() as u32;
        vertices.push(a.component_mul(&scale));
        vertices.push(b.component_mul(&scale));
        vertices.push(c.component_mul(&scale));
        triangle_indices.push([base, base + 1, base + 2]);
    }
    let mut mesh = Mesh::new(vertices, triangle_indices)?;
    mesh.merge_vertices(0.0);
    // `createMeshFromVertices(vertices, triangles)` -- the overload the real
    // `extractMeshData`/`createMeshFromResource` call chain uses -- computes
    // both unconditionally before returning, every time, not lazily on first
    // use (geometric_shapes 2.3.3's `mesh_operations.cpp`, `createMeshFromVertices`).
    // Without this, any later [`Mesh::scale_and_padd`] call (a collision
    // backend applying link padding) would hit the `vertex_normals: None`
    // error path unconditionally, since nothing else in this load path ever
    // populates it.
    mesh.compute_vertex_normals();
    Ok(mesh)
}

/// Every triangle's three vertices, in file order, with no vertex sharing —
/// the raw shape both the binary and ASCII STL formats store data in.
fn parse_triangles(bytes: &[u8]) -> Result<Vec<[Vector3; 3]>> {
    match parse_binary_triangles(bytes) {
        Some(triangles) => Ok(triangles),
        None => parse_ascii_triangles(bytes),
    }
}

/// `Some` only if `bytes` satisfies binary STL's exact length identity —
/// see the module doc for why this, not the `solid` prefix, is the test.
fn parse_binary_triangles(bytes: &[u8]) -> Option<Vec<[Vector3; 3]>> {
    if bytes.len() < BINARY_HEADER_LEN {
        return None;
    }
    let triangle_count_bytes: [u8; 4] = bytes[80..BINARY_HEADER_LEN].try_into().ok()?;
    let triangle_count = u32::from_le_bytes(triangle_count_bytes) as usize;
    let expected_len =
        BINARY_HEADER_LEN.checked_add(triangle_count.checked_mul(BINARY_TRIANGLE_RECORD_LEN)?)?;
    if expected_len != bytes.len() {
        return None;
    }

    let mut triangles = Vec::with_capacity(triangle_count);
    for i in 0..triangle_count {
        let record = &bytes[BINARY_HEADER_LEN + i * BINARY_TRIANGLE_RECORD_LEN..];
        // Bytes 0..12 are the facet normal, which this loader does not read:
        // `Mesh::compute_triangle_normals` derives it from the vertices
        // themselves, matching `aiProcess_RemoveComponent`'s
        // `aiComponent_NORMALS` stripping normals before they ever reach
        // `extractMeshData`.
        let v0 = read_f32_vec3(&record[12..24]);
        let v1 = read_f32_vec3(&record[24..36]);
        let v2 = read_f32_vec3(&record[36..48]);
        triangles.push([v0, v1, v2]);
    }
    Some(triangles)
}

fn read_f32_vec3(bytes: &[u8]) -> Vector3 {
    let x = f32::from_le_bytes(bytes[0..4].try_into().expect("4-byte slice"));
    let y = f32::from_le_bytes(bytes[4..8].try_into().expect("4-byte slice"));
    let z = f32::from_le_bytes(bytes[8..12].try_into().expect("4-byte slice"));
    Vector3::new(x as f64, y as f64, z as f64)
}

/// Every `vertex x y z` line in the file, grouped into triangles of three —
/// ASCII STL has no other structure a loader needs: `solid`/`facet
/// normal`/`outer loop`/`endloop`/`endfacet`/`endsolid` are all skipped by
/// scanning for the `vertex` keyword rather than tracking nesting.
fn parse_ascii_triangles(bytes: &[u8]) -> Result<Vec<[Vector3; 3]>> {
    let text = std::str::from_utf8(bytes)
        .map_err(|e| Error::construct(format!("ASCII STL is not valid UTF-8: {e}")))?;

    let mut tokens = text.split_ascii_whitespace();
    let mut flat_vertices = Vec::new();
    while let Some(token) = tokens.next() {
        if token != "vertex" {
            continue;
        }
        let x = next_coordinate(&mut tokens)?;
        let y = next_coordinate(&mut tokens)?;
        let z = next_coordinate(&mut tokens)?;
        flat_vertices.push(Vector3::new(x, y, z));
    }

    if flat_vertices.is_empty() {
        return Err(Error::construct(
            "not a binary STL (length mismatch) and no `vertex` line found for ASCII STL",
        ));
    }
    if flat_vertices.len() % 3 != 0 {
        return Err(Error::construct(format!(
            "ASCII STL has {} `vertex` lines, which is not a multiple of 3",
            flat_vertices.len()
        )));
    }
    Ok(flat_vertices
        .chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect())
}

fn next_coordinate<'a>(tokens: &mut impl Iterator<Item = &'a str>) -> Result<f64> {
    let token = tokens
        .next()
        .ok_or_else(|| Error::construct("ASCII STL `vertex` line is missing a coordinate"))?;
    token.parse::<f64>().map_err(|e| {
        Error::construct(format!(
            "ASCII STL coordinate {token:?} is not a number: {e}"
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Builds a binary STL buffer: 80-byte header (caller-supplied, so a test
    /// can start it with the literal bytes `solid`), a `u32` triangle count,
    /// then 50-byte records (12-byte normal — left zero — plus three `f32`
    /// vertex triples, plus a 2-byte attribute count, also left zero).
    fn binary_stl(header: &[u8], triangles: &[[[f32; 3]; 3]]) -> Vec<u8> {
        let mut buffer = vec![0u8; BINARY_HEADER_LEN];
        buffer[..header.len().min(80)].copy_from_slice(&header[..header.len().min(80)]);
        buffer[80..84].copy_from_slice(&(triangles.len() as u32).to_le_bytes());
        for triangle in triangles {
            buffer.extend_from_slice(&[0u8; 12]); // normal, unread
            for vertex in triangle {
                for component in vertex {
                    buffer.extend_from_slice(&component.to_le_bytes());
                }
            }
            buffer.extend_from_slice(&[0u8; 2]); // attribute byte count
        }
        buffer
    }

    #[test]
    fn binary_stl_whose_header_starts_with_solid_is_still_read_as_binary() {
        // The case a `solid`-prefix sniffer gets wrong: this is a real binary
        // STL (its length is exactly 84 + 50 * triangle_count), but its
        // 80-byte header starts with the ASCII word "solid", same as panda's
        // real link0.stl starts with "Exported from Bl..." -- free-form text,
        // not a format signal.
        let triangle = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let bytes = binary_stl(b"solid this-is-actually-binary, not ascii", &[triangle]);

        let mesh = mesh_from_bytes(&bytes, Vector3::new(1.0, 1.0, 1.0)).expect("valid binary STL");

        assert_eq!(mesh.vertices.len(), 3);
        assert_eq!(mesh.triangles.len(), 1);
    }

    #[test]
    fn binary_stl_merges_vertices_shared_across_triangles() {
        // Two triangles sharing an edge: 6 raw (vertex, triangle) slots but
        // only 4 distinct positions, matching aiProcess_JoinIdenticalVertices
        // merging a shared edge into shared indices.
        let t1 = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
        let t2 = [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
        let bytes = binary_stl(b"solid quad", &[t1, t2]);

        let mesh = mesh_from_bytes(&bytes, Vector3::new(1.0, 1.0, 1.0)).expect("valid binary STL");

        assert_eq!(mesh.vertices.len(), 4);
        assert_eq!(mesh.triangles.len(), 2);
    }

    #[test]
    fn binary_stl_applies_scale_per_component() {
        let triangle = [[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0]];
        let bytes = binary_stl(b"solid degenerate", &[triangle]);

        let mesh = mesh_from_bytes(&bytes, Vector3::new(2.0, 0.5, 10.0)).expect("valid binary STL");

        assert_relative_eq!(mesh.vertices[0], Vector3::new(2.0, 1.0, 30.0));
    }

    #[test]
    fn ascii_stl_is_read_when_the_length_does_not_match_binary() {
        let text = "solid ascii_cube\n\
             facet normal 0 0 1\n\
               outer loop\n\
                 vertex 0 0 0\n\
                 vertex 1 0 0\n\
                 vertex 0 1 0\n\
               endloop\n\
             endfacet\n\
             endsolid ascii_cube\n";

        let mesh =
            mesh_from_bytes(text.as_bytes(), Vector3::new(1.0, 1.0, 1.0)).expect("valid ASCII STL");

        assert_eq!(mesh.vertices.len(), 3);
        assert_eq!(mesh.triangles.len(), 1);
    }

    #[test]
    fn empty_input_is_rejected() {
        assert!(mesh_from_bytes(&[], Vector3::new(1.0, 1.0, 1.0)).is_err());
    }
}
