// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Regression coverage for `ConvexMesh::ray_intersections`'s NaN handling
//! (`bodies.rs`): a NaN confined to one axis of `origin`/`dir` that no
//! per-triangle comparison can catch at all, because a dot product zeroes
//! out any component the other operand doesn't touch -- see that method's
//! own doc comment for the full mechanism.
//!
//! Before this fix, both `nan_component_*` tests below returned a point
//! with a NaN coordinate, indistinguishable from any other entry in the
//! `Vec<Vector3>` -- a corrupted answer handed back through the public
//! API, not merely a NaN observed mid-computation (verified directly: with
//! the entry-level finite check removed, both fail with `got [[NaN, NaN,
//! NaN]], ...]`).

use moveit_geometry::bodies::ConvexMesh;
use moveit_geometry::{Mesh as ShapeMesh, Vector3};

/// Upstream's own `test/test_shapes.cpp` tetrahedron: vertices at the
/// origin and one unit step along each axis. Reused here (rather than
/// invented) because its planes are simple enough to reason about by hand:
/// the `x = 0` face has outward normal `(-1, 0, 0)`.
fn tetrahedron() -> ShapeMesh {
    let vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    let triangles = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    ShapeMesh::new(vertices, triangles).expect("tetrahedron is a valid mesh")
}

/// A ray whose origin carries a NaN entirely in `y`, aimed along `+x` (`dir
/// = (1, 0, 0)`) so it hits the tetrahedron's `y = 0` face
/// (triangle `[0, 1, 3]`, outward normal `(0, -1, 0)`) dead on: `normal.dot
/// (&dir) = (0,-1,0)-dot-(1,0,0) = 0.0` is the one component `dir` carries,
/// so the NaN in `origin.y` never reaches `tmp` through this triangle's
/// normal either -- both `tmp` and `t` come out perfectly finite. The NaN
/// survives regardless, in `pt = origin + dir_norm * t`, computed straight
/// from `origin` and never routed through `tmp`/`t` at all.
#[test]
fn nan_component_orthogonal_to_every_hit_triangles_normal_is_not_silently_dropped() {
    let body = ConvexMesh::new(&tetrahedron()).expect("tetrahedron is a valid mesh");
    let origin = Vector3::new(-1.0, f64::NAN, 0.1);
    let dir = Vector3::new(1.0, 0.0, 0.0);

    let hits = body.ray_intersections(&origin, &dir, None);

    assert!(
        hits.iter().all(|pt| pt.iter().all(|c| c.is_finite())),
        "a NaN confined to one axis of the ray's origin must not reach a returned intersection \
         point: got {hits:?}"
    );
}

/// Same defect, `dir` carrying the NaN instead of `origin`. `dir = (1, NaN,
/// 0)` is not unit length, so `normalize_dir` rescales every component
/// (including the NaN) by a finite factor -- the NaN survives normalization
/// unchanged in sign-of-non-finiteness, just relabelled.
#[test]
fn nan_component_in_dir_is_not_silently_dropped_either() {
    let body = ConvexMesh::new(&tetrahedron()).expect("tetrahedron is a valid mesh");
    let origin = Vector3::new(-1.0, 0.1, 0.1);
    let dir = Vector3::new(1.0, f64::NAN, 0.0);

    let hits = body.ray_intersections(&origin, &dir, None);

    assert!(
        hits.iter().all(|pt| pt.iter().all(|c| c.is_finite())),
        "a NaN confined to one axis of the ray's direction must not reach a returned \
         intersection point: got {hits:?}"
    );
}

/// The fix must not turn every ray into "no intersections": an ordinary,
/// fully finite ray that actually crosses the mesh must still report a
/// finite hit.
#[test]
fn an_ordinary_finite_ray_still_reports_its_hit() {
    let body = ConvexMesh::new(&tetrahedron()).expect("tetrahedron is a valid mesh");
    let origin = Vector3::new(-1.0, 0.2, 0.2);
    let dir = Vector3::new(1.0, 0.0, 0.0);

    let hits = body.ray_intersections(&origin, &dir, None);

    assert!(
        !hits.is_empty(),
        "a ray aimed straight through the tetrahedron's y=0/z=0 wedge must hit it"
    );
    assert!(hits.iter().all(|pt| pt.iter().all(|c| c.is_finite())));
}
