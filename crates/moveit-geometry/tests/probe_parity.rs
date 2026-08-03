// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`moveit_geometry::bodies`] against the real
//! `libgeometric_shapes.so`.
//!
//! `geometric_shapes` is a separate upstream package with no source on this
//! machine, so `PORTING-PLAN.md` §9.1 fixes the verification route for it:
//! ask the *binary* that actually ships, not the source we fetched off a
//! GitHub tag. `tests/fixtures/bodies_probe.json` is the stdout of a C++
//! probe linked against `/opt/ros/rolling/lib/libgeometric_shapes.so.2.3.3`
//! inside the `moveit-rs/oracle-base` image, printed at `%.17g` so no
//! precision is lost on the way out.
//!
//! The shapes layer already went through this; `bodies::` was ported from
//! upstream's own gtest literals instead, which is weaker in one specific
//! way: those literals live in `test/*.cpp`, which is not compiled into the
//! `.so`, so the string-table check that ties our fetched source to the
//! shipped binary does not cover them.
//!
//! Every body is posed at a translated *and* rotated pose rather than at the
//! identity. A pure rotation cannot expose the defect this layer actually
//! had — `Isometry3 * Vector3` dropping translation — so an identity-posed
//! fixture would assert nothing about the fix.
//!
//! To regenerate: see `tools/ci/` and the compile recipe in
//! `PORTING-PLAN.md` §9.1.

use std::collections::BTreeMap;
use std::fs;

use moveit_geometry::bodies::{ConvexMesh, Cuboid, Cylinder, OBB, Sphere};
use moveit_geometry::{Isometry3, Mesh as ShapeMesh, Vector3};
use nalgebra::{Translation3, UnitQuaternion};
use serde_json::Value;

/// Componentwise tolerance. The probe prints shortest-round-trip `%.17g` and
/// both sides run the same double-precision arithmetic, so anything above
/// accumulated rounding is a real disagreement.
const EPS: f64 = 1e-12;

fn fixture() -> BTreeMap<String, Value> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/bodies_probe.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn expect_f64(f: &BTreeMap<String, Value>, key: &str) -> f64 {
    f.get(key)
        .unwrap_or_else(|| panic!("fixture has no key {key}"))
        .as_f64()
        .unwrap_or_else(|| panic!("{key} is not a number"))
}

fn expect_vec3(f: &BTreeMap<String, Value>, key: &str) -> Vector3 {
    let a = f
        .get(key)
        .unwrap_or_else(|| panic!("fixture has no key {key}"))
        .as_array()
        .unwrap_or_else(|| panic!("{key} is not an array"));
    assert_eq!(a.len(), 3, "{key} is not a 3-vector");
    Vector3::new(
        a[0].as_f64().unwrap(),
        a[1].as_f64().unwrap(),
        a[2].as_f64().unwrap(),
    )
}

fn expect_usize(f: &BTreeMap<String, Value>, key: &str) -> usize {
    f.get(key)
        .unwrap_or_else(|| panic!("fixture has no key {key}"))
        .as_u64()
        .unwrap_or_else(|| panic!("{key} is not an integer")) as usize
}

fn assert_close(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= EPS,
        "{what}: rust {actual:.17} vs geometric_shapes {expected:.17}"
    );
}

fn assert_vec_close(actual: Vector3, expected: Vector3, what: &str) {
    for i in 0..3 {
        assert_close(actual[i], expected[i], &format!("{what}[{i}]"));
    }
}

/// The probe's `POSE`: translated and rotated about a non-axis direction.
fn probe_pose() -> Isometry3 {
    let axis = nalgebra::Unit::new_normalize(Vector3::new(1.0, 2.0, -0.5));
    Isometry3::from_parts(
        Translation3::new(0.3, -0.7, 1.1),
        UnitQuaternion::from_axis_angle(&axis, 0.9),
    )
}

/// The probe's containment query points, in its order.
fn probe_points() -> Vec<Vector3> {
    vec![
        Vector3::new(0.3, -0.7, 1.1),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.35, -0.62, 1.18),
        Vector3::new(1.5, 1.5, 1.5),
        Vector3::new(0.3, -0.7, 1.35),
        Vector3::new(0.3, -0.45, 1.1),
    ]
}

/// The probe's rays, in its order. Directions are normalized on both sides.
fn probe_rays() -> Vec<(Vector3, Vector3)> {
    vec![
        (Vector3::new(0.3, -0.7, -3.0), Vector3::new(0.0, 0.0, 1.0)),
        (Vector3::new(-3.0, -0.7, 1.1), Vector3::new(1.0, 0.0, 0.0)),
        (Vector3::new(0.3, -3.0, 1.1), Vector3::new(0.0, 1.0, 0.0)),
        (Vector3::new(5.0, 5.0, 5.0), Vector3::new(1.0, 1.0, 1.0)),
        (Vector3::new(0.3, -0.7, 1.1), Vector3::new(0.4, 0.5, 0.77)),
    ]
}

/// The probe's tetrahedron, vertices and winding as written there.
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

/// Every body exposes the same query surface but through inherent methods on
/// distinct types, so the shared assertions are a macro rather than a
/// function over a trait object.
macro_rules! check_body {
    ($f:expr, $name:literal, $body:expr) => {{
        let f = $f;
        let body = $body;

        for (i, p) in probe_points().iter().enumerate() {
            let key = format!("{}.contains[{}]", $name, i);
            let expected = expect_usize(&f, &key) == 1;
            assert_eq!(body.contains_point(p), expected, "{key}");
        }

        for (i, (origin, dir)) in probe_rays().iter().enumerate() {
            let dir = dir.normalize();
            let hits = body.ray_intersections(origin, &dir, Some(2));
            let hit_key = format!("{}.ray[{}].hit", $name, i);
            let count_key = format!("{}.ray[{}].count", $name, i);
            assert_eq!(
                !hits.is_empty(),
                expect_usize(&f, &hit_key) == 1,
                "{hit_key}"
            );
            assert_eq!(hits.len(), expect_usize(&f, &count_key), "{count_key}");
            for (j, hit) in hits.iter().enumerate() {
                let key = format!("{}.ray[{}].pt[{}]", $name, i, j);
                assert_vec_close(*hit, expect_vec3(&f, &key), &key);
            }
        }

        assert_close(
            body.compute_volume(),
            expect_f64(&f, &format!("{}.volume", $name)),
            &format!("{}.volume", $name),
        );

        let bs = body.compute_bounding_sphere();
        assert_vec_close(
            bs.center,
            expect_vec3(&f, &format!("{}.bsphere.center", $name)),
            &format!("{}.bsphere.center", $name),
        );
        assert_close(
            bs.radius,
            expect_f64(&f, &format!("{}.bsphere.radius", $name)),
            &format!("{}.bsphere.radius", $name),
        );

        let bc = body.compute_bounding_cylinder();
        assert_close(
            bc.radius,
            expect_f64(&f, &format!("{}.bcyl.radius", $name)),
            &format!("{}.bcyl.radius", $name),
        );
        assert_close(
            bc.length,
            expect_f64(&f, &format!("{}.bcyl.length", $name)),
            &format!("{}.bcyl.length", $name),
        );
        assert_vec_close(
            bc.pose.translation.vector,
            expect_vec3(&f, &format!("{}.bcyl.origin", $name)),
            &format!("{}.bcyl.origin", $name),
        );

        let aabb = body.compute_bounding_aabb();
        assert_vec_close(
            aabb.min(),
            expect_vec3(&f, &format!("{}.aabb.min", $name)),
            &format!("{}.aabb.min", $name),
        );
        assert_vec_close(
            aabb.max(),
            expect_vec3(&f, &format!("{}.aabb.max", $name)),
            &format!("{}.aabb.max", $name),
        );

        let obb = body.compute_bounding_obb();
        assert_vec_close(
            obb.extents(),
            expect_vec3(&f, &format!("{}.obb.extents", $name)),
            &format!("{}.obb.extents", $name),
        );
        assert_vec_close(
            obb.pose().translation.vector,
            expect_vec3(&f, &format!("{}.obb.origin", $name)),
            &format!("{}.obb.origin", $name),
        );
    }};
}

#[test]
fn sphere_matches_libgeometric_shapes() {
    let f = fixture();
    let mut body = Sphere::new(0.4).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.3).unwrap();
    body.set_padding(0.05).unwrap();
    check_body!(f, "sphere", body);
}

#[test]
fn cylinder_matches_libgeometric_shapes() {
    let f = fixture();
    let mut body = Cylinder::new(0.25, 0.9).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.1).unwrap();
    body.set_padding(0.02).unwrap();
    check_body!(f, "cylinder", body);
}

#[test]
fn cuboid_matches_libgeometric_shapes() {
    let f = fixture();
    let mut body = Cuboid::new(0.5, 0.7, 0.3).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.2).unwrap();
    body.set_padding(0.03).unwrap();
    check_body!(f, "box", body);
}

#[test]
fn convex_mesh_matches_libgeometric_shapes() {
    let f = fixture();
    let mut body = ConvexMesh::new(&tetrahedron()).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.15).unwrap();
    body.set_padding(0.01).unwrap();
    check_body!(f, "convexmesh", body);
}

/// `OBB::overlaps`/`contains`/`extend_approx` is the corner of this layer
/// upstream's own tests cover only with loose sanity bounds, because the real
/// values come out of FCL internals. The probe pins them exactly.
#[test]
fn obb_predicates_match_libgeometric_shapes() {
    let f = fixture();
    let identity = Isometry3::identity();
    let a = OBB::new(identity, Vector3::new(1.0, 1.0, 1.0));

    let axis = nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
    let c = OBB::new(
        Isometry3::from_parts(
            Translation3::new(0.5, 0.5, 0.5),
            UnitQuaternion::from_axis_angle(&axis, 0.7),
        ),
        Vector3::new(0.4, 1.2, 0.6),
    );

    assert_eq!(
        a.overlaps(&c),
        expect_usize(&f, "obb.a_overlaps_c") == 1,
        "obb.a_overlaps_c"
    );
    assert_eq!(
        a.contains_obb(&c),
        expect_usize(&f, "obb.a_contains_c") == 1,
        "obb.a_contains_c"
    );
    assert_eq!(
        a.contains_point(&Vector3::zeros()),
        expect_usize(&f, "obb.a_contains_origin") == 1,
        "obb.a_contains_origin"
    );

    let expected_vertices = f
        .get("obb.a.vertices")
        .expect("fixture has no key obb.a.vertices")
        .as_array()
        .expect("obb.a.vertices is not an array");
    let mut actual: Vec<[f64; 3]> = a
        .compute_vertices()
        .iter()
        .map(|v| [v.x, v.y, v.z])
        .collect();
    let mut expected: Vec<[f64; 3]> = expected_vertices
        .iter()
        .map(|v| {
            let a = v.as_array().unwrap();
            [
                a[0].as_f64().unwrap(),
                a[1].as_f64().unwrap(),
                a[2].as_f64().unwrap(),
            ]
        })
        .collect();
    // Upstream does not document a vertex order, so compare as sets: an order
    // difference is not a defect, a missing or displaced corner is.
    let by_coord = |x: &[f64; 3], y: &[f64; 3]| x.partial_cmp(y).unwrap();
    actual.sort_by(by_coord);
    expected.sort_by(by_coord);
    assert_eq!(actual.len(), expected.len(), "obb.a vertex count");
    for (i, (got, want)) in actual.iter().zip(&expected).enumerate() {
        for k in 0..3 {
            assert_close(got[k], want[k], &format!("obb.a.vertex[{i}][{k}]"));
        }
    }

    let mut merged = a;
    merged.extend_approx(&c);
    assert_vec_close(
        merged.extents(),
        expect_vec3(&f, "obb.merged.extents"),
        "obb.merged.extents",
    );
    assert_vec_close(
        merged.pose().translation.vector,
        expect_vec3(&f, "obb.merged.origin"),
        "obb.merged.origin",
    );
}
