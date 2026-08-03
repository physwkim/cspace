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
//! `obb_predicates_match_libgeometric_shapes`'s pair is close enough
//! together that `OBB::extend_approx`'s general case only ever exercised
//! FCL's `merge_smalldist` branch; the `obb2.*` fixture keys and
//! `obb_extend_approx_merge_largedist_matches_libgeometric_shapes` below add
//! a far-apart pair that forces the previously-untested `merge_largedist`
//! branch, needed to close the disagreement documented in `bodies.rs`'s
//! module docs, deviation 3.
//!
//! `convexmesh.ray[0]`, enabled here for the first time, turned out not to
//! be a lone case: the shipped `.so`'s own `intersectsRay` and
//! `containsPoint` disagree with each other on `ray[0]`, `[2]` and `[4]`
//! (`bodies.rs`'s module docs, deviation 7). `convex_mesh_matches_libgeometric_shapes`
//! excludes those three from the generic comparison;
//! `convex_mesh_sign_bug_upstream_defect` below pins this port's own
//! (internally consistent) answers for them instead, with the topological
//! proof that the fixture's values for those rays cannot be right.
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
    ($f:expr, $name:literal, $body:expr) => {
        check_body!($f, $name, $body, skip_rays: [])
    };
    ($f:expr, $name:literal, $body:expr, skip_rays: [$($skip:expr),* $(,)?]) => {{
        let f = $f;
        let body = $body;
        let skip_rays: &[usize] = &[$($skip),*];

        for (i, p) in probe_points().iter().enumerate() {
            let key = format!("{}.contains[{}]", $name, i);
            let expected = expect_usize(&f, &key) == 1;
            assert_eq!(body.contains_point(p), expected, "{key}");
        }

        for (i, (origin, dir)) in probe_rays().iter().enumerate() {
            if skip_rays.contains(&i) {
                continue;
            }
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

/// `convexmesh.ray[0]`, `[2]` and `[4]` are excluded from the fixture
/// comparison: they are `bodies.rs`'s documented deviation 7, where the
/// shipped `.so`'s own `intersectsRay` and `containsPoint` contradict each
/// other (proved in the module docs), so the fixture's hit counts for these
/// three rays are upstream's bug, not ground truth to match. See
/// `convex_mesh_sign_bug_upstream_defect` below for the assertions this
/// port keeps in their place.
#[test]
fn convex_mesh_matches_libgeometric_shapes() {
    let f = fixture();
    let mut body = ConvexMesh::new(&tetrahedron()).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.15).unwrap();
    body.set_padding(0.01).unwrap();
    check_body!(f, "convexmesh", body, skip_rays: [0, 2, 4]);
}

/// The upstream sign-convention defect behind deviation 7, pinned against
/// this port's own (internally self-consistent) answers rather than the
/// fixture, on all three rays it manifests on. Each proof needs no
/// reference beyond the binary's own other outputs, already in the
/// fixture: a bounded body can only be entered and later passed (or, when
/// the ray starts inside, only ever left) after crossing its boundary a
/// parity-fixed number of times, and the fixture's own `containsPoint`
/// results pin which side of the boundary each relevant point is on.
///
/// - ray[0]: exterior origin, ray runs to a confirmed-interior point
///   (`convexmesh.contains[0]`) and beyond. A bounded region can only be
///   entered then exited around that point, so bracketing hits are
///   required; upstream reports 1 (unbracketed) hit.
/// - ray[2]: same interior point, orthogonal ray. Upstream reports 1 hit,
///   *after* the interior point on this ray's parametrization — meaning
///   upstream's own account has zero crossings before reaching a point its
///   own `containsPoint` calls interior, which is the same contradiction as
///   ray[0].
/// - ray[4]: origin *is* the confirmed-interior point itself
///   (`probe_points()[0]`), so escaping to infinity requires an odd number
///   of crossings. This port reports 1 (odd); upstream reports 2 (even).
///
/// The exact values pinned here are this port's own, hand-verified in the
/// investigation behind deviation 7 against the shipped `.so`'s real
/// (qhull-computed) hull data via `ConvexMesh`'s `getVertices`/
/// `getScaledVertices`/`getPlanes` accessors and an independently
/// recomputed ray-plane intersection — not merely "whatever this port
/// currently outputs" — so pinning them here is a real regression guard,
/// not a circular one.
#[test]
fn convex_mesh_sign_bug_upstream_defect() {
    let f = fixture();
    let mut body = ConvexMesh::new(&tetrahedron()).unwrap();
    body.set_pose(probe_pose());
    body.set_scale(1.15).unwrap();
    body.set_padding(0.01).unwrap();

    assert!(
        expect_usize(&f, "convexmesh.contains[0]") == 1,
        "sanity: the topological arguments need the fixture's own containsPoint(origin) to be true"
    );
    let origin_point = probe_points()[0];

    // ray[0]: bracket check on z.
    let (origin, dir) = probe_rays()[0];
    let hits = body.ray_intersections(&origin, &dir.normalize(), Some(2));
    assert_eq!(hits.len(), 2, "convexmesh.ray[0] (see deviation 7)");
    assert_close(hits[0].z, 1.0323458853008276, "convexmesh.ray[0].pt[0].z");
    assert_close(
        hits[1].z,
        1.160_120_379_681_655,
        "convexmesh.ray[0].pt[1].z",
    );
    assert!(
        hits[0].z < origin_point.z && origin_point.z < hits[1].z,
        "convexmesh.ray[0]: hits should bracket the confirmed-interior probe origin point"
    );

    // ray[2]: bracket check on y.
    let (origin, dir) = probe_rays()[2];
    let hits = body.ray_intersections(&origin, &dir.normalize(), Some(2));
    assert_eq!(hits.len(), 2, "convexmesh.ray[2] (see deviation 7)");
    assert_close(hits[0].y, -0.7475581529289892, "convexmesh.ray[2].pt[0].y");
    assert_close(hits[1].y, -0.5954610418532651, "convexmesh.ray[2].pt[1].y");
    assert!(
        hits[0].y < origin_point.y && origin_point.y < hits[1].y,
        "convexmesh.ray[2]: hits should bracket the confirmed-interior probe origin point"
    );

    // ray[4]: origin is the confirmed-interior point, so the forward ray
    // must leave the body an odd number of times.
    let (origin, dir) = probe_rays()[4];
    assert_vec_close(origin, origin_point, "convexmesh.ray[4] origin sanity");
    let hits = body.ray_intersections(&origin, &dir.normalize(), Some(2));
    assert_eq!(
        hits.len(),
        1,
        "convexmesh.ray[4] (see deviation 7): starting inside a bounded body requires an odd exit count"
    );
    assert_close(hits[0].x, 0.3596564673983505, "convexmesh.ray[4].pt[0].x");
    assert_close(hits[0].y, -0.6254294157520618, "convexmesh.ray[4].pt[0].y");
    assert_close(hits[0].z, 1.2148386997418248, "convexmesh.ray[4].pt[0].z");
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

/// `obb_predicates_match_libgeometric_shapes`'s pair is close enough
/// together that `OBB::extend_approx`'s general case only ever exercises
/// FCL's `merge_smalldist` branch (`center_diff.norm() ≈ 0.866 < 2*(0.5 +
/// 0.6) = 2.2`) — `merge_largedist`'s PCA fit (`getCovariance` +
/// `eigen_old` Jacobi + `getExtentAndCenter`) had no binary ground truth at
/// all until this case was added. This pair's centers are 4 units apart
/// against half-extent maxima of 0.5 each (`4 > 2*(0.5+0.5) = 2`), forcing
/// the large-distance branch.
#[test]
fn obb_extend_approx_merge_largedist_matches_libgeometric_shapes() {
    let f = fixture();
    let mut a = OBB::new(Isometry3::identity(), Vector3::new(1.0, 1.0, 1.0));

    let axis = nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0));
    let c = OBB::new(
        Isometry3::from_parts(
            Translation3::new(4.0, 0.3, -0.2),
            UnitQuaternion::from_axis_angle(&axis, 1.1),
        ),
        Vector3::new(0.6, 0.8, 1.0),
    );

    a.extend_approx(&c);
    assert_vec_close(
        a.extents(),
        expect_vec3(&f, "obb2.merged.extents"),
        "obb2.merged.extents",
    );
    assert_vec_close(
        a.pose().translation.vector,
        expect_vec3(&f, "obb2.merged.origin"),
        "obb2.merged.origin",
    );
}
