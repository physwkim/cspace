// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`cspace_geometry::bodies::Body`]'s posed algorithms
//! (`containsPoint`, `intersectsRay`, `computeVolume`,
//! `computeBoundingSphere`, `computeBoundingCylinder`, `computeBoundingBox`
//! for both `AABB` and `OBB`) against the real moveit2 oracle's new
//! `body_query` op.
//!
//! `Body`'s posed algorithms were already ported in `bodies.rs` before this
//! round (see the module docs there for the per-type deviation list); this
//! test closes the oracle-verification gap the round-3 task flagged --
//! prior coverage (`probe_parity.rs`) replayed a standalone probe binary
//! linked against the shipped `libgeometric_shapes.so`, not the oracle's
//! JSON-line protocol.
//!
//! `tests/fixtures/body_query_{request,response}.json` is the request array
//! fed to the oracle's `body_query` op and its unedited response, captured
//! against `moveit-rs/oracle:round3-bodies`. Every ray in the fixture is a
//! deliberately chosen boundary case, not a narrative scenario. id 1
//! (`Sphere`) covers: a miss, a pass-through (both hits, then again with
//! `count: 1` to check the truncation contract), a ray exactly tangent to
//! the sphere (collapses to one point), a ray starting inside the body. id 2
//! (`Cylinder`, radius 0.5, length 2) covers: an axial ray through both end
//! caps (isolates the cap-plane branch, 2 hits at the two cap z-limits); a
//! ray from inside exiting through the curved side; a ray parallel to the
//! axis, offset from it by exactly the radius, at the cylinder's
//! mid-length (isolates the pure quadratic curved-side branch at its
//! zero-discriminant tangent point, collapsing to one point exactly like
//! the sphere's tangent case); a miss; and a ray parallel to the axis lying
//! exactly on the radius boundary through both caps (this one still takes
//! the cap-plane branch, since it is parallel to the axis, and exercises
//! that branch's own boundary tolerance at exactly the threshold -- it
//! reports the two cap z-limit points rather than collapsing to one). id 3
//! (`Cuboid`, 2x2x2) covers: a pass-through, a ray from inside, a miss, and
//! a ray running exactly along a shared box edge. id 4 (`Sphere`, radius 1,
//! scale 1.5, padding 0.1) checks the scale/padding interaction on every
//! bounding-surface computation and on `containsPoint` at the exact
//! padded/scaled boundary.
//!
//! This project has no `bodies::Cone` to port: upstream's
//! `createEmptyBodyFromShapeType` (`body_operations.cpp`) has no `CONE`
//! case (falls to `default:`, logs an error, returns `nullptr`; the caller
//! then unconditionally calls `setDimensions` on that null body). There is
//! nothing named "Cone" in the `bodies::` namespace to test here.
//!
//! That null dereference was recorded here as "moot, since nothing upstream
//! constructs a Cone body". That half is false and is corrected rather than
//! deleted, because the reasoning it stood on is the reusable part: an
//! absence claim about upstream needs the callers enumerated, not the
//! definition read. `kinematic_constraint.cpp:412-413` constructs a body
//! from every entry of `constraint_region.primitives`, which is a
//! `shape_msgs/SolidPrimitive[]`; `SolidPrimitive::CONE` is one of the four
//! types `constructShapeFromMsg` builds (`shape_operations.cpp:101-106`),
//! and nothing between the two filters it out. A `PositionConstraint`
//! carrying a cone region therefore dereferences null upstream. The port
//! returns `Ok(None)` from `Body::from_shape` and the caller turns that into
//! an error -- see `cspace-constraints`'
//! `new_rejects_a_shape_with_no_body_counterpart`.

use std::fs;

use cspace_geometry::bodies::Body;
use cspace_geometry::{Shape, Vector3};
use cspace_test_support::isometry_from_row_major;
use serde::Deserialize;

const LINEAR_EPS: f64 = 1e-9;
const VOLUME_EPS: f64 = 1e-9;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ShapeSpec {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
    Box { size: [f64; 3] },
}

#[derive(Deserialize)]
struct RaySpec {
    origin: [f64; 3],
    dir: [f64; 3],
    count: Option<usize>,
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    shape: ShapeSpec,
    pose: [f64; 16],
    #[serde(default = "default_scale")]
    scale: f64,
    #[serde(default)]
    padding: f64,
    points: Vec<[f64; 3]>,
    rays: Vec<RaySpec>,
}

fn default_scale() -> f64 {
    1.0
}

#[derive(Deserialize)]
struct AabbResult {
    min: [f64; 3],
    max: [f64; 3],
}

#[derive(Deserialize)]
struct BsphereResult {
    center: [f64; 3],
    radius: f64,
}

#[derive(Deserialize)]
struct BcylResult {
    origin: [f64; 3],
    radius: f64,
    length: f64,
}

#[derive(Deserialize)]
struct ObbResult {
    origin: [f64; 3],
    extents: [f64; 3],
}

#[derive(Deserialize)]
struct RayResult {
    hit: bool,
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct BodyQueryResult {
    contains: Vec<bool>,
    rays: Vec<RayResult>,
    volume: f64,
    bsphere: BsphereResult,
    bcyl: BcylResult,
    aabb: AabbResult,
    obb: ObbResult,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    ok: bool,
    result: BodyQueryResult,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("body_query_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse body_query_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("body_query_response.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse body_query_response.json: {e}"))
}

fn assert_close(actual: f64, expected: f64, eps: f64, ctx: &str) {
    assert!(
        (actual - expected).abs() < eps,
        "{ctx}: {actual} vs oracle {expected}"
    );
}

fn assert_vec_close(actual: &Vector3, expected: &[f64; 3], eps: f64, ctx: &str) {
    assert_close(actual.x, expected[0], eps, &format!("{ctx}.x"));
    assert_close(actual.y, expected[1], eps, &format!("{ctx}.y"));
    assert_close(actual.z, expected[2], eps, &format!("{ctx}.z"));
}

#[test]
fn body_posed_algorithms_match_the_oracle() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        let ctx = format!("id {}", request.id);
        assert_eq!(
            request.id, response.id,
            "{ctx}: request/response id mismatch"
        );
        assert!(response.ok, "{ctx}: oracle reported ok=false");

        let shape = match &request.shape {
            ShapeSpec::Sphere { radius } => {
                Shape::Sphere(cspace_geometry::shapes::Sphere { radius: *radius })
            }
            ShapeSpec::Cylinder { radius, length } => {
                Shape::Cylinder(cspace_geometry::shapes::Cylinder {
                    radius: *radius,
                    length: *length,
                })
            }
            ShapeSpec::Box { size } => {
                Shape::Cuboid(cspace_geometry::shapes::Cuboid { size: *size })
            }
        };
        let mut body = Body::from_shape(&shape)
            .unwrap_or_else(|e| panic!("{ctx}: from_shape failed: {e}"))
            .unwrap_or_else(|| panic!("{ctx}: from_shape returned None for a supported shape"));

        body.set_pose(isometry_from_row_major(&request.pose));
        body.set_scale(request.scale)
            .unwrap_or_else(|e| panic!("{ctx}: set_scale failed: {e}"));
        body.set_padding(request.padding)
            .unwrap_or_else(|e| panic!("{ctx}: set_padding failed: {e}"));

        assert_eq!(
            request.points.len(),
            response.result.contains.len(),
            "{ctx}: points/contains count mismatch"
        );
        for (point, expected) in request.points.iter().zip(&response.result.contains) {
            let actual = body.contains_point(&Vector3::new(point[0], point[1], point[2]));
            assert_eq!(actual, *expected, "{ctx}: contains_point({point:?})");
        }

        assert_eq!(
            request.rays.len(),
            response.result.rays.len(),
            "{ctx}: rays/results count mismatch"
        );
        for (i, (ray, expected)) in request.rays.iter().zip(&response.result.rays).enumerate() {
            let ray_ctx = format!("{ctx} ray {i}");
            let origin = Vector3::new(ray.origin[0], ray.origin[1], ray.origin[2]);
            let dir = Vector3::new(ray.dir[0], ray.dir[1], ray.dir[2]);
            let points = body.ray_intersections(&origin, &dir, ray.count);
            assert_eq!(!points.is_empty(), expected.hit, "{ray_ctx}: hit mismatch");
            assert_eq!(
                points.len(),
                expected.points.len(),
                "{ray_ctx}: point count mismatch (got {points:?}, oracle {:?})",
                expected.points
            );
            for (j, (actual, expected_pt)) in points.iter().zip(&expected.points).enumerate() {
                assert_vec_close(
                    actual,
                    expected_pt,
                    LINEAR_EPS,
                    &format!("{ray_ctx} point {j}"),
                );
            }
        }

        assert_close(
            body.compute_volume(),
            response.result.volume,
            VOLUME_EPS,
            &format!("{ctx} volume"),
        );

        let bsphere = body.compute_bounding_sphere();
        assert_vec_close(
            &bsphere.center,
            &response.result.bsphere.center,
            LINEAR_EPS,
            &format!("{ctx} bsphere.center"),
        );
        assert_close(
            bsphere.radius,
            response.result.bsphere.radius,
            LINEAR_EPS,
            &format!("{ctx} bsphere.radius"),
        );

        let bcyl = body.compute_bounding_cylinder();
        assert_vec_close(
            &bcyl.pose.translation.vector,
            &response.result.bcyl.origin,
            LINEAR_EPS,
            &format!("{ctx} bcyl.origin"),
        );
        assert_close(
            bcyl.radius,
            response.result.bcyl.radius,
            LINEAR_EPS,
            &format!("{ctx} bcyl.radius"),
        );
        assert_close(
            bcyl.length,
            response.result.bcyl.length,
            LINEAR_EPS,
            &format!("{ctx} bcyl.length"),
        );

        let aabb = body.compute_bounding_aabb();
        assert_vec_close(
            &aabb.min(),
            &response.result.aabb.min,
            LINEAR_EPS,
            &format!("{ctx} aabb.min"),
        );
        assert_vec_close(
            &aabb.max(),
            &response.result.aabb.max,
            LINEAR_EPS,
            &format!("{ctx} aabb.max"),
        );

        let obb = body.compute_bounding_obb();
        assert_vec_close(
            &obb.pose().translation.vector,
            &response.result.obb.origin,
            LINEAR_EPS,
            &format!("{ctx} obb.origin"),
        );
        assert_vec_close(
            &obb.extents(),
            &response.result.obb.extents,
            LINEAR_EPS,
            &format!("{ctx} obb.extents"),
        );
    }
}
