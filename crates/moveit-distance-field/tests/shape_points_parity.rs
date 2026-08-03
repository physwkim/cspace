// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `shape_points` op.
//!
//! `oracle_parity.rs` covers `PropagationDistanceField` from an explicit
//! `occupied_cells` grid-coordinate list, so it never exercises the
//! shape-to-obstacle-points step at all: [`find_internal_points_convex`] --
//! upstream `distance_field::findInternalPointsConvex`, called from
//! [`moveit_distance_field::DistanceField::add_shape_to_field`] and its
//! siblings -- was unverified for every shape. This test closes that gap.
//!
//! Both sides are driven from `tests/fixtures/shape_points_request.json`, an
//! array of `shape_points` op requests, paired with the oracle's own
//! unedited `shape_points_response.json`: one shape (Sphere, Box, Cylinder,
//! Mesh -- the four `bodies::` has a case for), one pose with both a
//! translation and a non-trivial rotation (not identity), at two different
//! resolutions each.
//!
//! Upstream builds the `bodies::Body` for `findInternalPointsConvex` via
//! `createEmptyBodyFromShapeType` + `setDimensionsDirty` + `setPoseDirty` +
//! `updateInternalData` (see `oracle.cpp`'s `shapePoints`); this test builds
//! the equivalent [`moveit_geometry::bodies::Body`] the same way this
//! crate's own `posed_body` (in `distance_field.rs`) does --
//! `Body::from_shape` + `set_pose` -- so what is under test is this crate's
//! actual production construction path, not a parallel reimplementation of
//! it.
//!
//! Points are compared as sets, not order-sensitive lists: upstream's own
//! triple-nested loop and this port's mirror of it enumerate the same
//! sampling grid in the same order, but the pose on each side reaches
//! [`nalgebra::Isometry3`] by a different route than
//! `Eigen::Isometry3d::matrix() = m` (nalgebra stores rotation as a
//! [`nalgebra::UnitQuaternion`], so a general rotation matrix is
//! reconstructed through a quaternion round-trip -- see
//! `isometry_from_row_major` below), which can perturb a coordinate by a few
//! ULPs relative to the oracle's direct matrix assignment. Comparing the
//! coordinates gridded onto [`POINT_EPS`] absorbs that expected noise while
//! still catching a real disagreement: a genuine off-by-one grid defect
//! moves a point by a whole `resolution`, many orders of magnitude past
//! [`POINT_EPS`]. The comparison is exact set equality on those gridded
//! keys, so a missing or extra point is reported as such rather than
//! being matched to a near neighbour.
//!
//! Unlike this crate's other parity tests' `TOL`/`DISTANCE_TOL`, a coarser
//! [`POINT_EPS`] is the safe direction here (it can only turn a real defect
//! into a false pass at a scale within a few ULPs of that defect, never
//! hide a bigger one), so there is nothing to tighten. The noise itself was
//! measured directly, by zipping this port's and the oracle's point arrays
//! by index (both sides enumerate the same sampling grid in the same
//! order, so this needs no set-matching): `5.55e-17` absolute / `1.78e-16`
//! relative across every fixture case, i.e. [`POINT_EPS`] is roughly ten
//! orders of magnitude coarser than the measured noise, not the six this
//! comment previously estimated without measuring.

use std::collections::HashSet;
use std::fs;

use serde::Deserialize;

use moveit_distance_field::find_internal_points_convex;
use moveit_geometry::bodies::Body;
use moveit_geometry::{Cuboid, Cylinder, Isometry3, Mesh, Shape, Sphere};
use nalgebra::{Matrix3, Translation3, UnitQuaternion, Vector3};

/// Kind: a structurally-safe grid-bucket size, not a measured-margin
/// tolerance like this crate's other parity files' `TOL`/`DISTANCE_TOL`/
/// `RADIUS_TOL` constants. Those are sized just above a bisected failure
/// point, so shrinking one toward that point is what re-opens a gate;
/// `POINT_EPS` has no such floor to re-litigate -- coarser is always the
/// safe direction here (see below), so there is nothing to bisect down to.
///
/// Grid the coordinates onto this spacing before comparing, so the pose's
/// quaternion round-trip (see the module docs) does not read as a
/// disagreement. Well below the coarsest resolution used here (0.05) and
/// well above that noise.
///
/// This buckets rather than compares within a tolerance, so two points a few
/// ULPs apart that happen to straddle a bucket edge land in different
/// buckets and read as a mismatch. That direction is the safe one — it can
/// cost a false failure, never a false pass — and it is why this constant is
/// roughly ten orders of magnitude coarser than the measured noise it
/// absorbs (see the module doc) rather than merely larger than it.
const POINT_EPS: f64 = 1e-6;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ShapeSpec {
    Sphere {
        radius: f64,
    },
    Box {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Mesh {
        vertices: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
    },
}

impl ShapeSpec {
    fn to_shape(&self) -> Shape {
        match self {
            Self::Sphere { radius } => Shape::Sphere(Sphere::new(*radius).unwrap()),
            Self::Box { size } => Shape::Cuboid(Cuboid::new(size[0], size[1], size[2]).unwrap()),
            Self::Cylinder { radius, length } => {
                Shape::Cylinder(Cylinder::new(*radius, *length).unwrap())
            }
            Self::Mesh {
                vertices,
                triangles,
            } => {
                let vertices = vertices.iter().map(|&v| Vector3::from(v)).collect();
                let triangles = triangles.clone();
                Shape::Mesh(Mesh::new(vertices, triangles).unwrap())
            }
        }
    }
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    shape: ShapeSpec,
    pose: [f64; 16],
    resolution: f64,
}

#[derive(Deserialize)]
struct ShapePointsDump {
    points: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    result: ShapePointsDump,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("shape_points_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse shape_points_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("shape_points_response.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse shape_points_response.json: {e}"))
}

/// Row-major 4x4, matching `toRowMajor4x4`/`fromRowMajor4x4` in `oracle.cpp`.
fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn point_key(p: [f64; 3]) -> [i64; 3] {
    [
        (p[0] / POINT_EPS).round() as i64,
        (p[1] / POINT_EPS).round() as i64,
        (p[2] / POINT_EPS).round() as i64,
    ]
}

#[test]
fn find_internal_points_convex_matches_the_oracle_for_every_shape() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");

        let shape = request.shape.to_shape();
        let pose = isometry_from_row_major(&request.pose);
        let mut body = Body::from_shape(&shape)
            .unwrap_or_else(|e| panic!("id {}: Body::from_shape: {e}", request.id))
            .unwrap_or_else(|| panic!("id {}: shape has no bodies:: counterpart", request.id));
        body.set_pose(pose);

        let mut actual_points = Vec::new();
        find_internal_points_convex(&body, request.resolution, &mut actual_points);

        let actual: HashSet<[i64; 3]> = actual_points
            .iter()
            .map(|p| point_key([p.x, p.y, p.z]))
            .collect();
        let expected: HashSet<[i64; 3]> = response
            .result
            .points
            .iter()
            .copied()
            .map(point_key)
            .collect();

        let missing: Vec<_> = expected.difference(&actual).collect();
        let extra: Vec<_> = actual.difference(&expected).collect();

        assert!(
            missing.is_empty() && extra.is_empty(),
            "id {}: point sets disagree -- {} cells the oracle found that this port did not: {:?}; \
             {} cells this port found that the oracle did not: {:?}",
            request.id,
            missing.len(),
            missing,
            extra.len(),
            extra
        );
    }
}
