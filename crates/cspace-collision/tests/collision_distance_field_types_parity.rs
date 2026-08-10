// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision_distance_field_types` op.
//!
//! `test/test_collision_distance_field.cpp` has no case that exercises this
//! module without a `RobotModel` (see `collision_distance_field_types.rs`'s
//! own module doc), so none of it could be ported. This test is the only
//! verification this module's numeric core has: `BodyDecomposition`'s sphere
//! decomposition and bounding sphere, [`PosedDistanceField::distance_gradient`],
//! and [`PosedDistanceField::get_collision_sphere_gradients`], all compared
//! against the oracle for a Sphere, a Box, a Cylinder, and a Mesh at two
//! resolutions each (`tests/fixtures/collision_distance_field_types_request.json`
//! / `..._response.json`, ids 1-8).
//!
//! # `relative_cylinder_pose` is skipped for the two Sphere fixtures
//!
//! `collision_distance_field_types.rs`'s [`BodyDecomposition`] doc comment
//! documents an upstream defect discovered while capturing this fixture:
//! `BodyDecomposition::relative_cylinder_pose_` is a raw `Eigen::Isometry3d`
//! member upstream never initializes for a Sphere-only body (only the
//! cylinder/box/mesh branch of `determineCollisionSpheres` writes it). The
//! captured oracle response for both Sphere fixtures (ids 1 and 5) contains
//! genuinely uninitialized memory -- non-deterministic, non-reproducible
//! floating point garbage (e.g. `8.242212364724648e+115`) -- not a value
//! this port could ever match. This test asserts `relative_cylinder_pose`
//! only for the non-Sphere fixtures (ids 2, 3, 4, 6, 7, 8), where upstream's
//! cylinder branch does write a real, comparable value.
//!
//! # Tolerance
//!
//! Used uniformly here rather than `shape_points_parity.rs`'s coarser
//! grid-bucketing, because every value compared in this test (sphere
//! positions/radii, bounding sphere, gradients, collision-gradient outputs)
//! is a direct floating point number read off a fixed-order, deterministic
//! computation on both sides, not an unordered point cloud that needs set
//! comparison.
//!
//! This is a measured-margin tolerance, not an exactness assertion: unlike
//! `oracle_parity.rs`, this file's values pass through mesh-decomposition
//! geometry, which is not bit-exact between the two implementations.
//! Bisecting `TOL` directly against every `assert_relative_eq!` call in this
//! file (not a separate instrumentation harness) found `1e-16` fails --
//! `actual.gradient.x = 0.466025403784439` vs
//! `expected.gradient[0] = 0.4660254037844388` -- while `3e-16` and `1e-15`
//! both pass; the binding point sits between `1e-16` and `3e-16`, matching a
//! single ULP-scale gradient bucket. `TOL = 1e-12` keeps roughly four orders
//! of margin above that measured binding point. Re-bisected under
//! `float_roundtrip` (PORTING-PLAN.md §115) to check for the fixture-parsing
//! contamination that affected `collision_common_distance_field_parity.rs`'s
//! and `collision_env_distance_field_parity.rs`'s own constants: this file's
//! floor is unaffected -- `1e-16` still fails on the identical
//! `0.466025403784439` vs `0.4660254037844388` pair, so `TOL` is unchanged.
//!
//! `max_relative = TOL` is passed explicitly alongside `epsilon = TOL` at
//! every call below. Without it, `approx`'s `assert_relative_eq!` falls back
//! to `max_relative = f64::EPSILON` (~2.22e-16) whenever no `max_relative`
//! is given, which silently becomes the binding term for any `epsilon`
//! smaller than `largest_operand * f64::EPSILON` -- exactly how this file's
//! own bisection down toward `0.0` kept "passing" past the true binding
//! point during earlier measurement. Pinning `max_relative = TOL` removes
//! that hidden second tolerance so `TOL` alone is what a future bisection of
//! this file will measure.

use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use serde::Deserialize;

use cspace_collision::distance_field::BodyDecomposition;
use cspace_collision::distance_field::{
    CollisionType, DistanceField, GradientInfo, PosedDistanceField, SphereGradientQuery,
};
use cspace_core::geometry::{Cuboid, Cylinder, Isometry3, Mesh, Shape, Sphere};
use cspace_core::test_support::isometry_from_row_major;
use nalgebra::{Point3, Vector3};

/// Measured-margin tolerance (~4 orders above the bisected binding point,
/// `1e-16`..`3e-16`). See the module doc's "Tolerance" section.
const TOL: f64 = 1e-12;

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

    fn is_sphere(&self) -> bool {
        matches!(self, Self::Sphere { .. })
    }
}

#[derive(Deserialize)]
struct RequestGeometry {
    size: [f64; 3],
    origin: [f64; 3],
    resolution: f64,
}

#[derive(Deserialize)]
struct PosedFieldRequest {
    geometry: RequestGeometry,
    max_distance: f64,
    propagate_negative: bool,
    occupied_cells: Vec<[i32; 3]>,
    field_pose: [f64; 16],
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    shape: ShapeSpec,
    shape_pose: [f64; 16],
    resolution: f64,
    padding: f64,
    posed_field: PosedFieldRequest,
    gradient_queries: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct SphereDump {
    relative_vec: [f64; 3],
    radius: f64,
}

#[derive(Deserialize)]
struct BoundingSphereDump {
    center: [f64; 3],
    radius: f64,
}

#[derive(Deserialize)]
struct GradientDump {
    distance: f64,
    gradient: [f64; 3],
    in_bounds: bool,
}

#[derive(Deserialize)]
struct CollisionGradientDump {
    closest_distance: f64,
    collision: bool,
    distances: Vec<f64>,
    types: Vec<i32>,
    gradients: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct CdftDump {
    collision_spheres: Vec<SphereDump>,
    relative_cylinder_pose: [f64; 16],
    bounding_sphere: BoundingSphereDump,
    gradients: Vec<GradientDump>,
    collision_gradient: CollisionGradientDump,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    result: CdftDump,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/distance_field/{}"
        ),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("collision_distance_field_types_request.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse collision_distance_field_types_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("collision_distance_field_types_response.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse collision_distance_field_types_response.json: {e}"))
}

/// `pose * v` treating `v` as a point (Eigen's `Isometry3d * Vector3d`
/// semantics) -- see `transform_as_point`'s doc comment in
/// `collision_distance_field_types.rs` for why this is not the same as
/// nalgebra's `Isometry3 * Vector3`. Inlined here since the crate's own
/// helper is private.
fn transform_as_point(pose: &Isometry3, v: Vector3<f64>) -> Vector3<f64> {
    (pose * Point3::from(v)).coords
}

fn collision_type_from_i32(v: i32) -> CollisionType {
    match v {
        0 => CollisionType::None,
        1 => CollisionType::SelfCollision,
        2 => CollisionType::Intra,
        3 => CollisionType::Environment,
        other => panic!("unknown CollisionType discriminant {other}"),
    }
}

#[test]
fn collision_distance_field_types_match_the_oracle() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");
        let id = request.id;

        // -- BodyDecomposition: collision spheres and bounding sphere --

        let shape = request.shape.to_shape();
        let shape_pose = isometry_from_row_major(&request.shape_pose);
        let body_decomposition = Arc::new(
            BodyDecomposition::from_shapes(
                std::slice::from_ref(&shape),
                std::slice::from_ref(&shape_pose),
                request.resolution,
                request.padding,
            )
            .unwrap_or_else(|e| panic!("id {id}: BodyDecomposition::from_shapes: {e}")),
        );

        assert_eq!(
            body_decomposition.collision_spheres().len(),
            response.result.collision_spheres.len(),
            "id {id}: collision sphere count mismatch"
        );
        for (actual, expected) in body_decomposition
            .collision_spheres()
            .iter()
            .zip(&response.result.collision_spheres)
        {
            assert_relative_eq!(
                actual.relative_vec.x,
                expected.relative_vec[0],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.relative_vec.y,
                expected.relative_vec[1],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.relative_vec.z,
                expected.relative_vec[2],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.radius,
                expected.radius,
                epsilon = TOL,
                max_relative = TOL
            );
        }

        let actual_bounding = body_decomposition.relative_bounding_sphere();
        assert_relative_eq!(
            actual_bounding.center.x,
            response.result.bounding_sphere.center[0],
            epsilon = TOL,
            max_relative = TOL
        );
        assert_relative_eq!(
            actual_bounding.center.y,
            response.result.bounding_sphere.center[1],
            epsilon = TOL,
            max_relative = TOL
        );
        assert_relative_eq!(
            actual_bounding.center.z,
            response.result.bounding_sphere.center[2],
            epsilon = TOL,
            max_relative = TOL
        );
        assert_relative_eq!(
            actual_bounding.radius,
            response.result.bounding_sphere.radius,
            epsilon = TOL,
            max_relative = TOL
        );

        // `relative_cylinder_pose` is genuinely uninitialized memory upstream
        // for a Sphere body -- see this file's module doc.
        if !request.shape.is_sphere() {
            let actual_cyl = body_decomposition.relative_cylinder_pose();
            let expected_cyl = isometry_from_row_major(&response.result.relative_cylinder_pose);
            assert_relative_eq!(
                actual_cyl.translation.vector,
                expected_cyl.translation.vector,
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual_cyl.rotation.to_rotation_matrix().matrix(),
                expected_cyl.rotation.to_rotation_matrix().matrix(),
                epsilon = TOL,
                max_relative = TOL
            );
        }

        // -- PosedDistanceField: distance_gradient --

        let field = &request.posed_field;
        let mut posed_field = PosedDistanceField::new(
            Vector3::from(field.geometry.size),
            Vector3::from(field.geometry.origin),
            field.geometry.resolution,
            field.max_distance,
            field.propagate_negative,
        )
        .unwrap_or_else(|e| panic!("id {id}: PosedDistanceField::new: {e}"));

        let occupied_points: Vec<Vector3<f64>> = field
            .occupied_cells
            .iter()
            .map(|c| posed_field.field().grid_to_world(c[0], c[1], c[2]))
            .collect();
        posed_field
            .field_mut()
            .add_points_to_field(&occupied_points);
        let field_pose = isometry_from_row_major(&field.field_pose);
        posed_field.update_pose(field_pose);

        assert_eq!(
            request.gradient_queries.len(),
            response.result.gradients.len(),
            "id {id}: gradient query count mismatch"
        );
        for (query, expected) in request
            .gradient_queries
            .iter()
            .zip(&response.result.gradients)
        {
            let actual = posed_field.distance_gradient(query[0], query[1], query[2]);
            assert_relative_eq!(
                actual.distance,
                expected.distance,
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.gradient.x,
                expected.gradient[0],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.gradient.y,
                expected.gradient[1],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                actual.gradient.z,
                expected.gradient[2],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_eq!(
                actual.in_bounds, expected.in_bounds,
                "id {id}: in_bounds mismatch"
            );
        }

        // -- PosedDistanceField::get_collision_sphere_gradients --

        let sphere_centers: Vec<Vector3<f64>> = body_decomposition
            .collision_spheres()
            .iter()
            .map(|s| transform_as_point(&shape_pose, s.relative_vec))
            .collect();

        let n = body_decomposition.collision_spheres().len();
        let mut gradient = GradientInfo {
            distances: vec![f64::MAX; n],
            types: vec![CollisionType::None; n],
            gradients: vec![Vector3::zeros(); n],
            ..GradientInfo::default()
        };
        let collision = posed_field.get_collision_sphere_gradients(
            body_decomposition.collision_spheres(),
            &sphere_centers,
            &mut gradient,
            &SphereGradientQuery {
                collision_type: CollisionType::SelfCollision,
                tolerance: 0.0,
                subtract_radii: true,
                maximum_value: 1.0e6,
                stop_at_first_collision: false,
            },
        );

        assert_eq!(
            collision, response.result.collision_gradient.collision,
            "id {id}: collision mismatch"
        );
        assert_relative_eq!(
            gradient.closest_distance,
            response.result.collision_gradient.closest_distance,
            epsilon = TOL,
            max_relative = TOL
        );
        assert_eq!(
            gradient.distances.len(),
            response.result.collision_gradient.distances.len(),
            "id {id}: collision gradient length mismatch"
        );
        for i in 0..n {
            assert_relative_eq!(
                gradient.distances[i],
                response.result.collision_gradient.distances[i],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_eq!(
                gradient.types[i],
                collision_type_from_i32(response.result.collision_gradient.types[i]),
                "id {id}: collision gradient type mismatch at sphere {i}"
            );
            assert_relative_eq!(
                gradient.gradients[i].x,
                response.result.collision_gradient.gradients[i][0],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                gradient.gradients[i].y,
                response.result.collision_gradient.gradients[i][1],
                epsilon = TOL,
                max_relative = TOL
            );
            assert_relative_eq!(
                gradient.gradients[i].z,
                response.result.collision_gradient.gradients[i][2],
                epsilon = TOL,
                max_relative = TOL
            );
        }
    }
}
