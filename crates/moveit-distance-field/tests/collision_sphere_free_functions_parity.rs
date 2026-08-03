// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `collision_sphere_free_functions`
//! op.
//!
//! `collision_distance_field_types_parity.rs` exercises
//! [`PosedDistanceField::get_collision_sphere_gradients`] (the *member*
//! overload) but nothing else in this crate ever called, or tested against
//! the oracle, the free [`get_collision_sphere_gradients`] function or
//! either [`get_collision_sphere_collision`]/[`get_collision_sphere_collisions`]
//! overload -- confirmed by grep, zero references beyond doc comments and
//! `pub use` re-exports. Upstream's own gtest suite
//! (`test/test_collision_distance_field.cpp`) never calls them directly
//! either, only indirectly through `CollisionEnvDistanceField`, so there was
//! no case to port. This test is the first verification any of these three
//! symbols has ever received against upstream ground truth.
//!
//! # Why the free function needs its own fixture, not a shared one
//!
//! The member and free `getCollisionSphereGradients` overloads disagree with
//! each other on two points, both documented on
//! [`PosedDistanceField::get_collision_sphere_gradients`]'s "Deviation from
//! upstream" doc: the out-of-bounds threshold (`grad.norm() > 0` on the
//! member vs `grad.norm() > EPSILON` on the free function) and whether
//! `dist` is `abs()`-ed after subtracting the sphere radius (yes on the
//! member, no on the free function). A fixture built for the member proves
//! nothing about the free function's own arithmetic, so this test drives
//! the free function directly against a bare
//! [`PropagationDistanceField`]/`dyn DistanceField`, with no
//! [`PosedDistanceField`]/pose/`BodyDecomposition` involved -- matching
//! upstream's own free-function signature, which takes a raw
//! `const distance_field::DistanceField*` and a sphere list already in the
//! field's own frame.
//!
//! # Fixture design
//!
//! `tests/fixtures/collision_sphere_free_functions_request.json` (ids 1-5)
//! is a hand-built, not captured-from-a-larger-scenario, case set, chosen to
//! cover every *reachable* branch in
//! [`get_collision_sphere_collision`]/[`get_collision_sphere_collisions`]:
//!
//! - **Not covered, and not coverable through this API**: the
//!   `!in_bounds && grad.norm() > threshold` early-return guard at the top
//!   of every function in this family. See
//!   [`PosedDistanceField::get_collision_sphere_gradients`]'s "Decision" doc
//!   section -- [`DistanceField::distance_gradient`] zeroes the gradient on
//!   every out-of-bounds return, so the guard can never be true regardless
//!   of which of the two thresholds this family uses, and no fixture can
//!   exercise it. The five cases below accordingly cover every branch
//!   *after* that guard, not literally "every branch" in the function.
//!
//! - id 1: one sphere penetrating an occupied cell (`subtract_radii = true`,
//!   negative post-subtraction `dist`), `num_coll = 0` -- exercises
//!   `getCollisionSphereCollision`'s "`num_coll == 0` means report the first
//!   collision without collecting" immediate-return branch (`colls` stays
//!   empty even though a collision was found).
//! - id 2: three colliding spheres, `num_coll = 2` -- exercises the
//!   `colls.len() >= num_coll` early-return branch (`colls` has exactly 2
//!   entries, the third colliding sphere is never reached).
//! - id 3: the same geometry as id 2, `num_coll = 10` -- exercises the
//!   loop-completes-normally, `!colls.is_empty()` branch (all 3 colliding
//!   indices collected).
//! - id 4: `subtract_radii = false`, one sphere colliding via
//!   `sphere.radius - dist > tolerance`, `num_coll = 1` -- exercises the
//!   `subtract_radii = false` branch (untested by any other fixture in this
//!   crate) and the `colls.len() >= num_coll` branch triggering on the very
//!   first collision.
//! - id 5: no sphere collides anywhere -- every sphere is at or beyond the
//!   field's own `max_distance` (`0.3`, chosen equal to `maximum_value` so
//!   the query gate `dist < maximum_value` excludes all three; one sphere
//!   additionally falls in the grid's invalid-gradient margin, `in_bounds =
//!   false`, but reports the same `max_distance`-capped `dist` as the
//!   in-bounds ones, not a distinct sentinel) -- exercises the
//!   all-`false`/empty-`colls` end of every branch above.
//!
//! # Tolerance
//!
//! A uniform epsilon is correct here for the same reason as in
//! `collision_distance_field_types_parity.rs`'s module doc: every value
//! compared is a direct, deterministic floating point read on both sides,
//! not an unordered point cloud. The value, `1e-9`, is measured: every
//! quantity this test compares is a raw [`PropagationDistanceField`]
//! wavefront result with no mesh-decomposition step upstream on either
//! side, and temporary instrumentation over every fixture case found them
//! bit-exact between this port and the oracle (`max_abs = 0.0`, `max_rel =
//! 0.0`) -- the same result `oracle_parity.rs`'s `DISTANCE_TOL` measured for
//! the same class of computation. This file used to pin `1e-4`, matching a
//! neighbouring file rather than anything measured here.

use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_distance_field::{
    CollisionSphere, CollisionType, DistanceField, GradientInfo, GridGeometry,
    PropagationDistanceField, SphereGradientQuery, get_collision_sphere_collision,
    get_collision_sphere_collisions, get_collision_sphere_gradients,
};
use nalgebra::Vector3;

/// `1e-9`, measured -- see the module doc's "Tolerance" section.
const TOL: f64 = 1e-9;

#[derive(Deserialize)]
struct RequestGeometry {
    size: [f64; 3],
    origin: [f64; 3],
    resolution: f64,
}

#[derive(Deserialize)]
struct SphereSpec {
    center: [f64; 3],
    radius: f64,
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    geometry: RequestGeometry,
    max_distance: f64,
    propagate_negative: bool,
    occupied_cells: Vec<[i32; 3]>,
    spheres: Vec<SphereSpec>,
    maximum_value: f64,
    tolerance: f64,
    subtract_radii: bool,
    num_coll: u32,
}

#[derive(Deserialize)]
struct GradientsDump {
    closest_distance: f64,
    collision: bool,
    distances: Vec<f64>,
    types: Vec<i32>,
    gradients: Vec<[f64; 3]>,
}

#[derive(Deserialize)]
struct CollisionWithLimitDump {
    collision: bool,
    colls: Vec<u32>,
}

#[derive(Deserialize)]
struct ResultDump {
    gradients: GradientsDump,
    collision_bool: bool,
    collision_with_limit: CollisionWithLimitDump,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    result: ResultDump,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("collision_sphere_free_functions_request.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse collision_sphere_free_functions_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("collision_sphere_free_functions_response.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse collision_sphere_free_functions_response.json: {e}"))
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
fn collision_sphere_free_functions_match_the_oracle() {
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

        let geometry = GridGeometry::new(
            Vector3::from(request.geometry.size),
            Vector3::from(request.geometry.origin),
            request.geometry.resolution,
        )
        .unwrap_or_else(|e| panic!("id {id}: GridGeometry::new: {e}"));
        let mut field = PropagationDistanceField::new(
            geometry,
            request.max_distance,
            request.propagate_negative,
        )
        .unwrap_or_else(|e| panic!("id {id}: PropagationDistanceField::new: {e}"));

        let occupied_points: Vec<Vector3<f64>> = request
            .occupied_cells
            .iter()
            .map(|c| field.grid_to_world(c[0], c[1], c[2]))
            .collect();
        field.add_points_to_field(&occupied_points);

        let sphere_list: Vec<CollisionSphere> = request
            .spheres
            .iter()
            .map(|s| CollisionSphere::new(Vector3::from(s.center), s.radius))
            .collect();
        let sphere_centers: Vec<Vector3<f64>> = request
            .spheres
            .iter()
            .map(|s| Vector3::from(s.center))
            .collect();

        let n = sphere_list.len();
        let query = SphereGradientQuery {
            collision_type: CollisionType::SelfCollision,
            tolerance: request.tolerance,
            subtract_radii: request.subtract_radii,
            maximum_value: request.maximum_value,
            stop_at_first_collision: false,
        };

        // -- free `get_collision_sphere_gradients` --

        let mut gradient = GradientInfo {
            distances: vec![f64::MAX; n],
            types: vec![CollisionType::None; n],
            gradients: vec![Vector3::zeros(); n],
            ..GradientInfo::default()
        };
        let gradients_collision = get_collision_sphere_gradients(
            &field as &dyn DistanceField,
            &sphere_list,
            &sphere_centers,
            &mut gradient,
            &query,
        );

        assert_eq!(
            gradients_collision, response.result.gradients.collision,
            "id {id}: gradients collision mismatch"
        );
        assert_relative_eq!(
            gradient.closest_distance,
            response.result.gradients.closest_distance,
            epsilon = TOL
        );
        assert_eq!(
            gradient.distances.len(),
            response.result.gradients.distances.len(),
            "id {id}: gradients length mismatch"
        );
        for i in 0..n {
            assert_relative_eq!(
                gradient.distances[i],
                response.result.gradients.distances[i],
                epsilon = TOL
            );
            assert_eq!(
                gradient.types[i],
                collision_type_from_i32(response.result.gradients.types[i]),
                "id {id}: gradients type mismatch at sphere {i}"
            );
            assert_relative_eq!(
                gradient.gradients[i].x,
                response.result.gradients.gradients[i][0],
                epsilon = TOL
            );
            assert_relative_eq!(
                gradient.gradients[i].y,
                response.result.gradients.gradients[i][1],
                epsilon = TOL
            );
            assert_relative_eq!(
                gradient.gradients[i].z,
                response.result.gradients.gradients[i][2],
                epsilon = TOL
            );
        }

        // -- `get_collision_sphere_collision` (bool-only overload) --

        let collision_bool = get_collision_sphere_collision(
            &field as &dyn DistanceField,
            &sphere_list,
            &sphere_centers,
            request.maximum_value,
            request.tolerance,
        );
        assert_eq!(
            collision_bool, response.result.collision_bool,
            "id {id}: collision_bool mismatch"
        );

        // -- `get_collision_sphere_collisions` (num_coll/colls overload) --

        let mut colls = Vec::new();
        let collision_with_limit = get_collision_sphere_collisions(
            &field as &dyn DistanceField,
            &sphere_list,
            &sphere_centers,
            request.maximum_value,
            request.tolerance,
            request.num_coll,
            &mut colls,
        );
        assert_eq!(
            collision_with_limit, response.result.collision_with_limit.collision,
            "id {id}: collision_with_limit mismatch"
        );
        assert_eq!(
            colls, response.result.collision_with_limit.colls,
            "id {id}: colls mismatch"
        );
    }
}
