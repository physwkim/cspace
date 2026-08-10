// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `distance_field` op.
//!
//! `PropagationDistanceField` has no `RobotModel` dependency, so — like
//! `world_parity.rs` in `cspace-collision` — both sides of this test are
//! driven from a committed request fixture rather than a hand-derived
//! expectation: `fixtures/distance_field_request.json` /
//! `fixtures/distance_field_negative_request.json` (identical scenario,
//! `propagate_negative` false and true respectively), each paired with the
//! oracle's own unedited response. There is exactly one copy of "which
//! geometry, which occupied cells, which queries" in the tree.
//!
//! The scenario is designed to hit every boundary this crate documents:
//! cells inside the occupied region, a free interior cell within
//! `max_distance` of an obstacle, all six grid faces (near-obstacle and
//! far-from-obstacle variants), points outside the grid on one axis, all
//! axes, and far outside, and — at the far-face queries — the
//! `PropDistanceFieldVoxel::UNINITIALIZED` (`-1, -1, -1`) sentinel path in
//! `getNearestCell`: a cell farther than `max_distance` from every obstacle
//! is never visited by propagation, so upstream reads
//! `voxel_grid_->getCell(-1, -1, -1)` unguarded and reports a non-null
//! neighbor at that position (`voxel_present: true`, `position: [-1, -1,
//! -1]` in the fixture) where this crate's
//! [`PropagationDistanceField::nearest_cell`] instead validates the position,
//! reports `voxel: None`, and falls the reported `position` back to the
//! queried cell — see that method's own deviation doc. This test asserts the
//! *oracle's* fields reflect that real upstream defect, and that this port's
//! `nearest_cell` disagrees with it in exactly that documented way, rather
//! than re-deriving the claim from a reading of the C++ source.
//!
//! # Exactness
//!
//! PORTING-PLAN.md §5 Phase 3 names `1e-4` as the distance tolerance, but
//! that is a policy floor, not a measurement, and this file used to pin it
//! as a `DISTANCE_TOL` constant on that basis alone. Every value compared
//! below -- `grid_to_world`, `distance`, `distance_cell`, `distance_gradient`,
//! `nearest_cell` -- is a raw [`PropagationDistanceField`] wavefront result,
//! with no mesh-decomposition or FK step upstream of it on either side: grid
//! coordinates are small integers (exact in `f64`), squared distances are
//! sums of their squares (exact, no rounding at these magnitudes), and the
//! one `sqrt` per query is IEEE-754-correctly-rounded on both sides. There
//! is no accumulation-order step for the two implementations to disagree
//! about.
//!
//! That is a structural argument, not just an empirical one, but it was
//! checked empirically too: bisecting a shared epsilon from `1e-9` down to
//! `0.0` for every comparison in [`check_scenario`], across both fixtures,
//! never produced a single failure. A constant nothing can violate is not a
//! gate, so this file compares with plain `assert_eq!` instead of
//! `assert_relative_eq!` -- if a future change to either side's arithmetic
//! ever introduces real drift (say, from FMA contraction differing between
//! the two toolchains), that is exactly the kind of regression an exact
//! comparison catches and a loose tolerance would have hidden.

use std::fs;

use serde::Deserialize;

use cspace_collision::distance_field::{DistanceField, GridGeometry, PropagationDistanceField};
use nalgebra::Vector3;

#[derive(Deserialize)]
struct RequestGeometry {
    size: [f64; 3],
    origin: [f64; 3],
    resolution: f64,
}

#[derive(Deserialize)]
struct RequestFixture {
    geometry: RequestGeometry,
    max_distance: f64,
    propagate_negative: bool,
    occupied_cells: Vec<[i32; 3]>,
    queries: Vec<[i32; 3]>,
}

#[derive(Deserialize)]
struct GradientDump {
    distance: f64,
    gradient: [f64; 3],
    in_bounds: bool,
}

#[derive(Deserialize)]
struct NearestDump {
    distance: f64,
    position: [i32; 3],
    voxel_present: bool,
}

#[derive(Deserialize)]
struct QueryDump {
    cell: [i32; 3],
    in_grid: bool,
    world: [f64; 3],
    distance_world: f64,
    #[serde(default)]
    distance_cell: Option<f64>,
    gradient: GradientDump,
    #[serde(default)]
    nearest: Option<NearestDump>,
}

#[derive(Deserialize)]
struct DistanceFieldDump {
    queries: Vec<QueryDump>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: DistanceFieldDump,
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

fn load_request(name: &str) -> RequestFixture {
    let raw = read_fixture(name);
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {name}: {e}"))
}

fn load_response(name: &str) -> DistanceFieldDump {
    let raw = read_fixture(name);
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {name}: {e}"));
    response.result
}

/// Build the field and seed `occupied_cells`, the single source this test
/// and the committed oracle request share — mirroring `world_parity.rs`'s
/// `build_world`.
fn build_field(request: &RequestFixture) -> PropagationDistanceField {
    let geometry = GridGeometry::new(
        Vector3::from(request.geometry.size),
        Vector3::from(request.geometry.origin),
        request.geometry.resolution,
    )
    .expect("fixture geometry must be valid");
    let mut field =
        PropagationDistanceField::new(geometry, request.max_distance, request.propagate_negative)
            .expect("fixture max_distance/resolution must be valid");

    let points: Vec<Vector3<f64>> = request
        .occupied_cells
        .iter()
        .map(|&[x, y, z]| field.grid_to_world(x, y, z))
        .collect();
    field.add_points_to_field(&points);
    field
}

fn check_scenario(request_name: &str, response_name: &str) {
    let request = load_request(request_name);
    let field = build_field(&request);
    let fixture = load_response(response_name);

    assert_eq!(
        request.queries.len(),
        fixture.queries.len(),
        "{request_name}/{response_name}: query count mismatch"
    );

    for (query, dump) in request.queries.iter().zip(&fixture.queries) {
        let [x, y, z] = *query;
        assert_eq!(
            dump.cell, *query,
            "{response_name}: cell echo for {query:?}"
        );

        let in_grid = field.is_cell_valid(x, y, z);
        assert_eq!(
            in_grid, dump.in_grid,
            "{response_name}: in_grid for {query:?}"
        );

        // Bit-exact, not merely close: bisecting `assert_relative_eq!`'s
        // `epsilon` down to `0.0` for every comparison in this function
        // still passes on both fixtures (see the module doc's "Exactness"
        // section). These `assert_eq!`s are that finding, not a stronger
        // check bolted on afterward.
        let world = field.grid_to_world(x, y, z);
        assert_eq!(world.x, dump.world[0]);
        assert_eq!(world.y, dump.world[1]);
        assert_eq!(world.z, dump.world[2]);

        let distance_world = field.distance(world.x, world.y, world.z);
        assert_eq!(distance_world, dump.distance_world);

        match dump.distance_cell {
            Some(expected) => {
                let actual = field.distance_cell(x, y, z);
                assert_eq!(actual, expected);
            }
            None => assert!(
                !in_grid,
                "{response_name}: oracle omitted distance_cell for an in-grid query {query:?}"
            ),
        }

        let gradient = field.distance_gradient(world.x, world.y, world.z);
        assert_eq!(
            gradient.in_bounds, dump.gradient.in_bounds,
            "{response_name}: gradient.in_bounds for {query:?}"
        );
        assert_eq!(gradient.distance, dump.gradient.distance);
        assert_eq!(gradient.gradient, Vector3::from(dump.gradient.gradient));

        match &dump.nearest {
            Some(expected) => {
                let actual = field.nearest_cell(x, y, z);
                assert_eq!(actual.distance, expected.distance);
                if expected.position == [-1, -1, -1] {
                    // The documented deviation (see `nearest_cell`'s
                    // "Deviations from upstream"): a cell farther than
                    // `max_distance` from every obstacle is never visited by
                    // propagation, so its `closest_point` stays at the
                    // UNINITIALIZED sentinel. Upstream's raw pointer
                    // comparison forms a well-defined-but-never-dereferenced
                    // out-of-bounds pointer there and reports it as *present*
                    // at that sentinel position — failing its own doc's "if
                    // nearest cell is unknown, return nullptr" contract. This
                    // port validates the position first and reports `voxel:
                    // None` with `position` falling back to the queried
                    // cell, matching upstream's own (correctly-implemented)
                    // third branch for "nearest cell is unknown". Both sides
                    // are asserted explicitly, from the oracle's own run, not
                    // from this test assuming its own conclusion.
                    assert!(
                        expected.voxel_present,
                        "{response_name}: fixture must show upstream reporting the sentinel neighbor as present for {query:?}"
                    );
                    // ASSERTION-DISCRIMINATION AUDIT (round 2): `nearest_cell`
                    // (propagation.rs) has five `voxel: None`-producing return
                    // sites, not one -- `:351`/`:368`/`:381` return `None`
                    // explicitly, and `:358`/`:375`'s
                    // `(pos != queried).then_some(neighbor)` is also `None`
                    // whenever the nearest cell is the queried cell itself.
                    // They are not indistinguishable: `distance` separates
                    // them into three classes -- `:381` is `0.0`,
                    // `:351`/`:358` are positive, `:368`/`:375` negative --
                    // and `assert_eq!(actual.distance, expected.distance)`
                    // above already pins the class against the oracle's own
                    // value. Bite-checked: giving `:351` the fallthrough's
                    // `distance: 0.0` fails that assertion in both fixtures
                    // (`left: 0.0, right: 0.4`), so a cause swap does not
                    // survive.
                    //
                    // Within this sentinel branch the remaining `:351` vs
                    // `:358` ambiguity cannot arise. `:358` is reached only
                    // when `is_cell_valid(pos)` holds, and this branch is
                    // entered precisely when `closest_point` is still the
                    // UNINITIALIZED sentinel -- which is invalid, so `:351`
                    // is the only reachable cause here. That matches the
                    // instrumentation, which showed both fixtures' three
                    // sentinel queries reaching `positive-invalid-pos`.
                    assert!(
                        actual.voxel.is_none(),
                        "{response_name}: this port must report no voxel at the sentinel position for {query:?}"
                    );
                    assert_eq!(
                        [actual.position.x, actual.position.y, actual.position.z],
                        [x, y, z],
                        "{response_name}: this port's position must fall back to the queried cell for {query:?}"
                    );
                } else {
                    assert_eq!(
                        [actual.position.x, actual.position.y, actual.position.z],
                        expected.position,
                        "{response_name}: nearest_cell position for {query:?}"
                    );
                    assert_eq!(
                        actual.voxel.is_some(),
                        expected.voxel_present,
                        "{response_name}: nearest_cell voxel presence for {query:?}"
                    );
                }
            }
            None => assert!(
                !in_grid,
                "{response_name}: oracle omitted nearest for an in-grid query {query:?}"
            ),
        }
    }
}

#[test]
fn distance_field_matches_oracle_without_negative_propagation() {
    check_scenario(
        "distance_field_request.json",
        "distance_field_response.json",
    );
}

#[test]
fn distance_field_matches_oracle_with_negative_propagation() {
    check_scenario(
        "distance_field_negative_request.json",
        "distance_field_negative_response.json",
    );
}

/// The fixture must actually exercise the documented sentinel deviation —
/// otherwise the assertions inside `check_scenario` for it would vacuously
/// never run.
#[test]
fn fixture_reaches_the_uninitialized_sentinel_path() {
    for name in [
        "distance_field_response.json",
        "distance_field_negative_response.json",
    ] {
        let fixture = load_response(name);
        let hit_sentinel = fixture
            .queries
            .iter()
            .filter_map(|q| q.nearest.as_ref())
            .any(|n| n.position == [-1, -1, -1] && n.voxel_present);
        assert!(hit_sentinel, "{name}: no query reached the sentinel path");
    }
}
