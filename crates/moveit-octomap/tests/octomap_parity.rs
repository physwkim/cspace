// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `octomap` op.
//!
//! `octomap` is a separate upstream package with no `.cpp` source on this
//! machine (see `src/lib.rs`'s module docs), so this crate's own unit tests
//! in `tree.rs` are this port's only ground truth for most of its behaviour.
//! This test adds an independent, real-`liboctomap.so.1.9.7`-backed check
//! for the four boundary cases `tree.rs`'s unit tests exercise as pure
//! assertions on this port's own arithmetic: repeated hits actually converge
//! to the *same* clamp the shared library reaches (and stop there), a miss
//! sequence drops below the *same* occupancy threshold, pruning eight
//! uniform siblings collapses to the *same* node count and preserves the
//! *same* log-odds, and a ray leaving the tree's representable range is
//! rejected the same way.
//!
//! `tests/fixtures/octomap_request.json` is the literal request array fed to
//! the oracle's `octomap` op (one object per scenario); `octomap_response.json`
//! is its unedited response, captured via a probe container run against
//! `liboctomap.so.1.9.7` (see this change's commit body for the exact
//! `docker run` invocation and image digest). This test replays each
//! request's actions through this crate's own [`OcTree`] and compares its
//! answers against the captured response for the same request's queries.
//!
//! Only the action/query variants this fixture actually uses are modelled
//! here (`update_point`, `update_key`, `prune`; `occupancy`,
//! `occupancy_by_key`, `node_count`, `ray_keys`) -- this is a fixture replay,
//! not a mirror of the oracle's full wire protocol.

use std::fs;

use moveit_octomap::{OcTree, OcTreeKey};
use nalgebra::Point3;
use serde::Deserialize;
use serde_json::Value;

/// log-odds are stored as `f32` on both sides of the parity check (upstream:
/// `OcTreeDataNode<float>`; this port: `Node::log_odds: f32`), so anything
/// above single-precision rounding is a real disagreement.
const LOG_ODDS_EPS: f64 = 1e-5;
/// `occupancy` is `probability(log_odds)` computed in `f64` on both sides;
/// the only expected noise is the `f32` rounding already covered by
/// [`LOG_ODDS_EPS`] propagating through `exp`.
const OCCUPANCY_EPS: f64 = 1e-6;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ActionSpec {
    UpdatePoint {
        point: [f64; 3],
        occupied: bool,
        #[serde(default)]
        lazy_eval: bool,
    },
    UpdateKey {
        key: [u16; 3],
        occupied: bool,
        #[serde(default)]
        lazy_eval: bool,
    },
    Prune,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum QuerySpec {
    Occupancy { point: [f64; 3] },
    OccupancyByKey { key: [u16; 3] },
    NodeCount,
    RayKeys { origin: [f64; 3], end: [f64; 3] },
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    resolution: f64,
    actions: Vec<ActionSpec>,
    queries: Vec<QuerySpec>,
}

#[derive(Deserialize)]
struct OracleResult {
    results: Vec<Value>,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    ok: bool,
    result: OracleResult,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("octomap_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse octomap_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("octomap_response.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse octomap_response.json: {e}"))
}

/// Upstream `keyToCoord(key_type)` at the finest depth, matching
/// `OcTree::key_to_coord_at_depth`'s private formula -- reproduced here
/// rather than exposed publicly since only this test needs to turn an
/// `occupancy_by_key` fixture key back into a point for the public
/// point-keyed API.
fn key_to_coord(resolution: f64, key: u16) -> f64 {
    (f64::from(key) - f64::from(OcTree::TREE_MAX_VAL) + 0.5) * resolution
}

fn expect_bool(v: &Value, key: &str, ctx: &str) -> bool {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_bool()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a bool"))
}

fn expect_f64(v: &Value, key: &str, ctx: &str) -> f64 {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_f64()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a number"))
}

fn expect_u64(v: &Value, key: &str, ctx: &str) -> u64 {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_u64()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not an integer"))
}

#[test]
fn octomap_matches_liboctomap_for_every_boundary_scenario() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");
        assert!(response.ok, "id {}: oracle reported ok=false", request.id);
        assert_eq!(
            request.queries.len(),
            response.result.results.len(),
            "id {}: query/result count mismatch",
            request.id
        );

        let mut tree = OcTree::new(request.resolution);
        for action in &request.actions {
            match action {
                ActionSpec::UpdatePoint {
                    point,
                    occupied,
                    lazy_eval,
                } => {
                    tree.update_node(Point3::from(*point), *occupied, *lazy_eval);
                }
                ActionSpec::UpdateKey {
                    key,
                    occupied,
                    lazy_eval,
                } => {
                    tree.update_node_by_key(
                        OcTreeKey::new(key[0], key[1], key[2]),
                        *occupied,
                        *lazy_eval,
                    );
                }
                ActionSpec::Prune => tree.prune(),
            }
        }

        for (query, result) in request.queries.iter().zip(&response.result.results) {
            let ctx = format!("id {}", request.id);
            match query {
                QuerySpec::Occupancy { point } => {
                    let mapped = expect_bool(result, "mapped", &ctx);
                    let actual = tree.log_odds_at(Point3::from(*point));
                    assert_eq!(mapped, actual.is_some(), "{ctx}: occupancy mapped mismatch");
                    if mapped {
                        let expected_log_odds = expect_f64(result, "log_odds", &ctx);
                        let expected_occupancy = expect_f64(result, "occupancy", &ctx);
                        assert!(
                            (f64::from(actual.unwrap()) - expected_log_odds).abs() < LOG_ODDS_EPS,
                            "{ctx}: log_odds {} vs oracle {expected_log_odds}",
                            actual.unwrap()
                        );
                        let actual_occupancy = tree.occupancy_at(Point3::from(*point)).unwrap();
                        assert!(
                            (actual_occupancy - expected_occupancy).abs() < OCCUPANCY_EPS,
                            "{ctx}: occupancy {actual_occupancy} vs oracle {expected_occupancy}"
                        );
                    }
                }
                QuerySpec::OccupancyByKey { key } => {
                    let point = Point3::new(
                        key_to_coord(request.resolution, key[0]),
                        key_to_coord(request.resolution, key[1]),
                        key_to_coord(request.resolution, key[2]),
                    );
                    let mapped = expect_bool(result, "mapped", &ctx);
                    let actual = tree.log_odds_at(point);
                    assert_eq!(
                        mapped,
                        actual.is_some(),
                        "{ctx}: occupancy_by_key {key:?} mapped mismatch"
                    );
                    if mapped {
                        let expected_log_odds = expect_f64(result, "log_odds", &ctx);
                        assert!(
                            (f64::from(actual.unwrap()) - expected_log_odds).abs() < LOG_ODDS_EPS,
                            "{ctx}: key {key:?} log_odds {} vs oracle {expected_log_odds}",
                            actual.unwrap()
                        );
                    }
                }
                QuerySpec::NodeCount => {
                    let expected_count = expect_u64(result, "count", &ctx);
                    assert_eq!(
                        tree.num_nodes() as u64,
                        expected_count,
                        "{ctx}: node_count mismatch"
                    );
                }
                QuerySpec::RayKeys { origin, end } => {
                    let expected_ok = expect_bool(result, "ok", &ctx);
                    let actual = tree.compute_ray_keys(Point3::from(*origin), Point3::from(*end));
                    assert_eq!(
                        expected_ok,
                        actual.is_some(),
                        "{ctx}: ray_keys ok mismatch for origin {origin:?} end {end:?}"
                    );
                    if expected_ok {
                        let expected_keys = result
                            .get("keys")
                            .unwrap_or_else(|| panic!("{ctx}: missing keys"))
                            .as_array()
                            .unwrap_or_else(|| panic!("{ctx}: keys is not an array"));
                        let actual_ray = actual.unwrap();
                        assert_eq!(
                            actual_ray.len(),
                            expected_keys.len(),
                            "{ctx}: ray_keys length mismatch"
                        );
                        for (actual_key, expected_key) in actual_ray.iter().zip(expected_keys) {
                            let e = expected_key
                                .as_array()
                                .unwrap_or_else(|| panic!("{ctx}: ray key is not an array"));
                            assert_eq!(
                                [actual_key[0], actual_key[1], actual_key[2]],
                                [
                                    e[0].as_u64().unwrap() as u16,
                                    e[1].as_u64().unwrap() as u16,
                                    e[2].as_u64().unwrap() as u16,
                                ],
                                "{ctx}: ray key mismatch"
                            );
                        }
                    }
                }
            }
        }
    }
}
