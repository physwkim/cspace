// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`OcTree::read_binary_data`]/[`OcTree::read_data`]
//! against the moveit2 C++ oracle's `octomap` op `serialize` query (added
//! by the orchestrator, commit `5a02d49`).
//!
//! `tests/fixtures/decode_request.json` asks the oracle to build a tree
//! (empty, one node, an eight-sibling prune collapse, a mixed
//! occupied/free tree with a partially-filled inner node), then, in this
//! order, `serialize` (which calls `tree.prune()` as a side effect before
//! writing `binary`/`full`, see this crate's `oracle.cpp` reading in this
//! change's commit body -- so `serialize` must run before `tree_walk` in
//! each request's `queries` array for the two results to describe the same
//! post-prune tree) and `tree_walk`. `decode_response.json` is the
//! unedited response, captured via `tools/moveit-oracle/run-oracle.sh`
//! against the fanuc fixture model already used by `octomap_parity.rs`.
//!
//! This test decodes each fixture's `binary`/`full` hex payload through
//! this crate's own [`OcTree::read_binary_data`]/[`OcTree::read_data`] and
//! compares the result against the same request's `tree_walk` result --
//! structurally exact for both wire formats (coordinate/size/depth/is_leaf
//! for every node, in the same pre-order [`OcTree::tree_nodes`] yields),
//! but occupied-classification-only for the binary path's log-odds:
//! `readBinaryData` is lossy, reconstructing a leaf's log-odds as exactly
//! `clamping_thres_min`/`clamping_thres_max` rather than the value that was
//! actually serialized (see `tree.rs`'s `read_binary_node` doc). The full
//! (`read_data`) path is lossless, so its log-odds/occupancy are compared
//! exactly (within the same `LOG_ODDS_EPS`/`OCCUPANCY_EPS` tolerance
//! `octomap_parity.rs` uses).

use std::fs;

use cspace_octomap::{DecodeError, OcTree};
use serde::Deserialize;
use serde_json::Value;

const LOG_ODDS_EPS: f64 = 1e-5;
const OCCUPANCY_EPS: f64 = 1e-6;

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    resolution: f64,
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
    let raw = read_fixture("decode_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse decode_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("decode_response.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse decode_response.json: {e}"))
}

/// `binary`/`full` are hex-encoded byte strings in the oracle's JSON
/// response; no hex crate is a dependency of this crate (or its
/// dev-dependencies) elsewhere, so this is a small self-contained decoder
/// rather than a new dependency for one test file.
fn decode_hex(s: &str) -> Vec<u8> {
    assert_eq!(s.len() % 2, 0, "odd-length hex string: {s}");
    (0..s.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&s[i..i + 2], 16)
                .unwrap_or_else(|e| panic!("bad hex byte {:?}: {e}", &s[i..i + 2]))
        })
        .collect()
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

fn expect_str<'a>(v: &'a Value, key: &str, ctx: &str) -> &'a str {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_str()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a string"))
}

/// Structural fields every decoded node must match exactly, on both wire
/// formats: coordinate, size, depth, is_leaf. Log-odds/occupancy are
/// checked by the caller, since the binary path's tolerance for them
/// differs from the full path's.
fn assert_structure_matches(
    actual: &cspace_octomap::TreeNode<'_>,
    expected: &Value,
    node_ctx: &str,
) {
    let c = actual.coordinate();
    assert!(
        (c.x - expect_f64(expected, "x", node_ctx)).abs() < 1e-9,
        "{node_ctx}: x mismatch"
    );
    assert!(
        (c.y - expect_f64(expected, "y", node_ctx)).abs() < 1e-9,
        "{node_ctx}: y mismatch"
    );
    assert!(
        (c.z - expect_f64(expected, "z", node_ctx)).abs() < 1e-9,
        "{node_ctx}: z mismatch"
    );
    assert!(
        (actual.size() - expect_f64(expected, "size", node_ctx)).abs() < 1e-9,
        "{node_ctx}: size mismatch"
    );
    assert_eq!(
        u64::from(actual.depth()),
        expect_u64(expected, "depth", node_ctx),
        "{node_ctx}: depth mismatch"
    );
    assert_eq!(
        actual.is_leaf(),
        expect_bool(expected, "is_leaf", node_ctx),
        "{node_ctx}: is_leaf mismatch"
    );
}

#[test]
fn decode_matches_liboctomap_serialize_for_every_boundary_scenario() {
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
            response.result.results.len(),
            2,
            "id {}: expected [serialize, tree_walk] results",
            request.id
        );
        let ctx = format!("id {}", request.id);
        let serialize = &response.result.results[0];
        let tree_walk = &response.result.results[1];
        assert_eq!(
            expect_str(serialize, "id", &ctx),
            "OcTree",
            "{ctx}: serialize id is not \"OcTree\""
        );
        let expected_nodes = tree_walk
            .get("nodes")
            .unwrap_or_else(|| panic!("{ctx}: missing nodes"))
            .as_array()
            .unwrap_or_else(|| panic!("{ctx}: nodes is not an array"));

        // Binary path: lossy at leaves, checked structurally plus
        // occupied/free classification only.
        let binary_bytes = decode_hex(expect_str(serialize, "binary", &ctx));
        if binary_bytes.is_empty() {
            assert!(
                expected_nodes.is_empty(),
                "{ctx}: empty binary payload but tree_walk has nodes"
            );
            let mut tree = OcTree::new(request.resolution);
            assert_eq!(
                tree.read_binary_data(&binary_bytes),
                Err(DecodeError::UnexpectedEof),
                "{ctx}: empty binary payload should be UnexpectedEof"
            );
        } else {
            let mut tree = OcTree::new(request.resolution);
            tree.read_binary_data(&binary_bytes)
                .unwrap_or_else(|e| panic!("{ctx}: read_binary_data failed: {e}"));
            let actual_nodes: Vec<_> = tree.tree_nodes().collect();
            assert_eq!(
                actual_nodes.len(),
                expected_nodes.len(),
                "{ctx}: binary decode node count mismatch"
            );
            for (i, (actual, expected)) in actual_nodes.iter().zip(expected_nodes).enumerate() {
                let node_ctx = format!("{ctx}: binary decode node {i}");
                assert_structure_matches(actual, expected, &node_ctx);
                let expected_occupied = expect_f64(expected, "occupancy", &node_ctx) > 0.5;
                assert_eq!(
                    actual.is_occupied(),
                    expected_occupied,
                    "{node_ctx}: occupied-side classification mismatch (binary decode is lossy \
                     at leaves, so only the occupied/free side is checked, not the exact value)"
                );
            }
        }

        // Full path: lossless, checked structurally plus exact
        // log-odds/occupancy.
        let full_bytes = decode_hex(expect_str(serialize, "full", &ctx));
        if full_bytes.is_empty() {
            assert!(
                expected_nodes.is_empty(),
                "{ctx}: empty full payload but tree_walk has nodes"
            );
            let mut tree = OcTree::new(request.resolution);
            assert_eq!(
                tree.read_data(&full_bytes),
                Err(DecodeError::UnexpectedEof),
                "{ctx}: empty full payload should be UnexpectedEof"
            );
        } else {
            let mut tree = OcTree::new(request.resolution);
            tree.read_data(&full_bytes)
                .unwrap_or_else(|e| panic!("{ctx}: read_data failed: {e}"));
            let actual_nodes: Vec<_> = tree.tree_nodes().collect();
            assert_eq!(
                actual_nodes.len(),
                expected_nodes.len(),
                "{ctx}: full decode node count mismatch"
            );
            for (i, (actual, expected)) in actual_nodes.iter().zip(expected_nodes).enumerate() {
                let node_ctx = format!("{ctx}: full decode node {i}");
                assert_structure_matches(actual, expected, &node_ctx);
                assert!(
                    (f64::from(actual.log_odds()) - expect_f64(expected, "log_odds", &node_ctx))
                        .abs()
                        < LOG_ODDS_EPS,
                    "{node_ctx}: log_odds mismatch"
                );
                assert!(
                    (actual.occupancy() - expect_f64(expected, "occupancy", &node_ctx)).abs()
                        < OCCUPANCY_EPS,
                    "{node_ctx}: occupancy mismatch"
                );
            }
        }
    }
}
