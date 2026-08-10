// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`OcTree::write_binary_data`]/[`OcTree::write_data`]
//! against the same oracle fixtures `decode_parity.rs` uses
//! (`tests/fixtures/decode_request.json`/`decode_response.json`, see that
//! file's own doc for how they were captured).
//!
//! A round trip through this crate's own decode-then-encode is not
//! sufficient ground truth on its own: both directions could share the same
//! mistake and still agree with each other. This test instead decodes the
//! oracle's own `binary`/`full` bytes (already independently verified
//! structurally correct against the oracle's `tree_walk` result by
//! `decode_parity.rs`), re-encodes the resulting tree, and compares the
//! re-encoded bytes byte-for-byte against the oracle's *original* bytes --
//! pinning the encode direction against the oracle, not against this
//! crate's own decoder.
//!
//! Byte-exact equality is the correct expectation on both wire formats, not
//! just a looser structural match: the binary format's per-leaf
//! classification (free/occupied/has-children/absent) is exactly what
//! `read_binary_data` preserves even though it discards the leaf's original
//! log-odds value, and the full format is lossless end to end, so
//! re-encoding a freshly decoded tree must reproduce the exact input bytes
//! for every non-empty fixture.

use std::fs;

use cspace_octomap::OcTree;
use serde::Deserialize;
use serde_json::Value;

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

/// Same self-contained hex decoder as `decode_parity.rs` -- no hex crate is
/// a dependency of this crate or its dev-dependencies elsewhere.
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

fn expect_str<'a>(v: &'a Value, key: &str, ctx: &str) -> &'a str {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_str()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a string"))
}

#[test]
fn encode_matches_liboctomap_serialize_bytes_for_every_boundary_scenario() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    let mut binary_cases = 0;
    let mut full_cases = 0;

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");
        assert!(response.ok, "id {}: oracle reported ok=false", request.id);
        let ctx = format!("id {}", request.id);
        let serialize = &response.result.results[0];
        assert_eq!(
            expect_str(serialize, "id", &ctx),
            "OcTree",
            "{ctx}: serialize id is not \"OcTree\""
        );

        let binary_bytes = decode_hex(expect_str(serialize, "binary", &ctx));
        if !binary_bytes.is_empty() {
            let mut tree = OcTree::new(request.resolution);
            tree.read_binary_data(&binary_bytes)
                .unwrap_or_else(|e| panic!("{ctx}: read_binary_data failed: {e}"));
            assert_eq!(
                tree.write_binary_data(),
                binary_bytes,
                "{ctx}: write_binary_data did not reproduce the oracle's own binary bytes"
            );
            binary_cases += 1;
        }

        let full_bytes = decode_hex(expect_str(serialize, "full", &ctx));
        if !full_bytes.is_empty() {
            let mut tree = OcTree::new(request.resolution);
            tree.read_data(&full_bytes)
                .unwrap_or_else(|e| panic!("{ctx}: read_data failed: {e}"));
            assert_eq!(
                tree.write_data(),
                full_bytes,
                "{ctx}: write_data did not reproduce the oracle's own full bytes"
            );
            full_cases += 1;
        }
    }

    // No silent no-op: if the fixtures ever stopped exercising a non-empty
    // tree, this test would still pass on an empty set of comparisons --
    // guard against that reading as coverage while providing none.
    assert!(binary_cases > 0, "no non-empty binary fixtures exercised");
    assert!(full_cases > 0, "no non-empty full fixtures exercised");
}
