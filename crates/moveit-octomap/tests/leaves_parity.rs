// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`OcTree::leaves`] against the real `liboctomap.so.1.9.7`
//! `leaf_iterator`, using the oracle's `octree_points` op's `leaves` field
//! (PORTING-PLAN.md §102, added this round for `moveit-distance-field`'s own
//! `getOcTreePoints` question; `leaves` is a plain `tree.begin_leafs()` walk,
//! independent of that op's distance-field-specific `points`/`count`
//! outputs, so it is equally good ground truth here).
//!
//! Round 18, item 3: [`OcTree::leaves`] (`Leaves`, upstream `leaf_iterator`)
//! shared its ordering claim with [`OcTree::tree_nodes`] (`TreeNodes`,
//! upstream `tree_iterator`) only by argument -- both are built on the same
//! `push_children` descent, and `tree_nodes`'s order is already pinned
//! field-by-field against the oracle's `tree_walk` query
//! (`octomap_parity.rs`). But `tree_iterator` and `leaf_iterator` are
//! distinct upstream classes; sharing this port's own `push_children` helper
//! does not by itself prove upstream's `leaf_iterator` visits leaves in the
//! same relative order `tree_iterator` visits nodes. This test measures
//! `leaf_iterator` directly instead of relying on that inference.
//!
//! `tests/fixtures/leaves_request.json`/`leaves_response.json` are the
//! literal request/response captured via the oracle's `octree_points` op
//! (see this change's commit body for the exact `docker run` invocation and
//! oracle stamp). Two scenarios: id 1 collapses 8 uniform siblings via
//! `prune` into one coarser leaf (pins a pruned leaf's `coordinate`/`size`);
//! id 2 puts one leaf in each of four different octants, one left
//! unoccupied, with no pruning (pins pre-order sibling ordering and the
//! `occupied` flag together).

use std::fs;

use moveit_octomap::{OcTree, OcTreeKey};
use nalgebra::Point3;
use serde::Deserialize;
use serde_json::Value;

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
struct RequestFixture {
    id: u64,
    octree_resolution: f64,
    actions: Vec<ActionSpec>,
}

#[derive(Deserialize)]
struct OracleResult {
    leaves: Vec<Value>,
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

#[test]
fn leaves_matches_liboctomap_leaf_iterator_order_and_fields() {
    let requests: Vec<RequestFixture> = serde_json::from_str(&read_fixture("leaves_request.json"))
        .unwrap_or_else(|e| panic!("parse leaves_request.json: {e}"));
    let responses: Vec<OracleResponse> =
        serde_json::from_str(&read_fixture("leaves_response.json"))
            .unwrap_or_else(|e| panic!("parse leaves_response.json: {e}"));
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.id, response.id, "request/response id mismatch");
        assert!(response.ok, "id {}: oracle reported ok=false", request.id);

        let mut tree = OcTree::new(request.octree_resolution);
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

        let actual: Vec<_> = tree.leaves().collect();
        let ctx = format!("id {}", request.id);
        assert_eq!(
            actual.len(),
            response.result.leaves.len(),
            "{ctx}: leaf count mismatch"
        );
        for (i, (leaf, expected)) in actual.iter().zip(&response.result.leaves).enumerate() {
            let leaf_ctx = format!("{ctx}: leaf {i}");
            let expected_coord = expected
                .get("coordinate")
                .unwrap_or_else(|| panic!("{leaf_ctx}: missing coordinate"))
                .as_array()
                .unwrap_or_else(|| panic!("{leaf_ctx}: coordinate is not an array"));
            // Same 1e-9 coordinate/size tolerance octomap_parity.rs's
            // tree_walk check uses -- both sides compute keyToCoord from the
            // same integer key and resolution, but this is not asserted
            // bit-exact there either, so this test matches that precedent
            // rather than introducing a stricter, untested guarantee.
            let c = leaf.coordinate();
            assert!(
                (c.x - expected_coord[0].as_f64().unwrap()).abs() < 1e-9,
                "{leaf_ctx}: x mismatch"
            );
            assert!(
                (c.y - expected_coord[1].as_f64().unwrap()).abs() < 1e-9,
                "{leaf_ctx}: y mismatch"
            );
            assert!(
                (c.z - expected_coord[2].as_f64().unwrap()).abs() < 1e-9,
                "{leaf_ctx}: z mismatch"
            );
            assert!(
                (leaf.size()
                    - expected
                        .get("size")
                        .unwrap_or_else(|| panic!("{leaf_ctx}: missing size"))
                        .as_f64()
                        .unwrap())
                .abs()
                    < 1e-9,
                "{leaf_ctx}: size mismatch"
            );
            assert_eq!(
                leaf.is_occupied(),
                expected
                    .get("occupied")
                    .unwrap_or_else(|| panic!("{leaf_ctx}: missing occupied"))
                    .as_bool()
                    .unwrap(),
                "{leaf_ctx}: occupied mismatch"
            );
        }
    }
}
