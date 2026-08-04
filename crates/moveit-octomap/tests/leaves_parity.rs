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
//! oracle stamp). Three scenarios: id 1 collapses 8 uniform siblings via
//! `prune` into one coarser leaf (pins a pruned leaf's `coordinate`/`size`);
//! id 2 puts one leaf in each of four different octants, one left
//! unoccupied, with no pruning (pins pre-order sibling ordering and the
//! `occupied` flag together); id 3 additionally carries a `bbx` request
//! field and a `leaves_bbx` response field (PORTING-PLAN.md §123.2, round
//! 21: [`OcTree::leaves_in_bbx`] against upstream `leaf_bbx_iterator`) --
//! see [`leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`]
//! for why id 3's geometry was chosen and how its order-sensitivity is
//! guarded against a vacuous pass.

use std::fs;

use moveit_octomap::{Leaf, OcTree, OcTreeKey};
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
struct Bbx {
    min: [f64; 3],
    max: [f64; 3],
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    octree_resolution: f64,
    actions: Vec<ActionSpec>,
    #[serde(default)]
    bbx: Option<Bbx>,
}

#[derive(Deserialize)]
struct OracleResult {
    leaves: Vec<Value>,
    #[serde(default)]
    leaves_bbx: Option<Vec<Value>>,
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

/// Boolean twin of [`assert_leaves_match`], used only by the perturbation
/// self-check in
/// [`leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`]:
/// that check needs to confirm a *mismatch* is detected, and doing that by
/// asserting-then-catching-the-panic would require a process-global panic
/// hook swap, which races against every other test in this binary running
/// concurrently on another thread. A plain boolean has no such interaction.
fn leaves_match(actual: &[Leaf], expected: &[Value]) -> bool {
    actual.len() == expected.len()
        && actual.iter().zip(expected).all(|(leaf, expected)| {
            let c = leaf.coordinate();
            let Some(coord) = expected.get("coordinate").and_then(Value::as_array) else {
                return false;
            };
            let (Some(ex), Some(ey), Some(ez)) = (
                coord.first().and_then(Value::as_f64),
                coord.get(1).and_then(Value::as_f64),
                coord.get(2).and_then(Value::as_f64),
            ) else {
                return false;
            };
            if (c.x - ex).abs() >= 1e-9 || (c.y - ey).abs() >= 1e-9 || (c.z - ez).abs() >= 1e-9 {
                return false;
            }
            let Some(size) = expected.get("size").and_then(Value::as_f64) else {
                return false;
            };
            if (leaf.size() - size).abs() >= 1e-9 {
                return false;
            }
            let Some(occupied) = expected.get("occupied").and_then(Value::as_bool) else {
                return false;
            };
            leaf.is_occupied() == occupied
        })
}

/// Shared per-leaf, index-wise comparison used by both [`Leaves`] and
/// [`LeavesInBbx`] parity checks. Order-sensitive: `actual[i]` is compared
/// against `expected[i]`, not matched by set membership, so a swapped-order
/// bug is caught as long as the leaves being compared are not themselves
/// coordinate-identical (see
/// [`leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`]
/// for where that precondition is asserted).
fn assert_leaves_match(actual: &[Leaf], expected: &[Value], ctx: &str) {
    assert_eq!(actual.len(), expected.len(), "{ctx}: leaf count mismatch");
    for (i, (leaf, expected)) in actual.iter().zip(expected).enumerate() {
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
        assert_leaves_match(&actual, &response.result.leaves, &ctx);
    }
}

/// Round 21, PORTING-PLAN.md §123.2 sub-item 1: [`OcTree::leaves_in_bbx`]
/// (upstream `leaf_bbx_iterator`) against a real oracle capture, id 3 of
/// `leaves_request.json`/`leaves_response.json`.
///
/// Geometry: the same four-octant, one-unoccupied layout as id 2, plus a
/// `bbx` query (`min = (-0.5,-0.5,-0.5)`, `max = (-0.02,0.5,0.5)`) that
/// clips to `x < 0` with margin from every voxel boundary (0.1 resolution,
/// leaf centers at `x = ±0.05`), so no rounding-boundary ambiguity is
/// exercised here -- deliberately, since that is a separate question from
/// the one this test answers (order and field values, not boundary
/// inclusion). This proposal was sent to the orchestrator via a
/// `caucus signal note --kind question` mid-round and captured for real
/// against `liboctomap.so.1.9.7` (oracle stamp `8ed8a9395b730b08`); see
/// `moveit-octomap/src/lib.rs`'s "`LeavesInBbx` split, round 21" doc section
/// for the full decision record, including why the client-side-filtered
/// self-derived alternative was rejected in favor of this real capture.
///
/// **Guarding against a vacuous order check.** A pairwise indexed
/// comparison (see [`assert_leaves_match`]) only actually tests order if
/// the compared leaves are not themselves coordinate-identical -- two
/// identical leaves would pass whether or not this port emitted them in
/// upstream's order. `leaves_bbx`'s two leaves for id 3,
/// `(-0.05,-0.05,-0.05)` and `(-0.05,0.05,0.05)`, differ (in `y` and `z`),
/// which is asserted explicitly below before the order-sensitive check
/// runs, and confirmed by an explicit perturbation: swapping the two
/// expected leaves must no longer match this port's actual output, i.e. a
/// hypothetical order-reversing bug in [`OcTree::leaves_in_bbx`] would be
/// caught, not silently accepted as "the right leaves, don't care which
/// order."
#[test]
fn leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields() {
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

    let mut exercised = false;
    for (request, response) in requests.iter().zip(&responses) {
        let (Some(bbx), Some(expected_bbx)) = (&request.bbx, &response.result.leaves_bbx) else {
            continue;
        };
        exercised = true;
        assert_eq!(request.id, response.id, "request/response id mismatch");
        assert!(response.ok, "id {}: oracle reported ok=false", request.id);
        assert!(
            expected_bbx.len() >= 2,
            "id {}: leaves_bbx needs at least 2 leaves to test order at all",
            request.id
        );

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

        let min = Point3::from(bbx.min);
        let max = Point3::from(bbx.max);
        let actual: Vec<_> = tree
            .leaves_in_bbx(min, max)
            .unwrap_or_else(|| panic!("id {}: leaves_in_bbx returned None", request.id))
            .collect();
        let ctx = format!("id {}", request.id);

        let first_coord = actual
            .first()
            .unwrap_or_else(|| panic!("{ctx}: leaves_bbx is empty, nothing to order-test"))
            .coordinate();
        let second_coord = actual[1].coordinate();
        assert_ne!(
            first_coord, second_coord,
            "{ctx}: leaves_bbx's first two leaves must have distinct coordinates, or a \
             pairwise indexed comparison cannot actually distinguish their order"
        );

        assert_leaves_match(&actual, expected_bbx, &ctx);

        // Perturbation: swapping the expected leaves 0 and 1 must stop
        // matching this port's actual output. If it still matched, the
        // comparison above would be vacuous with respect to order --
        // equally happy whichever order leaves_in_bbx produced.
        let mut swapped_expected = expected_bbx.clone();
        swapped_expected.swap(0, 1);
        assert!(
            !leaves_match(&actual, &swapped_expected),
            "{ctx}: swapping leaves_bbx's expected order must fail the comparison, \
             or this test cannot actually detect an order regression"
        );
    }
    assert!(
        exercised,
        "no fixture id carried both a bbx request field and a leaves_bbx response field"
    );
}
