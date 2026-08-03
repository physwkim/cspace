// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `acm` op.
//!
//! Ground truth is the oracle's own response, captured verbatim into
//! `tests/fixtures/{panda,fanuc,dual_arm_panda,pr2}_acm.json` by querying
//! `tools/moveit-oracle` (built at the pinned SHA in `PORTING-PLAN.md`) with
//! `fixtures/{panda,fanuc,dual_arm_panda,pr2}.{urdf,srdf}`. Comparing against a deserialized
//! fixture, rather than hand-transcribed Rust literals, means a transcription
//! typo can't make this test assert the wrong thing and a future oracle
//! change shows up as a fixture diff instead of silent drift — the same
//! pattern `crates/moveit-model/tests/urdf_parity.rs` uses.
//!
//! `fixtures/pr2.srdf` is a deliberately truncated excerpt (see its own
//! `<!-- and many more disable_collisions tags -->` comment): only one real
//! `<disable_collisions>` tag survives the truncation. No complete PR2 SRDF
//! was found on this host (`third_party/moveit_resources` ships only
//! `pr2_description`, not a `pr2_moveit_config`) or inside the oracle
//! container's filesystem, searched per role instructions before concluding
//! absence. The PR2 case below is therefore a genuine but thin differential
//! test: it proves the oracle and `AllowedCollisionMatrix::from_srdf` agree
//! on the one pair the fixture carries, not full PR2 collision-matrix parity.

use std::collections::BTreeMap;
use std::fs;

use serde::Deserialize;

use moveit_collision::{AllowedCollisionMatrix, AllowedCollisionType};
use moveit_srdf::SrdfModel;

#[derive(Deserialize)]
struct OracleAcmEntry {
    link1: String,
    link2: String,
    #[serde(rename = "type")]
    kind: String,
}

#[derive(Deserialize)]
struct OracleAcmResult {
    names: Vec<String>,
    entries: Vec<OracleAcmEntry>,
    defaults: BTreeMap<String, String>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: OracleAcmResult,
}

fn load_fixture(file_name: &str) -> OracleAcmResult {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result
}

fn load_srdf(srdf_file: &str) -> SrdfModel {
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let srdf_xml =
        fs::read_to_string(&srdf_path).unwrap_or_else(|e| panic!("read {srdf_path}: {e}"));
    SrdfModel::parse_str(&srdf_xml).unwrap_or_else(|e| panic!("parse {srdf_path}: {e}"))
}

fn kind_name(kind: AllowedCollisionType) -> &'static str {
    match kind {
        AllowedCollisionType::Never => "NEVER",
        AllowedCollisionType::Always => "ALWAYS",
        AllowedCollisionType::Conditional => "CONDITIONAL",
    }
}

/// Assert `matrix` reproduces `expected` field by field: every explicit pair
/// entry, every per-name default, and — since [`OracleAcmResult::names`] is
/// the union the oracle's own `getAllEntryNames` reports — the same name set
/// [`AllowedCollisionMatrix::all_entry_names`] computes.
fn assert_matches_oracle(matrix: &AllowedCollisionMatrix, expected: &OracleAcmResult) {
    assert_eq!(matrix.all_entry_names(), expected.names, "all_entry_names");

    for entry in &expected.entries {
        let actual = matrix.entry(&entry.link1, &entry.link2).unwrap_or_else(|| {
            panic!(
                "missing explicit entry for ({}, {})",
                entry.link1, entry.link2
            )
        });
        assert_eq!(
            kind_name(actual.kind()),
            entry.kind,
            "entry kind for ({}, {})",
            entry.link1,
            entry.link2
        );
    }
    assert_eq!(
        matrix.len(),
        expected
            .entries
            .iter()
            .flat_map(|e| [e.link1.clone(), e.link2.clone()])
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        "row count (getSize)"
    );

    for (name, kind) in &expected.defaults {
        let actual = matrix
            .default_entry(name)
            .unwrap_or_else(|| panic!("missing default entry for {name}"));
        assert_eq!(kind_name(actual.kind()), *kind, "default kind for {name}");
    }
}

#[test]
fn panda_matches_oracle() {
    let srdf = load_srdf("panda.srdf");
    let matrix = AllowedCollisionMatrix::from_srdf(&srdf);
    let expected = load_fixture("panda_acm.json");
    assert_eq!(
        expected.entries.len(),
        34,
        "panda ground truth is 34 disable_collisions entries"
    );
    assert_matches_oracle(&matrix, &expected);
}

#[test]
fn fanuc_matches_oracle() {
    let srdf = load_srdf("fanuc.srdf");
    let matrix = AllowedCollisionMatrix::from_srdf(&srdf);
    let expected = load_fixture("fanuc_acm.json");
    assert_eq!(
        expected.entries.len(),
        10,
        "fanuc ground truth is 10 disable_collisions entries"
    );
    assert_matches_oracle(&matrix, &expected);
}

#[test]
fn dual_arm_panda_matches_oracle() {
    // The largest fixture: 68 entries over 22 links, exactly panda's 34
    // twice. Every pair is intra-arm -- there is no `left_panda_*` against
    // `right_panda_*` entry -- so the matrix is two disjoint blocks over two
    // link-name prefixes, which is the property no single-arm SRDF here can
    // test: a `from_srdf` that let entries leak across the prefixes, or that
    // collapsed the two blocks into one, still passes panda and fanuc.
    let srdf = load_srdf("dual_arm_panda.srdf");
    let matrix = AllowedCollisionMatrix::from_srdf(&srdf);
    let expected = load_fixture("dual_arm_panda_acm.json");
    assert_eq!(
        expected.entries.len(),
        68,
        "dual_arm_panda ground truth is 68 disable_collisions entries"
    );
    assert_matches_oracle(&matrix, &expected);
}

#[test]
fn pr2_matches_oracle() {
    let srdf = load_srdf("pr2.srdf");
    let matrix = AllowedCollisionMatrix::from_srdf(&srdf);
    let expected = load_fixture("pr2_acm.json");
    // fixtures/pr2.srdf is a deliberately truncated excerpt (see module docs):
    // only one real <disable_collisions> tag survives. This still exercises
    // the from_srdf/oracle agreement on the pair that fixture does carry.
    assert_eq!(
        expected.entries.len(),
        1,
        "fixtures/pr2.srdf is a truncated stub with one entry"
    );
    assert_matches_oracle(&matrix, &expected);
}
