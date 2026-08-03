// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Exercises the runner end to end against `fake-oracle.py`.
//!
//! `fake-oracle.py` existed before this file and nothing ran it: cargo only
//! collects `tests/*.rs`, so the stand-in oracle drifted from the real one
//! unchecked -- it answered `type_name: "REVOLUTE"` where moveit2 answers
//! `"Revolute"`. These tests are what make the stand-in a check rather than a
//! file.
//!
//! Every case here reports failed. `rust_impl` is fully wired to
//! `moveit-model`/`moveit-state` now, but `tests/fixtures/tiny.{urdf,srdf}`
//! (needed so the runner can build a real `RobotModel` at all) describes a
//! different robot than `fake-oracle.py`'s hand-rolled "fake" model, so
//! model_info and every fk case disagree by construction. That is the
//! property under test: the runner must reach the comparison and report a
//! disagreement, not die on the way there. Agreement against the real oracle
//! is `moveit-diff`'s own binary run in CI, not this test.

use std::path::PathBuf;
use std::process::{Command, Output};

/// Run the runner against the fake oracle with `cases` FK cases and any
/// `extra` arguments inserted before `--oracle` (which must stay last: it
/// swallows every argument after it as the oracle's own command line).
fn run_with(cases: &str, extra: &[&str]) -> Output {
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let fake = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fake-oracle.py");
    Command::new(env!("CARGO_BIN_EXE_moveit-diff"))
        .args([
            "--urdf",
            fixtures.join("tiny.urdf").to_str().unwrap(),
            "--srdf",
            fixtures.join("tiny.srdf").to_str().unwrap(),
            "--cases",
            cases,
        ])
        .args(extra)
        .arg("--oracle")
        .arg("python3")
        .arg(&fake)
        .output()
        .expect("failed to run moveit-diff")
}

/// [`run_with`] with no extra arguments -- every pre-existing call site.
fn run(cases: &str) -> Output {
    run_with(cases, &[])
}

#[test]
fn the_runner_completes_a_session_against_the_fake_oracle() {
    let out = run("3");
    let stdout = String::from_utf8(out.stdout).expect("stdout is not utf-8");

    // model_info plus one case per random state.
    assert!(
        stdout.contains("cases:  4"),
        "expected 4 cases, got:\n{stdout}"
    );
    assert!(
        stdout.contains("oracle model: fake (3 links, 2 joints, 1 groups)"),
        "model line missing or changed:\n{stdout}"
    );
}

#[test]
fn a_disagreeing_rust_side_exits_with_failure_not_a_crash() {
    let out = run("1");
    let stdout = String::from_utf8(out.stdout).expect("stdout is not utf-8");

    // Exit 1 means "cases failed"; exit 2 means the runner itself broke, which
    // is the failure this distinction exists to keep visible.
    assert_eq!(
        out.status.code(),
        Some(1),
        "expected the failure exit, got {:?}:\n{stdout}",
        out.status.code()
    );
    assert!(
        stdout.contains("passed: 0"),
        "expected every case to fail:\n{stdout}"
    );
}

/// `--stats-json` must write the same counts the stdout summary prints, as
/// real JSON a caller can parse rather than another string to scrape --
/// PORTING-PLAN.md §60.3 is two denominators wrong for exactly the opposite
/// reason this flag exists.
#[test]
fn stats_json_writes_the_report_as_machine_readable_json() {
    let dir = std::env::temp_dir();
    let path = dir.join(format!("moveit-diff-stats-{}.json", std::process::id()));
    let path_str = path.to_str().unwrap();

    let out = run_with("1", &["--stats-json", path_str]);
    let stdout = String::from_utf8(out.stdout).expect("stdout is not utf-8");
    assert!(
        stdout.contains("passed: 0"),
        "expected every case to fail:\n{stdout}"
    );

    let contents = std::fs::read_to_string(&path).expect("--stats-json did not write a file");
    std::fs::remove_file(&path).ok();
    let value: serde_json::Value =
        serde_json::from_str(&contents).expect("--stats-json output is not valid JSON");

    // "1" case plus model_info, matching `run`'s "cases:  4" comment for "3".
    assert_eq!(value["cases"], 2);
    assert_eq!(value["passed"], 0);
    assert_eq!(value["failed"], 2);
    assert_eq!(value["underpowered"], 0);
    // No --group/--collision/--ik on this run, so every optional block is
    // absent rather than a zeroed-out struct that would misread as "ran and
    // found nothing".
    assert!(value["worst_jacobian_deviation"].is_null());
    assert!(value["worst_distance_deviation"].is_null());
    assert!(value["distance_pairs"].is_null());
    assert!(value["ik"].is_null());
}

#[test]
fn the_fake_oracle_spells_joint_types_the_way_moveit_does() {
    // moveit2's JointModel::getTypeName() switch returns these strings
    // verbatim. A stand-in that answers "REVOLUTE" would let a case-sensitive
    // comparison in the runner pass here and fail against the real oracle.
    let fake = std::fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("fake-oracle.py"),
    )
    .expect("fake-oracle.py is unreadable");

    for wrong in ["REVOLUTE", "PRISMATIC", "PLANAR", "FLOATING", "FIXED"] {
        assert!(
            !fake.contains(wrong),
            "fake-oracle.py uses the enumerator spelling {wrong:?}; \
             getTypeName() returns the capitalized form"
        );
    }
    assert!(
        fake.contains("\"Revolute\""),
        "fake-oracle.py no longer exercises a joint type"
    );
}
