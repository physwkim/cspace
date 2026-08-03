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
//! Every case here reports failed, because `rust_impl` is deliberately
//! unimplemented until Phase 1/2 land. That is the property under test: the
//! runner must reach the comparison and report a disagreement, not die on the
//! way there.

use std::path::PathBuf;
use std::process::{Command, Output};

/// Run the runner against the fake oracle with `cases` FK cases.
fn run(cases: &str) -> Output {
    let fake = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fake-oracle.py");
    Command::new(env!("CARGO_BIN_EXE_moveit-diff"))
        .args([
            "--urdf",
            "/dev/null",
            "--srdf",
            "/dev/null",
            "--cases",
            cases,
        ])
        .arg("--oracle")
        .arg("python3")
        .arg(&fake)
        .output()
        .expect("failed to run moveit-diff")
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
fn an_unimplemented_rust_side_exits_nonzero() {
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
