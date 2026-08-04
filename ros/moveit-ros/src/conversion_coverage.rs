// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Structural round-trip coverage self-check (`task-p9-ros-round6.md`, item 1).
//!
//! Round 5 found `VisibilityConstraint`'s core->msg direction had landed
//! while this crate's own doc comment still said "not implemented" -- caught
//! only by manually re-reading 48 `rg` hits for staleness. That method does
//! not scale and does not fire again on the next drift. This module turns
//! the same class of gap into a `cargo test` failure: it scans this crate's
//! own `src/` for every `impl (TryFrom|From)<...>` block (the same anchor
//! `task-p9-ros-round6.md` names) and every `round_trip`-named test
//! function, per file, and cross-checks the counts against [`FILES`] below.
//! A new conversion `impl` added anywhere in `src/` changes the scanned
//! count and desyncs it from [`FILES`]'s hardcoded expectation -- failing
//! this test until a human either adds a round-trip test (raising
//! `round_trip_tests`) or explicitly records why the new impl needs none
//! (see [`FileCoverage`]'s field docs). Either way the omission becomes a
//! visible diff to this file instead of a silently stale doc claim.
//!
//! # What this proves and what it does not
//!
//! This is a coverage *census*, not a correctness proof: it confirms a
//! `round_trip`-named test function exists in the same file as a
//! conversion's `impl` blocks, not that the test asserts every field with
//! distinct values (that discipline -- `task-p9-ros-round6.md` item 1 step
//! 4 -- is a code-review concern this census cannot enforce). Coverage is
//! tracked per *file*, not per *pair*: a file with two bidirectional pairs
//! and one round-trip test satisfies "at least one test exists" but not
//! "every pair has its own test." It also cannot detect a whole new file of
//! un-registered conversions on its own -- [`every_rs_file_with_a_conversion_impl_is_registered`]
//! is the second test that closes exactly that gap by walking `src/`
//! directly rather than trusting [`FILES`]'s path list.

use std::fs;
use std::path::Path;

/// One `src/` file's conversion-impl and round-trip-test counts.
struct FileCoverage {
    /// Path relative to `src/`.
    path: &'static str,
    /// Number of `impl (TryFrom|From)<...>` blocks this file should contain.
    impls: usize,
    /// Number of test functions with `round_trip` in their name this file
    /// should contain. Zero is correct only when every conversion in this
    /// file is genuinely one-directional (documented at its own impl site,
    /// with a §153.1 expiry condition) or is exercised by another
    /// conversion's round-trip test in the same file (documented in that
    /// impl's own doc comment).
    round_trip_tests: usize,
}

const FILES: &[FileCoverage] = &[
    // RobotTrajectory (planning wrapper) + PlanningRequest + PlanningResponse.
    // RobotTrajectory's own direction has no dedicated test name; it is
    // exercised inside round_trip_response_through_msg (see planning.rs).
    FileCoverage {
        path: "planning.rs",
        impls: 6,
        round_trip_tests: 2,
    },
    FileCoverage {
        path: "state.rs",
        impls: 2,
        round_trip_tests: 1,
    },
    FileCoverage {
        path: "model.rs",
        impls: 2,
        round_trip_tests: 1,
    },
    // Point, Vector3, Quaternion, Pose.
    FileCoverage {
        path: "geometry.rs",
        impls: 8,
        round_trip_tests: 4,
    },
    FileCoverage {
        path: "trajectory.rs",
        impls: 2,
        round_trip_tests: 1,
    },
    // round_trip_through_msg (aggregate) + visibility_member_round_trips.
    FileCoverage {
        path: "constraints/set.rs",
        impls: 2,
        round_trip_tests: 2,
    },
    // TryFrom<u8> for CollisionObjectOperation: one-directional by design
    // (decodes the wire discriminant; this crate never re-encodes a
    // CollisionObject command message). Expires if this crate ever needs to
    // build an outgoing CollisionObject message.
    FileCoverage {
        path: "scene/collision_object.rs",
        impls: 1,
        round_trip_tests: 0,
    },
    // SensorViewDirection + VisibilityConstraint.
    FileCoverage {
        path: "constraints/visibility.rs",
        impls: 4,
        round_trip_tests: 2,
    },
    FileCoverage {
        path: "constraints/orientation.rs",
        impls: 2,
        round_trip_tests: 1,
    },
    FileCoverage {
        path: "constraints/joint.rs",
        impls: 2,
        round_trip_tests: 1,
    },
    // TryFrom<SolidPrimitiveMsg> for Shape (one-directional impl; its
    // reverse is the manual fn `body_to_solid_primitive`, not a TryFrom/From
    // impl, so it never appears in this scan -- exercised together with
    // PositionConstraint's own round-trip test) + PositionConstraint pair.
    FileCoverage {
        path: "constraints/position.rs",
        impls: 3,
        round_trip_tests: 1,
    },
    // Mesh (asymmetric: TryFrom<MeshMsg> for Shape / TryFrom<Mesh> for
    // MeshMsgOut) + Plane.
    FileCoverage {
        path: "scene/shapes.rs",
        impls: 4,
        round_trip_tests: 2,
    },
];

fn count_matching_lines(text: &str, pattern: impl Fn(&str) -> bool) -> usize {
    text.lines().filter(|l| pattern(l.trim_start())).count()
}

fn is_conversion_impl_line(line: &str) -> bool {
    line.starts_with("impl") && (line.contains("TryFrom<") || line.contains(" From<"))
}

fn is_round_trip_test_line(line: &str) -> bool {
    line.starts_with("fn ") && line.contains("round_trip")
}

#[test]
fn every_registered_file_matches_its_declared_conversion_and_round_trip_counts() {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    for file in FILES {
        let full_path = src_dir.join(file.path);
        let text = fs::read_to_string(&full_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", full_path.display()));
        let actual_impls = count_matching_lines(&text, is_conversion_impl_line);
        let actual_tests = count_matching_lines(&text, is_round_trip_test_line);
        assert_eq!(
            actual_impls, file.impls,
            "{}: found {actual_impls} `impl (TryFrom|From)<...>` block(s), but \
             conversion_coverage::FILES says {} -- a conversion was added or \
             removed without updating this file's FileCoverage entry",
            file.path, file.impls
        );
        assert_eq!(
            actual_tests, file.round_trip_tests,
            "{}: found {actual_tests} `round_trip`-named test function(s), but \
             conversion_coverage::FILES says {} -- either a round-trip test \
             changed without updating this file's FileCoverage entry, or (if \
             this mismatch is because you just added a conversion) this file \
             needs a round-trip test before its FileCoverage entry can \
             legitimately claim coverage",
            file.path, file.round_trip_tests
        );
    }
}

/// Every `impl (TryFrom|From)<...>` block anywhere in `src/` must belong to
/// a file [`FILES`] lists -- otherwise a whole new file of conversions could
/// exist uncounted. Walks `src/` directly rather than trusting [`FILES`]'s
/// own path list, so a new file is caught even before anyone adds it there.
#[test]
fn every_rs_file_with_a_conversion_impl_is_registered() {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let registered: std::collections::BTreeSet<&str> = FILES.iter().map(|f| f.path).collect();
    let mut unregistered = Vec::new();
    visit_rs_files(&src_dir, &src_dir, &mut |rel_path, text| {
        if count_matching_lines(text, is_conversion_impl_line) > 0
            && !registered.contains(rel_path.as_str())
        {
            unregistered.push(rel_path);
        }
    });
    assert!(
        unregistered.is_empty(),
        "file(s) with conversion impls missing from conversion_coverage::FILES: {unregistered:?}"
    );
}

fn visit_rs_files(root: &Path, dir: &Path, f: &mut impl FnMut(String, &str)) {
    for entry in fs::read_dir(dir).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            visit_rs_files(root, &path, f);
        } else if path.extension().is_some_and(|e| e == "rs") {
            let text = fs::read_to_string(&path).unwrap();
            let rel = path
                .strip_prefix(root)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            f(rel, &text);
        }
    }
}
