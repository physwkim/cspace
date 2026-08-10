// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Structural round-trip coverage self-check (`task-p9-ros-round7.md`, item 1).
//!
//! Round 5 found `VisibilityConstraint`'s core->msg direction had landed
//! while this crate's own doc comment still said "not implemented" -- caught
//! only by manually re-reading 48 `rg` hits for staleness. Round 6 closed
//! that with a per-*file* census (impl count vs. round-trip-test count must
//! match a hardcoded table per file). Round 7 found the hole in that design
//! by mutation-testing it: a file with two bidirectional pairs and one
//! round-trip test satisfies "counts agree" without either pair actually
//! having its own test, so adding a *third* pair to such a file and bumping
//! only the impl count (never writing a test) still passes.
//!
//! This module tracks coverage per *type pair* instead. It scans every
//! `impl (TryFrom|From)<...>` block in `src/` (this file excluded -- see
//! below), extracts the `(from, to)` type names, and groups directed edges
//! into pairs: `(A, B)` and `(B, A)` both existing means this crate converts
//! `A` <-> `B` in both directions, and that pair must have a round-trip test
//! naming both types -- not just "some round-trip test exists somewhere in
//! this file."
//!
//! # Why type names, not the wrapper naming convention
//!
//! Most conversions in this crate follow the `XMsg`/`XMsgOut` convention
//! (`lib.rs`'s module doc): msg->core reads `impl TryFrom<XMsg> for Y`,
//! core->msg reads `impl TryFrom<Y> for XMsgOut`. Both edges share the core
//! type `Y`, so [`canon`] strips a trailing `Out` before matching reverses --
//! `(XMsg, Y)` and `(Y, XMsgOut)` become `(XMsg, Y)` and `(Y, XMsg)`, which
//! are literal reverses. [`ALIASES`] is the one further exception this crate
//! has: `shapes.rs`'s Mesh pair reads `impl TryFrom<MeshMsg> for Shape` /
//! `impl TryFrom<Mesh> for MeshMsgOut` -- the msg->core direction targets the
//! general `Shape` enum, the core->msg direction takes the concrete `Mesh`
//! variant, and this module cannot know from a line scan alone that the two
//! name the same real conversion pair without being told.
//!
//! # What this proves and what it does not
//!
//! Matching is substring-based (does a `round_trip`-named `#[test]`'s body
//! contain every one of the pair's type names), not a real call-graph check
//! -- precise enough that inserting a probe conversion with no matching test
//! reliably goes red (see this round's report for the actual failure
//! messages), but it cannot verify the test asserts *every field* with
//! distinct values (`task-p9-ros-round7.md`/round 6 item 1 step 4 is a
//! code-review discipline, not something a line scan can enforce).
//!
//! [`ONE_DIRECTIONAL`] and [`TRANSITIVELY_COVERED`] are named exemptions, not
//! escape hatches: each names the exact pair, why it is exempt, and what
//! event would end the exemption (§153.1). [`every_one_directional_pair_is_still_one_directional`]
//! and the `covered_by` existence/content check inside
//! [`every_bidirectional_pair_has_a_round_trip_test`] both re-verify the
//! exemption's premise on every run, not just at the moment it was written.
//!
//! This file is excluded from its own scan (both as a source of conversion
//! impls -- it has none -- and as a source of round-trip test bodies -- its
//! own helper/test function names happen to contain "round_trip" without
//! being a domain conversion's test). Auditing the auditor would be
//! circular, not additional coverage.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

/// This module's own file, relative to `src/` -- excluded from every scan
/// below (see the module doc's closing paragraph).
const SELF_PATH: &str = "conversion_coverage.rs";

/// A conversion this crate ports in one direction only, on purpose.
struct OneDirectional {
    from: &'static str,
    to: &'static str,
    /// Why, and what would end the exemption (§153.1).
    reason: &'static str,
}

const ONE_DIRECTIONAL: &[OneDirectional] = &[
    OneDirectional {
        from: "u8",
        to: "CollisionObjectOperation",
        reason: "decodes the wire discriminant only -- this crate never \
                 re-encodes a CollisionObject command message. Expires if \
                 this crate ever needs to build an outgoing CollisionObject \
                 message.",
    },
    OneDirectional {
        from: "SolidPrimitiveMsg",
        to: "Shape",
        reason: "the reverse is the manual fn `body_to_solid_primitive` \
                 (Body -> shape_msgs::SolidPrimitive), not a TryFrom/From \
                 impl, so it never appears in this scan; exercised together \
                 with PositionConstraint's own round-trip test. Expires if \
                 `body_to_solid_primitive` is ever replaced by a real \
                 `TryFrom<Shape> for SolidPrimitiveMsgOut` impl, which would \
                 make this pair subject to the same-file round-trip rule.",
    },
    OneDirectional {
        from: "OrientationConstraintQuaternion",
        to: "UnitQuaternion",
        reason: "round 15/§211: OrientationConstraintQuaternion exists solely \
                 to name OrientationConstraint.orientation's own, stricter \
                 upstream rule (kinematic_constraint.cpp:609-615) apart from \
                 the generic Quaternion<->UnitQuaternion pair every other \
                 site uses; the reverse direction (UnitQuaternion -> wire) \
                 has only one shape everywhere in this crate and already \
                 goes through the shared `TryFrom<UnitQuaternion> for \
                 Quaternion`, tested by orientation.rs's own \
                 round_trip_through_msg. Expires if a second msg->core \
                 caller ever needs this same stricter rule and a core->msg \
                 direction is added for it.",
    },
    OneDirectional {
        from: "Transform",
        to: "Isometry3",
        reason: "geometry_msgs/Transform arrives only as \
                 PlanningScene.fixed_frame_transforms; the core->msg \
                 direction of that field is upstream's \
                 Transforms::copyTransforms, which only \
                 getPlanningSceneMsg calls -- D1-deferred in cspace-scene. \
                 The reverse core type is shared with Pose, whose own \
                 `TryFrom<Isometry3> for Pose` is round-trip tested. \
                 Expires when getPlanningSceneMsg is ported and needs \
                 `TryFrom<Isometry3> for TransformMsgOut`.",
    },
    OneDirectional {
        from: "AllowedCollisionMatrixMsg",
        to: "AllowedCollisionMatrix",
        reason: "the reverse is upstream's \
                 AllowedCollisionMatrix::getMessage, reached only from \
                 getPlanningSceneMsg/getPlanningSceneDiffMsg -- both \
                 D1-deferred in cspace-scene, so nothing in this crate has \
                 an outgoing AllowedCollisionMatrix to build. Expires when \
                 either of those is ported.",
    },
    OneDirectional {
        from: "moveit_msgs::PlanningScene",
        to: "PlanningSceneUpdate",
        reason: "classification of the wire `is_diff` flag into the two \
                 wrapper types set_planning_scene_msg/\
                 set_planning_scene_diff_msg accept (scene/planning_scene.rs \
                 module doc). The reverse would be getPlanningSceneMsg, \
                 D1-deferred in cspace-scene; and it is not this enum's \
                 inverse anyway, since the wrappers hold the message \
                 verbatim. Expires when getPlanningSceneMsg is ported.",
    },
    OneDirectional {
        from: "ExecutionEventMsg",
        to: "ExecutionEvent",
        reason: "decodes the trajectory_execution_event payload, which this \
                 node only ever receives -- publishing the event is the \
                 *client's* side of the topic (move_group_interface.cpp:179), \
                 and this crate has no MoveGroupInterface port to publish it \
                 from. Expires if this crate ever gains a client that calls \
                 stop() on some other node.",
    },
];

/// A bidirectional pair with no round-trip test of its own, because it is
/// exercised as a sub-step of a different, named round-trip test.
struct TransitivelyCovered {
    /// Either directed edge of the pair identifies it uniquely, even though
    /// its core-side type can be shared with another pair (`RobotTrajectory`
    /// is the hub for both `planning.rs`'s own pair and `trajectory.rs`'s
    /// `JointTrajectoryMsg` pair).
    from: &'static str,
    to: &'static str,
    /// The `round_trip`-named `#[test]` function that exercises this pair.
    /// Its own body is checked for at least one of this pair's type names,
    /// so a stale claim (the covering test renamed or rewritten to no longer
    /// touch this pair) is caught, not just trusted.
    covered_by: &'static str,
    /// Why this pair has no test of its own, and what would end the
    /// exemption (§153.1).
    reason: &'static str,
}

const TRANSITIVELY_COVERED: &[TransitivelyCovered] = &[TransitivelyCovered {
    from: "RobotTrajectoryMsg",
    to: "RobotTrajectory",
    covered_by: "round_trip_response_through_msg",
    reason: "planning.rs's own RobotTrajectoryMsg<->RobotTrajectoryMsgOut pair \
             has no dedicated round-trip test; PlanningResponse embeds a \
             RobotTrajectory field, so its own round trip exercises this pair \
             as a sub-step. Expires if PlanningResponse ever stops carrying a \
             RobotTrajectory field, or if this pair gets a dedicated test.",
}];

/// Type names this scan treats as interchangeable when matching a reverse
/// edge -- see the module doc's "Why type names" section. Checked in both
/// directions.
const ALIASES: &[(&str, &str)] = &[("Shape", "Mesh")];

fn canon(name: &str) -> String {
    let stripped = name.strip_suffix("Out").unwrap_or(name);
    for (a, b) in ALIASES {
        // Both members of an alias pair must canonicalize to the *same*
        // representative (`a`) -- mapping each to the other instead would
        // make `canon("Shape") == "Mesh"` but `canon("Mesh") == "Shape"`,
        // which never converge to a common form for the reverse-edge check
        // below to compare.
        if stripped == *a || stripped == *b {
            return (*a).to_string();
        }
    }
    stripped.to_string()
}

#[derive(Debug, Clone)]
struct Edge {
    file: String,
    from: String,
    to: String,
}

fn is_conversion_impl_line(line: &str) -> bool {
    line.starts_with("impl") && (line.contains("TryFrom<") || line.contains(" From<"))
}

/// Extracts `(from_base_type, to_base_type)` from a conversion-impl line,
/// stripping lifetime/generic parameters from each side (`RobotTrajectoryMsg<'m>`
/// -> `RobotTrajectoryMsg`) and path qualifiers are kept as-is (`cspace_constraints::VisibilityConstraint`
/// stays whole, since both directions of that pair use the same qualified name).
fn parse_conversion(line: &str) -> Option<(String, String)> {
    let marker_len = if let Some(i) = line.find("TryFrom<") {
        i + "TryFrom<".len()
    } else {
        line.find(" From<").map(|i| i + " From<".len())?
    };
    let rest = &line[marker_len..];
    let mut depth = 1i32;
    let mut close = None;
    for (i, c) in rest.char_indices() {
        match c {
            '<' => depth += 1,
            '>' => {
                depth -= 1;
                if depth == 0 {
                    close = Some(i);
                    break;
                }
            }
            _ => {}
        }
    }
    let close = close?;
    let from_base = rest[..close].split('<').next()?.trim().to_string();
    let after = rest[close + 1..].trim_start().strip_prefix("for ")?;
    let after = after.trim_start();
    let end = after
        .find(|c: char| c == '<' || c == '{' || c.is_whitespace())
        .unwrap_or(after.len());
    let to_base = after[..end].to_string();
    if from_base.is_empty() || to_base.is_empty() {
        None
    } else {
        Some((from_base, to_base))
    }
}

// Assertion-discrimination sweep (round 8, folded-operand audit): the
// no-return-value guard above is `from_base.is_empty() || to_base.is_empty()`
// but neither operand had a direct test -- this file's own `#[test]`
// functions only ever call `parse_conversion` indirectly, through
// `all_edges` scanning real, always-well-formed source lines, so neither
// branch of this `||` was ever exercised at all. These isolate each
// operand directly: bite-checked by dropping one `is_empty()` clause from
// the guard and confirming only the *other* operand's test still catches
// the resulting `Some` where `None` was expected.
#[test]
fn parse_conversion_returns_none_for_empty_from_base() {
    assert_eq!(parse_conversion("impl TryFrom<> for X {"), None);
}

#[test]
fn parse_conversion_returns_none_for_empty_to_base() {
    assert_eq!(parse_conversion("impl TryFrom<X> for {"), None);
}

fn is_round_trip_test_line(line: &str) -> bool {
    line.starts_with("fn ") && line.contains("round_trip")
}

/// `(function_name, full_body_text)` for every `#[test]` function whose name
/// contains `round_trip`, across `src/` (excluding [`SELF_PATH`]).
fn round_trip_test_bodies(files: &[(String, String)]) -> Vec<(String, String)> {
    let mut bodies = Vec::new();
    for (_file, text) in files {
        let lines: Vec<&str> = text.lines().collect();
        let mut i = 0;
        while i < lines.len() {
            let trimmed = lines[i].trim_start();
            let preceded_by_test_attr = i > 0 && lines[i - 1].trim_start().starts_with("#[test]");
            if preceded_by_test_attr && is_round_trip_test_line(trimmed) {
                let name = trimmed
                    .strip_prefix("fn ")
                    .and_then(|s| s.split('(').next())
                    .unwrap_or(trimmed)
                    .to_string();
                let mut depth = 0i32;
                let mut started = false;
                let mut body = String::new();
                let mut j = i;
                while j < lines.len() {
                    for c in lines[j].chars() {
                        match c {
                            '{' => {
                                depth += 1;
                                started = true;
                            }
                            '}' => depth -= 1,
                            _ => {}
                        }
                    }
                    body.push_str(lines[j]);
                    body.push('\n');
                    if started && depth <= 0 {
                        break;
                    }
                    j += 1;
                }
                bodies.push((name, body));
                i = j;
            }
            i += 1;
        }
    }
    bodies
}

fn all_rs_files_except_self() -> Vec<(String, String)> {
    let src_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut out = Vec::new();
    visit_rs_files(&src_dir, &src_dir, &mut out);
    out.retain(|(path, _)| path != SELF_PATH);
    out
}

fn visit_rs_files(root: &Path, dir: &Path, out: &mut Vec<(String, String)>) {
    let mut entries: Vec<PathBuf> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            visit_rs_files(root, &path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            let text = fs::read_to_string(&path).unwrap();
            let rel = path
                .strip_prefix(root)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            out.push((rel, text));
        }
    }
}

fn all_edges(files: &[(String, String)]) -> Vec<Edge> {
    let mut edges = Vec::new();
    for (file, text) in files {
        for line in text.lines() {
            let trimmed = line.trim_start();
            if !is_conversion_impl_line(trimmed) {
                continue;
            }
            if let Some((from, to)) = parse_conversion(trimmed) {
                edges.push(Edge {
                    file: file.clone(),
                    from,
                    to,
                });
            }
        }
    }
    edges
}

fn has_reverse(edges: &[Edge], e: &Edge) -> bool {
    let (cf, ct) = (canon(&e.from), canon(&e.to));
    edges
        .iter()
        .any(|o| canon(&o.from) == ct && canon(&o.to) == cf)
}

#[test]
fn every_bidirectional_pair_has_a_round_trip_test() {
    let files = all_rs_files_except_self();
    let edges = all_edges(&files);
    let bodies = round_trip_test_bodies(&files);

    let mut seen: BTreeSet<(String, String)> = BTreeSet::new();
    let mut missing = Vec::new();

    for e in &edges {
        if !has_reverse(&edges, e) {
            continue; // one-directional; checked by the tests below.
        }
        let key = {
            let (cf, ct) = (canon(&e.from), canon(&e.to));
            if cf <= ct { (cf, ct) } else { (ct, cf) }
        };
        if !seen.insert(key) {
            continue; // already processed from the reverse edge.
        }

        let reverse = edges
            .iter()
            .find(|o| canon(&o.from) == canon(&e.to) && canon(&o.to) == canon(&e.from))
            .expect("has_reverse just confirmed one exists");
        let mut names: Vec<&str> = vec![&e.from, &e.to, &reverse.from, &reverse.to];
        names.sort();
        names.dedup();

        let direct = bodies
            .iter()
            .any(|(_, body)| names.iter().all(|n| body.contains(n)));
        if direct {
            continue;
        }

        let transitive = TRANSITIVELY_COVERED.iter().find(|t| {
            (t.from == e.from && t.to == e.to) || (t.from == reverse.from && t.to == reverse.to)
        });
        match transitive {
            Some(t) => {
                let covering = bodies.iter().find(|(name, _)| name == t.covered_by);
                match covering {
                    Some((_, body)) => {
                        assert!(
                            names.iter().any(|n| body.contains(n)),
                            "TRANSITIVELY_COVERED entry {} -> {} claims coverage by \
                             `{}`, but that test's body no longer mentions any of \
                             {names:?} -- the exemption's premise ({}) no longer \
                             holds; give this pair its own round-trip test or fix \
                             the entry",
                            t.from,
                            t.to,
                            t.covered_by,
                            t.reason
                        );
                    }
                    None => panic!(
                        "TRANSITIVELY_COVERED entry {} -> {} names `{}` as the \
                         covering test, but no `#[test]` `round_trip`-named \
                         function with that name exists anymore",
                        t.from, t.to, t.covered_by
                    ),
                }
            }
            None => missing.push(format!(
                "{:?} (in {}, reverse in {})",
                names, e.file, reverse.file
            )),
        }
    }

    assert!(
        missing.is_empty(),
        "bidirectional conversion pair(s) with no round-trip test (direct or \
         registered in TRANSITIVELY_COVERED): {missing:#?}"
    );
}

#[test]
fn every_one_directional_impl_is_registered_with_a_reason() {
    let files = all_rs_files_except_self();
    let edges = all_edges(&files);
    let mut unregistered = Vec::new();
    for e in &edges {
        if has_reverse(&edges, e) {
            continue;
        }
        let registered = ONE_DIRECTIONAL
            .iter()
            .any(|r| r.from == e.from && r.to == e.to);
        if !registered {
            unregistered.push(format!("{} -> {} (in {})", e.from, e.to, e.file));
        }
    }
    assert!(
        unregistered.is_empty(),
        "one-directional conversion impl(s) missing from ONE_DIRECTIONAL: {unregistered:#?}"
    );
}

#[test]
fn every_one_directional_pair_is_still_one_directional() {
    let files = all_rs_files_except_self();
    let edges = all_edges(&files);
    let mut stale = Vec::new();
    for r in ONE_DIRECTIONAL {
        let reverse_exists = edges.iter().any(|e| e.from == r.to && e.to == r.from);
        if reverse_exists {
            stale.push(format!("{} -> {} ({})", r.from, r.to, r.reason));
        }
    }
    assert!(
        stale.is_empty(),
        "ONE_DIRECTIONAL entry(ies) now have a reverse impl -- no longer \
         one-directional; remove the entry and add a round-trip test: {stale:#?}"
    );
}
