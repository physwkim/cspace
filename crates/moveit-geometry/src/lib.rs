// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/transforms/include/moveit/transforms/transforms.hpp
//   moveit_core/transforms/src/transforms.cpp
// and from geometric_shapes 2.3.3 (see shapes.rs's provenance comment).

//! Frame transforms and geometric primitives for moveit-rs.
//!
//! This crate carries [`Transforms`] (`moveit_core/transforms`) and the
//! `geometric_shapes` shape and body layers (see the [`shapes`] and
//! [`bodies`] module docs for scope and provenance).
//!
//! # Completion statement (round 15, item 3)
//!
//! Every number below is a command someone can re-run, not a claim to
//! trust -- modeled on `moveit-scene`'s own completion statement (commit
//! `08ab3c7`), which exists for the same reason: `PORTING-PLAN.md` §65
//! caught a crate's plan and code silently disagreeing for ten rounds
//! because nothing forced a re-check.
//!
//! **Symbol audits.** [`shapes`]'s `shapes.h`/`shape_operations.h` audit
//! (round 8) and [`bodies`]'s `bodies.h`/`body_operations.h` audit (round 8)
//! are each a classified line per upstream declaration or overload group:
//!
//! ```text
//! rg -c '^//! - `' crates/moveit-geometry/src/shapes.rs                      # whole-file bullet count
//! awk 'NR==247,NR==371' crates/moveit-geometry/src/bodies.rs | grep -c '^//! - `'  # the round-8 audit block only
//! ```
//!
//! are **14** and **12** respectively (`bodies.rs`'s audit block also
//! classifies `BodyVector` in prose, not a bullet -- 13 symbols total there).
//! Every classification lands in `ported`, `subsumed by D4`, `D1 excludes
//! it`, or a justified `unported` (dead code confirmed by disassembly, zero
//! callers confirmed by `rg` across the pinned `moveit2` tree, or superseded
//! by this port's own design) -- zero unclassified or "still needs doing"
//! entries in either audit. [`bodies`]'s own "Who actually calls this"
//! section names the one method that is ported with genuinely zero external
//! consumer, [`bodies::Body::intersects_ray`], and its reopening condition;
//! see that section rather than duplicating it here.
//!
//! **Transfer boundary (round 15, item 2).** [`shapes`]'s "Transfer
//! boundary, symbol by symbol" section names, for every symbol that crosses
//! a crate boundary, which crate owns it now and what that crate still needs
//! to receive it -- not repeated here either.
//!
//! **Tests, and what each checks against the real oracle.**
//!
//! ```text
//! cargo nextest run -p moveit-geometry --no-fail-fast   # 148 tests run: 148 passed, 0 skipped
//! rg -c '#\[test\]' crates/moveit-geometry/tests/*.rs
//! ```
//!
//! 148 total; **18** of those are oracle- or shipped-`.so`-backed integration
//! tests, not self-referential unit tests: `mesh_parity.rs` (1, every Panda/
//! Fanuc/PR2 collision STL against the real oracle), `body_query_parity.rs`
//! (1, posed-body algorithms), `octree_in_world_parity.rs` (1, `Shape::OcTree`
//! posed in a `World`), `octree_shape_query_parity.rs` (1, the leaf-`Cuboid`
//! `Compound` against FCL's real octree query), and `probe_parity.rs` (14,
//! `bodies::`/`shapes::` primitives probed directly against the shipped
//! `libgeometric_shapes.so.2.3.3` -- this file also carries the two
//! documented-upstream-defect tests, `convex_mesh_sign_bug_upstream_defect`
//! and `convex_mesh_ray1_anchor_choice_deviation`, which assert this port's
//! *documented deviation* from upstream, not agreement with it). The
//! remaining 130 are `#[cfg(test)]` unit tests inside `src/`, most of them
//! per-invariant-boundary (zero/negative dimensions, degenerate shapes,
//! masking-proof bisected tolerances -- see `shapes.rs`'s and `bodies.rs`'s
//! own provenance comments for that history) rather than one test per
//! upstream method.
//!
//! **`assert_relative_eq!` reckoning (round 18, item 2), recounted fresh in
//! the current tree, not trusted from PORTING-PLAN.md's §104 workspace
//! table** (which predates several rounds of change and never broke this
//! crate's own share out on its own). Not counted by `rg -c
//! assert_relative_eq! crates/moveit-geometry/src/*.rs` (13, the exact
//! §73.1/§83.3/§92/§104.1 miscount class: 4 of those 13 are doc-comment
//! prose mentioning the macro by name, not a call to it -- `shapes.rs`'s own
//! comment records that round 14's §79 sweep already converted every one of
//! that file's 45 `assert_relative_eq!` calls to `assert_eq!`, and
//! `octree_collision.rs`'s comment records the same for its one bit-exact
//! call). Counted instead by stripping `//` tails and paren-bracket-matching
//! each `assert_relative_eq!(`/`relative_eq!(` call's own argument text --
//! a script rather than an eyeball pass, so the same miscount can't recur
//! next round:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-geometry/src/*.rs
//! both=9 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! All **9** real call sites (6 in [`bodies`]'s tests, 3 in `transforms.rs`'s
//! tests) pass both `epsilon` and `max_relative` explicitly, each with a
//! comment recording the bisection that measured the given `epsilon` as real
//! headroom above a found floor (not an unmeasured carryover) and pins
//! `max_relative = 0.0` so the relative term never masks it -- the exact
//! §85.3/§103.4 discipline. `neither` is 0, so there is nothing to bisect or
//! convert to `assert_eq!` this round. An earlier PORTING-PLAN.md passage
//! (pre-§104) recorded 4 outstanding sites outside `bodies.rs` --
//! `transforms.rs` (3, still present, accounted for above) and `stl.rs` (1,
//! now 0: `grep -n assert_relative_eq crates/moveit-geometry/src/stl.rs`
//! finds nothing, its tests are plain `assert_eq!`/`assert!`). Sibling crate
//! `moveit-octomap`'s own reckoning is 0 calls, `approx` never being a
//! dependency there; see that crate's completion statement.
//!
//! Two manual `assert!((lhs - rhs).abs() < tol)` sites in
//! `octree_collision.rs` (lines 132-133, 163 as of this round) are outside
//! this reckoning's scope -- they take no named `epsilon`/`max_relative`
//! argument to classify, being a different pattern (a bare tolerance
//! literal) than the `assert_relative_eq!`/`relative_eq!` macro family this
//! item covers.
//!
//! **Audit scripts checked against themselves (round 19, item 1).** Both
//! `tools/ci/count-relative-eq.pl` and `tools/ci/count-public-declarations.sh` are
//! now committed files, so a sibling panel's own source tree includes them
//! once copied -- `PORTING-PLAN.md` §117.4's trap (a paragraph's own text
//! changing the count the paragraph cites). Run against themselves:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl tools/ci/count-relative-eq.pl
//! both=0 epsilon_only=0 max_relative_only=0 neither=0   # after this round's fix; was both=2 before it
//! bash tools/ci/count-public-declarations.sh tools/ci/count-public-declarations.sh count_public_declarations
//! 0   # a bash script has no `class` to match; unaffected by the fix below
//! ```
//!
//! Before the fix, `count_relative_eq.pl` counted 2 false calls from its own
//! `#`-Perl-comment doc header (it only stripped `//`-style comments, so
//! running it against itself -- a Perl file -- stripped nothing); a
//! synthetic `.rs` fixture with a fake call inside a `/* */` block comment
//! and inside a `"..."` string literal reproduced the same false-positive
//! class against real Rust syntax. `count_public_declarations.sh` never
//! stripped string-literal contents at all, so a `"..."` containing a bare
//! `{`/`}` corrupts its brace-depth counter; a synthetic header with such a
//! literal reproduced this (3 real members undercounted to 1). Neither bug
//! changed any count already committed above or in `shapes.rs`/`bodies.rs`/
//! `tree.rs`: `grep -no '"[^"]*[{}][^"]*"'` against every header those counts
//! were taken from finds no braced string literal, and none of this crate's
//! `.rs` files has a `/* */` block comment or a string literal containing
//! `assert_relative_eq!`/`relative_eq!`-shaped text. Both scripts now strip
//! `/* */` block comments (the perl one already did for header text; it did
//! not for its own doc header) and blank string-literal contents before
//! scanning; every count in this file and in `shapes.rs`/`bodies.rs`/
//! `tree.rs` was re-run against the fixed scripts and is unchanged.
//!
//! **§79 recount (round 19, item 2).** Re-run fresh against the fixed
//! script rather than trusting round 18's number:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-geometry/src/*.rs
//! both=9 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Unchanged from round 18. p3-acm disposed its own 51 sites this round (41
//! `epsilon`-only + 10 `neither`, all bit-identical once bisected, converted
//! to `assert_eq!`) -- this crate has 0 in either bucket to dispose the same
//! way; all 9 are `both`, and none is a disposal candidate: each carries its
//! own one-line reason already, in its own doc comment at the call site
//! (`bodies.rs`'s three pairs and one single call, round 14's §79 sweep;
//! `transforms.rs`'s three calls, round 16 item 3) -- a measured non-zero
//! floor above `epsilon = 0.0`, not an unmeasured carryover, so keeping
//! `assert_relative_eq!` there is correct, not merely unreviewed.
//!
//! **Tolerance-floor re-measurement mandate, this crate: none.** Commit
//! `70a6b31` fixed the workspace's `serde_json` to `float_roundtrip`
//! because the default parser returned 6,859/84,221 (8.1%) committed
//! fixture float literals one ULP off, and "tolerances hide it... it sets
//! the floor every bisection here measures against" (that commit's own
//! body). Surveyed every tolerance constant in this crate against that
//! risk:
//!
//! - The 9 bisected `assert_relative_eq!` sites above (`bodies.rs`,
//!   `transforms.rs`) each measure their floor by comparing an in-Rust-
//!   computed `f64` against a hand-written Rust source literal (e.g.
//!   `-0.475`, `[0.0, 1.0, 0.0]`) -- parsed by `rustc`'s own
//!   correctly-rounded float-literal parser, never by `serde_json`. No
//!   fixture JSON is read in any of these tests
//!   (`rg -n serde_json crates/moveit-geometry/src/bodies.rs
//!   crates/moveit-geometry/src/transforms.rs` -- no match, neither file
//!   imports it). Unaffected regardless of the parser bug.
//! - `bodies.rs`'s `OBB::overlaps` `const EPS: f64 = 1e-9` is not a
//!   comparison tolerance at all -- Ericson's SAT numerical-stability
//!   guard baked into the algorithm itself, added to `abs_r`'s components
//!   before the separating-axis test, not measured against anything.
//! - The oracle-fixture-comparison constants in `tests/*.rs`
//!   (`LINEAR_EPS`/`VOLUME_EPS` in `body_query_parity.rs`; `LOG_ODDS_EPS`/
//!   `OCCUPANCY_EPS`/`POSE_EPS` in `octree_in_world_parity.rs`;
//!   `VERTEX_EPS` in `mesh_parity.rs`; `DISTANCE_EPS` in
//!   `octree_shape_query_parity.rs`; `EPS` in `probe_parity.rs`) do
//!   compare against `serde_json`-parsed oracle values, but none was
//!   chosen by bisecting down to an observed floor: each has stayed at
//!   its introduction-commit value with zero revisions since
//!   (`git log -p --follow` on each file, checked this round), and each
//!   with a stated rationale carries an analytic derivation from known
//!   type precision (`f32` log-odds rounding, STL `f32` vertex
//!   precision, `f64` arithmetic rounding) rather than an empirically
//!   bisected minimum -- `LINEAR_EPS`/`VOLUME_EPS` alone carry no stated
//!   rationale, but at a flat `1e-9` (roughly 1e7x a 1-ULP diff at unit
//!   magnitude) an extra ULP of parser noise could not have driven their
//!   choice either. Nothing here was "the floor every bisection measures
//!   against" in the sense `70a6b31` warns about; nothing to re-measure.

pub mod bodies;
mod octree_collision;
pub mod quaternion;
pub mod shapes;
pub mod stl;
mod transforms;

pub use octree_collision::compound_from_octree;
pub use shapes::{
    BoundingSphere, Cone, Cuboid, Cylinder, Mesh, OcTree, Plane, Shape, ShapeType, Sphere,
};
pub use stl::mesh_from_bytes;
pub use transforms::Transforms;

/// Rigid-body transform. Replaces upstream `Eigen::Isometry3d`.
pub type Isometry3 = nalgebra::Isometry3<f64>;
/// 3-vector. Replaces upstream `Eigen::Vector3d`.
pub type Vector3 = nalgebra::Vector3<f64>;
/// Unit quaternion. Replaces upstream `Eigen::Quaterniond`.
pub type UnitQuaternion = nalgebra::UnitQuaternion<f64>;
/// Rotation matrix. Replaces upstream `Eigen::Matrix3d` where it holds a rotation.
pub type Rotation3 = nalgebra::Rotation3<f64>;
