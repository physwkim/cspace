// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (Debian package liboctomap-dev 1.9.7+dfsg-3.1build3).

//! A probabilistic occupancy octree, the moveit-rs counterpart of the
//! `octomap` C++ library moveit2 depends on.
//!
//! PORTING-PLAN.md §6.3 flags octomap as a Phase 3 risk: no crates.io crate
//! is a mature drop-in (`bye_octomap_rs` 0.1.1, `octree` 0.1.0 from 2020, and
//! `bempp-octree`, a boundary-element tree, are the closest hits, none
//! maintained or occupancy-mapping-shaped), so this crate ports the subset
//! of upstream `octomap` that moveit2 actually calls, rather than pulling in
//! a dependency.
//!
//! # Version and provenance
//!
//! Ported from headers extracted from the `moveit-rs` oracle container's
//! `liboctomap-dev 1.9.7+dfsg-3.1build3` package (`octomap-config.cmake`'s
//! `OCTOMAP_VERSION` confirms `1.9.7`; that container ships only the Debian
//! headers, not the upstream `.cpp` sources, so every constant this crate
//! could not read from a header -- the five sensor-model defaults in
//! [`OcTree::new`] and the fresh-node "unknown" value in
//! `Node::new` -- was instead measured directly off the shipped
//! `liboctomap.so.1.9.7` with a standalone probe rather than assumed; see
//! `tree.rs` and `node.rs`'s doc comments for what was measured and how.
//!
//! # What was ported
//!
//! - `key` (crate-private module): [`OcTreeKey`], [`KeySet`], [`KeyRay`],
//!   and the bit-level tree descent primitives (`compute_child_key`,
//!   `compute_child_idx`, `compute_index_key`).
//! - `node` (crate-private module): the tree node representation, log-odds
//!   value, child bookkeeping, `expand`/`prune`/`is_collapsible`, and
//!   max-of-children occupancy propagation.
//! - `tree` (crate-private module): [`OcTree`] itself -- log-odds
//!   sensor-model update (`update_node`/`update_node_log_odds`, both by
//!   point and by key), Amanatides & Woo ray casting (`compute_ray_keys`),
//!   the batch free/occupied key computation behind `insertPointCloud`
//!   (`compute_update`), `insert_ray`, tree-wide `prune`, and the
//!   lazy-eval companion `update_inner_occupancy`.
//! - `iter` (crate-private module): full-tree leaf iteration
//!   (`OcTree::leaves`) and bounding-box-limited leaf iteration
//!   (`OcTree::leaves_in_bbx`), yielding [`Leaf`]/[`Leaves`]/[`LeavesInBbx`].
//!
//! # What was deliberately not ported
//!
//! - **`AbstractOcTree`'s runtime-type registry** (`StaticMemberInitializer`,
//!   `createTree`, `className()`). PORTING-PLAN.md's D4 rules out upstream's
//!   pluginlib-shaped runtime polymorphism; this crate has exactly one
//!   concrete tree type ([`OcTree`], corresponding to upstream's plain
//!   `OcTree` template instantiation), so nothing needs a factory to look up.
//! - **`ColorOcTree`/`CountingOcTree`/`OcTreeStamped`** and their node types.
//!   No moveit2 consumer references any of them.
//! - **Change detection** (`enableChangeDetection`, `KeyBoolMap`). Confirmed
//!   unused by every moveit2 consumer of octomap.
//! - **`setNodeValue`** (the direct, non-relative value setter). The one
//!   moveit2 call site that conceptually wants it
//!   (`lazy_free_space_updater.cpp`, forcing a cell to the clamp minimum)
//!   does so through the relative `update_node_log_odds` primitive with a
//!   deliberately saturating delta instead; see `tree.rs`'s module docs.
//! - **`insertPointCloud` and `computeDiscreteUpdate`** (the OpenMP-parallel
//!   convenience wrappers, and discretized/BBX-limited updates).  Neither
//!   moveit2 sensor updater calls them; both hand-roll their own ray casting
//!   and key-set bookkeeping using the lower-level primitives this crate
//!   does port (`compute_ray_keys`, `compute_update`, `update_node`).
//! - **`bodies::Body`-style posed-body algorithms are not in this crate's
//!   scope at all.** Stale as of round 15: this bullet used to say
//!   `moveit-geometry`'s `shapes::OcTree` was still a stub deferred to
//!   Phase 3 collision; it is not -- `shapes::OcTree` has been fully ported
//!   since round 3, and there is no `bodies::`-level posed counterpart for
//!   an octree upstream at all (`bodies::createBodyFromShape` returns
//!   `nullptr` for `shapes::OCTREE`; this crate's own `Body::from_shape`
//!   matches that with `Shape::OcTree => None`, see `bodies.rs`). See
//!   `moveit-geometry`'s `shapes.rs`, "Who consumes `Shape::OcTree`" for the
//!   actual, current transfer boundary and consumer-by-consumer status.
//!
//! # Symbol-by-symbol audit against upstream's public surface (round 16, item 2)
//!
//! Superseded round 12's "24 ported / 2 unported / 15 distinct / 41 symbol
//! group" table: "symbol group" was this crate's own undefined grouping
//! unit, not reproducible by an outside reader. [`OcTree`]'s own module doc
//! now carries the replacement -- one bullet per literal `public:`
//! declaration read from the five headers that make up the upstream
//! inheritance chain (`OcTree.h` -> `OccupancyOcTreeBase.h` ->
//! `OcTreeBaseImpl.h` -> `AbstractOccupancyOcTree.h` -> `AbstractOcTree.h`,
//! most-derived first), each read from inside the oracle container
//! (`octomap` is not checked out on this host) rather than guessed, in the
//! same literal-extraction style `moveit-scene`'s `planning_scene.hpp`
//! audit (commit `943e909`) uses. That walk also states, in one line, the
//! rule for collapsing the five-level inheritance chain into "this crate's
//! exposed surface" before counting, so a future round can re-verify the
//! count mechanically rather than trusting a judgment call.
//!
//! ```text
//! ported                55
//! unported, in scope     8
//! distinct               88
//! ------------------------
//! total                 151
//! ```
//!
//! See [`OcTree`]'s doc, "Symbol-by-symbol audit against upstream's public
//! surface (round 16, item 2)", for the full per-declaration walk and every
//! reason. One correction the fresh walk found: `AbstractOccupancyOcTree
//! ::isNodeAtThreshold` (both overloads) was never classified anywhere in
//! round 12's table -- not `ported`, not `unported, in scope`, not
//! `distinct` -- a genuine gap in that walk, the same class of drift round
//! 15 found for `tree_iterator`, caught only by reading the header fresh
//! rather than trusting the prior table. It has zero `moveit_core`
//! consumer and is now classified `distinct`.
//!
//! **Second correction (round 17, item 2).** Independently re-deriving
//! each header's raw declaration count and reconciling it against the
//! bullets actually present found `OcTreeBaseImpl.h`'s own concrete
//! `getTreeType()` (header line 104, shadowed by `OcTree.h`'s
//! more-derived override) absent from every walk through round 16 --
//! not a fresh bullet, not an "already counted above" cross-reference,
//! simply missing. Same class of drift as `isNodeAtThreshold` above,
//! this time inside round 16's own table. See [`OcTree`]'s doc for the
//! added cross-reference bullet; the total below is now 159, not 158.
//!
//! **A naming precision, not a gap:** [`OcTree::num_nodes`] is upstream's
//! `calcNumNodes()` -- an O(n) recursive traversal -- not `size()`, the O(1)
//! `tree_size` counter upstream maintains incrementally across every
//! insert/delete/prune/expand, checked directly against
//! `OcTreeBaseImpl.h:241` (`size()`) versus `OcTreeBaseImpl.h:269`
//! (`calcNumNodes()`) rather than assumed from the Rust name alone.
//!
//! **`size()`/`tree_size`: NO-GO, decided (round 13, item 2).** Its one
//! `moveit_core`-reachable caller is `collision_detection_bullet/src/
//! bullet_integration/bullet_utils.cpp:209`'s `geom->octree->size()`
//! (sizing a `btCompoundShape`'s child capacity) -- the same file whose
//! `getOccupancyThres()` call at line 210 is this port's other confirmed
//! Bullet-only consumer. `collision_detection_bullet` (4,278 LOC) is
//! dropped by PORTING-PLAN.md outright, folded into the single
//! `parry3d-f64` backend that replaces both FCL and Bullet -- so `size()`'s
//! only known upstream consumer does not survive the port under any crate.
//! Porting it anyway would mean giving every mutation path
//! (`update_node`/`prune`/`expand`/their `_by_key` siblings) a
//! single-owner incremental counter to maintain for a value nothing reads.
//! Not ported. Reopens if a future `parry3d-f64` collision backend (or any
//! other consumer) needs O(1) node-count introspection -- at that point
//! `tree_size` is an invariant with one owner across every mutation path,
//! not a bare getter, per this project's structure-over-patch rule.
//!
//! # Representation
//!
//! See [`OcTree`]'s own docs for why the tree is pointer-linked nodes
//! (`Box`-owned) rather than a flat keyed map: `prune`'s lossless
//! multi-resolution collapse and leaf iteration's per-leaf depth both need a
//! notion of tree level a flat map has no way to express.
//!
//! # Completion statement (round 16, item 2)
//!
//! Every number below is a command someone can re-run -- same model as
//! `moveit-scene`'s completion statement, commit `08ab3c7`.
//!
//! **Symbol audit.** Superseded round 15's "24 ported / 2 unported-in-scope
//! / 15 distinct / 41 symbol groups" -- "symbol group" was never a defined,
//! reproducible unit. [`OcTree`]'s own module doc now carries a full
//! literal, one-bullet-per-declaration walk against the five headers that
//! make up the upstream inheritance chain, in the same bullet-per-line
//! format `moveit-scene`'s `planning_scene.hpp` audit uses --
//! `rg -c '^/// - \`' crates/moveit-octomap/src/tree.rs` over that walk's
//! line range reproduces **159** bullets (8 of them are non-symbols or
//! cross-references to a declaration already tallied elsewhere in the
//! walk, see [`OcTree`]'s doc for which); the remaining 151 audited
//! bullets are 55 ported, 8 unported-in-scope (all named there, each with
//! the concrete call site it would need), 88 architecturally distinct.
//! That walk found two symbols no prior table ever classified at all --
//! round 12's table never named `isNodeAtThreshold` (both overloads),
//! and round 16's own fresh table never named `OcTreeBaseImpl`'s own
//! concrete `getTreeType()` (round 17 item 2's correction) -- the same
//! class of stale-audit drift round 15 found for `tree_iterator`,
//! recurring inside the very table built to fix it.
//!
//! **Tests.**
//!
//! ```text
//! cargo nextest run -p moveit-octomap --no-fail-fast   # 30 tests run: 30 passed, 0 skipped
//! rg -c '#\[test\]' crates/moveit-octomap/src/*.rs      # sums to 28
//! ```
//!
//! 30 total: 28 unit tests inside `src/` (per-invariant-boundary, e.g.
//! [`OcTree`]'s own clamp/threshold/prune boundary tests, plus round 16
//! item 1's `set_prob_hit_below_half_panics_in_debug`/
//! `set_prob_miss_above_half_panics_in_debug`) plus 2 oracle-backed
//! integration tests. The first,
//! `octomap_matches_liboctomap_for_every_boundary_scenario`
//! (`tests/octomap_parity.rs`), which replays
//! `python3 -c "import json; print(len(json.load(open('tests/fixtures/octomap_request.json'))))"`
//! -- **12** request/response pairs (from
//! `crates/moveit-octomap/tests/fixtures/`) against this crate's own
//! [`OcTree`] and compares every result field-by-field, including
//! [`OcTree::is_occupied`], against the real `liboctomap.so.1.9.7`'s answer
//! captured through the `moveit-rs` oracle. Ids 1-7 predate round 15; ids
//! 8-12 (round 15, item 1) each isolate one of the five sensor-model setters'
//! effect against oracle ground truth. `tools/ci/verify-fixture-replay.sh`
//! (docker-gated, not part of this count) independently confirms the
//! committed fixture still reproduces against a freshly built oracle image
//! rather than only against a stale capture.
//!
//! The second, `leaves_matches_liboctomap_leaf_iterator_order_and_fields`
//! (`tests/leaves_parity.rs`, round 18 item 3), closes the one audit item
//! this round's survey found still closed only by argument rather than
//! measurement: [`OcTree::leaves`]'s pre-order sibling ordering was inferred
//! from sharing `push_children` with [`OcTree::tree_nodes`] (whose own order
//! *is* oracle-measured via `tree_walk`), not measured against upstream's
//! actual `leaf_iterator` class itself. The new `octree_points` oracle op
//! (added this round for `moveit-distance-field`) exposes a `leaves` field
//! that is exactly a `tree.begin_leafs()` walk, independent of that op's own
//! distance-field-specific purpose -- cheap ground truth this crate did not
//! have to ask the orchestrator to add. Surveyed and found already
//! measured, not argued: `isNodeOccupied`'s threshold
//! ([`OcTree::is_node_occupied_log_odds`], pinned via `octomap_parity.rs`'s
//! `set_occupancy_thres` scenario) and every field `tree_walk` already
//! covers (coordinate/size/depth/is_leaf/log_odds/occupancy for the full
//! pre-order node walk). `getTreeType()` is not ported as a callable symbol
//! at all (see the symbol-by-symbol audit above), so there is nothing there
//! for an oracle op to confirm.
//!
//! **`assert_relative_eq!` reckoning (round 18, item 2).** This crate has
//! **zero** calls, not counted by `rg -c` (which mixes doc-comment mentions
//! into the total, the exact class PORTING-PLAN.md §73.1/§83.3/§92/§104.1
//! got bitten by four times) but confirmed two ways: `grep -n approx
//! crates/moveit-octomap/Cargo.toml` has no match -- this crate never took
//! the `approx` dependency `moveit-geometry` did -- and `moveit-geometry`'s
//! own `//`-tail-stripped, paren-bracket-matched scanner (see that crate's
//! completion statement) confirms it by running clean against this crate
//! too:
//!
//! ```text
//! perl crates/moveit-geometry/audit/count_relative_eq.pl crates/moveit-octomap/src/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Nothing to classify into epsilon-only/max_relative-only/both/neither;
//! nothing to bisect.
//!
//! **Round 19, item 1.** `count_relative_eq.pl` and
//! `audit/count_public_declarations.sh` (this crate's own copy) both had a
//! doc-comment/string-literal filtering gap this round found and fixed --
//! see `moveit-geometry`'s completion statement for the self-count evidence
//! and the fix, since the `.pl` script lives there and this crate's copy of
//! the `.sh` script is byte-identical. Neither bug changed any count already
//! committed in this file or in `tree.rs`.

mod iter;
mod key;
mod node;
mod tree;

pub use iter::{Leaf, Leaves, LeavesInBbx, TreeNode, TreeNodes};
pub use key::{KeyRay, KeySet, KeyType, OcTreeKey};
pub use tree::OcTree;
