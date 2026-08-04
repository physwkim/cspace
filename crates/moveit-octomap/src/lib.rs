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
//! **Round 19, item 3 -- re-surveyed exhaustively, not re-answered with the
//! same one item.** `octree_points`'s `leaves` field closes exactly the one
//! gap above (`leaves_matches_liboctomap_leaf_iterator_order_and_fields`) and
//! nothing further in this crate's own audit: every other `argued`-tagged
//! item from round 18's survey was already re-checked and found
//! already-measured (previous paragraph), and `rg -n "argued|not measured|
//! inferred|by argument"` across both crates' `src/` this round surfaces no
//! new candidate beyond what round 18 already covered.
//!
//! One related item surveyed and found genuinely still argued-only, but NOT
//! closeable by `octree_points` as it exists today:
//! [`OcTree::leaves_in_bbx`] (`LeavesInBbx`, upstream `leaf_bbx_iterator`,
//! public in `OcTreeIterator.hxx`, reachable the same way `leaf_iterator`
//! is). Its own unit tests
//! (`leaves_in_bbx_only_yields_leaves_overlapping_the_box`,
//! `leaves_in_bbx_returns_none_for_an_out_of_range_corner`) are
//! self-consistency only. `moveit-distance-field`'s own `octree_points`
//! function calls [`OcTree::leaves_in_bbx`] internally, and its output is
//! oracle-verified bit-for-bit by that crate's own
//! `octree_points_matches_the_oracle_for_all_three_pinned_boundary_cases`
//! -- real, but indirect: that test pins the *subdivided point list* a
//! consumer builds from `leaves_in_bbx`'s output, not
//! `leaves_in_bbx`'s own key/coordinate/size/occupied fields or its
//! traversal order field-by-field, the way `leaves_parity.rs` now does for
//! the unrestricted walk. `octree_points`'s oracle op always walks
//! `tree.begin_leafs()`/`end_leafs()` unrestricted (`oracle.cpp`'s
//! `octreePoints`) -- it has no bbox parameter, so it cannot directly
//! exercise `begin_leafs_bbx()`/`end_leafs_bbx()` the way it now does for
//! the plain walk. **Oracle extension request, text only (not applied --
//! `tools/moveit-oracle/` is the orchestrator's):** an optional `bbx: {min:
//! [f64;3], max: [f64;3]}` field on the `octree_points` request that, when
//! present, walks `tree.begin_leafs_bbx(min, max)`/`end_leafs_bbx()` instead
//! of the unrestricted walk for the `leaves` output, so a
//! `leaves_in_bbx_parity.rs` symmetric to `leaves_parity.rs` becomes
//! possible.
//!
//! **`getTreeType()` gap, re-checked in the current tree (round 19, item
//! 4).** Tagged round 17, diffed against `geometric_shapes` 2.3.3 at the
//! time. Re-verified now rather than carried forward on last round's word
//! (§113.3): that gap was an *audit-classification* gap, not a porting
//! gap -- `OcTreeBaseImpl.h`'s own concrete `getTreeType() const` (header
//! line 104) was absent from every symbol-audit walk through round 16, not
//! a missing port. It was closed in round 17 itself, commit `8313f91`
//! ("octomap: cross-check the remaining four headers, fix a getTreeType
//! gap"), merged `c46b4f6`, recorded `PORTING-PLAN.md` §111.2. Confirmed
//! still closed in the current tree, not just in that commit message:
//!
//! ```text
//! rg -c '^/// - `' crates/moveit-octomap/src/tree.rs   # 159, matches §111.2's corrected total
//! rg -n '^/// - `getTreeType' crates/moveit-octomap/src/tree.rs
//! #  182: - `getTreeType() const` -- distinct, same registry reasoning.
//! #  294: - `getTreeType() const` (concrete, returns `"OcTreeBaseImpl"`) -- already counted above, ...
//! ```
//! Both bullets classify it `distinct` (D4 rules out the `AbstractOcTree`
//! runtime-type registry `create()`/`getTreeType()` exist for; this crate
//! has exactly one concrete tree type), cross-referenced correctly between
//! `OcTree.h`'s override and `OcTreeBaseImpl.h`'s shadowed base
//! declaration, counted once. No `unported` classification appears near
//! either bullet. Nothing to change; the round-18-era sentence above
//! ("`getTreeType()` is not ported as a callable symbol at all ... nothing
//! there for an oracle op to confirm") was already correct and still is.
//!
//! **Tolerance-floor re-measurement mandate, this crate: none.** Commit
//! `70a6b31` fixed the workspace's `serde_json` to `float_roundtrip`
//! because the default parser returned 6,859/84,221 (8.1%) committed
//! fixture float literals one ULP off, and "tolerances hide it... it sets
//! the floor every bisection here measures against" (that commit's own
//! body). This crate has zero `assert_relative_eq!`/`relative_eq!` calls
//! (previous paragraph) -- nothing bisected in `src/` to re-check. The one
//! place this crate compares against `serde_json`-parsed oracle data is
//! `tests/octomap_parity.rs`'s `LOG_ODDS_EPS`/`OCCUPANCY_EPS`, unchanged
//! since their introduction commit (`git log -p --follow` on that file,
//! checked this round) and each with a stated analytic rationale (`f32`
//! log-odds rounding; `f64` `probability()` arithmetic propagating that
//! same rounding) rather than an empirically bisected minimum -- not "the
//! floor every bisection measures against" in the sense `70a6b31` warns
//! about. Nothing to re-measure.
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
//! perl tools/ci/count-relative-eq.pl crates/moveit-octomap/src/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Nothing to classify into epsilon-only/max_relative-only/both/neither;
//! nothing to bisect.
//!
//! **Round 19, item 1.** `count_relative_eq.pl` and
//! `tools/ci/count-public-declarations.sh` (this crate's own copy) both had a
//! doc-comment/string-literal filtering gap this round found and fixed --
//! see `moveit-geometry`'s completion statement for the self-count evidence
//! and the fix, since the `.pl` script lives there and this crate's copy of
//! the `.sh` script is byte-identical. Neither bug changed any count already
//! committed in this file or in `tree.rs`.
//!
//! **§79 recount (round 19, item 2).** Re-run fresh against the fixed
//! script:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-octomap/src/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Unchanged: still 0 in every bucket, nothing to dispose.
//!
//! **`LeavesInBbx` split, round 21 (PORTING-PLAN.md §123.2).**
//! `p3-distance-field` confirmed the precise gap: `octree_points`'s
//! consumer (`distance_field.rs`) reads only three of [`Leaf`]'s eight
//! accessors (`is_occupied`, `coordinate`, `size`); its own emission-order
//! *transfer* question (does this crate's `push_children` output what
//! `octree_points` expects) is closed on that side. Two pieces remained
//! here:
//!
//! 1. **Does [`LeavesInBbx`]'s leaf-to-leaf order match upstream
//!    `leaf_bbx_iterator`'s?** **Closed this round.** The "oracle extension
//!    request" paragraph above already named this gap; re-confirmed this
//!    round with a precise citation fix -- `begin_leafs_bbx`/`end_leafs_bbx`
//!    are declared `public` at `OcTreeBaseImpl.h:337-345` (not merely
//!    "public in `OcTreeIterator.hxx`", the file that defines
//!    `leaf_bbx_iterator` itself but not the tree-side entry points a
//!    caller reaches it through), confirmed by reading the header directly,
//!    satisfying PORTING-PLAN.md §107.3 before asking. Reading
//!    `leaf_bbx_iterator::singleIncrement` directly (same file) shows its
//!    descent is `leaf_iterator`'s exact reverse-index child-push order
//!    with one added guard -- a per-child bounding-box overlap test -- so
//!    it was tempting to derive the expected bbx-restricted order by
//!    filtering `leaves_parity.rs`'s already-oracle-captured id-2 leaf list
//!    client-side instead of waiting on a new oracle capture. Deliberately
//!    **not done that way**: this project's own repeated position is that
//!    reading upstream source and reasoning from it is not a substitute
//!    for a real `liboctomap.so` execution (this round's own
//!    `moveit-planners-stomp` citation mistake is a fresh instance of that
//!    exact failure mode), so a real capture was requested via a
//!    `caucus signal note --kind question` instead, restating the same
//!    optional `bbx: {min: [f64;3], max: [f64;3]}` field on `octree_points`
//!    (emitting a `leaves_bbx` field shaped like `leaves`) and proposing
//!    the concrete capture: mirror `leaves_request.json`'s id-2 actions
//!    (four octant leaves, 0.1 resolution) under a new id with `bbx.min =
//!    (-0.5, -0.5, -0.5)`, `bbx.max = (-0.02, 0.5, 0.5)` -- clear of every
//!    leaf's own voxel boundary (each spans exactly `[-0.1, 0.0]` or `[0.0,
//!    0.1]` per axis), so the two leaves with x-center `-0.05` overlap and
//!    the two with x-center `0.05` do not, with no rounding-boundary
//!    ambiguity for either side to disagree on. The orchestrator implemented
//!    exactly that request against the real oracle (`octree_points`'s
//!    `bbx`/`leaves_bbx` fields via `begin_leafs_bbx`/`end_leafs_bbx`,
//!    verifying the same `OcTreeBaseImpl.h:337-345` `public` declaration
//!    independently inside the built image) and captured the proposed
//!    geometry as id 3, oracle stamp `cd8ee2c1bdcf7148` →
//!    `8ed8a9395b730b08`: `leaves` (unfiltered) emits all four leaves in the
//!    same order as id 2; `leaves_bbx` emits only the two `x < 0` leaves,
//!    `(-0.05,-0.05,-0.05)` then `(-0.05,0.05,0.05)`, in that same relative
//!    order -- both the filter and the order are exercised by one capture.
//!    Landed as id 3 of `leaves_request.json`/`leaves_response.json` and
//!    checked by
//!    `leaves_parity.rs::leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`,
//!    which also guards against a vacuous order check: it asserts the two
//!    `leaves_bbx` leaves have distinct coordinates (or an indexed pairwise
//!    comparison couldn't distinguish their order at all) and confirms, by
//!    swapping the expected pair and checking the comparison then fails,
//!    that an order-reversing regression in [`OcTree::leaves_in_bbx`] would
//!    actually be caught.
//!
//! 2. **`key()`/`index_key()`/`depth()`/`log_odds()`/`occupancy()` have no
//!    consumer anywhere in this workspace -- is fixture coverage for them
//!    on [`Leaf`] worth adding anyway?** Decided **no** this round, for a
//!    reason stronger than "no consumer, skip" alone (PORTING-PLAN.md's own
//!    framing of that answer requires the reason, not just the absence of
//!    a caller): all five are non-virtual, non-overridden accessors on
//!    upstream's own shared `iterator_base` base class (`getKey`,
//!    `getIndexKey`, `getDepth`) or on the dereferenced `OcTreeNode` itself
//!    (`getLogOdds`, `getOccupancy` via `operator*`) -- read directly in
//!    `OcTreeIterator.hxx`: neither `leaf_iterator` nor `leaf_bbx_iterator`
//!    nor `tree_iterator` redefines any of them. That is a materially
//!    different situation from `leaf_iterator`/`tree_iterator`'s emission
//!    *order*, which each subclass genuinely reimplements in its own
//!    `singleIncrement` -- the exact reason §123.2 refused to let one
//!    iterator's pinned order stand in for the other's. `depth()`,
//!    `log_odds()`, and `occupancy()` are already oracle-pinned, field by
//!    field, on [`TreeNode`] (structurally the same three accessors, over
//!    the same `OcTree`/`Node` data) via `octomap_parity.rs`'s `tree_walk`
//!    case; because upstream's implementation is the identical shared
//!    method regardless of which concrete iterator produced the node, that
//!    coverage transfers to [`Leaf`]'s copies directly -- unlike the
//!    §123.2 case, this is not an inference from this port's own code
//!    reuse, it is upstream's own code being literally the same function.
//!    `key()` has no oracle field on *either* type (neither `tree_walk` nor
//!    `octree_points` emits raw keys), but is evidenced indirectly:
//!    [`Leaf::coordinate`]/[`TreeNode::coordinate`] (both oracle-pinned)
//!    compute `key_to_coord_at_depth(key, depth)` directly from the same
//!    `key`/`depth` pair, so a wrong `key()` would need to be wrong in a
//!    way `key_to_coord`'s specific arithmetic happens not to expose --
//!    named here as the one residual, non-airtight gap in this reasoning,
//!    not hidden. `index_key()` calls `key::compute_index_key`, which is
//!    verified directly against upstream's literal formula (`mask =
//!    65535 << level`, `OcTreeKey.h:227-236`) by
//!    `compute_index_key_masks_low_bits` in `key.rs` -- a pure integer
//!    mask operation with no cross-language floating-point ambiguity, so a
//!    translation-fidelity unit test against the read formula is adequate
//!    ground truth here without needing an oracle round-trip. No new
//!    fixture added for any of the five.

mod iter;
mod key;
mod node;
mod tree;

pub use iter::{Leaf, Leaves, LeavesInBbx, TreeNode, TreeNodes};
pub use key::{KeyRay, KeySet, KeyType, OcTreeKey};
pub use tree::OcTree;
