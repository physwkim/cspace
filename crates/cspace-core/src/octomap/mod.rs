// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (Debian package liboctomap-dev 1.9.7+dfsg-3.1build3,
// version confirmed by octomap-config.cmake's OCTOMAP_VERSION inside the
// moveit-rs oracle container). This crate root re-exports iter/key/node/tree
// and carries no ported logic of its own -- see each module's own provenance
// comment for its exact octomap header citations; this one names the header
// shared by the whole crate's addressing scheme:
//   include/octomap/OcTreeKey.h

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
//!   (`compute_update`), `insert_ray`, tree-wide `prune`, the lazy-eval
//!   companion `update_inner_occupancy`, and (round 33)
//!   `moveit_msgs::Octomap.data` decode for both wire formats
//!   ([`OcTree::read_binary_data`], [`OcTree::read_data`]).
//! - `error` (crate-private module): [`DecodeError`], the typed failure of
//!   the two decode entry points above.
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
//!   `cspace_core::geometry`'s `shapes::OcTree` was still a stub deferred to
//!   Phase 3 collision; it is not -- `shapes::OcTree` has been fully ported
//!   since round 3, and there is no `bodies::`-level posed counterpart for
//!   an octree upstream at all (`bodies::createBodyFromShape` returns
//!   `nullptr` for `shapes::OCTREE`; this crate's own `Body::from_shape`
//!   matches that with `Shape::OcTree => None`, see `bodies.rs`). See
//!   `cspace_core::geometry`'s `shapes.rs`, "Who consumes `Shape::OcTree`" for the
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
//! same literal-extraction style `cspace_planning::scene`'s `planning_scene.hpp`
//! audit (commit `943e909`) uses. That walk also states, in one line, the
//! rule for collapsing the five-level inheritance chain into "this crate's
//! exposed surface" before counting, so a future round can re-verify the
//! count mechanically rather than trusting a judgment call.
//!
//! ```text
//! ported                58
//! unported, in scope     8
//! distinct               85
//! ------------------------
//! total                 151
//! ```
//!
//! (Round 33: `readBinaryData`/`readBinaryNode`/`readData` moved from
//! `distinct` to `ported`, 55 -> 58 / 88 -> 85; see [`OcTree`]'s own doc for
//! the three updated bullets and "Round 27, item 1(a)" below for the format
//! these three decode.)
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
//! `cspace_planning::scene`'s completion statement, commit `08ab3c7`.
//!
//! **Symbol audit.** Superseded round 15's "24 ported / 2 unported-in-scope
//! / 15 distinct / 41 symbol groups" -- "symbol group" was never a defined,
//! reproducible unit. [`OcTree`]'s own module doc now carries a full
//! literal, one-bullet-per-declaration walk against the five headers that
//! make up the upstream inheritance chain, in the same bullet-per-line
//! format `cspace_planning::scene`'s `planning_scene.hpp` audit uses --
//! `rg -c '^/// - \`' crates/cspace-core/src/octomap/tree.rs` over that walk's
//! line range reproduces **159** bullets (8 of them are non-symbols or
//! cross-references to a declaration already tallied elsewhere in the
//! walk, see [`OcTree`]'s doc for which); the remaining 151 audited
//! bullets are, as of round 33, 58 ported, 8 unported-in-scope (all named
//! there, each with the concrete call site it would need), 85
//! architecturally distinct (round 16's original count was 55/8/88; round
//! 33 moved `readBinaryData`/`readBinaryNode`/`readData` from `distinct` to
//! `ported`, see "Round 27, item 1(a)" below).
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
//! cargo nextest run -p cspace-core --no-fail-fast   # 48 tests run: 48 passed, 0 skipped
//! rg -c '#\[test\]' crates/cspace-core/src/octomap/*.rs      # sums to 44
//! ```
//!
//! **Round 33.** 44 in-source unit tests (up from 28: round 33 added
//! [`OcTree::read_binary_data`]/[`OcTree::read_data`]'s own boundary tests)
//! plus 4 oracle-backed integration tests (`octomap_parity`, two in
//! `leaves_parity`, and the new `decode_parity`) is the 48 nextest reports.
//!
//! **Stale count fixed this round (round 27).** This section previously
//! read "30 total ... plus 2 oracle-backed integration tests", left
//! unreconciled since round 21 added a second `leaves_parity.rs` test
//! (`leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`,
//! see "`LeavesInBbx` split, round 21" below) without updating this
//! earlier-written total -- caught only because item 1(a)/1(b)'s own gate
//! run this round surfaced 31, not the 30 this section still claimed. 31
//! total: 28 unit tests inside `src/` (per-invariant-boundary, e.g.
//! [`OcTree`]'s own clamp/threshold/prune boundary tests, plus round 16
//! item 1's `set_prob_hit_below_half_panics_in_debug`/
//! `set_prob_miss_above_half_panics_in_debug`) plus 3 oracle-backed
//! integration tests. The first,
//! `octomap_matches_liboctomap_for_every_boundary_scenario`
//! (`tests/octomap_parity.rs`), which replays
//! `python3 -c "import json; print(len(json.load(open('tests/fixtures/octomap_request.json'))))"`
//! -- **12** request/response pairs (from
//! `crates/cspace-core/tests/fixtures/octomap/octomap/`) against this crate's own
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
//! (added this round for `cspace_collision::distance_field`) exposes a `leaves` field
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
//! The third, `leaves_in_bbx_matches_liboctomap_leaf_bbx_iterator_order_and_fields`
//! (`tests/leaves_parity.rs`, round 21), closes [`OcTree::leaves_in_bbx`]'s
//! own leaf-to-leaf order and field parity the same way the second test
//! does for the unrestricted walk; see "`LeavesInBbx` split, round 21"
//! below for the full gap this closed and how the fixture was captured.
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
//! self-consistency only. `cspace_collision::distance_field`'s own `octree_points`
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
//! rg -c '^/// - `' crates/cspace-core/src/octomap/tree.rs   # 159, matches §111.2's corrected total
//! rg -n '^/// - `getTreeType' crates/cspace-core/src/octomap/tree.rs
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
//! crates/cspace-core/Cargo.toml` has no match -- this crate never took
//! the `approx` dependency `cspace_core::geometry` did -- and `cspace_core::geometry`'s
//! own `//`-tail-stripped, paren-bracket-matched scanner (see that crate's
//! completion statement) confirms it by running clean against this crate
//! too:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/cspace-core/src/octomap/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Nothing to classify into epsilon-only/max_relative-only/both/neither;
//! nothing to bisect.
//!
//! **Round 19, item 1.** `count_relative_eq.pl` and
//! `tools/ci/count-public-declarations.sh` (this crate's own copy) both had a
//! doc-comment/string-literal filtering gap this round found and fixed --
//! see `cspace_core::geometry`'s completion statement for the self-count evidence
//! and the fix, since the `.pl` script lives there and this crate's copy of
//! the `.sh` script is byte-identical. Neither bug changed any count already
//! committed in this file or in `tree.rs`.
//!
//! **§79 recount (round 19, item 2).** Re-run fresh against the fixed
//! script:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/cspace-core/src/octomap/*.rs
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
//!    `cspace_planners::stomp` citation mistake is a fresh instance of that
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

//! # Round 27, item 1(a): `moveit_msgs::Octomap.data`'s wire format
//!
//! p9-ros is blocked on decoding `moveit_msgs::Octomap.data` (empty-payload
//! no-op, error on non-empty, "별도 기능 규모" -- an unmeasured sizing
//! judgment). This crate owns octomap; answering it is this round's item
//! 1(a). The `readBinaryData`/`writeBinaryData`/`readBinaryNode`/
//! `writeBinaryNode`/`readData`/`writeData` bullets in [`OcTree`]'s own
//! audit above were, until this round, classified `distinct` on the
//! premise "octrees enter this workspace only via ROS messages, never
//! `.bt`/`.ot` files" -- backwards, corrected in place there. What follows
//! is what was actually read to correct it.
//!
//! **Source.** `octomap/OccupancyOcTreeBase.hxx` and
//! `octomap/OcTreeBaseImpl.hxx` are not in either header cache this crate
//! has used before (`OccupancyOcTreeBase.h` only forward-declares these
//! methods and `#include`s the `.hxx` at its own line 506 for the template
//! body) -- originally fetched from inside the `moveit-rs` oracle container
//! (`moveit-rs/oracle:a75076d8ca13d25b`,
//! `/usr/include/octomap/{OccupancyOcTreeBase,OcTreeBaseImpl,OcTreeDataNode}.hxx`,
//! same package already pinned by this crate: `dpkg -s liboctomap-dev` in
//! that container gave `1.9.7+dfsg-3.1build3`, matching `key.rs`'s existing
//! provenance). octomap 1.9.7 (tag `v1.9.7`) is now vendored at
//! `third_party/octomap/octomap/` and every citation below has been
//! re-opened and re-measured directly against that checkout, not carried
//! forward from the container fetch on trust -- paths below are
//! repo-relative from there. The ROS-side wiring
//! (`octomap_msgs::msg::Octomap`, `octomap_msgs/conversions.h`) is not
//! vendored (`third_party/` has `geometric_shapes`, `srdfdom`, `octomap`,
//! not `octomap_msgs`), so that half of this citation still comes from the
//! oracle container: `/opt/ros/rolling/include/octomap_msgs/octomap_msgs/
//! conversions.h`, package `ros-rolling-octomap-msgs
//! 2.0.1-1noble.20260113.095330` (`dpkg -s`), matching `package.xml`'s
//! `<version>2.0.1</version>`.
//!
//! **`Octomap.msg`:**
//!
//! ```text
//! std_msgs/Header header
//! bool binary        # true: compact free/occupied only (.bt); false: full probabilities (.ot)
//! string id          # octree class, e.g. "OcTree" or "ColorOcTree"
//! float64 resolution
//! int8[] data         # see below -- header-less, no magic bytes, no length prefix
//! ```
//!
//! `conversions.h`'s `binaryMapToMsg`/`fullMapToMsg` (the functions that
//! actually fill this message, not the also-present but message-unused
//! `binaryMapToMsgData`/`fullMapToMsgData` pair that wrap a file-style
//! header via `writeBinaryConst`/`write`) set `id`/`resolution`/`binary`
//! from separate accessors and write `data` as the *raw* output of
//! `writeBinaryData`/`writeData` with **no header, no magic bytes, no
//! length prefix, no root value for the binary case** -- every byte of
//! `data` is tree structure, decodable only in the context of `resolution`
//! and `id` the sibling fields already carry.
//!
//! **`binary == true` (`third_party/octomap/octomap/include/octomap/
//! OccupancyOcTreeBase.hxx`: `readBinaryData` 931-943, `writeBinaryData`
//! 946-951, `readBinaryNode` 954-1022, `writeBinaryNode` 1025-1086 --
//! function-body boundaries taken by brace depth, not by eyeballed line
//! range):** the root node's own value is
//! never written; `writeBinaryData` calls `writeBinaryNode(s, root)`
//! directly. `writeBinaryNode` emits exactly 2 bytes per node it is called
//! on: 8 children packed 2 bits each (`std::bitset<8>` split into two
//! `char`s, children 0-3 then 4-7), each pair meaning `00` no child, `10`
//! child is a free leaf, `01` child is an occupied leaf, `11` child has its
//! own children (recurse into it *after* this node's own 2 bytes, depth
//! first, same child-index order 0..7). A child classified free/occupied
//! is reconstructed on read as exactly `clamping_thres_min`/
//! `clamping_thres_max` -- **the actual log-odds value is not preserved,
//! only which side of the occupied/free split it fell on**; a child with
//! grandchildren is read back with a `-200.` sentinel log-odds, corrected
//! after its subtree is read to `getMaxChildLogOdds()` (its own children's
//! max, `Node::update_occupancy_from_children`'s exact upstream
//! counterpart). `readBinaryData` refuses to read into a tree that already
//! has a root (`this->root` non-null is an error, not silently merged).
//!
//! **`binary == false` (`third_party/octomap/octomap/include/octomap/
//! OcTreeBaseImpl.hxx`: `writeData` 763-768, `writeNodesRecurs` 771-798,
//! `readData` 801-821, `readNodesRecurs` 824-844; `OcTreeDataNode.hxx`:
//! `readData` 114-117, `writeData` 121-124):** per node, depth first: the
//! node's raw `value` (`f32`, `sizeof(float)` bytes, a direct
//! `s.write((char*) &value, sizeof(value))` memory dump -- native-endian.
//! Checked, not assumed: `rg -ni endian third_party/octomap/octomap/
//! include/octomap/ third_party/octomap/octomap/src/` has zero hits
//! anywhere in upstream octomap, so there genuinely is no explicit
//! endianness contract to read, only the native-endian fact the memory
//! dump itself implies -- every platform this workspace's CI or the oracle
//! container runs on is little-endian, so `f32::from_le_bytes` is the
//! correct read), then 1 byte with 1
//! bit per child (bit set = child exists, recurse into existing children in
//! index order after the byte). No 2-bit quantization here: every node's
//! exact log-odds survives the round trip, at the cost of `sizeof(float) +
//! 1` bytes per node instead of a shared 2 bytes per *parent*.
//!
//! **Can [`OcTree`]/`Node` hold what's decoded?** Yes, at
//! the type level, for both variants, with no new field: `Node`
//! is already exactly `{ log_odds: f32, children: Option<Box<[Option<Node>;
//! 8]>> }` (`node.rs`) -- the same shape both formats serialize (a per-node
//! `f32` plus up to 8 present/absent children), and [`OcTree`] already
//! carries `resolution`/`clamping_thres_min`/`clamping_thres_max` as its
//! own fields (needed by the binary-path reconstruction above). The actual
//! gap was not representation, it was that **no decode function existed**
//! -- closed round 33, [`OcTree::read_binary_data`]/[`OcTree::read_data`].
//! The encode direction, deferred round 33 as out of brief, was ported this
//! round too: [`OcTree::write_binary_data`]/[`OcTree::write_data`], pinned
//! byte-for-byte against the oracle's own `serialize` output rather than
//! only round-tripped through this crate's own decoder (see
//! `tests/encode_parity.rs`). The decoder/encoder both live inside this
//! crate (not e.g. `cspace-ros`), because they need `Node::create_child`/
//! `Node::child`/`log_odds` directly, and all are `pub(crate)`, not
//! exported (`lib.rs`'s `pub use` list has neither `Node` nor a way to
//! construct an [`OcTree`] from an already-built tree). A decoder would
//! also need to reject `id !=
//! "OcTree"` (`"ColorOcTree"` is not ported, see "What was deliberately not
//! ported" above) -- note this only matters for the `binary == false` path:
//! `ColorOcTreeNode::writeData` appends 3 extra color bytes per node beyond
//! `OcTreeDataNode::writeData`'s `f32`, but the `binary == true` path never
//! touches node color at all, so a plain-`OcTree` binary decoder happens to
//! read a `ColorOcTree`'s compact payload correctly regardless of `id`
//! (structure-only, not verified for the full-data path). The encoder has
//! no such concern at all: it only ever writes a plain `OcTree`'s own
//! fields, never a `ColorOcTree`'s.
//!
//! **Size estimate, corrected (§161): state the unit, not just a number.**
//! The paragraph below originally gave "~215 lines of upstream C++" from
//! eyeballed ranges and a "roughly 650-800 lines" Rust-implementation
//! estimate, placed next to p9-ros's own "~130 lines" as if the two were
//! the same measurement -- they were never commensurable (upstream
//! algorithm size versus estimated Rust output including tests and error
//! handling), and presenting them side by side read as a size
//! disagreement that was not actually one. With `third_party/octomap` now
//! vendored, the upstream number is exact rather than eyeballed: the
//! *read* path this crate needs first (`readBinaryData` 13 +
//! `readBinaryNode` 69 + `OcTreeBaseImpl::readData` 21 +
//! `readNodesRecurs` 21 + `OcTreeDataNode::readData` 4, brace-depth
//! function-body boundaries, see the two paragraphs above for exact
//! line ranges) is **128 lines of upstream C++** -- matching p9-ros's own
//! ~130 exactly, now confirmed rather than coincidental. Adding the write
//! path (`writeBinaryData` 6 + `writeBinaryNode` 62 +
//! `OcTreeBaseImpl::writeData` 6 + `writeNodesRecurs` 28 +
//! `OcTreeDataNode::writeData` 4) gives **234 lines of upstream C++** for
//! both directions combined.
//!
//! Separately, and not comparable to the number above: this crate's own
//! established ratio of Rust source to the upstream it ports (`node.rs`:
//! 132 lines of implementation against ~40 lines of the upstream headers
//! it ports, roughly 3:1) applied to the 234-line upstream figure, once
//! `Result`-based error handling is counted (upstream silently leaves a
//! truncated read as a `std::istream` fail-bit the caller may never
//! check; a Rust decoder over `&[u8]` must return `Result` at every
//! recursive step instead of panicking on a short buffer), gives
//! **roughly 400-500 lines** of Rust implementation and docs for both
//! decode and encode paths together. Using `node.rs`'s own measured
//! test-to-implementation ratio (81 lines of test code against 132 lines
//! of implementation, ~0.6:1) for per-invariant-boundary coverage (empty
//! tree, single leaf, one full 8-child level, nested recursion, truncated
//! buffer, wrong `id`, and an oracle-captured golden fixture round-trip
//! comparable to `octomap_parity.rs`'s existing pattern) adds **roughly
//! 250-300 more lines** of test code. Total: **roughly 650-800 lines of
//! Rust** (implementation + docs + tests), a distinct unit from the
//! 128/234-line upstream C++ figures above, both stated as ranges because
//! neither has been built -- not a single unmeasured number, and not
//! "별도 기능 규모". §157's decision stands: the decoder lives in this
//! crate, `Node`/`OcTree::root` do not become public; 128-234 lines of
//! upstream algorithm is a size this crate absorbs directly, not a reason
//! to change that boundary.
//!
//! # Round 27, item 1(b): `refineContactNormals`'s octomap operations
//!
//! p3-acm will next port `collision_octomap_filter.cpp`
//! (`moveit_core/collision_detection/src/collision_octomap_filter.cpp`,
//! moveit2 `e017c91e`, 318 lines) -- `cspace-collision`'s exclusion note for
//! it cites "needs an octomap dependency and `RobotState`"
//! (PORTING-PLAN.md §153), which §153 already found half wrong (zero
//! `RobotState` references) and half expired (`cspace_core::octomap` now exists).
//! §153.2 asks this crate's owner to check whether the octomap operations
//! `refineContactNormals` actually calls already exist here. They do, in
//! full -- reading the whole file (not just the entry point) turns up
//! exactly four octomap-touching calls, all in `refineContactNormals`
//! itself (`:67-160`); its other three free functions
//! (`getMetaballSurfaceProperties`, `findSurface`, `sampleCloud`, the
//! Wyvill-metaball implicit-surface math, `:162-318`) take
//! `octomap::point3d_list`/`octomath::Vector3` by value and call no octomap
//! API at all -- pure numerical porting, not an octomap-surface question:
//!
//! - `octree->getResolution()` (`:113`) -- [`OcTree::resolution`].
//! - `octree->begin_leafs_bbx(bbx_min, bbx_max)` /
//!   `octree->end_leafs_bbx()` (`:120-123`, the `point3d`-bounded overload,
//!   default depth 0) -- [`OcTree::leaves_in_bbx`] /
//!   [`crate::octomap::iter::LeavesInBbx`]'s `Iterator` impl.
//! - `it.getCoordinate()` (`:125`) -- [`crate::octomap::iter::Leaf::coordinate`].
//! - `octree->isNodeOccupied(*it)` (`:127`) -- [`crate::octomap::iter::Leaf::is_occupied`].
//!
//! Zero missing. `leaves_in_bbx` returns `Option<LeavesInBbx>` (`None` for
//! an out-of-range corner, see its own tests in `tree.rs`) where upstream's
//! `begin_leafs_bbx` cannot fail that way -- a detail for whoever ports
//! this function to handle (treat `None` as an empty result, matching
//! upstream's own empty-iterator behavior when a bbx has no leaves), not a
//! missing operation. This exclusion ("collision_octomap_filter needs
//! octomap operations not yet available") is now closed as false rather
//! than merely expired: it was checked against every octomap call the file
//! makes, not just the entry point's obvious ones, and none is missing.
//!
mod error;
mod iter;
mod key;
mod node;
mod tree;

pub use error::DecodeError;
pub use iter::{Leaf, Leaves, LeavesInBbx, TreeNode, TreeNodes};
pub use key::{KeyRay, KeySet, KeyType, OcTreeKey};
pub use tree::OcTree;
