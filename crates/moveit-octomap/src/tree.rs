// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (see key.rs's provenance comment for how the
// version was matched):
//   include/octomap/OcTreeBaseImpl.h, OcTreeBaseImpl.hxx
//   include/octomap/OccupancyOcTreeBase.h, OccupancyOcTreeBase.hxx
//   include/octomap/OcTreeDataNode.h, OcTreeDataNode.hxx (`readData` only,
//     fused into this file's own `read_data_node`; the rest of
//     `OcTreeDataNode` is ported in node.rs)
//   include/octomap/AbstractOccupancyOcTree.h
//   include/octomap/octomap_utils.h (logodds / probability)
//   include/octomap/OcTree.h (the concrete, non-template `OcTree`; its
//     sensor-model defaults live in OcTree.cpp, compiled into
//     liboctomap.so.1.9 and not shipped as source -- see the `DEFAULT_*`
//     constants below for how those five numbers were confirmed rather than
//     guessed)

use nalgebra::{Point3, Vector3};

use crate::error::DecodeError;
use crate::key::{KeyRay, KeySet, KeyType, OcTreeKey, compute_child_idx};
use crate::node::Node;

/// Upstream `octomap::logodds`: `log(p / (1 - p))`, computed in `f64` and
/// rounded to `f32` at the end, matching upstream's `(float) log(...)` cast.
fn logodds(probability: f64) -> f32 {
    (probability / (1.0 - probability)).ln() as f32
}

/// Upstream `octomap::probability`: the inverse of [`logodds`].
pub(crate) fn probability(log_odds: f64) -> f64 {
    1.0 - (1.0 / (1.0 + log_odds.exp()))
}

/// A probabilistic occupancy octree. Upstream `octomap::OcTree` (via
/// `OccupancyOcTreeBase<OcTreeNode>` and `OcTreeBaseImpl<OcTreeNode,
/// AbstractOccupancyOcTree>`).
///
/// # What this port does not carry over
///
/// - **No `AbstractOcTree` registry.** Upstream self-registers each concrete
///   tree type (`OcTree`, `ColorOcTree`, ...) into a runtime factory keyed by
///   `className()`, so e.g. a binary file's header can look up the right
///   type to deserialize into. That is a pluginlib-shaped runtime-type
///   registry, and PORTING-PLAN.md's D4 rules out runtime polymorphism in
///   favor of compile-time dispatch: this crate has exactly one concrete
///   tree type, so nothing needs registering or looking up.
/// - **Only the plain occupancy node is ported**, not `ColorOcTreeNode`,
///   `CountingOcTreeNode`, or `OcTreeNodeStamped` -- moveit2 never
///   references any of those (confirmed by a workspace-wide
///   `grep -rl "octomap::"` over `moveit_core`/`moveit_ros`, see this
///   change's commit body for the file:line inventory).
/// - **`changed_keys` / `enableChangeDetection` is not ported.** No moveit2
///   caller enables change detection; it is an opt-in upstream feature nothing
///   in this port's target surface exercises.
/// - **`setNodeValue` (the direct, non-relative set) is not ported.** The one
///   moveit2 call site that conceptually wants "set to the clamp minimum"
///   (`lazy_free_space_updater.cpp`'s `tree_->updateNode(it, lg_0)`, where
///   `lg_0 = clampingThresMinLog - clampingThresMaxLog`) does so through the
///   *relative* `updateNode(key, log_odds_update)` primitive instead, adding
///   a delta so large that the result clamps to `clamping_thres_min`
///   regardless of the node's prior value -- so `update_node_log_odds`
///   already covers it.
/// - **`computeDiscreteUpdate`/BBX-limited updates are not ported.** No
///   moveit2 caller passes `discretize=true` or calls `useBBXLimit`; both
///   sensor updaters build their own key sets by hand instead of calling
///   upstream's `insertPointCloud` family at all (see this crate's docs and
///   the commit body's Step 1 API-surface inventory).
///
/// # Representation: pointer-linked nodes, not a flat keyed map
///
/// The tree is a genuine parent-child node tree (`Node`, `Box`-owned
/// instead of upstream's manually `new`/`delete`d raw pointers), not a flat
/// `HashMap<OcTreeKey, f32>` at the finest resolution. This is not just
/// fidelity to upstream: [`Self::prune`] and the eager per-update pruning in
/// [`Self::update_node`] collapse 8 identical children into their parent,
/// and multi-resolution leaf iteration ([`Self::leaves`]) reports a node's
/// size at whatever depth it was collapsed to. A flat map has no notion of
/// "level" for either of those to collapse into or report from -- it would
/// have to simulate the tree internally anyway, at which point it is not
/// actually flat. The pointer tree is the structurally necessary
/// representation for the semantics this crate's callers need, not an
/// arbitrary upstream-fidelity choice.
///
/// # Symbol-by-symbol audit against upstream's public surface (round 16, item 2)
///
/// Round 12's audit (`lib.rs`'s old "24 ported / 2 unported / 15 distinct /
/// 41 symbol groups" table) classified by "symbol group", a grouping rule
/// stated nowhere and not reproducible by inspection -- round 16's brief
/// rejected it on exactly that ground. This section replaces it with the
/// format `moveit-scene`'s `planning_scene.hpp` audit uses (commit
/// `943e909`): one bullet per literal `public:` declaration pulled from the
/// header text itself, not a paraphrase.
///
/// **Headers read.** `octomap` is not checked out on this host at all (see
/// this crate's provenance note in `lib.rs`); all five headers below were
/// read from inside the `moveit-rs` oracle container --
/// `sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:7b8463d6943edaac -c 'cat /usr/include/octomap/<header>'"`
/// -- in this order (most-derived class first):
///
/// ```text
/// octomap/OcTree.h                    (101 lines)
/// octomap/OccupancyOcTreeBase.h       (508 lines)
/// octomap/OcTreeBaseImpl.h            (575 lines)
/// octomap/AbstractOccupancyOcTree.h   (241 lines)
/// octomap/AbstractOcTree.h            (164 lines)
/// ```
///
/// **Collapsing rule.** This crate's exposed surface is every public member
/// callable on an [`OcTree`] instance -- the union of `OcTree`'s own
/// `public:` section with every ancestor's (`OccupancyOcTreeBase<OcTreeNode>`
/// -> `OcTreeBaseImpl<OcTreeNode, AbstractOccupancyOcTree>` ->
/// `AbstractOccupancyOcTree` -> `AbstractOcTree`, that exact order --
/// confirmed from each header's own `class X : public Y` line, not
/// assumed), where a name a base class declares (pure) virtual and a
/// derived class overrides with the identical signature is counted
/// **once**, at its most-derived declaration -- the walk below visits
/// headers most-derived-first and marks a re-declaration "already counted
/// above" instead of re-listing it. Non-most-derived constructors/
/// destructors (`AbstractOcTree()`, `AbstractOccupancyOcTree()`,
/// `~AbstractOccupancyOcTree()`, etc.) are *not* separately counted: a
/// caller builds an `OcTree` by calling `OcTree::new`, never a base
/// constructor directly, so those are implementation plumbing, not part of
/// the callable surface. Protected/private members and non-callable
/// declarations (type aliases, forward-declared nested classes) are out of
/// scope entirely (same rule `planning_scene.hpp`'s audit used).
///
/// One bullet per declaration: `ported as <symbol>` / `unported, in scope
/// (<reason>)` / `distinct (<reason>)`.
///
/// **Reproducible raw counts (round 18, item 1).** Every raw count named
/// above and in each header's own section below was produced by running
/// `tools/ci/count-public-declarations.sh <header>
/// <ClassName>` against a fresh oracle fetch, not eyeballed -- the script
/// strips `//`/`/* */` comments first (so a doc-comment's `@code` example
/// or prose can't be mistaken for a declaration, the bug this round found
/// in a first draft of it against `AbstractOcTree.h`'s `read()` doc
/// comment) and counts one unit per semicolon-terminated statement or
/// complete inline `{ ... }` body at the named class's own brace depth,
/// nested classes/structs excluded. It reports the **raw textual** count;
/// the "expected" number after applying this section's own exclusions
/// (non-most-derived ctor/dtor, forward-declared nested classes) is a
/// judgment call spelled out in prose, not something the script decides.
/// Re-run against the current stamp:
///
/// ```text
/// $ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 -c 'cat /usr/include/octomap/OcTree.h'" > /tmp/OcTree.h
/// $ tools/ci/count-public-declarations.sh /tmp/OcTree.h OcTree
/// 5
/// $ ... OccupancyOcTreeBase.h OccupancyOcTreeBase
/// 49
/// $ ... OcTreeBaseImpl.h OcTreeBaseImpl
/// 77
/// $ ... AbstractOccupancyOcTree.h AbstractOccupancyOcTree
/// 34
/// $ ... AbstractOcTree.h AbstractOcTree
/// 25
/// ```
///
/// All five match this section's already-stated raw counts exactly --
/// `OcTreeBaseImpl.h`'s 77 in particular reconciles bullet-for-bullet with
/// the 69 bullets in that section once its 3 excluded non-most-derived
/// special members and 5 multi-declaration folds (the `coordToKey`
/// pairs, `typedef leaf_iterator iterator` + `begin`) are accounted for:
/// 77 - 3 - 5 = 69. `AbstractOcTree.h`'s 25 raw minus the one
/// forward-declared `iterator_base` (excluded per this section's own
/// rule) is 24, the figure already cross-checked bullet-for-bullet
/// against that section below. No new gap found this round beyond the
/// `getTreeType()` one round 17 already fixed.
///
/// ## `OcTree.h`
///
/// - `OcTree(double resolution)` -- ported as [`OcTree::new`].
/// - `OcTree(std::string filename)` -- distinct (binary-file-backed
///   constructor; this crate ports no file/stream IO at all, see `IO`
///   below). Not D1: a prior version of this bullet claimed octrees enter
///   this workspace "only via ROS messages", but `moveit-scene`'s
///   `PlanningScene::process_octomap_ptr` (`crates/moveit-scene/src/scene.rs`)
///   takes a plain, message-free [`OcTree`] parameter directly -- the real
///   reason this constructor is unported is narrower and unrelated to D1:
///   no file/stream IO at all, full stop.
/// - `~OcTree()` -- distinct (upstream's own body is `{}`; nothing here
///   needs a user-visible destructor beyond Rust's implicit `Drop` over the
///   owned `Box<Node>` tree).
/// - `create() const` -- distinct (`AbstractOcTree`'s runtime-type
///   registry; D4 rules out runtime polymorphism, this crate has exactly
///   one concrete tree type).
/// - `getTreeType() const` -- distinct, same registry reasoning.
///
/// ## `OccupancyOcTreeBase.h`
///
/// - `OccupancyOcTreeBase(double resolution)` -- ported, collapsed into
///   [`OcTree::new`] (the public 3-arg constructor for tree-constant
///   subclassing is protected, out of scope).
/// - `~OccupancyOcTreeBase()` -- distinct, same as `~OcTree()` above.
/// - `OccupancyOcTreeBase(const OccupancyOcTreeBase&)` (copy constructor)
///   -- distinct: this workspace's octrees are shared via `Arc`, never
///   deep-cloned.
/// - `insertPointCloud(const Pointcloud&, const point3d&, double, bool,
///   bool)` -- distinct: zero consumer, not D1. A prior version of this
///   bullet classified all four overloads below as D1 on the strength of
///   `octomap::Pointcloud`'s name alone; that was wrong on two counts.
///   First, `Pointcloud`/`point3d`/`pose6d`/`ScanNode` are octomap's own
///   message-free types (`third_party/octomap/octomap/include/octomap/Pointcloud.h`
///   depends only on `<vector>`/`<list>`, no ROS/msg include), so calling
///   this overload never requires touching a ROS message. Second, and
///   dispositive on its own: `rg -rn insertPointCloud
///   /home/stevek/work/moveit2` (pinned `e017c91e`) returns zero hits —
///   not "every caller is a ROS depth-camera updater", there is no caller
///   at all. `moveit_ros/perception`'s two octomap updaters
///   (`pointcloud_octomap_updater.cpp:336-418`,
///   `depth_image_octomap_updater.cpp:554-619`) both populate their trees
///   by calling `updateNode` directly, one key at a time, via
///   `computeRayKeys` — never this overload or the three below. Same
///   zero-consumer reasoning as `castRay`/`getRayIntersection` below.
/// - `insertPointCloud(const Pointcloud&, const point3d&, const pose6d&,
///   double, bool, bool)` -- distinct: zero consumer, same reasoning.
/// - `insertPointCloud(const ScanNode&, double, bool, bool)` -- distinct:
///   zero consumer, same reasoning.
/// - `insertPointCloudRays(const Pointcloud&, const point3d&, double,
///   bool)` -- distinct: zero consumer, same reasoning.
/// - `setNodeValue(const OcTreeKey&, float, bool)` -- unported, in scope
///   (PORTING-PLAN.md §13; this crate's "What this port does not carry
///   over" section above names the one upstream call site that
///   conceptually wants it and explains why it routes through
///   [`OcTree::update_node_log_odds_by_key`] with a saturating delta
///   instead).
/// - `setNodeValue(const point3d&, float, bool)` -- unported, in scope,
///   same reason.
/// - `setNodeValue(double, double, double, float, bool)` -- unported, in
///   scope, same reason.
/// - `updateNode(const OcTreeKey&, float, bool)` -- ported as
///   [`OcTree::update_node_log_odds_by_key`].
/// - `updateNode(const point3d&, float, bool)` -- ported as
///   [`OcTree::update_node_log_odds`].
/// - `updateNode(double, double, double, float, bool)` -- unported, in
///   scope (pure ergonomic wrapper around the point3d overload above; no
///   caller in `moveit_core` needs the triple-arg form).
/// - `updateNode(const OcTreeKey&, bool, bool)` -- ported as
///   [`OcTree::update_node_by_key`].
/// - `updateNode(const point3d&, bool, bool)` -- ported as
///   [`OcTree::update_node`].
/// - `updateNode(double, double, double, bool, bool)` -- unported, in
///   scope, same ergonomic-wrapper reasoning.
/// - `toMaxLikelihood()` -- distinct (this port inlines the max-likelihood
///   clamp directly into the private `update_node_recurs` step
///   [`OcTree::update_node_log_odds_by_key`] calls, rather than keeping it
///   a separately callable pass).
/// - `insertRay(const point3d&, const point3d&, double, bool)` -- ported
///   as [`OcTree::insert_ray`].
/// - `castRay(const point3d&, const point3d&, point3d&, bool, double)
///   const` -- distinct: zero `moveit_core` consumer
///   (`rg -l castRay moveit_core`, excluding `moveit_ros`/tests, is
///   empty, checked round 16); this workspace's octree collision path is
///   the leaf-`Cuboid` `Compound` approximation, PORTING-PLAN.md §4.8's
///   decision, not octomap's own raycasting.
/// - `getRayIntersection(...)` -- distinct, zero consumer, same reasoning.
/// - `getNormals(const point3d&, std::vector<point3d>&, bool) const` --
///   distinct, zero consumer (marching-cubes surface reconstruction).
/// - `useBBXLimit(bool)` -- distinct: zero consumer, only meaningful
///   alongside the already-zero-consumer BBX-limited `insertPointCloud`
///   path above.
/// - `bbxSet() const` -- distinct, same reasoning.
/// - `setBBXMin(point3d&)` -- distinct, same reasoning.
/// - `setBBXMax(point3d&)` -- distinct, same reasoning.
/// - `getBBXMin() const` -- distinct, same reasoning.
/// - `getBBXMax() const` -- distinct, same reasoning.
/// - `getBBXBounds() const` -- distinct, same reasoning.
/// - `getBBXCenter() const` -- distinct, same reasoning.
/// - `inBBX(const point3d&) const` -- distinct, same reasoning.
/// - `inBBX(const OcTreeKey&) const` -- distinct, same reasoning.
/// - `enableChangeDetection(bool)` -- distinct: zero consumer, change
///   detection confirmed unused by every moveit2 consumer since this
///   crate's original round.
/// - `isChangeDetectionEnabled() const` -- distinct, same reasoning.
/// - `resetChangeDetection()` -- distinct, same reasoning.
/// - `changedKeysBegin() const` -- distinct, same reasoning.
/// - `changedKeysEnd() const` -- distinct, same reasoning.
/// - `numChangesDetected() const` -- distinct, same reasoning.
/// - `computeUpdate(const Pointcloud&, const point3d&, KeySet&, KeySet&,
///   double)` -- ported as [`OcTree::compute_update`] (point-slice-shaped,
///   not `octomap::Pointcloud`-typed -- see `lib.rs`'s "What was
///   deliberately not ported").
/// - `computeDiscreteUpdate(...)` -- distinct: no moveit2 sensor updater
///   calls it, both hand-roll their own key-set bookkeeping using the
///   lower-level primitives this crate does port instead.
/// - `readBinaryData(std::istream&)` -- ported as
///   [`OcTree::read_binary_data`] (round 33). Commit `91bec85`'s framing
///   here ("octrees enter this workspace only via ROS messages, never
///   `.bt`/`.ot` files") had the ROS-message case exactly backwards --
///   checked against `octomap_msgs/conversions.h` (ROS rolling,
///   `octomap_msgs` 2.0.1): `binaryMapToMsg`/`readTree` call
///   `writeBinaryData`/`readBinaryData` directly on a header-less
///   stringstream to fill `moveit_msgs::Octomap.data` whenever `msg.binary
///   == true`. This *is* the algorithm an `Octomap.data` decoder needs, not
///   something only `.bt` files use. See `lib.rs`, "Round 27, item 1(a)"
///   for the byte format derivation.
/// - `readBinaryNode(std::istream&, NODE*)` -- ported as the private
///   recursive helper `read_binary_node`, called from
///   [`OcTree::read_binary_data`].
/// - `writeBinaryNode(std::ostream&, const NODE*) const` -- ported as the
///   private recursive helper `write_binary_node` (this round), the exact
///   inverse of `read_binary_node`, called from
///   [`OcTree::write_binary_data`]. Round 33's brief scoped decode only;
///   this round's brief made encode in-scope too.
/// - `writeBinaryData(std::ostream&) const` -- ported as
///   [`OcTree::write_binary_data`] (this round). Pinned byte-for-byte
///   against the oracle's own `serialize` output, not just round-tripped
///   through this crate's own decoder -- see `tests/encode_parity.rs`.
/// - `updateInnerOccupancy()` -- ported as
///   [`OcTree::update_inner_occupancy`].
/// - `integrateHit(NODE*) const` -- distinct, inlined into
///   [`OcTree::update_node_by_key`]'s sensor-model dispatch
///   (`prob_hit_log`) rather than kept as a separately callable step.
/// - `integrateMiss(NODE*) const` -- distinct, same reasoning, inlined
///   into the ray-miss path (`integrate_miss_on_ray`).
/// - `updateNodeLogOdds(NODE*, const float&) const` -- ported, inlined
///   into the private `update_node_recurs`'s add-then-clamp step (the
///   split upstream makes between `addValue` and `updateNodeLogOdds`).
/// - `nodeToMaxLikelihood(NODE*) const` -- distinct, same reasoning as
///   `toMaxLikelihood` above.
/// - `nodeToMaxLikelihood(NODE&) const` -- distinct, same reasoning.
///
/// ## `OcTreeBaseImpl.h`
///
/// - `typedef NODE NodeType` -- not a callable member, skipped.
/// - `getTreeType() const` (concrete, returns `"OcTreeBaseImpl"`) --
///   already counted above, `OcTree.h` (`OcTree`'s own non-virtual
///   `getTreeType()`, returning `"OcTree"`, is the declaration a caller
///   actually reaches). **Correction, round 17 item 2:** this
///   declaration (header line 104) was absent from every walk through
///   round 16 -- neither a fresh bullet nor an "already counted above"
///   cross-reference named it anywhere in this section. Found only by
///   independently re-deriving this header's raw declaration count
///   (77) and reconciling it against the bullets actually present (73
///   accounted for, a 1-declaration gap) rather than trusting the
///   existing walk. Same class of drift as `isNodeAtThreshold` in
///   round 16, this time inside round 16's own table rather than round
///   12's.
/// - `setResolution(double)` -- distinct: zero `moveit_core` consumer,
///   resolution is fixed at [`OcTree::new`] and never changed in place.
/// - `getResolution() const` -- ported as [`OcTree::resolution`].
/// - `getTreeDepth() const` -- ported as the [`OcTree::TREE_DEPTH`]
///   associated constant, not a runtime getter -- upstream's own value is
///   fixed to `16` for every instance too (set once by the
///   `tree_depth`-parameterized constructor `OcTree` never uses), so this
///   is a shape deviation, not a gap.
/// - `getNodeSize(unsigned depth) const` -- ported as
///   [`OcTree::node_size`].
/// - `clearKeyRays()` -- distinct: upstream's own doc comment calls this
///   "only useful for the StaticMemberInitializer classes" (the registry
///   machinery D4 already excludes); [`OcTree::compute_ray_keys`] builds
///   and returns a fresh `KeyRay` per call instead of reusing a shared
///   scratch buffer, so there is no buffer here to clear.
/// - `createNodeChild(NODE*, unsigned)` -- ported, fused with
///   `allocNodeChildren` into `Node::create_child` (`pub(crate)`; no
///   consumer outside this crate needs direct child manipulation).
/// - `deleteNodeChild(NODE*, unsigned)` -- distinct: no per-child deletion
///   primitive exists in `node.rs` at all. [`OcTree::prune`] only ever
///   collapses a *whole* 8-child array at once (`Node::prune` sets
///   `children = None`); `OcTreeBaseImpl::deleteNode`'s structural
///   per-key deletion path -- the actual caller of `deleteNodeChild`
///   upstream -- is itself unported below, so nothing in this crate's
///   scope deletes a single child in isolation.
/// - `getNodeChild(NODE*, unsigned) const` -- ported, fused with
///   `nodeChildExists` into `Node::child`/`Node::child_mut` (`Option`
///   covers both "no children array" and "this slot empty").
/// - `getNodeChild(const NODE*, unsigned) const` -- ported, same fusion
///   (the const overload).
/// - `isNodeCollapsible(const NODE*) const` -- ported as
///   `Node::is_collapsible` (`pub(crate)`).
/// - `nodeChildExists(const NODE*, unsigned) const` -- ported, fused into
///   `Node::child` (see `getNodeChild` above).
/// - `nodeHasChildren(const NODE*) const` -- ported as
///   `Node::has_children`.
/// - `expandNode(NODE*)` -- ported as `Node::expand` (`pub(crate)`).
/// - `pruneNode(NODE*)` -- ported as `Node::prune` (`pub(crate)`).
/// - `getRoot() const` -- ported, `pub(crate)` as `OcTree::root`.
/// - `search(double x, double y, double z, unsigned depth) const` --
///   unported, in scope: pure ergonomic triple-arg wrapper around the
///   `point3d` overload below, no caller needs it.
/// - `search(const point3d&, unsigned depth) const` -- ported, fused into
///   [`OcTree::log_odds_at`]/[`OcTree::occupancy_at`]/
///   [`OcTree::is_occupied`], which return typed values rather than a raw
///   nullable `NODE*` -- a deliberate shape deviation, not a gap.
/// - `search(const OcTreeKey&, unsigned depth) const` -- ported,
///   `pub(crate)` as `OcTree::search`.
/// - `deleteNode(double, double, double, unsigned)` -- distinct: zero
///   consumer, this port's sensor-model update path only ever adds/clamps
///   log-odds, never structurally deletes an existing node.
/// - `deleteNode(const point3d&, unsigned)` -- distinct, same reasoning.
/// - `deleteNode(const OcTreeKey&, unsigned)` -- distinct, same reasoning.
/// - `clear()` -- distinct: zero consumer, nothing resets a tree to empty
///   in place in this crate's scope.
/// - `prune()` -- ported as [`OcTree::prune`].
/// - `expand()` (tree-wide) -- distinct: zero consumer, and upstream's own
///   doc warns it is "an expensive operation, especially when the tree is
///   nearly empty" -- the per-node primitive this algorithm would walk the
///   tree calling, `expandNode`, is ported above; the tree-wide sweep
///   itself is not.
/// - `size() const` -- distinct, **NO-GO, decided round 13 item 2** (see
///   `lib.rs`'s dedicated section for the full reasoning -- its one
///   `moveit_core`-reachable caller is `collision_detection_bullet`,
///   dropped by PORTING-PLAN.md outright).
/// - `memoryUsage() const` -- distinct: zero consumer.
/// - `memoryUsageNode() const` -- distinct, same.
/// - `memoryFullGrid() const` -- distinct, same.
/// - `volume()` -- distinct, same.
/// - `getMetricSize(double&, double&, double&)` -- distinct: zero consumer
///   anywhere in `moveit_core`, including the perception layer.
/// - `getMetricSize(double&, double&, double&) const` -- distinct, same.
/// - `getMetricMin(double&, double&, double&)` -- distinct, same.
/// - `getMetricMin(double&, double&, double&) const` -- distinct, same.
/// - `getMetricMax(double&, double&, double&)` -- distinct, same.
/// - `getMetricMax(double&, double&, double&) const` -- distinct, same.
/// - `calcNumNodes() const` -- ported as [`OcTree::num_nodes`] (checked
///   directly against `OcTreeBaseImpl.h:269` versus `size()`'s
///   `OcTreeBaseImpl.h:241` -- [`OcTree::num_nodes`] is this one, an O(n)
///   recursive traversal, not the O(1) `tree_size` counter).
/// - `getNumLeafNodes() const` -- ported as [`OcTree::num_leaf_nodes`].
/// - `getUnknownLeafCenters(point3d_list&, point3d, point3d, unsigned)
///   const` -- distinct: zero consumer, introspection/debugging helper.
/// - `computeRayKeys(const point3d&, const point3d&, KeyRay&) const` --
///   ported as [`OcTree::compute_ray_keys`].
/// - `computeRay(const point3d&, const point3d&, std::vector<point3d>&)`
///   -- distinct: zero consumer, and upstream's own doc says "use the
///   faster computeRayKeys method if possible".
/// - `readData(std::istream&)` -- ported as [`OcTree::read_data`] (round
///   33), fused with `readNodesRecurs`/`OcTreeDataNode::readData` into the
///   private `read_data_node` helper -- `octomap_msgs/conversions.h`'s
///   `fullMapToMsg`/`fullMsgToMap` call `writeData`/`readData` directly
///   (via `OcTreeBaseImpl`) to fill `moveit_msgs::Octomap.data` when
///   `msg.binary == false`, the full (non-quantized) counterpart of the
///   binary path above.
/// - `writeData(std::ostream&) const` -- ported as [`OcTree::write_data`]
///   (this round), fused with `writeNodesRecurs`/`OcTreeDataNode::writeData`
///   into the private `write_data_node` helper, same reasoning and same
///   oracle-byte pinning as `writeBinaryData` above.
/// - `typedef leaf_iterator iterator` / `begin(unsigned char) const` --
///   ported as [`OcTree::leaves`] (upstream's default iterator *is*
///   `leaf_iterator`, the same primitive `begin_leafs` below returns).
/// - `end() const` -- ported, folded into Rust's `Iterator` protocol:
///   [`crate::iter::Leaves`] implements `Iterator` and stops on `None`
///   rather than comparing against a separate sentinel end-value.
/// - `begin_leafs(unsigned char) const` -- ported as [`OcTree::leaves`].
/// - `end_leafs() const` -- ported, folded into `Iterator` (see `end()`
///   above).
/// - `begin_leafs_bbx(const OcTreeKey&, const OcTreeKey&, unsigned char)
///   const` -- unported, in scope: [`OcTree::leaves_in_bbx`] only takes
///   the `point3d`-bounded overload below; no caller needs the
///   raw-key-bounded entry point.
/// - `begin_leafs_bbx(const point3d&, const point3d&, unsigned char)
///   const` -- ported as [`OcTree::leaves_in_bbx`].
/// - `end_leafs_bbx() const` -- ported, folded into `Iterator`.
/// - `begin_tree(unsigned char) const` -- ported as
///   [`OcTree::tree_nodes`].
/// - `end_tree() const` -- ported, folded into `Iterator`.
/// - `coordToKey(double) const`, `coordToKey(const point3d&) const`,
///   `coordToKey(double, double, double) const` (3 declarations, finest
///   depth) -- ported, fused: the unchecked single-axis cast these
///   overloads perform is the first step inside the private
///   `coord_to_key_checked_axis`, not kept as a separately callable
///   primitive since every conversion entry point this crate exposes
///   ([`OcTree::coord_to_key_checked`]) needs the bounds check anyway.
/// - `coordToKey(double, unsigned depth) const`,
///   `coordToKey(const point3d&, unsigned depth) const`,
///   `coordToKey(double, double, double, unsigned depth) const` (3
///   declarations) -- distinct: no depth-parameterized *encode* path
///   exists in this crate at all (only `OcTree::key_to_coord_at_depth`,
///   the *decode* direction, takes a `depth`); no caller in scope inserts
///   or queries at a depth other than the finest -- multi-resolution
///   *read* (leaf iteration reporting a coarser depth after pruning) is
///   the direction this crate needs, not multi-resolution *write*.
/// - `adjustKeyAtDepth(const OcTreeKey&, unsigned) const` -- distinct, same
///   "no caller needs multi-resolution write" reasoning.
/// - `adjustKeyAtDepth(key_type, unsigned) const` -- distinct, same.
/// - `coordToKeyChecked(const point3d&, OcTreeKey&) const` -- ported as
///   [`OcTree::coord_to_key_checked`].
/// - `coordToKeyChecked(const point3d&, unsigned depth, OcTreeKey&)
///   const` -- distinct, same depth-write reasoning as `coordToKey`
///   above.
/// - `coordToKeyChecked(double, double, double, OcTreeKey&) const` --
///   unported, in scope: pure ergonomic triple-arg wrapper around the
///   `point3d` overload, no caller needs it.
/// - `coordToKeyChecked(double, double, double, unsigned depth,
///   OcTreeKey&) const` -- distinct, same depth-write reasoning.
/// - `coordToKeyChecked(double, key_type&) const` -- ported, `pub(crate)`
///   as the private `coord_to_key_checked_axis` (the single-axis step
///   [`OcTree::coord_to_key_checked`] calls three times).
/// - `coordToKeyChecked(double, unsigned depth, key_type&) const` --
///   distinct, same depth-write reasoning.
/// - `keyToCoord(key_type, unsigned depth) const` -- ported, `pub(crate)`
///   as `key_to_coord_axis_at_depth`.
/// - `keyToCoord(key_type) const` (finest depth) -- ported, `pub(crate)`
///   as `key_to_coord_axis` (used directly inside
///   [`OcTree::compute_ray_keys`]'s voxel-border math).
/// - `keyToCoord(const OcTreeKey&) const` (finest, all 3 axes) -- ported,
///   fused: `OcTree::key_to_coord_at_depth` handles the finest depth as
///   one branch (`depth == Self::TREE_DEPTH`) of the depth-parameterized
///   version below rather than keeping a separate wrapper.
/// - `keyToCoord(const OcTreeKey&, unsigned depth) const` -- ported,
///   `pub(crate)` as `OcTree::key_to_coord_at_depth`.
/// - `swapContent(OcTreeBaseImpl&)` -- distinct: this workspace's octrees
///   are shared via `Arc`, never swapped.
/// - `operator==(const OcTreeBaseImpl&) const` -- distinct, same
///   reasoning (the one `octree ==` hit in `moveit_core` outside
///   tests/`moveit_ros`, `planning_scene.cpp:1510`, is a `shared_ptr`
///   identity comparison, not this operator -- confirmed by reading the
///   call site itself, round 16, not assumed from the grep hit alone).
///
/// ## `AbstractOccupancyOcTree.h`
///
/// - `writeBinary(const std::string&)` -- distinct, binary file IO.
/// - `writeBinary(std::ostream&)` -- distinct, same.
/// - `writeBinaryConst(const std::string&) const` -- distinct, same.
/// - `writeBinaryConst(std::ostream&) const` -- distinct, same.
/// - `readBinary(std::istream&)` -- distinct, same.
/// - `readBinary(const std::string&)` -- distinct, same.
/// - `isNodeOccupied(const OcTreeNode*) const` -- ported, fused into
///   [`OcTree::is_node_occupied_log_odds`]: `OcTree::search` already
///   returns `Option<&Node>`, so the natural shape compares the found
///   node's own `log_odds` field directly (see [`OcTree::is_occupied`])
///   rather than re-dereferencing a pointer.
/// - `isNodeOccupied(const OcTreeNode&) const` -- ported, same fusion.
/// - `isNodeAtThreshold(const OcTreeNode*) const` -- distinct: zero
///   `moveit_core` consumer (`rg -l isNodeAtThreshold moveit_core`,
///   excluding tests/`moveit_ros`, is empty, checked round 16).
///   **Correction, round 16 item 2:** round 12's audit never classified
///   this symbol at all -- neither `ported` nor `unported, in scope` nor
///   `distinct` named it anywhere in the old table. It was a genuine gap
///   in that walk, the same class of drift round 15 found for
///   `tree_iterator`, caught only by this round's fresh literal read of
///   the header rather than trusting the prior table.
/// - `isNodeAtThreshold(const OcTreeNode&) const` -- distinct, same
///   correction and reasoning.
/// - `updateNode(const OcTreeKey&, float, bool)` (pure virtual),
///   `updateNode(const point3d&, float, bool)` (pure virtual),
///   `updateNode(const OcTreeKey&, bool, bool)` (pure virtual),
///   `updateNode(const point3d&, bool, bool)` (pure virtual) -- already
///   counted above, `OccupancyOcTreeBase.h`.
/// - `toMaxLikelihood()` (pure virtual) -- already counted above,
///   `OccupancyOcTreeBase.h`.
/// - `readBinaryData(std::istream&)` (pure virtual), `writeBinaryData(
///   std::ostream&) const` (pure virtual) -- already counted above,
///   `OccupancyOcTreeBase.h`.
/// - `setOccupancyThres(double)` -- ported as
///   [`OcTree::set_occupancy_thres`] (round 15, item 1).
/// - `setProbHit(double)` -- ported as [`OcTree::set_prob_hit`] (round 15
///   item 1; the `assert(prob_hit_log >= 0.0)` at `:190` ported as
///   `debug_assert!`, round 16 item 1).
/// - `setProbMiss(double)` -- ported as [`OcTree::set_prob_miss`] (same
///   rounds, `assert(prob_miss_log <= 0.0)` at `:192`).
/// - `setClampingThresMin(double)` -- ported as
///   [`OcTree::set_clamping_thres_min`] (round 15, item 1).
/// - `setClampingThresMax(double)` -- ported as
///   [`OcTree::set_clamping_thres_max`] (round 15, item 1).
/// - `getOccupancyThres() const` -- distinct: its one `moveit_core` caller
///   (`bullet_utils.cpp:210`) is the Bullet collision backend, dropped by
///   PORTING-PLAN.md (`parry3d-f64` replaces both FCL and Bullet).
/// - `getOccupancyThresLog() const` -- ported as
///   [`OcTree::occupancy_thres_log`].
/// - `getProbHit() const` -- distinct, zero consumer.
/// - `getProbHitLog() const` -- ported as [`OcTree::prob_hit_log`].
/// - `getProbMiss() const` -- distinct, zero consumer.
/// - `getProbMissLog() const` -- ported as [`OcTree::prob_miss_log`].
/// - `getClampingThresMin() const` -- distinct, zero consumer.
/// - `getClampingThresMinLog() const` -- ported as
///   [`OcTree::clamping_thres_min_log`].
/// - `getClampingThresMax() const` -- distinct, zero consumer.
/// - `getClampingThresMaxLog() const` -- ported as
///   [`OcTree::clamping_thres_max_log`].
///
/// ## `AbstractOcTree.h`
///
/// - `getResolution() const`, `setResolution(double)`, `size() const`,
///   `memoryUsage() const`, `memoryUsageNode() const`,
///   `getMetricMin(double&,double&,double&)` (both overloads),
///   `getMetricMax(double&,double&,double&)` (both overloads),
///   `getMetricSize(double&,double&,double&)`, `prune()`, `expand()`,
///   `clear()`, `readData(std::istream&)`, `writeData(std::ostream&)
///   const` (all pure virtual) -- already counted above, `OcTreeBaseImpl.h`.
/// - `create() const`, `getTreeType() const` (pure virtual) -- already
///   counted above, `OcTree.h`.
/// - `class iterator_base` (forward declaration only; the real
///   `leaf_iterator`/`tree_iterator`/`leaf_bbx_iterator` declarations in
///   this header are commented out, `AbstractOcTree.h:82-104`) -- not a
///   callable member, skipped.
/// - `write(const std::string&) const` -- distinct, file IO.
/// - `write(std::ostream&) const` -- distinct, same.
/// - `createTree(const std::string, double)` (static) -- distinct: the
///   registry/factory pattern D4 excludes, zero consumer.
/// - `read(const std::string&)` (static) -- distinct, file IO.
/// - `read(std::istream&)` (static) -- distinct, same.
///
/// **Total, by `rg -c '^/// - \`' crates/moveit-octomap/src/tree.rs`
/// (every such bullet in the file is inside this audit, so the plain
/// per-file count is exact, no line range needed):** **159** bullets --
/// the same unit `moveit-scene`'s `planning_scene.hpp` audit counts by (a
/// bullet sometimes names more than one C++ declaration when they share
/// one classification and reason, e.g. the three finest-depth `coordToKey`
/// overloads under one `ported, fused` bullet -- exactly how that audit's
/// own `checkCollision`/`getCollidingLinks` bundle several overloads per
/// bullet too). Of the 159: 2 are not callable symbols at all (`typedef
/// NODE NodeType`, the forward-declared `iterator_base`) and 6 are
/// cross-references to declarations already tallied under a more-derived
/// header (`OcTreeBaseImpl.h`'s own concrete `getTreeType()`, already
/// counted at `OcTree.h`'s more-derived override -- round 17 item 2's
/// correction, see that bullet; the `updateNode`/`toMaxLikelihood`/
/// `readBinaryData`/`writeBinaryData` pure-virtual re-declarations in
/// `AbstractOccupancyOcTree.h`; and the `getResolution`/`setResolution`/
/// `size`/`memoryUsage`/`memoryUsageNode`/`getMetricMin`/`getMetricMax`/
/// `getMetricSize`/`prune`/`expand`/`clear`/`readData`/`writeData`/
/// `create`/`getTreeType` pure-virtual re-declarations in
/// `AbstractOcTree.h`) -- neither group is counted again. The remaining
/// **151** audited bullets:
///
/// **Round 33 update:** `readBinaryData`/`readBinaryNode`/`readData` moved
/// from `distinct` to `ported` (3 bullets); their `write*` counterparts
/// stayed `distinct` at the time, the encode direction being out of scope
/// for that round.
///
/// **This round's update:** `writeBinaryData`/`writeBinaryNode`/`writeData`
/// (the `write*` counterparts round 33 left `distinct`) moved to `ported`
/// too (3 more bullets) -- encode is in scope this round, see
/// [`OcTree::write_binary_data`]/[`OcTree::write_data`].
///
/// ```text
/// ported                61
/// unported, in scope     8
/// distinct               82
/// ------------------------
/// total                 151
/// ```
///
/// **`unported, in scope`** (8), all named above: the three `setNodeValue`
/// overloads; the two triple-`double`-argument `updateNode` overloads; the
/// triple-`double`-argument `search`/`coordToKeyChecked` overloads (pure
/// ergonomic wrappers, one bullet each); and `begin_leafs_bbx`'s
/// raw-`OcTreeKey`-bounded overload.
#[derive(Debug)]
pub struct OcTree {
    root: Option<Box<Node>>,
    resolution: f64,
    resolution_factor: f64,
    clamping_thres_min: f32,
    clamping_thres_max: f32,
    prob_hit_log: f32,
    prob_miss_log: f32,
    occ_prob_thres_log: f32,
}

impl OcTree {
    /// Upstream `tree_depth`: fixed maximum tree depth.
    pub const TREE_DEPTH: u32 = 16;
    /// Upstream `tree_max_val`: the key-space coordinate of the tree center
    /// (`2^15`), fixed alongside `TREE_DEPTH` since both come from the same
    /// non-template `OcTreeBaseImpl(double)` constructor upstream's `OcTree`
    /// actually uses (the `tree_depth`/`tree_max_val`-parameterized
    /// constructor exists only for hypothetical subclasses; nothing in
    /// moveit2 or this crate's scope needs one).
    pub const TREE_MAX_VAL: i32 = 32768;

    /// Sensor-model defaults for a freshly constructed tree. Upstream's
    /// `OcTree` constructor sets these via `setProbHit`/`setProbMiss`/
    /// `setClampingThresMin`/`setClampingThresMax` with literal probabilities
    /// baked into `OcTree.cpp` -- compiled into `liboctomap.so.1.9`, not
    /// shipped as header source, so these five numbers cannot be read from
    /// any file on this machine. They were instead measured directly off the
    /// real `liboctomap.so.1.9.7` inside the oracle container: a throwaway
    /// probe (`octomap::OcTree(0.1)`, then `getProbHit()`/`getProbHitLog()`/
    /// etc.) printed the exact log-odds this port now hardcodes, and those
    /// measured log-odds round-trip through this file's own `logodds()`
    /// applied to the human-readable probabilities below to within `f32`
    /// rounding -- see this change's commit body for the full probe output.
    const DEFAULT_PROB_HIT: f64 = 0.7;
    const DEFAULT_PROB_MISS: f64 = 0.4;
    const DEFAULT_CLAMPING_THRES_MIN: f64 = 0.1192;
    const DEFAULT_CLAMPING_THRES_MAX: f64 = 0.971;
    const DEFAULT_OCCUPANCY_THRES: f64 = 0.5;

    /// Upstream `OcTree(double resolution)`.
    pub fn new(resolution: f64) -> Self {
        Self {
            root: None,
            resolution,
            resolution_factor: 1.0 / resolution,
            clamping_thres_min: logodds(Self::DEFAULT_CLAMPING_THRES_MIN),
            clamping_thres_max: logodds(Self::DEFAULT_CLAMPING_THRES_MAX),
            prob_hit_log: logodds(Self::DEFAULT_PROB_HIT),
            prob_miss_log: logodds(Self::DEFAULT_PROB_MISS),
            occ_prob_thres_log: logodds(Self::DEFAULT_OCCUPANCY_THRES),
        }
    }

    /// Upstream `getResolution`.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Upstream `getProbHitLog`.
    pub fn prob_hit_log(&self) -> f32 {
        self.prob_hit_log
    }
    /// Upstream `getProbMissLog`.
    pub fn prob_miss_log(&self) -> f32 {
        self.prob_miss_log
    }
    /// Upstream `getClampingThresMinLog`.
    pub fn clamping_thres_min_log(&self) -> f32 {
        self.clamping_thres_min
    }
    /// Upstream `getClampingThresMaxLog`.
    pub fn clamping_thres_max_log(&self) -> f32 {
        self.clamping_thres_max
    }
    /// Upstream `getOccupancyThresLog`.
    pub fn occupancy_thres_log(&self) -> f32 {
        self.occ_prob_thres_log
    }

    /// Upstream `setOccupancyThres`: reconfigures the boundary
    /// [`Self::is_node_occupied_log_odds`] compares node log-odds against.
    /// `prob` is a probability in `(0, 1)`, converted and stored in
    /// log-odds space like every other sensor-model parameter here. Round
    /// 15, item 1: ported and pinned against the real `liboctomap.so.1.9.7`
    /// (see `crates/moveit-octomap/tests/octomap_parity.rs`'s
    /// `set_occupancy_thres` scenario) -- previously this field could only
    /// ever hold `Self::DEFAULT_OCCUPANCY_THRES`, a hardcoded value wearing
    /// a per-instance-config shape with nothing able to reconfigure it.
    pub fn set_occupancy_thres(&mut self, prob: f64) {
        self.occ_prob_thres_log = logodds(prob);
    }

    /// Upstream `setProbHit`. `prob` is the hit sensor-model probability;
    /// upstream asserts the resulting log-odds is non-negative (`prob >=
    /// 0.5`) with plain C `assert()` (`AbstractOccupancyOcTree.h:190`,
    /// `void setProbHit(double prob){prob_hit_log = logodds(prob);
    /// assert(prob_hit_log >= 0.0);}`, read directly from the oracle
    /// container this round, round 16 item 1) -- a debug-build-only sanity
    /// check under `NDEBUG`, matched here with `debug_assert!` for the same
    /// reason rather than a `Result` this crate has no other
    /// fallible-construction convention for. Round 15, item 1; the
    /// `debug_assert!` firing on an out-of-range `prob` is itself tested by
    /// `tests::set_prob_hit_below_half_panics_in_debug` (round 16 item 1 --
    /// round 15 ported the assertion but nothing exercised it).
    pub fn set_prob_hit(&mut self, prob: f64) {
        self.prob_hit_log = logodds(prob);
        debug_assert!(self.prob_hit_log >= 0.0);
    }

    /// Upstream `setProbMiss`. `prob` is the miss sensor-model probability;
    /// upstream asserts the resulting log-odds is non-positive (`prob <=
    /// 0.5`) with plain C `assert()` (`AbstractOccupancyOcTree.h:192`, same
    /// container read as [`Self::set_prob_hit`]), matched here with
    /// `debug_assert!` for the same reason. Round 15, item 1; see
    /// `tests::set_prob_miss_above_half_panics_in_debug` (round 16 item 1).
    pub fn set_prob_miss(&mut self, prob: f64) {
        self.prob_miss_log = logodds(prob);
        debug_assert!(self.prob_miss_log <= 0.0);
    }

    /// Upstream `setClampingThresMin`: the lowest log-odds a node's
    /// occupancy is ever clamped to. Round 15, item 1.
    pub fn set_clamping_thres_min(&mut self, prob: f64) {
        self.clamping_thres_min = logodds(prob);
    }

    /// Upstream `setClampingThresMax`: the highest log-odds a node's
    /// occupancy is ever clamped to. Round 15, item 1.
    pub fn set_clamping_thres_max(&mut self, prob: f64) {
        self.clamping_thres_max = logodds(prob);
    }

    /// Upstream `getNodeSize`: the metric size of a voxel at `depth` (0:
    /// root, [`Self::TREE_DEPTH`]: finest resolution).
    ///
    /// # Deviation: `debug_assert!`, not `assert!` (Task G)
    ///
    /// Upstream's own precondition check is `assert(depth <= tree_depth);`
    /// (`OcTreeBaseImpl.h:113`, inline in the header), which compiles out
    /// under `NDEBUG` -- a release build with an out-of-range `depth` falls
    /// through to an out-of-bounds `sizeLookupTable[depth]` read. This
    /// function is `pub`, so `depth` is caller-controlled; a literal
    /// `assert!` here turned that release-mode no-op into a release-mode
    /// abort for every external caller -- the opposite direction from
    /// upstream's own release behaviour. `debug_assert!` matches upstream's
    /// NDEBUG semantics instead: checked in debug, compiled out in release,
    /// same as upstream's `assert()`.
    pub fn node_size(&self, depth: u32) -> f64 {
        debug_assert!(
            depth <= Self::TREE_DEPTH,
            "node_size: depth {depth} exceeds TREE_DEPTH ({})",
            Self::TREE_DEPTH
        );
        self.resolution * f64::from(1u32 << (Self::TREE_DEPTH - depth))
    }

    /// Upstream `isNodeOccupied`: compares a node's log-odds against the
    /// tree's occupancy threshold. Note the `>=`: a never-updated node
    /// freshly touched by an update (log-odds `0.0`, probability exactly
    /// `0.5`) reads as occupied under the default threshold, since
    /// `occ_prob_thres_log` is also exactly `0.0` -- confirmed against the
    /// real binary, not assumed (see `Self::DEFAULT_OCCUPANCY_THRES`'s
    /// doc comment).
    ///
    /// **Threshold source (round 14, item 2; superseded round 15, item 1):**
    /// `occ_prob_thres_log` is per-instance configuration state, matching
    /// upstream `AbstractOccupancyOcTree`'s own `occ_prob_thres_log_`
    /// member variable -- not a bare constant baked into this comparison.
    /// Round 14 found it was *always* `Self::DEFAULT_OCCUPANCY_THRES` in
    /// practice, because [`Self::set_occupancy_thres`] did not exist yet;
    /// round 15 ported that setter (along with its four sensor-model
    /// siblings) and pinned its effect on this very comparison against the
    /// real `liboctomap.so.1.9.7` -- see
    /// `crates/moveit-octomap/tests/octomap_parity.rs`'s
    /// `set_occupancy_thres` scenario, which checks a hit's `occupied`
    /// determination flips at a raised threshold, matching the oracle's own
    /// `isNodeOccupied` on the same tree state.
    pub fn is_node_occupied_log_odds(&self, log_odds: f32) -> bool {
        log_odds >= self.occ_prob_thres_log
    }

    /// Upstream `keyToCoord(key_type)` at the finest depth.
    fn key_to_coord_axis(&self, key: KeyType) -> f64 {
        (f64::from(key) - f64::from(Self::TREE_MAX_VAL) + 0.5) * self.resolution
    }

    /// Upstream `keyToCoord(key_type, depth)`.
    ///
    /// # Deviation: explicit `debug_assert!` added for `depth <= TREE_DEPTH` (Task G)
    ///
    /// Upstream's own precondition (`assert(depth <= tree_depth);`,
    /// `OcTreeBaseImpl.hxx:395`) was previously reproduced only by
    /// accident: an out-of-range `depth` underflows `TREE_DEPTH - depth`
    /// (this function's `else` branch below) the same way [`Self::search`]'s
    /// `diff` did before that call's own `debug_assert!` was added -- same
    /// defect family, same fix; see `search`'s doc comment for the full
    /// upstream-asymmetry argument this shares. This function is private,
    /// reached only through [`Self::key_to_coord_at_depth`] (`pub(crate)`),
    /// currently always called with an iterator-bounded `depth` -- safe
    /// today, but only by caller discipline, not by construction.
    fn key_to_coord_axis_at_depth(&self, key: KeyType, depth: u32) -> f64 {
        debug_assert!(
            depth <= Self::TREE_DEPTH,
            "key_to_coord_axis_at_depth: depth {depth} exceeds TREE_DEPTH ({})",
            Self::TREE_DEPTH
        );
        if depth == 0 {
            0.0
        } else if depth == Self::TREE_DEPTH {
            self.key_to_coord_axis(key)
        } else {
            let shift = Self::TREE_DEPTH - depth;
            (((f64::from(key) - f64::from(Self::TREE_MAX_VAL)) / f64::from(1u32 << shift)).floor()
                + 0.5)
                * self.node_size(depth)
        }
    }

    /// Upstream `keyToCoord(const OcTreeKey&, depth)`.
    pub(crate) fn key_to_coord_at_depth(&self, key: OcTreeKey, depth: u32) -> Point3<f64> {
        Point3::new(
            self.key_to_coord_axis_at_depth(key[0], depth),
            self.key_to_coord_axis_at_depth(key[1], depth),
            self.key_to_coord_axis_at_depth(key[2], depth),
        )
    }

    /// Upstream `coordToKeyChecked(double, key_type&)` for one axis.
    ///
    /// **Deviation (§172, §153.1):** upstream narrows the scaled coordinate
    /// to `int` unconditionally (`(int) floor(...)`) before its own bounds
    /// check; for a NaN, infinite, or merely huge-but-finite `coord`, that
    /// `double -> int` narrowing is undefined behaviour in C++, so there is
    /// no upstream "correct" answer to reproduce, only a boundary to reject
    /// cleanly. A direct transcription (`.floor() as i64 + ...`) is *not*
    /// equivalent here: Rust's `as` saturates out-of-range floats and turns
    /// NaN into `0` (never UB), so a naive port would silently accept
    /// `coord = NaN` as the tree's exact center key (`0 as i64` lands
    /// in-bounds after the `+ TREE_MAX_VAL` offset) and would arithmetic-
    /// overflow-panic on `coord = +inf` or any `coord` whose scaled
    /// magnitude exceeds `i64::MAX` (`i64::MAX + TREE_MAX_VAL` overflows).
    /// Bounding `scaled_f` in `f64` space -- both finiteness and the tree's
    /// actual representable range -- before ever narrowing to `i64` closes
    /// both failure modes at the one place both originate, rather than
    /// guarding each call site. **Expiry condition:** reopens if upstream
    /// ever adds its own finite/range check to `coordToKeyChecked`, at
    /// which point this becomes a plain transcription instead of a
    /// deviation.
    fn coord_to_key_checked_axis(&self, coord: f64) -> Option<KeyType> {
        let scaled_f = (self.resolution_factor * coord).floor();
        let min = -f64::from(Self::TREE_MAX_VAL);
        let max = f64::from(Self::TREE_MAX_VAL);
        // `scaled_f.is_finite()` is not checked separately: IEEE 754
        // comparisons with NaN are always false and +-infinity always fall
        // outside a finite [min, max) range, so `scaled_f >= min && scaled_f
        // < max` already rejects NaN and both infinities on its own -- an
        // explicit finite check would be a third guard clause with no input
        // that could ever make it, not either of the other two, the reason
        // this returns `None`.
        if !(scaled_f >= min && scaled_f < max) {
            return None;
        }
        let scaled = scaled_f as i64 + i64::from(Self::TREE_MAX_VAL);
        Some(scaled as KeyType)
    }

    /// Upstream `coordToKeyChecked(const point3d&, OcTreeKey&)`. `None` if
    /// any axis is out of the tree's representable range (upstream:
    /// `false`).
    pub fn coord_to_key_checked(&self, p: Point3<f64>) -> Option<OcTreeKey> {
        Some(OcTreeKey::new(
            self.coord_to_key_checked_axis(p.x)?,
            self.coord_to_key_checked_axis(p.y)?,
            self.coord_to_key_checked_axis(p.z)?,
        ))
    }

    pub(crate) fn root_key() -> OcTreeKey {
        let c = Self::TREE_MAX_VAL as KeyType;
        OcTreeKey::new(c, c, c)
    }

    /// Read-only access to the root node, for [`crate::iter`]'s stack-based
    /// traversal.
    pub(crate) fn root(&self) -> Option<&Node> {
        self.root.as_deref()
    }

    /// Upstream `search(const OcTreeKey&, depth)`. `depth == 0` means the
    /// finest resolution, matching upstream's own `0`-means-full-depth
    /// convention.
    ///
    /// # Deviation: explicit `debug_assert!` added for `depth <= TREE_DEPTH` (Task G)
    ///
    /// Upstream's own precondition, `assert(depth <= tree_depth);`
    /// (`OcTreeBaseImpl.hxx:435`), was previously reproduced only by
    /// accident: `TREE_DEPTH - depth`'s unsigned underflow for an
    /// out-of-range `depth` happened to produce an empty
    /// `(diff..TREE_DEPTH)` range below, which looks like a safe skip but is
    /// **not** upstream's own behaviour. Upstream computes `int diff =
    /// tree_depth - depth` in *unsigned* arithmetic first, landing a huge
    /// value that is then reinterpreted as *negative* on assignment to the
    /// signed `int diff` -- its loop then runs `tree_depth` extra
    /// iterations down to a negative index and calls `computeChildIdx` with
    /// a negative shift amount, unconditional UB, not a clean skip. The two
    /// were never symmetric; the accidental skip here only looked safe
    /// because every current call site happens to pass `depth == 0`.
    /// `search` is `pub(crate)`, so a future same-crate caller could break
    /// that accident silently; this makes the precondition explicit and
    /// tested instead of implicit.
    pub(crate) fn search(&self, key: OcTreeKey, depth: u32) -> Option<&Node> {
        let mut cur = self.root.as_deref()?;
        let depth = if depth == 0 { Self::TREE_DEPTH } else { depth };
        debug_assert!(
            depth <= Self::TREE_DEPTH,
            "search: depth {depth} exceeds TREE_DEPTH ({})",
            Self::TREE_DEPTH
        );
        let diff = Self::TREE_DEPTH - depth;
        for i in (diff..Self::TREE_DEPTH).rev() {
            let pos = compute_child_idx(key, i) as usize;
            match cur.child(pos) {
                Some(child) => cur = child,
                None => {
                    return if cur.has_children() { None } else { Some(cur) };
                }
            }
        }
        Some(cur)
    }

    /// Log-odds occupancy at `point`, or `None` if unmapped (out of tree
    /// bounds, or never touched by an update). Convenience wrapper; not a
    /// direct upstream method (upstream callers use `search` + `getLogOdds`
    /// inline).
    pub fn log_odds_at(&self, point: Point3<f64>) -> Option<f32> {
        let key = self.coord_to_key_checked(point)?;
        self.search(key, 0).map(|n| n.log_odds)
    }

    /// Occupancy probability at `point` (`[0, 1]`), or `None` if unmapped.
    pub fn occupancy_at(&self, point: Point3<f64>) -> Option<f64> {
        self.log_odds_at(point).map(|lo| probability(f64::from(lo)))
    }

    /// Upstream `isNodeOccupied` applied to whatever `search` finds at
    /// `point`, or `None` if unmapped.
    pub fn is_occupied(&self, point: Point3<f64>) -> Option<bool> {
        self.log_odds_at(point)
            .map(|lo| self.is_node_occupied_log_odds(lo))
    }

    /// Upstream `getNumLeafNodes`/`getNumLeafNodesRecurs`: a fresh recursive
    /// count, not a maintained counter -- upstream itself recomputes this on
    /// every call rather than tracking it incrementally, so this port does
    /// too.
    pub fn num_leaf_nodes(&self) -> usize {
        fn recurs(node: &Node) -> usize {
            if !node.has_children() {
                return 1;
            }
            (0..8).filter_map(|i| node.child(i)).map(recurs).sum()
        }
        self.root.as_deref().map(recurs).unwrap_or(0)
    }

    /// Total node count (inner + leaf), for tests observing that [`Self::prune`]
    /// actually collapsed a subtree. Not a direct upstream method (upstream's
    /// `size()` returns a maintained `tree_size` counter this port does not
    /// keep, for the same reason it does not keep one for leaf counts).
    pub fn num_nodes(&self) -> usize {
        fn recurs(node: &Node) -> usize {
            1 + (0..8)
                .filter_map(|i| node.child(i))
                .map(recurs)
                .sum::<usize>()
        }
        self.root.as_deref().map(recurs).unwrap_or(0)
    }

    /// Upstream `updateNode(const OcTreeKey&, float, bool)`: adds
    /// `log_odds_update` to the node at `key`, clamped to the tree's
    /// clamping thresholds. Upstream's early-return optimization for an
    /// already-saturated node (`search` first, skip if already at threshold
    /// and the update would push further past it) is not ported: it is a
    /// pure performance optimization upstream's own doc comment calls out
    /// as situational ("may cause an overhead in some configuration"), and
    /// skipping it cannot change the final clamped value, only how many
    /// nodes get transiently expanded and re-pruned to reach it.
    pub fn update_node_log_odds_by_key(
        &mut self,
        key: OcTreeKey,
        log_odds_update: f32,
        lazy_eval: bool,
    ) {
        let params = UpdateParams {
            log_odds_update,
            lazy_eval,
            clamp_min: self.clamping_thres_min,
            clamp_max: self.clamping_thres_max,
        };
        let created_root = self.root.is_none();
        if created_root {
            self.root = Some(Box::new(Node::new()));
        }
        update_node_recurs(
            self.root.as_mut().expect("just ensured"),
            created_root,
            key,
            0,
            &params,
        );
    }

    /// Upstream `updateNode(const OcTreeKey&, bool, bool)`.
    pub fn update_node_by_key(&mut self, key: OcTreeKey, occupied: bool, lazy_eval: bool) {
        let log_odds = if occupied {
            self.prob_hit_log
        } else {
            self.prob_miss_log
        };
        self.update_node_log_odds_by_key(key, log_odds, lazy_eval);
    }

    /// Upstream `updateNode(const point3d&, float, bool)`. Returns `false`
    /// if `point` is out of the tree's representable range (matching
    /// upstream returning a null node pointer).
    pub fn update_node_log_odds(
        &mut self,
        point: Point3<f64>,
        log_odds_update: f32,
        lazy_eval: bool,
    ) -> bool {
        match self.coord_to_key_checked(point) {
            Some(key) => {
                self.update_node_log_odds_by_key(key, log_odds_update, lazy_eval);
                true
            }
            None => false,
        }
    }

    /// Upstream `updateNode(const point3d&, bool, bool)`.
    pub fn update_node(&mut self, point: Point3<f64>, occupied: bool, lazy_eval: bool) -> bool {
        let log_odds = if occupied {
            self.prob_hit_log
        } else {
            self.prob_miss_log
        };
        self.update_node_log_odds(point, log_odds, lazy_eval)
    }

    /// Upstream `updateInnerOccupancy`/`updateInnerOccupancyRecurs`: after a
    /// batch of `lazy_eval = true` updates, propagate children's occupancy
    /// up through the inner nodes they left stale. This is the second half
    /// of the "lazy-eval update pattern" the sensor pipeline depends on
    /// (`pointcloud_octomap_updater.cpp`/`lazy_free_space_updater.cpp`
    /// coalesce many rays' worth of key updates before touching the tree at
    /// all; this port's equivalent caller would do the coalescing the same
    /// way those do, then call this once instead of re-pruning after every
    /// single key).
    pub fn update_inner_occupancy(&mut self) {
        fn recurs(node: &mut Node) {
            if node.has_children() {
                for i in 0..8 {
                    if let Some(child) = node.child_mut(i) {
                        recurs(child);
                    }
                }
                node.update_occupancy_from_children();
            }
        }
        if let Some(root) = self.root.as_mut() {
            recurs(root);
        }
    }

    /// Upstream `prune`/`pruneRecurs`: a bottom-up sweep collapsing every
    /// collapsible subtree, one tree level at a time, stopping as soon as a
    /// level collapses nothing.
    pub fn prune(&mut self) {
        fn recurs(node: &mut Node, depth: u32, max_depth: u32, num_pruned: &mut usize) {
            if depth < max_depth {
                for i in 0..8 {
                    if let Some(child) = node.child_mut(i) {
                        recurs(child, depth + 1, max_depth, num_pruned);
                    }
                }
            } else if node.prune() {
                *num_pruned += 1;
            }
        }
        let Some(root) = self.root.as_mut() else {
            return;
        };
        for depth in (1..Self::TREE_DEPTH).rev() {
            let mut num_pruned = 0;
            recurs(root, 0, depth, &mut num_pruned);
            if num_pruned == 0 {
                break;
            }
        }
    }

    /// Upstream `computeRayKeys`: Amanatides & Woo fast voxel traversal
    /// ("A Faster Voxel Traversal Algorithm for Ray Tracing"), a 3D DDA.
    /// Returns the keys of every voxel the ray from `origin` to `end`
    /// passes through, excluding `end` itself. `None` if either endpoint is
    /// out of the tree's representable range (matching upstream's `false`).
    pub fn compute_ray_keys(&self, origin: Point3<f64>, end: Point3<f64>) -> Option<KeyRay> {
        let key_origin = self.coord_to_key_checked(origin)?;
        let key_end = self.coord_to_key_checked(end)?;

        let mut ray = KeyRay::new();
        if key_origin == key_end {
            return Some(ray);
        }
        ray.push(key_origin);

        let direction: Vector3<f64> = end - origin;
        let length = direction.norm();
        let direction = direction / length;

        let mut step = [0i32; 3];
        let mut t_max = [f64::MAX; 3];
        let mut t_delta = [f64::MAX; 3];
        let mut current_key = key_origin;

        for i in 0..3 {
            step[i] = if direction[i] > 0.0 {
                1
            } else if direction[i] < 0.0 {
                -1
            } else {
                0
            };
            if step[i] != 0 {
                let voxel_border = self.key_to_coord_axis(current_key[i])
                    + f64::from(step[i]) * self.resolution * 0.5;
                t_max[i] = (voxel_border - origin[i]) / direction[i];
                t_delta[i] = self.resolution / direction[i].abs();
            }
        }

        loop {
            let dim = if t_max[0] < t_max[1] {
                if t_max[0] < t_max[2] { 0 } else { 2 }
            } else if t_max[1] < t_max[2] {
                1
            } else {
                2
            };

            current_key = step_key(current_key, dim, step[dim]);
            t_max[dim] += t_delta[dim];

            if current_key == key_end {
                break;
            }

            let dist_from_origin = t_max[0].min(t_max[1]).min(t_max[2]);
            if dist_from_origin > length {
                break;
            }
            ray.push(current_key);
        }

        Some(ray)
    }

    /// Upstream `computeUpdate`: the batch helper behind `insertPointCloud`,
    /// computing every free and occupied key a point cloud touches at once
    /// (occupied cells win ties -- the sets are made disjoint, occupied
    /// side). This port omits upstream's BBX-limited branch (`useBBXLimit`)
    /// since no moveit2 caller ever enables it.
    ///
    /// `max_range` is upstream's `double maxrange`, whose negative values
    /// mean "unlimited". `None` says that directly; `Some(r)` with `r <
    /// 0.0` still means it too, since a caller translating an upstream
    /// call site keeps the sentinel it was given.
    pub fn compute_update(
        &self,
        points: &[Point3<f64>],
        origin: Point3<f64>,
        max_range: Option<f64>,
    ) -> (KeySet, KeySet) {
        let mut free_cells = KeySet::new();
        let mut occupied_cells = KeySet::new();

        for &p in points {
            // `(maxrange < 0.0) || ((p - origin).norm() <= maxrange)`
            // verbatim (`OccupancyOcTreeBase.hxx:190`). The `r < 0.0`
            // disjunct is not redundant with `None`: upstream's parameter
            // is a plain `double` whose negative values mean "unlimited",
            // so a caller porting `computeUpdate(..., -1.0, ...)` passes
            // `Some(-1.0)` and must get the unlimited branch. Its
            // counterpart in `insert_ray` is `maxrange > 0`, not the
            // negation of this one -- upstream really does treat `0.0`
            // and `NaN` differently between the two, so neither can be
            // factored into a shared normalisation of `max_range`.
            let within_range = max_range.is_none_or(|r| r < 0.0 || (p - origin).norm() <= r);
            if within_range {
                if let Some(ray) = self.compute_ray_keys(origin, p) {
                    free_cells.extend(ray);
                }
                if let Some(key) = self.coord_to_key_checked(p) {
                    occupied_cells.insert(key);
                }
            } else if let Some(r) = max_range {
                // Upstream `point3d direction = (p - origin).normalized ();`
                // (`OccupancyOcTreeBase.hxx:211`) -- octomath's `normalized()`
                // (`third_party/octomap/octomap/include/octomap/math/
                // Vector3.h:270-276`) leaves a zero vector unchanged; plain
                // `.normalize()` divides `0.0 / 0.0` instead. This branch's
                // own guard (`within_range` false with `r >= 0.0`) makes
                // `(p - origin).norm() > r >= 0.0` hold for every finite
                // `r`, but a caller-supplied `max_range = Some(f64::NAN)`
                // slips past both disjuncts of `within_range`'s closure
                // (`NAN < 0.0` and `norm() <= NAN` are both false) without
                // that inequality ever holding, so `p == origin` is
                // reachable here after all -- direction genuinely becomes
                // `(NaN, NaN, NaN)`.
                //
                // Not a defect, though: `coord_to_key_checked_axis`'s own
                // `!(scaled_f >= min && scaled_f < max)` guard independently
                // rejects a `NaN` coordinate before a key is ever built from
                // it, so `new_end`'s `NaN` never reaches `free_cells` or
                // `occupied_cells` -- measured directly, not just argued: a
                // temporary local revert to plain `.normalize()` here
                // (`(p - origin).normalize()`, no `try_normalize`) produces
                // byte-identical output from `compute_update_nan_max_range_
                // at_a_coincident_point_stays_empty_not_nan_poisoned`
                // (below) to the guarded version -- the return value cannot
                // distinguish a `NaN` direction from a zero one here, so
                // there is no observable divergence from upstream to fix.
                let direction = (p - origin).normalize();
                let new_end = origin + direction * r;
                if let Some(ray) = self.compute_ray_keys(origin, new_end) {
                    free_cells.extend(ray);
                }
            }
        }

        free_cells.retain(|k| !occupied_cells.contains(k));
        (free_cells, occupied_cells)
    }

    /// Upstream `integrateMissOnRay`.
    fn integrate_miss_on_ray(
        &mut self,
        origin: Point3<f64>,
        end: Point3<f64>,
        lazy_eval: bool,
    ) -> bool {
        match self.compute_ray_keys(origin, end) {
            Some(ray) => {
                let miss = self.prob_miss_log;
                for key in ray {
                    self.update_node_log_odds_by_key(key, miss, lazy_eval);
                }
                true
            }
            None => false,
        }
    }

    /// Upstream `begin_leafs()`/`end_leafs()`: iterate every leaf (a node
    /// with no children, at whatever depth it was pruned to) in the tree.
    /// See [`crate::iter::Leaves`].
    pub fn leaves(&self) -> crate::iter::Leaves<'_> {
        crate::iter::Leaves::new(self)
    }

    /// Upstream `begin_leafs_bbx(min, max)`/`end_leafs_bbx()`: iterate every
    /// leaf whose voxel overlaps the axis-aligned box `[min, max]`. `None`
    /// if `min` or `max` is out of the tree's representable range. See
    /// [`crate::iter::LeavesInBbx`].
    pub fn leaves_in_bbx(
        &self,
        min: Point3<f64>,
        max: Point3<f64>,
    ) -> Option<crate::iter::LeavesInBbx<'_>> {
        crate::iter::LeavesInBbx::new(self, min, max)
    }

    /// Upstream `begin_tree()`/`end_tree()`: iterate every node in the tree,
    /// inner nodes and leaves alike, in pre-order. See
    /// [`crate::iter::TreeNodes`].
    pub fn tree_nodes(&self) -> crate::iter::TreeNodes<'_> {
        crate::iter::TreeNodes::new(self)
    }

    /// Upstream `insertRay`: integrates a miss along the whole ray and, if
    /// the ray was not cut short by `max_range`, a hit at `end`. A ray cut
    /// short by `max_range` records only the miss up to the cut point --
    /// upstream does not guess at occupancy beyond the sensor's range.
    ///
    /// Only a strictly positive `max_range` cuts anything: upstream's
    /// guard is `maxrange > 0`, so `None`, `Some(0.0)`, `Some(negative)`
    /// and `Some(NaN)` all insert the complete ray. Note this is *not*
    /// [`Self::compute_update`]'s rule, which cuts at `0.0` and at `NaN`.
    pub fn insert_ray(
        &mut self,
        origin: Point3<f64>,
        end: Point3<f64>,
        max_range: Option<f64>,
        lazy_eval: bool,
    ) -> bool {
        // `(maxrange > 0) && ((end - origin).norm() > maxrange)` verbatim
        // (`OccupancyOcTreeBase.hxx:868`) -- see `compute_update` for why
        // the `r > 0.0` half cannot be folded into `max_range`'s `None`.
        if let Some(r) = max_range
            && r > 0.0
            && (end - origin).norm() > r
        {
            // Upstream `point3d direction = (end - origin).normalized ();`
            // (`OccupancyOcTreeBase.hxx:870`) is octomath-guarded (leaves a
            // zero vector unchanged, same contract as `compute_update`'s
            // `:1243` above) but that guard is unreachable here, not
            // dropped: this branch's own `r > 0.0 && norm() > r` conjunction
            // forces `norm() > 0` for every value of `r`, `NaN` included --
            // `NaN > 0.0` is `false`, so a `NaN` `max_range` fails the first
            // conjunct and never reaches this line at all (unlike
            // `compute_update`'s `is_none_or` gate, whose `NAN < 0.0 ||
            // norm() <= NAN` disjunction is false for a *different* reason
            // and lets `NaN` through). Plain `.normalize()` stays correct
            // by construction of this guard, not upstream's guard.
            let direction = (end - origin).normalize();
            let new_end = origin + direction * r;
            return self.integrate_miss_on_ray(origin, new_end, lazy_eval);
        }
        if !self.integrate_miss_on_ray(origin, end, lazy_eval) {
            return false;
        }
        self.update_node(end, true, lazy_eval);
        true
    }

    /// Upstream `readBinaryData`/`readBinaryNode`: decode the compact,
    /// lossy wire format `moveit_msgs::Octomap.data` carries when
    /// `msg.binary == true` (`writeBinaryData`'s exact inverse; see
    /// `lib.rs`'s "Round 27, item 1(a)" for the full format derivation).
    /// `self` must be a freshly constructed, empty tree -- matching
    /// upstream's own refusal to decode into a tree that already has a
    /// root, see [`DecodeError::TreeAlreadyPopulated`]. `self.resolution`
    /// must also be a positive, finite value -- see
    /// [`DecodeError::InvalidResolution`] for why that is checked here
    /// rather than in [`Self::new`].
    ///
    /// Each node's own 2-byte record packs its 8 children 2 bits each: `10`
    /// free leaf (read back as exactly [`Self::clamping_thres_min_log`]),
    /// `01` occupied leaf (exactly [`Self::clamping_thres_max_log`]), `11`
    /// has children (recurse; the child's own log-odds is corrected to the
    /// max of its children after that recursion returns, via the private
    /// `update_occupancy_from_children`), `00` absent. **A leaf's
    /// true log-odds is not preserved -- only which side of the
    /// occupied/free split it fell on.**
    ///
    /// # Trailing bytes are accepted, matching upstream's message path
    ///
    /// Neither this function nor upstream's `readBinaryNode` checks that the
    /// cursor/stream is exhausted once the recursive decode returns --
    /// `vec![1, 2, 3]` decodes as two leaves from the first 2 bytes and
    /// returns `Ok`, silently ignoring byte 3, and this is exact parity, not
    /// a gap. `octomap_msgs::readTree`
    /// (`/opt/ros/rolling/include/octomap_msgs/octomap_msgs/conversions.h`,
    /// the `moveit_msgs::Octomap` message path this decodes for) writes
    /// `msg.data` straight into a `std::stringstream` and calls
    /// `octree->readBinaryData(datastream)` with no length header and no
    /// exhaustion check of its own. Upstream *does* have an integrity check
    /// -- `AbstractOccupancyOcTree::readBinary`
    /// (`third_party/octomap/octomap/src/AbstractOccupancyOcTree.cpp:172-176`)
    /// compares `size != this->size()` -- but `size` there comes from the
    /// `.bt` *file* header (`# Octomap OcTree binary file` plus a node
    /// count line), which the message path never has: a
    /// `moveit_msgs::Octomap` carries no analogous count field. So the file
    /// path can detect a short read and the message path structurally
    /// cannot, upstream included -- this function has no less information
    /// than `readBinaryNode` does, it is simply never given a count to
    /// check against in the first place.
    ///
    /// **§153.1 expiry:** if a future round decides to reject trailing
    /// bytes here, that is a deliberate *deviation* from upstream's
    /// documented message-path behavior, not a bug fix -- it needs sign-off
    /// and a doc update here and on [`Self::read_data`], not a silent
    /// "hardening".
    pub fn read_binary_data(&mut self, bytes: &[u8]) -> Result<(), DecodeError> {
        if self.root.is_some() {
            return Err(DecodeError::TreeAlreadyPopulated);
        }
        if !(self.resolution.is_finite() && self.resolution > 0.0) {
            return Err(DecodeError::InvalidResolution);
        }
        let params = BinaryReadParams {
            clamp_min: self.clamping_thres_min,
            clamp_max: self.clamping_thres_max,
        };
        let mut cursor = Cursor::new(bytes);
        let mut root = Node::new();
        read_binary_node(&mut cursor, &mut root, 0, &params)?;
        self.root = Some(Box::new(root));
        Ok(())
    }

    /// Upstream `OcTreeBaseImpl::readData`/`readNodesRecurs` +
    /// `OcTreeDataNode::readData`: decode the full, lossless wire format
    /// `moveit_msgs::Octomap.data` carries when `msg.binary == false`
    /// (`writeData`'s exact inverse; see `lib.rs`'s "Round 27, item 1(a)").
    /// `self` must be a freshly constructed, empty tree, and `self.resolution`
    /// must be a positive, finite value, for the same reasons as
    /// [`Self::read_binary_data`].
    ///
    /// Each node is written depth-first as its own raw `f32` log-odds (a
    /// direct little-endian read, see the private `Cursor::read_f32_le`'s
    /// doc for why) followed by 1 byte with 1 bit per child (bit set = child exists,
    /// recurse in index order). Every node's exact log-odds survives this
    /// format's round trip; nothing here is quantized the way
    /// [`Self::read_binary_data`]'s leaves are.
    ///
    /// # Trailing bytes are accepted, for the same reason as `read_binary_data`
    ///
    /// This function does not check that the cursor is exhausted once the
    /// recursive decode returns either -- 5 real bytes plus 32 bytes of
    /// junk still decodes `Ok`. `octomap_msgs::fullMsgToMap`
    /// (`/opt/ros/rolling/include/octomap_msgs/octomap_msgs/conversions.h`,
    /// the `msg.binary == false` counterpart to `readTree`) writes
    /// `msg.data` into a `std::stringstream` and calls
    /// `tree->readData(datastream)` directly, with the same absence of a
    /// length header or exhaustion check -- see [`Self::read_binary_data`]'s
    /// doc for the full citation and why the file-path integrity check
    /// (`AbstractOccupancyOcTree::readBinary`'s `size != this->size()`) has
    /// no counterpart on this message path. Same §153.1 expiry: rejecting
    /// trailing bytes here would be a deviation requiring sign-off, not a
    /// fix.
    pub fn read_data(&mut self, bytes: &[u8]) -> Result<(), DecodeError> {
        if self.root.is_some() {
            return Err(DecodeError::TreeAlreadyPopulated);
        }
        if !(self.resolution.is_finite() && self.resolution > 0.0) {
            return Err(DecodeError::InvalidResolution);
        }
        let mut cursor = Cursor::new(bytes);
        let mut root = Node::new();
        read_data_node(&mut cursor, &mut root, 0)?;
        self.root = Some(Box::new(root));
        Ok(())
    }

    /// Upstream `OccupancyOcTreeBase::writeBinaryData`: the exact inverse of
    /// [`Self::read_binary_data`], encoding this tree into the compact,
    /// lossy wire format `moveit_msgs::Octomap.data` carries when
    /// `msg.binary == true`. Upstream's own `if (this->root) ...` guard
    /// (`writeBinaryData` never writes when the tree is empty) is preserved
    /// exactly -- an empty tree encodes to an empty `Vec`, which is *not*
    /// itself decodable by [`Self::read_binary_data`] (that always needs at
    /// least the root's own 2-byte record); this is upstream's own
    /// asymmetry, not one this port introduces.
    pub fn write_binary_data(&self) -> Vec<u8> {
        let mut out = Vec::new();
        if let Some(root) = &self.root {
            write_binary_node(self, root, &mut out);
        }
        out
    }

    /// Upstream `OcTreeBaseImpl::writeData`: the exact inverse of
    /// [`Self::read_data`], encoding this tree into the full, lossless wire
    /// format `moveit_msgs::Octomap.data` carries when `msg.binary ==
    /// false`. Same empty-tree asymmetry as [`Self::write_binary_data`]:
    /// upstream's `if (root) writeNodesRecurs(root, s);` writes nothing for
    /// an empty tree, and that empty output does not round-trip through
    /// [`Self::read_data`] either.
    pub fn write_data(&self) -> Vec<u8> {
        let mut out = Vec::new();
        if let Some(root) = &self.root {
            write_data_node(root, &mut out);
        }
        out
    }
}

/// Upstream `computeChildKey`'s `+/-1` step on one axis, in `OcTreeKey`
/// space. Upstream relies on `key_type` (`uint16_t`) wraparound at the
/// bounds of representable key-space; `wrapping_add`/`wrapping_sub` is the
/// literal Rust equivalent of that well-defined C++ unsigned overflow.
fn step_key(key: OcTreeKey, axis: usize, step: i32) -> OcTreeKey {
    let mut k = [key[0], key[1], key[2]];
    k[axis] = match step {
        1 => k[axis].wrapping_add(1),
        -1 => k[axis].wrapping_sub(1),
        _ => k[axis],
    };
    OcTreeKey::new(k[0], k[1], k[2])
}

/// Bundles one `updateNode` call's per-call parameters so
/// [`update_node_recurs`] takes one value per genuinely distinct piece of
/// recursion state (`node`, `node_just_created`, `key`, `depth`) instead of
/// growing an argument per field; the alternative was `#[allow(clippy::
/// too_many_arguments)]`, which this project's lint policy forbids as a way
/// to silence rather than fix a shape problem.
struct UpdateParams {
    log_odds_update: f32,
    lazy_eval: bool,
    clamp_min: f32,
    clamp_max: f32,
}

/// Upstream `OccupancyOcTreeBase::updateNodeRecurs`.
fn update_node_recurs(
    node: &mut Node,
    node_just_created: bool,
    key: OcTreeKey,
    depth: u32,
    params: &UpdateParams,
) {
    if depth < OcTree::TREE_DEPTH {
        let pos = compute_child_idx(key, OcTree::TREE_DEPTH - 1 - depth) as usize;
        let created_now = if node.child(pos).is_none() {
            if !node.has_children() && !node_just_created {
                node.expand();
                false
            } else {
                node.create_child(pos);
                true
            }
        } else {
            false
        };
        update_node_recurs(
            node.child_mut(pos).expect("just ensured to exist"),
            created_now,
            key,
            depth + 1,
            params,
        );
        if !params.lazy_eval && !node.prune() {
            node.update_occupancy_from_children();
        }
    } else {
        node.log_odds = clamp_log_odds(
            node.log_odds + params.log_odds_update,
            params.clamp_min,
            params.clamp_max,
        );
    }
}

/// `updateNodeLogOdds`'s clamp
/// (`third_party/octomap/octomap/include/octomap/OccupancyOcTreeBase.hxx:1091-1100`),
/// as upstream writes it: two one-sided comparisons, low bound first.
///
/// Not `f32::clamp`. `clamp` carries a `min <= max` precondition and panics
/// when it does not hold, or when either bound is `NaN`. Upstream carries no
/// such precondition and cannot panic —
/// [`OcTree::set_clamping_thres_min`](crate::OcTree::set_clamping_thres_min)
/// and [`OcTree::set_clamping_thres_max`](crate::OcTree::set_clamping_thres_max)
/// are public and validate nothing, exactly like the
/// `setClampingThresMin`/`setClampingThresMax` they port, so a caller can
/// leave `min > max` set, and `logodds(prob)` yields `NaN` for `prob` outside
/// `[0, 1]`. On the inverted-bound input upstream's first `if` fires and
/// returns `min`; a `NaN` value fails both comparisons and passes through
/// unchanged. This reproduces all three.
fn clamp_log_odds(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// A read cursor over a byte slice, for [`read_binary_node`]/
/// [`read_data_node`]. Not an upstream type -- upstream reads directly from
/// a `std::istream`, whose short-read behaviour
/// ([`crate::error::DecodeError`]'s doc comment) this port replaces with an
/// explicit `Result` at every read.
struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    fn read_u8(&mut self) -> Result<u8, DecodeError> {
        let byte = *self.bytes.get(self.pos).ok_or(DecodeError::UnexpectedEof)?;
        self.pos += 1;
        Ok(byte)
    }

    /// Upstream `s.read((char*)&value, sizeof(value))`: a raw memory dump,
    /// so a direct byte-for-byte read with no framing to validate --
    /// native-endian on the machine that wrote it. **Deviation (§153.1):**
    /// upstream has zero explicit endianness handling anywhere in
    /// `third_party/octomap` (`rg -ni endian` across `include/octomap/` and
    /// `src/` finds no hits), so there is no contract to read, only the
    /// native-endian fact the memory dump implies. This port hardcodes
    /// `from_le_bytes` because every producer in this workspace's actual
    /// reach -- the CI runners and the oracle container this crate's own
    /// fixtures are captured against -- is little-endian x86_64/aarch64.
    /// **Expiry condition:** a big-endian `octomap_msgs::Octomap` producer
    /// (e.g. a big-endian ROS 2 node on the wire) would decode every
    /// log-odds value wrong with no error raised -- silently, since a valid
    /// `f32` bit pattern read byte-swapped is still a valid, merely wrong,
    /// `f32`. That is a known limitation of this port, not a bug in it.
    fn read_f32_le(&mut self) -> Result<f32, DecodeError> {
        let end = self.pos + 4;
        let slice = self
            .bytes
            .get(self.pos..end)
            .ok_or(DecodeError::UnexpectedEof)?;
        self.pos = end;
        Ok(f32::from_le_bytes(
            slice.try_into().expect("slice length checked above"),
        ))
    }
}

/// A node whose grandchildren-not-yet-read state [`read_binary_node`] must
/// distinguish from "already resolved to a real clamp value" -- upstream
/// `getNodeChild(node, i)->setLogOdds(-200.)`, `readBinaryNode`'s own
/// comment: "child is unkown, we leave it uninitialized" until its own
/// subtree has been read.
const UNKNOWN_SENTINEL_LOG_ODDS: f32 = -200.0;

/// Bundles [`OcTree::read_binary_data`]'s clamp thresholds for
/// [`read_binary_node`], matching [`UpdateParams`]'s reasoning for grouping
/// per-call constants into one value instead of adding parameters.
struct BinaryReadParams {
    clamp_min: f32,
    clamp_max: f32,
}

/// Sets `node`'s children 0..4 (if `base_idx == 0`) or 4..8 (if `base_idx ==
/// 4`) from one packed byte: bit `2*i` and `2*i+1` (LSB first) are child
/// `base_idx + i`'s 2-bit code. Upstream `readBinaryNode`'s two near-
/// identical `for` loops (children 0-3 from `child1to4`, 4-7 from
/// `child5to8`), fused into one since the only difference between them is
/// which byte and which base index, matching this crate's usual "same
/// shape, one axis fused" style ([`OcTreeKey::new`]'s callers, for one).
fn create_binary_children(node: &mut Node, base_idx: u8, byte: u8, params: &BinaryReadParams) {
    for i in 0..4u8 {
        let low = (byte >> (i * 2)) & 1;
        let high = (byte >> (i * 2 + 1)) & 1;
        let idx = (base_idx + i) as usize;
        match (low, high) {
            (1, 0) => node.create_child(idx).log_odds = params.clamp_min, // free leaf
            (0, 1) => node.create_child(idx).log_odds = params.clamp_max, // occupied leaf
            (1, 1) => node.create_child(idx).log_odds = UNKNOWN_SENTINEL_LOG_ODDS, // has children
            _ => {}                                                       // (0, 0): no child
        }
    }
}

/// Upstream `OccupancyOcTreeBase::readBinaryNode`. `depth` is `node`'s own
/// depth (root: 0), matching [`update_node_recurs`]'s convention.
fn read_binary_node(
    cursor: &mut Cursor,
    node: &mut Node,
    depth: u32,
    params: &BinaryReadParams,
) -> Result<(), DecodeError> {
    let child1to4 = cursor.read_u8()?;
    let child5to8 = cursor.read_u8()?;

    // Upstream: "inner nodes default to occupied" -- set unconditionally,
    // before children are even inspected. A recursed-into child gets this
    // corrected below once its own subtree is read
    // ([`Node::update_occupancy_from_children`]); the outermost root never
    // does (`readBinaryData` never revisits it after this call returns), so
    // a decoded tree's root log-odds is always `clamp_max` regardless of
    // its actual content -- a faithfully-ported upstream quirk, not
    // something this port introduces.
    node.log_odds = params.clamp_max;

    create_binary_children(node, 0, child1to4, params);
    create_binary_children(node, 4, child5to8, params);

    for i in 0..8usize {
        if let Some(child) = node.child_mut(i)
            && child.log_odds == UNKNOWN_SENTINEL_LOG_ODDS
        {
            // A depth-15 node's children live at depth 16, the finest
            // representable level (a 16-bit key has no bit left to split
            // further) -- so a "has children" code on one of THOSE children
            // describes a node this format cannot legally contain. Checked
            // before recursing, not inside the next call, so a crafted
            // input hits `MaxDepthExceeded` instead of growing the call
            // stack past the one depth a real tree can ever reach.
            if depth + 1 >= OcTree::TREE_DEPTH {
                return Err(DecodeError::MaxDepthExceeded);
            }
            read_binary_node(cursor, child, depth + 1, params)?;
            child.update_occupancy_from_children();
        }
    }
    Ok(())
}

/// Upstream `OcTreeBaseImpl::readNodesRecurs` + `OcTreeDataNode::readData`
/// (the latter is exactly `node.log_odds = cursor.read_f32_le()?`, fused in
/// here rather than kept as a separate one-line function). `depth` is
/// `node`'s own depth, same convention as [`read_binary_node`].
fn read_data_node(cursor: &mut Cursor, node: &mut Node, depth: u32) -> Result<(), DecodeError> {
    node.log_odds = cursor.read_f32_le()?;
    let children = cursor.read_u8()?;

    for i in 0..8usize {
        if (children >> i) & 1 == 1 {
            // Unlike `read_binary_node`, a depth-16 node here DOES get its
            // own record (this format has no per-parent quantization to
            // pack leaves into) -- the invalid case is a depth-16 node's
            // own bitset claiming a depth-17 child, so the guard compares
            // against `depth`, not `depth + 1`.
            if depth >= OcTree::TREE_DEPTH {
                return Err(DecodeError::MaxDepthExceeded);
            }
            let child = node.create_child(i);
            read_data_node(cursor, child, depth + 1)?;
        }
    }
    Ok(())
}

/// Sets child `base_idx + i`'s 2-bit code in `byte` (bit `2*i` low, `2*i+1`
/// high, matching [`create_binary_children`]'s read-side convention exactly
/// so the two are inverses): `00` absent, `10` free leaf, `01` occupied
/// leaf, `11` has children. Upstream `writeBinaryNode`'s two near-identical
/// `for` loops (children 0-3 into `child1to4`, 4-7 into `child5to8`), fused
/// the same way [`create_binary_children`] fuses the read side.
fn set_binary_child_bits(tree: &OcTree, byte: &mut u8, i: u8, child: Option<&Node>) {
    let (low, high) = match child {
        None => (0u8, 0u8),
        Some(c) if c.has_children() => (1, 1),
        Some(c) if tree.is_node_occupied_log_odds(c.log_odds) => (0, 1),
        Some(_) => (1, 0),
    };
    *byte |= low << (i * 2);
    *byte |= high << (i * 2 + 1);
}

/// Upstream `OccupancyOcTreeBase::writeBinaryNode`, the exact inverse of
/// [`read_binary_node`]. `node`'s own `log_odds` is never written -- like
/// the read side, only which of its *children* are present/free/
/// occupied/branching is encoded; a root's real value is not part of this
/// format on either side of the round trip.
fn write_binary_node(tree: &OcTree, node: &Node, out: &mut Vec<u8>) {
    let mut child1to4 = 0u8;
    let mut child5to8 = 0u8;
    for i in 0..4u8 {
        set_binary_child_bits(tree, &mut child1to4, i, node.child(i as usize));
    }
    for i in 0..4u8 {
        set_binary_child_bits(tree, &mut child5to8, i, node.child((i + 4) as usize));
    }
    out.push(child1to4);
    out.push(child5to8);

    for i in 0..8usize {
        if let Some(child) = node.child(i)
            && child.has_children()
        {
            write_binary_node(tree, child, out);
        }
    }
}

/// Upstream `OcTreeBaseImpl::writeNodesRecurs` + `OcTreeDataNode::writeData`
/// (the latter is exactly `out.extend(node.log_odds.to_le_bytes())`, fused
/// in here rather than kept separate, matching [`read_data_node`]'s own
/// fusion of the read-side pair). Same little-endian §153.1 deviation as
/// [`Cursor::read_f32_le`]: a raw memory dump with no endianness contract in
/// upstream, hardcoded little-endian here to match every producer/consumer
/// this workspace actually reaches.
fn write_data_node(node: &Node, out: &mut Vec<u8>) {
    out.extend_from_slice(&node.log_odds.to_le_bytes());

    let mut children = 0u8;
    for i in 0..8usize {
        if node.child(i).is_some() {
            children |= 1 << i;
        }
    }
    out.push(children);

    for i in 0..8usize {
        if let Some(child) = node.child(i) {
            write_data_node(child, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_hits_converge_to_clamp_but_not_past_it() {
        let mut tree = OcTree::new(0.1);
        let p = Point3::new(0.05, 0.05, 0.05);
        for _ in 0..1000 {
            tree.update_node(p, true, false);
        }
        assert_eq!(tree.log_odds_at(p).unwrap(), tree.clamping_thres_max_log());
        assert!(tree.is_occupied(p).unwrap());
    }

    /// `setClampingThresMin`/`setClampingThresMax` validate nothing
    /// upstream, and neither do their ports, so `min > max` is a state a
    /// caller can leave the tree in. Upstream's `updateNodeLogOdds`
    /// (`OccupancyOcTreeBase.hxx:1091-1100`) tests the low bound first and
    /// returns, so an inverted pair yields `min`; `f32::clamp` panics on
    /// exactly this input, which is why this port does not use it.
    #[test]
    fn an_inverted_clamping_pair_yields_the_low_bound_rather_than_panicking() {
        let mut tree = OcTree::new(0.1);
        tree.set_clamping_thres_min(0.9);
        tree.set_clamping_thres_max(0.1);
        assert!(
            tree.clamping_thres_min_log() > tree.clamping_thres_max_log(),
            "setup must actually invert the pair"
        );

        let p = Point3::new(0.05, 0.05, 0.05);
        tree.update_node(p, true, false);

        assert_eq!(tree.log_odds_at(p).unwrap(), tree.clamping_thres_min_log());
    }

    /// `logodds(prob)` is `ln(prob / (1 - prob))`, so a `prob` above `1`
    /// makes the bound `NaN`. Upstream's two comparisons are both false
    /// against a `NaN` bound, leaving the accumulated value untouched;
    /// `f32::clamp` panics on a `NaN` bound instead.
    #[test]
    fn a_nan_clamping_bound_leaves_the_value_untouched_rather_than_panicking() {
        let mut tree = OcTree::new(0.1);
        tree.set_clamping_thres_max(2.0);
        assert!(
            tree.clamping_thres_max_log().is_nan(),
            "setup must actually produce a NaN bound"
        );

        let p = Point3::new(0.05, 0.05, 0.05);
        tree.update_node(p, true, false);

        let expected = tree.prob_hit_log();
        assert_eq!(tree.log_odds_at(p).unwrap(), expected);
    }

    #[test]
    fn miss_sequence_drives_occupancy_below_threshold() {
        let mut tree = OcTree::new(0.1);
        let p = Point3::new(0.05, 0.05, 0.05);
        tree.update_node(p, true, false);
        assert!(tree.is_occupied(p).unwrap());
        for _ in 0..10 {
            tree.update_node(p, false, false);
        }
        assert!(!tree.is_occupied(p).unwrap());
        assert!(tree.occupancy_at(p).unwrap() < 0.5);
        assert_eq!(tree.log_odds_at(p).unwrap(), tree.clamping_thres_min_log());
    }

    #[test]
    fn prune_collapses_eight_uniform_children_and_preserves_their_value() {
        let mut tree = OcTree::new(1.0);
        let hit = tree.prob_hit_log();
        // The 8 finest-level children of one common depth-15 parent: fix
        // every bit except the least significant one, which ranges over
        // {0, 1} on each axis. Two numerically adjacent keys are NOT
        // guaranteed to be siblings this way (e.g. 32767/32768 diverge at
        // the root, not the leaf) -- picking a key with no power-of-two
        // boundary nearby (100/101) avoids that trap.
        let mut keys = Vec::new();
        for &kx in &[100u16, 101] {
            for &ky in &[100u16, 101] {
                for &kz in &[100u16, 101] {
                    keys.push(OcTreeKey::new(kx, ky, kz));
                }
            }
        }
        for &k in &keys {
            // lazy_eval = true defers pruning, so all 8 leaves genuinely
            // exist as separate nodes for the explicit prune() below to
            // collapse, instead of the eager (non-lazy) path folding them
            // back together mid-batch after the 8th write.
            tree.update_node_log_odds_by_key(k, hit, true);
        }
        let nodes_before = tree.num_nodes();
        tree.prune();
        let nodes_after = tree.num_nodes();
        assert!(
            nodes_after < nodes_before,
            "prune should have collapsed the 8 identical siblings (before={nodes_before}, after={nodes_after})"
        );
        for &k in &keys {
            assert_eq!(tree.search(k, 0).unwrap().log_odds, hit);
        }
    }

    #[test]
    fn ray_with_end_outside_tree_bounds_returns_none() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        // resolution 0.1 with tree_max_val = 32768 represents roughly
        // [-3276.8, 3276.7]; well outside that on either side must fail.
        assert!(
            tree.compute_ray_keys(origin, Point3::new(1.0e9, 0.0, 0.0))
                .is_none()
        );
        assert!(
            tree.compute_ray_keys(origin, Point3::new(-1.0e9, 0.0, 0.0))
                .is_none()
        );
    }

    #[test]
    fn ray_with_origin_outside_tree_bounds_returns_none() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(1.0e9, 0.0, 0.0);
        let end = Point3::new(0.0, 0.0, 0.0);
        assert!(tree.compute_ray_keys(origin, end).is_none());
    }

    #[test]
    fn zero_log_odds_is_occupied_under_the_default_threshold() {
        // Documents the non-obvious upstream boundary: isNodeOccupied uses
        // `>=`, and the default occupancy threshold (probability 0.5) is
        // exactly log-odds 0.0, so a node that nets to exactly unknown
        // reads as occupied, not unoccupied.
        let tree = OcTree::new(0.1);
        assert_eq!(tree.occupancy_thres_log(), 0.0);
        assert!(tree.is_node_occupied_log_odds(0.0));
    }

    #[test]
    fn unmapped_coordinate_has_no_occupancy() {
        let tree = OcTree::new(0.1);
        let p = Point3::new(5.0, 5.0, 5.0);
        assert!(tree.log_odds_at(p).is_none());
        assert!(tree.is_occupied(p).is_none());
    }

    /// `log_odds_at`'s `None` has two structurally distinct causes --
    /// `coord_to_key_checked` rejecting an out-of-range coordinate, and
    /// `search` finding no node at an in-range key -- and neither this test
    /// nor `unmapped_coordinate_has_no_occupancy` nor the oracle parity
    /// fixtures (`octomap_parity.rs`, whose query points never leave
    /// tree bounds) previously exercised the first. Populate the key
    /// `coord_to_key_checked` would collapse to if that guard were dropped
    /// (the tree center, [`OcTree::root_key`]) so a version that silently
    /// clamped an out-of-bounds point instead of rejecting it would find
    /// this node and return `Some`, not `None`.
    #[test]
    fn out_of_bounds_coordinate_has_no_occupancy_even_when_the_tree_center_is_mapped() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.0, 0.0, 0.0), true, false);
        assert!(tree.log_odds_at(Point3::new(0.0, 0.0, 0.0)).is_some());

        let out_of_bounds = Point3::new(1e6, 1e6, 1e6);
        assert!(tree.log_odds_at(out_of_bounds).is_none());
        assert!(tree.is_occupied(out_of_bounds).is_none());
    }

    #[test]
    fn node_size_scales_by_depth() {
        let tree = OcTree::new(0.1);
        assert_eq!(tree.node_size(OcTree::TREE_DEPTH), 0.1);
        assert_eq!(
            tree.node_size(0),
            0.1 * f64::from(1u32 << OcTree::TREE_DEPTH)
        );
    }

    #[test]
    #[should_panic(expected = "node_size: depth")]
    fn node_size_rejects_depth_above_tree_depth() {
        // Pre-fix this was a bare `assert!` -- it already panicked, but with
        // the default "assertion failed: ..." text, not this message; the
        // custom message is what distinguishes `debug_assert!` having
        // actually landed here rather than the check merely surviving
        // unedited.
        let tree = OcTree::new(0.1);
        let _ = tree.node_size(OcTree::TREE_DEPTH + 1);
    }

    #[test]
    #[should_panic(expected = "search: depth")]
    fn search_rejects_depth_above_tree_depth() {
        // Pre-fix this had no check at all: `TREE_DEPTH - depth` underflows
        // and panics with "attempt to subtract with overflow" instead --
        // Finding 2's whole point being that upstream's own out-of-range
        // behaviour is not a clean skip either, see `search`'s doc comment.
        // A fresh, never-updated tree has no root, and `search` returns
        // `None` via `?` before ever reaching the debug_assert -- a real
        // node must exist for this to reach the check at all.
        let mut tree = OcTree::new(0.1);
        let key = OcTree::root_key();
        tree.update_node_by_key(key, true, false);
        let _ = tree.search(key, OcTree::TREE_DEPTH + 1);
    }

    #[test]
    #[should_panic(expected = "key_to_coord_axis_at_depth: depth")]
    fn key_to_coord_axis_at_depth_rejects_depth_above_tree_depth() {
        // Same pre-fix gap as `search` above: `TREE_DEPTH - depth`
        // underflows to "attempt to subtract with overflow" instead of this
        // message.
        let tree = OcTree::new(0.1);
        let _ = tree
            .key_to_coord_axis_at_depth(OcTree::TREE_MAX_VAL as KeyType, OcTree::TREE_DEPTH + 1);
    }

    #[test]
    fn insert_ray_marks_the_endpoint_occupied_and_the_path_free() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, None, false));
        assert!(tree.is_occupied(end).unwrap());
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
    }

    #[test]
    fn insert_ray_cut_short_by_max_range_records_only_a_miss() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(10.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, Some(1.0), false));
        // The ray was cut short at max_range, so upstream never guesses at
        // occupancy for `end`: it stays unmapped, not marked free or occupied.
        assert!(tree.log_odds_at(end).is_none());
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
    }

    /// `insertRay`'s guard is `(maxrange > 0) && ...`, so every
    /// non-positive value is upstream's "unlimited" sentinel and inserts
    /// the complete ray. Without the `r > 0.0` half, `Some(-1.0)` cut the
    /// ray to `origin + direction * -1.0` -- backwards, away from `end`.
    #[test]
    fn insert_ray_treats_a_negative_max_range_as_unlimited() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, Some(-1.0), false));
        assert!(tree.is_occupied(end).unwrap());
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
        // Nothing was integrated on the backwards side of the origin.
        assert!(tree.log_odds_at(Point3::new(-0.5, 0.0, 0.0)).is_none());
    }

    /// `0.0` is not `> 0`, so it is the sentinel too -- the boundary the
    /// missing half sat on. `Some(0.0)` previously cut the ray to `origin`.
    #[test]
    fn insert_ray_treats_a_zero_max_range_as_unlimited() {
        let mut tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        assert!(tree.insert_ray(origin, end, Some(0.0), false));
        assert!(tree.is_occupied(end).unwrap());
        // And the ray was traced, not collapsed to a point: the midpoint is
        // marked free rather than occupied.
        assert!(!tree.is_occupied(Point3::new(0.5, 0.0, 0.0)).unwrap());
    }

    /// `computeUpdate`'s guard is `(maxrange < 0.0) || (norm <= maxrange)`,
    /// so a negative `max_range` takes the *unlimited* branch -- free cells
    /// along the whole ray and an occupied endpoint. Without the `r < 0.0`
    /// disjunct this fell to the cut branch, which records no endpoint at
    /// all and normalised the zero vector when `p == origin`.
    #[test]
    fn compute_update_treats_a_negative_max_range_as_unlimited() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[end], origin, Some(-1.0));
        let end_key = tree.coord_to_key_checked(end).unwrap();
        assert_eq!(occupied, [end_key].into_iter().collect::<KeySet>());
        let mid_key = tree
            .coord_to_key_checked(Point3::new(0.5, 0.0, 0.0))
            .unwrap();
        assert!(free.contains(&mid_key));
    }

    /// The complement of the case above, and the reason the two guards
    /// cannot be factored into one: `0.0` is *not* `< 0.0`, so upstream's
    /// `computeUpdate` cuts here where its `insertRay` does not.
    #[test]
    fn compute_update_still_cuts_at_a_zero_max_range() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let (_, occupied) = tree.compute_update(&[end], origin, Some(0.0));
        assert!(occupied.is_empty());
    }

    #[test]
    fn compute_update_keeps_occupied_and_free_cells_disjoint() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let hit = Point3::new(0.5, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[hit], origin, None);
        let hit_key = tree.coord_to_key_checked(hit).unwrap();
        assert!(occupied.contains(&hit_key));
        assert!(!free.contains(&hit_key));
        assert!(!free.is_empty());
    }

    /// Upstream `computeUpdate`'s within-range gate is `(maxrange < 0.0) ||
    /// (norm <= maxrange)` -- a negative `maxrange` means unlimited.
    /// `Some(-1.0)` must behave exactly like `None`: the hit lands in
    /// `occupied` at its real coordinate, not truncated toward a negative
    /// distance.
    #[test]
    fn compute_update_negative_max_range_behaves_as_unlimited() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let hit = Point3::new(0.5, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[hit], origin, Some(-1.0));
        let hit_key = tree.coord_to_key_checked(hit).unwrap();
        assert!(occupied.contains(&hit_key));
        assert!(!free.contains(&hit_key));
        assert!(!free.is_empty());
    }

    /// Demonstrated opposite of the two tests above: a genuinely positive
    /// `max_range` still cuts the ray short, so the far hit point is never
    /// marked occupied at all.
    #[test]
    fn compute_update_positive_max_range_still_cuts() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let far_hit = Point3::new(5.0, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[far_hit], origin, Some(1.0));
        assert!(!occupied.contains(&tree.coord_to_key_checked(far_hit).unwrap()));
        // A cell short of the cut point is marked free (the ray was traced
        // that far); a cell beyond `max_range` is not (the ray never
        // reaches it) -- `compute_ray_keys` itself excludes its own
        // endpoint's cell (see its `if current_key == key_end { break; }`
        // above), matching upstream, so the cut point's own cell is not
        // the right thing to assert on here.
        let short_of_cut = tree
            .coord_to_key_checked(Point3::new(0.5, 0.0, 0.0))
            .unwrap();
        let beyond_cut = tree
            .coord_to_key_checked(Point3::new(2.0, 0.0, 0.0))
            .unwrap();
        assert!(free.contains(&short_of_cut));
        assert!(!free.contains(&beyond_cut));
        assert!(!occupied.contains(&beyond_cut));
    }

    /// Regression test for the invariant `compute_update`'s `else` branch
    /// (`:1243`) relies on for its Distinct verdict: a `NaN` `max_range`
    /// slips past `within_range`'s `NAN < 0.0 || norm() <= NAN` disjunction
    /// (both false) regardless of `p`, so `p == origin` reaches the
    /// direction-normalize with a genuinely zero vector, and plain
    /// `.normalize()` there produces `(NaN, NaN, NaN)` -- but that `NaN`
    /// never reaches this function's return value, because
    /// `coord_to_key_checked_axis` independently rejects it while building
    /// `new_end`'s key. If a future change to `coord_to_key_checked_axis`
    /// (or a refactor that reads `direction`/`new_end` before that call)
    /// starts letting a `NaN` coordinate through, this must start failing
    /// or `:1243` needs `try_normalize`, not a comment.
    #[test]
    fn compute_update_nan_max_range_at_a_coincident_point_stays_empty_not_nan_poisoned() {
        let tree = OcTree::new(0.1);
        let origin = Point3::new(0.0, 0.0, 0.0);
        let (free, occupied) = tree.compute_update(&[origin], origin, Some(f64::NAN));
        assert!(free.is_empty());
        assert!(
            occupied.is_empty(),
            "origin == p never satisfies within_range with a NaN max_range"
        );
    }

    #[test]
    fn coord_to_key_checked_axis_accepts_the_last_value_inside_range() {
        let tree = OcTree::new(1.0);
        // resolution_factor == 1.0, so scaled_f == coord.floor(). The last
        // value for which scaled_f < TREE_MAX_VAL (32768) holds.
        assert_eq!(tree.coord_to_key_checked_axis(32767.5), Some(u16::MAX));
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_the_first_value_outside_range() {
        let tree = OcTree::new(1.0);
        // scaled_f == 32768.0 == TREE_MAX_VAL, which fails the `< max` half
        // of the bound (upstream's own coordToKeyChecked also rejects this
        // key, so this is the shared boundary, not the deviation).
        assert_eq!(tree.coord_to_key_checked_axis(32768.0), None);
    }

    #[test]
    fn coord_to_key_checked_axis_accepts_the_negative_range_boundary() {
        let tree = OcTree::new(1.0);
        // scaled_f == -32768.0 == -TREE_MAX_VAL, the `>= min` boundary.
        assert_eq!(tree.coord_to_key_checked_axis(-32768.0), Some(0));
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_just_past_the_negative_boundary() {
        let tree = OcTree::new(1.0);
        assert_eq!(tree.coord_to_key_checked_axis(-32768.5), None);
    }

    #[test]
    fn coord_to_key_checked_axis_accepts_the_tree_center() {
        let tree = OcTree::new(1.0);
        assert_eq!(tree.coord_to_key_checked_axis(0.0), Some(32768));
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_nan_instead_of_returning_the_center_key() {
        // The §172 same-defect case: a naive `as i64` cast turns NaN into
        // `0`, which after the `+ TREE_MAX_VAL` offset lands in-bounds as
        // key 32768 -- the tree's exact center -- silently, with no error.
        let tree = OcTree::new(0.1);
        assert_eq!(tree.coord_to_key_checked_axis(f64::NAN), None);
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_positive_infinity_without_overflowing() {
        // The other §172 same-defect case: `f64::INFINITY as i64` saturates
        // to `i64::MAX`, and the following `+ TREE_MAX_VAL` then overflows
        // and panics under overflow checks. Must be rejected before either
        // cast runs.
        let tree = OcTree::new(0.1);
        assert_eq!(tree.coord_to_key_checked_axis(f64::INFINITY), None);
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_negative_infinity() {
        let tree = OcTree::new(0.1);
        assert_eq!(tree.coord_to_key_checked_axis(f64::NEG_INFINITY), None);
    }

    #[test]
    fn coord_to_key_checked_axis_rejects_a_huge_finite_coordinate_without_overflowing() {
        // Finite but large enough that `resolution_factor * coord` is far
        // outside `i64`'s range once narrowed -- must be caught by the
        // `f64`-space range check, not by narrowing first and hoping the
        // subsequent `+ TREE_MAX_VAL` doesn't overflow.
        let tree = OcTree::new(0.1);
        assert_eq!(tree.coord_to_key_checked_axis(1e300), None);
        assert_eq!(tree.coord_to_key_checked_axis(-1e300), None);
    }

    #[test]
    fn leaves_yields_every_leaf_with_its_coordinate_and_value() {
        let mut tree = OcTree::new(1.0);
        let hit_point = Point3::new(10.5, 0.5, 0.5);
        let miss_point = Point3::new(-10.5, 0.5, 0.5);
        tree.update_node(hit_point, true, false);
        tree.update_node(miss_point, false, false);

        let leaves: Vec<_> = tree.leaves().collect();
        assert_eq!(leaves.len(), 2);

        let mut saw_hit = false;
        let mut saw_miss = false;
        for leaf in &leaves {
            if (leaf.coordinate() - hit_point).norm() < 1e-9 {
                assert_eq!(leaf.log_odds(), tree.prob_hit_log());
                saw_hit = true;
            }
            if (leaf.coordinate() - miss_point).norm() < 1e-9 {
                assert_eq!(leaf.log_odds(), tree.prob_miss_log());
                saw_miss = true;
            }
        }
        assert!(saw_hit && saw_miss);
    }

    #[test]
    fn leaves_in_bbx_only_yields_leaves_overlapping_the_box() {
        let mut tree = OcTree::new(1.0);
        let inside = Point3::new(0.5, 0.5, 0.5);
        let outside = Point3::new(50.5, 0.5, 0.5);
        tree.update_node(inside, true, false);
        tree.update_node(outside, true, false);

        let leaves: Vec<_> = tree
            .leaves_in_bbx(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0))
            .expect("bbx corners are in range")
            .collect();
        assert_eq!(leaves.len(), 1);
        assert!((leaves[0].coordinate() - inside).norm() < 1e-9);
    }

    /// Distinct from `leaves_in_bbx_returns_none_for_an_out_of_range_min`
    /// below: `LeavesInBbx::new` checks `min` then `max`, each with its own
    /// `?`, so this fixture (`min` in range) exercises only the `max` guard.
    #[test]
    fn leaves_in_bbx_returns_none_for_an_out_of_range_max() {
        let tree = OcTree::new(0.1);
        assert!(
            tree.leaves_in_bbx(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0e9, 0.0, 0.0))
                .is_none()
        );
    }

    /// Distinct from `leaves_in_bbx_returns_none_for_an_out_of_range_max`
    /// above: see that test's doc comment. Before this test existed the
    /// `min` guard had no coverage at all -- neutralizing it left all 66
    /// tests green. Now each guard isolates to its own test: neutralizing
    /// `min` fails only this one, neutralizing `max` only the sibling.
    #[test]
    fn leaves_in_bbx_returns_none_for_an_out_of_range_min() {
        let tree = OcTree::new(0.1);
        assert!(
            tree.leaves_in_bbx(Point3::new(1.0e9, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0))
                .is_none()
        );
    }

    // Round 15 ported set_prob_hit/set_prob_miss's upstream `assert(prob_hit_log
    // >= 0.0)`/`assert(prob_miss_log <= 0.0)` as `debug_assert!`, but nothing
    // called either with an out-of-range probability -- removing both
    // debug_assert!s left nextest's debug-profile run at 27/27, unchanged.
    // These two exercise the boundary that assertion exists to catch.

    #[test]
    #[should_panic]
    fn set_prob_hit_below_half_panics_in_debug() {
        let mut tree = OcTree::new(0.1);
        tree.set_prob_hit(0.3);
    }

    #[test]
    #[should_panic]
    fn set_prob_miss_above_half_panics_in_debug() {
        let mut tree = OcTree::new(0.1);
        tree.set_prob_miss(0.7);
    }

    // `read_binary_data`/`read_data`, round 33 item 1. Cases are picked at
    // invariant boundaries (empty input, one byte short, the deepest
    // decodable tree, one level past it), not narrative scenarios -- see
    // `tests/decode_parity.rs` for the oracle-backed structural/leaf-value
    // parity check these unit tests don't attempt.

    #[test]
    fn read_binary_data_rejects_a_tree_that_already_has_a_root() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.05, 0.05, 0.05), true, false);
        assert_eq!(
            tree.read_binary_data(&[]),
            Err(DecodeError::TreeAlreadyPopulated)
        );
    }

    #[test]
    fn read_data_rejects_a_tree_that_already_has_a_root() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.05, 0.05, 0.05), true, false);
        assert_eq!(tree.read_data(&[]), Err(DecodeError::TreeAlreadyPopulated));
    }

    // `DecodeError::InvalidResolution`: an untrusted-wire resolution never
    // fails loudly on its own (`update_node` no-ops via the already-guarded
    // `coord_to_key_checked` direction), but decode never touches
    // `resolution` and would silently populate a tree whose leaves
    // `key_to_coord_axis` then collapses to the world origin -- see the
    // variant's own doc for the full measured chain. One boundary each side
    // of the valid range, plus the two non-finite values a wire `f64` can
    // carry.
    #[test]
    fn read_binary_data_rejects_zero_resolution() {
        let mut tree = OcTree::new(0.0);
        assert_eq!(
            tree.read_binary_data(&[0x02, 0x00]),
            Err(DecodeError::InvalidResolution)
        );
    }

    #[test]
    fn read_binary_data_rejects_negative_resolution() {
        let mut tree = OcTree::new(-0.1);
        assert_eq!(
            tree.read_binary_data(&[0x02, 0x00]),
            Err(DecodeError::InvalidResolution)
        );
    }

    #[test]
    fn read_binary_data_rejects_nan_and_infinite_resolution() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut tree = OcTree::new(bad);
            assert_eq!(
                tree.read_binary_data(&[0x02, 0x00]),
                Err(DecodeError::InvalidResolution),
                "resolution {bad}"
            );
        }
    }

    #[test]
    fn read_data_rejects_zero_resolution() {
        let mut tree = OcTree::new(0.0);
        assert_eq!(
            tree.read_data(&[0, 0, 0, 63, 0]),
            Err(DecodeError::InvalidResolution)
        );
    }

    #[test]
    fn read_binary_data_empty_input_is_unexpected_eof() {
        let mut tree = OcTree::new(0.1);
        assert_eq!(tree.read_binary_data(&[]), Err(DecodeError::UnexpectedEof));
    }

    #[test]
    fn read_data_empty_input_is_unexpected_eof() {
        let mut tree = OcTree::new(0.1);
        assert_eq!(tree.read_data(&[]), Err(DecodeError::UnexpectedEof));
    }

    #[test]
    fn read_binary_data_one_byte_short_of_the_root_record_is_unexpected_eof() {
        // A root's own 2-byte record needs both bytes; one byte is a
        // truncated stream, not a valid empty-children record.
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_binary_data(&[0x00]),
            Err(DecodeError::UnexpectedEof)
        );
    }

    #[test]
    fn read_data_truncated_mid_log_odds_is_unexpected_eof() {
        // A root's own record starts with a 4-byte f32; 3 bytes is short.
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_data(&[0x00, 0x00, 0x00]),
            Err(DecodeError::UnexpectedEof)
        );
    }

    #[test]
    fn read_data_truncated_before_the_children_byte_is_unexpected_eof() {
        // A complete 4-byte f32 but no trailing children-bitset byte.
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_data(&1.0f32.to_le_bytes()),
            Err(DecodeError::UnexpectedEof)
        );
    }

    #[test]
    fn read_binary_data_garbage_input_is_not_a_panic() {
        // Not a real octree encoding, just bytes -- must return an error,
        // never panic, regardless of what those bytes happen to spell. For
        // this exact 3-byte input, tracing `read_binary_node` shows only one
        // reachable `DecodeError`: two `read_u8` calls succeed (the root's
        // own `child1to4`/`child5to8` bytes, both `0xff`, decoding as "every
        // child has children"), then the recursion into child 0 reads its
        // `child1to4` byte from the third and last input byte and fails
        // reading `child5to8` with `UnexpectedEof`. `TreeAlreadyPopulated` is
        // excluded (fresh tree) and `MaxDepthExceeded` is unreachable (3
        // bytes cannot recurse to depth 16).
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_binary_data(&[0xff; 3]),
            Err(DecodeError::UnexpectedEof)
        );
    }

    #[test]
    fn read_data_garbage_input_is_not_a_panic() {
        // Same reasoning as `read_binary_data_garbage_input_is_not_a_panic`:
        // `read_data_node`'s first read, `cursor.read_f32_le()?`, needs 4
        // bytes and this input has 3, so `UnexpectedEof` fires immediately,
        // before any other `DecodeError` site is reachable.
        let mut tree = OcTree::new(0.1);
        assert_eq!(tree.read_data(&[0xff; 3]), Err(DecodeError::UnexpectedEof));
    }

    #[test]
    fn read_binary_data_decodes_a_single_occupied_leaf() {
        // Root's own record: child 0 is an occupied leaf (low=0, high=1 at
        // bits 0,1 of child1to4 -> 0b10 = 0x02), every other child absent.
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&[0x02, 0x00]).unwrap();
        assert_eq!(tree.num_nodes(), 2); // root + the one leaf
        let leaf = tree.leaves().next().unwrap();
        assert_eq!(leaf.log_odds(), tree.clamping_thres_max_log());
        assert!(leaf.is_occupied());
        // Root's own log-odds is unconditionally the "default to occupied"
        // clamp value upstream sets before it ever inspects a child --
        // never corrected for the outermost root (only a recursed-into
        // child is), see `read_binary_node`'s doc comment. `root()` is
        // `pub(crate)`, reachable here since this test lives inside
        // `tree.rs` itself.
        assert_eq!(tree.root().unwrap().log_odds, tree.clamping_thres_max_log());
    }

    #[test]
    fn read_binary_data_decodes_a_single_free_leaf() {
        // child 0 is a free leaf: low=1, high=0 -> 0b01 = 0x01.
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&[0x01, 0x00]).unwrap();
        assert_eq!(tree.num_nodes(), 2);
        let leaf = tree.leaves().next().unwrap();
        assert_eq!(leaf.log_odds(), tree.clamping_thres_min_log());
        assert!(!leaf.is_occupied());
    }

    #[test]
    fn read_binary_data_ignores_trailing_bytes_after_a_complete_decode() {
        // Same 2-byte complete record as the test above, plus 3 bytes that
        // are not part of any node -- matching upstream's own `readTree`,
        // which never tells `readBinaryData` how many bytes to expect. See
        // this function's own doc for the citation.
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&[0x01, 0x00, 0xff, 0xff, 0xff])
            .unwrap();
        assert_eq!(tree.num_nodes(), 2);
    }

    #[test]
    fn write_binary_data_of_an_empty_tree_is_empty() {
        let tree = OcTree::new(0.1);
        assert_eq!(tree.write_binary_data(), Vec::<u8>::new());
    }

    #[test]
    fn write_binary_data_is_the_exact_inverse_of_read_binary_data_for_a_single_free_leaf() {
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&[0x01, 0x00]).unwrap();
        assert_eq!(tree.write_binary_data(), vec![0x01, 0x00]);
    }

    #[test]
    fn write_binary_data_is_the_exact_inverse_of_read_binary_data_for_all_eight_children() {
        // Every one of the root's 8 children a free leaf: each 2-bit code
        // is `10` (low=1, high=0), four pairs per byte -> 0b01010101 = 0x55.
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&[0x55, 0x55]).unwrap();
        assert_eq!(tree.write_binary_data(), vec![0x55, 0x55]);
    }

    #[test]
    fn write_binary_data_re_encodes_a_nested_has_children_record() {
        // Root: child 0 has children (`11` = 0x03 in the low byte). Child
        // 0's own record: child 0 is an occupied leaf (`01` = 0x01).
        let bytes = vec![0x03, 0x00, 0x01, 0x00];
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&bytes).unwrap();
        assert_eq!(tree.write_binary_data(), bytes);
    }

    #[test]
    fn read_data_round_trips_a_two_node_chain_s_exact_log_odds() {
        // Root's record: an arbitrary (non-clamp) log-odds, one child (bit 0
        // of the children byte) -- then the child's own record: a
        // different arbitrary log-odds, no children.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&1.25f32.to_le_bytes());
        bytes.push(0x01); // child 0 exists
        bytes.extend_from_slice(&(-3.5f32).to_le_bytes());
        bytes.push(0x00); // leaf, no children
        let mut tree = OcTree::new(0.1);
        tree.read_data(&bytes).unwrap();
        assert_eq!(tree.num_nodes(), 2);
        // Full-format log-odds survive exactly -- not quantized to a clamp
        // the way `read_binary_data`'s leaves are, so this is the one case
        // where an arbitrary, non-clamp value is meaningful ground truth.
        let leaf = tree.leaves().next().unwrap();
        assert_eq!(leaf.log_odds(), -3.5);
    }

    #[test]
    fn write_data_of_an_empty_tree_is_empty() {
        let tree = OcTree::new(0.1);
        assert_eq!(tree.write_data(), Vec::<u8>::new());
    }

    #[test]
    fn write_data_is_the_exact_inverse_of_read_data_for_a_two_node_chain() {
        // Same bytes as `read_data_round_trips_a_two_node_chain_s_exact_log_odds`
        // above: this format has no lossy quantization on either side, so
        // the exact f32 bit pattern (not just the value) must survive the
        // decode-then-re-encode round trip.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&1.25f32.to_le_bytes());
        bytes.push(0x01); // child 0 exists
        bytes.extend_from_slice(&(-3.5f32).to_le_bytes());
        bytes.push(0x00); // leaf, no children
        let mut tree = OcTree::new(0.1);
        tree.read_data(&bytes).unwrap();
        assert_eq!(tree.write_data(), bytes);
    }

    #[test]
    fn read_data_ignores_trailing_bytes_after_a_complete_decode() {
        // A minimal complete record: one f32 log-odds, one children byte
        // (0x00 = no children), then bytes that are not part of any node --
        // matching upstream's own `fullMsgToMap`, which never tells
        // `readData` how many bytes to expect. See this function's own doc
        // for the citation.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&1.25f32.to_le_bytes());
        bytes.push(0x00);
        bytes.extend_from_slice(&[0xff; 32]);
        let mut tree = OcTree::new(0.1);
        tree.read_data(&bytes).unwrap();
        assert_eq!(tree.num_nodes(), 1);
    }

    /// `has_children_levels` pairs of (child 0 has children), followed by
    /// one terminal pair (child 0 is an occupied leaf). Not an oracle
    /// fixture -- a hand-built chain exercising exactly the recursion-depth
    /// boundary [`DecodeError::MaxDepthExceeded`] exists for.
    fn binary_child0_chain(has_children_levels: u32) -> Vec<u8> {
        let mut bytes = Vec::new();
        for _ in 0..has_children_levels {
            bytes.extend_from_slice(&[0x03, 0x00]);
        }
        bytes.extend_from_slice(&[0x02, 0x00]);
        bytes
    }

    #[test]
    fn read_binary_data_accepts_the_deepest_representable_chain() {
        // 15 "has children" levels (depths 0..14) plus the terminal record
        // at depth 15 describing a real depth-16 leaf -- the deepest chain
        // this format can represent under a 16-bit key.
        let mut tree = OcTree::new(0.1);
        tree.read_binary_data(&binary_child0_chain(15)).unwrap();
        assert_eq!(tree.num_nodes(), 17); // 16 recorded nodes + 1 leaf
    }

    #[test]
    fn read_binary_data_rejects_one_level_past_the_deepest_chain() {
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_binary_data(&binary_child0_chain(16)),
            Err(DecodeError::MaxDepthExceeded)
        );
    }

    /// `recurse_levels` records with "child 0 exists", followed by one
    /// terminal record with no children. Same purpose as
    /// [`binary_child0_chain`] for the full-format decoder.
    fn data_child0_chain(recurse_levels: u32) -> Vec<u8> {
        let mut bytes = Vec::new();
        for _ in 0..recurse_levels {
            bytes.extend_from_slice(&0.0f32.to_le_bytes());
            bytes.push(0x01);
        }
        bytes.extend_from_slice(&0.0f32.to_le_bytes());
        bytes.push(0x00);
        bytes
    }

    #[test]
    fn read_data_accepts_the_deepest_representable_chain() {
        // 16 records claiming a child (depths 0..15) plus the terminal
        // record at depth 16 itself -- unlike the binary format, every
        // depth including 16 gets its own record here.
        let mut tree = OcTree::new(0.1);
        tree.read_data(&data_child0_chain(16)).unwrap();
        assert_eq!(tree.num_nodes(), 17);
    }

    #[test]
    fn read_data_rejects_one_level_past_the_deepest_chain() {
        let mut tree = OcTree::new(0.1);
        assert_eq!(
            tree.read_data(&data_child0_chain(17)),
            Err(DecodeError::MaxDepthExceeded)
        );
    }
}
