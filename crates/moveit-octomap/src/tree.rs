// Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from octomap 1.9.7 (see key.rs's provenance comment for how the
// version was matched):
//   include/octomap/OcTreeBaseImpl.h, OcTreeBaseImpl.hxx
//   include/octomap/OccupancyOcTreeBase.h, OccupancyOcTreeBase.hxx
//   include/octomap/AbstractOccupancyOcTree.h
//   include/octomap/octomap_utils.h (logodds / probability)
//   include/octomap/OcTree.h (the concrete, non-template `OcTree`; its
//     sensor-model defaults live in OcTree.cpp, compiled into
//     liboctomap.so.1.9 and not shipped as source -- see the `DEFAULT_*`
//     constants below for how those five numbers were confirmed rather than
//     guessed)

use nalgebra::{Point3, Vector3};

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
///   below -- octrees enter this workspace only via ROS messages, itself
///   D1-excluded).
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
///   bool)` -- distinct, D1 (every caller is a `moveit_ros/perception`
///   depth-camera updater converting a ROS `sensor_msgs` cloud into
///   `octomap::Pointcloud` first).
/// - `insertPointCloud(const Pointcloud&, const point3d&, const pose6d&,
///   double, bool, bool)` -- distinct, same D1 reasoning.
/// - `insertPointCloud(const ScanNode&, double, bool, bool)` -- distinct,
///   same D1 reasoning.
/// - `insertPointCloudRays(const Pointcloud&, const point3d&, double,
///   bool)` -- distinct, same D1 reasoning.
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
///   alongside the already-D1-excluded BBX-limited `insertPointCloud`
///   path.
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
/// - `readBinaryData(std::istream&)` -- distinct, binary stream IO
///   (octrees enter this workspace only via ROS messages, never
///   `.bt`/`.ot` files).
/// - `readBinaryNode(std::istream&, NODE*)` -- distinct, same IO
///   reasoning.
/// - `writeBinaryNode(std::ostream&, const NODE*) const` -- distinct, same
///   IO reasoning.
/// - `writeBinaryData(std::ostream&) const` -- distinct, same IO
///   reasoning.
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
/// - `readData(std::istream&)` -- distinct, binary stream IO, same
///   reasoning as `readBinaryData` above.
/// - `writeData(std::ostream&) const` -- distinct, same IO reasoning.
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
/// ```text
/// ported                55
/// unported, in scope     8
/// distinct               88
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
    pub fn node_size(&self, depth: u32) -> f64 {
        assert!(depth <= Self::TREE_DEPTH);
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
    fn key_to_coord_axis_at_depth(&self, key: KeyType, depth: u32) -> f64 {
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
    fn coord_to_key_checked_axis(&self, coord: f64) -> Option<KeyType> {
        let scaled =
            (self.resolution_factor * coord).floor() as i64 + i64::from(Self::TREE_MAX_VAL);
        if scaled >= 0 && scaled < 2 * i64::from(Self::TREE_MAX_VAL) {
            Some(scaled as KeyType)
        } else {
            None
        }
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
    pub(crate) fn search(&self, key: OcTreeKey, depth: u32) -> Option<&Node> {
        let mut cur = self.root.as_deref()?;
        let depth = if depth == 0 { Self::TREE_DEPTH } else { depth };
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
    pub fn compute_update(
        &self,
        points: &[Point3<f64>],
        origin: Point3<f64>,
        max_range: Option<f64>,
    ) -> (KeySet, KeySet) {
        let mut free_cells = KeySet::new();
        let mut occupied_cells = KeySet::new();

        for &p in points {
            let within_range = max_range.is_none_or(|r| (p - origin).norm() <= r);
            if within_range {
                if let Some(ray) = self.compute_ray_keys(origin, p) {
                    free_cells.extend(ray);
                }
                if let Some(key) = self.coord_to_key_checked(p) {
                    occupied_cells.insert(key);
                }
            } else if let Some(r) = max_range {
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
    pub fn insert_ray(
        &mut self,
        origin: Point3<f64>,
        end: Point3<f64>,
        max_range: Option<f64>,
        lazy_eval: bool,
    ) -> bool {
        if let Some(r) = max_range
            && (end - origin).norm() > r
        {
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
        node.log_odds =
            (node.log_odds + params.log_odds_update).clamp(params.clamp_min, params.clamp_max);
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

    #[test]
    fn leaves_in_bbx_returns_none_for_an_out_of_range_corner() {
        let tree = OcTree::new(0.1);
        assert!(
            tree.leaves_in_bbx(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0e9, 0.0, 0.0))
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
}
