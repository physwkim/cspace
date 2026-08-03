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
//! # Symbol-by-symbol audit against upstream's public surface (round 12)
//!
//! This crate has had rounds of parity/oracle work but never a pass over
//! upstream `octomap`'s actual header surface, symbol by symbol, the way
//! `moveit-geometry`'s `shapes.rs`/`bodies.rs` module docs already do for
//! their own crate. Done here against the headers extracted from the oracle
//! container's `liboctomap-dev 1.9.7+dfsg-3.1build3`
//! (`AbstractOcTree.h`, `AbstractOccupancyOcTree.h`, `OcTree.h`,
//! `OcTreeNode.h`, `OcTreeDataNode.h`, `OccupancyOcTreeBase.h`,
//! `OcTreeBaseImpl.h`), cross-checked against every call site in
//! `/home/stevek/work/moveit2/moveit_core` (not `moveit_ros`, which is
//! `moveit-ros` territory and D1-excluded by the same reasoning applied
//! throughout this project -- PORTING-PLAN.md's own D1 note on the
//! `resolveConstraintFrames` TF fallback is the same pattern). 40 symbol
//! groups (overloads collapsed to one row per upstream operation), four-way
//! classified:
//!
//! ```text
//! ported                22
//! unported, in scope      3
//! distinct because ...   15
//! ------------------------
//! total                  40
//! ```
//!
//! **`ported`** (22): the constructor and logodds-space sensor-model
//! getters; `AbstractOccupancyOcTree::isNodeOccupied`-equivalent queries;
//! `getNodeSize`; `search` (fused into [`OcTree::log_odds_at`]/
//! [`OcTree::occupancy_at`]/[`OcTree::is_occupied`] rather than a raw
//! nullable `NODE*` -- a deliberate shape deviation, not a gap);
//! `coordToKeyChecked`'s point overload; the three `updateNode` overload
//! shapes this workspace's sensor-model callers actually use (key+logodds,
//! key+bool, point+bool); the protected relative `updateNodeLogOdds`;
//! `updateInnerOccupancy`; tree-wide `prune`; `computeRayKeys`;
//! `computeUpdate` (point-slice-shaped here, not `octomap::Pointcloud`-typed
//! -- see "What was deliberately not ported" above for why); `insertRay`;
//! leaf iteration (`begin_leafs`/`end_leafs`, `begin_leafs_bbx`/
//! `end_leafs_bbx`); `calcNumNodes`; `getNumLeafNodes`; `OcTreeKey`/
//! `KeyRay`/`KeySet` plus the bit-level `computeChildKey`/`computeChildIdx`/
//! `computeIndexKey`; `OcTreeNode`'s occupancy accessors and
//! `updateOccupancyChildren`; `addValue` (inlined into the log-odds clamp in
//! `update_node_recurs` rather than kept as a named method); and the
//! `OcTreeDataNode`-level child-structural primitives (`createChild`,
//! `expandNode`, `isNodeCollapsible`, `pruneNode`), present as `Node`'s
//! `pub(crate)` methods since nothing outside this crate needs direct child
//! manipulation.
//!
//! **`unported, in scope`** (3) -- no current consumer, but not
//! architecturally excluded either:
//!
//! - **`tree_iterator` (`begin_tree`/`end_tree`, all nodes including
//!   inner ones, as opposed to `leaf_iterator`'s leaves-only walk).**
//!   Checking this round found a concrete, named reason it matters:
//!   `collision_distance_field_types.cpp:355-364`'s
//!   `PosedBodyPointDecomposition(const std::shared_ptr<const
//!   octomap::OcTree>& octree)` constructor -- the one overload of that type
//!   that builds collision points directly from an octree rather than from a
//!   `BodyDecomposition` -- is implemented entirely in terms of
//!   `octree->begin_tree()`/`end_tree()`. `moveit-distance-field`'s own
//!   `PosedBodyPointDecomposition` (`collision_distance_field_types.rs:929`)
//!   ports only the `BodyDecomposition`-sourced constructors
//!   (`PosedBodyPointDecomposition::with_pose` and its plain form) -- the
//!   octree-sourced overload is itself unported there. So this is not a
//!   speculative gap: it is one specific upstream call site whose Rust-side
//!   consumer is *also* missing, which is PORTING-PLAN.md §13's own "still
//!   open" risk (standalone octree exists, but is not yet wired into a real
//!   collision/distance-field path) in a more concrete form than it had
//!   before this round.
//! - **`setNodeValue`** (3 overloads: key, point, xyz). Already named in
//!   PORTING-PLAN.md §13; still accurate -- `tree.rs`'s own module doc
//!   explains the one upstream call site that conceptually wants it
//!   (`lazy_free_space_updater.cpp`) routes through the relative
//!   `update_node_log_odds` primitive with a saturating delta instead.
//! - **The remaining `updateNode` overload shapes** (point+logodds, and
//!   every `(double x, double y, double z, ...)` triple-argument form for
//!   both logodds and bool) -- pure ergonomic wrappers around the three
//!   shapes already ported, with no caller needing the wider signature set.
//!
//! **`distinct because ...`** (15) -- architecturally out of this
//! workspace's scope, not merely uncalled so far: the `AbstractOcTree`
//! registry/factory (`create`/`getTreeType`/`createTree`/
//! `registerTreeType`/`StaticMemberInitializer`, D4 rules out runtime-type
//! polymorphism for a crate with exactly one concrete tree type);
//! `ColorOcTree`/`CountingOcTree`/`OcTreeStamped` and their node types (zero
//! `moveit_core` reference); binary file/stream IO (`writeBinary*`/
//! `readBinary*`/`AbstractOcTree::write`/`read`/`readHeader` -- octrees
//! enter this workspace only via ROS messages in `moveit_ros/perception`,
//! itself D1-excluded, never via `.bt`/`.ot` files); change detection
//! (`enableChangeDetection` and its accessors, confirmed unused since the
//! crate's original round); `insertPointCloud`/`insertPointCloudRays`/
//! `computeDiscreteUpdate` (every caller is a `moveit_ros/perception`
//! depth-camera updater converting a ROS `sensor_msgs` cloud into
//! `octomap::Pointcloud` first -- D1-excluded by the same "moveit-ros
//! territory" reasoning PORTING-PLAN.md already applies elsewhere, even
//! though `octomap::Pointcloud` itself is not a ROS type); `castRay`/
//! `getRayIntersection`/`getNormals` (this workspace's octree collision
//! path is the leaf-`Cuboid` `Compound` approximation, PORTING-PLAN.md
//! §4.8's decision, not octomap's own raycasting); the BBX-limit machinery
//! (`useBBXLimit` and its accessors -- only meaningful alongside the
//! already-excluded BBX-limited `insertPointCloud` path); introspection/
//! debugging helpers with zero `moveit_core` consumer
//! (`getUnknownLeafCenters`, `computeRay`, `volume`, `memoryUsage`,
//! `memoryUsageNode`, `memoryFullGrid`, `setResolution`); `swapContent`/
//! `operator==`/the deep-copy constructor (this workspace's octrees are
//! shared via `Arc`, never compared or deep-cloned); the tree-level
//! structural-editing surface (`createNodeChild`/`deleteNodeChild`/
//! `getNodeChild`/`isNodeCollapsible`/`nodeChildExists`/`nodeHasChildren`/
//! `deleteNode`/`getRoot`, already covered internally where `update_node`/
//! `prune` actually need child access, via `Node`'s own `pub(crate)`
//! methods); `getMeanChildLogOdds` (upstream itself only wires the *max*
//! variant into `updateOccupancyChildren`); the deprecated-upstream
//! `childExists`/`hasChildren`; `toMaxLikelihood`/`integrateHit`/
//! `integrateMiss`/`nodeToMaxLikelihood` as separately named methods (this
//! port inlines the sensor-model math directly into
//! `update_node_log_odds`/`update_node_recurs` instead); the
//! probability-space (non-log) sensor-model *getters*
//! (`getProbHit`/`getProbMiss`/`getOccupancyThres`/`getClampingThresMin`/
//! `getClampingThresMax` -- their one `moveit_core` caller is the Bullet
//! collision backend, which this project does not use, `parry` replacing
//! FCL/Bullet per PORTING-PLAN.md; the *setters* are ported --
//! [`OcTree::set_prob_hit`], [`OcTree::set_prob_miss`],
//! [`OcTree::set_occupancy_thres`], [`OcTree::set_clamping_thres_min`],
//! [`OcTree::set_clamping_thres_max`] -- since they mutate state this port
//! already exposes as configurable, round 15 item 1); and `getMetricMin`/
//! `getMetricMax`/`getMetricSize` (zero consumer anywhere in
//! `moveit_core`, including the perception layer).
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

mod iter;
mod key;
mod node;
mod tree;

pub use iter::{Leaf, Leaves, LeavesInBbx, TreeNode, TreeNodes};
pub use key::{KeyRay, KeySet, KeyType, OcTreeKey};
pub use tree::OcTree;
