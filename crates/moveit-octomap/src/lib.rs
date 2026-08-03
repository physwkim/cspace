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
//!   scope at all** -- that is `moveit-geometry`'s `shapes::OcTree` variant
//!   (already stubbed, deliberately deferred to Phase 3/5 collision) and is
//!   the future consumer of this crate's [`OcTree`], not something this
//!   crate implements itself.
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

pub use iter::{Leaf, Leaves, LeavesInBbx};
pub use key::{KeyRay, KeySet, KeyType, OcTreeKey};
pub use tree::OcTree;
