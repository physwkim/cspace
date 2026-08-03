// Copyright (c) 2013, Ioan A. Sucan
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
//! cargo nextest run -p moveit-geometry --no-fail-fast   # 141 tests run: 141 passed, 0 skipped
//! rg -c '#\[test\]' crates/moveit-geometry/tests/*.rs
//! ```
//!
//! 141 total; **18** of those are oracle- or shipped-`.so`-backed integration
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
//! remaining 123 are `#[cfg(test)]` unit tests inside `src/`, most of them
//! per-invariant-boundary (zero/negative dimensions, degenerate shapes,
//! masking-proof bisected tolerances -- see `shapes.rs`'s and `bodies.rs`'s
//! own provenance comments for that history) rather than one test per
//! upstream method.

pub mod bodies;
mod octree_collision;
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
