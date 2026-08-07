// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/
//   moveit_core/collision_detection/include/moveit/collision_detection/world_diff.hpp
//   moveit_core/collision_detection/src/world_diff.cpp

//! `PlanningScene` for moveit-rs: the world, attached bodies, the allowed
//! collision matrix, and the current [`moveit_state::RobotState`], plus the
//! parent/child *diff scene* relationship layered on top of all four.
//!
//! # Scope
//!
//! [`WorldDiff`] (a change record over a [`moveit_collision::World`]) and
//! [`PlanningScene`] (the world/ACM/attached-bodies/current-state bundle,
//! plus [`PlanningScene::diff`]/[`PlanningScene::push_diffs`]/
//! [`PlanningScene::decouple_parent`]/[`PlanningScene::clear_diffs`], and
//! state/path validity built on `moveit_constraints`'s
//! [`moveit_constraints::KinematicConstraintSet`] — see
//! [`PlanningScene::is_state_valid`]). See [`PlanningScene`]'s own doc for
//! the full scope audit, and the "Completion statement" below for the one
//! remaining blocked symbol (`getCostSources`) and what D1 permanently
//! excludes: message round-tripping. Object colors/types looked D1-shaped
//! (their value types, `std_msgs::msg::ColorRGBA`/
//! `object_recognition_msgs::msg::ObjectType`, live in a ROS message
//! namespace) but were not — [`ObjectColor`]/[`ObjectType`] port the
//! values message-free, the same way [`PlanningScene::OCTOMAP_NS`]/the
//! private `DEFAULT_SCENE_NAME` constant turned out not to need a message
//! either; see [`PlanningScene`]'s own doc, "Object colors and types," for
//! the message-free upstream callers that gave it away.
//!
//! See [`PlanningScene`]'s doc for the parent/child design — deliberately
//! reasoned through rather than transcribed from upstream's
//! `std::optional`-plus-const-pointer shape.
//!
//! # Completion statement
//!
//! Every number below is a command someone can re-run, not a claim to
//! trust — this section exists so the crate and this doc cannot silently
//! drift the way `PORTING-PLAN.md` §65 caught happening to another crate.
//!
//! **Headers audited.** `moveit_core/planning_scene/planning_scene.hpp`
//! (`planning_scene.h` is a deprecated `#pragma message`-only shim with no
//! symbols of its own — see [`PlanningScene`]'s own `# Scope` doc, which
//! opens with that check) has a full symbol-by-symbol walk, one line per
//! public symbol, landing in `ported` / `D1 excludes it` / `distinct` /
//! `blocked`:
//!
//! ```text
//! rg -c '^/// - `' crates/moveit-scene/src/scene.rs
//! ```
//!
//! is **60** bullets — the only such bullets anywhere in that file, so no
//! line-range restriction is needed to isolate the audit block from
//! anything else (re-verified this round: a prior version of this count,
//! 59, had drifted after the audit block grew and shifted past the
//! line-range this comment used to cite). Every one of the 60 classifies as
//! `ported as` (28), `D1` (21), `distinct` (10), or `blocked` (1), a sum of
//! 60 with nothing left over, so [`PlanningScene`]'s doc statement that the
//! walk found zero `unported, in scope` gaps among them still holds.
//! `moveit_core/collision_detection/{include/moveit/collision_detection/,src/}world_diff.{hpp,cpp}`:
//! [`WorldDiff`] ports every public member (`setWorld`/`reset`/
//! `getChanges`/`size`/`find`/`set`/`clearChanges`) except the
//! observer-subscribing constructors and `notify`, which [`WorldDiff`]'s
//! own module doc explains — `moveit_collision::World` has no observer
//! mechanism at all; every mutator returns the
//! [`moveit_collision::Notification`] it produced instead of pushing it
//! through a callback, so there is nothing for a constructor to subscribe
//! to.
//!
//! **What remains, precisely — one symbol.** `getCostSources` (all 4
//! overloads) is *blocked*, not merely deferred: every Rust-side type it
//! needs already exists in `moveit-collision`
//! ([`moveit_collision::CostSource`],
//! [`moveit_collision::CollisionResult::cost_sources`]), but
//! `moveit_collision::ParryCollisionEnv`'s collision callback hardcodes
//! `cost_sources: None`. [`PlanningScene`]'s own doc, "Cost sources and
//! diagnostics", traces exactly what would have to land in
//! `moveit-collision` to close it — split into a non-mesh fill-in (data
//! already in scope at the call site) and a mesh-pair BVH-leaf-pair
//! traversal (the `parry3d-f64` calls it needs, and the FCL-traversal-order
//! question still open once it is written) — and this crate has nothing
//! further of its own to do until that lands. Every other symbol in the
//! 59-bullet audit is ported, D1-excluded, or a documented "distinct"
//! design deviation; there is no second open item.
//!
//! **Fixtures, and what each checks against the real oracle.**
//!
//! ```text
//! rg -n '^fn .*matches_the_oracle' crates/moveit-scene/tests/*.rs
//! ```
//!
//! finds exactly **3**:
//! - `frame_transform_parity::panda_frame_transform_matches_the_oracle` —
//!   [`PlanningScene::frame_transform`]'s full tier order (upstream
//!   `getFrameTransform`) against `panda_frame_transform_{request,response}.json`.
//! - `is_state_valid_parity::panda_is_state_valid_matches_the_oracle` —
//!   [`PlanningScene::is_state_valid`] against `panda_is_state_valid.json`.
//! - `attached_collision_parity::pr2_attached_body_robot_collision_matches_the_oracle`
//!   — attached-body collision checking against `pr2_attached_collision.json`.
//!
//! [`PlanningScene::check_collision`]/[`PlanningScene::check_self_collision`]/
//! [`PlanningScene::distance_to_collision`]/[`PlanningScene::colliding_pairs`]/
//! [`PlanningScene::colliding_links`] are deliberately *not* independently
//! oracle-tested at this crate's level: each is a thin generic wrapper over
//! a caller-supplied `E: CollisionEnv`, delegating the entire numeric
//! collision check to it — re-asserting FCL-vs-`parry3d-f64` agreement here
//! would duplicate `moveit-collision`'s own oracle parity suite rather than
//! test anything specific to this crate. What this crate's own (non-oracle)
//! unit tests cover instead is the wiring around that delegation — that a
//! scene's ACM, attached bodies, and world are correctly folded into the
//! `E` a caller builds (`scene.rs`'s `mod tests`, the `// ---- collision
//! checking ----` section starting at `scene.rs:2672`).

mod attached_body;
mod layered;
mod scene;
mod world_diff;

pub use attached_body::AttachedBody;
pub use scene::{ObjectColor, ObjectType, PathValidity, PlanningScene};
pub use world_diff::WorldDiff;
