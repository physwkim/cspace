// Copyright (c) 2011, Willow Garage, Inc.
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
//! [`PlanningScene::decouple_parent`]). `moveit_constraints` — kinematic
//! constraint types and `PlanningScene::isStateConstrained` — is out of
//! scope for this crate; see [`PlanningScene`]'s own doc for the rest of
//! what upstream `planning_scene.cpp` carries that this crate does not yet
//! port (message round-tripping, named-frame transforms, collision-check
//! passthroughs).
//!
//! See [`PlanningScene`]'s doc for the parent/child design — deliberately
//! reasoned through rather than transcribed from upstream's
//! `std::optional`-plus-const-pointer shape.

mod attached_body;
mod layered;
mod scene;
mod world_diff;

pub use attached_body::AttachedBody;
pub use scene::PlanningScene;
pub use world_diff::WorldDiff;
