// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_env.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/world.hpp
//   moveit_core/collision_detection/src/collision_common.cpp
//   moveit_core/collision_detection/src/collision_env.cpp
//   moveit_core/collision_detection/src/collision_matrix.cpp
//   moveit_core/collision_detection/src/collision_tools.cpp
//   moveit_core/collision_detection/src/world.cpp

//! The `RobotModel`-independent slice of `moveit_core/collision_detection`:
//! [`AllowedCollisionMatrix`], the request/result types every collision
//! backend shares, the [`World`] collision-object container, the
//! [`CollisionEnv`] backend interface, [`LinkPaddingScale`]'s padding/scale
//! bookkeeping, pure [`CostSource`] utilities ([`total_cost`],
//! [`intersect_cost_sources`], [`remove_overlapping`],
//! [`remove_cost_sources`], [`sensor_positioning`]), and a concrete
//! [`ParryCollisionEnv`] backend over `parry3d-f64` (see `parry`'s module
//! docs for its scope and deviations from upstream's FCL backend).
//!
//! # Out of scope here
//!
//! The FCL/Bullet backends that implement [`CollisionEnv`] are owned by other
//! workers/tasks and not touched here. `collision_plugin_cache.*` (pluginlib
//! backend selection), `collision_octomap_filter.*` and `occupancy_map.*`
//! (both need an octomap dependency and a `RobotState`) are out of scope
//! entirely — see `env`'s module docs for the first; the latter two have no
//! `RobotState`-free piece to port at all. The `bodies::` posed-geometry
//! layer is likewise owned by other workers and out of scope for [`World`]
//! — see `world`'s module docs.

mod common;
mod env;
mod matrix;
mod parry;
mod tools;
mod world;

pub use common::{
    AttachedBodyGeometry, BodyType, CollisionDistance, CollisionRequest, CollisionResult, Contact,
    ContactData, CostSource, DistanceMap, DistanceRequest, DistanceRequestType, DistanceResult,
    DistanceResultsData, IsDoneFn,
};
pub use env::{CollisionEnv, LinkPaddingScale};
pub use matrix::{AllowedCollision, AllowedCollisionMatrix, AllowedCollisionType, DecideContactFn};
pub use parry::ParryCollisionEnv;
pub use tools::{
    intersect_cost_sources, remove_cost_sources, remove_overlapping, sensor_positioning, total_cost,
};
pub use world::{Action, MoveObjectOutcome, Notification, Object, ShapeEntry, World};
