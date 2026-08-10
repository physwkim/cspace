// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_env.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_octomap_filter.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/world.hpp
//   moveit_core/collision_detection/src/collision_common.cpp
//   moveit_core/collision_detection/src/collision_env.cpp
//   moveit_core/collision_detection/src/collision_matrix.cpp
//   moveit_core/collision_detection/src/collision_octomap_filter.cpp
//   moveit_core/collision_detection/src/collision_tools.cpp
//   moveit_core/collision_detection/src/world.cpp

//! The `RobotModel`-independent slice of `moveit_core/collision_detection`:
//! [`AllowedCollisionMatrix`], the request/result types every collision
//! backend shares, the [`World`] collision-object container, the
//! [`CollisionEnv`] backend interface, [`LinkPaddingScale`]'s padding/scale
//! bookkeeping, pure [`CostSource`] utilities ([`total_cost`],
//! [`intersect_cost_sources`], [`remove_overlapping`],
//! [`remove_cost_sources`], [`sensor_positioning`]), a concrete
//! [`ParryCollisionEnv`] backend over `parry3d-f64` (see `parry`'s module
//! docs for its scope and deviations from upstream's FCL backend),
//! [`AllValidCollisionEnv`], upstream's null backend that reports no
//! collision ever (see `all_valid`'s module docs for how a caller selects
//! it, there being no allocator to select it by name), and
//! [`refine_contact_normals`], the octomap contact-normal/depth refit (see
//! `octomap_filter`'s module docs).
//!
//! # Out of scope here
//!
//! The FCL/Bullet backends that implement [`CollisionEnv`] are owned by other
//! workers/tasks and not touched here — see `env`'s module docs.
//!
//! `collision_plugin_cache.*` stays out of scope, but not for the reason
//! previously written here: `CollisionPluginCache` has 0 `RobotState`
//! references, yet its entire body is pluginlib runtime class loading
//! (`#include <pluginlib/class_loader.hpp>`,
//! `pluginlib::ClassLoader<CollisionPlugin>`) plus `rclcpp` logging — no
//! algorithm exists independent of that ROS mechanism
//! (`collision_plugin_cache.cpp:37-38`). `CollisionPlugin::initialize` also
//! takes a `planning_scene::PlanningScenePtr` (`collision_plugin.hpp:93`);
//! `PlanningScene` lives in `cspace_planning::scene`, which already depends on
//! `cspace-collision`, so accepting it here would be a circular crate
//! dependency regardless of the pluginlib question. This expires only if some
//! other worker builds a non-ROS pluggable-backend mechanism directly against
//! [`CollisionEnv`] — not by any change local to this crate.
//!
//! `occupancy_map.hpp` (header-only, no `.cpp`) is a different case: its
//! `OccMapTree`/`OccMapNode` have 0 `RobotState` references and 0 ROS
//! includes (`<octomap/octomap.h>`, `<memory>`, `<string>`, `<shared_mutex>`,
//! `<mutex>`, `<functional>` only) — a `std::shared_mutex`-guarded
//! `octomap::OcTree` subclass with lock/unlock and an update callback. It is
//! genuinely `RobotState`-free and portable, so "no portable piece at all"
//! was also false for this header. But nothing in
//! `moveit_core/collision_detection` itself references `OccMapTree`; its real
//! callers are `moveit_core/planning_scene/src/planning_scene.cpp` and
//! `moveit_ros/{occupancy_map_monitor,perception/lazy_free_space_updater,
//! planning/planning_scene_monitor}` — none of them collision-detection code.
//! A thread-safe octree wrapper with no collision-detection-specific logic
//! belongs with `cspace_core::octomap` (owned by p3-shapes) or `cspace-planning-
//! scene`, not here; it is out of this crate's scope by directory ownership,
//! not by portability.
//!
//! That was a routing decision, not a decision, and §231.2 settles it as
//! **not ported anywhere**: `OccMapTree` adds exactly two forwarding
//! constructors, six `std::shared_mutex` lock methods
//! (`lockRead`/`unlockRead`/`lockWrite`/`unlockWrite`/`reading`/`writing`),
//! and `setUpdateCallback`/`triggerUpdateCallback`, and **no file in this
//! port's upstream corpus uses any of it**. The one in-corpus includer,
//! `moveit_core/planning_scene/src/planning_scene.cpp:39`, constructs it
//! with `OccMapTree(map.resolution)` and then calls only `readTree`/
//! `readData` (`createOctomap`, `:1417-1435`) — plain `octomap::OcTree`,
//! which `ros/cspace-ros/src/scene/planning_scene.rs:137-143` already does
//! with `cspace_core::octomap::OcTree::new` plus `read_binary_data`/`read_data`.
//! Every user of the added API is under `moveit_ros/*`, outside the corpus.
//! The two mechanisms it adds are also the two this port expresses
//! differently by construction: shared mutability lives at the use site
//! (every octree in this tree is an immutable `Arc<OcTree>`), and an update
//! callback is the monitor's, not the tree's. Re-opening this needs an
//! in-corpus caller of the lock or callback API, not a routing preference.
//!
//! The `bodies::` posed-geometry layer is likewise owned by other workers and
//! out of scope for [`World`] — see `world`'s module docs.

pub mod distance_field;

mod all_valid;
mod common;
mod env;
mod fcl_tangency_table;
mod matrix;
mod mesh_tangency_table;
mod numeric;
mod obb_bvh;
mod obb_fit;
mod octomap_filter;
mod parry;
mod tools;
mod world;

pub use all_valid::AllValidCollisionEnv;
pub use common::{
    AttachedBodyGeometry, BodyType, CollisionDistance, CollisionRequest, CollisionResult, Contact,
    ContactData, CostSource, DistanceMap, DistanceRequest, DistanceRequestType, DistanceResult,
    DistanceResultsData, IsDoneFn,
};
pub use env::{CollisionEnv, LinkPaddingScale};
pub use matrix::{AllowedCollision, AllowedCollisionMatrix, AllowedCollisionType, DecideContactFn};
pub use octomap_filter::refine_contact_normals;
pub use parry::ParryCollisionEnv;
pub use tools::{
    intersect_cost_sources, remove_cost_sources, remove_overlapping, sensor_positioning, total_cost,
};
pub use world::{Action, MoveObjectOutcome, Notification, Object, ShapeEntry, World};
