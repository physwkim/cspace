// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/world.hpp
//   moveit_core/collision_detection/src/collision_common.cpp
//   moveit_core/collision_detection/src/collision_matrix.cpp
//   moveit_core/collision_detection/src/world.cpp

//! The `RobotModel`-independent slice of `moveit_core/collision_detection`:
//! [`AllowedCollisionMatrix`], the request/result types every collision
//! backend shares, and the [`World`] collision-object container.
//!
//! # Out of scope here
//!
//! `CollisionEnv`, and the FCL/Bullet/parry backends that implement it, all
//! need a `RobotModel` to check collisions against. PORTING-PLAN.md Phase 3
//! puts them wherever `moveit-model`'s consumer lands, not in this module.
//! `RobotState` and the `bodies::` posed-geometry layer are likewise owned by
//! other workers and out of scope for [`World`] — see `world`'s module docs.

mod common;
mod matrix;
mod world;

pub use common::{
    BodyType, CollisionDistance, CollisionRequest, CollisionResult, Contact, ContactData,
    CostSource, DistanceMap, DistanceRequest, DistanceRequestType, DistanceResult,
    DistanceResultsData, IsDoneFn,
};
pub use matrix::{AllowedCollision, AllowedCollisionMatrix, AllowedCollisionType, DecideContactFn};
pub use world::{Action, MoveObjectOutcome, Notification, Object, ShapeEntry, World};
