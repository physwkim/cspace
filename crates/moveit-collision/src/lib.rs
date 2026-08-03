// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_matrix.hpp
//   moveit_core/collision_detection/src/collision_common.cpp
//   moveit_core/collision_detection/src/collision_matrix.cpp

//! The `RobotModel`-independent slice of `moveit_core/collision_detection`:
//! [`AllowedCollisionMatrix`] and the request/result types every collision
//! backend shares.
//!
//! # Out of scope here
//!
//! `CollisionEnv`, and the FCL/Bullet/parry backends that implement it, all
//! need a `RobotModel` to check collisions against. PORTING-PLAN.md Phase 3
//! puts them wherever `moveit-model`'s consumer lands, not in this module.

mod common;
mod matrix;

pub use common::{
    BodyType, CollisionRequest, CollisionResult, Contact, CostSource, DistanceMap, DistanceRequest,
    DistanceRequestType, DistanceResult, DistanceResultsData, IsDoneFn,
};
pub use matrix::{AllowedCollision, AllowedCollisionMatrix, AllowedCollisionType, DecideContactFn};
