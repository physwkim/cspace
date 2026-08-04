// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs::msg::{PlanningScene, PlanningSceneWorld, CollisionObject,
//! AttachedCollisionObject}` <-> `moveit_scene`/`moveit_collision`.
//! See `doc/message-mapping.md` §11.

pub mod attached;
pub mod collision_object;
pub mod planning_scene;
pub mod shapes;
