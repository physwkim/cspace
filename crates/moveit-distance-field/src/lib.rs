// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/voxel_grid.hpp
//   moveit_core/distance_field/include/moveit/distance_field/distance_field.hpp
//   moveit_core/distance_field/include/moveit/distance_field/propagation_distance_field.hpp
//   moveit_core/distance_field/include/moveit/distance_field/find_internal_points.hpp
//   moveit_core/distance_field/src/distance_field.cpp
//   moveit_core/distance_field/src/propagation_distance_field.cpp
//   moveit_core/distance_field/src/find_internal_points.cpp

//! Voxel distance fields for moveit-rs: dense 3D grids holding the distance
//! from every cell to the nearest obstacle.
//!
//! # Scope
//!
//! This crate ports `moveit_core/distance_field` only — self-contained voxel
//! maths with no `RobotModel` dependency. `collision_distance_field` (which
//! layers robot-link tracking on top of this) belongs to a later phase; see
//! `PORTING-PLAN.md` §3.
//!
//! - [`VoxelGrid`] — the generic dense grid, with the world↔cell coordinate
//!   conversion whose rounding convention is load-bearing (see its `impl`
//!   doc on [`VoxelGrid::cell_from_location`]).
//! - [`DistanceField`] — the query interface. Per `PORTING-PLAN.md` D4 this
//!   is a trait rather than upstream's abstract base class.
//! - [`PropagationDistanceField`] — the (currently only) implementer,
//!   propagating distances outward via bucketed-queue wavefront expansion.
//! - [`find_internal_points_convex`] / [`ConvexBody`] — the point-sampling
//!   helper used to turn a shape into obstacle points.
//!
//! See [`DistanceField`]'s doc comment for what upstream's abstract base
//! class carries that is deliberately *not* ported here, and why.

mod distance_field;
mod find_internal_points;
mod posed_shape;
mod propagation;
mod voxel_grid;

pub use distance_field::{DistanceField, DistanceGradient};
pub use find_internal_points::{ConvexBody, find_internal_points_convex};
pub use propagation::{NearestCell, PropDistanceFieldVoxel, PropagationDistanceField};
pub use voxel_grid::{Dimension, GridGeometry, VoxelGrid};
