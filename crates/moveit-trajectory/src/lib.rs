// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (lines 62-192)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp
//   moveit_core/robot_trajectory/include/moveit/robot_trajectory/robot_trajectory.hpp
//   moveit_core/robot_trajectory/src/robot_trajectory.cpp

//! Two related but independent pieces of upstream trajectory handling:
//!
//! - The model-independent numeric core of time-optimal trajectory
//!   generation (Kunz & Stilman, "Time-Optimal Trajectory Generation for
//!   Path Following with Bounded Acceleration and Velocity"): [`Path`] (with
//!   its private [`PathSegment`](path_segment) kinds) and [`Trajectory`],
//!   the parts of upstream's `time_optimal_trajectory_generation.hpp`/`.cpp`
//!   that operate purely on `Vec<DVector<f64>>` waypoints and per-joint
//!   velocity/acceleration bounds, with no
//!   `moveit_core::robot_model`/`robot_trajectory` dependency anywhere.
//! - [`robot_trajectory::RobotTrajectory`], upstream's `robot_trajectory::
//!   RobotTrajectory` — a sequence of `RobotState` waypoints plus
//!   per-waypoint durations. Unlike the two types above, this one *does*
//!   depend on `moveit-model`/`moveit-state`; see that module's doc comment.
//!
//! # Out of scope
//!
//! `TimeOptimalTrajectoryGeneration` (the `robot_trajectory::RobotTrajectory`
//! adapter, header line 193 on) is **not** ported here — see
//! `PORTING-PLAN.md` and [`crate::trajectory`]'s module doc comment.

mod numeric;
mod path;
pub mod path_segment;
pub mod robot_trajectory;
pub mod trajectory;

pub use path::{DEFAULT_PATH_TOLERANCE, Path};
pub use robot_trajectory::RobotTrajectory;
pub use trajectory::Trajectory;
