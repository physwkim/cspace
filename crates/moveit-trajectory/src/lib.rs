// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (lines 62-192)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp

//! The model-independent numeric core of time-optimal trajectory generation
//! (Kunz & Stilman, "Time-Optimal Trajectory Generation for Path Following
//! with Bounded Acceleration and Velocity").
//!
//! This crate ports [`Path`] (with its private [`PathSegment`](path_segment)
//! kinds) and [`Trajectory`] — the parts of upstream's
//! `time_optimal_trajectory_generation.hpp`/`.cpp` that operate purely on
//! `Vec<DVector<f64>>` waypoints and per-joint velocity/acceleration bounds,
//! with no `moveit_core::robot_model`/`robot_trajectory` dependency anywhere.
//!
//! # Out of scope
//!
//! `TimeOptimalTrajectoryGeneration` (the `robot_trajectory::RobotTrajectory`
//! adapter, header line 193 on) is **not** ported here — see
//! `PORTING-PLAN.md` and [`crate::trajectory`]'s module doc comment.

mod numeric;
mod path;
mod path_segment;
mod trajectory;

pub use path::{DEFAULT_PATH_TOLERANCE, Path};
pub use trajectory::Trajectory;
