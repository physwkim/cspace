// Copyright (c) 2021, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/butterworth_filter.hpp
//   moveit_core/online_signal_smoothing/src/butterworth_filter.cpp

//! The model-independent numeric core of upstream `online_signal_smoothing`:
//! a single-signal first-order Butterworth low-pass filter, ported as
//! [`ButterworthFilter`], and a `Synchronization::Phase` Ruckig streaming
//! filter, ported as [`ruckig_filter::RuckigFilter`].
//!
//! # Out of scope
//!
//! - `SmoothingBaseClass` (`smoothing_base_class.hpp`/`.cpp`) — a pluginlib
//!   abstract interface (`initialize(rclcpp::Node::SharedPtr,
//!   RobotModelConstPtr, size_t)`, `doSmoothing`, `reset`) with no
//!   model-independent content: `.cpp` is a default constructor/destructor.
//! - `ButterworthFilterPlugin` (in `butterworth_filter.hpp`/`.cpp`) — the
//!   `SmoothingBaseClass` implementation wrapping a `std::vector<ButterworthFilter>`,
//!   one per joint. It reads `butterworth_filter_coeff` from a
//!   `generate_parameter_library`-generated `ParamListener` (ROS parameter
//!   YAML tooling, out of scope per `PORTING-PLAN.md` D1) and calls
//!   `RCLCPP_ERROR_THROTTLE` against an `rclcpp::Node`. [`ButterworthFilter`]
//!   here takes its coefficient as a plain constructor argument instead, so
//!   a caller can reproduce the per-joint fan-out itself without any ROS
//!   dependency.
//! - `AccelerationLimitedFilter` (`acceleration_filter.hpp`/`.cpp`) — solves
//!   a QP each step via `<osqp.h>` (the `osqp` C library) to enforce
//!   acceleration/jerk limits jointly across all DOF. No pure-Rust `osqp`
//!   binding is a workspace dependency; porting this needs one adopted
//!   first.
//! - `RuckigFilterPlugin` (`ruckig_filter.hpp`/`.cpp`) — ported as
//!   [`ruckig_filter::RuckigFilter`]/[`ruckig_filter::joint_vel_accel_jerk_bounds`].
//!   See `ruckig_filter.rs`'s module doc for the ROS-coupling analysis this
//!   rests on (only `initialize`'s ROS-parameter-YAML loading was ever
//!   coupled to `rclcpp::Node`; `doSmoothing`/`reset`/`getVelAccelJerkBounds`
//!   were not) and every deviation from upstream.

/// A `Synchronization::Phase` Ruckig streaming filter — see the module doc's
/// `RuckigFilterPlugin` entry.
pub mod ruckig_filter;

mod butterworth;

pub use butterworth::{ButterworthFilter, EPSILON};
