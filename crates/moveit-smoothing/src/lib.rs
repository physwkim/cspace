// Copyright (c) 2021, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/butterworth_filter.hpp
//   moveit_core/online_signal_smoothing/src/butterworth_filter.cpp

//! The model-independent numeric core of upstream `online_signal_smoothing`:
//! a single-signal first-order Butterworth low-pass filter, ported as
//! [`ButterworthFilter`].
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
//! - `RuckigFilterPlugin` (`ruckig_filter.hpp`/`.cpp`) — unlike
//!   `ButterworthFilter`, upstream never splits out a model-independent
//!   piece here: `RuckigFilterPlugin` is the only class in the file, and it
//!   is a `SmoothingBaseClass` implementation coupled to `rclcpp::Node`,
//!   `RobotModelConstPtr` and a `generate_parameter_library`-generated
//!   `Params` struct throughout (`initialize`, `getVelAccelJerkBounds`) —
//!   the same ROS/`RobotModel` coupling `ButterworthFilterPlugin` has, with
//!   nothing underneath it to extract. `PORTING-PLAN.md` §4.6 no longer
//!   defers the crate choice — it names `rsruckig` (pure Rust, not an FFI
//!   `-sys` binding) — but still defers wiring it in here, because the
//!   blocker is the ROS/`RobotModel` coupling above, not the crate
//!   question.

mod butterworth;

pub use butterworth::{ButterworthFilter, EPSILON};
