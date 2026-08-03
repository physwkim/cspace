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
//! # Symbol audit: every `online_signal_smoothing/src/` file
//!
//! - `butterworth_filter.cpp`:
//!   - `ButterworthFilter` (class) — ported as [`ButterworthFilter`].
//!   - `ButterworthFilterPlugin` (class) — excluded (D1). The
//!     `SmoothingBaseClass` implementation wrapping a
//!     `std::vector<ButterworthFilter>`, one per joint. It reads
//!     `butterworth_filter_coeff` from a `generate_parameter_library`
//!     `ParamListener` (ROS parameter YAML tooling) and its `initialize`
//!     takes an `rclcpp::Node::SharedPtr` — a ROS type in the signature,
//!     out of scope per `PORTING-PLAN.md` D1. [`ButterworthFilter`] takes
//!     its coefficient as a plain constructor argument instead, so a
//!     caller can reproduce the per-joint fan-out itself without any ROS
//!     dependency.
//! - `smoothing_base_class.cpp` — excluded (D1 + D4). `SmoothingBaseClass`
//!   is a pluginlib abstract interface: `initialize` takes
//!   `rclcpp::Node::SharedPtr` in the trait itself (D1), and the class
//!   exists specifically to be `pluginlib`-loadable
//!   (`PLUGINLIB_EXPORT_CLASS`), which D4 replaces workspace-wide with a
//!   compile-time trait + `linkme` registry rather than a runtime plugin
//!   interface — so even a Rust-native common trait over
//!   [`ButterworthFilter`]/[`ruckig_filter::RuckigFilter`] would not mirror
//!   this shape. `.cpp` has no content to port regardless: it is a default
//!   constructor/destructor.
//! - `ruckig_filter.cpp`:
//!   - `RuckigFilterPlugin` (class) — ported as
//!     [`ruckig_filter::RuckigFilter`] (`initialize`/`doSmoothing`/`reset`)
//!     and [`ruckig_filter::joint_vel_accel_jerk_bounds`]
//!     (`getVelAccelJerkBounds`). See `ruckig_filter.rs`'s module doc for
//!     the ROS-coupling analysis this rests on and every deviation from
//!     upstream. `printRuckigState` (private) is not ported — a diagnostic
//!     with no effect on computed output, matching `ruckig_smoothing.rs`'s
//!     equivalent logging exclusions.
//! - `acceleration_filter.cpp`:
//!   - `AccelerationLimitedPlugin` (class) — ported as
//!     [`acceleration_filter::AccelerationLimitedFilter`]
//!     (`initialize`/`doSmoothing`/`reset`) and
//!     [`acceleration_filter::joint_acceleration_bounds`]. Verified against
//!     a real `tools/moveit-oracle/src/oracle.cpp` `acceleration_filter` op
//!     (not closed-form-derived test vectors alone — see
//!     `acceleration_filter.rs`'s module doc for why that distinction
//!     matters here specifically, and every deviation from upstream).
//!     `jointLimitAccelerationScalingFactor` (free function) is ported as
//!     a private helper of the same name's-worth of behavior in
//!     `acceleration_filter.rs`.

/// A `Synchronization::Phase` Ruckig streaming filter — see the module doc's
/// `ruckig_filter.cpp` entry.
pub mod ruckig_filter;

/// The acceleration-limiting QP filter — see the module doc's
/// `acceleration_filter.cpp` entry.
pub mod acceleration_filter;

mod butterworth;

pub use butterworth::{ButterworthFilter, EPSILON};
