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
//!
//! # Completion condition
//!
//! This section is a check, not a claim: it names exactly what "done" means
//! for this crate's current scope, so plan and code can be compared directly
//! instead of re-diverging silently (the pattern `cspace-distance-field`'s
//! own "Completion condition" section established, after PORTING-PLAN.md
//! §65/§71 caught a plan claim nobody could verify against the code).
//!
//! **Headers, fully audited (read in full against the pinned SHA, not
//! inferred from what is already ported):**
//!
//! - `moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/{butterworth_filter,smoothing_base_class,ruckig_filter,acceleration_filter}.hpp`
//!   plus their four `.h` deprecated-forwarding-shim siblings — see the
//!   "Symbol audit: every `online_signal_smoothing/src/` file" section above
//!   for the per-symbol table (headers and `.cpp` files audited together
//!   there, since every class this package declares is declared in its
//!   header and defined in the matching `.cpp`, unlike
//!   `cspace-trajectory`'s `time_optimal_trajectory_generation.cpp`, which
//!   has two classes with no header declaration at all).
//!
//! Every symbol in all four files is classified above as ported (with its
//! Rust name), D-decision-excluded (with the decision), or unported (with
//! the specific reason) — there is no symbol from any of the four left
//! unclassified.
//!
//! **Fixtures, and what they cover:**
//!
//! - `tests/ruckig_filter_parity.rs` — the oracle's `ruckig_filter` op
//!   against [`ruckig_filter::RuckigFilter`]'s `initialize`/`do_smoothing`/
//!   `reset`.
//! - `tests/acceleration_filter_parity.rs` — the oracle's
//!   `acceleration_filter` op against
//!   [`acceleration_filter::AccelerationLimitedFilter`]'s `initialize`/
//!   `do_smoothing`/`reset`, real ground truth rather than the closed-form
//!   derivation alone (see `acceleration_filter.rs`'s own module doc on why
//!   that distinction matters here specifically).
//! - [`ruckig_filter`]'s own `#[cfg(test)]` module — boundary tests with no
//!   oracle op to compare against: `joint_vel_accel_jerk_bounds` rejecting a
//!   group missing acceleration or jerk limits, a typed error (not a silent
//!   last-variable-wins) for a multi-DOF active joint, mismatched-length
//!   rejection for both `do_smoothing` and `reset`, and a first-tick-from-rest
//!   case exercising the streaming `Synchronization::Phase` state machine
//!   directly.
//! - [`acceleration_filter`]'s own `#[cfg(test)]` module — the QP
//!   feasible-interval boundary cases no oracle fixture reaches: an empty
//!   feasible-velocity intersection falling back to decelerate-toward-rest,
//!   a single-point intersection forcing `alpha = 1.0`, a tiny offset
//!   holding at the last commanded value, plus the same
//!   `joint_acceleration_bounds` and typed-error coverage
//!   `ruckig_filter.rs`'s tests give their own bound function.
//! - [`ButterworthFilter`]'s own `#[cfg(test)]` module, in `butterworth.rs`
//!   — both upstream `SMOOTHING_PLUGINS` gtest cases
//!   (`FilterConverge`/`FilterReset`), plus boundary tests for every
//!   constructor rejection (`coeff < 1.0`, a feedback term landing within
//!   [`EPSILON`] of zero from either side, an infinite feedback or scale
//!   term) and the documented NaN-coefficient passthrough.
//!
//! Both oracle-backed fixtures are registered in
//! `tests/fixtures/oracle-models.json` (`acceleration_filter`,
//! `ruckig_filter`, each naming the URDF/SRDF pair its request/response JSON
//! was captured against), and both keys there match a real `op == "..."`
//! branch in `tools/moveit-oracle/src/oracle.cpp`.
//!
//! **What is still missing, and why it is not a gap in the above:** every
//! item is already named individually in the symbol-audit section above
//! with its own reason; this is the roll-up. `ButterworthFilterPlugin`,
//! `RuckigFilterPlugin`, `AccelerationLimitedPlugin`, and
//! `SmoothingBaseClass` itself are all D-decision-excluded for the same two
//! reasons, stated once here rather than per class: each `initialize` takes
//! an `rclcpp::Node::SharedPtr` in its signature (D1), and each class exists
//! specifically to be `pluginlib`-loadable (`PLUGINLIB_EXPORT_CLASS`), which
//! D4 replaces workspace-wide with a compile-time trait + `linkme` registry
//! rather than a runtime plugin interface — so even a Rust-native common
//! trait over the three filters would not mirror this shape. Every
//! `*Filter`/`*LimitedFilter` type these plugins wrap is fully ported (see
//! above); nothing behind a plugin boundary is missing, only the boundary
//! itself, deliberately.
//!
//! This crate's completion condition, stated as a check rather than a
//! claim: every symbol in all four audited files is classified above; every
//! classified-as-ported symbol has either an oracle-driven fixture or a
//! boundary/unit test with a documented reason no oracle op covers it; and
//! every classified-as-unported symbol names the specific D-decision — not
//! "not yet" on its own. If a future symbol or fixture cannot be placed in
//! one of those buckets, this section is stale and needs re-auditing before
//! the plan is updated to match it.

/// A `Synchronization::Phase` Ruckig streaming filter — see the module doc's
/// `ruckig_filter.cpp` entry.
pub mod ruckig_filter;

/// The acceleration-limiting QP filter — see the module doc's
/// `acceleration_filter.cpp` entry.
pub mod acceleration_filter;

mod butterworth;
mod numeric;

pub use butterworth::{ButterworthFilter, EPSILON};
