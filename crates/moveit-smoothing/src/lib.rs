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
//! - `acceleration_filter.cpp` — unported (see "Acceleration-limited
//!   filter: not excluded, not yet ported" below). Not excluded by any
//!   D-decision.

/// A `Synchronization::Phase` Ruckig streaming filter — see the module doc's
/// `ruckig_filter.cpp` entry.
pub mod ruckig_filter;

mod butterworth;

pub use butterworth::{ButterworthFilter, EPSILON};

// Acceleration-limited filter: not excluded, not yet ported
// ===========================================================
//
// `AccelerationLimitedPlugin` (`acceleration_filter.hpp`/`.cpp`) computes,
// every control cycle, the scalar `alpha` of smallest magnitude (i.e.
// minimizing `alpha^2`) such that `p_n = alpha*p_t + (1-alpha)*p_c` stays
// within every joint's acceleration limits, by solving:
//
// ```text
// minimize   alpha^2
// subject to l_i <= (p_c_i - p_t_i) * alpha <= u_i   for each joint i
//            0 <= alpha <= 1
// ```
//
// via `osqp` (the `osqp` C QP solver, linked through `<osqp.h>`). This
// crate's previous doc comment excluded it citing "ROS surface" and "no
// pure-Rust `osqp` binding is a workspace dependency" — both wrong or
// incomplete, corrected here:
//
// - **Not a ROS surface.** `initialize`'s `node` argument is used only for
//   `ParamListener`/YAML loading (`params_.planning_group_name`), the same
//   D1-excludable coupling `ButterworthFilterPlugin` and (this round)
//   `RuckigFilterPlugin` have; `doSmoothing`/`reset` touch only
//   `robot_model_`, `Eigen::VectorXd`, and `osqp` state.
// - **Not genuinely a general QP.** The problem above has exactly one
//   optimization variable (`alpha`) and every constraint is `alpha` scaled
//   by a constant, bounded by an interval — a 1-D box-constrained QP with a
//   closed-form solution: for each joint `i` with `(p_c_i - p_t_i) != 0`,
//   divide the constraint through to get an interval for `alpha`
//   (flipping the interval if the coefficient is negative); intersect every
//   joint's interval with `[0, 1]`; the minimizer of `alpha^2` over a
//   nonempty interval is the point in that interval closest to `0`. No
//   general-purpose QP solver is mathematically required — `osqp` here is
//   solving a problem simple enough not to need one.
//
// So neither the old exclusion reason holds, and no pure-Rust QP crate
// needs to be adopted to port this. What remains unresolved: this crate's
// established discipline (`PORTING-PLAN.md` §40/§54.2, this round's own
// `verify-fixture-replay.sh` 15/15) is real-oracle-captured ground truth
// for every ported numeric core, not hand-derived test vectors — and
// `tools/moveit-oracle/src/oracle.cpp` has no scaffolding at all for a
// `SmoothingBaseClass`/pluginlib-shaped op. `RuckigFilterPlugin` above
// was ported without this too — it is not oracle-verified either — but its
// risk is different in kind, not degree: `RuckigFilter::do_smoothing`
// wraps `rsruckig::Ruckig::update`, an already-established third-party
// crate's own public streaming API, called per its own documented usage
// pattern, with no new algorithm of this port's own underneath it. Porting
// `AccelerationLimitedPlugin` here would mean shipping a from-scratch
// closed-form QP reduction — this port's own derivation above, not a
// library's already-exercised code path — with no ground truth at all to
// catch a mistake in it, including at the boundary where `osqp`'s
// `eps_abs`-widened feasibility check (`alpha` accepted within `eps_abs`
// of `[0, 1]`) could diverge from an exact interval intersection on a
// degenerate case. This is a well-scoped follow-up, not a redesign
// question — the derivation above is the whole algorithm — but it needs
// either an oracle op for this file (new scaffolding: this plugin's
// `initialize` needs an `rclcpp::Node`, unlike every op the oracle
// currently has) or an explicit sign-off to test it by closed-form-derived
// unit tests alone.
