// Copyright (c) 2020, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/

//! STOMP planner support types, ported from `moveit_planners/stomp/`'s
//! ROS-independent headers: matrix<->trajectory conversion
//! ([`conversion_functions`]) and trajectory filters ([`filter_functions`]).
//!
//! # `stomp`, the optimizer core: ported, but into `moveit-stomp-core`, not here
//!
//! STOMP's actual optimization loop lives in the separate upstream
//! repository `ros-industrial/stomp`, not in `moveit2` -- see
//! `moveit-stomp-core`'s own module doc for how that source was obtained
//! and verified, and for the one-crate-one-upstream reasoning behind
//! keeping it in its own crate rather than here. This crate now depends on
//! it and calls `moveit_stomp_core::generate_smoothing_matrix` from
//! `filter_functions::simple_smoothing_matrix`.
//!
//! Not ported into either crate:
//!
//! - `cost_functions.hpp` (collision/validity cost) is deferred to a later
//!   round: it needs `moveit-scene`'s collision surface, out of this
//!   crate's dependency reach this round.
//! - `noise_generators.hpp` is out of this round's stated scope. Its only
//!   `MultivariateGaussian::sample` call site (`rand_generators[i]
//!   ->sample(*raw_noise)`, no second argument, i.e. the with-covariance
//!   branch) was read to confirm `moveit-sampling`'s two-method split
//!   matches STOMP's actual usage, but `noise_generators.hpp` itself is not
//!   ported here.
//!
//! # Not ported: the ROS/task-engine layer (D1/D2 exclusion)
//!
//! `stomp_moveit_planning_context.{hpp,cpp}`, `stomp_moveit_task.hpp`,
//! `trajectory_visualization.hpp`, and the plugin registration `.cpp` are
//! not ported. Per PORTING-PLAN.md's D1 ("final form: a ROS-independent
//! Rust motion-planning library") and D2 ("ROS 2 bindings isolated to an
//! optional `moveit-ros` crate"), this crate carries only the
//! ROS-independent computational core; the `planning_interface::PlanningContext`
//! plugin glue, the `rclcpp`-visible task/trajectory-visualization types,
//! and pluginlib registration belong to a ROS integration layer this
//! workspace does not port into `moveit-planners-stomp` itself. `FilterFn`
//! is the one piece of `stomp_moveit_task.hpp` this crate does carry, since
//! every filter function here needs the signature -- see
//! `filter_functions`' module doc, "`FilterFn`'s home".
//!
//! # `MultivariateGaussian`'s new home
//!
//! `multivariate_gaussian.hpp`'s class does not live in this crate.
//! `moveit_planners/chomp/chomp_motion_planner/`'s own
//! `MultivariateGaussian` (a future round's `moveit-planners-chomp`) is the
//! same algorithm in a separately maintained file, diffed directly against
//! this one this round -- see `moveit_sampling::multivariate_gaussian`'s
//! module doc for the full comparison. Rather than have one planner crate
//! depend on the other, both depend on the `moveit-sampling` crate instead.
//!
//! # `assert_relative_eq!` reckoning (§79 convention, applied from the start)
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-planners-stomp/src/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Run for real against the tree as committed this round. Zero calls: this
//! crate's tests compare exact-integer-representable f64 values (bound
//! clamps, waypoint counts, matrix round-trips through the same arithmetic
//! on both sides) with `assert_eq!`/`assert_ne!`, never a tolerance-bearing
//! float comparison. Nothing to classify, nothing to bisect, no tolerance
//! floor to re-measure.

use moveit_error::{Error, Result};

pub mod composable_task;
pub mod conversion_functions;
pub mod cost_functions;
pub mod filter_functions;
pub mod noise_generators;

/// The precondition `conversion_functions` and `filter_functions` both
/// require: `name`'s joint must have exactly one variable. See
/// `conversion_functions`' module doc, "Single-variable-joint
/// precondition".
pub(crate) fn require_single_variable(name: &str, variable_count: usize) -> Result<()> {
    if variable_count != 1 {
        return Err(Error::Other(format!(
            "joint '{name}' has {variable_count} variables, but STOMP's matrix \
             representation requires every active joint in the group to have \
             exactly one variable"
        )));
    }
    Ok(())
}
