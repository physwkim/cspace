// Copyright (c) 2009, 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_parameters.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_parameters.cpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_utils.hpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_trajectory.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_trajectory.cpp

//! CHOMP (Covariant Hamiltonian Optimization for Motion Planning), ported
//! from upstream's `moveit_planners/chomp` package.
//!
//! # `chomp_interface/` is not ported
//!
//! Upstream `moveit_planners/chomp` splits into two subpackages:
//! `chomp_motion_planner/` (the algorithm and its data structures — no ROS
//! type anywhere) and `chomp_interface/` (a `pluginlib` `PlanningContext`
//! adapter exposing it to `move_group`). Only `chomp_motion_planner/` is
//! ported here. `chomp_interface/` is excluded per `PORTING-PLAN.md` D1/D2:
//! this port's core crates reference no ROS type at all, and
//! `chomp_interface`'s reason to exist — being a `pluginlib`-loadable
//! `planning_interface::PlanningContext` — is exactly the `move_group`
//! drop-in shape D1 puts out of scope for the core. Nothing in
//! `chomp_interface/` is algorithmic; it only adapts `ChompPlanner` (see
//! below) to ROS parameters and a live `PlanningScene`.
//!
//! # Round 15 scope: 3 of 7 upstream files
//!
//! `chomp_motion_planner/` has 7 header/source pairs. This round ports and
//! audits exactly 3: `chomp_parameters`, `chomp_utils`, `chomp_trajectory`.
//! The remaining 4 — `chomp_cost`, `multivariate_gaussian`,
//! `chomp_optimizer`, `chomp_planner` — are **not yet audited**, not
//! silently absent: `chomp_optimizer` is the hardest piece and depends on
//! the data structures ported this round being right first, so it is
//! deliberately deferred rather than rushed in the same round. Do not infer
//! from their absence below that they were considered and skipped; they
//! have not been read for this crate at all yet.
//!
//! `multivariate_gaussian.hpp` is shared, near-verbatim, with upstream's
//! `moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`.
//! The two are not byte-identical — STOMP's `sample()` takes an extra `bool
//! use_covariance = true` parameter CHOMP's does not have — but the
//! Cholesky-based sampling core (`mean_`/`covariance_`/
//! `covariance_cholesky_` via `llt().matrixL()`,
//! `std::normal_distribution<double>(0.0, 1.0)`) is the same algorithm.
//! Decided (human orchestrator): one shared port lives in a new
//! `moveit-sampling` crate, owned by `p3-shapes`, depended on by both
//! `moveit-planners-chomp` and `moveit-planners-stomp` rather than one
//! planner depending on its sibling. The `use_covariance` bool is not
//! carried over as a bool parameter — a bool that switches what a function
//! computes is exactly the dual-meaning shape this port avoids elsewhere —
//! `moveit-sampling` instead exposes two named methods (covariance applied /
//! not applied); this crate calls only the covariance-applied one, matching
//! CHOMP's own `sample()`, which has no such parameter at all. This crate
//! does not yet depend on `moveit-sampling`: that dependency, and the actual
//! `chomp_optimizer`/`chomp_cost` files that would use it, are out of this
//! round's 3-file scope, added when the optimizer is ported.
//!
//! # Symbol audit: `chomp_parameters.{hpp,cpp}`
//!
//! - `ChompParameters` (class) — ported as [`parameters::ChompParameters`].
//!   All 20 public data members ported as public fields with upstream's
//!   exact default-constructor values; `setRecoveryParams` ported as
//!   [`parameters::ChompParameters::set_recovery_params`];
//!   `setTrajectoryInitializationMethod` ported as
//!   [`parameters::ChompParameters::set_trajectory_initialization_method`],
//!   kept a validated `String` rather than redesigned into an enum — see
//!   that method's doc comment for why. `VALID_INITIALIZATION_METHODS`
//!   ported as [`parameters::VALID_INITIALIZATION_METHODS`]. The default
//!   destructor (`virtual ~ChompParameters()`) has no Rust equivalent to
//!   port; `virtual` exists upstream only so a subclass (none exists in
//!   this package) could override it.
//!
//! # Symbol audit: `chomp_utils.hpp`
//!
//! - `DIFF_RULE_LENGTH` — ported as [`utils::DIFF_RULE_LENGTH`].
//! - `DIFF_RULES` — ported as [`utils::DIFF_RULES`].
//! - `normalizeAnglePositive` — ported as [`utils::normalize_angle_positive`].
//! - `normalizeAngle` — ported as [`utils::normalize_angle`].
//! - `shortestAngularDistance` — ported as [`utils::shortest_angular_distance`].
//! - `robotStateToArray` — ported as [`utils::robot_state_to_array`].
//!
//! # Symbol audit: `chomp_trajectory.{hpp,cpp}`
//!
//! - `ChompTrajectory` (class) — ported as [`trajectory::ChompTrajectory`].
//!   The duration-based and num-points-based constructors are ported as
//!   [`trajectory::ChompTrajectory::from_duration`] and
//!   [`trajectory::ChompTrajectory::from_num_points`]; the copy-with-padding
//!   constructor as [`trajectory::ChompTrajectory::from_source_trajectory`].
//!   The `trajectory_msgs::msg::JointTrajectory`-typed constructor is
//!   excluded (D1): its signature carries a ROS message type directly, and
//!   nothing else in this round's scope constructs a `ChompTrajectory` from
//!   one. `operator()` (both overloads) is ported as
//!   `impl `[`std::ops::Index`]`/`[`std::ops::IndexMut`]` for
//!   `[`trajectory::ChompTrajectory`]` on `(usize, usize)``. All other
//!   accessors and the three `fillIn*` methods, `fillInFromTrajectory`,
//!   `assignCHOMPTrajectoryPointFromRobotState` and `getJointVelocities` are
//!   ported as their `snake_case` equivalents on
//!   [`trajectory::ChompTrajectory`] — see that module's own doc comment for
//!   the full name mapping and every deviation from upstream.
//!   `getFreeTrajectoryBlock`/`getFreeJointTrajectoryBlock` are declared but
//!   **not yet ported** — see that module doc's own deviation note for why.
//!   The private `init` has no separate Rust equivalent: every public
//!   constructor allocates its matrix directly via `DMatrix::zeros` instead.
//!
//! # Completion condition
//!
//! Stated as a check on this round's 3-file scope, not a claim about the
//! crate: `chomp_parameters.{hpp,cpp}`, `chomp_utils.hpp`, and
//! `chomp_trajectory.{hpp,cpp}` are read in full against the pinned SHA and
//! every symbol in them is classified above as ported (with its Rust name)
//! or D-decision-excluded (with the decision). No numeric oracle op backs
//! any of this round's tests — Phase 8's completion condition uses
//! property-based verification (`PORTING-PLAN.md` §5), not a trajectory
//! oracle, and CHOMP specifically is not the one Phase-8 planner
//! (`moveit-planners-pilz`) with directly comparable deterministic output.
//! What is pinned by unit test instead: [`trajectory::ChompTrajectory`]'s
//! copy-with-padding indexing/`full_trajectory_index_` convention, which
//! silently diverges if wrong with no compiler or oracle signal. This
//! section does not cover `chomp_cost`, `multivariate_gaussian`,
//! `chomp_optimizer`, or `chomp_planner` — they are out of scope this round
//! per the section above, not implicitly satisfied by anything here.

/// `ChompParameters` and its trajectory-initialization-method validation —
/// see the module doc's `chomp_parameters.{hpp,cpp}` entry.
pub mod parameters;

/// `DIFF_RULE_LENGTH`/`DIFF_RULES` finite-difference stencils and CHOMP's own
/// angle-normalization helpers — see the module doc's `chomp_utils.hpp`
/// entry.
pub mod utils;

/// `ChompTrajectory` — see the module doc's `chomp_trajectory.{hpp,cpp}`
/// entry.
pub mod trajectory;

pub use parameters::ChompParameters;
pub use trajectory::ChompTrajectory;
