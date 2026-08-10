// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp

//! Random sampling shared across planner ports.
//!
//! # Why its own crate
//!
//! `stomp_moveit::math::MultivariateGaussian`
//! (`moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp`)
//! and `chomp::MultivariateGaussian`
//! (`moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp`)
//! are two separately maintained upstream files, diffed directly against
//! each other this round: the algorithmic core (`mean_`/`covariance_`/
//! `covariance_cholesky_` via `covariance.llt().matrixL()`, the same
//! standard-normal sample loop) is the same; only the namespace, a member
//! name, and STOMP's extra `use_covariance` parameter (see
//! [`MultivariateGaussian`]'s module doc) differ. `cspace-planners-stomp`
//! and (in a future round) `cspace-planners-chomp` both need this class.
//! Putting it in either planner crate and having the other depend on it
//! would make one planner depend on a sibling planner, which is not this
//! workspace's dependency direction -- so it lives here instead, and both
//! planner crates depend on `cspace-sampling`.
//!
//! # `assert_relative_eq!` reckoning (round 20, ported to this crate from scratch)
//!
//! Per PORTING-PLAN.md's tolerance-floor mandate and the established §79
//! counting convention (`cspace-geometry`/`cspace-octomap`), every
//! `assert_relative_eq!`/`relative_eq!` call this crate's tests introduce is
//! counted here from the start, not retrofitted later:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/cspace-core/src/sampling/*.rs
//! both=0 epsilon_only=6 max_relative_only=0 neither=0
//! ```
//!
//! Run for real against the tree as committed this round (not fabricated --
//! see PORTING-PLAN.md §117.5, never cite an unreproduced number). All six
//! are `epsilon`-only: the four empirical-mean/variance assertions in
//! `empirical_mean_and_variance_converge_over_many_samples` and the two
//! correlation assertions in
//! `without_covariance_ignores_correlation_with_covariance_does_not`, every
//! epsilon justified at its own call site against the estimator's standard
//! error over the sample count used -- see the tolerance-floor paragraph
//! below.
//!
//! **Tolerance-floor re-measurement.** All three epsilons this crate's
//! tests use (0.15 for empirical mean, 0.5/0.9 for empirical variance) are
//! sized from the standard error of the estimator over the sample count
//! used (documented at each call site), measured against this workspace's
//! current `float_roundtrip`-fixed fixture floor (commit `70a6b31`) -- none
//! of these constants were carried over from a pre-fix measurement, since
//! this crate and its tests are new this round.

mod multivariate_gaussian;

pub use multivariate_gaussian::MultivariateGaussian;
