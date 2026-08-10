// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright 2020, PAL Robotics S.L.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_container.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_extension.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/joint_limits_copy/joint_limits.hpp  (Apache-2.0, PAL Robotics; vendored upstream)
//   moveit_planners/pilz_industrial_motion_planner/src/joint_limits_container.cpp
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/limits_container.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/limits_container.cpp
//   moveit_planners/pilz_industrial_motion_planner/src/cartesian_limits_parameters.yaml

//! Per-joint and Cartesian motion limits ([`JointLimit`],
//! [`JointLimitsContainer`], [`CartesianLimits`], [`LimitsContainer`]),
//! ported from upstream's `JointLimitsContainer` and `LimitsContainer`.
//!
//! [`JointLimit`] folds together two upstream types: `joint_limits::JointLimits`
//! (vendored from a `ros2_control` draft PR into
//! `include/joint_limits_copy/joint_limits.hpp`, Apache-2.0) and
//! `pilz_industrial_motion_planner::joint_limits_interface::JointLimits`,
//! which extends it with `max_deceleration`/`has_deceleration_limits`
//! (deceleration is stored as a *negative* number by upstream convention;
//! [`JointLimitsContainer::add_limit`] rejects a non-negative one).
//!
//! # Deviations from upstream
//!
//! - **Logging is dropped, not swapped.** `JointLimitsContainer::addLimit`'s
//!   two `RCLCPP_ERROR_STREAM` calls only restate, as text, the failure the
//!   `bool` return already reports; [`JointLimitsContainer::add_limit`] keeps
//!   the same boolean contract with no logging side effect, so there is no
//!   information the caller loses.
//! - **`printCartesianLimits()`'s `RCLCPP_DEBUG` becomes `impl Display for
//!   CartesianLimits`.** Unlike `addLimit`'s logging, this method carries no
//!   redundant return value — formatting *is* its entire job — so the native
//!   replacement is a native formatting impl a caller can feed into whatever
//!   logging this port ends up using, rather than a hard-wired `rclcpp`
//!   macro.
//! - **`JointLimit::to_string()`/`debug_to_string()` are not ported.** Same
//!   reasoning as `VelocityProfileAtrap::Write` in [`crate::pilz::velocity_profile`]:
//!   pure formatting helpers, unexercised by any computation this crate
//!   ports; `#[derive(Debug)]` covers the same debugging need.
//! - **`LimitsContainer::has_cartesian_limits()` is added.** Upstream defines
//!   `has_cartesian_limits_` and sets it in `setCartesianLimits`, but never
//!   exposes a getter for it (`hasJointLimits()` has one, its Cartesian
//!   counterpart does not — an upstream asymmetry, not a deliberate
//!   omission). A field this port writes but can never read is dead code
//!   under this workspace's `deny(warnings)`, so the accessor is added,
//!   mirroring [`LimitsContainer::has_joint_limits`]'s shape.
//! - **`getLimit`/`getCommonLimit(joint_names)`'s `std::out_of_range` becomes
//!   [`cspace_core::error::Error::UnknownName`].** Matches this crate's house error
//!   convention; see `cspace_core::error`.
//! - **Lookups are single-pass.** Upstream's `verify*Limit` methods call
//!   `hasLimit()` then `getLimit()`, walking the map twice; the port uses one
//!   `BTreeMap::get`. Pure implementation detail, not a behaviour change.

use std::collections::BTreeMap;
use std::fmt;

use cspace_core::error::{Error, Result};

use crate::pilz::numeric::{cxx_max, cxx_min};

/// A single joint's position/velocity/acceleration/deceleration/jerk/effort
/// limits.
///
/// Upstream: `joint_limits::JointLimits` (vendored, Apache-2.0) extended by
/// `pilz_industrial_motion_planner::joint_limits_interface::JointLimits`
/// (deceleration fields). See the [module docs](self) for why they are one
/// type here.
///
/// Every `has_*_limits` flag defaults to `false`; the corresponding numeric
/// field defaults to `NaN`, matching upstream's constructor, so a limit that
/// was never set can never be silently mistaken for a limit of `0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimit {
    /// Lower position bound. Meaningful only if [`Self::has_position_limits`].
    pub min_position: f64,
    /// Upper position bound. Meaningful only if [`Self::has_position_limits`].
    pub max_position: f64,
    /// Velocity bound (symmetric: `|v| <= max_velocity`).
    pub max_velocity: f64,
    /// Acceleration bound (symmetric: `|a| <= max_acceleration`).
    pub max_acceleration: f64,
    /// Jerk bound.
    pub max_jerk: f64,
    /// Effort bound.
    pub max_effort: f64,
    /// Deceleration bound. MUST be negative when [`Self::has_deceleration_limits`]
    /// (upstream convention; enforced by [`JointLimitsContainer::add_limit`]).
    pub max_deceleration: f64,

    /// Whether [`Self::min_position`]/[`Self::max_position`] are meaningful.
    pub has_position_limits: bool,
    /// Whether [`Self::max_velocity`] is meaningful.
    pub has_velocity_limits: bool,
    /// Whether [`Self::max_acceleration`] is meaningful.
    pub has_acceleration_limits: bool,
    /// Whether [`Self::max_jerk`] is meaningful.
    pub has_jerk_limits: bool,
    /// Whether [`Self::max_effort`] is meaningful.
    pub has_effort_limits: bool,
    /// Whether [`Self::max_deceleration`] is meaningful.
    pub has_deceleration_limits: bool,
    /// Whether this joint wraps around (continuous revolute).
    pub angle_wraparound: bool,
}

impl Default for JointLimit {
    fn default() -> Self {
        Self {
            min_position: f64::NAN,
            max_position: f64::NAN,
            max_velocity: f64::NAN,
            max_acceleration: f64::NAN,
            max_jerk: f64::NAN,
            max_effort: f64::NAN,
            max_deceleration: 0.0,
            has_position_limits: false,
            has_velocity_limits: false,
            has_acceleration_limits: false,
            has_jerk_limits: false,
            has_effort_limits: false,
            has_deceleration_limits: false,
            angle_wraparound: false,
        }
    }
}

/// A named collection of [`JointLimit`]s with fusion (most-restrictive-
/// envelope) and per-dimension boundary checks.
///
/// Upstream: `JointLimitsContainer`. Backed by a [`BTreeMap`] rather than
/// `std::map` for the same reason as `cspace_core::geometry::Transforms`: matching
/// ordering.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct JointLimitsContainer {
    container: BTreeMap<String, JointLimit>,
}

impl JointLimitsContainer {
    /// Add a limit for `joint_name`.
    ///
    /// Returns `false` (and leaves the container unchanged) if
    /// `joint_limit.has_deceleration_limits` but `max_deceleration >= 0`, or
    /// if `joint_name` is already present.
    pub fn add_limit(&mut self, joint_name: impl Into<String>, joint_limit: JointLimit) -> bool {
        if joint_limit.has_deceleration_limits && joint_limit.max_deceleration >= 0.0 {
            return false;
        }
        let joint_name = joint_name.into();
        if self.container.contains_key(&joint_name) {
            return false;
        }
        self.container.insert(joint_name, joint_limit);
        true
    }

    /// Whether a limit for `joint_name` is present.
    pub fn has_limit(&self, joint_name: &str) -> bool {
        self.container.contains_key(joint_name)
    }

    /// Number of limits in the container.
    pub fn count(&self) -> usize {
        self.container.len()
    }

    /// Whether the container has no limits.
    pub fn is_empty(&self) -> bool {
        self.container.is_empty()
    }

    /// Fuse every limit in the container into the single most-restrictive
    /// envelope: position narrows to the tightest `[min, max]`, velocity/
    /// acceleration take the smallest maximum, deceleration takes the
    /// smallest magnitude (largest, since it is stored negative). A
    /// dimension with no limit set on any joint keeps its `has_*_limits`
    /// flag `false`.
    pub fn common_limit(&self) -> JointLimit {
        let mut common = JointLimit::default();
        for limit in self.container.values() {
            update_common_limit(limit, &mut common);
        }
        common
    }

    /// Same as [`Self::common_limit`], fused over only `joint_names`.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any name in `joint_names` has no limit in
    /// this container.
    pub fn common_limit_for(&self, joint_names: &[String]) -> Result<JointLimit> {
        let mut common = JointLimit::default();
        for joint_name in joint_names {
            update_common_limit(&self.limit(joint_name)?, &mut common);
        }
        Ok(common)
    }

    /// The limit for `joint_name`.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `joint_name` has no limit in this container.
    pub fn limit(&self, joint_name: &str) -> Result<JointLimit> {
        self.container
            .get(joint_name)
            .copied()
            .ok_or_else(|| Error::unknown_name("joint", joint_name))
    }

    /// Iterate over every `(joint_name, limit)` pair, in name order.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &JointLimit)> {
        self.container.iter()
    }

    /// Whether `joint_position` is within `joint_name`'s position limit.
    /// Vacuously `true` if `joint_name` is unknown or has no position limit.
    pub fn verify_position_limit(&self, joint_name: &str, joint_position: f64) -> bool {
        match self.container.get(joint_name) {
            None => true,
            Some(limit) => {
                !limit.has_position_limits
                    || (joint_position >= limit.min_position
                        && joint_position <= limit.max_position)
            }
        }
    }

    /// Whether `joint_velocity` is within `joint_name`'s velocity limit.
    /// Vacuously `true` if `joint_name` is unknown or has no velocity limit.
    pub fn verify_velocity_limit(&self, joint_name: &str, joint_velocity: f64) -> bool {
        match self.container.get(joint_name) {
            None => true,
            Some(limit) => !limit.has_velocity_limits || joint_velocity.abs() <= limit.max_velocity,
        }
    }

    /// Whether `joint_acceleration` is within `joint_name`'s acceleration
    /// limit. Vacuously `true` if `joint_name` is unknown or has no
    /// acceleration limit.
    pub fn verify_acceleration_limit(&self, joint_name: &str, joint_acceleration: f64) -> bool {
        match self.container.get(joint_name) {
            None => true,
            Some(limit) => {
                !limit.has_acceleration_limits || joint_acceleration.abs() <= limit.max_acceleration
            }
        }
    }

    /// Whether `joint_acceleration` is within `joint_name`'s deceleration
    /// limit (compared by magnitude, since `max_deceleration` is stored
    /// negative). Vacuously `true` if `joint_name` is unknown or has no
    /// deceleration limit.
    pub fn verify_deceleration_limit(&self, joint_name: &str, joint_acceleration: f64) -> bool {
        match self.container.get(joint_name) {
            None => true,
            Some(limit) => {
                !limit.has_deceleration_limits
                    || joint_acceleration.abs() <= -limit.max_deceleration
            }
        }
    }
}

/// Upstream `JointLimitsContainer::updateCommonLimit`.
///
/// The fusion arms use [`cxx_max`]/[`cxx_min`], not [`f64::max`]/[`f64::min`]:
/// upstream's `std::max(common_limit.X, X)`/`std::min(common_limit.X, X)`
/// return a NaN `common_limit.X` (the running fusion, already in the first
/// argument position) rather than discarding it in favor of the next
/// joint's finite bound — see this crate's `numeric` module.
fn update_common_limit(joint_limit: &JointLimit, common_limit: &mut JointLimit) {
    if joint_limit.has_position_limits {
        common_limit.min_position = if !common_limit.has_position_limits {
            joint_limit.min_position
        } else {
            cxx_max(common_limit.min_position, joint_limit.min_position)
        };
        common_limit.max_position = if !common_limit.has_position_limits {
            joint_limit.max_position
        } else {
            cxx_min(common_limit.max_position, joint_limit.max_position)
        };
        common_limit.has_position_limits = true;
    }

    if joint_limit.has_velocity_limits {
        common_limit.max_velocity = if !common_limit.has_velocity_limits {
            joint_limit.max_velocity
        } else {
            cxx_min(common_limit.max_velocity, joint_limit.max_velocity)
        };
        common_limit.has_velocity_limits = true;
    }

    if joint_limit.has_acceleration_limits {
        common_limit.max_acceleration = if !common_limit.has_acceleration_limits {
            joint_limit.max_acceleration
        } else {
            cxx_min(common_limit.max_acceleration, joint_limit.max_acceleration)
        };
        common_limit.has_acceleration_limits = true;
    }

    if joint_limit.has_deceleration_limits {
        common_limit.max_deceleration = if !common_limit.has_deceleration_limits {
            joint_limit.max_deceleration
        } else {
            cxx_max(common_limit.max_deceleration, joint_limit.max_deceleration)
        };
        common_limit.has_deceleration_limits = true;
    }
}

/// Cartesian motion limits: max translational velocity/acceleration/
/// deceleration and max rotational velocity.
///
/// Upstream: `cartesian_limits::Params`, generated by
/// `generate_parameter_library` from `cartesian_limits_parameters.yaml`. All
/// fields default to `0.0`, matching the generated struct.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CartesianLimits {
    /// Max translational velocity.
    pub max_trans_vel: f64,
    /// Max translational acceleration.
    pub max_trans_acc: f64,
    /// Max translational deceleration.
    pub max_trans_dec: f64,
    /// Max rotational velocity.
    pub max_rot_vel: f64,
}

impl fmt::Display for CartesianLimits {
    /// Matches the text upstream's `printCartesianLimits()` logs at debug
    /// level; see the [module docs](self) for why this replaces
    /// `RCLCPP_DEBUG` rather than a log call.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Pilz Cartesian Limits - Max Trans Vel : {}, Max Trans Acc : {}, Max Trans Dec : {}, Max Rot Vel : {}",
            self.max_trans_vel, self.max_trans_acc, self.max_trans_dec, self.max_rot_vel
        )
    }
}

/// Combines [`JointLimitsContainer`] and [`CartesianLimits`], tracking
/// whether each was ever explicitly set.
///
/// Upstream: `LimitsContainer`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LimitsContainer {
    has_joint_limits: bool,
    joint_limits: JointLimitsContainer,
    has_cartesian_limits: bool,
    cartesian_limits: CartesianLimits,
}

impl LimitsContainer {
    /// An empty container: no joint or Cartesian limits set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether [`Self::set_joint_limits`] has been called.
    pub fn has_joint_limits(&self) -> bool {
        self.has_joint_limits
    }

    /// Set the joint limits.
    pub fn set_joint_limits(&mut self, joint_limits: JointLimitsContainer) {
        self.has_joint_limits = true;
        self.joint_limits = joint_limits;
    }

    /// The joint limits (empty if [`Self::set_joint_limits`] was never
    /// called).
    pub fn joint_limits(&self) -> &JointLimitsContainer {
        &self.joint_limits
    }

    /// Whether [`Self::set_cartesian_limits`] has been called. See the
    /// [module docs](self) for why this accessor exists but has no upstream
    /// counterpart.
    pub fn has_cartesian_limits(&self) -> bool {
        self.has_cartesian_limits
    }

    /// Set the Cartesian limits.
    pub fn set_cartesian_limits(&mut self, cartesian_limits: CartesianLimits) {
        self.has_cartesian_limits = true;
        self.cartesian_limits = cartesian_limits;
    }

    /// The Cartesian limits (all-zero if [`Self::set_cartesian_limits`] was
    /// never called).
    pub fn cartesian_limits(&self) -> &CartesianLimits {
        &self.cartesian_limits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Six joints, each contributing a different limit dimension, fused into
    /// one common envelope. Golden values transcribed from upstream's
    /// `JointLimitsContainerTest` fixture.
    fn fixture() -> JointLimitsContainer {
        let mut container = JointLimitsContainer::default();

        let lim1 = JointLimit {
            has_position_limits: true,
            min_position: -2.0,
            max_position: 2.0,
            has_acceleration_limits: true,
            max_acceleration: 3.0, // expected common max_acceleration
            ..Default::default()
        };

        let lim2 = JointLimit {
            has_position_limits: true,
            min_position: -1.0, // expected common min_position
            max_position: 1.0,  // expected common max_position
            has_deceleration_limits: true,
            max_deceleration: -5.0, // expected common max_deceleration
            ..Default::default()
        };

        let lim3 = JointLimit {
            has_velocity_limits: true,
            max_velocity: 10.0,
            ..Default::default()
        };

        let lim4 = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            has_acceleration_limits: true,
            max_acceleration: 400.0,
            ..Default::default()
        };

        let lim5 = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };

        let lim6 = JointLimit {
            has_velocity_limits: true,
            max_velocity: 2.0, // expected common max_velocity
            has_deceleration_limits: true,
            max_deceleration: -100.0,
            ..Default::default()
        };

        assert!(container.add_limit("joint1", lim1));
        assert!(container.add_limit("joint2", lim2));
        assert!(container.add_limit("joint3", lim3));
        assert!(container.add_limit("joint4", lim4));
        assert!(container.add_limit("joint5", lim5));
        assert!(container.add_limit("joint6", lim6));

        container
    }

    #[test]
    fn common_limit_position_is_the_narrowest_window() {
        let common = fixture().common_limit();
        assert_eq!(common.min_position, -1.0);
        assert_eq!(common.max_position, 1.0);
    }

    #[test]
    fn common_limit_velocity_is_the_smallest_maximum() {
        assert_eq!(fixture().common_limit().max_velocity, 2.0);
    }

    #[test]
    fn common_limit_acceleration_is_the_smallest_maximum() {
        assert_eq!(fixture().common_limit().max_acceleration, 3.0);
    }

    #[test]
    fn common_limit_deceleration_is_the_smallest_magnitude() {
        assert_eq!(fixture().common_limit().max_deceleration, -5.0);
    }

    // -- update_common_limit's std::min/std::max NaN semantics --
    //
    // BTreeMap iteration is by joint name, so "a"/"b" fixes which limit is
    // already in `common_limit` (upstream's std::min/std::max first
    // argument) when the second joint is fused: "a" is always processed
    // first, seeding `common_limit` directly (the `if
    // !common_limit.has_*_limits` branch); "b" is fused against it via the
    // `.min()`/`.max()` call under test. Upstream's std::min(a, b) / std::max
    // (a, b) return a NaN `a` and discard a NaN `b` -- see this crate's
    // `numeric` module.

    #[test]
    fn common_limit_min_position_keeps_a_nan_first_bound_not_the_second_joints_finite_one() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_position_limits: true,
                min_position: f64::NAN,
                max_position: 1.0,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_position_limits: true,
                min_position: -1.0,
                max_position: 1.0,
                ..Default::default()
            },
        );
        assert!(
            container.common_limit().min_position.is_nan(),
            "a NaN min_position on the first-fused joint must survive the fusion, not be \
             silently replaced by the second joint's finite bound"
        );
    }

    #[test]
    fn common_limit_max_position_keeps_a_nan_first_bound_not_the_second_joints_finite_one() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_position_limits: true,
                min_position: -1.0,
                max_position: f64::NAN,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_position_limits: true,
                min_position: -1.0,
                max_position: 1.0,
                ..Default::default()
            },
        );
        assert!(
            container.common_limit().max_position.is_nan(),
            "a NaN max_position on the first-fused joint must survive the fusion"
        );
    }

    #[test]
    fn common_limit_max_velocity_keeps_a_nan_first_bound_not_the_second_joints_finite_one() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: f64::NAN,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 2.0,
                ..Default::default()
            },
        );
        assert!(
            container.common_limit().max_velocity.is_nan(),
            "a NaN max_velocity on the first-fused joint must survive the fusion"
        );
    }

    /// The demonstrated opposite for [`common_limit_max_velocity_keeps_a_nan_first_bound_not_the_second_joints_finite_one`]:
    /// the same NaN on the *non-diverging* (second-fused) joint must still
    /// be discarded, so the finite answer comes out -- a fix that always
    /// propagates NaN regardless of which argument carries it would fail
    /// this test.
    #[test]
    fn common_limit_max_velocity_discards_a_nan_second_bound() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 2.0,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: f64::NAN,
                ..Default::default()
            },
        );
        assert_eq!(
            container.common_limit().max_velocity,
            2.0,
            "a NaN max_velocity on the second-fused joint must be discarded, matching \
             std::min(2.0, NaN) == 2.0"
        );
    }

    #[test]
    fn common_limit_max_acceleration_keeps_a_nan_first_bound_not_the_second_joints_finite_one() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: f64::NAN,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: 3.0,
                ..Default::default()
            },
        );
        assert!(
            container.common_limit().max_acceleration.is_nan(),
            "a NaN max_acceleration on the first-fused joint must survive the fusion"
        );
    }

    #[test]
    fn common_limit_max_deceleration_keeps_a_nan_first_bound_not_the_second_joints_finite_one() {
        let mut container = JointLimitsContainer::default();
        container.add_limit(
            "a",
            JointLimit {
                has_deceleration_limits: true,
                max_deceleration: f64::NAN,
                ..Default::default()
            },
        );
        container.add_limit(
            "b",
            JointLimit {
                has_deceleration_limits: true,
                max_deceleration: -5.0,
                ..Default::default()
            },
        );
        assert!(
            container.common_limit().max_deceleration.is_nan(),
            "a NaN max_deceleration on the first-fused joint must survive the fusion"
        );
    }

    /// Boundary: zero and positive deceleration are both rejected; only a
    /// strictly negative one is accepted.
    #[test]
    fn add_limit_rejects_non_negative_deceleration() {
        let zero_dec = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: 0.0,
            ..Default::default()
        };

        let positive_dec = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: 1.0,
            ..Default::default()
        };

        let negative_dec = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: -1.0,
            ..Default::default()
        };

        let mut container = JointLimitsContainer::default();
        assert!(!container.add_limit("joint_invalid1", zero_dec));
        assert!(!container.add_limit("joint_invalid2", positive_dec));
        assert!(container.add_limit("joint_valid", negative_dec));
    }

    #[test]
    fn add_limit_rejects_duplicate_joint_name() {
        let valid = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: -1.0,
            ..Default::default()
        };

        let mut container = JointLimitsContainer::default();
        assert!(container.add_limit("joint_valid", valid));
        assert!(!container.add_limit("joint_valid", valid));
    }

    #[test]
    fn empty_container_common_limit_has_no_flags_set() {
        let limits = JointLimitsContainer::default().common_limit();
        assert!(!limits.has_position_limits);
        assert!(!limits.has_velocity_limits);
        assert!(!limits.has_acceleration_limits);
        assert!(!limits.has_deceleration_limits);
    }

    /// Boundary: the first joint contributing no position limit does not
    /// prevent a later joint's limit from becoming the common one.
    #[test]
    fn common_limit_skips_joints_with_no_position_limit() {
        let lim1 = JointLimit::default();

        let lim2 = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };

        let mut container = JointLimitsContainer::default();
        container.add_limit("joint1", lim1);
        container.add_limit("joint2", lim2);

        let limits = container.common_limit();
        assert!(limits.has_position_limits);
        assert_eq!(limits.max_position, 1.0);
        assert_eq!(limits.min_position, -1.0);
    }

    /// ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` --
    /// `limit` has exactly one `Error::` site (`rg -c 'Error::'` scoped to
    /// its body: 1), reached whenever `joint_name` is absent from
    /// `self.container`.
    #[test]
    fn limit_of_unknown_joint_is_an_error() {
        let container = JointLimitsContainer::default();
        assert!(container.limit("nonexistent").is_err());
    }

    /// ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` --
    /// `common_limit_for`'s only fallible call is `self.limit(joint_name)?`
    /// in its loop, which is the same single `Error::` site `limit` has;
    /// the loop introduces no second cause, only a second call to it.
    #[test]
    fn common_limit_for_unknown_joint_is_an_error() {
        let container = fixture();
        assert!(
            container
                .common_limit_for(&["nonexistent".to_string()])
                .is_err()
        );
    }

    #[test]
    fn verify_limits_are_vacuously_true_when_unset() {
        let container = JointLimitsContainer::default();
        assert!(container.verify_position_limit("nonexistent", 1e9));
        assert!(container.verify_velocity_limit("nonexistent", 1e9));
        assert!(container.verify_acceleration_limit("nonexistent", 1e9));
        assert!(container.verify_deceleration_limit("nonexistent", 1e9));
    }

    #[test]
    fn verify_deceleration_limit_compares_by_magnitude() {
        let limit = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: -5.0,
            ..Default::default()
        };

        let mut container = JointLimitsContainer::default();
        container.add_limit("joint1", limit);

        assert!(container.verify_deceleration_limit("joint1", 5.0));
        assert!(container.verify_deceleration_limit("joint1", -5.0));
        assert!(!container.verify_deceleration_limit("joint1", 5.001));
    }

    #[test]
    fn cartesian_limits_default_to_zero() {
        let limits = CartesianLimits::default();
        assert_eq!(limits.max_trans_vel, 0.0);
        assert_eq!(limits.max_trans_acc, 0.0);
        assert_eq!(limits.max_trans_dec, 0.0);
        assert_eq!(limits.max_rot_vel, 0.0);
    }

    #[test]
    fn cartesian_limits_display_matches_upstream_format() {
        let limits = CartesianLimits {
            max_trans_vel: 1.0,
            max_trans_acc: 2.0,
            max_trans_dec: 3.0,
            max_rot_vel: 4.0,
        };
        assert_eq!(
            limits.to_string(),
            "Pilz Cartesian Limits - Max Trans Vel : 1, Max Trans Acc : 2, Max Trans Dec : 3, Max Rot Vel : 4"
        );
    }

    #[test]
    fn limits_container_tracks_whether_each_part_was_set() {
        let mut container = LimitsContainer::new();
        assert!(!container.has_joint_limits());
        assert!(!container.has_cartesian_limits());

        container.set_joint_limits(fixture());
        assert!(container.has_joint_limits());
        assert!(!container.has_cartesian_limits());

        container.set_cartesian_limits(CartesianLimits {
            max_trans_vel: 1.0,
            ..Default::default()
        });
        assert!(container.has_cartesian_limits());
        assert_eq!(container.cartesian_limits().max_trans_vel, 1.0);
    }
}
