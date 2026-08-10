// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_validator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/joint_limits_validator.cpp

//! Whether every joint in a [`JointLimitsContainer`] carries the *same*
//! limit, checked one dimension at a time.
//!
//! Upstream `JointLimitsValidator`, a class with four public static methods
//! and no state; here four free functions in a module, since there is no
//! object to be a method of.
//!
//! Each function answers one question — do all joints agree on position /
//! velocity / acceleration / deceleration? — and each returns `true`
//! vacuously for a container of 0 or 1 joints, exactly as upstream's own
//! `@note` on each declaration says.
//!
//! "Agree" is a two-part test, and both parts matter:
//!
//! 1. every joint's `has_*_limits` flag is the same, and
//! 2. *if* that flag is set, every joint's value is the same.
//!
//! So two joints that both leave a dimension unlimited agree on it however
//! far apart the (meaningless) numbers behind the flag are, and two joints
//! that both limit it disagree the moment the numbers differ.
//!
//! # Deviations from upstream
//!
//! - **The empty case is structural, not a guard.**
//!   `validateWithEqualFunc` opens with `if (joint_limits.empty()) return
//!   true;` before taking `joint_limits.begin()->second` as its reference.
//!   `validate_with` takes the reference from the iterator itself, so an
//!   empty container yields `None` and the answer is `true` with no branch
//!   to get wrong — and a one-element container is a zero-iteration loop
//!   rather than a second special case. Same answers, one fewer boundary.
//! - **`ValidationException` and its three subclasses are not ported.**
//!   `joint_limits_validator.hpp:104-151` declares `ValidationException`,
//!   `ValidationJointMissingException`, `ValidationDifferentLimitsException`
//!   and `ValidationBoundsViolationException`. Measured against the whole
//!   upstream checkout, **no site constructs, throws or catches any of the
//!   four** (`rg -n 'Validation(Exception|JointMissingException|
//!   DifferentLimitsException|BoundsViolationException)' moveit_planners/
//!   moveit_core/ moveit_ros/` -> 12 lines, every one of them inside this
//!   header's own declarations). Nothing in this file's implementation can
//!   throw: all four public functions return `bool`. There is no behaviour
//!   to port.
//! - **Float comparison stays `!=`, NaN and all.** Upstream compares
//!   `max_velocity != rhs.max_velocity` directly, so a dimension whose flag
//!   is set while its value was never filled in (upstream's
//!   `joint_limits::JointLimits` constructor leaves every number `NaN`;
//!   [`JointLimit::default`] matches it) reports *unequal* against another
//!   joint in exactly the same state, since `NaN != NaN`. Rust's `f64` `!=`
//!   is the same IEEE-754 comparison, so transcribing it preserves that;
//!   see `partially_specified_limits_are_never_equal_to_each_other`.

use crate::pilz::limits::{JointLimit, JointLimitsContainer};

/// Whether every joint's position limit is the same.
///
/// Upstream `validateAllPositionLimitsEqual`. `true` for a container of 0
/// or 1 joints, and for any container in which no joint sets
/// `has_position_limits`.
pub fn validate_all_position_limits_equal(joint_limits: &JointLimitsContainer) -> bool {
    validate_with(position_equal, joint_limits)
}

/// Whether every joint's velocity limit is the same.
///
/// Upstream `validateAllVelocityLimitsEqual`. `true` for a container of 0
/// or 1 joints, and for any container in which no joint sets
/// `has_velocity_limits`.
pub fn validate_all_velocity_limits_equal(joint_limits: &JointLimitsContainer) -> bool {
    validate_with(velocity_equal, joint_limits)
}

/// Whether every joint's acceleration limit is the same.
///
/// Upstream `validateAllAccelerationLimitsEqual`. `true` for a container of
/// 0 or 1 joints, and for any container in which no joint sets
/// `has_acceleration_limits`.
pub fn validate_all_acceleration_limits_equal(joint_limits: &JointLimitsContainer) -> bool {
    validate_with(acceleration_equal, joint_limits)
}

/// Whether every joint's deceleration limit is the same.
///
/// Upstream `validateAllDecelerationLimitsEqual`. `true` for a container of
/// 0 or 1 joints, and for any container in which no joint sets
/// `has_deceleration_limits`.
pub fn validate_all_deceleration_limits_equal(joint_limits: &JointLimitsContainer) -> bool {
    validate_with(deceleration_equal, joint_limits)
}

/// Upstream `validateWithEqualFunc`: compare every joint against the first
/// one in name order.
///
/// Comparing against a fixed reference rather than pairwise is upstream's
/// shape and needs no defence: each `*_equal` below is an equivalence
/// relation (flag equality conjoined with a value equality that is only
/// consulted when the shared flag is set), so "all equal to the first" and
/// "all equal to each other" are the same predicate.
fn validate_with(
    eq_func: fn(&JointLimit, &JointLimit) -> bool,
    joint_limits: &JointLimitsContainer,
) -> bool {
    let mut limits = joint_limits.iter().map(|(_, limit)| limit);
    match limits.next() {
        None => true,
        Some(reference) => limits.all(|limit| eq_func(reference, limit)),
    }
}

/// Upstream `positionEqual`.
fn position_equal(lhs: &JointLimit, rhs: &JointLimit) -> bool {
    if lhs.has_position_limits != rhs.has_position_limits {
        return false;
    }
    if lhs.has_position_limits
        && (lhs.max_position != rhs.max_position || lhs.min_position != rhs.min_position)
    {
        return false;
    }
    true
}

/// Upstream `velocityEqual`.
fn velocity_equal(lhs: &JointLimit, rhs: &JointLimit) -> bool {
    if lhs.has_velocity_limits != rhs.has_velocity_limits {
        return false;
    }
    if lhs.has_velocity_limits && lhs.max_velocity != rhs.max_velocity {
        return false;
    }
    true
}

/// Upstream `accelerationEqual`.
fn acceleration_equal(lhs: &JointLimit, rhs: &JointLimit) -> bool {
    if lhs.has_acceleration_limits != rhs.has_acceleration_limits {
        return false;
    }
    if lhs.has_acceleration_limits && lhs.max_acceleration != rhs.max_acceleration {
        return false;
    }
    true
}

/// Upstream `decelerationEqual`.
fn deceleration_equal(lhs: &JointLimit, rhs: &JointLimit) -> bool {
    if lhs.has_deceleration_limits != rhs.has_deceleration_limits {
        return false;
    }
    if lhs.has_deceleration_limits && lhs.max_deceleration != rhs.max_deceleration {
        return false;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The four answers for one container, in `[position, velocity,
    /// acceleration, deceleration]` order.
    ///
    /// Every test below asserts all four at once, not just the family it is
    /// about. That is what makes each case a statement of *independence*:
    /// a disagreement in one dimension must leave the other three saying
    /// `true`, which is the property upstream's own test file asserts on
    /// every one of its cases.
    fn verdicts(container: &JointLimitsContainer) -> [bool; 4] {
        [
            validate_all_position_limits_equal(container),
            validate_all_velocity_limits_equal(container),
            validate_all_acceleration_limits_equal(container),
            validate_all_deceleration_limits_equal(container),
        ]
    }

    /// Build a container from `(name, limit)` pairs, asserting every insert
    /// lands — [`JointLimitsContainer::add_limit`] silently rejects a
    /// non-negative `max_deceleration`, and a test whose fixture was
    /// rejected would be measuring an empty container instead of the case
    /// it names.
    fn container(limits: &[(&str, JointLimit)]) -> JointLimitsContainer {
        let mut container = JointLimitsContainer::default();
        for (name, limit) in limits {
            assert!(
                container.add_limit(*name, *limit),
                "fixture limit for {name} was rejected by add_limit"
            );
        }
        assert_eq!(container.count(), limits.len());
        container
    }

    // -- Boundary: container size ------------------------------------------

    /// Boundary: zero joints. Upstream's `@note` on all four declarations
    /// promises `true` here, and its `validateWithEqualFunc` has an
    /// explicit `empty()` guard for it; this port has none, so the case is
    /// worth pinning.
    #[test]
    fn an_empty_container_agrees_on_every_dimension() {
        assert_eq!(verdicts(&container(&[])), [true; 4]);
    }

    /// Boundary: exactly one joint — the case where "all joints agree" has
    /// nothing to compare against. Every dimension is set, and set to a
    /// value no second joint could match, so a `true` here can only come
    /// from the loop being empty.
    #[test]
    fn a_single_joint_agrees_with_itself_on_every_dimension() {
        let only = JointLimit {
            has_position_limits: true,
            min_position: -3.0,
            max_position: 7.0,
            has_velocity_limits: true,
            max_velocity: 11.0,
            has_acceleration_limits: true,
            max_acceleration: 13.0,
            has_deceleration_limits: true,
            max_deceleration: -17.0,
            ..Default::default()
        };
        assert_eq!(verdicts(&container(&[("joint1", only)])), [true; 4]);
    }

    /// Boundary: the disagreement is between the *first* joint and the
    /// *third*, with the first two agreeing. An implementation that
    /// compared only the first pair — or that stopped at the first
    /// agreement — would call this container equal.
    #[test]
    fn a_third_joint_disagreeing_with_the_first_two_is_still_a_disagreement() {
        let agreeing = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let odd_one_out = JointLimit {
            max_position: 2.0,
            ..agreeing
        };
        let container = container(&[
            ("joint1", agreeing),
            ("joint2", agreeing),
            ("joint3", odd_one_out),
        ]);
        assert_eq!(verdicts(&container), [false, true, true, true]);
    }

    // -- Boundary: position -------------------------------------------------

    #[test]
    fn position_flags_that_differ_are_a_disagreement() {
        let limited = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let unlimited = JointLimit {
            has_position_limits: false,
            ..limited
        };
        let container = container(&[("joint1", limited), ("joint2", unlimited)]);
        assert_eq!(verdicts(&container), [false, true, true, true]);
    }

    #[test]
    fn a_differing_min_position_is_a_disagreement() {
        let lhs = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            min_position: -2.0,
            ..lhs
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [false, true, true, true]);
    }

    #[test]
    fn a_differing_max_position_is_a_disagreement() {
        let lhs = JointLimit {
            has_position_limits: true,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            max_position: 2.0,
            ..lhs
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [false, true, true, true]);
    }

    /// Boundary: partially-specified limits. Both joints leave position
    /// *unlimited* while carrying different numbers in the fields behind
    /// the flag. Upstream reads those fields only under
    /// `if (lhs.has_position_limits)`, so the numbers must not be
    /// consulted — this is the case that fails if the flag check is
    /// dropped.
    #[test]
    fn position_values_behind_a_clear_flag_are_not_compared() {
        let lhs = JointLimit {
            has_position_limits: false,
            min_position: -1.0,
            max_position: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            has_position_limits: false,
            min_position: -99.0,
            max_position: 99.0,
            ..Default::default()
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true; 4]);
    }

    // -- Boundary: velocity -------------------------------------------------

    #[test]
    fn velocity_flags_that_differ_are_a_disagreement() {
        let limited = JointLimit {
            has_velocity_limits: true,
            max_velocity: 1.0,
            ..Default::default()
        };
        let unlimited = JointLimit {
            has_velocity_limits: false,
            ..limited
        };
        let container = container(&[("joint1", limited), ("joint2", unlimited)]);
        assert_eq!(verdicts(&container), [true, false, true, true]);
    }

    #[test]
    fn a_differing_max_velocity_is_a_disagreement() {
        let lhs = JointLimit {
            has_velocity_limits: true,
            max_velocity: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            max_velocity: 2.0,
            ..lhs
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true, false, true, true]);
    }

    #[test]
    fn velocity_values_behind_a_clear_flag_are_not_compared() {
        let lhs = JointLimit {
            has_velocity_limits: false,
            max_velocity: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            has_velocity_limits: false,
            max_velocity: 99.0,
            ..Default::default()
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true; 4]);
    }

    // -- Boundary: acceleration ---------------------------------------------

    #[test]
    fn acceleration_flags_that_differ_are_a_disagreement() {
        let limited = JointLimit {
            has_acceleration_limits: true,
            max_acceleration: 1.0,
            ..Default::default()
        };
        let unlimited = JointLimit {
            has_acceleration_limits: false,
            ..limited
        };
        let container = container(&[("joint1", limited), ("joint2", unlimited)]);
        assert_eq!(verdicts(&container), [true, true, false, true]);
    }

    #[test]
    fn a_differing_max_acceleration_is_a_disagreement() {
        let lhs = JointLimit {
            has_acceleration_limits: true,
            max_acceleration: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            max_acceleration: 2.0,
            ..lhs
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true, true, false, true]);
    }

    #[test]
    fn acceleration_values_behind_a_clear_flag_are_not_compared() {
        let lhs = JointLimit {
            has_acceleration_limits: false,
            max_acceleration: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            has_acceleration_limits: false,
            max_acceleration: 99.0,
            ..Default::default()
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true; 4]);
    }

    // -- Boundary: deceleration ---------------------------------------------

    #[test]
    fn deceleration_flags_that_differ_are_a_disagreement() {
        let limited = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: -1.0,
            ..Default::default()
        };
        let unlimited = JointLimit {
            has_deceleration_limits: false,
            ..limited
        };
        let container = container(&[("joint1", limited), ("joint2", unlimited)]);
        assert_eq!(verdicts(&container), [true, true, true, false]);
    }

    #[test]
    fn a_differing_max_deceleration_is_a_disagreement() {
        let lhs = JointLimit {
            has_deceleration_limits: true,
            max_deceleration: -1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            max_deceleration: -2.0,
            ..lhs
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true, true, true, false]);
    }

    /// Boundary: deceleration is the one dimension whose "behind a clear
    /// flag" case can carry a *positive* number —
    /// [`JointLimitsContainer::add_limit`]'s negativity rule is gated on
    /// `has_deceleration_limits`, so with the flag clear the container
    /// accepts a value the validator must then ignore.
    #[test]
    fn deceleration_values_behind_a_clear_flag_are_not_compared() {
        let lhs = JointLimit {
            has_deceleration_limits: false,
            max_deceleration: 1.0,
            ..Default::default()
        };
        let rhs = JointLimit {
            has_deceleration_limits: false,
            max_deceleration: 99.0,
            ..Default::default()
        };
        let container = container(&[("joint1", lhs), ("joint2", rhs)]);
        assert_eq!(verdicts(&container), [true; 4]);
    }

    // -- Boundary: a flag set over a value that was never filled in ---------

    /// Boundary: `has_*_limits` set while the value behind it is still
    /// [`JointLimit::default`]'s `NaN` — the shape a partially-written
    /// limits specification produces (upstream's parameter path declares
    /// every numeric limit with a `quiet_NaN` default and only overwrites
    /// the ones actually given).
    ///
    /// Two joints in *identical* such states report **unequal**, because
    /// `NaN != NaN`. That is upstream's behaviour, not an accident of this
    /// port: `positionEqual` and friends compare with `!=` and nothing
    /// guards against `NaN`. Position and velocity are asserted here;
    /// acceleration and deceleration are `true` in this fixture only
    /// because their flags are clear, which is what makes the two `false`s
    /// attributable to the `NaN` rather than to the flags.
    #[test]
    fn partially_specified_limits_are_never_equal_to_each_other() {
        let partial = JointLimit {
            has_position_limits: true,
            has_velocity_limits: true,
            ..Default::default()
        };
        assert!(partial.min_position.is_nan());
        assert!(partial.max_position.is_nan());
        assert!(partial.max_velocity.is_nan());

        let container = container(&[("joint1", partial), ("joint2", partial)]);
        assert_eq!(verdicts(&container), [false, false, true, true]);
    }
}
