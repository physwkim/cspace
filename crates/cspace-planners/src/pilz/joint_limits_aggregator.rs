// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/joint_limits_aggregator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/joint_limits_aggregator.cpp

//! Fusing a robot model's own joint bounds with an external override set
//! into one [`JointLimitsContainer`] ([`aggregate_limits`]).
//!
//! Upstream `JointLimitsAggregator`, a class with one public static method
//! and four protected helpers; here one public function and four private
//! ones, since there is no object to be a method of.
//!
//! The rules upstream's `getAggregatedLimits` doc states, unchanged here:
//!
//! 1. an override's position and velocity limits must be **stricter than or
//!    equal to** the model's own, or aggregation fails;
//! 2. an override whose `has_<position|velocity|acceleration|deceleration>_
//!    limits` flag is clear is *undefined* for that dimension, not "limited
//!    to whatever number is behind the flag" — the model's bound is used
//!    instead;
//! 3. not every joint has to be overridden; selective limitation works;
//! 4. an override that sets acceleration but not deceleration gets
//!    `max_deceleration = -max_acceleration`.
//!
//! And upstream's `@note`, which still holds: acceleration and deceleration
//! can *only* come from the override set. A URDF has no place to put them,
//! so `cspace_core::model::joint::VariableBounds`'s `acceleration_bounded` /
//! `max_acceleration` are never read here — same as upstream, which reads
//! only `position_bounded_`/`min_position_`/`max_position_` and
//! `velocity_bounded_`/`max_velocity_`.
//!
//! # The override set replaces upstream's `(node, param_namespace)`
//!
//! Upstream's signature is
//!
//! ```cpp
//! static JointLimitsContainer getAggregatedLimits(
//!     const rclcpp::Node::SharedPtr& node, const std::string& param_namespace,
//!     const std::vector<const moveit::core::JointModel*>& joint_models);
//! ```
//!
//! and the first two arguments exist only to reach a ROS parameter server.
//! `PORTING-PLAN.md` §224.2 records the measurement behind the replacement:
//! across `joint_limits_interface_extension.hpp` and
//! `joint_limits_copy/joint_limits_rosparam.hpp` — the two files that turn
//! `(node, param_namespace)` into limits — every use of `node` is
//! `declare_parameter` / `has_parameter` / `get_parameter` plus
//! `get_logger` / `get_name` for the log lines. No arithmetic, no
//! model access, no state. What the parameter server contributes to
//! aggregation is one thing: **a per-joint, partially-filled
//! `JointLimit`**.
//!
//! So this port takes that directly, as `overrides: &JointLimitsContainer`
//! — the same type aggregation returns, and the type
//! [`crate::pilz::limits`] already defines. The YAML file was always the real
//! input; the parameter server was its transport, and this port has no
//! parameter server to be a transport. `overrides.has_limit(name)` is
//! upstream's `getJointLimits(...)` returning `true`, and
//! `overrides.limit(name)` is its out-parameter.
//!
//! One consequence worth stating: [`JointLimitsContainer::add_limit`]
//! refuses a non-negative `max_deceleration`, so an override set *cannot*
//! be built holding one. Upstream's YAML path has no such gate, which is
//! why the defect in `# Deviations from upstream` below can exist there.
//!
//! # Deviations from upstream
//!
//! - **"No parameters for this joint" and "parameters with every flag
//!   clear" stop being two states.** Upstream branches on
//!   `getJointLimits(...)`'s `bool` (`joint_limits_aggregator.cpp:71`) and
//!   its `else` arm is `updatePositionLimitFromJointModel` +
//!   `updateVelocityLimitFromJointModel` (`:97-98`) — textually the same
//!   two calls the `if` arm makes when both `has_*` flags are clear
//!   (`:79`, `:88`), on a `joint_limit` that is still default-constructed.
//!   Reading the two arms against each other is what licenses
//!   `overrides.limit(name).unwrap_or_default()` here: it is an exact
//!   collapse, not an approximation, and it removes a dual meaning rather
//!   than papering over one.
//! - **`addLimit`'s `bool` is not discarded.**
//!   `joint_limits_aggregator.cpp:109` calls `container.addLimit(...)` and
//!   ignores the result, so a rejected joint silently vanishes from a
//!   container upstream's own `ExpectedMapSize` test then expects to be
//!   the same size as the joint list. Reachable: rule 4 above sets
//!   `max_deceleration = -max_acceleration`, and an override of
//!   `has_acceleration_limits: true, max_acceleration: 0.0` makes that
//!   `-0.0`, which `addLimit` rejects because `-0.0 >= 0.0`. Here the two
//!   rejection causes are separated — `has_limit` is asked first, so a
//!   `false` from `add_limit` can only be the deceleration rule — and each
//!   becomes its own [`AggregationError`]. See
//!   `doc/upstream-bugs.md`'s `aggregated-limits-drops-rejected-joint-silently`.
//! - **Multi-variable joints are rejected at the bounds check instead of
//!   being read past.** `checkPositionBoundsThrowing` passes
//!   `&joint_limit.min_position` — the address of one `double` member — to
//!   `satisfiesPositionBounds`, whose planar and floating overrides read
//!   `values[0..2]` and `values[0..6]`. For a multi-DOF joint upstream
//!   therefore compares the *next members of the struct* against position
//!   bounds. This port answers [`AggregationError::MultiDofBoundsCheck`]
//!   instead; see `doc/upstream-bugs.md`'s
//!   `check-position-bounds-multidof-adjacent-members`. The guard covers
//!   the velocity check too, for one uniform rule rather than a per-
//!   dimension one: `satisfiesVelocityBounds` iterates `bounds.size()` and
//!   so reads the same adjacent members. Joints with **zero** variables
//!   (fixed joints) are *not* rejected — `satisfies_position_bounds` and
//!   `satisfies_velocity_bounds` both answer `true` for them without
//!   reading `values`, exactly as upstream's `FixedJointModel` does.
//! - **One upstream exception class becomes three error variants.**
//!   `AggregationBoundsViolationException` is thrown from three sites with
//!   three different messages (`joint_limits_aggregator.cpp:174`, `:181`,
//!   `:192`). The variants keep the distinction the single class throws
//!   away, following [`crate::pilz::command_list_manager::SequenceError`]'s
//!   precedent. `AggregationException`, the abstract base, has no
//!   equivalent: nothing upstream throws or catches it.
//! - **The logging is dropped.** `RCLCPP_INFO_STREAM` naming the namespace
//!   being read, `RCLCPP_DEBUG_STREAM` echoing each position limit, and the
//!   four `RCLCPP_WARN_STREAM`s in the `LCOV_EXCL` arms of the two
//!   `update*FromJointModel` helpers are all `D1`. None carries information
//!   the return value does not: the warnings narrate branches whose effect
//!   on `joint_limit` the caller can read directly.

use cspace_core::model::joint::JointModel;

use crate::pilz::limits::{JointLimit, JointLimitsContainer};

/// Why a set of joint limits could not be aggregated.
///
/// The first three variants are upstream
/// `AggregationBoundsViolationException`, one per throw site; the last
/// three have no upstream counterpart because upstream does not detect
/// those conditions at all — see the module's `# Deviations from upstream`.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum AggregationError {
    /// The override's `min_position` is outside the model's position
    /// bounds. Upstream `joint_limits_aggregator.cpp:174`.
    #[error("min_position of {joint} violates min limit from URDF")]
    MinPositionViolation {
        /// The joint whose override was rejected.
        joint: String,
    },

    /// The override's `max_position` is outside the model's position
    /// bounds. Upstream `joint_limits_aggregator.cpp:181`.
    #[error("max_position of {joint} violates max limit from URDF")]
    MaxPositionViolation {
        /// The joint whose override was rejected.
        joint: String,
    },

    /// The override's `max_velocity` is outside the model's velocity
    /// bounds. Upstream `joint_limits_aggregator.cpp:192`.
    #[error("max_velocity of {joint} violates velocity limit from URDF")]
    MaxVelocityViolation {
        /// The joint whose override was rejected.
        joint: String,
    },

    /// An override set a position or velocity limit on a joint with more
    /// than one variable, which the bounds check does not support. No
    /// upstream counterpart — upstream reads adjacent struct members
    /// instead of refusing.
    #[error(
        "{joint} has {variable_count} variables; aggregation can bounds-check \
         single-variable joints only"
    )]
    MultiDofBoundsCheck {
        /// The joint whose override was rejected.
        joint: String,
        /// How many variables that joint has.
        variable_count: usize,
    },

    /// The same joint appeared twice in the aggregated joint list. No
    /// upstream counterpart — upstream discards the `addLimit` that
    /// rejects the second one.
    #[error("joint {joint} appears more than once in the joint list")]
    DuplicateJoint {
        /// The repeated joint.
        joint: String,
    },

    /// The aggregated limit ended up with a non-negative
    /// `max_deceleration`, which [`JointLimitsContainer::add_limit`]
    /// refuses. No upstream counterpart — upstream discards that rejection
    /// and drops the joint.
    #[error("max_deceleration of {joint} is {max_deceleration}, which is not negative")]
    NonNegativeDeceleration {
        /// The joint whose aggregated limit was rejected.
        joint: String,
        /// The offending value.
        max_deceleration: f64,
    },
}

/// Combine each joint model's own bounds with `overrides` into one
/// container.
///
/// Upstream `JointLimitsAggregator::getAggregatedLimits`. `joint_models`
/// is upstream's third argument; `overrides` replaces its first two — see
/// the [module docs](self) for the measurement behind that.
///
/// A joint absent from `overrides` takes its position and velocity limits
/// from the model and leaves acceleration and deceleration unset. A joint
/// present in `overrides` keeps every dimension the override defines, and
/// fills the two the model knows about from the model wherever the
/// override leaves them undefined.
///
/// # Errors
///
/// Any [`AggregationError`]; every one names the joint it came from, and
/// the first one aborts aggregation, matching upstream's `throw`.
pub fn aggregate_limits<'a>(
    joint_models: impl IntoIterator<Item = &'a JointModel>,
    overrides: &JointLimitsContainer,
) -> Result<JointLimitsContainer, AggregationError> {
    let mut container = JointLimitsContainer::default();

    for joint_model in joint_models {
        let name = joint_model.name();
        let mut joint_limit = overrides.limit(name).unwrap_or_default();

        if joint_limit.has_position_limits || joint_limit.has_velocity_limits {
            check_bounds_check_is_supported(joint_model)?;
        }

        if joint_limit.has_position_limits {
            check_position_bounds(joint_model, &joint_limit)?;
        } else {
            update_position_limit_from_joint_model(joint_model, &mut joint_limit);
        }

        if joint_limit.has_velocity_limits {
            check_velocity_bounds(joint_model, &joint_limit)?;
        } else {
            update_velocity_limit_from_joint_model(joint_model, &mut joint_limit);
        }

        // Rule 4: derive the deceleration limit from the acceleration one
        // when only the latter was given.
        if joint_limit.has_acceleration_limits && !joint_limit.has_deceleration_limits {
            joint_limit.max_deceleration = -joint_limit.max_acceleration;
            joint_limit.has_deceleration_limits = true;
        }

        // Asking `has_limit` first is what makes `add_limit`'s `false`
        // attributable: with the name known to be new, the only rejection
        // left is the deceleration-sign rule.
        if container.has_limit(name) {
            return Err(AggregationError::DuplicateJoint {
                joint: name.to_owned(),
            });
        }
        if !container.add_limit(name, joint_limit) {
            return Err(AggregationError::NonNegativeDeceleration {
                joint: name.to_owned(),
                max_deceleration: joint_limit.max_deceleration,
            });
        }
    }

    Ok(container)
}

/// The uniform guard replacing upstream's two differently-broken multi-DOF
/// bounds-check paths; see the module's `# Deviations from upstream`.
fn check_bounds_check_is_supported(joint_model: &JointModel) -> Result<(), AggregationError> {
    let variable_count = joint_model.variable_bounds().len();
    if variable_count > 1 {
        return Err(AggregationError::MultiDofBoundsCheck {
            joint: joint_model.name().to_owned(),
            variable_count,
        });
    }
    Ok(())
}

/// Upstream `checkPositionBoundsThrowing`.
fn check_position_bounds(
    joint_model: &JointModel,
    joint_limit: &JointLimit,
) -> Result<(), AggregationError> {
    if !joint_model.satisfies_position_bounds(&[joint_limit.min_position], 0.0) {
        return Err(AggregationError::MinPositionViolation {
            joint: joint_model.name().to_owned(),
        });
    }
    if !joint_model.satisfies_position_bounds(&[joint_limit.max_position], 0.0) {
        return Err(AggregationError::MaxPositionViolation {
            joint: joint_model.name().to_owned(),
        });
    }
    Ok(())
}

/// Upstream `checkVelocityBoundsThrowing`.
///
/// Upstream's comment on this function reads `// Check min position`; it is
/// a copy-paste of the previous function's and describes nothing this code
/// does.
fn check_velocity_bounds(
    joint_model: &JointModel,
    joint_limit: &JointLimit,
) -> Result<(), AggregationError> {
    if !joint_model.satisfies_velocity_bounds(&[joint_limit.max_velocity], 0.0) {
        return Err(AggregationError::MaxVelocityViolation {
            joint: joint_model.name().to_owned(),
        });
    }
    Ok(())
}

/// Upstream `updatePositionLimitFromJointModel`.
///
/// The `min_position`/`max_position` copy is unconditional in the
/// single-variable arm: upstream copies the numbers whether or not
/// `position_bounded_` is set, so a joint that is not position-bounded
/// still reports the model's range behind a clear flag.
fn update_position_limit_from_joint_model(joint_model: &JointModel, joint_limit: &mut JointLimit) {
    match joint_model.variable_bounds() {
        // Upstream `case 0`: warn and change nothing.
        [] => {}
        [bounds] => {
            joint_limit.has_position_limits = bounds.position_bounded;
            joint_limit.min_position = bounds.min_position;
            joint_limit.max_position = bounds.max_position;
        }
        // Upstream `default`: warn that multi-DOF is unsupported and pin
        // the joint to a zero-width window.
        _ => {
            joint_limit.has_position_limits = true;
            joint_limit.min_position = 0.0;
            joint_limit.max_position = 0.0;
        }
    }
}

/// Upstream `updateVelocityLimitFromJointModel`.
fn update_velocity_limit_from_joint_model(joint_model: &JointModel, joint_limit: &mut JointLimit) {
    match joint_model.variable_bounds() {
        // Upstream `case 0`: warn and change nothing.
        [] => {}
        [bounds] => {
            joint_limit.has_velocity_limits = bounds.velocity_bounded;
            joint_limit.max_velocity = bounds.max_velocity;
        }
        // Upstream `default`: warn that multi-DOF is unsupported and pin
        // the joint to zero velocity.
        _ => {
            joint_limit.has_velocity_limits = true;
            joint_limit.max_velocity = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use cspace_core::model::joint::VariableBounds;

    use super::*;

    /// A revolute joint bounded to `[-2, 2]` in position and `[-3, 3]` in
    /// velocity — the shape a URDF `<limit lower= upper= velocity=>` tag
    /// produces (see `cspace_core::model::joint::joint_model_from_urdf`, which
    /// sets `min_velocity = -max_velocity`).
    fn bounded_joint(name: &str) -> JointModel {
        let mut joint = JointModel::new_revolute(name);
        joint
            .set_variable_bounds(
                name,
                VariableBounds {
                    min_position: -2.0,
                    max_position: 2.0,
                    position_bounded: true,
                    min_velocity: -3.0,
                    max_velocity: 3.0,
                    velocity_bounded: true,
                    ..Default::default()
                },
            )
            .expect("a revolute joint's variable is named after the joint");
        joint
    }

    fn overrides(limits: &[(&str, JointLimit)]) -> JointLimitsContainer {
        let mut container = JointLimitsContainer::default();
        for (name, limit) in limits {
            assert!(
                container.add_limit(*name, *limit),
                "fixture override for {name} was rejected by add_limit"
            );
        }
        container
    }

    // -- Boundary: the joint list ------------------------------------------

    /// Boundary: no joints at all. Upstream's loop body never runs and the
    /// default-constructed container is returned.
    #[test]
    fn an_empty_joint_list_aggregates_to_an_empty_container() {
        let aggregated = aggregate_limits([], &JointLimitsContainer::default())
            .expect("an empty joint list cannot fail");
        assert!(aggregated.is_empty());
    }

    /// Upstream's `ExpectedMapSize`: one limit per joint model, no matter
    /// how many of them the override set mentions.
    #[test]
    fn every_joint_gets_exactly_one_limit() {
        let joints = [
            bounded_joint("j1"),
            bounded_joint("j2"),
            bounded_joint("j3"),
        ];
        let overrides = overrides(&[(
            "j2",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 1.0,
                ..Default::default()
            },
        )]);

        let aggregated =
            aggregate_limits(&joints, &overrides).expect("every override is within bounds");
        assert_eq!(aggregated.count(), 3);
        for name in ["j1", "j2", "j3"] {
            assert!(aggregated.has_limit(name));
        }
    }

    /// Boundary: the same joint twice. Upstream drops the second silently
    /// by discarding `addLimit`'s return.
    #[test]
    fn a_repeated_joint_is_rejected_rather_than_dropped() {
        let joint = bounded_joint("j1");
        let error = aggregate_limits([&joint, &joint], &JointLimitsContainer::default())
            .expect_err("the same joint twice cannot aggregate");
        assert_eq!(
            error,
            AggregationError::DuplicateJoint {
                joint: "j1".to_owned()
            }
        );
    }

    // -- Boundary: a joint with no override --------------------------------

    /// Rule 3's other half: a joint the override set does not mention
    /// takes position and velocity from the model, and leaves acceleration
    /// and deceleration exactly as [`JointLimit::default`] has them.
    #[test]
    fn a_joint_with_no_override_takes_the_models_bounds_and_nothing_else() {
        let joints = [bounded_joint("j1")];
        let aggregated = aggregate_limits(&joints, &JointLimitsContainer::default())
            .expect("no override cannot violate a bound");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");

        assert!(limit.has_position_limits);
        assert_eq!(limit.min_position, -2.0);
        assert_eq!(limit.max_position, 2.0);
        assert!(limit.has_velocity_limits);
        assert_eq!(limit.max_velocity, 3.0);

        assert!(!limit.has_acceleration_limits);
        assert!(limit.max_acceleration.is_nan());
        assert!(!limit.has_deceleration_limits);
        assert_eq!(limit.max_deceleration, 0.0);
    }

    /// Boundary: the model's numbers are copied even when the model's flag
    /// is clear. Upstream's `case 1` assigns `min_position`/`max_position`
    /// outside the `position_bounded_` test, so the numbers land in the
    /// aggregated limit while `has_position_limits` stays `false`.
    #[test]
    fn an_unbounded_model_position_still_contributes_its_numbers() {
        let mut joint = JointModel::new_revolute("j1");
        joint
            .set_variable_bounds(
                "j1",
                VariableBounds {
                    min_position: -5.0,
                    max_position: 5.0,
                    position_bounded: false,
                    max_velocity: 7.0,
                    velocity_bounded: false,
                    ..Default::default()
                },
            )
            .expect("a revolute joint's variable is named after the joint");

        let joints = [joint];
        let aggregated = aggregate_limits(&joints, &JointLimitsContainer::default())
            .expect("no override cannot violate a bound");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");

        assert!(!limit.has_position_limits);
        assert_eq!(limit.min_position, -5.0);
        assert_eq!(limit.max_position, 5.0);
        assert!(!limit.has_velocity_limits);
        assert_eq!(limit.max_velocity, 7.0);
    }

    /// Boundary: zero variables. Upstream's `case 0` leaves `joint_limit`
    /// untouched, so a fixed joint aggregates to the default limit —
    /// `NaN` positions behind a clear flag, not the zero-width window the
    /// *multi*-variable arm produces.
    ///
    /// Asserted field by field rather than against `JointLimit::default()`:
    /// the derived `PartialEq` compares the `NaN`s, so that equality is
    /// `false` for two identical defaults and would make the case vacuous
    /// in the other direction.
    #[test]
    fn a_joint_with_no_variables_contributes_nothing() {
        let joints = [JointModel::new_fixed("j1")];
        let aggregated = aggregate_limits(&joints, &JointLimitsContainer::default())
            .expect("no override cannot violate a bound");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");

        assert!(!limit.has_position_limits);
        assert!(limit.min_position.is_nan());
        assert!(limit.max_position.is_nan());
        assert!(!limit.has_velocity_limits);
        assert!(limit.max_velocity.is_nan());
        assert!(!limit.has_deceleration_limits);
        assert_eq!(limit.max_deceleration, 0.0);
    }

    /// Boundary: more than one variable, no override. Upstream's `default`
    /// arm sets the flags *and* zeroes the values, which is a stricter
    /// limit than the model carries, not a looser one.
    #[test]
    fn a_multi_variable_joint_with_no_override_is_pinned_to_zero() {
        let joints = [JointModel::new_planar("j1")];
        let aggregated = aggregate_limits(&joints, &JointLimitsContainer::default())
            .expect("no override means no bounds check");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");

        assert!(limit.has_position_limits);
        assert_eq!(limit.min_position, 0.0);
        assert_eq!(limit.max_position, 0.0);
        assert!(limit.has_velocity_limits);
        assert_eq!(limit.max_velocity, 0.0);
    }

    // -- Boundary: position overrides --------------------------------------

    /// Rule 1 accepting: an override strictly inside the model's window
    /// replaces it.
    #[test]
    fn a_stricter_position_override_replaces_the_models_window() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -1.0,
                max_position: 1.0,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("[-1, 1] is inside [-2, 2]");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.min_position, -1.0);
        assert_eq!(limit.max_position, 1.0);
    }

    /// Boundary: an override sitting exactly *on* the model's bounds.
    /// `satisfiesPositionBounds` is inclusive (`>=` / `<=`), so equal is
    /// allowed — upstream's own doc says "stricter **or equal**".
    #[test]
    fn a_position_override_equal_to_the_models_bounds_is_accepted() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -2.0,
                max_position: 2.0,
                ..Default::default()
            },
        )]);

        let aggregated =
            aggregate_limits(&joints, &overrides).expect("equal is not stricter-than-required");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.min_position, -2.0);
        assert_eq!(limit.max_position, 2.0);
    }

    #[test]
    fn a_min_position_below_the_models_is_rejected() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -2.001,
                max_position: 2.0,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides).expect_err("-2.001 is outside [-2, 2]");
        assert_eq!(
            error,
            AggregationError::MinPositionViolation {
                joint: "j1".to_owned()
            }
        );
    }

    #[test]
    fn a_max_position_above_the_models_is_rejected() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -2.0,
                max_position: 2.001,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides).expect_err("2.001 is outside [-2, 2]");
        assert_eq!(
            error,
            AggregationError::MaxPositionViolation {
                joint: "j1".to_owned()
            }
        );
    }

    /// Boundary: a continuous revolute joint. Rule 1 has nothing to
    /// enforce — `RevoluteJointModel::satisfiesPositionBounds` returns
    /// `true` unconditionally when the joint wraps — so an override a
    /// bounded joint would be rejected for is accepted here.
    #[test]
    fn a_continuous_joint_accepts_any_position_override() {
        let mut joint = JointModel::new_revolute("j1");
        joint.set_continuous(true).expect("j1 is revolute");
        let joints = [joint];

        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -1000.0,
                max_position: 1000.0,
                ..Default::default()
            },
        )]);

        let aggregated =
            aggregate_limits(&joints, &overrides).expect("a continuous joint has no window");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.min_position, -1000.0);
        assert_eq!(limit.max_position, 1000.0);
    }

    // -- Boundary: velocity overrides --------------------------------------

    #[test]
    fn a_stricter_velocity_override_replaces_the_models_bound() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 1.1,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("1.1 is inside [-3, 3]");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.max_velocity, 1.1);
    }

    #[test]
    fn a_max_velocity_above_the_models_is_rejected() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 3.001,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides).expect_err("3.001 is outside [-3, 3]");
        assert_eq!(
            error,
            AggregationError::MaxVelocityViolation {
                joint: "j1".to_owned()
            }
        );
    }

    /// Boundary: the *lower* side of the velocity window. Upstream checks
    /// `max_velocity` with `satisfiesVelocityBounds`, which tests both
    /// ends, so a negative `max_velocity` fails against `min_velocity` —
    /// the shape upstream's own `violate_velocity` fixture uses
    /// (`max_velocity: -90.0`).
    #[test]
    fn a_negative_max_velocity_below_the_models_min_is_rejected() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: -3.001,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides).expect_err("-3.001 is outside [-3, 3]");
        assert_eq!(
            error,
            AggregationError::MaxVelocityViolation {
                joint: "j1".to_owned()
            }
        );
    }

    // -- Boundary: acceleration and deceleration ---------------------------

    /// Rule 4: acceleration alone derives the deceleration limit.
    #[test]
    fn an_acceleration_override_alone_derives_the_deceleration_limit() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: 5.5,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("-5.5 is a valid limit");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert!(limit.has_acceleration_limits);
        assert_eq!(limit.max_acceleration, 5.5);
        assert!(limit.has_deceleration_limits);
        assert_eq!(limit.max_deceleration, -5.5);
    }

    /// Boundary: rule 4 does *not* run in reverse. A deceleration override
    /// alone leaves acceleration undefined, which is upstream's
    /// `prbt_joint_5` expectation (`isnan(max_acceleration)`,
    /// `max_deceleration == -6.6`).
    #[test]
    fn a_deceleration_override_alone_leaves_acceleration_undefined() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_deceleration_limits: true,
                max_deceleration: -6.6,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("-6.6 is a valid limit");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert!(!limit.has_acceleration_limits);
        assert!(limit.max_acceleration.is_nan());
        assert_eq!(limit.max_deceleration, -6.6);
    }

    /// Boundary: both given. Rule 4's `&& !has_deceleration_limits` is what
    /// keeps the explicit value; without it the override would be
    /// overwritten by `-max_acceleration`.
    #[test]
    fn an_explicit_deceleration_override_survives_rule_four() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: 5.5,
                has_deceleration_limits: true,
                max_deceleration: -6.6,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("-6.6 is a valid limit");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.max_acceleration, 5.5);
        assert_eq!(limit.max_deceleration, -6.6);
    }

    /// Boundary: `max_acceleration == 0.0` makes rule 4 produce `-0.0`,
    /// and `-0.0 >= 0.0` is true, so
    /// [`JointLimitsContainer::add_limit`] refuses the aggregated limit.
    /// Upstream discards that refusal and the joint disappears from a
    /// container its own `ExpectedMapSize` test then measures.
    #[test]
    fn a_zero_acceleration_override_is_reported_not_dropped() {
        let joints = [bounded_joint("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: 0.0,
                ..Default::default()
            },
        )]);

        let error =
            aggregate_limits(&joints, &overrides).expect_err("-0.0 is not a negative deceleration");
        assert_eq!(
            error,
            AggregationError::NonNegativeDeceleration {
                joint: "j1".to_owned(),
                max_deceleration: -0.0,
            }
        );
    }

    // -- Boundary: multi-variable joints under an override -----------------

    #[test]
    fn a_position_override_on_a_multi_variable_joint_is_rejected() {
        let joints = [JointModel::new_planar("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_position_limits: true,
                min_position: -1.0,
                max_position: 1.0,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides)
            .expect_err("a planar joint cannot be bounds-checked");
        assert_eq!(
            error,
            AggregationError::MultiDofBoundsCheck {
                joint: "j1".to_owned(),
                variable_count: 3,
            }
        );
    }

    #[test]
    fn a_velocity_override_on_a_multi_variable_joint_is_rejected() {
        let joints = [JointModel::new_floating("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_velocity_limits: true,
                max_velocity: 1.0,
                ..Default::default()
            },
        )]);

        let error = aggregate_limits(&joints, &overrides)
            .expect_err("a floating joint cannot be bounds-checked");
        assert_eq!(
            error,
            AggregationError::MultiDofBoundsCheck {
                joint: "j1".to_owned(),
                variable_count: 7,
            }
        );
    }

    /// Boundary: a multi-variable joint whose override touches only
    /// acceleration. No bounds check is reached, so the guard must *not*
    /// fire — this is what keeps the guard scoped to the checks rather
    /// than to the joint.
    #[test]
    fn an_acceleration_only_override_on_a_multi_variable_joint_is_allowed() {
        let joints = [JointModel::new_planar("j1")];
        let overrides = overrides(&[(
            "j1",
            JointLimit {
                has_acceleration_limits: true,
                max_acceleration: 2.0,
                ..Default::default()
            },
        )]);

        let aggregated = aggregate_limits(&joints, &overrides).expect("no bounds check is reached");
        let limit = aggregated.limit("j1").expect("j1 was aggregated");
        assert_eq!(limit.max_acceleration, 2.0);
        assert_eq!(limit.max_deceleration, -2.0);
        // and the multi-DOF arms of the two update helpers still ran
        assert_eq!(limit.max_position, 0.0);
        assert_eq!(limit.max_velocity, 0.0);
    }
}
