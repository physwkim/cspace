// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp

//! Kinematic constraints and their `decide()` — [`JointConstraint`],
//! [`PositionConstraint`], [`OrientationConstraint`], [`VisibilityConstraint`]
//! and the aggregate [`KinematicConstraintSet`].
//!
//! # Scope
//!
//! This crate ports `kinematic_constraints/kinematic_constraint.{hpp,cpp}` —
//! the four constraint types and their `decide()`/construction logic.
//! `kinematic_constraints/utils.{hpp,cpp}` is **not** ported: every function
//! there (`constructGoalConstraints`, `mergeConstraints`,
//! `constructConstraints` from YAML, `resolveConstraintFrames`, ...) takes or
//! returns a `moveit_msgs::msg::Constraints`/`geometry_msgs` value, or an
//! `rclcpp::Node`. Per `PORTING-PLAN.md` D1/D2 this core crate references no
//! ROS type at all, so those helpers have no home here — they are
//! `moveit-ros`/`moveit-planning` convenience wrappers around the types this
//! crate defines, not part of the constraint model itself.
//!
//! `equal()`, `print()`, `clear()` and `getMarkers()` are also not ported:
//! none of them are exercised by `decide()`, which is this phase's
//! completion condition (`PORTING-PLAN.md` §5 Phase 5). See the report this
//! crate's introducing commits carry for what remains `UNFIXED`.
//!
//! # No `moveit_msgs::Constraints` — and no `configure(msg, tf)`
//!
//! Upstream's four `configure()` methods each take a
//! `moveit_msgs::msg::{Joint,Position,Orientation,Visibility}Constraint`.
//! Every one of those messages encodes optionality as a `bool has_x` field
//! beside an `x` value — the same dual-meaning problem `PORTING-PLAN.md` §4.1
//! already named for `RobotState`'s dirty flags. This crate has no
//! `moveit_msgs` type to receive at all (D1), so there is no `configure()`
//! parallel to port in the first place; instead each constraint type has a
//! `new()` that takes plain Rust arguments chosen so the illegal
//! combinations upstream's message shape allows cannot be constructed here.
//! `PORTING-PLAN.md` §4.3 (renumbered under Phase 5) records the specific
//! mapping decision for each type and names the conversions a future
//! `moveit-ros::TryFrom<moveit_msgs::...>` will have to report as lossy.
//!
//! # `VisibilityConstraint` is ported in full
//!
//! Upstream's `VisibilityConstraint::decide()` builds a mesh cone between
//! sensor and target and collision-checks it against the robot via a local,
//! throwaway `collision_detection::CollisionEnvFCL`. This port does the
//! same over `moveit_collision::ParryCollisionEnv` — see
//! [`VisibilityConstraint::decide`]'s doc for why that needs no
//! `PlanningScene`/broader collision world, only `moveit-collision`
//! (already a dependency of this crate, no `moveit-scene` needed).

mod joint;
mod orientation;
mod position;
mod set;
mod visibility;

pub use joint::JointConstraint;
pub use orientation::{OrientationConstraint, OrientationTolerance};
pub use position::{ConstraintRegion, PositionConstraint};
pub use set::{Constraint, KinematicConstraintSet};
pub use visibility::{
    SensorSpec, SensorViewDirection, TargetSpec, VisibilityConstraint, VisibilityCriteria,
};

/// The result of evaluating one constraint against a state. Upstream
/// `kinematic_constraints::ConstraintEvaluationResult`.
///
/// Unlike the four constraint types themselves, this struct needed no
/// `Option`/enum redesign: both fields always hold one meaning regardless of
/// context (`satisfied` is never conditionally overloaded, and `distance` is
/// always "how far from satisfied, in the constraint's own units" —
/// `0.0` both when perfectly satisfied and, degenerately, for a disabled
/// constraint that always reports satisfied). It is ported as a direct
/// transcription.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConstraintEvaluationResult {
    /// Whether the constraint was satisfied by the state it was evaluated
    /// against.
    pub satisfied: bool,
    /// The distance from being satisfied, weighted by the constraint's own
    /// weight. `0.0` when satisfied.
    pub distance: f64,
}

impl ConstraintEvaluationResult {
    /// Build a result. Upstream's `ConstraintEvaluationResult` constructor.
    pub fn new(satisfied: bool, distance: f64) -> Self {
        Self {
            satisfied,
            distance,
        }
    }
}
