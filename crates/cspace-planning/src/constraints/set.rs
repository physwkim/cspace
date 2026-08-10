// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class KinematicConstraintSet)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (KinematicConstraintSet::decide)

use cspace_core::state::Posed;

use crate::constraints::{
    ConstraintEvaluationResult, JointConstraint, OrientationConstraint, PositionConstraint,
    VisibilityConstraint,
};

/// One constraint held by a [`KinematicConstraintSet`].
///
/// Upstream's `KinematicConstraintSet` stores every added constraint twice:
/// once as a `KinematicConstraintPtr` (the polymorphic base pointer
/// `decide()` actually calls) and once more as its own
/// `moveit_msgs::msg::{Joint,Position,Orientation,Visibility}Constraint`,
/// segregated into four parallel `std::vector`s plus one
/// `moveit_msgs::msg::Constraints all_constraints_` that re-aggregates all
/// four — five copies of the same information, kept in sync by convention
/// across `add()`'s four overloads. This crate has no `moveit_msgs` type to
/// keep a second copy in (D1) and no polymorphic base to call through (D4:
/// plugins are enums, not trait objects, here) — [`Constraint`] is the one
/// representation, and [`KinematicConstraintSet`] is a `Vec` of it.
#[derive(Debug, Clone, PartialEq)]
pub enum Constraint {
    /// A [`JointConstraint`].
    Joint(JointConstraint),
    /// A [`PositionConstraint`].
    Position(PositionConstraint),
    /// An [`OrientationConstraint`].
    Orientation(OrientationConstraint),
    /// A [`VisibilityConstraint`].
    Visibility(VisibilityConstraint),
}

/// A collection of constraints, satisfied exactly when every member is.
///
/// Upstream `kinematic_constraints::KinematicConstraintSet`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct KinematicConstraintSet {
    constraints: Vec<Constraint>,
}

impl KinematicConstraintSet {
    /// An empty set — vacuously satisfied by every state, matching
    /// upstream's default-constructed `KinematicConstraintSet`.
    pub fn new() -> Self {
        Self::default()
    }

    /// `add`, folded to one constraint at a time since this port has no
    /// `moveit_msgs::msg::Constraints` to add many from at once.
    pub fn push(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    /// The constraints in this set, in the order they were pushed.
    pub fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    /// Mutable access to the same members, in the same order. Used by
    /// `crate::constraints::utils`'s `updateXConstraint` family, which finds an existing
    /// constraint by link/joint name and replaces it in place — upstream's
    /// equivalent mutates one field of a stored `moveit_msgs` value; this
    /// crate's constraint types have no such mutable form to patch (see this
    /// crate's introducing doc comment on why `new()` replaces `configure()`),
    /// so "update" here means "replace the matching entry with a freshly
    /// reconstructed one".
    pub fn constraints_mut(&mut self) -> &mut [Constraint] {
        &mut self.constraints
    }

    /// `empty`
    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty()
    }

    /// Number of constraints in this set.
    pub fn len(&self) -> usize {
        self.constraints.len()
    }

    /// `decide(state, results, verbose)`: every member's individual result,
    /// in [`KinematicConstraintSet::constraints`] order.
    pub fn decide_each(&self, state: &Posed) -> Vec<ConstraintEvaluationResult> {
        self.constraints
            .iter()
            .map(|constraint| match constraint {
                Constraint::Joint(c) => c.decide(state),
                Constraint::Position(c) => c.decide(state),
                Constraint::Orientation(c) => c.decide(state),
                Constraint::Visibility(c) => c.decide(state),
            })
            .collect()
    }

    /// `decide(state, verbose)`: satisfied iff every member is, with a
    /// distance that is the sum of every member's.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        self.decide_each(state).into_iter().fold(
            ConstraintEvaluationResult::new(true, 0.0),
            |acc, r| {
                ConstraintEvaluationResult::new(
                    acc.satisfied && r.satisfied,
                    acc.distance + r.distance,
                )
            },
        )
    }
}
