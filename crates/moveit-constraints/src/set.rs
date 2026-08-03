// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class KinematicConstraintSet)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (KinematicConstraintSet::decide)

use moveit_state::Posed;

use crate::visibility::VisibilityDecision;
use crate::{
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

/// [`KinematicConstraintSet::decide`] could not fully evaluate the set: the
/// constraint at `index` is a [`VisibilityConstraint`] whose criteria are not
/// decidable by geometry alone (see
/// [`VisibilityConstraint::decide_geometry`] and the crate's module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "constraint at index {index} (a VisibilityConstraint) needs a cone-vs-robot \
     collision check this port cannot yet perform"
)]
pub struct UndecidedConstraint {
    /// Index into [`KinematicConstraintSet::constraints`] of the constraint
    /// that could not be decided.
    pub index: usize,
}

/// A collection of constraints, satisfied exactly when every member is.
///
/// Upstream `kinematic_constraints::KinematicConstraintSet`.
///
/// # Deviation from upstream: `decide()` can refuse to answer
///
/// Upstream's `decide()` always returns a `ConstraintEvaluationResult`. This
/// port's [`KinematicConstraintSet::decide`] returns
/// `Result<_, UndecidedConstraint>` instead, because a contained
/// [`VisibilityConstraint`] can genuinely not be decided yet (no collision
/// backend — see the crate's module docs). Reporting such a set as
/// "satisfied" would be worse than refusing to answer: it is exactly the
/// silently-wrong-satisfied outcome
/// [`crate::visibility::VisibilityDecision::NeedsConeCollisionCheck`] exists
/// to prevent one layer up.
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
    ///
    /// # Errors
    ///
    /// [`UndecidedConstraint`] naming the first contained
    /// [`VisibilityConstraint`] (by index) that needed a cone-vs-robot
    /// collision check this port cannot yet perform.
    pub fn decide_each(
        &self,
        state: &Posed,
    ) -> Result<Vec<ConstraintEvaluationResult>, UndecidedConstraint> {
        self.constraints
            .iter()
            .enumerate()
            .map(|(index, constraint)| match constraint {
                Constraint::Joint(c) => Ok(c.decide(state)),
                Constraint::Position(c) => Ok(c.decide(state)),
                Constraint::Orientation(c) => Ok(c.decide(state)),
                Constraint::Visibility(c) => match c.decide_geometry(state) {
                    VisibilityDecision::Decided(r) => Ok(r),
                    VisibilityDecision::NeedsConeCollisionCheck => {
                        Err(UndecidedConstraint { index })
                    }
                },
            })
            .collect()
    }

    /// `decide(state, verbose)`: satisfied iff every member is, with a
    /// distance that is the sum of every member's.
    ///
    /// # Errors
    ///
    /// See [`KinematicConstraintSet::decide_each`].
    pub fn decide(&self, state: &Posed) -> Result<ConstraintEvaluationResult, UndecidedConstraint> {
        let results = self.decide_each(state)?;
        Ok(results
            .into_iter()
            .fold(ConstraintEvaluationResult::new(true, 0.0), |acc, r| {
                ConstraintEvaluationResult::new(
                    acc.satisfied && r.satisfied,
                    acc.distance + r.distance,
                )
            }))
    }
}
