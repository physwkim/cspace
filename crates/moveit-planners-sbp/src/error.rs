// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Error type for this crate's boundary inputs.
//!
//! Scoped narrowly on purpose: the fallible *constructions* in this crate
//! are [`crate::space::RealVectorSpace::new`] and its callers (bounds that
//! could plausibly come from external input, a robot description in
//! particular) and, since the `RobotModel` bridge landed,
//! [`crate::joint_model_group_space::JointModelGroupSpace::new`], whose
//! group name is a caller-supplied string looked up against a
//! [`moveit_model::RobotModel`] built at runtime.
//! [`crate::rrt_connect::RrtConnectParams`] is validated by `assert!`
//! instead — passing it a negative step size is a programming error, not
//! external input, so panicking immediately with a clear message is
//! preferable to plumbing a `Result` through the planner's success path.

/// An error constructing a planning type in this crate from caller-supplied
/// data.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SbpError {
    /// A [`crate::space::RealVectorSpace`] was built with no dimensions.
    #[error("a state space needs at least one dimension, got 0 bounds")]
    NoDimensions,

    /// A per-dimension bound was empty (`min > max`) or non-finite.
    #[error(
        "invalid bound at dimension {index}: min {min} and max {max} must both be finite with min <= max"
    )]
    InvalidBound {
        /// Which dimension the bad bound was at.
        index: usize,
        /// The offending lower bound.
        min: f64,
        /// The offending upper bound.
        max: f64,
    },

    /// A subspace weight (e.g. [`crate::se3::Se3Space`]'s rotation weight)
    /// was negative or non-finite.
    #[error("invalid weight {value}: must be finite and non-negative")]
    InvalidWeight {
        /// The offending weight.
        value: f64,
    },

    /// A [`crate::compound::CompoundSpace`] was built with no subspaces.
    #[error("a compound state space needs at least one subspace, got 0")]
    NoSubspaces,

    /// A [`crate::compound::CompoundSpace`] subspace's weight was negative
    /// or non-finite.
    #[error("invalid weight {value} for subspace {index}: must be finite and non-negative")]
    InvalidSubspaceWeight {
        /// Which subspace (in the order passed to
        /// [`crate::compound::CompoundSpace::new`]) had the bad weight.
        index: usize,
        /// The offending weight.
        value: f64,
    },

    /// [`crate::joint_model_group_space::JointModelGroupSpace::new`] was
    /// given a group name the [`moveit_model::RobotModel`] does not have.
    #[error("unknown joint model group '{name}'")]
    UnknownGroup {
        /// The group name that was not found.
        name: String,
    },
}
