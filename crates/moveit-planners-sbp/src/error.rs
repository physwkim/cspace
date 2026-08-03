// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Error type for this crate's boundary inputs.
//!
//! Scoped narrowly on purpose: the only fallible *construction* in this
//! crate's Phase 7 scope is [`crate::space::RealVectorSpace::new`], whose
//! bounds could plausibly come from external input (a robot description,
//! eventually). [`crate::rrt_connect::RrtConnectParams`] is validated by
//! `assert!` instead — passing it a negative step size is a programming
//! error, not external input, so panicking immediately with a clear message
//! is preferable to plumbing a `Result` through the planner's success path.

/// An error constructing a Phase 7 planning type from caller-supplied data.
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
}
