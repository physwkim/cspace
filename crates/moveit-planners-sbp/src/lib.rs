// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream C++ file to port: PORTING-PLAN.md §2 records that the Rust
// ecosystem has no OMPL equivalent, and §6.3 lists that as the top risk
// (D3: native Rust planners first, an OMPL FFI bridge only as fallback).
// This crate is original design work against that gap, not a transcription.
// RRT-Connect follows the published algorithm (Kuffner & LaValle, ICRA
// 2000); the nearest-neighbour index is GNAT-family (Brin, 1995) for the
// reason recorded in `nn`'s doc comment, not a port of OMPL's C++ GNAT.

//! Sampling-based motion planning for moveit-rs.
//!
//! # Scope
//!
//! This is the abstract foundation plus one planner, deliberately without a
//! dependency on `moveit-model` or `moveit-state`: everything here compiles
//! and is tested standalone against [`RealVectorSpace`], a plain bounded
//! `R^n`. Compound spaces matching MoveIt's actual joint types (a revolute
//! joint's wraparound, a floating joint's `SO(3)` orientation, a
//! `JointModelGroup`'s product space) are future work layered on
//! [`StateSpace`]; nothing in this crate assumes `RealVectorSpace` is the
//! only space that will ever exist.
//!
//! - [`space`] — the [`StateSpace`] trait and [`RealVectorSpace`].
//! - [`validity`] — [`StateValidityChecker`] and [`MotionValidator`], kept
//!   separate on purpose (see [`validity`]'s doc comment).
//! - [`nn`] — [`Gnat`], the nearest-neighbour index.
//! - [`rrt_connect`] — bidirectional RRT-Connect.
//!
//! # Why properties, not an oracle
//!
//! Every other crate in this workspace is checked against `tools/moveit-oracle`,
//! a C++ binary linking the real moveit2. There is nothing to link here: no
//! C++ RRT-Connect or GNAT exists in this workspace to compare against, and
//! a sampling planner's *specific* output path is not a meaningful thing to
//! match bit-for-bit against a different implementation's RNG draws anyway.
//! Correctness here is established by properties that would fail if the
//! implementation were wrong — path endpoints are exact, every returned
//! segment is independently re-checked against the same
//! [`MotionValidator`] used to build it, nearest-neighbour results are
//! checked against brute force over thousands of queries, and a closed
//! passage is checked to fail rather than hang or return an invalid path.
//! See each module's `tests` for the specific claims and the crate's commit
//! history / report for which parts of this design are least certain.

mod error;
mod nn;
mod rrt_connect;
mod space;
mod validity;

pub use error::SbpError;
pub use nn::Gnat;
pub use rrt_connect::{RrtConnectParams, rrt_connect};
pub use space::{RealVectorSpace, StateSpace};
pub use validity::{DiscreteMotionValidator, MotionValidator, StateValidityChecker};
