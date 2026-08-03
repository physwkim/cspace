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
//! This is the abstract foundation plus one planner, built against four
//! [`StateSpace`] implementations covering MoveIt's actual joint types:
//! [`RealVectorSpace`] (plain bounded `R^n` — prismatic and bounded revolute
//! joints), [`so2::So2Space`] (a continuous joint's wraparound),
//! [`se3::Se3Space`] (a floating joint's `R^3 x SO(3)`), and
//! [`compound::CompoundSpace`] (a `JointModelGroup`'s heterogeneous product
//! of any of the above, weighted). All four were first tested standalone
//! with no dependency on `moveit-model` or `moveit-state`;
//! [`joint_model_group_space::JointModelGroupSpace`] is the bridge from an
//! actual `RobotModel`/`JointModelGroup` to a [`StateSpace`], and is what
//! brings those two crates in as dependencies.
//!
//! - [`space`] — the [`StateSpace`] trait and [`RealVectorSpace`].
//! - [`so2`] — [`so2::So2Space`], a wraparound revolute joint.
//! - [`se3`] — [`se3::Se3Space`], a floating joint.
//! - [`compound`] — [`compound::CompoundSpace`], a weighted product of
//!   subspaces of any of the above kinds.
//! - [`joint_model_group_space`] —
//!   [`joint_model_group_space::JointModelGroupSpace`], a `RobotModel` joint
//!   model group as a [`StateSpace`].
//! - [`validity`] — [`StateValidityChecker`] and [`MotionValidator`], kept
//!   separate on purpose (see [`validity`]'s doc comment).
//! - [`planning_scene_validity`] —
//!   [`planning_scene_validity::PlanningSceneValidityChecker`], the bridge
//!   from a [`joint_model_group_space::JointModelGroupSpace`] sample to a
//!   real `moveit_scene::PlanningScene` collision/constraint check.
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

pub mod compound;
mod error;
pub mod joint_model_group_space;
pub mod nn;
pub mod planning_scene_validity;
mod rrt_connect;
mod sampling;
pub mod se3;
pub mod so2;
pub mod space;
#[cfg(test)]
mod test_support;
pub mod validity;

pub use compound::{CompoundSpace, CompoundValue};
pub use error::SbpError;
pub use joint_model_group_space::JointModelGroupSpace;
pub use nn::Gnat;
pub use planning_scene_validity::PlanningSceneValidityChecker;
pub use rrt_connect::{PlanningFailure, RrtConnectParams, Termination, rrt_connect};
pub use se3::{Se3Space, Se3State};
pub use so2::So2Space;
pub use space::{RealVectorSpace, StateSpace};
pub use validity::{DiscreteMotionValidator, MotionValidator, StateValidityChecker};
