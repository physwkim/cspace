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
//! - [`registry`] — [`registry::PlannerManager`]/[`registry::PlanningContext`]
//!   and the [`registry::PLANNER_MANAGERS`] compile-time registry (D4),
//!   plus [`registry::RrtConnectManager`], the one registered planner.
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
//!
//! # Round 6 symbol audit
//!
//! This crate has two upstream relationships, not one, and they get audited
//! separately:
//!
//! - The state-space/algorithm modules ([`space`], [`so2`], [`se3`],
//!   [`compound`], [`nn`], [`rrt_connect`]) have **no upstream C++ file at
//!   all** (D3, see the top-of-file comment) — there is no OMPL header in
//!   this workspace to audit them against, so they are out of scope for a
//!   symbol-closure audit by construction, not by omission.
//! - [`registry`]'s [`registry::PlannerManager`]/[`registry::PlanningContext`]
//!   *do* have an upstream counterpart —
//!   `moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp`
//!   — read directly for this audit. Every symbol below:
//!
//! ## `PlannerConfigurationSettings` / `PlannerConfigurationMap`
//!
//! - Both -> unported: a stringly-typed `HashMap<String, String>` plugin
//!   config bag for a runtime plugin-loading boundary this crate doesn't
//!   have (D4: [`registry::PLANNER_MANAGERS`] is a compile-time registry).
//!   [`registry::PlanningRequest::params`]/[`registry::PlanningRequest::resolution`]
//!   are concretely-typed fields instead — see `registry`'s own doc comment,
//!   "Planner-specific tuning" paragraph.
//!
//! ## `PlanningContext`
//!
//! - `ctor(name, group)` -> unported: no persistent named/grouped identity
//!   object; `registry`'s private `RrtConnectContext` borrows exactly what
//!   `solve()` needs and nothing more.
//! - `getGroupName()`/`getName()` -> unported: the group name already lives
//!   on the [`registry::PlanningRequest`] the caller holds; no second
//!   accessor is needed since this port keeps no separate identity struct.
//! - `getPlanningScene()`/`getMotionPlanRequest()` -> unported: the caller
//!   already owns the `&mut PlanningScene` and [`registry::PlanningRequest`]
//!   it passed to [`registry::PlannerManager::get_planning_context`]; nothing
//!   needs them handed back.
//! - `setPlanningScene()`/`setMotionPlanRequest()` -> unported: every
//!   [`registry::PlanningContext`] here is single-use, built fresh per
//!   `solve()` — see [`registry::PlanningContext`]'s own "Deviation from
//!   upstream: no `terminate`/`clear`" doc section, which this shares the
//!   same reasoning with.
//! - `solve(MotionPlanResponse&)` and `solve(MotionPlanDetailedResponse&)`
//!   -> collapsed and ported as one [`registry::PlanningContext::solve`]
//!   returning `Result<`[`registry::PlanningResponse`]`, `[`registry::PlanError`]`>`;
//!   no detailed-response variant exists because nothing in this workspace
//!   consumes upstream's extra per-stage timing/trajectory detail.
//! - `terminate()`/`clear()` -> unported — see [`registry::PlanningContext`]'s
//!   own "Deviation from upstream" doc: no concurrency model here for
//!   cross-thread cancellation, and no context reuse to clear.
//!
//! ## `PlannerManager`
//!
//! - `initialize(model, node, parameter_namespace)` -> unported: no
//!   `rclcpp::Node`/ROS parameter namespace exists anywhere in this
//!   workspace (D1/D2); [`registry::RrtConnectManager`] needs no
//!   initialization step (`#[derive(Default)]`, a unit struct).
//! - `getDescription()` -> unported: no caller needs a human-readable
//!   description string; [`registry::PlannerManager::name`] (below) already
//!   identifies the manager uniquely for the registry lookup.
//! - `getPlanningAlgorithms(algs)` -> unported: this crate registers one
//!   algorithm per [`registry::PlannerManager`] impl (1:1, not 1:many like
//!   upstream's plugin-hosts-multiple-algorithms model), so there is no
//!   "algorithm names within one manager" list to enumerate;
//!   [`registry::PLANNER_MANAGERS`] itself is the cross-manager list.
//! - `getPlanningContext(scene, req, error_code)` -> ported as
//!   [`registry::PlannerManager::get_planning_context`], collapsed: the
//!   `moveit_msgs::msg::MoveItErrorCodes` out-parameter becomes the ordinary
//!   `Result<_, `[`registry::PlanError`]`>` return.
//! - `getPlanningContext(scene, req)` (the error-code-ignoring overload) ->
//!   unported: Rust already makes ignoring a `Result` an explicit
//!   `.unwrap()`/`let _ =` at the call site; no second overload is needed to
//!   spell that.
//! - `canServiceRequest(req)` -> unported: `get_planning_context` itself is
//!   the only admission check (it fails with e.g. `SbpError::UnknownGroup`);
//!   no separate dry-run query exists to ask "would you accept this" without
//!   also building the context.
//! - `setPlannerConfigurations(pcs)`/`getPlannerConfigurations()` -> unported:
//!   no `PlannerConfigurationMap` exists here (see above).
//! - `terminate()` (non-virtual, base-class async-cancel) -> unported, same
//!   reasoning as `PlanningContext::terminate()` above.
//! - Not upstream: [`registry::PlannerManager::name`] — new API this port's
//!   registry lookup needs (matches `moveit_kinematics::registry`'s
//!   `SolverRegistration` D4 precedent, per `registry.rs`'s own top-of-file
//!   comment).

pub mod compound;
mod error;
pub mod joint_model_group_space;
pub mod nn;
pub mod planning_scene_validity;
pub mod registry;
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
pub use registry::{
    PLANNER_MANAGERS, PlanError, PlannerManager, PlannerRegistration, PlanningContext,
    PlanningRequest, PlanningResponse, RrtConnectManager,
};
pub use rrt_connect::{PlanningFailure, RrtConnectParams, Termination, rrt_connect};
pub use se3::{Se3Space, Se3State};
pub use so2::So2Space;
pub use space::{RealVectorSpace, StateSpace};
pub use validity::{DiscreteMotionValidator, MotionValidator, StateValidityChecker};
