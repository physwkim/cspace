// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_request_adapter_plugins/src/
//     check_for_stacked_constraints.cpp
//     check_start_state_bounds.cpp
//     check_start_state_collision.cpp
//     resolve_constraint_frames.cpp
//     validate_workspace_bounds.cpp
//   moveit_ros/planning/planning_response_adapter_plugins/src/
//     validate_path.cpp
//     add_ruckig_traj_smoothing.cpp
//     add_time_optimal_parameterization.cpp
//
// Considered and deliberately not ported:
//   moveit_ros/planning/planning_response_adapter_plugins/src/display_motion_path.cpp
//   (D1: rviz `MarkerArray`/`DisplayTrajectory` publishing, no other content)

//! §5 Phase 7's `moveit-planning`: the planner-agnostic
//! [`PlanningRequest`]/[`PlanningResponse`] types every planner in this
//! workspace should eventually speak, and the default request/response
//! adapter chain moveit2 runs around a planner's own `solve()` call —
//! before this round, neither existed anywhere in this workspace (only
//! `moveit-planners-sbp`, one concrete planner with its own
//! planner-specific request type, did).
//!
//! # Deviation from `moveit-planners-sbp::registry`'s existing types
//!
//! `moveit-planners-sbp::registry` currently defines its own
//! `PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`.
//! Architecturally that is backwards — a *planner* crate should depend on
//! the planning-request vocabulary, not define it — and sbp's own module doc
//! says so directly. This round fixes the layering going forward without
//! touching sbp itself (off limits this round: p1-robotmodel is running
//! Phase 7 benchmarks on that crate). Relocating sbp onto these types is
//! deferred to next round, precondition: the p1-robotmodel branch merges
//! first (see this crate's own doc below and the round report).
//!
//! Two shape differences from sbp's existing types are not cosmetic; each is
//! forced by what this round's adapters actually need to read or write:
//!
//! - [`PlanningRequest::goal_constraints`] is `Vec<KinematicConstraintSet>`
//!   (already-typed constraint sets — a state satisfying *any one* set is an
//!   acceptable goal), not sbp's `goal: Vec<CompoundValue>` (one concrete
//!   state). [`crate::request_adapters::CheckForStackedConstraints`] counts
//!   `position_constraints`/`orientation_constraints` per set and
//!   [`crate::request_adapters::ResolveConstraintFrames`] inspects
//!   constraint frames — neither operation is expressible against a
//!   concrete state, which by definition has already thrown that structure
//!   away. sbp's concrete-state shape remains correct for what actually
//!   consumes it today (`moveit-planners-sbp::rrt_connect::rrt_connect`
//!   takes one fixed goal, not a region) — see "What this does *not* yet
//!   close" below for why the two shapes still cannot be unified this round.
//! - [`PlanningResponse::trajectory`] is a [`moveit_trajectory::RobotTrajectory`]
//!   (one `duration_from_previous` per waypoint), not sbp's
//!   `Vec<RobotState<'m>>` (bare waypoints, no timing).
//!   [`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
//!   [`crate::response_adapters::AddTimeOptimalParameterization`] both exist
//!   to *compute* per-waypoint timing; a response type with nowhere to put a
//!   duration cannot carry what they produce. See [`PlanningResponse`]'s own
//!   doc comment for the full accounting, including why the upstream
//!   `if (!res.trajectory)` null check every response adapter opens with is
//!   not ported here.
//!
//! Planner-specific tuning (sbp's own `RrtConnectParams`/`resolution`/
//! `seed` fields, or an eventual STOMP planner's iteration count) is
//! deliberately *not* part of [`PlanningRequest`] either, matching upstream
//! more closely than sbp's current shape does: real MoveIt2 configures a
//! `PlannerManager`'s per-group `PlannerConfigurationSettings` once, at
//! setup time, not fresh on every `MotionPlanRequest` — the request itself
//! only ever carries planner-agnostic controls
//! (`num_planning_attempts`/`allowed_planning_time`/`planner_id`, none of
//! which any adapter in this crate reads yet, so none is ported speculatively
//! here). sbp's own module doc already explains why a
//! `HashMap<String, String>` config bag was rejected in favor of a
//! concretely-typed `RrtConnectParams` field — that reasoning holds for a
//! single always-known planner, but stops scaling once one [`PlanningRequest`]
//! shape has to serve more than one planner algorithm (RRT-Connect today,
//! eventually STOMP). The fix is not a bigger bag on [`PlanningRequest`]; it
//! is keeping planner tuning where sbp's registry work next round will put
//! it: on each concrete `PlannerManager`-analogous type's own construction,
//! the same place upstream's `PlannerConfigurationSettings` already lives.
//!
//! # What this does *not* yet close: `moveit-constraints`'s sampler has
//! nowhere to hand its output
//!
//! `moveit-planners-sbp::registry`'s own module doc (as of round 14) already
//! corrected an older claim that `constraint_samplers` was never ported —
//! it has been: [`moveit_constraints::ConstraintSampler`]/
//! [`moveit_constraints::JointConstraintSampler`]/
//! [`moveit_constraints::UnionConstraintSampler`]
//! (`moveit-constraints/src/sampler.rs`),
//! [`moveit_constraints::IkConstraintSamplerAdapter`]
//! (`moveit-constraints/src/ik_sampler.rs`), and
//! [`moveit_constraints::select_default_sampler`]
//! (`moveit-constraints/src/constraint_sampler_manager.rs`) all exist and
//! `moveit-constraints` depends on `moveit-kinematics` to run IK-backed
//! sampling. Checked directly for this round (not assumed from that note
//! still being accurate): `crates/moveit-constraints/src/` does contain all
//! three files, confirming the port itself is real.
//!
//! What is verified still missing, by inspection of every call site of
//! [`moveit_constraints::select_default_sampler`] and
//! [`moveit_constraints::ConstraintSampler`] workspace-wide (`rg
//! 'select_default_sampler|ConstraintSampler' --type rust`): **no caller
//! anywhere in this workspace invokes them outside `moveit-constraints`'s
//! own tests.** In particular, this crate's own
//! [`PlanningRequest::goal_constraints`] is exactly the
//! `Vec<KinematicConstraintSet>` a sampler would consume to produce
//! candidate goal states — but nothing in this round's six adapters, and
//! nothing in sbp's `rrt_connect` (whose signature still takes one fixed
//! `goal: S::State`, not a region or a resampleable source — unchanged from
//! round 14's finding), calls the sampler to turn one into the other. So the
//! answer to "can a caller of this crate express a pose (position/
//! orientation) goal now?" is: **the sampler exists and can build one, but
//! nothing wires its output to a planner** — not "still unported" (false,
//! per the file listing above) and not "yes, end to end" (also false, per
//! the zero-call-sites check above). This crate's [`PlanningRequest`] is
//! shaped so that connection has somewhere to plug in
//! ([`PlanningRequest::goal_constraints`] instead of a concrete state) —
//! wiring it through to a concrete planner's `solve()` is planner-specific
//! work for whichever crate owns that planner (sbp's `rrt_connect`, or a
//! future STOMP integration), not something a planner-agnostic adapter
//! chain can do on its own.
//!
//! # The adapter chain
//!
//! [`PlanningRequestAdapter`]/[`PlanningResponseAdapter`] replace
//! `planning_interface::PlanningRequestAdapter`/`PlanningResponseAdapter`;
//! [`run_request_adapters`]/[`run_response_adapters`] replace the chain a
//! `planning_pipeline::PlanningPipeline` runs each adapter through in order,
//! short-circuiting on the first failure — the "adapter chain" §5 Phase 7
//! named but never built. A caller assembles its own
//! `&[Box<dyn PlanningRequestAdapter>]` (see each module's tests for an
//! example); this crate does not hand out a fixed default ordering, since no
//! caller of one exists yet to have an opinion on it.
//!
//! Both traits specialize directly to [`moveit_collision::ParryCollisionEnv`]
//! rather than being generic over `E: CollisionEnv<..>`, for the same reason
//! `moveit-planners-sbp::registry::PlannerManager` does (see that trait's own
//! "Deviation from upstream" doc): a generic *type* parameter on a trait
//! method breaks `dyn` object-safety, and `ParryCollisionEnv` is the only
//! [`moveit_collision::CollisionEnv`] implementation anywhere in this
//! workspace.
//!
pub mod error;
pub mod request;
pub mod request_adapters;
pub mod response;
pub mod response_adapters;

pub use error::{RequestAdapterError, ResponseAdapterError};
pub use request::{PlanningRequest, WorkspaceBounds};
pub use response::PlanningResponse;

use moveit_collision::ParryCollisionEnv;
use moveit_scene::PlanningScene;

/// Replaces `planning_interface::PlanningRequestAdapter`.
pub trait PlanningRequestAdapter {
    /// `getDescription`.
    fn description(&self) -> &'static str;

    /// `adapt(planning_scene, req)`. `SUCCESS` is `Ok(())`; every other
    /// `moveit_msgs::msg::MoveItErrorCodes` this crate's adapters return is a
    /// [`RequestAdapterError`] variant instead.
    fn adapt<'m>(
        &self,
        scene: &mut PlanningScene<'m>,
        env: &ParryCollisionEnv,
        request: &mut PlanningRequest,
    ) -> Result<(), RequestAdapterError>;
}

/// Replaces `planning_interface::PlanningResponseAdapter`.
pub trait PlanningResponseAdapter {
    /// `getDescription`.
    fn description(&self) -> &'static str;

    /// `adapt(planning_scene, req, res)`. Upstream's `void` return (which
    /// sets `res.error_code` in place) becomes a `Result` here so
    /// [`run_response_adapters`] can short-circuit the same way
    /// [`run_request_adapters`] does.
    fn adapt<'m>(
        &self,
        scene: &mut PlanningScene<'m>,
        env: &ParryCollisionEnv,
        request: &PlanningRequest,
        response: &mut PlanningResponse<'m>,
    ) -> Result<(), ResponseAdapterError>;
}

/// Runs every adapter in `chain` against `request`, in order, stopping at
/// the first [`RequestAdapterError`]. Replaces the request half of
/// `PlanningPipeline::generatePlan`'s adapter loop
/// (`planning_pipeline.cpp`), minus the parts of that loop D1 already
/// excludes (publishing a `moveit_msgs::msg::MotionPlanRequest` display
/// event) or this round's scope does not reach (the terminal call into a
/// `moveit-planners-sbp`-style `PlannerManager` itself, which stays a
/// separate step a caller takes after this function returns `Ok`).
pub fn run_request_adapters<'m>(
    chain: &[Box<dyn PlanningRequestAdapter>],
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    request: &mut PlanningRequest,
) -> Result<(), RequestAdapterError> {
    for adapter in chain {
        adapter.adapt(scene, env, request)?;
    }
    Ok(())
}

/// Runs every adapter in `chain` against `response`, in order, stopping at
/// the first [`ResponseAdapterError`]. Replaces the response half of
/// `PlanningPipeline::generatePlan`'s adapter loop.
pub fn run_response_adapters<'m>(
    chain: &[Box<dyn PlanningResponseAdapter>],
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    request: &PlanningRequest,
    response: &mut PlanningResponse<'m>,
) -> Result<(), ResponseAdapterError> {
    for adapter in chain {
        adapter.adapt(scene, env, request, response)?;
    }
    Ok(())
}
