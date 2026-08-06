// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp

//! [`PlannerManager`] and [`PlanningContext`]: the two-step
//! "build a context, then solve it" interface every concrete planner
//! implements, ported from `planning_interface::PlannerManager`
//! (`planning_interface.hpp:148-211`) and
//! `planning_interface::PlanningContext` (`:78-143`).
//!
//! # Why these live here and not in a planner crate
//!
//! Upstream puts all four of `MotionPlanRequest`, `MotionPlanResponse`,
//! `PlanningContext` and `PlannerManager` in one package
//! (`moveit_core/planning_interface`): `planning_interface.hpp` includes
//! `planning_request.hpp` and `planning_response.hpp` on its first two
//! lines (`:40-41`), because the two traits are *defined* by the request
//! and response types they move between. This module is that unit's Rust
//! half, and [`crate::request::PlanningRequest`]/
//! [`crate::response::PlanningResponse`] are the other. Splitting them
//! across crates is what produced the defect this module exists to close —
//! `moveit-planners-sbp::registry` used to declare its own
//! `PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`
//! set that shared only *names* with this crate's, so
//! [`crate::pipeline::generate_plan`] could not call the workspace's only
//! concrete planner (PORTING-PLAN.md D8/§140.2).
//!
//! Registration — going from a planner-id *string* to a
//! [`PlannerManager`] instance — is deliberately *not* here: it is
//! `moveit-planner-registry`'s job, one crate up, so that the
//! `linkme::distributed_slice` it needs keeps its `unsafe_code = "allow"`
//! confined to a crate with no other code in it (PORTING-PLAN.md §140.1).
//! Nothing in this module or in [`crate::pipeline`] resolves a name.
//!
//! # Deviations from upstream, one line each
//!
//! - `PlanningContext`'s `name_`/`group_`/`planning_scene_`/`request_`
//!   members and their `get`/`set` pairs (`:82-114`) — upstream's context is
//!   re-settable (`setPlanningScene`/`setMotionPlanRequest`) so one context
//!   object can be reused for a second query; every context here is built by
//!   [`PlannerManager::get_planning_context`] for exactly one query and
//!   dropped, so there is no second query to re-point it at and no reader
//!   for the getters.
//! - `terminate()`/`clear()` (`:126-129`) — asynchronous cancellation and
//!   reuse-after-clear, both of which need a caller on another thread
//!   holding the same context. [`PlanningContext::solve`] takes `&mut self`
//!   and runs to completion synchronously, so no such caller can exist.
//! - `solve(MotionPlanDetailedResponse&)` (`:122`) — the detailed response
//!   has no port on this side (`doc/port-coverage.md`'s
//!   `planning_response.hpp` row: its only counterpart is
//!   `moveit_planners_chomp::ChompSolution`, narrowed to what chomp fills).
//! - `initialize` (`:164-165`) — takes an `rclcpp::Node` and a ROS parameter
//!   namespace (D1); the per-planner configuration it reads is constructor
//!   arguments on the concrete manager here (e.g.
//!   `moveit_planners_sbp::RrtConnectManager`'s own tuning fields).
//! - `setPlannerConfigurations`/`getPlannerConfigurations`/
//!   `PlannerConfigurationMap` (`:56-72`, `:193-199`) — same: a
//!   `map<string, map<string, string>>` filled from ROS parameters, whose
//!   typed equivalent is the concrete manager's own fields.
//! - `canServiceRequest` (`:190`) and `getPlanningAlgorithms` (`:172`) —
//!   `PlanningPipeline::generatePlan` calls neither (`planning_pipeline.cpp:294-330`
//!   calls `getPlanningContext`, `solve` and `getDescription` only); their
//!   upstream callers are `move_group`'s `query_planners` service capability
//!   (`query_planners_service_capability.cpp:98,102`) and the concrete
//!   plugins' own early-outs. Adding either here would be API surface with
//!   no caller in this workspace.

use moveit_collision::ParryCollisionEnv;
use moveit_scene::PlanningScene;

use crate::request::PlanningRequest;
use crate::response::PlanningResponse;

/// Opaque planner failure: a [`PlannerManager`] implementation boxes
/// whatever error its own concrete planner produced (e.g.
/// `moveit_planners_sbp::PlanError`) into this. This crate cannot name a
/// concrete planner error type — it
/// does not, and must not, depend on any concrete planner crate; the
/// dependency runs the other way (see this module's doc, "Why these live
/// here and not in a planner crate").
///
/// Replaces the `moveit_msgs::msg::MoveItErrorCodes& error_code` out
/// parameter of `getPlanningContext` (`planning_interface.hpp:183`) and the
/// `res.error_code` a `solve()` sets (`planning_response.hpp:63`) alike.
pub type PlanError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// A planning query bound to a scene, ready to run. Ports
/// `planning_interface::PlanningContext` (`planning_interface.hpp:78-143`);
/// see this module's doc for the members and methods that have no
/// counterpart here and why.
pub trait PlanningContext<'m> {
    /// Runs the query to completion. Ports
    /// `virtual void solve(MotionPlanResponse& res)` (`:118`), with
    /// upstream's mutated-in-place output parameter and its `error_code`
    /// replaced by one `Result`.
    fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError>;
}

/// Builds a [`PlanningContext`] for a `(scene, request)` pair. Ports
/// `planning_interface::PlannerManager` (`planning_interface.hpp:148-211`);
/// see this module's doc for the methods that have no counterpart here and
/// why.
///
/// # Deviation from upstream: specialized to [`ParryCollisionEnv`]
///
/// Upstream's `PlannerManager` is not itself generic over the collision
/// checker — the scene it is given already owns one.
/// [`moveit_scene::PlanningScene`] is generic over `E: CollisionEnv<..>`
/// instead of owning one (see that type's own doc comment), which would
/// force [`PlannerManager::get_planning_context`] to be generic over `E`
/// too — and a generic *type* parameter on a trait method breaks `dyn`
/// object-safety (a generic *lifetime* parameter, like this method's
/// `'a`/`'m`, does not). [`ParryCollisionEnv`] is the only
/// [`moveit_collision::CollisionEnv`] implementation anywhere in this
/// workspace (PORTING-PLAN.md D4.5: parry3d-f64 replaces FCL+Bullet
/// outright, not as one plugin among several), so specializing directly to
/// it costs nothing today and keeps this trait usable as
/// `dyn PlannerManager` — which `moveit-planner-registry`'s slice and
/// [`crate::pipeline::generate_plan`]'s `planners` parameter both require.
pub trait PlannerManager {
    /// This manager's name. Plays the role of upstream's
    /// `getDescription()` (`planning_interface.hpp:168`, the string
    /// `generatePlan` logs at `planning_pipeline.cpp:311,318,324` and that
    /// [`crate::pipeline::PipelineError::Planner`] carries here), and is
    /// simultaneously the key a caller matches on when scanning
    /// `moveit-planner-registry`'s `PLANNER_MANAGERS` — upstream keeps
    /// those two apart (a pluginlib class name vs. a free-form
    /// description), this port deliberately does not, so that a name found
    /// in the registry and a name reported in an error are the same string.
    fn name(&self) -> &'static str;

    /// Builds a [`PlanningContext`] that will plan `request` against
    /// `scene`, using `env` for collision checking. Ports
    /// `getPlanningContext(planning_scene, req, error_code)`
    /// (`planning_interface.hpp:181-183`), whose "empty ptr is returned and
    /// error code is set" contract is this `Result`.
    ///
    /// Fails only if `request` cannot be resolved against
    /// `scene.robot_model()` (e.g. an unknown
    /// [`PlanningRequest::group_name`]); planning failure itself surfaces
    /// from [`PlanningContext::solve`], matching upstream's own split
    /// between context construction (`planning_pipeline.cpp:306-315`) and
    /// solving (`:318-329`).
    ///
    /// `request` is borrowed, not moved: upstream takes
    /// `const MotionPlanRequest&` and copies what the context keeps into
    /// its own `request_` member (`:142`), and
    /// [`crate::pipeline::generate_plan`] needs the request back afterwards
    /// to feed the response-adapter chain and the next planner in the
    /// chain.
    fn get_planning_context<'a, 'm>(
        &self,
        scene: &'a mut PlanningScene<'m>,
        env: &'a ParryCollisionEnv,
        request: &PlanningRequest,
    ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError>;
}
