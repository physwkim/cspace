// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_ros/planning/planning_pipeline/include/moveit/planning_pipeline/planning_pipeline.hpp
//   moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp

//! [`generate_plan`] replaces `planning_pipeline::PlanningPipeline::generatePlan`
//! (`planning_pipeline.cpp:251-374`): the orchestration around
//! [`run_request_adapters`]/[`run_response_adapters`] that runs a planner (or
//! a sequential chain of planners) between them. Round 19's chain doctest
//! (see the crate doc's "# The adapter chain") built this by hand; this
//! module is the type that should have done it.
//!
//! # Planners are supplied by the caller, not looked up by name
//!
//! Upstream resolves `pipeline_parameters_.planning_plugins` (a list of
//! names) against `planner_map_`, a `pluginlib`-populated
//! `unordered_map<string, PlannerManagerPtr>` (`planning_pipeline.hpp:263`).
//! This workspace's D4 compile-time equivalent is `cspace_planning::planner_registry`'s
//! `PLANNER_MANAGERS` `distributed_slice`. [`generate_plan`] does not depend
//! on it and takes planners as `&[Box<dyn PlannerManager>]` instead:
//! **name-to-implementation resolution is a concern orthogonal to the
//! pipeline**, not part of what this function does. The pipeline's actual
//! substance — the adapter chains and their failure/exit rules — needs no
//! registry at all; only a *caller* who wants to go from a planner-id string
//! to a boxed [`PlannerManager`] does, and that caller layers
//! `cspace_planning::planner_registry` on top of this function (as
//! `ros/cspace-ros`'s `/move_action` server does).
//!
//! What [`generate_plan`] *does* depend on is [`crate::planner`], this
//! crate's own port of upstream's `planning_interface::PlannerManager`/
//! `PlanningContext` — the same package upstream defines
//! `MotionPlanRequest`/`MotionPlanResponse` in, so a planner speaks this
//! crate's [`PlanningRequest`]/[`PlanningResponse`] by construction rather
//! than needing a conversion at the crate boundary. Before D8/§140 that was
//! not true: `cspace_planners::sbp::registry` declared its own set of all
//! four names, and this function could not call the workspace's only
//! concrete planner at all.
//!
//! # Seven semantics invisible to a bare diff read
//!
//! ## 1. Planner-chain feedforward (`planning_pipeline.cpp:295-302`)
//!
//! `pipeline_parameters_.planning_plugins` can name more than one planner,
//! run in sequence — not a fallback chain (try the next on failure), a
//! *pipeline* (every planner must succeed). Before calling *every* planner,
//! upstream checks `if (res.trajectory)` (cpp:299) and, only if it's set,
//! overwrites the mutable request: `mutable_request.trajectory_constraints.constraints
//! = getTrajectoryConstraints(res.trajectory)` (cpp:301) — the gate is on
//! whether `res` (a caller-supplied `MotionPlanResponse&`, not locally
//! constructed) already carries a trajectory, not on loop position. In the
//! normal case a freshly-called pipeline's `res` starts empty, so this is a
//! no-op for the first planner and fires from the second one on, once a
//! prior planner has succeeded — but a caller who pre-populates `res.trajectory`
//! before calling would make it fire on the first planner too.
//! [`generate_plan`] below reproduces the same state-based trigger with a
//! private `trajectory_constraints_for` helper (porting
//! `getTrajectoryConstraints`, `planning_pipeline.cpp:57-79`) called between
//! successive [`crate::planner::PlanningContext::solve`] calls, never before the first — this port's
//! `response` starts empty on every call, so the two conditions coincide
//! here even though upstream's gate is not itself positional. See
//! "Semantic-1 tests" below for the two-planner regression that proves the
//! second request is actually rewritten, not merely eligible to be.
//!
//! ## 2. The response-adapter chain runs only on planner success (`cpp:332-351`)
//!
//! Upstream wraps the whole response-adapter loop in `if (res.error_code)`
//! (cpp:333) — a successful `error_code`, i.e. every planner in the chain
//! already succeeded. In this port that guarantee is structural rather than
//! a checked condition: [`run_response_adapters`] is only ever reached after
//! every `?` in the planner loop above it has already returned `Ok`, so
//! there is no path to it on a planner failure to guard against — see
//! "Semantic 5" below for why this generalizes to the whole function.
//!
//! ## 3. The upstream `active_` flag has nothing to observe here
//!
//! Upstream sets `active_ = true` at entry (cpp:259) and resets it to
//! `false` on every one of five exit paths — request-adapter failure
//! (cpp:289), context-creation failure (cpp:313), planner failure (cpp:327),
//! response-adapter failure (cpp:347), and the `catch (std::exception&)`
//! block (cpp:357) — before the final `active_ = false` at cpp:372. Six
//! manual resets guarding one boolean is exactly the "strong state
//! transition must pass through a single finalizer" shape this project's
//! own conduct rules call out (see the "Strong state transitions" section):
//! upstream needs `isActive()` (`planning_pipeline.hpp:217-220`) because
//! `generatePlan` runs on a caller's thread while the *same* `PlanningPipeline`
//! object can be asked `isActive()`/`terminate()` from another (ROS service
//! callbacks, typically) — a concurrent, persistent object with observers.
//! [`generate_plan`] is a free function taking everything it touches as a
//! parameter and returning an owned [`PlanningResponse`]; nothing outlives
//! one call, nothing else holds a reference to observe mid-call, and this
//! crate defines no `terminate`-equivalent for the same reason
//! [`crate::planner`]'s own module doc gives for omitting one:
//! synchronous, single-caller, no concurrent observer to serve. Modeling an
//! `active_`/`is_active()` pair here would be state with no reader —
//! speculative API surface this project's own rules reject. What upstream's
//! six manual resets guard against — an early return that forgets to flip
//! the flag — is instead enforced structurally: every fallible step below
//! uses `?`, so [`generate_plan`] has exactly one exit path per outcome (an
//! early `Err` via `?`, or the final `Ok` at the bottom), not six
//! hand-written ones. If this crate later grows a genuinely concurrent,
//! long-lived pipeline object, that object — not this free function — is
//! where an `is_active` would belong.
//!
//! ## 4. `planner_id` fallback (`cpp:361-369`)
//!
//! If the planner leaves [`crate::PlanningResponse::planner_id`] empty,
//! [`generate_plan`] fills it from [`crate::PlanningRequest::planner_id`]
//! (upstream additionally logs a `RCLCPP_WARN`, cpp:364-367 — D1, no logging
//! facade in this port, same deviation every adapter in this crate already
//! takes).
//!
//! ## 5. The final result reflects the last state, not unconditional success (`cpp:372-373`)
//!
//! Upstream's final line is `return static_cast<bool>(res)`, not `return true`
//! — even after every adapter and planner nominally ran, the *last* recorded
//! `error_code` is what is trusted, not the fact that control reached the
//! end. In this port that check does not need to exist as separate code:
//! every fallible step already returns early via `?` the moment it fails, so
//! reaching [`generate_plan`]'s final `Ok(response)` is *itself* the proof
//! that every step succeeded — there is no way to arrive there with a
//! stale-but-unchecked failure the way a mutated-in-place `res.error_code`
//! could go stale in upstream's C++. Asserting this in prose instead of
//! reproducing upstream's redundant check is not a gap; it is what "the
//! type system enforces the invariant, not a runtime check" (this project's
//! "Structural fix vs. clever patch" guidance) looks like when the language
//! already gives you the finalizer for free.
//!
//! ## 6. `start_state` is captured once, before the planner ever runs
//!
//! `cspace_planners::sbp::planning_scene_validity::PlanningSceneValidityChecker`
//! (a downstream crate's type — the dependency runs sbp → here, never the
//! reverse) documents
//! that it does not restore `scene`'s
//! current state after a validity check, by design — restoring it would add
//! a full-state clone to each of the hundreds of thousands of calls one
//! planning query makes. Its own doc states the caller-side obligation this
//! creates: "a caller that needs the pre-planning state preserved clones it
//! once, itself, before handing the scene to this type."
//! [`generate_plan`] is that caller, and fulfills the obligation here: it
//! clones `scene.current_state()` exactly once per call — after
//! [`run_request_adapters`] returns (a request adapter can mutate
//! `scene.current_state()`, e.g. a bounds-clamping one, so the value
//! captured is the state the planner(s) below actually start from, not the
//! caller's original pre-adapter one) and before the first
//! [`PlannerManager::get_planning_context`] is
//! called — and stores it as
//! [`PlanningResponse::start_state`] once the response is otherwise
//! complete. One clone for the whole query, not one per validity check, is
//! exactly the cost `PlanningSceneValidityChecker`'s own doc says this
//! contract is designed to avoid paying per-call.
//!
//! ## 7. `start_state` is applied to the scene once, by this function alone
//!
//! Upstream has no single site for this: every reader of
//! `req.start_state` re-derives the planning start state itself, from the
//! same two lines (`check_start_state_bounds.cpp:87-88`,
//! `check_start_state_collision.cpp:74-75`, and through
//! `PlanningScene::getCurrentStateUpdated` at
//! `planning_context_manager.cpp:586`, `stomp_moveit_planning_context.cpp:226`,
//! `chomp_planner.cpp:77-78`). Five re-derivations of one value is a shape
//! this project's own conduct rules name — an invariant with no owner — and
//! the port does not need it: [`crate::PlanningRequestAdapter`] and
//! [`crate::PlannerManager`] both already receive `&mut PlanningScene` and
//! already read the start state off `scene.current_state()`
//! ([`crate::request_adapters::CheckStartStateBounds`] and
//! [`crate::request_adapters::CheckStartStateCollision`] do exactly that).
//!
//! **Invariant: once [`generate_plan`] has been entered, `scene`'s current
//! state IS the requested start state.** [`generate_plan`] establishes it by
//! calling [`crate::StartState::apply_to`] on `scene.current_state_mut()`
//! before `request_chain` runs — before, because upstream's own
//! `CheckStartStateBounds` sees `getCurrentState()` already overlaid with
//! `req.start_state` and may then correct it (`check_start_state_bounds.cpp:196`
//! writes the corrected state back into `req.start_state`; here the correction
//! lands in the scene, which is where every later reader looks). No other site
//! in this crate applies [`crate::PlanningRequest::start_state`], and no
//! adapter or [`crate::PlannerManager`] needs to: reading
//! `scene.current_state()` is already reading it.
//!
//! The cost of the ownership move is that `scene` is left holding the
//! requested start state rather than the caller's — the same already-documented
//! non-restoration "Semantic 6" and
//! `scene_current_state_is_allowed_to_differ_from_start_state_after_generate_plan_returns`
//! record for the planner's own side effects, now reached one step earlier.
//!
//! # Deviation: zero planners is `Err`, not an unset response
//!
//! Upstream's `res` is a caller-supplied, mutated-in-place output parameter,
//! so an empty `pipeline_parameters_.planning_plugins` list still leaves the
//! caller holding *some* `MotionPlanResponse` object (with `error_code`
//! unset, so `static_cast<bool>(res)` is `false` — see "Semantic 5"). This
//! port has nothing to hand back: [`PlanningResponse`] carries no `Option`
//! around its [`PlanningResponse::trajectory`] (see that type's own doc,
//! "No `Option`" — a trajectory is never absent by construction), so there
//! is no sensible empty value to construct. [`generate_plan`] returns
//! [`PipelineError::NoPlanners`] instead — the same "make the illegal state
//! unrepresentable rather than hand back a hollow success-shaped value"
//! choice [`PlanningResponse`] itself already made.
//!
//! # D1: excluded, one line each
//!
//! - `publishPipelineState` (`planning_pipeline.hpp:250`) — ROS progress
//!   publishing (`moveit_msgs::msg::MotionPlanRequest`/`MotionPlanResponse`
//!   over a topic); no non-ROS content.
//! - `publish_received_requests` (`generatePlan`'s fourth parameter,
//!   `planning_pipeline.hpp:187`) — exists only to trigger the above.
//! - `node_` (`planning_pipeline.hpp:257`) and the `RCLCPP_INFO`/
//!   `RCLCPP_ERROR`/`RCLCPP_WARN` calls throughout `generatePlan` — no logging
//!   facade anywhere in this port (see every adapter's own module doc).
//! - `pipeline_parameters_` (`planning_pipeline.hpp:259`) — the
//!   ROS-parameter-sourced config bag holding, among other things, the
//!   plugin-name lists a registry resolves; see "Owned by a named planner
//!   chain, not by this function" below for the parts of its surface that
//!   are deferred rather than dropped.
//! - `terminate()` (`planning_pipeline.hpp:190`, `cpp:376-385`) — same
//!   reasoning [`crate::planner`]'s own module doc
//!   already gives for omitting a `terminate`/`clear` pair: nothing in this
//!   port is asynchronous or persistent enough for another caller to ever
//!   need to interrupt it.
//! - The whole deprecated block (`planning_pipeline.hpp:134-173`):
//!   `displayComputedMotionPlans`/`publishReceivedRequests`/
//!   `checkSolutionPaths`/their `get*` counterparts/the six-argument
//!   `generatePlan` overload/`getPlannerPluginName()`/the zero-argument
//!   `getPlannerManager()` — upstream's own doc comment marks this block
//!   `BEGIN/END BLOCK OF DEPRECATED FUNCTIONS`; every one of them is either a
//!   no-op stub (`return false`/`return false`) already superseded by a
//!   current-block member, or an alias for a function named below under
//!   "Decided after registry relocation".
//!
//! # Owned by a named planner chain, not by this function
//!
//! `getPlannerPluginNames`/`getRequestAdapterPluginNames`/
//! `getResponseAdapterPluginNames` (`hpp:193-208`) read `pipeline_parameters_`,
//! and `getPlannerManager` (`hpp:229-236`) reads `planner_map_` — state a
//! name-to-implementation registry owns, not state this call-scoped
//! function has anywhere to keep between calls. `getName` (`hpp:222-226`)
//! is a separate case: it returns `parameter_namespace_`, a third field
//! this port also has no long-lived home for, so it belongs with this group
//! for the same "not part of `generate_plan`'s scope today" reason even
//! though it does not read either container. `cspace_planning::planner_registry` now
//! holds the slice these would read from, but nothing in this workspace
//! owns a *named, configured* planner chain — the object those four
//! accessors describe — so they stay unported; they are not part of
//! [`generate_plan`]'s own scope.

use crate::constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use crate::scene::PlanningScene;
use cspace_collision::ParryCollisionEnv;
use cspace_core::trajectory::RobotTrajectory;

use crate::error::{RequestAdapterError, ResponseAdapterError};
use crate::planner::{PlanError, PlannerManager};
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;
use crate::{
    PlanningRequestAdapter, PlanningResponseAdapter, run_request_adapters, run_response_adapters,
};

/// Why [`generate_plan`] failed. Replaces the untyped `bool`/`res.error_code`
/// upstream's `generatePlan` returns.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum PipelineError {
    /// A [`PlanningRequestAdapter`] in `request_chain` rejected `request`.
    #[error(transparent)]
    Request(#[from] RequestAdapterError),
    /// A [`PlannerManager`] failed — either building its
    /// [`crate::planner::PlanningContext`] (upstream's `nullptr` context,
    /// `cpp:308-315`) or
    /// solving it (`res.error_code` unset after `solve`, `cpp:323-328`);
    /// both upstream branches log the same way and abort the pipeline, so
    /// they share one variant here. `planner` is that planner's own
    /// [`PlannerManager::name`], matching how upstream's `RCLCPP_ERROR` at
    /// `cpp:325-326` names the failing planner.
    #[error("planner '{planner}' failed: {source}")]
    Planner {
        /// The failing planner's [`PlannerManager::name`].
        planner: &'static str,
        /// The boxed underlying planner error.
        #[source]
        source: PlanError,
    },
    /// A [`PlanningResponseAdapter`] in `response_chain` rejected the
    /// planned response.
    #[error(transparent)]
    Response(#[from] ResponseAdapterError),
    /// `planners` was empty. See this module's doc, "Deviation: zero
    /// planners is `Err`, not an unset response".
    #[error("generate_plan was called with no planners")]
    NoPlanners,
    /// [`crate::PlanningRequest::start_state`] could not be written onto
    /// `scene`'s current state — it names a variable this scene's
    /// [`cspace_core::model::RobotModel`] does not have. Upstream reaches the same
    /// condition as a `moveit::Exception` thrown out of
    /// `RobotModel::getVariableIndex` inside `setVariablePositions`
    /// (`robot_state.cpp:395-406`), which `generatePlan`'s own
    /// `catch (std::exception&)` (`planning_pipeline.cpp:353-359`) turns into
    /// a failed `res`. Separate from [`PipelineError::Feedforward`] despite
    /// both wrapping a [`cspace_core::error::Error`]: they are two different steps,
    /// and a shared variant could not say which one rejected.
    #[error("failed to apply the requested start state: {0}")]
    StartState(#[source] cspace_core::error::Error),
    /// The private `trajectory_constraints_for` helper (the "Semantic 1"
    /// feedforward step) failed to look up the previous planner's
    /// group/joints/waypoint positions. Upstream's `getTrajectoryConstraints`
    /// has no failure path
    /// at all (`cpp:57-79`) because the C++ object graph it reads
    /// guarantees those lookups always succeed; this port's equivalent
    /// lookups are typed `Result`s (`cspace_core::error::Result`), and surfacing
    /// that as an error here — rather than `.expect()`-ing an upstream
    /// invariant this port cannot independently verify — is the same
    /// "moveit-rs prefers to surface as an error" choice
    /// [`crate::constraints::JointConstraint::new`]'s own doc already makes
    /// for a structurally analogous case.
    #[error("failed to build trajectory-constraints feedforward: {0}")]
    Feedforward(#[from] cspace_core::error::Error),
}

/// Ports `getTrajectoryConstraints` (`planning_pipeline.cpp:57-79`): one
/// [`KinematicConstraintSet`] per waypoint of `trajectory`, each holding one
/// [`Constraint::Joint`] per active joint of `trajectory`'s planning group —
/// see this module's doc, "Semantic 1: planner-chain feedforward".
///
/// # Weight and tolerance: the effective value, not the literal one
///
/// Upstream builds a default-constructed `moveit_msgs::msg::JointConstraint`
/// per joint and sets only `joint_name`/`position` (cpp:71-73); every other
/// field — `tolerance_above`/`tolerance_below`/`weight` — stays at the ROS
/// message default of `0.0`. That literal `0.0` is not what a
/// `kinematic_constraints::JointConstraint` actually evaluates once
/// configured: `JointConstraint::configure` substitutes `weight = 1.0`
/// whenever the incoming weight is `<= epsilon`
/// (`kinematic_constraint.cpp:263-270`). [`JointConstraint::new`] in this
/// port rejects a non-positive weight as an error rather than silently
/// substituting one (see its own doc, "moveit-rs prefers to surface as an
/// error"), so passing the literal `0.0` here would make every feedforward
/// call fail — this function passes `1.0`, the value upstream's own
/// substitution already arrives at, so the constraint this port builds
/// evaluates identically to upstream's, not merely compiles. Tolerances are
/// only *rejected* by one substitution path upstream — `JointConstraint::configure`
/// rejects a *negative* tolerance (`kinematic_constraint.cpp:146-151`), and
/// `0.0` is not negative, so that path leaves `0.0` alone. A second, uncited
/// path can still rewrite it: if the waypoint's own position plus/minus the
/// (here, zero) tolerance falls outside the joint's bounds,
/// `configure` silently substitutes `tolerance_above_`/`tolerance_below_`
/// to `f64::EPSILON` and clamps the position to the bound
/// (`kinematic_constraint.cpp:243-260`). For a trajectory whose every
/// waypoint already satisfies its own joint bounds — the only kind
/// [`trajectory_constraints_for`] is ever called on — that branch's guard
/// is false at every call, so `0.0` still passes through unchanged in
/// practice; the claim that upstream has *no* tolerance substitution at all
/// was wrong, not the claim that this function's own behavior matches it.
fn trajectory_constraints_for(
    scene: &PlanningScene<'_>,
    trajectory: &RobotTrajectory<'_>,
) -> Result<Vec<KinematicConstraintSet>, cspace_core::error::Error> {
    let model = scene.robot_model();
    let group = model.joint_model_group(trajectory.group_name())?;
    let joint_names = group.active_joint_names().to_vec();

    let mut trajectory_constraints = Vec::with_capacity(trajectory.way_point_count());
    for index in 0..trajectory.way_point_count() {
        let waypoint = trajectory.way_point(index)?;
        let mut set = KinematicConstraintSet::new();
        for joint_name in &joint_names {
            let position = waypoint.variable_position(joint_name)?;
            set.push(Constraint::Joint(JointConstraint::new(
                model, joint_name, position, 0.0, 0.0, 1.0,
            )?));
        }
        trajectory_constraints.push(set);
    }
    Ok(trajectory_constraints)
}

/// One planner's `getPlanningContext(...)` + `context->solve(res)`
/// (`planning_pipeline.cpp:306-329`), with upstream's two abort branches —
/// a `nullptr` context (`:308-315`) and an unset `res.error_code` after
/// `solve` (`:323-328`) — folded into the single [`PipelineError::Planner`]
/// they both log and abort identically for.
///
/// A separate function rather than inlined at [`generate_plan`]'s two call
/// sites because the context borrows `scene` mutably for as long as it
/// lives: returning the response by value ends that borrow at this
/// function's own return, which is what lets `generate_plan`'s loop hand
/// `scene` to `trajectory_constraints_for` and then to the next planner.
fn run_planner<'m>(
    planner: &dyn PlannerManager,
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    request: &PlanningRequest,
) -> Result<PlanningResponse<'m>, PipelineError> {
    let mut context = planner
        .get_planning_context(scene, env, request)
        .map_err(|source| PipelineError::Planner {
            planner: planner.name(),
            source,
        })?;
    context.solve().map_err(|source| PipelineError::Planner {
        planner: planner.name(),
        source,
    })
}

/// Replaces `PlanningPipeline::generatePlan` (`planning_pipeline.cpp:251-374`):
/// runs `request_chain`, then every planner in `planners` in sequence (with
/// the "Semantic 1" feedforward between successive planners), then
/// `response_chain`, then the "Semantic 4" `planner_id` fallback. See this
/// module's doc for the seven semantics ported here that a bare diff read
/// would miss, for what D1 excludes, and for what a named planner chain —
/// not this function — would own.
///
/// # Errors
///
/// See [`PipelineError`]'s variants: a request-adapter rejection, a planner
/// failure (including a feedforward lookup failure between two planners), a
/// response-adapter rejection, [`PipelineError::StartState`] if
/// `request.start_state` names a variable this scene's model lacks, or
/// [`PipelineError::NoPlanners`] if `planners` is empty.
pub fn generate_plan<'m>(
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    request_chain: &[Box<dyn PlanningRequestAdapter>],
    planners: &[Box<dyn PlannerManager>],
    response_chain: &[Box<dyn PlanningResponseAdapter>],
    mut request: PlanningRequest,
) -> Result<PlanningResponse<'m>, PipelineError> {
    let Some((first_planner, later_planners)) = planners.split_first() else {
        return Err(PipelineError::NoPlanners);
    };

    // Semantic 7: the one site that turns `request.start_state` into scene
    // state, before the adapter chain, so every later reader gets it by
    // reading `scene.current_state()` and none re-derives it.
    request
        .start_state
        .apply_to(scene.current_state_mut())
        .map_err(PipelineError::StartState)?;

    run_request_adapters(request_chain, scene, env, &mut request)?;
    let start_state = scene.current_state().clone();

    let mut response = run_planner(&**first_planner, scene, env, &request)?;

    for planner in later_planners {
        request.trajectory_constraints = trajectory_constraints_for(scene, &response.trajectory)?;
        response = run_planner(&**planner, scene, env, &request)?;
    }

    run_response_adapters(response_chain, scene, env, &request, &mut response)?;

    if response.planner_id.is_empty() {
        response.planner_id = request.planner_id.clone();
    }
    response.start_state = start_state;

    Ok(response)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use cspace_core::error::Error as MoveitError;
    use cspace_core::model::{MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_core::state::RobotState;

    use super::*;
    use crate::PlanningRequestAdapter;
    use crate::planner::PlanningContext;
    use crate::request::WorkspaceBounds;
    use crate::start_state::StartState;

    fn panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    fn request() -> PlanningRequest {
        PlanningRequest {
            group_name: "panda_arm".to_string(),
            goal_constraints: vec![],
            path_constraints: None,
            workspace_bounds: WorkspaceBounds::default(),
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            ..Default::default()
        }
    }

    /// A trajectory from `start` to a fixed `panda_joint1 = 0.4` goal,
    /// two waypoints, zero duration (timing is not what these tests check).
    fn two_waypoint_trajectory<'m>(
        model: &'m RobotModel,
        start: RobotState<'m>,
    ) -> RobotTrajectory<'m> {
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();
        let mut trajectory = RobotTrajectory::for_group_name(model, "panda_arm").unwrap();
        trajectory.add_suffix_way_point(start, 0.0).unwrap();
        trajectory.add_suffix_way_point(goal, 0.0).unwrap();
        trajectory
    }

    /// Always fails, so it never has to produce a real response — used by
    /// the "response adapter fails" boundary, where reaching this adapter at
    /// all already proves the planner loop succeeded.
    struct RejectingRequestAdapter;
    impl PlanningRequestAdapter for RejectingRequestAdapter {
        fn description(&self) -> &'static str {
            "RejectingRequestAdapter"
        }
        fn adapt<'m>(
            &self,
            _scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &mut PlanningRequest,
        ) -> Result<(), RequestAdapterError> {
            Err(RequestAdapterError::StartStateInvalid {
                adapter: self.description(),
            })
        }
    }

    /// Always succeeds without touching `request` — used as the first
    /// element of a 2-adapter `request_chain` to prove a later adapter's
    /// rejection is attributed to that adapter, not silently to this one.
    struct PassingRequestAdapter;
    impl PlanningRequestAdapter for PassingRequestAdapter {
        fn description(&self) -> &'static str {
            "PassingRequestAdapter"
        }
        fn adapt<'m>(
            &self,
            _scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &mut PlanningRequest,
        ) -> Result<(), RequestAdapterError> {
            Ok(())
        }
    }

    struct RejectingResponseAdapter;
    impl PlanningResponseAdapter for RejectingResponseAdapter {
        fn description(&self) -> &'static str {
            "RejectingResponseAdapter"
        }
        fn adapt<'m>(
            &self,
            _scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &PlanningRequest,
            _response: &mut PlanningResponse<'m>,
        ) -> Result<(), ResponseAdapterError> {
            Err(ResponseAdapterError::InvalidMotionPlan {
                adapter: self.description(),
                invalid_waypoints: vec![0],
            })
        }
    }

    /// Always succeeds without touching `response` — used as the first
    /// element of a 2-adapter `response_chain` to prove a later adapter's
    /// rejection is attributed to that adapter, not silently to this one.
    struct PassingResponseAdapter;
    impl PlanningResponseAdapter for PassingResponseAdapter {
        fn description(&self) -> &'static str {
            "PassingResponseAdapter"
        }
        fn adapt<'m>(
            &self,
            _scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &PlanningRequest,
            _response: &mut PlanningResponse<'m>,
        ) -> Result<(), ResponseAdapterError> {
            Ok(())
        }
    }

    /// The [`PlanningContext`] half of every double below. A real
    /// [`PlannerManager`] does work in `get_planning_context` that its
    /// `solve` then reuses (`cspace_planners::sbp::RrtConnectManager`
    /// resolves the group's state space there); none of these four doubles
    /// has any, so all four capture what they need into one closure and
    /// hand it over as this. One shared context type, rather than four
    /// near-identical ones, keeps each double to the single function it
    /// actually differs in.
    type SolveFn<'a, 'm> = Box<dyn FnMut() -> Result<PlanningResponse<'m>, PlanError> + 'a>;
    struct ClosureContext<'a, 'm>(SolveFn<'a, 'm>);
    impl<'m> PlanningContext<'m> for ClosureContext<'_, 'm> {
        fn solve(&mut self) -> Result<PlanningResponse<'m>, PlanError> {
            (self.0)()
        }
    }

    /// Plans by handing back a fixed two-waypoint trajectory to `panda_joint1
    /// = 0.4`, regardless of `request`.
    struct FixedGoalPlanner {
        name: &'static str,
        planner_id: &'static str,
    }
    impl PlannerManager for FixedGoalPlanner {
        fn name(&self) -> &'static str {
            self.name
        }
        fn get_planning_context<'a, 'm>(
            &self,
            scene: &'a mut PlanningScene<'m>,
            _env: &'a ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
            let planner_id = self.planner_id.to_string();
            Ok(Box::new(ClosureContext(Box::new(move || {
                let model = scene.robot_model();
                let mut start = RobotState::new(model);
                start.set_to_default_values();
                Ok(PlanningResponse {
                    start_state: start.clone(),
                    trajectory: two_waypoint_trajectory(model, start),
                    planner_id: planner_id.clone(),
                })
            }))))
        }
    }

    /// Always fails with an opaque boxed error — the "planner fails"
    /// boundary. Carries its own `name` so a chain can hold two
    /// distinctly-named failing planners: [`PipelineError::Planner`] is
    /// constructed at two call sites in [`generate_plan`] (the first
    /// planner and the `later_planners` loop), and telling them apart
    /// requires each occupying a different chain position under a
    /// different name.
    ///
    /// It fails in `solve`, not in `get_planning_context`: those are
    /// upstream's two distinct abort branches (`cpp:308-315` vs.
    /// `:323-328`) and [`run_planner`] maps both to the same
    /// [`PipelineError::Planner`], so which one a double takes does not
    /// change what these tests observe — see
    /// `context_construction_failure_is_reported_as_a_planner_failure` for
    /// the other branch's own coverage.
    struct FailingPlanner(&'static str);
    impl PlannerManager for FailingPlanner {
        fn name(&self) -> &'static str {
            self.0
        }
        fn get_planning_context<'a, 'm>(
            &self,
            _scene: &'a mut PlanningScene<'m>,
            _env: &'a ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
            let name = self.0;
            Ok(Box::new(ClosureContext(Box::new(move || {
                Err(Box::new(MoveitError::other(format!("{name} always fails"))) as PlanError)
            }))))
        }
    }

    /// Fails at context construction instead of at `solve`, covering
    /// upstream's *other* abort branch (a `nullptr` context,
    /// `cpp:308-315`). Both branches collapse into
    /// [`PipelineError::Planner`] here, which is exactly what
    /// `context_construction_failure_is_reported_as_a_planner_failure`
    /// pins down.
    struct UnbuildablePlanner(&'static str);
    impl PlannerManager for UnbuildablePlanner {
        fn name(&self) -> &'static str {
            self.0
        }
        fn get_planning_context<'a, 'm>(
            &self,
            _scene: &'a mut PlanningScene<'m>,
            _env: &'a ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
            Err(Box::new(MoveitError::other(format!(
                "{} cannot build a context",
                self.0
            ))))
        }
    }

    /// Records the `trajectory_constraints` of every request it is called
    /// with, then delegates to a [`FixedGoalPlanner`]-style plan. Used only
    /// by the feedforward test, where the assertion is "the second call's
    /// request.trajectory_constraints is non-empty", not merely "a second
    /// call happened". `Rc`, not a borrow, so two boxed instances can share
    /// one recording sink without fighting `Box<dyn PlannerManager>`'s
    /// implicit `'static` object-lifetime bound.
    struct RecordingPlanner {
        seen_trajectory_constraints_len: std::rc::Rc<std::cell::RefCell<Vec<usize>>>,
    }
    impl PlannerManager for RecordingPlanner {
        fn name(&self) -> &'static str {
            "RecordingPlanner"
        }
        fn get_planning_context<'a, 'm>(
            &self,
            scene: &'a mut PlanningScene<'m>,
            _env: &'a ParryCollisionEnv,
            request: &PlanningRequest,
        ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
            // Recorded here, not in `solve`: `get_planning_context` is
            // where the request reaches a planner, and it is the borrow
            // `request_` upstream copies into the context at the same
            // point (`planning_interface.hpp:142`).
            self.seen_trajectory_constraints_len
                .borrow_mut()
                .push(request.trajectory_constraints.len());
            Ok(Box::new(ClosureContext(Box::new(move || {
                let model = scene.robot_model();
                let mut start = RobotState::new(model);
                start.set_to_default_values();
                Ok(PlanningResponse {
                    start_state: start.clone(),
                    trajectory: two_waypoint_trajectory(model, start),
                    planner_id: String::new(),
                })
            }))))
        }
    }

    /// Mutates `scene`'s current state as a side effect of planning, the
    /// same way `cspace_planners::sbp::planning_scene_validity::PlanningSceneValidityChecker`
    /// leaves `scene` at whatever state its last validity check posed —
    /// see this module's doc, "Semantic 6". Used to prove
    /// [`PlanningResponse::start_state`] is captured *before* that
    /// mutation, not read back from a scene a planner may have since
    /// moved.
    struct SideEffectPlanner;
    impl PlannerManager for SideEffectPlanner {
        fn name(&self) -> &'static str {
            "SideEffectPlanner"
        }
        fn get_planning_context<'a, 'm>(
            &self,
            scene: &'a mut PlanningScene<'m>,
            _env: &'a ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<Box<dyn PlanningContext<'m> + 'a>, PlanError> {
            Ok(Box::new(ClosureContext(Box::new(move || {
                let model = scene.robot_model();
                let mut start = RobotState::new(model);
                start.set_to_default_values();
                let response = Ok(PlanningResponse {
                    start_state: start.clone(),
                    trajectory: two_waypoint_trajectory(model, start),
                    planner_id: String::new(),
                });
                scene
                    .current_state_mut()
                    .set_joint_positions("panda_joint1", &[1.0])
                    .unwrap();
                response
            }))))
        }
    }

    #[test]
    fn zero_planners_is_an_error() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![];
        let err = generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect_err("an empty planner slice must be rejected");
        assert!(matches!(err, PipelineError::NoPlanners));
    }

    #[test]
    fn request_adapter_failure_short_circuits_before_any_planner_runs() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> =
            vec![Box::new(RejectingRequestAdapter)];
        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        let err = generate_plan(&mut scene, &env, &request_chain, &planners, &[], request())
            .expect_err("a rejecting request adapter must abort before planning");
        // `PipelineError::Request(_)` is a thin wrapper around whatever
        // `run_request_adapters`'s loop propagates; `(_)` alone doesn't check
        // which chain element produced it. Checking `adapter` pins this to
        // the (only) chained adapter — see
        // `second_request_adapter_failure_is_attributed_to_the_second_adapter`
        // for the case where that matters.
        match err {
            PipelineError::Request(RequestAdapterError::StartStateInvalid { adapter }) => {
                assert_eq!(adapter, "RejectingRequestAdapter");
            }
            other => panic!("expected PipelineError::Request(StartStateInvalid), got {other:?}"),
        }
    }

    #[test]
    fn second_request_adapter_failure_is_attributed_to_the_second_adapter() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        // First adapter passes (exercising the chain's non-terminal path),
        // second rejects — the only element this test targets.
        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> = vec![
            Box::new(PassingRequestAdapter),
            Box::new(RejectingRequestAdapter),
        ];
        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        let err = generate_plan(&mut scene, &env, &request_chain, &planners, &[], request())
            .expect_err("the second request adapter's rejection must abort the pipeline");
        match err {
            PipelineError::Request(RequestAdapterError::StartStateInvalid { adapter }) => {
                assert_eq!(
                    adapter, "RejectingRequestAdapter",
                    "the rejection must be attributed to the adapter that actually rejected"
                );
            }
            other => panic!("expected PipelineError::Request(StartStateInvalid), got {other:?}"),
        }
    }

    #[test]
    fn planner_failure_short_circuits_before_response_adapters_run() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FailingPlanner("FirstPlanner"))];
        // If the response chain ran despite the planner failing, this
        // adapter's `Err` would surface as `PipelineError::Response`
        // instead of `PipelineError::Planner` below — the test would still
        // fail, just for the more informative reason of catching semantic 2
        // broken rather than merely "some Err came back".
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> =
            vec![Box::new(RejectingResponseAdapter)];
        let err = generate_plan(&mut scene, &env, &[], &planners, &response_chain, request())
            .expect_err("a failing planner must abort before response adapters run");
        // `planner` pins this to `generate_plan`'s *first*-planner
        // construction site (the call before the `later_planners` loop) —
        // see `a_later_planner_failure_is_attributed_to_that_planner` for
        // the loop's own site.
        match err {
            PipelineError::Planner { planner, .. } => assert_eq!(planner, "FirstPlanner"),
            other => panic!("expected PipelineError::Planner, got {other:?}"),
        }
    }

    #[test]
    fn a_later_planner_failure_is_attributed_to_that_planner() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        // First planner succeeds, so the failure below is only reachable
        // through the `later_planners` loop's `PipelineError::Planner`
        // construction site -- distinct from the first-planner site
        // `planner_failure_short_circuits_before_response_adapters_run`
        // exercises. If the loop's `map_err` misattributed the failure to
        // the first planner (or any other name), this assertion -- not a
        // bare `matches!(err, PipelineError::Planner { .. })` -- is what
        // would catch it.
        let planners: Vec<Box<dyn PlannerManager>> = vec![
            Box::new(FixedGoalPlanner {
                name: "FirstPlanner",
                planner_id: "fixed",
            }),
            Box::new(FailingPlanner("SecondPlanner")),
        ];
        let err = generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect_err("a later planner failing must abort generate_plan");
        match err {
            PipelineError::Planner { planner, .. } => assert_eq!(planner, "SecondPlanner"),
            other => panic!("expected PipelineError::Planner, got {other:?}"),
        }
    }

    /// The `get_planning_context` half of [`run_planner`], which
    /// `planner_failure_short_circuits_before_response_adapters_run` and
    /// `a_later_planner_failure_is_attributed_to_that_planner` cannot
    /// reach: both of those fail inside `solve`, so `run_planner`'s first
    /// `map_err` never runs in them. Upstream aborts on a `nullptr` context
    /// (`planning_pipeline.cpp:308-315`) separately from an unset error
    /// code after `solve` (`:323-328`); this port collapses the two into
    /// one [`PipelineError::Planner`], and that collapse is only honest if
    /// the construction branch is attributed to the same planner and
    /// carries the same source.
    #[test]
    fn context_construction_failure_is_reported_as_a_planner_failure() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> =
            vec![Box::new(UnbuildablePlanner("UnbuildablePlanner"))];
        let err = generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect_err("a planner that cannot build a context must abort generate_plan");
        match err {
            PipelineError::Planner { planner, source } => {
                assert_eq!(planner, "UnbuildablePlanner");
                // The source is the planner's own error, not a message
                // this function invented: if `run_planner` dropped it and
                // synthesised one, the text below would not survive.
                assert_eq!(
                    source.to_string(),
                    "UnbuildablePlanner cannot build a context",
                    "the construction failure's own error must reach the caller intact"
                );
            }
            other => panic!("expected PipelineError::Planner, got {other:?}"),
        }
    }

    #[test]
    fn response_adapter_failure_is_reported_after_a_successful_plan() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> =
            vec![Box::new(RejectingResponseAdapter)];
        let err = generate_plan(&mut scene, &env, &[], &planners, &response_chain, request())
            .expect_err("a rejecting response adapter must fail generate_plan");
        // Same reasoning as `request_adapter_failure_short_circuits_before_
        // any_planner_runs`: `(_)` doesn't check which chain element fired.
        match err {
            PipelineError::Response(ResponseAdapterError::InvalidMotionPlan {
                adapter, ..
            }) => {
                assert_eq!(adapter, "RejectingResponseAdapter");
            }
            other => panic!("expected PipelineError::Response(InvalidMotionPlan), got {other:?}"),
        }
    }

    #[test]
    fn second_response_adapter_failure_is_attributed_to_the_second_adapter() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        // First adapter passes, second rejects — the only element this test
        // targets.
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> = vec![
            Box::new(PassingResponseAdapter),
            Box::new(RejectingResponseAdapter),
        ];
        let err = generate_plan(&mut scene, &env, &[], &planners, &response_chain, request())
            .expect_err("the second response adapter's rejection must fail generate_plan");
        match err {
            PipelineError::Response(ResponseAdapterError::InvalidMotionPlan {
                adapter, ..
            }) => {
                assert_eq!(
                    adapter, "RejectingResponseAdapter",
                    "the rejection must be attributed to the adapter that actually rejected"
                );
            }
            other => panic!("expected PipelineError::Response(InvalidMotionPlan), got {other:?}"),
        }
    }

    #[test]
    fn full_success_runs_every_stage_and_fills_planner_id() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> = vec![];
        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed-goal",
        })];
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> = vec![];
        let response = generate_plan(
            &mut scene,
            &env,
            &request_chain,
            &planners,
            &response_chain,
            request(),
        )
        .expect("an unobstructed two-waypoint plan must succeed end to end");
        assert_eq!(response.trajectory.way_point_count(), 2);
        assert_eq!(response.planner_id, "fixed-goal");
    }

    #[test]
    fn two_planners_feed_the_first_trajectory_forward_into_the_second_request() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let seen = std::rc::Rc::new(std::cell::RefCell::new(Vec::new()));
        let planners: Vec<Box<dyn PlannerManager>> = vec![
            Box::new(RecordingPlanner {
                seen_trajectory_constraints_len: seen.clone(),
            }),
            Box::new(RecordingPlanner {
                seen_trajectory_constraints_len: seen.clone(),
            }),
        ];
        generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect("two planners, each handed a fixed two-waypoint plan, must both succeed");

        let seen = seen.borrow().clone();
        assert_eq!(
            seen,
            vec![0, 2],
            "the first planner must see the caller's empty trajectory_constraints, and the \
             second must see one KinematicConstraintSet per waypoint of the first planner's \
             two-waypoint trajectory — not the same empty list twice"
        );
    }

    #[test]
    fn planner_id_already_set_by_the_planner_is_left_untouched() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "planner-set-id",
        })];
        let mut req = request();
        req.planner_id = "request-set-id".to_string();
        let response = generate_plan(&mut scene, &env, &[], &planners, &[], req)
            .expect("an unobstructed two-waypoint plan must succeed");
        assert_eq!(
            response.planner_id, "planner-set-id",
            "a planner that already filled planner_id must win over the request's value"
        );
    }

    #[test]
    fn planner_id_left_empty_by_the_planner_falls_back_to_the_request() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "",
        })];
        let mut req = request();
        req.planner_id = "request-set-id".to_string();
        let response = generate_plan(&mut scene, &env, &[], &planners, &[], req)
            .expect("an unobstructed two-waypoint plan must succeed");
        assert_eq!(
            response.planner_id, "request-set-id",
            "an empty planner_id from the planner must fall back to the request's value"
        );
    }

    #[test]
    fn start_state_is_the_pre_planning_state_even_after_a_planner_moves_the_scene() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let pre_planning_state = scene.current_state().clone();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(SideEffectPlanner)];
        let response = generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect("an unobstructed plan must succeed even though the planner moves the scene");

        assert_eq!(response.start_state, pre_planning_state);
    }

    /// Records the scene's `panda_joint1`/`panda_joint2` positions as the
    /// *adapter chain* sees them, i.e. after `generate_plan` has applied
    /// `request.start_state` and before any planner runs. Semantic 7's
    /// ordering claim ("before `request_chain`") is only checkable from
    /// inside the chain.
    struct StartStateRecordingAdapter {
        seen: std::rc::Rc<std::cell::RefCell<Vec<(f64, f64)>>>,
    }
    impl PlanningRequestAdapter for StartStateRecordingAdapter {
        fn description(&self) -> &'static str {
            "StartStateRecordingAdapter"
        }
        fn adapt<'m>(
            &self,
            scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &mut PlanningRequest,
        ) -> Result<(), RequestAdapterError> {
            let state = scene.current_state();
            self.seen.borrow_mut().push((
                state.variable_position("panda_joint1").unwrap(),
                state.variable_position("panda_joint2").unwrap(),
            ));
            Ok(())
        }
    }

    #[test]
    fn the_requested_start_state_reaches_the_scene_before_the_request_adapters_run() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        scene
            .current_state_mut()
            .set_joint_positions("panda_joint2", &[-0.75])
            .unwrap();

        let seen = std::rc::Rc::new(std::cell::RefCell::new(Vec::new()));
        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> =
            vec![Box::new(StartStateRecordingAdapter { seen: seen.clone() })];
        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FixedGoalPlanner {
            name: "FixedGoalPlanner",
            planner_id: "fixed",
        })];

        let mut req = request();
        req.start_state = StartState::new(vec!["panda_joint1".to_string()], vec![0.25], vec![])
            .expect("a one-variable overlay is well formed");
        let response = generate_plan(&mut scene, &env, &request_chain, &planners, &[], req)
            .expect("an unobstructed plan must succeed");

        assert_eq!(
            seen.borrow().as_slice(),
            // `panda_joint2` is the discriminator: an implementation that
            // replaced the scene's state with the overlay instead of writing
            // over it would report the model default here, not -0.75.
            [(0.25, -0.75)],
            "the adapter chain must see the requested start state, overlaid on the scene's own \
             current state rather than replacing it"
        );
        assert_eq!(
            response
                .start_state
                .variable_position("panda_joint1")
                .unwrap(),
            0.25,
            "Semantic 6 captures the same state Semantic 7 established"
        );
    }

    #[test]
    fn a_start_state_naming_an_unknown_variable_fails_before_any_adapter_or_planner_runs() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let seen = std::rc::Rc::new(std::cell::RefCell::new(Vec::new()));
        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> =
            vec![Box::new(StartStateRecordingAdapter { seen: seen.clone() })];
        // A failing planner, so that reaching the planner at all would be
        // reported as `PipelineError::Planner` rather than as this variant.
        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(FailingPlanner("FirstPlanner"))];

        let mut req = request();
        req.start_state = StartState::new(vec!["no_such_joint".to_string()], vec![0.1], vec![])
            .expect("the shape is well formed; only the name is wrong");
        let err = generate_plan(&mut scene, &env, &request_chain, &planners, &[], req)
            .expect_err("a start state naming a variable the model lacks must abort the pipeline");
        match err {
            PipelineError::StartState(e) => {
                assert!(
                    e.to_string().contains("no_such_joint"),
                    "the rejection must name the variable that could not be written, got: {e}"
                );
            }
            other => panic!("expected PipelineError::StartState, got {other:?}"),
        }
        assert!(
            seen.borrow().is_empty(),
            "Semantic 7 applies the start state before request_chain, so a start state that \
             cannot be applied must stop the pipeline before any adapter observes the scene"
        );
    }

    #[test]
    fn scene_current_state_is_allowed_to_differ_from_start_state_after_generate_plan_returns() {
        // The flip side of the case above: `PlanningSceneValidityChecker`'s
        // documented contract is that `generate_plan` captures `start_state`
        // itself rather than relying on `scene.current_state()` staying put
        // -- it does not promise `scene.current_state()` is restored. This
        // asserts the scene really is left moved, so the case above is
        // proving `start_state` survives a real mutation, not a mutation
        // that never happened.
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();
        let pre_planning_state = scene.current_state().clone();

        let planners: Vec<Box<dyn PlannerManager>> = vec![Box::new(SideEffectPlanner)];
        generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect("an unobstructed plan must succeed even though the planner moves the scene");

        assert_ne!(scene.current_state(), &pre_planning_state);
    }
}
