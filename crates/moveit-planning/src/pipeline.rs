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
//! This workspace's D4 compile-time equivalent — the `PLANNER_MANAGERS`
//! `distributed_slice` and the `PlannerManager`/`PlanningContext` traits —
//! currently lives in `moveit-planners-sbp::registry`. Depending on it from
//! here would invert the intended layering (a planner crate should depend on
//! the planning-request vocabulary this crate defines, not the other way
//! around — see the crate doc's "Deviation from `moveit-planners-sbp::registry`"),
//! and `moveit-planners-sbp` is off-limits this round regardless (its
//! `registry.rs` is under concurrent edit elsewhere). So [`generate_plan`]
//! takes planners as `&[Box<dyn Planner<'m>>]`: **name-to-implementation
//! resolution is a concern orthogonal to the pipeline**, not part of what
//! this function does. The pipeline's actual substance — the adapter chains
//! and their failure/exit rules — does not need a registry at all; only a
//! *caller* who wants to go from a planner-id string to a boxed [`Planner`]
//! does, and that caller can be layered on top of this function once the
//! registry relocates (a decision for a later round, not this one).
//!
//! [`Planner`] mirrors [`crate::PlanningRequestAdapter`]/
//! [`crate::PlanningResponseAdapter`]'s existing shape
//! (`description()` + one action method, run through a `Box<dyn _>` chain)
//! for the same reason those two share it: consistency with an established
//! pattern already in this crate, and — unlike a bare `Fn` closure bound —
//! room for a caller to mix planners of different concrete types in one
//! chain, the same way upstream's `planner_map_` can hold different
//! `PlannerManager` implementations side by side.
//!
//! # Five semantics invisible to a bare diff read
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
//! `getTrajectoryConstraints`, `planning_pipeline.cpp:57-73`) called between
//! successive [`Planner::plan`] calls, never before the first — this port's
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
//! `moveit-planners-sbp::registry::PlanningContext` gives for omitting one:
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
//! `moveit-planners-sbp::planning_scene_validity::PlanningSceneValidityChecker`
//! (read-only from this crate, and not a dependency — see this module's doc,
//! "Planners are supplied by the caller, not looked up by name") documents
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
//! caller's original pre-adapter one) and before [`Planner::plan`] is
//! called for the first time — and stores it as
//! [`PlanningResponse::start_state`] once the response is otherwise
//! complete. One clone for the whole query, not one per validity check, is
//! exactly the cost `PlanningSceneValidityChecker`'s own doc says this
//! contract is designed to avoid paying per-call.
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
//! - `node_`/the `RCLCPP_INFO`/`RCLCPP_ERROR`/`RCLCPP_WARN` calls throughout
//!   `generatePlan` (`planning_pipeline.hpp:257`) — no logging facade
//!   anywhere in this port (see every adapter's own module doc).
//! - `pipeline_parameters_` (`planning_pipeline.hpp:259`) — the
//!   ROS-parameter-sourced config bag holding, among other things, the
//!   plugin-name lists a registry would resolve; see "Decided after registry
//!   relocation" below for the parts of its surface that are deferred rather
//!   than dropped.
//! - `terminate()` (`planning_pipeline.hpp:190`, `cpp:376-385`) — same
//!   reasoning `moveit-planners-sbp::registry::PlanningContext`'s own doc
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
//! # Decided after registry relocation, not dropped
//!
//! `getPlannerPluginNames`/`getRequestAdapterPluginNames`/
//! `getResponseAdapterPluginNames` (`hpp:193-208`) read `pipeline_parameters_`,
//! and `getPlannerManager` (`hpp:229-236`) reads `planner_map_` — state a
//! name-to-implementation registry owns, not state this call-scoped
//! function has anywhere to keep between calls. `getName` (`hpp:222-226`)
//! is a separate case: it returns `parameter_namespace_`, a third field
//! this port also has no long-lived home for, so it belongs with this group
//! for the same "not part of `generate_plan`'s scope today" reason even
//! though it does not read either container. Once
//! the registry relocates out of `moveit-planners-sbp::registry` (see this
//! round's separate item-2 report), whichever type ends up owning a named
//! planner chain is where these belong; they are not part of
//! [`generate_plan`]'s own scope today.

use moveit_collision::ParryCollisionEnv;
use moveit_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use moveit_scene::PlanningScene;
use moveit_trajectory::RobotTrajectory;

use crate::error::{RequestAdapterError, ResponseAdapterError};
use crate::request::PlanningRequest;
use crate::response::PlanningResponse;
use crate::{
    PlanningRequestAdapter, PlanningResponseAdapter, run_request_adapters, run_response_adapters,
};

/// Opaque planner failure: a caller's [`Planner`] implementation boxes
/// whatever error its own concrete planner produced (e.g.
/// `moveit_planners_sbp::registry::PlanError`) into this. This crate cannot
/// name a concrete planner error type — it does not, and must not, depend on
/// any concrete planner crate (see this module's doc, "Planners are
/// supplied by the caller, not looked up by name").
pub type PlanError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// Replaces `planner->getPlanningContext(...)` + `context->solve(res)`
/// (`planning_pipeline.cpp:304-320`): the caller-supplied bridge from this
/// crate's [`PlanningRequest`] to a concrete planner. See this module's doc
/// for why a trait (mirroring [`PlanningRequestAdapter`]/
/// [`PlanningResponseAdapter`]) rather than a bare `Fn` bound.
pub trait Planner<'m> {
    /// `planner->getDescription()`.
    fn description(&self) -> &'static str;

    /// Solve `request` against `scene`. Replaces
    /// `getPlanningContext(...)` failing (a `nullptr` context,
    /// `cpp:308-315`) and `context->solve(res)` failing (`res.error_code`
    /// unset after `solve`, `cpp:323-328`) alike — this port has one
    /// fallible step where upstream has two, since a caller's own
    /// [`Planner`] impl is exactly where "can I even build a context for
    /// this request" and "did solving it succeed" already have to be
    /// reconciled into one `Result`.
    fn plan(
        &self,
        scene: &mut PlanningScene<'m>,
        env: &ParryCollisionEnv,
        request: &PlanningRequest,
    ) -> Result<PlanningResponse<'m>, PlanError>;
}

/// Why [`generate_plan`] failed. Replaces the untyped `bool`/`res.error_code`
/// upstream's `generatePlan` returns.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum PipelineError {
    /// A [`PlanningRequestAdapter`] in `request_chain` rejected `request`.
    #[error(transparent)]
    Request(#[from] RequestAdapterError),
    /// A [`Planner`] failed. `planner` is that planner's own
    /// [`Planner::description`], matching how upstream's `RCLCPP_ERROR` at
    /// `cpp:325-326` names the failing planner.
    #[error("planner '{planner}' failed: {source}")]
    Planner {
        /// The failing planner's [`Planner::description`].
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
    /// The private `trajectory_constraints_for` helper (the "Semantic 1"
    /// feedforward step) failed to look up the previous planner's
    /// group/joints/waypoint positions. Upstream's `getTrajectoryConstraints`
    /// has no failure path
    /// at all (`cpp:57-73`) because the C++ object graph it reads
    /// guarantees those lookups always succeed; this port's equivalent
    /// lookups are typed `Result`s (`moveit_error::Result`), and surfacing
    /// that as an error here — rather than `.expect()`-ing an upstream
    /// invariant this port cannot independently verify — is the same
    /// "moveit-rs prefers to surface as an error" choice
    /// [`moveit_constraints::JointConstraint::new`]'s own doc already makes
    /// for a structurally analogous case.
    #[error("failed to build trajectory-constraints feedforward: {0}")]
    Feedforward(#[from] moveit_error::Error),
}

/// Ports `getTrajectoryConstraints` (`planning_pipeline.cpp:57-73`): one
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
) -> Result<Vec<KinematicConstraintSet>, moveit_error::Error> {
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

/// Replaces `PlanningPipeline::generatePlan` (`planning_pipeline.cpp:251-374`):
/// runs `request_chain`, then every planner in `planners` in sequence (with
/// the "Semantic 1" feedforward between successive planners), then
/// `response_chain`, then the "Semantic 4" `planner_id` fallback. See this
/// module's doc for the five semantics ported here that a bare diff read
/// would miss, and for what D1 excludes and what is deferred to a
/// not-yet-relocated registry rather than dropped.
///
/// # Errors
///
/// See [`PipelineError`]'s variants: a request-adapter rejection, a planner
/// failure (including a feedforward lookup failure between two planners), a
/// response-adapter rejection, or [`PipelineError::NoPlanners`] if `planners`
/// is empty.
pub fn generate_plan<'m>(
    scene: &mut PlanningScene<'m>,
    env: &ParryCollisionEnv,
    request_chain: &[Box<dyn PlanningRequestAdapter>],
    planners: &[Box<dyn Planner<'m>>],
    response_chain: &[Box<dyn PlanningResponseAdapter>],
    mut request: PlanningRequest,
) -> Result<PlanningResponse<'m>, PipelineError> {
    let Some((first_planner, later_planners)) = planners.split_first() else {
        return Err(PipelineError::NoPlanners);
    };

    run_request_adapters(request_chain, scene, env, &mut request)?;
    let start_state = scene.current_state().clone();

    let mut response =
        first_planner
            .plan(scene, env, &request)
            .map_err(|source| PipelineError::Planner {
                planner: first_planner.description(),
                source,
            })?;

    for planner in later_planners {
        request.trajectory_constraints = trajectory_constraints_for(scene, &response.trajectory)?;
        response = planner
            .plan(scene, env, &request)
            .map_err(|source| PipelineError::Planner {
                planner: planner.description(),
                source,
            })?;
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

    use moveit_error::Error as MoveitError;
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;
    use crate::PlanningRequestAdapter;
    use crate::request::WorkspaceBounds;

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

    /// Plans by handing back a fixed two-waypoint trajectory to `panda_joint1
    /// = 0.4`, regardless of `request`. Records every `request` it was
    /// called with (cloned goal-constraint/planner-id/trajectory-constraint
    /// summary) so a test can assert on what a later planner in a chain
    /// actually received — this is how the feedforward test proves the
    /// second request was rewritten, not merely eligible to be.
    struct FixedGoalPlanner {
        description: &'static str,
        planner_id: &'static str,
    }
    impl<'m> Planner<'m> for FixedGoalPlanner {
        fn description(&self) -> &'static str {
            self.description
        }
        fn plan(
            &self,
            scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<PlanningResponse<'m>, PlanError> {
            let model = scene.robot_model();
            let mut start = RobotState::new(model);
            start.set_to_default_values();
            Ok(PlanningResponse {
                start_state: start.clone(),
                trajectory: two_waypoint_trajectory(model, start),
                planner_id: self.planner_id.to_string(),
            })
        }
    }

    /// Always fails with an opaque boxed error — the "planner fails"
    /// boundary. Carries its own `description` so a chain can hold two
    /// distinctly-named failing planners: [`PipelineError::Planner`] is
    /// constructed at two call sites in [`generate_plan`] (the first
    /// planner and the `later_planners` loop), and telling them apart
    /// requires each occupying a different chain position under a
    /// different name.
    struct FailingPlanner(&'static str);
    impl<'m> Planner<'m> for FailingPlanner {
        fn description(&self) -> &'static str {
            self.0
        }
        fn plan(
            &self,
            _scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<PlanningResponse<'m>, PlanError> {
            Err(Box::new(MoveitError::other(format!(
                "{} always fails",
                self.0
            ))))
        }
    }

    /// Records the `trajectory_constraints` of every request it is called
    /// with, then delegates to a [`FixedGoalPlanner`]-style plan. Used only
    /// by the feedforward test, where the assertion is "the second call's
    /// request.trajectory_constraints is non-empty", not merely "a second
    /// call happened". `Rc`, not a borrow, so two boxed instances can share
    /// one recording sink without fighting `Box<dyn Planner<'m>>`'s implicit
    /// `'static` object-lifetime bound.
    struct RecordingPlanner {
        seen_trajectory_constraints_len: std::rc::Rc<std::cell::RefCell<Vec<usize>>>,
    }
    impl<'m> Planner<'m> for RecordingPlanner {
        fn description(&self) -> &'static str {
            "RecordingPlanner"
        }
        fn plan(
            &self,
            scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            request: &PlanningRequest,
        ) -> Result<PlanningResponse<'m>, PlanError> {
            self.seen_trajectory_constraints_len
                .borrow_mut()
                .push(request.trajectory_constraints.len());
            let model = scene.robot_model();
            let mut start = RobotState::new(model);
            start.set_to_default_values();
            Ok(PlanningResponse {
                start_state: start.clone(),
                trajectory: two_waypoint_trajectory(model, start),
                planner_id: String::new(),
            })
        }
    }

    /// Mutates `scene`'s current state as a side effect of planning, the
    /// same way `moveit-planners-sbp::planning_scene_validity::PlanningSceneValidityChecker`
    /// leaves `scene` at whatever state its last validity check posed —
    /// see this module's doc, "Semantic 6". Used to prove
    /// [`PlanningResponse::start_state`] is captured *before* that
    /// mutation, not read back from a scene a planner may have since
    /// moved.
    struct SideEffectPlanner;
    impl<'m> Planner<'m> for SideEffectPlanner {
        fn description(&self) -> &'static str {
            "SideEffectPlanner"
        }
        fn plan(
            &self,
            scene: &mut PlanningScene<'m>,
            _env: &ParryCollisionEnv,
            _request: &PlanningRequest,
        ) -> Result<PlanningResponse<'m>, PlanError> {
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
        }
    }

    #[test]
    fn zero_planners_is_an_error() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn Planner>> = vec![];
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
        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FixedGoalPlanner {
            description: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        let err = generate_plan(&mut scene, &env, &request_chain, &planners, &[], request())
            .expect_err("a rejecting request adapter must abort before planning");
        assert!(matches!(err, PipelineError::Request(_)));
    }

    #[test]
    fn planner_failure_short_circuits_before_response_adapters_run() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FailingPlanner("FirstPlanner"))];
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
        let planners: Vec<Box<dyn Planner>> = vec![
            Box::new(FixedGoalPlanner {
                description: "FirstPlanner",
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

    #[test]
    fn response_adapter_failure_is_reported_after_a_successful_plan() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FixedGoalPlanner {
            description: "FixedGoalPlanner",
            planner_id: "fixed",
        })];
        let response_chain: Vec<Box<dyn PlanningResponseAdapter>> =
            vec![Box::new(RejectingResponseAdapter)];
        let err = generate_plan(&mut scene, &env, &[], &planners, &response_chain, request())
            .expect_err("a rejecting response adapter must fail generate_plan");
        assert!(matches!(err, PipelineError::Response(_)));
    }

    #[test]
    fn full_success_runs_every_stage_and_fills_planner_id() {
        let (model, srdf) = panda();
        let mut scene = PlanningScene::new(&model, &srdf);
        let env = ParryCollisionEnv::default();

        let request_chain: Vec<Box<dyn PlanningRequestAdapter>> = vec![];
        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FixedGoalPlanner {
            description: "FixedGoalPlanner",
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
        let planners: Vec<Box<dyn Planner>> = vec![
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

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FixedGoalPlanner {
            description: "FixedGoalPlanner",
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

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(FixedGoalPlanner {
            description: "FixedGoalPlanner",
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

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(SideEffectPlanner)];
        let response = generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect("an unobstructed plan must succeed even though the planner moves the scene");

        assert_eq!(response.start_state, pre_planning_state);
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

        let planners: Vec<Box<dyn Planner>> = vec![Box::new(SideEffectPlanner)];
        generate_plan(&mut scene, &env, &[], &planners, &[], request())
            .expect("an unobstructed plan must succeed even though the planner moves the scene");

        assert_ne!(scene.current_state(), &pre_planning_state);
    }
}
