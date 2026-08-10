// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2019, Bielefeld University
// Copyright (c) 2021, PickNik Robotics
// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, cspace contributors
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

//! §5 Phase 7's `cspace-planning`: the planner-agnostic
//! [`PlanningRequest`]/[`PlanningResponse`] types every planner in this
//! workspace should eventually speak, and the default request/response
//! adapter chain moveit2 runs around a planner's own `solve()` call —
//! before this round, neither existed anywhere in this workspace (only
//! `cspace_planners::sbp`, one concrete planner with its own
//! planner-specific request type, did).
//!
//! # These types are the only ones a planner speaks (D8)
//!
//! `cspace_planners::sbp::registry` used to define a second, unrelated
//! `PlanningRequest`/`PlanningResponse`/`PlanningContext`/`PlannerManager`
//! set that shared only *names* with these. Architecturally that was
//! backwards — a *planner* crate should depend on the planning-request
//! vocabulary, not define it — and it had a concrete cost: no caller could
//! hand a request across the boundary, so `ros/cspace-ros`'s endpoints had
//! no planner to call at all. PORTING-PLAN.md D8 removed the duplicate set;
//! [`crate::planner::PlannerManager`]/[`crate::planner::PlanningContext`]
//! here are the ones `RrtConnectManager` implements, and
//! `cspace_planning::planner_registry` is where a manager is looked up by name.
//!
//! Two shape differences from sbp's deleted types survived the merge, in
//! this crate's favour, because each is what the adapters actually need to
//! read or write:
//!
//! - [`PlanningRequest::goal_constraints`] is `Vec<KinematicConstraintSet>`
//!   (already-typed constraint sets — a state satisfying *any one* set is an
//!   acceptable goal), not sbp's old `goal: Goal` (one concrete state, or
//!   one set). [`crate::request_adapters::CheckForStackedConstraints`] counts
//!   `position_constraints`/`orientation_constraints` per set and
//!   [`crate::request_adapters::ResolveConstraintFrames`] inspects
//!   constraint frames — neither operation is expressible against a
//!   concrete state, which by definition has already thrown that structure
//!   away. A caller that means one concrete state writes it the way upstream
//!   does, `constructGoalConstraints(state, jmg, tolerance)`
//!   ([`crate::constraints::utils::construct_goal_joint_constraints`]).
//! - [`PlanningResponse::trajectory`] is a [`cspace_core::trajectory::RobotTrajectory`]
//!   (one `duration_from_previous` per waypoint), not sbp's old
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
//! it: on each concrete [`crate::planner::PlannerManager`] implementation's
//! own construction, the same place upstream's
//! `PlannerConfigurationSettings` already lives. D8 is where that happened:
//! `RrtConnectManager` is a struct carrying `resolution`/`seed`/`params`/
//! `solver`, and [`PlanningRequest`] carries none of them.
//!
//! # `cspace_planning::constraints`'s sampler now has somewhere to hand its output
//!
//! `cspace_planners::sbp::registry`'s own module doc (as of round 14) already
//! corrected an older claim that `constraint_samplers` was never ported —
//! it has been: [`crate::constraints::ConstraintSampler`]/
//! [`crate::constraints::JointConstraintSampler`]/
//! [`crate::constraints::UnionConstraintSampler`]
//! (`cspace-planning/src/constraints/sampler.rs`),
//! [`crate::constraints::IkConstraintSamplerAdapter`]
//! (`cspace-planning/src/constraints/ik_sampler.rs`), and
//! [`crate::constraints::select_default_sampler`]
//! (`cspace-planning/src/constraints/constraint_sampler_manager.rs`) all exist and
//! `cspace_planning::constraints` depends on `cspace_core::kinematics` to run IK-backed
//! sampling. Checked directly for this round (not assumed from that note
//! still being accurate): `crates/cspace-planning/src/constraints/` does contain all
//! three files, confirming the port itself is real.
//!
//! Rounds 20-25 wired it, inside `cspace_planners::sbp::registry`:
//! `RrtConnectContext::solve` calls
//! [`crate::constraints::select_default_sampler`] once per goal set and once
//! for `path_constraints`, and D8 pointed those calls at
//! [`PlanningRequest::goal_constraints`] — this crate's own field — rather
//! than at sbp's deleted `Goal` enum. So the answer to "can a caller of this
//! crate express a pose (position/orientation) goal?" is now yes, through
//! `cspace_planning::planner_registry` to `RrtConnectManager`, with the constraint
//! sampler turning the region into candidate states.
//!
//! What that answer is still bounded by is `rrt_connect` itself: the goal
//! region is collapsed to *one* concrete state before the search starts
//! (sampled with a bounded attempt count, then fixed), where upstream's
//! `ConstrainedGoalSampler` keeps producing new goal states for the whole
//! duration of the search. A goal whose sampled state is unreachable
//! therefore fails even when another state in the same region would have
//! been reachable. See `cspace_planners::sbp::registry`'s own module doc for
//! that gap.
//!
//! # Upstream inventory: every adapter file at the pinned commit, accounted for
//!
//! Swept `moveit_ros/planning/planning_request_adapter_plugins/src/` and
//! `moveit_ros/planning/planning_response_adapter_plugins/src/` at the
//! pinned commit file by file (`find … -name '*.cpp'`, cross-checked
//! against each directory's own `CMakeLists.txt` `add_library` source list
//! and each file's `^class ` declaration — neither directory has a separate
//! plugin-registration XML, so the `.cpp` file list plus each file's class
//! name is the ground truth): **5** request-adapter files, **4**
//! response-adapter files, 9 total.
//!
//! - Request (5/5 ported): `check_for_stacked_constraints.cpp` →
//!   [`request_adapters::CheckForStackedConstraints`],
//!   `check_start_state_bounds.cpp` → [`request_adapters::CheckStartStateBounds`],
//!   `check_start_state_collision.cpp` → [`request_adapters::CheckStartStateCollision`],
//!   `resolve_constraint_frames.cpp` → [`request_adapters::ResolveConstraintFrames`],
//!   `validate_workspace_bounds.cpp` → [`request_adapters::ValidateWorkspaceBounds`].
//! - Response (3/4 ported, 1 D1-excluded): `add_ruckig_traj_smoothing.cpp` →
//!   [`response_adapters::AddRuckigTrajectorySmoothing`],
//!   `add_time_optimal_parameterization.cpp` →
//!   [`response_adapters::AddTimeOptimalParameterization`], `validate_path.cpp`
//!   → [`response_adapters::ValidateSolution`]; `display_motion_path.cpp` — D1
//!   (rviz `MarkerArray`/`DisplayTrajectory` publishing, no other content, see
//!   the crate header comment).
//!
//! Four older MoveIt names sometimes associated with this plugin set —
//! `fix_start_state_bounds`, `fix_start_state_collision`,
//! `fix_workspace_bounds`, `add_iterative_spline_parameterization` — name no
//! file at the pinned commit. Checked against upstream's own git history,
//! not assumed:
//! - `fix_start_state_bounds`/`fix_start_state_collision`/
//!   `fix_workspace_bounds` are pre-rename names upstream commit `915b400e4`
//!   ("adding one more request adapter (to fix colliding states) + update
//!   listing of plugins + rename workspace fixing adapter") retired in favor
//!   of `check_start_state_bounds.cpp`/`check_start_state_collision.cpp`/
//!   `validate_workspace_bounds.cpp` — already ported under the current
//!   name, listed above.
//! - `add_iterative_spline_parameterization` was permanently removed from
//!   upstream at commit `62e6f9e71` ("Remove Iterative Spline and Iterative
//!   Parabola time-param algorithms (v2) (#1780)"), confirmed an ancestor of
//!   the pinned commit via `git merge-base --is-ancestor` — there is no file
//!   left to port; the pinned commit's only time-parameterization adapters
//!   are ruckig and TOTG, both already ported above.
//!
//! So the gap this section exists to check for is empty: every upstream file
//! the pinned commit ships is either ported or D1-excluded, and none of the
//! four candidate names above still name a file with anything left to port.
//!
//! # The adapter chain
//!
//! [`PlanningRequestAdapter`]/[`PlanningResponseAdapter`] replace
//! `planning_interface::PlanningRequestAdapter`/`PlanningResponseAdapter`;
//! [`run_request_adapters`]/[`run_response_adapters`] replace the chain a
//! `planning_pipeline::PlanningPipeline` runs each adapter through in order,
//! short-circuiting on the first failure — the "adapter chain" §5 Phase 7
//! named but never built. A caller assembles its own
//! `&[Box<dyn PlanningRequestAdapter>]`/`&[Box<dyn PlanningResponseAdapter>]`
//! (the doctest below is the first example anywhere in this crate that
//! actually does — every adapter module's own tests call `adapter.adapt`
//! directly, not through [`run_request_adapters`]/[`run_response_adapters`]);
//! this crate does not hand out a fixed default ordering, since no caller of
//! one exists yet to have an opinion on it.
//!
//! ```
//! use cspace_collision::ParryCollisionEnv;
//! use cspace_core::model::{MeshSearchPaths, RobotModel};
//! use cspace_planning::request_adapters::{
//!     CheckForStackedConstraints, CheckStartStateBounds, CheckStartStateCollision,
//!     ResolveConstraintFrames, ValidateWorkspaceBounds,
//! };
//! use cspace_planning::response_adapters::{AddRuckigTrajectorySmoothing, ValidateSolution};
//! use cspace_planning::{
//!     run_request_adapters, run_response_adapters, PlanningRequest, PlanningRequestAdapter,
//!     PlanningResponse, PlanningResponseAdapter, WorkspaceBounds,
//! };
//! use cspace_planning::scene::PlanningScene;
//! use cspace_core::srdf::SrdfModel;
//! use cspace_core::state::RobotState;
//! use cspace_core::trajectory::RobotTrajectory;
//! use std::fs;
//!
//! let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
//! let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
//! let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
//! let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
//! let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
//!     .expect("fixture model must build");
//!
//! let mut scene = PlanningScene::new(&model, &srdf);
//! let env = ParryCollisionEnv::default();
//!
//! // The request half: every request adapter this crate has, run once each,
//! // in one call.
//! let mut request = PlanningRequest {
//!     group_name: "panda_arm".to_string(),
//!     goal_constraints: vec![],
//!     path_constraints: None,
//!     workspace_bounds: WorkspaceBounds::default(),
//!     max_velocity_scaling_factor: 1.0,
//!     max_acceleration_scaling_factor: 1.0,
//!     ..Default::default()
//! };
//! let request_chain: Vec<Box<dyn PlanningRequestAdapter>> = vec![
//!     Box::new(CheckForStackedConstraints),
//!     Box::new(CheckStartStateBounds::new(false)),
//!     Box::new(CheckStartStateCollision),
//!     Box::new(ResolveConstraintFrames),
//!     Box::new(ValidateWorkspaceBounds::new(2.0)),
//! ];
//! run_request_adapters(&request_chain, &mut scene, &env, &mut request)
//!     .expect("a default-pose, empty-world panda_arm request must pass every request adapter");
//! assert_ne!(
//!     request.workspace_bounds,
//!     WorkspaceBounds::default(),
//!     "ValidateWorkspaceBounds must have filled in the unset box"
//! );
//!
//! // The response half. A hand-built trajectory stands in for a planner's
//! // output here so this example stays about the adapter chain; the
//! // "Plan once" example below runs a real planner instead.
//! let mut start = RobotState::new(&model);
//! start.set_to_default_values();
//! let mut goal = start.clone();
//! goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();
//! let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
//! let start_state = start.clone();
//! trajectory.add_suffix_way_point(start, 0.0).unwrap();
//! trajectory.add_suffix_way_point(goal, 0.0).unwrap();
//! let mut response = PlanningResponse {
//!     start_state,
//!     trajectory,
//!     planner_id: String::new(),
//! };
//!
//! let response_chain: Vec<Box<dyn PlanningResponseAdapter>> =
//!     vec![Box::new(AddRuckigTrajectorySmoothing), Box::new(ValidateSolution)];
//! run_response_adapters(&response_chain, &mut scene, &env, &request, &mut response)
//!     .expect("a two-waypoint panda_arm move must smooth and then validate successfully");
//!
//! assert!(response.trajectory.duration() > 0.0);
//! ```
//!
//! Both traits specialize directly to [`cspace_collision::ParryCollisionEnv`]
//! rather than being generic over `E: CollisionEnv<..>`, for the same reason
//! [`crate::planner::PlannerManager`] does (see that trait's own
//! "Deviation from upstream" doc): a generic *type* parameter on a trait
//! method breaks `dyn` object-safety, and `ParryCollisionEnv` is the only
//! [`cspace_collision::CollisionEnv`] implementation anywhere in this
//! workspace.
//!
//! # Plan once, with no ROS anywhere in the call graph
//!
//! §129 (Phase 9, "usable and interoperable without ROS 2") splits at this
//! crate: `ros/cspace-ros/` (D5/D6, a new panel's crate — not this one, and
//! not a root workspace member, since `r2r` needs ROS 2 at build time and
//! neither this host nor CI has it) owns the ROS-facing half; this crate is
//! the top-level pure-Rust one, so the "plan without ROS" entry point lives
//! here. The capability already existed —
//! `cspace-planners/examples/plan_benchmark_problem_set.rs` and
//! `cspace_planners::sbp::registry`'s own
//! `end_to_end_solve_on_panda_arm_reaches_the_requested_goal` test both run
//! URDF/SRDF → [`cspace_core::model::RobotModel`] → [`crate::scene::PlanningScene`]
//! → [`cspace_collision::ParryCollisionEnv`] → RRT-Connect with no ROS
//! anywhere — but buried inside a benchmark generator and a private test,
//! neither reachable as a documented entry point. This is that entry point,
//! promoted to a compiling doctest so it cannot silently rot out of date
//! (§126: a `text` block asserting this would be exactly the unverified
//! claim that rule exists to catch).
//!
//! **No type boundary any more (D8):** the example below plans through this
//! crate's own [`PlanningRequest`]/[`PlanningResponse`], selects the planner
//! by name out of `crate::planner_registry::PLANNER_MANAGERS`, and runs it
//! through [`crate::pipeline::generate_plan`] — the same three steps
//! `ros/cspace-ros`'s `/move_action` takes. `cspace_planners::sbp` and
//! `cspace_planning::planner_registry` are dev-dependencies of this crate, for this
//! doctest only; nothing in the library above knows either exists, and the
//! planner is reached entirely through [`crate::planner::PlannerManager`].
//!
//! ```
//! use std::fs;
//!
//! use cspace_collision::ParryCollisionEnv;
//! use cspace_planning::constraints::utils::construct_goal_joint_constraints;
//! use cspace_core::model::{MeshSearchPaths, RobotModel};
//! use cspace_planning::planner_registry::resolve_planner;
//! use cspace_planning::{PlannerConfigurationMap, PlanningRequest, generate_plan};
//! use cspace_planning::scene::PlanningScene;
//! use cspace_core::srdf::SrdfModel;
//! use cspace_core::state::RobotState;
//!
//! // Linked for its side effect, not for any symbol: `RrtConnectManager`
//! // registers itself into `PLANNER_MANAGERS` through a
//! // `linkme::distributed_slice` static, and nothing below names a
//! // `cspace_planners::sbp` item. Without this line the registration sits in
//! // an rlib object file no symbol references, the linker drops it, and
//! // `resolve_planner("rrt_connect")` below fails with `UnknownName`
//! // (measured — this example failed exactly that way before the line
//! // existed).
//! use cspace_planners as _;
//!
//! // Fixture URDF/SRDF loaded from disk, not a ROS parameter server or
//! // `robot_description` topic — the whole point of this example.
//! let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
//! let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
//! let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
//! let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
//! let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
//!     .expect("fixture model must build");
//!
//! let mut scene = PlanningScene::new(&model, &srdf);
//! let env = ParryCollisionEnv::default();
//!
//! // A concrete-state goal, written the way upstream writes one:
//! // `constructGoalConstraints(state, jmg, tolerance)`, one joint
//! // constraint per group variable. Tolerance zero because this is a
//! // concrete state and not a region around one -- a planner that resolves
//! // the set by sampling reproduces it exactly only at that width.
//! let mut goal_state = RobotState::new(&model);
//! goal_state.set_to_default_values();
//! goal_state.set_joint_positions("panda_joint1", &[0.4]).unwrap();
//! let goal = construct_goal_joint_constraints(&model, &goal_state.update(), "panda_arm", 0.0, 0.0)
//!     .unwrap();
//!
//! let request = PlanningRequest {
//!     group_name: "panda_arm".to_string(),
//!     goal_constraints: vec![goal],
//!     ..PlanningRequest::default()
//! };
//!
//! // Selected by name, never by slice position: `PLANNER_MANAGERS` is a
//! // `linkme::distributed_slice` and its order is the linker's
//! // (PORTING-PLAN.md §177). The map is the configuration the manager
//! // plans under -- upstream's `setPlannerConfigurations` argument, taken
//! // at construction here so a manager cannot exist without one. Empty
//! // means "this planner's own documented defaults", which is what a
//! // caller with no `/set_planner_params` in the picture wants.
//! let planner = resolve_planner("rrt_connect", &PlannerConfigurationMap::new())
//!     .expect("rrt_connect is registered");
//!
//! // Empty adapter chains: this example is about reaching the planner, and
//! // the chains have their own example above.
//! let response = generate_plan(&mut scene, &env, &[], &[planner], &[], request)
//!     .expect("an empty-world panda_arm query must be solvable");
//!
//! assert_eq!(response.planner_id, "rrt_connect");
//! assert!(response.trajectory.way_point_count() >= 2);
//! let last = response.trajectory.way_point(response.trajectory.way_point_count() - 1).unwrap();
//! assert!((last.variable_position("panda_joint1").unwrap() - 0.4).abs() < 1e-6);
//! ```

#[forbid(unsafe_code)]
pub mod error;

#[forbid(unsafe_code)]
pub mod pipeline;

#[forbid(unsafe_code)]
pub mod plan_responses;

#[forbid(unsafe_code)]
pub mod planner;

#[forbid(unsafe_code)]
pub mod request;

#[forbid(unsafe_code)]
pub mod request_adapters;

#[forbid(unsafe_code)]
pub mod response;

#[forbid(unsafe_code)]
pub mod response_adapters;

#[forbid(unsafe_code)]
pub mod start_state;

pub use error::{RequestAdapterError, ResponseAdapterError};
pub use pipeline::{PipelineError, generate_plan};
pub use plan_responses::{
    PlanOutcome, PlanResponsesContainer, shortest_solution, stop_at_first_solution,
};
pub use planner::{
    PlanError, PlannerConfigurationMap, PlannerConfigurationSettings, PlannerManager,
    PlanningContext, configuration_for, configuration_name,
};
pub use request::{PlanningRequest, WorkspaceBounds};
pub use response::PlanningResponse;
pub use start_state::{StartState, StartStateOverride};

use crate::scene::PlanningScene;
use cspace_collision::ParryCollisionEnv;

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
/// `cspace_planners::sbp`-style `PlannerManager` itself, which stays a
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

#[forbid(unsafe_code)]
pub mod constraints;

#[forbid(unsafe_code)]
pub mod scene;

// The one module the crate-level `unsafe_code = "allow"` exists for: it
// declares the `PLANNER_MANAGERS` distributed slice.
pub mod planner_registry;
