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
//! says so directly. This crate fixes the layering going forward without
//! touching sbp itself: relocating sbp onto these types is
//! `moveit-planners-sbp`'s own crate's job (assigned there alongside its
//! constraint-sampler wiring, round 19), not this one's — this crate's part
//! of that relocation is proving these canonical types can actually receive
//! it, which the full request/response adapter chain doctest below (see
//! "# The adapter chain") now exercises end to end.
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
//! use moveit_collision::ParryCollisionEnv;
//! use moveit_model::{MeshSearchPaths, RobotModel};
//! use moveit_planning::request_adapters::{
//!     CheckForStackedConstraints, CheckStartStateBounds, CheckStartStateCollision,
//!     ResolveConstraintFrames, ValidateWorkspaceBounds,
//! };
//! use moveit_planning::response_adapters::{AddRuckigTrajectorySmoothing, ValidateSolution};
//! use moveit_planning::{
//!     run_request_adapters, run_response_adapters, PlanningRequest, PlanningRequestAdapter,
//!     PlanningResponse, PlanningResponseAdapter, WorkspaceBounds,
//! };
//! use moveit_scene::PlanningScene;
//! use moveit_srdf::SrdfModel;
//! use moveit_state::RobotState;
//! use moveit_trajectory::RobotTrajectory;
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
//! // output here — wiring a concrete planner (`moveit-planners-sbp`) onto
//! // this crate's own canonical `PlanningRequest`/`PlanningResponse` is that
//! // crate's job, not this one's (see "Deviation from
//! // `moveit-planners-sbp::registry`'s existing types" above).
//! let mut start = RobotState::new(&model);
//! start.set_to_default_values();
//! let mut goal = start.clone();
//! goal.set_joint_positions("panda_joint1", &[0.4]).unwrap();
//! let mut trajectory = RobotTrajectory::for_group_name(&model, "panda_arm").unwrap();
//! trajectory.add_suffix_way_point(start, 0.0).unwrap();
//! trajectory.add_suffix_way_point(goal, 0.0).unwrap();
//! let mut response = PlanningResponse {
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
//! Both traits specialize directly to [`moveit_collision::ParryCollisionEnv`]
//! rather than being generic over `E: CollisionEnv<..>`, for the same reason
//! `moveit-planners-sbp::registry::PlannerManager` does (see that trait's own
//! "Deviation from upstream" doc): a generic *type* parameter on a trait
//! method breaks `dyn` object-safety, and `ParryCollisionEnv` is the only
//! [`moveit_collision::CollisionEnv`] implementation anywhere in this
//! workspace.
//!
//! # Plan once, with no ROS anywhere in the call graph
//!
//! §129 (Phase 9, "usable and interoperable without ROS 2") splits at this
//! crate: `ros/moveit-ros/` (D5/D6, a new panel's crate — not this one, and
//! not a root workspace member, since `r2r` needs ROS 2 at build time and
//! neither this host nor CI has it) owns the ROS-facing half; this crate is
//! the top-level pure-Rust one, so the "plan without ROS" entry point lives
//! here. The capability already existed —
//! `moveit-planners-sbp/examples/plan_benchmark_problem_set.rs` and
//! `moveit-planners-sbp::registry`'s own
//! `end_to_end_solve_on_panda_arm_reaches_the_requested_goal` test both run
//! URDF/SRDF → [`moveit_model::RobotModel`] → [`moveit_scene::PlanningScene`]
//! → [`moveit_collision::ParryCollisionEnv`] → RRT-Connect with no ROS
//! anywhere — but buried inside a benchmark generator and a private test,
//! neither reachable as a documented entry point. This is that entry point,
//! promoted to a compiling doctest so it cannot silently rot out of date
//! (§126: a `text` block asserting this would be exactly the unverified
//! claim that rule exists to catch).
//!
//! **Type boundary, deliberately visible below, not smoothed over:** this
//! example plans using `moveit-planners-sbp`'s own concrete
//! `PlanningRequest`/`PlanningResponse`/`RrtConnectManager`
//! (`moveit-planners-sbp` is a dev-dependency of this crate — for this
//! doctest only, not [`PlanningRequest`]/[`PlanningResponse`] above, which
//! remain sbp-independent production types), not this crate's own canonical
//! [`PlanningRequest`]/[`PlanningResponse`]. Relocating sbp's registry onto
//! this crate's types is `moveit-planners-sbp`'s own job (see "Deviation
//! from `moveit-planners-sbp::registry`'s existing types" above) — until
//! that lands, actually running a plan means speaking sbp's vocabulary end
//! to end, not this crate's. That seam (sbp's
//! `registry::PlanningRequest`/`PlanningResponse` on one side, this crate's
//! [`PlanningRequest`]/[`PlanningResponse`] on the other) is exactly what a
//! `TryFrom` conversion will need to bridge once the relocation lands, so
//! it is named here rather than blurred.
//!
//! ```
//! use std::fs;
//!
//! use moveit_collision::ParryCollisionEnv;
//! use moveit_model::{MeshSearchPaths, RobotModel};
//! use moveit_planners_sbp::{
//!     Goal, JointModelGroupSpace, PlannerManager, PlanningContext,
//!     PlanningRequest as SbpPlanningRequest, RrtConnectManager, RrtConnectParams, StateSpace,
//!     Termination,
//! };
//! use moveit_scene::PlanningScene;
//! use moveit_srdf::SrdfModel;
//! use rand::SeedableRng;
//! use rand_chacha::ChaCha8Rng;
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
//! let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
//! let mut rng = ChaCha8Rng::seed_from_u64(7);
//! let goal = space.sample_uniform(&mut rng);
//!
//! let request = SbpPlanningRequest {
//!     group_name: "panda_arm".to_string(),
//!     goal: Goal::State(goal.clone()),
//!     path_constraints: None,
//!     resolution: 0.05,
//!     seed: 7,
//!     params: RrtConnectParams {
//!         step_size: 0.5,
//!         goal_bias: 0.05,
//!         termination: Termination::Iterations(20_000),
//!         nn_degree: 8,
//!     },
//! };
//!
//! let manager = RrtConnectManager;
//! let mut context = manager
//!     .get_planning_context(&mut scene, &env, request)
//!     .expect("panda_arm is a real group");
//! let response = context
//!     .solve()
//!     .expect("an empty-world panda_arm query must be solvable");
//!
//! assert!(response.trajectory.len() >= 2);
//! assert_eq!(
//!     space.read_robot_state(response.trajectory.last().unwrap()),
//!     goal,
//!     "the last waypoint must equal the requested goal exactly"
//! );
//! ```

pub mod error;
pub mod pipeline;
pub mod request;
pub mod request_adapters;
pub mod response;
pub mod response_adapters;

pub use error::{RequestAdapterError, ResponseAdapterError};
pub use pipeline::{PipelineError, PlanError, Planner, generate_plan};
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
