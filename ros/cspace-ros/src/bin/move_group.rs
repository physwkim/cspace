// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The `cspace-ros` node: upstream's `move_group` capabilities that Phase 9's
//! completion condition names, on one `r2r::Node`.
//!
//! Six `move_group` endpoints are hosted here, matching upstream's own
//! arrangement -- `move_group` loads `MoveGroupPlanService`,
//! `MoveGroupMoveAction`, `MoveGroupExecuteTrajectoryAction`,
//! `MoveGroupStateValidationService` and `MoveGroupCartesianPathService` as
//! capabilities of a single node, not as separate processes, and that node's
//! `PlanningSceneMonitor` opens the scene subscription those capabilities
//! read through:
//!
//! * `/plan_kinematic_path` (`moveit_msgs/srv/GetMotionPlan`), PORTING-PLAN.md
//!   §241 -- upstream `move_group/src/default_capabilities/
//!   plan_service_capability.cpp`.
//! * `/move_action` (`moveit_msgs/action/MoveGroup`), PORTING-PLAN.md §250 --
//!   upstream `move_action_capability.cpp`. This is the endpoint an
//!   unmodified `MoveGroupInterface::plan()` actually calls:
//!   `move_group_interface.cpp:659` returns `FAILURE` locally without sending
//!   anything at all unless this action server is up, and `:712` then sends
//!   the goal here. That file mentions `GetMotionPlan`/`plan_kinematic_path`
//!   nowhere, so the service above is unreachable from that client (§241.4).
//! * `/execute_trajectory` (`moveit_msgs/action/ExecuteTrajectory`) --
//!   upstream `execute_trajectory_action_capability.cpp`. Bound because
//!   `MoveGroupInterface`'s constructor opens it
//!   (`move_group_interface.cpp:191-193`) whether or not the client ever
//!   executes anything, and its absence costs that constructor one silent
//!   `wait_for_servers` timeout. Everything the server answers, and which of
//!   the two possible servers it is, lives in
//!   [`moveit_ros::execute_trajectory`].
//! * `planning_scene` (`moveit_msgs/msg/PlanningScene`, subscription) --
//!   upstream `PlanningSceneMonitor::startSceneMonitor`
//!   (`planning_scene_monitor.cpp:1197`). See "The monitored scene" below.
//! * `/check_state_validity` (`moveit_msgs/srv/GetStateValidity`) -- upstream
//!   `state_validation_service_capability.cpp`. It is here because a
//!   subscription with no reader is unobservable: the scene monitor's whole
//!   point is that the capabilities beside it answer differently once the
//!   scene changes, and this is the one upstream capability in reach that
//!   consults the scene *and* can be answered end to end by this workspace
//!   (it needs a collision environment, which exists, not a planning
//!   pipeline, which does not).
//! * `compute_cartesian_path` (`moveit_msgs/srv/GetCartesianPath`) --
//!   upstream `cartesian_path_service_capability.cpp`. The second endpoint an
//!   unmodified `MoveGroupInterface` calls in its own right:
//!   `computeCartesianPath` sends here and returns the reply's `fraction`
//!   verbatim (`move_group_interface.cpp:873-911`). Its body lives in
//!   [`moveit_ros::cartesian_path`], which documents what that fraction means
//!   and which request fields this port refuses rather than ignores.
//!
//! Three more topics are published here that upstream's `move_group` does
//! not publish at all, because the client needs them from *somewhere* and in
//! this workspace there is nowhere else:
//!
//! * `robot_description` and `robot_description_semantic`
//!   (`std_msgs/msg/String`, latched) -- what an unmodified
//!   `MoveGroupInterface` falls back to when its own node has no such
//!   parameter, and therefore what its constructor blocks 10 s per
//!   description without. Upstream publishes these only when told to;
//!   [`moveit_ros::robot_description`] has the derivation and the
//!   both-or-neither invariant.
//! * `joint_states` (`sensor_msgs/msg/JointState`, 10 Hz) -- the robot
//!   driver's topic, which upstream's `move_group` only ever subscribes to.
//!   It is what a client's `getCurrentState()` waits on, and `plan()` does
//!   not need it; [`moveit_ros::joint_states`] has both halves of that.
//!
//! The name is upstream's own for the executable that loads exactly these
//! capabilities: `add_executable(move_group src/move_group.cpp)`
//! (`moveit_ros/move_group/CMakeLists.txt:89`), whose node is
//! `rclcpp::Node::make_shared("move_group", opt)` (`move_group.cpp:235`).
//! §241 called this file `plan_kinematic_path_server` when it hosted one
//! endpoint; §255 renamed it, because a name that tracks which subset of
//! capabilities happens to be built has to move every time one is added.
//!
//! # The monitored scene
//!
//! Upstream's monitor holds one `planning_scene::PlanningScenePtr scene_`
//! behind a `std::shared_mutex scene_update_mutex_`: the subscription
//! callback takes it exclusively (`std::unique_lock`,
//! `planning_scene_monitor.cpp:748`) and every capability takes it shared
//! (`LockedPlanningSceneRO`), which is what lets a capability read a scene it
//! must not modify.
//!
//! Here the scene is an `Rc<RefCell<Arc<PlanningScene>>>`, and the two halves
//! of that shape are the two halves of upstream's lock:
//!
//! * The `RefCell` is the exclusive-writer half. This node runs one
//!   `LocalPool` and `spin_once` on a single thread, so there is no
//!   concurrent reader for a `shared_mutex` to admit; what remains is that a
//!   borrow must never be held across an `.await`, and none is.
//! * The `Arc<PlanningScene>` is the read-only half, and it is why a
//!   capability cannot modify what it reads even by mistake. A scene update
//!   builds a decoupled copy, applies the message to *that*, and swaps the
//!   `Arc`; a query takes `PlanningScene::diff` off the `Arc` and mutates
//!   only the child. `LockedPlanningSceneRO`'s guarantee is a `const` pointer
//!   -- a promise the caller can cast away; here the snapshot is genuinely
//!   shared and the only mutable handle is the child.
//!
//! The copy-per-update is the price of that. `PlanningScene::cloned` is a
//! `diff` plus `decouple_parent`, so it is proportional to the scene's own
//! world and attached bodies, not to the message; a monitor fed at a high
//! rate would want the in-place update instead, and would then need
//! upstream's real reader/writer lock to keep the queries safe.
//!
//! # Both endpoints plan
//!
//! Each converts its incoming `moveit_msgs/MotionPlanRequest` into a
//! [`cspace_planning::PlanningRequest`] (the existing
//! `TryFrom<PlanningRequestMsg>` impl), hands it to
//! [`moveit_ros::move_group::plan_only`], and encodes whatever comes back.
//! That function is upstream's own two steps --
//! `MoveGroupCapability::resolvePlanningPipeline` followed by
//! `PlanningPipeline::generatePlan` -- and both capabilities call them in
//! that order (`plan_service_capability.cpp:69-106`,
//! `move_action_capability.cpp:206-227`), which is why one function serves
//! both here too.
//!
//! Until PORTING-PLAN.md D8 landed there was nothing to call: no planner
//! crate depended on `cspace-planning`, and
//! `cspace_planners_sbp::registry`'s `PlanningRequest` shared only a *name*
//! with `cspace_planning`'s. D8 merged the two, so `RrtConnectManager` now
//! implements [`cspace_planning::PlannerManager`] and reaches these
//! endpoints through `cspace_planner_registry::PLANNER_MANAGERS` -- which
//! this binary sees only because `moveit_ros`'s own `lib.rs` names the
//! planner crate, `linkme` registrations being link-time, not
//! dependency-time (`cspace_planner_registry::PLANNER_MANAGERS`' doc).
//!
//! Both plan against the monitored scene rather than a fresh one: `plan`
//! below takes the same `Arc` snapshot `handle_state_validity` takes and
//! runs on `snapshot.diff()`, so a collision object that arrived on
//! `planning_scene` is in the world the planner checks against, and a
//! planner that leaves the current state where it finished mutates only the
//! child. That is upstream's shape for both capabilities:
//! `LockedPlanningSceneRO ps(context_->planning_scene_monitor_)`
//! (`plan_service_capability.cpp:88`) and
//! `copyPlanningScene(goal->get_goal()->planning_options.planning_scene_diff)`
//! (`move_action_capability.cpp:216-217`).
//!
//! The second of those takes an argument this port drops: a goal's
//! `planning_options.planning_scene_diff` is ignored, so a client that ships
//! its world inside the goal instead of publishing it on `planning_scene`
//! plans against an emptier scene than it asked for, with nothing on the
//! wire saying so.
//!
//! # One error code for every conversion failure, and what that costs
//!
//! Both handlers below answer a failed `TryFrom<PlanningRequestMsg>` with
//! `MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS`, whatever the conversion
//! actually rejected. That conversion has one entry point and one
//! `cspace_core::error::Error` return, so the handler cannot tell a malformed goal
//! constraint from an unrepresentable `start_state.multi_dof_joint_state`
//! from a `reference_trajectories` this port has nowhere to put; the
//! `message` string names the reason and the code does not. Upstream has no
//! counterpart to compare against -- the conversion is this port's own -- but
//! `MoveItErrorCodes` does carry `START_STATE_INVALID`, so the code is
//! narrower than the failure set it reports. Closing it means giving the
//! conversion a typed error enum with one variant per rejected field, which
//! is `cspace-ros`'s own call and a change to every one of its callers.
//!
//! # Both endpoints report a failure to plan as `MoveItErrorCodes::FAILURE`
//!
//! Upstream reaches "there is no planning pipeline" through
//! `resolvePlanningPipeline` returning null and "the pipeline ran and did not
//! solve" through `generatePlan` returning false, and encodes *both* as
//! `FAILURE` in both capabilities
//! (`move_action_capability.cpp:207-211,218-227`,
//! `plan_service_capability.cpp:82-85,92-97`), reserving `PLANNING_FAILED`
//! for elsewhere. `/plan_kinematic_path` here answered `PLANNING_FAILED` from
//! §241 until §255 corrected it; that correction stands, and the live leg of
//! `ros/verify-ros-interop.sh` stays pinned to `val=99999`.
//!
//! What §255 could say and this round cannot is that `FAILURE` is the *only*
//! reachable answer. The null-pipeline branch it stood in is still reachable
//! -- [`moveit_ros::move_group::PlanOnlyError::UnknownPipeline`] is what a
//! `pipeline_id` naming no registered planner produces, and what *every*
//! request would produce if `PLANNER_MANAGERS` were empty, which is the state
//! §255 was written in. It is no longer the only outcome: a request naming a
//! registered planner now reaches it and can come back `SUCCESS`.

use std::env;
use std::fs;
use std::process::ExitCode;
use std::rc::Rc;
use std::sync::Arc;
use std::time::Duration;

use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use cspace_collision::{BodyType, CollisionRequest, ParryCollisionEnv};
use cspace_constraints::KinematicConstraintSet;
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_planning::PlanningRequest;
use moveit_ros::constraints::set::ConstraintsMsg;
use moveit_ros::execute_trajectory;
use moveit_ros::execution::{ExecutionEvent, ExecutionEventMsg, StopOutcome, TrajectoryExecution};
use moveit_ros::joint_states::JointSampler;
// `use_planning_scene_msg` is no longer imported here: every scene write in
// this binary now goes through `monitored_scene`, which is the one owner both
// the diff path and the topic path route through.
use moveit_ros::monitored_scene::{self, MonitoredScene};
use moveit_ros::move_group::plan_only;
use moveit_ros::planner_params::PlannerConfigurations;
use moveit_ros::planning::{PlanningRequestMsg, PlanningResponseMsgOut};
use moveit_ros::robot_description;
use moveit_ros::state::RobotStateMsg;
use cspace_scene::PlanningScene;
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use r2r::QosProfile;
use r2r::moveit_msgs::action::{ExecuteTrajectory, MoveGroup};
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::moveit_msgs::srv::{GetCartesianPath, GetMotionPlan, GetStateValidity};
use r2r::sensor_msgs::msg::JointState;

/// `MoveItErrorCodes::source` for each endpoint -- see [`plan`], the one
/// place either is written, for what the field is used for here.
///
/// The endpoint, not the binary: a `source` built from the binary name goes
/// stale on the wire every time the binary is renamed -- which is what §255
/// did to this one.
const PLAN_SERVICE_SOURCE: &str = "cspace-ros/plan_kinematic_path";
const MOVE_ACTION_SOURCE: &str = "cspace-ros/move_action";

/// A non-`SUCCESS` answer, minus its `source`: the `val` and `message` are
/// the same whichever endpoint asked, and the `source` names the endpoint,
/// which only [`plan`] knows.
struct PlanFailure {
    val: i32,
    message: String,
}

/// Both capabilities' shared body, and the one place a reply is built: plan,
/// then stamp `source` on whichever arm answered.
///
/// The stamp is here and not at the two call sites because it has to hold on
/// *both* arms. Upstream leaves `MoveItErrorCodes::source` empty; this port
/// uses it as "which endpoint built this reply", and that is the only thing
/// separating an answer that crossed DDS from one `MoveGroupInterface`
/// synthesised locally when no server answered
/// (`move_group_interface.cpp:659-663`, which returns `FAILURE` with both
/// strings empty -- the same `val` this node answers a failed plan with).
/// Stamping only the failing arm would have left the success reply, the one
/// Phase 9's completion condition is about, indistinguishable from a client
/// that never reached this node.
fn plan(
    snapshot: &Arc<PlanningScene<'static>>,
    configs: &PlannerConfigurations,
    msg: r2r::moveit_msgs::msg::MotionPlanRequest,
    source: &str,
) -> r2r::moveit_msgs::msg::MotionPlanResponse {
    let mut response = plan_inner(snapshot, configs, msg).unwrap_or_else(|failure| {
        r2r::moveit_msgs::msg::MotionPlanResponse {
            error_code: MoveItErrorCodes {
                val: failure.val,
                message: failure.message,
                ..Default::default()
            },
            ..Default::default()
        }
    });
    response.error_code.source = source.to_string();
    response
}

/// Convert the wire request, plan against the monitored scene, encode.
///
/// The three failures are upstream's, in upstream's order -- a request that
/// does not convert (no upstream analogue: `MotionPlanRequest` *is*
/// upstream's planning request, so there is nothing there to reject), an
/// unresolved pipeline or an unsolved plan (both `FAILURE`,
/// `move_action_capability.cpp:207-211,218-227`), and encoding the answer
/// back onto the wire (no upstream analogue either: `getMessage` cannot
/// fail).
///
/// `snapshot` stands in for `LockedPlanningSceneRO ps(...)` exactly as it
/// does in [`handle_state_validity`]: planning runs on `snapshot.diff()`, so
/// a planner that leaves the current state where it finished writes to the
/// child and the monitored scene the next request reads is untouched.
fn plan_inner(
    snapshot: &Arc<PlanningScene<'static>>,
    configs: &PlannerConfigurations,
    msg: r2r::moveit_msgs::msg::MotionPlanRequest,
) -> Result<r2r::moveit_msgs::msg::MotionPlanResponse, PlanFailure> {
    // Read before the move below: `PlanningRequest` has no `pipeline_id`
    // field (`doc/message-mapping.md` records it as dropped), so the
    // selection upstream makes from this field has to be made off the
    // message.
    let pipeline_id = msg.pipeline_id.clone();

    let request = PlanningRequest::try_from(PlanningRequestMsg {
        model: snapshot.robot_model(),
        msg,
    })
    .map_err(|e| PlanFailure {
        val: MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS as i32,
        message: format!("MotionPlanRequest -> PlanningRequest: {e}"),
    })?;

    let mut scene = snapshot.diff();
    // Built from the scene's own world, the same way `handle_state_validity`
    // builds one: an env constructed once at startup would be checking every
    // plan against the world as it stood before the first `planning_scene`
    // message arrived.
    let env = ParryCollisionEnv::new(scene.world().clone(), Default::default());
    // Borrowed for the length of the plan, not cloned: this is the store
    // `/set_planner_params` writes into, and the planner is constructed from
    // whatever it holds *now* -- upstream's
    // `setPlannerConfigurations(configs)` on the pipeline instance
    // (`query_planners_service_capability.cpp:205`), with the direction of
    // the hand-off reversed. The borrow ends with this statement and never
    // spans an `.await`, the rule the whole node's `Rc<RefCell<_>>` use
    // rests on.
    let response =
        plan_only(&mut scene, &env, &pipeline_id, &configs.borrow(), request).map_err(|e| {
            PlanFailure {
                val: MoveItErrorCodes::FAILURE as i32,
                message: e.to_string(),
            }
        })?;

    PlanningResponseMsgOut::try_from(response)
        .map(|out| out.0)
        .map_err(|e| PlanFailure {
            val: MoveItErrorCodes::FAILURE as i32,
            message: format!("PlanningResponse -> MotionPlanResponse: {e}"),
        })
}

/// `MoveGroupPlanService::computePlanService`
/// (`plan_service_capability.cpp:69-106`).
fn handle_request(
    snapshot: &Arc<PlanningScene<'static>>,
    configs: &PlannerConfigurations,
    msg: GetMotionPlan::Request,
) -> GetMotionPlan::Response {
    GetMotionPlan::Response {
        motion_plan_response: plan(
            snapshot,
            configs,
            msg.motion_plan_request,
            PLAN_SERVICE_SOURCE,
        ),
    }
}

/// `MoveGroupMoveAction::executeMoveCallback`
/// (`move_action_capability.cpp:85`), as far as it is reachable here.
///
/// Upstream branches on `plan_only || !allow_trajectory_execution_`. This
/// node executes nothing -- there is no
/// `trajectory_execution_manager::TrajectoryExecutionManager` in this
/// workspace -- so its `allow_trajectory_execution_` is false and the
/// plan-only arm is the only reachable one, which is why the branch appears
/// here as upstream's warning rather than as a second code path. A
/// `plan_only == false` goal therefore gets a plan-only answer, and says so,
/// exactly as upstream does at `:98-102`.
///
/// `executed_trajectory` stays empty for that same reason, and
/// `planning_time` stays `0.0`: upstream fills it from
/// `MotionPlanResponse::planning_time` (`:232`), which
/// [`cspace_planning::PlanningResponse`] has no field for -- see that type's
/// own doc comment. Reporting this handler's wall clock there would put a
/// different number under upstream's name for one.
fn handle_move_group_goal(
    snapshot: &Arc<PlanningScene<'static>>,
    configs: &PlannerConfigurations,
    goal: MoveGroup::Goal,
) -> MoveGroup::Result {
    if !goal.planning_options.plan_only {
        eprintln!(
            "This instance of MoveGroup is not allowed to execute trajectories \
             but the goal request has plan_only set to false. Only a motion \
             plan will be computed anyway."
        );
    }

    // `convertToMsg(res.trajectory, action_res->trajectory_start,
    // action_res->planned_trajectory)` (`:230`) followed by `action_res->
    // error_code = res.error_code` (`:231`) -- the same three fields
    // `MotionPlanResponse` carries them in, moved across. Unconditional, as
    // upstream's is: on the failing arm they are the empty values
    // `MotionPlanResponse::default` supplies, which is what upstream's own
    // untouched `res.trajectory` holds there too.
    let response = plan(snapshot, configs, goal.request, MOVE_ACTION_SOURCE);
    MoveGroup::Result {
        error_code: response.error_code,
        trajectory_start: response.trajectory_start,
        planned_trajectory: response.trajectory,
        ..Default::default()
    }
}

/// Upstream `contactToMsg` (`collision_tools.cpp:284`), plus the two header
/// fields `MoveGroupStateValidationService::isStateValid` fills in right
/// after it (`state_validation_service_capability.cpp:98-99`): the planning
/// frame, and the time the contact was reported.
///
/// `stamp` is left at zero rather than read from a clock. Upstream takes it
/// from the node's clock; this binary has no other use for one, and a
/// timestamp is not something any assertion downstream can check, so
/// fabricating one would put a number where upstream has a measurement.
fn contact_to_msg(
    contact: &cspace_collision::Contact,
    planning_frame: &str,
) -> r2r::moveit_msgs::msg::ContactInformation {
    let body_type = |t: BodyType| match t {
        BodyType::RobotLink => 0u32,     // ContactInformation::ROBOT_LINK
        BodyType::WorldObject => 1u32,   // ContactInformation::WORLD_OBJECT
        BodyType::RobotAttached => 2u32, // ContactInformation::ROBOT_ATTACHED
    };
    r2r::moveit_msgs::msg::ContactInformation {
        header: r2r::std_msgs::msg::Header {
            frame_id: planning_frame.to_string(),
            ..Default::default()
        },
        position: r2r::geometry_msgs::msg::Point {
            x: contact.pos.x,
            y: contact.pos.y,
            z: contact.pos.z,
        },
        normal: r2r::geometry_msgs::msg::Vector3 {
            x: contact.normal.x,
            y: contact.normal.y,
            z: contact.normal.z,
        },
        depth: contact.depth,
        contact_body_1: contact.body_name_1.clone(),
        body_type_1: body_type(contact.body_type_1),
        contact_body_2: contact.body_name_2.clone(),
        body_type_2: body_type(contact.body_type_2),
    }
}

/// Upstream `costSourceToMsg` (`collision_tools.cpp:273`).
fn cost_source_to_msg(source: &cspace_collision::CostSource) -> r2r::moveit_msgs::msg::CostSource {
    r2r::moveit_msgs::msg::CostSource {
        cost_density: source.cost,
        aabb_min: r2r::geometry_msgs::msg::Vector3 {
            x: source.aabb_min[0],
            y: source.aabb_min[1],
            z: source.aabb_min[2],
        },
        aabb_max: r2r::geometry_msgs::msg::Vector3 {
            x: source.aabb_max[0],
            y: source.aabb_max[1],
            z: source.aabb_max[2],
        },
    }
}

/// `MoveGroupStateValidationService::computeService` and its `isStateValid`
/// helper (`state_validation_service_capability.cpp:135` and `:62`).
///
/// `snapshot` stands in for `LockedPlanningSceneRO ls(...)`: the query runs on
/// `snapshot.diff()`, so `req.robot_state` lands on the child and the
/// monitored scene is untouched. Upstream reaches the same guarantee with a
/// shared lock and a `const` handle.
///
/// Two deviations, both from what this workspace can represent:
///
/// * A `req.robot_state` this port cannot build (`is_diff`, multi-DOF joints,
///   attached objects -- `crate::state`'s documented gaps) answers `valid:
///   false` with an empty result and a line on stderr, because the response
///   type has no error field to say anything else in. Upstream has no such
///   case: `robotStateMsgToRobotState` handles `is_diff` by applying onto the
///   state it was seeded with. Fail-closed is the safe direction, but it is
///   not upstream's answer, and the stderr line is what tells the two apart.
/// * `creq.cost = true` is upstream's, and `cres.cost_sources` is filled by
///   this port's `ParryCollisionEnv` -- but `distance` is left off, matching
///   upstream, which never sets `creq.distance` here either.
fn handle_state_validity(
    snapshot: &Arc<PlanningScene<'static>>,
    request: GetStateValidity::Request,
) -> GetStateValidity::Response {
    let model = snapshot.robot_model();
    let mut scene = snapshot.diff();

    // `moveit::core::RobotState rs = ls->getCurrentState();` followed by
    // `robotStateMsgToRobotState(req->robot_state, rs)` (`:138-139`). This
    // port's conversion builds the state from the model rather than from the
    // scene's current one -- the same thing for the `is_diff == false` case
    // upstream's helper handles by calling `setToDefaultValues()` first, and
    // the only case this port can represent at all.
    let state = match RobotState::try_from(RobotStateMsg {
        model,
        msg: request.robot_state,
    }) {
        Ok(state) => state,
        Err(e) => {
            eprintln!(
                "check_state_validity: GetStateValidity.robot_state is not representable, \
                 answering valid=false: {e}"
            );
            return GetStateValidity::Response {
                valid: false,
                ..Default::default()
            };
        }
    };
    scene.set_current_state(state);

    // `creq` exactly as upstream builds it (`:70-77`): every contact and
    // every cost source, bounded by the scene's own size.
    //
    // `getLinkModelsWithCollisionGeometry()` is upstream's cached list of
    // links whose `shapes_` is non-empty (`robot_model.cpp`'s
    // `link_models_with_collision_geometry_vector_`); this port has no such
    // cache, so the same predicate is applied to `link_models()` directly.
    let links_with_collision_geometry = model
        .link_models()
        .iter()
        .filter(|link| !link.shapes().is_empty())
        .count();
    let max_contacts = scene.world().len() + links_with_collision_geometry;
    let collision_request = CollisionRequest {
        group_name: (!request.group_name.is_empty()).then(|| request.group_name.clone()),
        cost: true,
        contacts: true,
        max_contacts: max_contacts * max_contacts,
        max_cost_sources: max_contacts,
        ..Default::default()
    };

    let env = ParryCollisionEnv::new(scene.world().clone(), Default::default());
    let planning_frame = scene.planning_frame().to_string();
    let result = scene.check_collision(&env, &collision_request);

    let mut valid = !result.collision;
    let contacts = result
        .contacts
        .iter()
        .flat_map(|data| data.by_pair.values())
        .flatten()
        .map(|contact| contact_to_msg(contact, &planning_frame))
        .collect();
    let cost_sources = result
        .cost_sources
        .iter()
        .flatten()
        .map(cost_source_to_msg)
        .collect();

    // `if (!moveit::core::isEmpty(constraints))` (`:110`): an all-default
    // `Constraints` is "none stated", and upstream skips the evaluation
    // entirely rather than deciding an empty set (which would be trivially
    // satisfied and would still emit an empty `constraint_result`).
    let mut constraint_result = Vec::new();
    if !constraints_are_empty(&request.constraints) {
        match KinematicConstraintSet::try_from(ConstraintsMsg {
            model,
            msg: request.constraints,
        }) {
            Ok(set) => {
                let evaluated = set.decide_each(&scene.current_state_mut().update());
                if evaluated.iter().any(|r| !r.satisfied) {
                    valid = false;
                }
                constraint_result = evaluated
                    .iter()
                    .map(|r| r2r::moveit_msgs::msg::ConstraintEvalResult {
                        result: r.satisfied,
                        distance: r.distance,
                    })
                    .collect();
            }
            Err(e) => {
                eprintln!(
                    "check_state_validity: GetStateValidity.constraints is not representable, \
                     answering valid=false: {e}"
                );
                valid = false;
            }
        }
    }

    GetStateValidity::Response {
        valid,
        contacts,
        cost_sources,
        constraint_result,
    }
}

/// Upstream `moveit::core::isEmpty(const moveit_msgs::msg::Constraints&)`
/// (`utils/src/message_checks.cpp:70-75`): a `Constraints` states nothing
/// when all four constraint arrays are empty. `name` is not part of the
/// test upstream, and is not here.
fn constraints_are_empty(constraints: &r2r::moveit_msgs::msg::Constraints) -> bool {
    constraints.position_constraints.is_empty()
        && constraints.orientation_constraints.is_empty()
        && constraints.visibility_constraints.is_empty()
        && constraints.joint_constraints.is_empty()
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    let [_, urdf_path, srdf_path] = args.as_slice() else {
        eprintln!("usage: move_group <urdf-path> <srdf-path>");
        return ExitCode::FAILURE;
    };

    let urdf_xml = match fs::read_to_string(urdf_path) {
        Ok(xml) => xml,
        Err(e) => {
            eprintln!("reading {urdf_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let urdf = match urdf_rs::read_from_string(&urdf_xml) {
        Ok(urdf) => urdf,
        Err(e) => {
            eprintln!("parsing {urdf_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let srdf_xml = match fs::read_to_string(srdf_path) {
        Ok(xml) => xml,
        Err(e) => {
            eprintln!("reading {srdf_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let srdf = match SrdfModel::parse_str(&srdf_xml) {
        Ok(srdf) => srdf,
        Err(e) => {
            eprintln!("parsing {srdf_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let model =
        match RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none()) {
            Ok(model) => model,
            Err(e) => {
                eprintln!("building RobotModel: {e}");
                return ExitCode::FAILURE;
            }
        };
    // `spawner.spawn_local` requires `'static` even for a same-thread
    // `LocalPool` (the "local" in `LocalSpawnExt` means !Send, not a
    // shorter lifetime) -- leaking is the standard fix for state that
    // legitimately outlives every future spawned on it, which a running
    // node's model does: the process holds it until exit.
    let model: &'static RobotModel = Box::leak(Box::new(model));

    let ctx = match r2r::Context::create() {
        Ok(ctx) => ctx,
        Err(e) => {
            eprintln!("r2r::Context::create: {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut node = match r2r::Node::create(ctx, "moveit_ros", "") {
        Ok(node) => node,
        Err(e) => {
            eprintln!("r2r::Node::create: {e}");
            return ExitCode::FAILURE;
        }
    };
    let service = match node
        .create_service::<GetMotionPlan::Service>("plan_kinematic_path", QosProfile::default())
    {
        Ok(service) => service,
        Err(e) => {
            eprintln!("create_service(plan_kinematic_path): {e}");
            return ExitCode::FAILURE;
        }
    };

    // Upstream's name for this action, verbatim: `move_group::MOVE_ACTION`
    // (`moveit_ros/move_group/include/moveit/move_group/capability_names.hpp:52`)
    // is the unqualified `"move_action"`, and
    // `MoveGroupInterface` resolves it with `rclcpp::names::append(namespace,
    // MOVE_ACTION)` -- so a leading slash here would put the server at
    // `/move_action` for a default-namespace client and out of reach of a
    // namespaced one.
    let move_action = match node.create_action_server::<MoveGroup::Action>("move_action") {
        Ok(server) => server,
        Err(e) => {
            eprintln!("create_action_server(move_action): {e}");
            return ExitCode::FAILURE;
        }
    };

    // Upstream `PlanningSceneMonitor::startSceneMonitor`'s two arguments,
    // verbatim (`planning_scene_monitor.cpp:1205-1208`):
    //
    // * the topic is the `scene_topic` parameter's default,
    //   `DEFAULT_PLANNING_SCENE_TOPIC` (`:74`), which is the *unqualified*
    //   `"planning_scene"` -- the header's own comment says `/planning_scene`
    //   but the string has no slash, and it is resolved against the node's
    //   namespace like every other unqualified name (the same distinction
    //   `move_action` above turns on).
    // * the QoS is `rclcpp::ServicesQoS()`, which is
    //   `rmw_qos_profile_services_default`: keep-last-10, reliable, volatile.
    //   `QosProfile::services_default()` is that profile field for field
    //   (r2r `qos.rs:527-535`). A publisher on the default `QoSInitialization`
    //   profile is compatible with it; a best-effort publisher is not, and
    //   that incompatibility is upstream's too, not this port's.
    let scene_updates = match node.subscribe::<r2r::moveit_msgs::msg::PlanningScene>(
        "planning_scene",
        QosProfile::services_default(),
    ) {
        Ok(stream) => stream,
        Err(e) => {
            eprintln!("subscribe(planning_scene): {e}");
            return ExitCode::FAILURE;
        }
    };

    // `PlanningSceneMonitor::startStateMonitor` opens this one
    // (`planning_scene_monitor.cpp:1384-1388`) with the same
    // `rclcpp::ServicesQoS()` and the same unqualified default topic name,
    // `DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC` (`:71`). It is a plain
    // subscription, not a message filter -- upstream's own comment on the
    // line says why: "using regular message filter as there's no header".
    let attached_objects = match node.subscribe::<r2r::moveit_msgs::msg::AttachedCollisionObject>(
        "attached_collision_object",
        QosProfile::services_default(),
    ) {
        Ok(stream) => stream,
        Err(e) => {
            eprintln!("subscribe(attached_collision_object): {e}");
            return ExitCode::FAILURE;
        }
    };

    // `TrajectoryExecutionManager`'s own subscription
    // (`trajectory_execution_manager.cpp:204-206`), same QoS again. Upstream
    // puts it on a dedicated callback group so a `stop` can be processed
    // while another callback is running (`:199-202`); this node is
    // single-threaded and has no long-running callback to preempt, so there
    // is no group to port -- the reason that matters is that nothing here
    // blocks, not that the concern does not exist.
    let execution_events = match node.subscribe::<r2r::std_msgs::msg::String>(
        "trajectory_execution_event",
        QosProfile::services_default(),
    ) {
        Ok(stream) => stream,
        Err(e) => {
            eprintln!("subscribe(trajectory_execution_event): {e}");
            return ExitCode::FAILURE;
        }
    };

    let state_validity = match node
        .create_service::<GetStateValidity::Service>("check_state_validity", QosProfile::default())
    {
        Ok(service) => service,
        Err(e) => {
            eprintln!("create_service(check_state_validity): {e}");
            return ExitCode::FAILURE;
        }
    };

    // Upstream `move_group::EXECUTE_ACTION_NAME`
    // (`moveit_ros/move_group/include/moveit/move_group/capability_names.hpp:45`),
    // verbatim and unqualified for the reason `move_action` above is. The
    // literal is written here rather than referenced from a constant in
    // [`moveit_ros::execute_trajectory`] on purpose:
    // `tools/ci/measure-client-endpoint-surface.py`'s `PORT_OPENER` matches a
    // string literal inside the factory call, so a named constant would leave
    // this endpoint reading `absent` in `doc/client-endpoint-surface.md` with
    // the server running -- a wrong measurement, not a missing one.
    let execute_trajectory =
        match node.create_action_server::<ExecuteTrajectory::Action>("execute_trajectory") {
            Ok(server) => server,
            Err(e) => {
                eprintln!("create_action_server(execute_trajectory): {e}");
                return ExitCode::FAILURE;
            }
        };

    // The two descriptions the client's own constructor blocks on. Both or
    // neither, and fatal on failure -- the invariant and the reason a half
    // latch is worse than none are in
    // [`moveit_ros::robot_description`]. Held for the process's life: the
    // samples are transient-local, so dropping this would leave the client
    // subscribing to a topic that answers nothing.
    let _descriptions = match robot_description::latch(&mut node, &urdf_xml, &srdf_xml) {
        Ok(descriptions) => descriptions,
        Err(e) => {
            eprintln!("latching robot_description/robot_description_semantic: {e}");
            return ExitCode::FAILURE;
        }
    };

    // `CurrentStateMonitorMiddlewareHandle::createJointStateSubscription`
    // subscribes with `rclcpp::SystemDefaultsQoS()`
    // (`current_state_monitor_middleware_handle.cpp:69-74`), whose
    // reliability and durability the rmw resolves to reliable/volatile.
    // `QosProfile::default()` is that, which is also what
    // `robot_state_publisher` and `joint_state_publisher` send with; a
    // best-effort publisher here would be silently incompatible with
    // upstream's subscriber and the client would simply never get a state.
    //
    // Literal at the call site for the reason `execute_trajectory`'s
    // registration above records.
    let joint_state_publisher =
        match node.create_publisher::<JointState>("joint_states", QosProfile::default()) {
            Ok(publisher) => publisher,
            Err(e) => {
                eprintln!("create_publisher(joint_states): {e}");
                return ExitCode::FAILURE;
            }
        };
    // 10 Hz. The rate has to clear `getCurrentState`'s own budget --
    // `waitForCurrentState(node_->now(), wait_seconds)` with
    // `wait_seconds` defaulting to 1.0 (`move_group_interface.cpp:635`) --
    // and a period of 100 ms puts the first message stamped after any such
    // call inside a tenth of it.
    let mut joint_state_timer = match node.create_timer(Duration::from_millis(100)) {
        Ok(timer) => timer,
        Err(e) => {
            eprintln!("create_timer(joint_states): {e}");
            return ExitCode::FAILURE;
        }
    };
    // Resolved once, here, where a model this topic cannot describe is a
    // reason to refuse to start rather than an error every 100 ms.
    let joint_sampler = match JointSampler::new(model) {
        Ok(sampler) => sampler,
        Err(e) => {
            eprintln!("joint_states sampler: {e}");
            return ExitCode::FAILURE;
        }
    };
    let clock = node.get_ros_clock();

    // Upstream `move_group::CARTESIAN_PATH_SERVICE_NAME`
    // (`moveit_ros/move_group/include/moveit/move_group/capability_names.hpp:59-60`
    // -- the basename alone is ambiguous, pilz ships one too).
    // Spelled as a literal rather than as
    // `moveit_ros::cartesian_path::SERVICE_NAME`, which holds the same string:
    // `tools/ci/measure-client-endpoint-surface.py`'s `PORT_OPENER` matches a
    // string literal in the factory call, so a constant here would leave the
    // endpoint reading `absent` in `doc/client-endpoint-surface.md` while the
    // server was in fact up.
    let cartesian_path = match node.create_service::<GetCartesianPath::Service>(
        "compute_cartesian_path",
        QosProfile::default(),
    ) {
        Ok(service) => service,
        Err(e) => {
            eprintln!("create_service(compute_cartesian_path): {e}");
            return ExitCode::FAILURE;
        }
    };

    // Leaked for the same reason `model` is: `spawn_local` requires
    // `'static`, and this outlives every future spawned on it.
    let srdf: &'static SrdfModel = Box::leak(Box::new(srdf));
    let scene: MonitoredScene = monitored_scene::new(model, srdf);

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Upstream's `MoveGroupQueryPlannersService` is one capability serving
    // three services off one configuration map, so it registers as one thing
    // here too -- see `moveit_ros::planner_params`.
    // The returned store is the one the three services share, and the same
    // handle every plan below is built from -- upstream hands its map to the
    // planner instance from inside `setParams`; here the node holds it and
    // the planner is constructed from it (PORTING-PLAN.md §285).
    let planner_configs = match moveit_ros::planner_params::spawn(&mut node, &spawner) {
        Ok(configs) => configs,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };

    let scene_for_plan_service = Rc::clone(&scene);
    let configs_for_plan_service = Rc::clone(&planner_configs);
    let spawned = spawner.spawn_local(async move {
        let mut service = service;
        while let Some(req) = service.next().await {
            // Scoped exactly as `check_state_validity`'s is: the `RefCell`
            // borrow ends before the reply, so the subscription task can
            // install a new snapshot while this one is still being planned
            // against.
            let response = {
                let snapshot = Arc::clone(&scene_for_plan_service.borrow());
                handle_request(&snapshot, &configs_for_plan_service, req.message.clone())
            };
            if let Err(e) = req.respond(response) {
                eprintln!("responding to plan_kinematic_path request: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning service task: {e}");
        return ExitCode::FAILURE;
    }

    let scene_for_move_action = Rc::clone(&scene);
    let configs_for_move_action = Rc::clone(&planner_configs);
    let spawned = spawner.spawn_local(async move {
        let mut requests = move_action;
        while let Some(request) = requests.next().await {
            // Upstream's goal callback is a constant
            // `GoalResponse::ACCEPT_AND_EXECUTE` -- it inspects neither the
            // UUID nor the goal (`move_action_capability.cpp:70-74`), so
            // there is no rejection branch to port.
            let (mut goal, _cancel) = match request.accept() {
                Ok(accepted) => accepted,
                Err(e) => {
                    eprintln!("accepting move_action goal: {e}");
                    continue;
                }
            };
            // `setMoveState(PLANNING, goal_)` (`:89`), which publishes the
            // state name as feedback. The matching `setMoveState(IDLE, ...)`
            // at `:126` is *not* ported: it runs after `succeed`/`abort` has
            // already terminated the goal, where rclcpp_action logs an error
            // and publishes nothing. Porting a call whose only effect
            // upstream is that error would put the error in this node's log
            // instead.
            if let Err(e) = goal.publish_feedback(MoveGroup::Feedback {
                state: "PLANNING".to_string(),
            }) {
                eprintln!("publishing move_action PLANNING feedback: {e}");
            }
            let result = {
                let snapshot = Arc::clone(&scene_for_move_action.borrow());
                handle_move_group_goal(&snapshot, &configs_for_move_action, goal.goal.clone())
            };
            // Upstream's three-way terminal branch (`:113-124`). Its
            // `PREEMPTED` arm has no counterpart: nothing here sets that
            // code, because this node ports neither `preempt_requested_` nor
            // a cancel callback, so a `canceled` arm would be unreachable by
            // construction rather than merely unexercised.
            let outcome = if result.error_code.val == MoveItErrorCodes::SUCCESS as i32 {
                goal.succeed(result)
            } else {
                goal.abort(result)
            };
            if let Err(e) = outcome {
                eprintln!("terminating move_action goal: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning move_action task: {e}");
        return ExitCode::FAILURE;
    }

    // `newPlanningSceneCallback` (`planning_scene_monitor.cpp:711`) hands the
    // message straight to `newPlanningSceneMessage` (`:739`).
    //
    // Upstream's `newPlanningSceneMessage` has two arms: with a
    // `parent_scene_` it routes a *full* scene into the parent and clears the
    // child's diffs, and otherwise it calls `usePlanningSceneMsg`
    // (`:759-772`). `parent_scene_` is set in exactly one place --
    // `startPublishingPlanningScene` (`:388`), i.e. only when the monitor is
    // also *republishing* a monitored scene, which this node does not do -- so
    // the `usePlanningSceneMsg` arm is the reachable one and the only one
    // ported.
    let scene_for_updates = Rc::clone(&scene);
    let spawned = spawner.spawn_local(async move {
        let mut updates = scene_updates;
        while let Some(msg) = updates.next().await {
            // Upstream returns the `bool` its callers ignore
            // (`newPlanningSceneCallback` discards it outright); this port has
            // a real error to report, so it reports it. `apply` is what
            // guarantees the previous scene is still installed after a
            // rejection -- see `monitored_scene`'s module doc for why that
            // lives there and not here.
            if let Err(e) = monitored_scene::apply_planning_scene_msg(&scene_for_updates, msg) {
                eprintln!("planning_scene update rejected, scene unchanged: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning planning_scene subscription task: {e}");
        return ExitCode::FAILURE;
    }

    // `PlanningSceneMonitor::processAttachedCollisionObjectMsg`
    // (`planning_scene_monitor.cpp:841`). Upstream returns `false` on a
    // rejected object *after* having already mutated the scene under the
    // lock (`:853`); `apply` cannot leave that state behind, so a rejection
    // here is reported and changes nothing.
    let scene_for_attach = Rc::clone(&scene);
    let spawned = spawner.spawn_local(async move {
        let mut objects = attached_objects;
        while let Some(msg) = objects.next().await {
            let id = msg.object.id.clone();
            if let Err(e) =
                monitored_scene::apply_attached_collision_object_msg(&scene_for_attach, msg)
            {
                eprintln!("attached_collision_object '{id}' rejected, scene unchanged: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning attached_collision_object subscription task: {e}");
        return ExitCode::FAILURE;
    }

    // `TrajectoryExecutionManager::receiveEvent`
    // (`trajectory_execution_manager.cpp:355`), which logs the payload and
    // hands it to `processEvent` (`:343`). The state this transitions lives
    // in `moveit_ros::execution`; the task owns the one instance so nothing
    // else can reach the transition.
    let spawned = spawner.spawn_local(async move {
        let mut events = execution_events;
        let mut execution = TrajectoryExecution::new();
        while let Some(msg) = events.next().await {
            match ExecutionEvent::try_from(ExecutionEventMsg(msg)) {
                Ok(ExecutionEvent::Stop) => match execution.stop() {
                    StopOutcome::Preempted => {
                        // Upstream's "Stopped trajectory execution." (`:1226`).
                        eprintln!("trajectory_execution_event stop: preempted execution");
                    }
                    StopOutcome::NothingToStop => {
                        eprintln!(
                            "trajectory_execution_event stop: nothing to stop, \
                             no trajectory is executing"
                        );
                    }
                },
                // Upstream's `RCLCPP_WARN_STREAM("Unknown event type: ...")`
                // (`:351`) -- same severity, and the payload is named for the
                // same reason.
                Err(e) => eprintln!("trajectory_execution_event ignored: {e}"),
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning trajectory_execution_event subscription task: {e}");
        return ExitCode::FAILURE;
    }

    // The whole registration: this capability owns its goals end to end in
    // [`moveit_ros::execute_trajectory::serve`], so no goal handle, no result
    // and no terminal transition is visible here. See that module for which of
    // the two possible servers this is -- it is the no-execution-backend one,
    // answering upstream's own `CONTROL_FAILED`, not a server reporting
    // `SUCCESS` having executed nothing.
    let spawned = spawner.spawn_local(execute_trajectory::serve(execute_trajectory));
    if let Err(e) = spawned {
        eprintln!("spawning execute_trajectory task: {e}");
        return ExitCode::FAILURE;
    }

    // The stand-in for the robot driver. Upstream's `move_group` subscribes
    // to this topic and never publishes it; what this node publishes is the
    // monitored scene's own current state, so a client's `getCurrentState()`
    // reads back the state this node plans from. See
    // [`moveit_ros::joint_states`] for that deviation and for why a latched
    // message could not serve this endpoint.
    let scene_for_joint_states = Rc::clone(&scene);
    let spawned = spawner.spawn_local(async move {
        while joint_state_timer.tick().await.is_ok() {
            // Read per tick, not per batch: the stamp is what
            // `waitForCurrentState` compares against the caller's own
            // `now()`, so a stamp taken once at startup would satisfy no
            // call ever made after it.
            let stamp = {
                let mut clock = match clock.lock() {
                    Ok(clock) => clock,
                    Err(e) => {
                        eprintln!("locking the ROS clock for joint_states: {e}");
                        continue;
                    }
                };
                match clock.get_now() {
                    Ok(now) => r2r::Clock::to_builtin_time(&now),
                    Err(e) => {
                        eprintln!("reading the ROS clock for joint_states: {e}");
                        continue;
                    }
                }
            };
            // Scoped like every other reader here: the borrow ends before
            // the next `.await`.
            let message = {
                let snapshot = Arc::clone(&scene_for_joint_states.borrow());
                joint_sampler.sample(snapshot.current_state(), stamp)
            };
            // Reported and retried on the next tick rather than ending the
            // loop: a publish that fails once must not stop every later
            // `getCurrentState()` from ever completing.
            if let Err(e) = joint_state_publisher.publish(&message) {
                eprintln!("publishing joint_states: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning joint_states task: {e}");
        return ExitCode::FAILURE;
    }

    let scene_for_validity = Rc::clone(&scene);
    let spawned = spawner.spawn_local(async move {
        let mut service = state_validity;
        while let Some(req) = service.next().await {
            let response = {
                let snapshot = monitored_scene::snapshot(&scene_for_validity);
                handle_state_validity(&snapshot, req.message.clone())
            };
            if let Err(e) = req.respond(response) {
                eprintln!("responding to check_state_validity request: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning check_state_validity task: {e}");
        return ExitCode::FAILURE;
    }

    let scene_for_cartesian = Rc::clone(&scene);
    let spawned = spawner.spawn_local(async move {
        let mut service = cartesian_path;
        while let Some(req) = service.next().await {
            let response = {
                let snapshot = Arc::clone(&scene_for_cartesian.borrow());
                moveit_ros::cartesian_path::handle(&snapshot, req.message.clone())
            };
            if let Err(e) = req.respond(response) {
                eprintln!("responding to compute_cartesian_path request: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning compute_cartesian_path task: {e}");
        return ExitCode::FAILURE;
    }

    loop {
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
