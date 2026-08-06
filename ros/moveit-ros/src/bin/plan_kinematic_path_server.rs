// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The `moveit-ros` node: upstream's `move_group` capabilities that Phase 9's
//! completion condition names, on one `r2r::Node`.
//!
//! Two endpoints are hosted here, matching upstream's own arrangement --
//! `move_group` loads `MoveGroupPlanService` and `MoveGroupMoveAction` as
//! two capabilities of a single node, not two processes:
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
//!
//! The file name is §241's and now names only half of what this binary hosts.
//! Renaming it means changing `cargo build --bin plan_kinematic_path_server`
//! in `ros/verify-ros-interop.sh`, which is outside the fence of the round
//! that added `/move_action`.
//!
//! # Both endpoints plan
//!
//! Each converts its incoming `moveit_msgs/MotionPlanRequest` into a
//! [`moveit_planning::PlanningRequest`] (the existing
//! `TryFrom<PlanningRequestMsg>` impl), hands it to
//! [`moveit_ros::move_group::plan_only`], and encodes whatever comes back.
//! That function is upstream's own two steps --
//! `MoveGroupCapability::resolvePlanningPipeline` followed by
//! `PlanningPipeline::generatePlan` -- and both capabilities call them in
//! that order (`plan_service_capability.cpp:79-97`,
//! `move_action_capability.cpp:206-227`), which is why one function serves
//! both here too.
//!
//! Until PORTING-PLAN.md D8 landed there was nothing to call: no planner
//! crate depended on `moveit-planning`, and
//! `moveit_planners_sbp::registry`'s `PlanningRequest` shared only a *name*
//! with `moveit_planning`'s. D8 merged the two, so
//! `RrtConnectManager` now implements [`moveit_planning::PlannerManager`]
//! and reaches these endpoints through
//! `moveit_planner_registry::PLANNER_MANAGERS`.
//!
//! # What still cannot round-trip
//!
//! A `MotionPlanRequest` with a non-default `start_state` is rejected by the
//! conversion, not planned -- [`moveit_planning::PlanningRequest`] has no
//! start-state field. That is the shape an unmodified
//! `MoveGroupInterface::plan()` always sends, so upstream's own client still
//! gets a typed rejection from these endpoints rather than a trajectory.
//!
//! There is also no scene monitor: each request plans against a freshly
//! built [`moveit_scene::PlanningScene`] at the model's default state, where
//! upstream plans against the monitored scene
//! (`planning_scene_monitor_->copyPlanningScene(...)`,
//! `move_action_capability.cpp:216-217`). A request's
//! `planning_options.planning_scene_diff` is therefore ignored, and no
//! collision object a caller published is in the scene the planner sees.
//!
//! # Both endpoints report failure as `FAILURE`
//!
//! Upstream reaches "no planning pipeline" through `resolvePlanningPipeline`
//! returning null and "the pipeline ran and did not solve" through
//! `generatePlan` returning false, and encodes *both* as
//! `MoveItErrorCodes::FAILURE` in both capabilities
//! (`move_action_capability.cpp:207-211,219-227`,
//! `plan_service_capability.cpp:82-85,92-97`), reserving `PLANNING_FAILED`
//! for elsewhere. The service used to answer `PLANNING_FAILED`, a parity
//! defect carried over from §241; it answers `FAILURE` now.

use std::env;
use std::fs;
use std::process::ExitCode;
use std::time::Duration;

use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use moveit_collision::ParryCollisionEnv;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planning::PlanningRequest;
use moveit_ros::move_group::plan_only;
use moveit_ros::planning::{PlanningRequestMsg, PlanningResponseMsgOut};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use r2r::QosProfile;
use r2r::moveit_msgs::action::MoveGroup;
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::moveit_msgs::srv::GetMotionPlan;

/// `MoveItErrorCodes::source` for each endpoint. Upstream leaves the field
/// empty; filling it is what lets `ros/verify-move-action-interop.sh` tell
/// a reply that crossed DDS from this node apart from one
/// `MoveGroupInterface` synthesised locally when no server answered
/// (`move_group_interface.cpp:659-663`).
const PLAN_SERVICE_SOURCE: &str = "moveit-ros/plan_kinematic_path_server";
const MOVE_ACTION_SOURCE: &str = "moveit-ros/move_action";

/// A non-`SUCCESS` answer, minus its `source`: the `val` and `message` are
/// the same whichever endpoint asked, and the `source` names the endpoint,
/// which only the caller knows.
struct PlanFailure {
    val: i32,
    message: String,
}

impl PlanFailure {
    fn into_error_code(self, source: &str) -> MoveItErrorCodes {
        MoveItErrorCodes {
            val: self.val,
            message: self.message,
            source: source.to_string(),
        }
    }
}

/// Both capabilities' shared body: convert the wire request, plan, encode.
///
/// The three failures are upstream's, in upstream's order -- a request that
/// does not convert (no upstream analogue: `MotionPlanRequest` *is*
/// upstream's planning request, so there is nothing there to reject),
/// an unresolved pipeline or an unsolved plan (both `FAILURE`,
/// `move_action_capability.cpp:207-211,219-227`), and encoding the answer
/// back onto the wire (no upstream analogue either: `getMessage` cannot
/// fail).
///
/// The scene is built here, per request, and thrown away after: upstream
/// takes a copy of the monitored scene instead
/// (`plan_service_capability.cpp:89`, `move_action_capability.cpp:216-217`)
/// and this port has no monitor to copy from. Building it fresh each time is
/// what keeps a planner that leaves the current state where it finished from
/// changing where the *next* request starts.
fn plan(
    model: &RobotModel,
    srdf: &SrdfModel,
    env: &ParryCollisionEnv,
    msg: r2r::moveit_msgs::msg::MotionPlanRequest,
) -> Result<r2r::moveit_msgs::msg::MotionPlanResponse, PlanFailure> {
    // Read before the move below: `PlanningRequest` has no `pipeline_id`
    // field (`doc/message-mapping.md` records it as dropped), so the
    // selection upstream makes from this field has to be made off the
    // message.
    let pipeline_id = msg.pipeline_id.clone();

    let request =
        PlanningRequest::try_from(PlanningRequestMsg { model, msg }).map_err(|e| PlanFailure {
            val: MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS as i32,
            message: format!("MotionPlanRequest -> PlanningRequest: {e}"),
        })?;

    let mut scene = PlanningScene::new(model, srdf);
    let response = plan_only(&mut scene, env, &pipeline_id, request).map_err(|e| PlanFailure {
        val: MoveItErrorCodes::FAILURE as i32,
        message: e.to_string(),
    })?;

    PlanningResponseMsgOut::try_from(response)
        .map(|out| out.0)
        .map_err(|e| PlanFailure {
            val: MoveItErrorCodes::FAILURE as i32,
            message: format!("PlanningResponse -> MotionPlanResponse: {e}"),
        })
}

/// `MoveGroupPlanService::computePlanService`
/// (`plan_service_capability.cpp:70-105`).
fn handle_request(
    model: &RobotModel,
    srdf: &SrdfModel,
    env: &ParryCollisionEnv,
    msg: GetMotionPlan::Request,
) -> GetMotionPlan::Response {
    match plan(model, srdf, env, msg.motion_plan_request) {
        Ok(motion_plan_response) => GetMotionPlan::Response {
            motion_plan_response,
        },
        Err(failure) => GetMotionPlan::Response {
            motion_plan_response: r2r::moveit_msgs::msg::MotionPlanResponse {
                error_code: failure.into_error_code(PLAN_SERVICE_SOURCE),
                ..Default::default()
            },
        },
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
/// `executed_trajectory` stays empty for the same reason, and
/// `planning_time` stays `0.0`: upstream fills it from
/// `MotionPlanResponse::planning_time` (`:228`), which
/// [`moveit_planning::PlanningResponse`] has no field for -- see that type's
/// own doc comment. Reporting this handler's wall clock there would put a
/// different number under upstream's name for one.
fn handle_move_group_goal(
    model: &RobotModel,
    srdf: &SrdfModel,
    env: &ParryCollisionEnv,
    goal: MoveGroup::Goal,
) -> MoveGroup::Result {
    if !goal.planning_options.plan_only {
        eprintln!(
            "This instance of MoveGroup is not allowed to execute trajectories \
             but the goal request has plan_only set to false. Only a motion \
             plan will be computed anyway."
        );
    }

    match plan(model, srdf, env, goal.request) {
        // `convertToMsg(res.trajectory, action_res->trajectory_start,
        // action_res->planned_trajectory)` (`:225`) -- the same two fields
        // `MotionPlanResponse` carries them in, moved across.
        Ok(response) => MoveGroup::Result {
            error_code: response.error_code,
            trajectory_start: response.trajectory_start,
            planned_trajectory: response.trajectory,
            ..Default::default()
        },
        Err(failure) => MoveGroup::Result {
            error_code: failure.into_error_code(MOVE_ACTION_SOURCE),
            ..Default::default()
        },
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    let [_, urdf_path, srdf_path] = args.as_slice() else {
        eprintln!("usage: plan_kinematic_path_server <urdf-path> <srdf-path>");
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
    // node's model does: the process holds it until exit. The SRDF and the
    // collision environment are leaked for the same reason and read by both
    // endpoints: `PlanningScene::new` needs the SRDF for its ACM, and
    // `plan_only` needs an environment to check states against.
    let model: &'static RobotModel = Box::leak(Box::new(model));
    let srdf: &'static SrdfModel = Box::leak(Box::new(srdf));
    // Empty: this node has no planning-scene subscription, so there is no
    // world to fill it from. Every plan below is therefore checked against
    // self-collision and joint limits only -- see this binary's module doc.
    let env: &'static ParryCollisionEnv = Box::leak(Box::new(ParryCollisionEnv::default()));

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

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let spawned = spawner.spawn_local(async move {
        let mut service = service;
        while let Some(req) = service.next().await {
            let response = handle_request(model, srdf, env, req.message.clone());
            if let Err(e) = req.respond(response) {
                eprintln!("responding to plan_kinematic_path request: {e}");
            }
        }
    });
    if let Err(e) = spawned {
        eprintln!("spawning service task: {e}");
        return ExitCode::FAILURE;
    }

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
            let result = handle_move_group_goal(model, srdf, env, goal.goal.clone());
            // Upstream's three-way terminal branch (`:113-124`). Its
            // `PREEMPTED` arm has no counterpart: nothing here sets that
            // code, because this node ports neither `preempt_requested_`
            // nor a cancel callback, so a `canceled` arm would be
            // unreachable by construction rather than merely unexercised.
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

    loop {
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
