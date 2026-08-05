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
//! * `/move_action` (`moveit_msgs/action/MoveGroup`), PORTING-PLAN.md §NEW --
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
//! # What this does not do, and why
//!
//! Both endpoints convert their incoming `moveit_msgs/MotionPlanRequest` into
//! a [`moveit_planning::PlanningRequest`] (the existing
//! `TryFrom<PlanningRequestMsg>` impl), then stop -- neither calls a planner,
//! so every reply carries an empty trajectory and a non-`SUCCESS`
//! `error_code`.
//!
//! That stop is measured, not a shortcut: `rg -n "impl.*Planner<'m>.*for"
//! crates/` (repeated this round, same as `moveit_planning::response`'s own
//! doc comment already recorded) finds zero hits outside
//! `moveit-planning`'s own test fixtures, and `rg -n moveit-planning
//! crates/moveit-planners-{sbp,chomp,stomp,pilz}/Cargo.toml` finds zero
//! hits -- no planner crate in this workspace depends on `moveit-planning`
//! at all, so no concrete [`moveit_planning::pipeline::Planner`] exists to
//! hand `generate_plan` here or anywhere else.
//!
//! The nearest candidate, `moveit_planners_sbp::registry::RrtConnectManager`,
//! cannot be bridged by a local adapter in this crate without re-deciding
//! `PORTING-PLAN.md` D8 out from under it: `crates/moveit-planning`'s
//! `PlanningRequest`/`PlanningResponse` and `moveit-planners-sbp::registry`'s
//! own same-named types are two distinct types today (confirmed this round
//! by re-reading `registry.rs:270-340` and `PORTING-PLAN.md` §140.2, not
//! assumed stale), and D8 (§140.3) is the standing, already-decided,
//! already-preconditioned plan to unify them -- explicitly *not* started
//! yet ("이건 구조적 해결을 미루는 게 아니라 순서다: 지금 하면 같은
//! 파일을 두 라운드가 동시에 고친다"). Writing a one-off translation here
//! instead would be exactly the "clever patch" CLAUDE.md's
//! structural-fix-over-patch rule warns against: it would leave `goal`
//! meaning two different things across the crate boundary for the next
//! round to untangle, for the sake of this round alone. Also out of this
//! binary's fence regardless (`ros/moveit-ros` only) -- adding a
//! `moveit-planners-sbp` dependency here is defensible; writing the
//! adapter D8 already owns is not.
//!
//! A caller therefore gets a genuine round trip -- a real
//! `moveit_msgs/GetMotionPlan` request, over live DDS, converted through
//! the same `TryFrom` impls the wire tests exercise in-process -- and a
//! genuine, typed failure, never a fabricated trajectory.
//!
//! # The two endpoints report that state with different `error_code`s
//!
//! `/move_action` reports it as `MoveItErrorCodes::FAILURE` and
//! `/plan_kinematic_path` as `PLANNING_FAILED`. `FAILURE` is the correct one
//! for both: upstream reaches "there is no planning pipeline" through
//! `resolvePlanningPipeline` returning null, and both capabilities encode
//! *that* as `FAILURE` (`move_action_capability.cpp:207-211`,
//! `plan_service_capability.cpp:82-85`), reserving `PLANNING_FAILED` for a
//! pipeline that ran and did not solve. The service's `PLANNING_FAILED` is a
//! parity defect carried over from §241; correcting it also means changing
//! the `grep -q "val=-1"` assertion in `ros/verify-ros-interop.sh`, which is
//! outside the fence of the round that found it.

use std::env;
use std::fs;
use std::process::ExitCode;
use std::time::Duration;

use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planning::PlanningRequest;
use moveit_ros::planning::PlanningRequestMsg;
use moveit_srdf::SrdfModel;
use r2r::QosProfile;
use r2r::moveit_msgs::action::MoveGroup;
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::moveit_msgs::srv::GetMotionPlan;

/// The one sentence both endpoints send back in place of a trajectory.
const NO_PLANNER: &str = "moveit-ros has no moveit_planning::pipeline::Planner to call yet \
     (PORTING-PLAN.md §241): the request converted, but there is no \
     planner in this workspace to hand it to.";

/// Builds a `MotionPlanResponse` carrying no trajectory and the given
/// `val`/`message` -- the shape every non-success reply in this binary
/// uses, since `moveit_ros::planning`'s `TryFrom<PlanningResponse> for
/// PlanningResponseMsgOut` only ever encodes `SUCCESS` (it has no `Err`
/// input to encode from: a `PlanningResponse` value only exists once a
/// planner already succeeded).
fn failure_response(val: i32, message: &str) -> GetMotionPlan::Response {
    GetMotionPlan::Response {
        motion_plan_response: r2r::moveit_msgs::msg::MotionPlanResponse {
            error_code: MoveItErrorCodes {
                val,
                message: message.to_string(),
                source: "moveit-ros/plan_kinematic_path_server".to_string(),
            },
            ..Default::default()
        },
    }
}

/// Converts the request, then reports why this round stops there. See this
/// binary's own module doc for the full explanation.
fn handle_request(model: &RobotModel, msg: GetMotionPlan::Request) -> GetMotionPlan::Response {
    let planning_request = match PlanningRequest::try_from(PlanningRequestMsg {
        model,
        msg: msg.motion_plan_request,
    }) {
        Ok(request) => request,
        Err(e) => {
            return failure_response(
                MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS as i32,
                &format!("MotionPlanRequest -> PlanningRequest: {e}"),
            );
        }
    };

    // The conversion above is the whole of what this round wires up -- see
    // the module doc for why calling a planner is not this round's decision
    // to make. `planning_request` is otherwise unused past proving the
    // conversion above actually ran.
    let _ = &planning_request;
    failure_response(MoveItErrorCodes::PLANNING_FAILED as i32, NO_PLANNER)
}

/// A `MoveGroup` result carrying no trajectory and the given `val`/`message`.
///
/// `planning_time` stays `0.0` rather than being timed: upstream sets it from
/// `MotionPlanResponse::planning_time`, which only a pipeline that ran can
/// produce (`move_action_capability.cpp:228`). Reporting the wall time this
/// handler spent converting under that field would put a number where
/// upstream has planning time.
fn move_group_failure(val: i32, message: &str) -> MoveGroup::Result {
    MoveGroup::Result {
        error_code: MoveItErrorCodes {
            val,
            message: message.to_string(),
            source: "moveit-ros/move_action".to_string(),
        },
        ..Default::default()
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
/// Inside that arm, upstream selects a pipeline and returns
/// `MoveItErrorCodes::FAILURE` when none resolves (`:207-211`). No pipeline
/// can resolve here for the reason this binary's module doc measures, so that
/// is the branch this port lands in and `FAILURE` is the code it reports.
fn handle_move_group_goal(model: &RobotModel, goal: MoveGroup::Goal) -> MoveGroup::Result {
    if !goal.planning_options.plan_only {
        eprintln!(
            "This instance of MoveGroup is not allowed to execute trajectories \
             but the goal request has plan_only set to false. Only a motion \
             plan will be computed anyway."
        );
    }

    if let Err(e) = PlanningRequest::try_from(PlanningRequestMsg {
        model,
        msg: goal.request,
    }) {
        return move_group_failure(
            MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS as i32,
            &format!("MotionPlanRequest -> PlanningRequest: {e}"),
        );
    }

    move_group_failure(MoveItErrorCodes::FAILURE as i32, NO_PLANNER)
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
    // (`capability_names.hpp:52`) is the unqualified `"move_action"`, and
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
            let response = handle_request(model, req.message.clone());
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
            let result = handle_move_group_goal(model, goal.goal.clone());
            // `error_code.val != SUCCESS && != PREEMPTED` is upstream's
            // `abort` arm (`:113-124`); neither of the other two is
            // reachable while no planner exists to return SUCCESS and no
            // cancel path sets PREEMPTED.
            if let Err(e) = goal.abort(result) {
                eprintln!("aborting move_action goal: {e}");
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
