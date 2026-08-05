// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The `/plan_kinematic_path` service host (PORTING-PLAN.md §NEW): the
//! `fn main`/`r2r::Node`/`spin` node entry point plus the service
//! registration §NEW.5 names as the smallest reachable piece of Phase 9's
//! completion condition, wired to `moveit_ros::planning`'s existing
//! `TryFrom` conversions (`PlanningRequestMsg`/`PlanningResponseMsgOut`).
//!
//! # What this does not do, and why
//!
//! This binary converts an incoming `moveit_msgs/GetMotionPlan` request's
//! `motion_plan_request` into a [`moveit_planning::PlanningRequest`] (the
//! existing `TryFrom<PlanningRequestMsg>` impl), then stops -- it never
//! calls a planner, so every response it sends carries an empty trajectory
//! and a non-`SUCCESS` `error_code`.
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
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::moveit_msgs::srv::GetMotionPlan;

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
    failure_response(
        MoveItErrorCodes::PLANNING_FAILED as i32,
        "moveit-ros has no moveit_planning::pipeline::Planner to call yet \
         (PORTING-PLAN.md §NEW): the request converted, but there is no \
         planner in this workspace to hand it to.",
    )
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

    loop {
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
