// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `MoveGroupCartesianPathService::computeService`
//! (`moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp:97-222`),
//! as far as it is expressible against this workspace's own types.
//!
//! Here in the library rather than inline in `src/bin/move_group.rs` for the
//! reason `crate::move_group` gives: a `[[bin]]`'s functions are reachable
//! only from that binary, so the node's handler and this module's tests have
//! to be the same code for the tests to say anything about the node.
//!
//! # The fraction is the answer, not a diagnostic
//!
//! `GetCartesianPath::Response` has four fields and the unmodified C++ client
//! branches on two of them:
//! `MoveGroupInterfaceImpl::computeCartesianPath` returns `-1.0` for any
//! `error_code.val != SUCCESS` and otherwise returns `response->fraction`
//! verbatim (`move_group_interface.cpp:896-905`). `fraction` is therefore the
//! return value of the client's own function, and a `SUCCESS` carrying a
//! fraction computed from a start state, a seed or a frame other than the one
//! the request named is worse than no answer: the client has no second field
//! to catch it with. Every branch below that cannot reproduce upstream's
//! inputs answers with an error code instead of a fraction — see "Where this
//! refuses rather than guesses".
//!
//! # Control flow, in upstream's order
//!
//! The order is load-bearing and is not the order the request declares its
//! fields in, because two of the tests are reachable only through another's
//! negative arm:
//!
//! 1. `getJointModelGroup(req->group_name)` — `INVALID_GROUP_NAME` if absent
//!    (`:108`, `:219`).
//! 2. `link_name` defaults to `jmg->getLinkModelNames().back()` when the
//!    request leaves it empty (`:110-112`).
//! 3. The waypoint loop, which transforms only when `no_transform` is false
//!    (`:121-143`) — `FRAME_TRANSFORM_FAILURE` if a transform fails (`:216`).
//! 4. `req->max_step < epsilon` — `FAILURE` (`:147-152`).
//! 5. `!waypoints.empty()` — the computation (`:155`).
//! 6. `SUCCESS`, whatever fraction came out (`:212`).
//!
//! Step 4 sits *after* step 3, and step 5 *inside* step 4's `else`, so an
//! empty-waypoint request with `max_step == 0` answers `FAILURE` rather than
//! `SUCCESS`, and an empty-waypoint request naming a foreign frame answers
//! `SUCCESS` rather than `FRAME_TRANSFORM_FAILURE` — the loop that would have
//! failed never runs. Both are upstream's and both are asserted below.
//!
//! Step 6 is unconditional: a path that solved nothing still answers
//! `SUCCESS`, with `fraction == 0.0`. That is what makes the client's `-1.0`
//! and a zero fraction mean two different things.
//!
//! # Two things a bare `fraction` cannot say
//!
//! This module's private `Computed` exists because `fraction == 0.0` with an
//! empty `solution` is
//! reachable two ways that upstream answers differently in the *other*
//! response fields. When `waypoints` is empty upstream never enters the block
//! at `:155-211`, so `res->start_state` and `res->solution` stay
//! default-constructed; when the interpolator runs and solves nothing,
//! `:189` and `:200` fill both in — a start state and a one-waypoint
//! trajectory. Collapsing the two into a `fraction` plus an "is it empty"
//! test is how the second one silently acquires the first one's empty
//! `start_state`.
//!
//! Likewise [`WaypointFrame`] replaces upstream's `no_transform` (`:117-119`)
//! and `global_frame` (`:174`) pair. Those two booleans are computed from the
//! same three strings and are not complements: `no_transform == false`
//! implies the frame is non-empty and unequal to `link_name`, which forces
//! `global_frame == true`, so one of their four combinations cannot occur.
//! Three variants make it unrepresentable instead of merely unreached.
//!
//! # No TF client, which is a branch upstream has
//!
//! [`WaypointFrame::Foreign`] answers `FRAME_TRANSFORM_FAILURE`. That is not
//! a stub for an unwritten lookup: upstream reaches its own
//! `FRAME_TRANSFORM_FAILURE` through `MoveGroupCapability::performTransform`,
//! whose *first* statement is `if (!context_ ||
//! !context_->planning_scene_monitor_->getTFClient()) return false`
//! (`move_group_capability.cpp:194-198`). This node runs no TF client, so
//! that first branch is the one upstream itself would take here.
//!
//! An unmodified client does not reach it: `MoveGroupInterface` sets
//! `req->header.frame_id = getPoseReferenceFrame()`
//! (`move_group_interface.cpp:883`), and `pose_reference_frame_` is
//! initialised to `getRobotModel()->getModelFrame()`
//! (`move_group_interface.cpp:175`), so the default request names the model
//! frame and lands in [`WaypointFrame::Model`].
//!
//! # Request fields upstream accepts and does not implement
//!
//! Five request fields are never applied by upstream's capability:
//! `jump_threshold` is read into a `moveit::core::JumpThreshold` local at
//! `:180-184` that the call at `:186-188` does not take (it passes
//! `moveit::core::CartesianPrecision{}` instead), and
//! `prismatic_jump_threshold`, `revolute_jump_threshold`,
//! `cartesian_speed_limited_link` and `max_cartesian_speed` are not mentioned
//! anywhere in the file.
//!
//! Dropping `jump_threshold` is deliberate, not an oversight: upstream commit
//! `dae612696` ("New implementation for computeCartesianPath() (#2916)")
//! rewrote the call site to `CartesianPrecision{}` in the same diff that
//! *removed the `jump_threshold` parameter from the client's own public
//! signature*, and the `JumpThreshold`-taking overloads carry
//! `[[deprecated("Replace JumpThreshold with CartesianPrecision")]]`
//! (`cartesian_interpolator.hpp:250,262,284,299`). So this port applies no
//! jump threshold either — reinstating one would compute a fraction upstream
//! stopped producing on purpose.
//!
//! What this port does not reproduce is *accepting* those fields silently.
//! Each is accepted at its own documented no-op value (the `.srv` says "if
//! jump_threshold is set > 0" and "if max_cartesian_speed <= 0 the trajectory
//! is not modified") and refused above it, so a client that asks for
//! truncation is told it did not get it rather than handed a fraction that
//! quietly ignores the request. The test is placed exactly where upstream
//! reads the field, `:180`, so every request upstream answers with an error
//! code still gets that same code here. Recorded as
//! `cartesian-path-capability-accepts-jump-thresholds-it-never-applies` in
//! `doc/upstream-bugs.md`.
//!
//! # Where this refuses rather than guesses
//!
//! Upstream discards the return value of `robotStateMsgToRobotState` (`:107`)
//! and of `kset->add` (`:164`), so a `start_state` or `path_constraints` it
//! could only partly apply silently changes what it computes from. This port
//! cannot represent those messages at all rather than partly, and a silently
//! different start state is a silently different fraction, so both answer
//! `FAILURE` with the reason in `error_code.message`. A `link_name` naming no
//! link is the third: see
//! `cartesian-path-capability-throws-on-an-unknown-link-name` in
//! `doc/upstream-bugs.md`.
//!
//! The fourth is not upstream's to blame and is a divergence, not a fix. A
//! `link_name` that *is* a link but is not rigidly connected to any tip the
//! group's solver reports -- `base_link` for the `arm` group of
//! `ros/fixtures/one_joint.urdf` -- makes `moveit_kinematics::set_from_ik`
//! return `Err`, where upstream's `setFromIK` returns `false` and leaves
//! `computeCartesianPath` to report `SUCCESS` with `fraction == 0.0`. This
//! module answers `FAILURE` rather than manufacturing the `0.0`: it is the
//! interpolator, not this module, that knows whether nothing was achieved or
//! nothing was attempted, and the two are the same number on the wire. An
//! unmodified client reaches this only through `setEndEffectorLink`, which
//! validates nothing (`move_group_interface.cpp:1712-1719`).
//!
//! # Time parameterization is attempted and may not happen
//!
//! `:197-198` runs `TimeOptimalTrajectoryGeneration::computeTimeStamps` and
//! discards its `bool`, so upstream ships an untimed trajectory when the
//! robot model carries no acceleration limits. This module does the same and
//! says so on stderr; `ros/fixtures/one_joint.urdf` declares no
//! `<limit acceleration=...>`, so that is the branch every test and both live
//! legs below actually take.
//!
//! # `display_planned_path` is not published
//!
//! Upstream's `display_computed_paths_` is hardcoded `true` (`:78`) and
//! `:203-210` publishes a `moveit_msgs/DisplayTrajectory` on
//! `display_planned_path` for RViz. Nothing in Phase 9's completion condition
//! reads that topic, and `MoveGroupInterface` never subscribes to it
//! (`move_group_interface.cpp` names neither the topic nor the message type),
//! so it is left out rather than published into a graph with no reader.

use std::sync::Arc;

use moveit_collision::{CollisionRequest, ParryCollisionEnv};
use moveit_constraints::KinematicConstraintSet;
use moveit_geometry::{Isometry3, Transforms};
use moveit_kinematics::{
    CartesianInterpolator, DEFAULT_SOLVER_NAME, IkContext, MaxEefStep, NoAttachedFrames,
    SolverParams, resolve_solver,
};
use moveit_model::JointModelGroup;
use moveit_planning::StartState;
use moveit_scene::PlanningScene;
use moveit_state::RobotState;
use moveit_trajectory::robot_trajectory::RobotTrajectory;
use moveit_trajectory::time_optimal_trajectory_generation::{TotgOptions, compute_time_stamps};
use r2r::moveit_msgs::msg::MoveItErrorCodes;
use r2r::moveit_msgs::srv::GetCartesianPath;

use crate::constraints::set::ConstraintsMsg;
use crate::planning::{RobotTrajectoryMsgOut, StartStateMsg};
use crate::state::RobotStateMsgOut;

/// `move_group::CARTESIAN_PATH_SERVICE_NAME`
/// (`moveit_ros/move_group/include/moveit/move_group/capability_names.hpp:59-60`),
/// verbatim — unqualified, so it resolves against the node's namespace the
/// way `MoveGroupInterface` resolves its own client
/// (`rclcpp::names::append(namespace, CARTESIAN_PATH_SERVICE_NAME)`). A
/// leading slash here would put the server out of reach of a namespaced
/// client, the same trap `move_action` documents in `src/bin/move_group.rs`.
pub const SERVICE_NAME: &str = "compute_cartesian_path";

/// How `req->waypoints` are to be read: upstream's `no_transform` (`:117-119`)
/// and `global_frame` (`:174`) booleans, with the combination the two of them
/// cannot jointly mean removed.
///
/// Upstream computes both from `(req->header.frame_id, model_frame,
/// link_name)`, and `no_transform == false && global_frame == false` is
/// unreachable: `no_transform` is false only when `frame_id` is non-empty and
/// differs from `link_name`, which is exactly what makes `global_frame` true.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WaypointFrame {
    /// The waypoints are already in the model frame: `frame_id` is empty, or
    /// names the model frame. Upstream `no_transform == true`,
    /// `global_frame == true`.
    Model,
    /// The waypoints are relative to `link_name`'s own current pose:
    /// `frame_id` names that link. Upstream `no_transform == true`,
    /// `global_frame == false`.
    Link,
    /// `frame_id` names some third frame, which only TF can resolve.
    /// Upstream `no_transform == false`, `global_frame == true`.
    Foreign(String),
}

impl WaypointFrame {
    /// Upstream's two boolean expressions, evaluated together.
    ///
    /// [`WaypointFrame::Link`] is tested first because upstream's
    /// `global_frame` is `!sameFrame(link_name, frame_id)` alone: a request
    /// whose `frame_id` equals *both* the model frame and `link_name` (a
    /// group whose last link is the model frame) is link-relative upstream,
    /// and testing the model frame first here would silently make it global.
    pub fn resolve(frame_id: &str, model_frame: &str, link_name: &str) -> Self {
        if Transforms::same_frame(frame_id, link_name) {
            Self::Link
        } else if frame_id.is_empty() || Transforms::same_frame(frame_id, model_frame) {
            Self::Model
        } else {
            Self::Foreign(frame_id.to_string())
        }
    }

    /// Upstream's `global_frame`, as
    /// [`CartesianInterpolator::global_reference_frame`] spells it.
    pub fn global_reference_frame(&self) -> bool {
        !matches!(self, Self::Link)
    }
}

/// What `computeService` decided, before it is flattened onto the wire.
///
/// The two `SUCCESS` variants are distinguished because upstream fills
/// `res->start_state` and `res->solution` in one of them and not the other —
/// see this module's "Two things a bare `fraction` cannot say".
#[derive(Debug)]
enum Computed {
    /// One of upstream's three non-`SUCCESS` arms (`:151`, `:216`, `:219`),
    /// or one of this port's own refusals. `fraction` stays `0.0`, which is
    /// the field's default and is not read by a client that saw a non-zero
    /// `val`.
    Refused {
        /// A `moveit_msgs::msg::MoveItErrorCodes` constant.
        val: i32,
        /// Goes into `error_code.message`. Upstream logs its reasons with
        /// `RCLCPP_ERROR` and sends an empty message; a service reply is the
        /// only channel a remote client can read.
        message: String,
    },
    /// `req->waypoints` was empty and `max_step` was usable: upstream skips
    /// `:155-211` entirely and falls through to `SUCCESS` at `:212` with
    /// every response field still default-constructed.
    NothingRequested,
    /// The interpolator ran. All three fields are set even when `fraction` is
    /// `0.0`, matching `:186-200`, which are unconditional inside the block.
    ///
    /// Boxed: the two wire structs are ~450 bytes together, and every
    /// `Refused` return would otherwise carry that much stack
    /// (`clippy::large_enum_variant`).
    Path(Box<SolvedPath>),
}

/// [`Computed::Path`]'s payload.
#[derive(Debug)]
struct SolvedPath {
    fraction: f64,
    start_state: r2r::moveit_msgs::msg::RobotState,
    solution: r2r::moveit_msgs::msg::RobotTrajectory,
}

/// `MoveGroupCartesianPathService::computeService` (`:97-222`).
///
/// `snapshot` stands in for `LockedPlanningSceneRO ls(...)` (`:106`, `:162`)
/// exactly as it does in `src/bin/move_group.rs`'s other handlers: every
/// mutation lands on a [`PlanningScene::diff`] child and the monitored scene
/// is untouched.
///
/// Upstream's `updateFrameTransforms()` (`:103`) has no counterpart, for the
/// same reason [`WaypointFrame::Foreign`] does not: it refreshes the scene's
/// TF-sourced transforms from a TF client this node does not run.
pub fn handle<'m>(
    snapshot: &Arc<PlanningScene<'m>>,
    request: GetCartesianPath::Request,
) -> GetCartesianPath::Response {
    match compute(snapshot, request) {
        Computed::Refused { val, message } => GetCartesianPath::Response {
            error_code: MoveItErrorCodes {
                val,
                message,
                source: SERVICE_NAME.to_string(),
            },
            ..Default::default()
        },
        Computed::NothingRequested => GetCartesianPath::Response {
            error_code: success(),
            ..Default::default()
        },
        Computed::Path(path) => GetCartesianPath::Response {
            start_state: path.start_state,
            solution: path.solution,
            fraction: path.fraction,
            error_code: success(),
        },
    }
}

/// `MoveItErrorCodes::SUCCESS`, stamped with the endpoint that built it.
///
/// `source` is this port's own use of an upstream-empty field, and
/// `src/bin/move_group.rs`'s `plan` explains why it is stamped on the success
/// arm too: it is the only thing separating a reply that crossed DDS from one
/// `MoveGroupInterface` synthesised locally when no server answered.
fn success() -> MoveItErrorCodes {
    MoveItErrorCodes {
        val: MoveItErrorCodes::SUCCESS as i32,
        message: String::new(),
        source: SERVICE_NAME.to_string(),
    }
}

/// A `Computed::Refused` with `MoveItErrorCodes::FAILURE`, upstream's own
/// code for "this request is not usable" here (`:151`).
fn failure(message: impl Into<String>) -> Computed {
    Computed::Refused {
        val: MoveItErrorCodes::FAILURE as i32,
        message: message.into(),
    }
}

fn compute<'m>(snapshot: &Arc<PlanningScene<'m>>, request: GetCartesianPath::Request) -> Computed {
    let model = snapshot.robot_model();

    // `:108`, and it comes first: an unknown group is `INVALID_GROUP_NAME`
    // whatever else the request got wrong.
    let Ok(group) = model.joint_model_group(&request.group_name) else {
        return Computed::Refused {
            val: MoveItErrorCodes::INVALID_GROUP_NAME as i32,
            message: format!("no group named '{}'", request.group_name),
        };
    };

    // `:110-112`. Upstream's guard is on the *group's* link list being
    // non-empty, not on `link_name` resolving to anything — see
    // `cartesian-path-capability-throws-on-an-unknown-link-name`.
    let link_name = if request.link_name.is_empty() {
        match group.link_names().last() {
            Some(last) => last.clone(),
            // Upstream leaves `link_name` empty here and reaches
            // `getLinkModel("")` at `:187`, which is the throwing path.
            None => {
                return failure(format!(
                    "group '{}' has no links and the request named no link_name",
                    request.group_name
                ));
            }
        }
    } else {
        request.link_name.clone()
    };

    // `:117-119` and `:174`, as one value.
    let frame = WaypointFrame::resolve(&request.header.frame_id, model.model_frame(), &link_name);

    // `:121-143`. The loop runs per waypoint, so an empty `waypoints` never
    // reaches the transform at all -- which is why the `Foreign` arm is
    // inside the emptiness test and not before it.
    let waypoints: Vec<Isometry3> = match &frame {
        WaypointFrame::Model | WaypointFrame::Link => {
            request.waypoints.iter().map(pose_to_isometry).collect()
        }
        WaypointFrame::Foreign(frame_id) => {
            if request.waypoints.is_empty() {
                Vec::new()
            } else {
                return Computed::Refused {
                    val: MoveItErrorCodes::FRAME_TRANSFORM_FAILURE as i32,
                    message: format!(
                        "waypoints are in frame '{frame_id}', which is neither the model \
                         frame '{}' nor '{link_name}', and this node runs no TF client",
                        model.model_frame()
                    ),
                };
            }
        }
    };

    // `:147-152`, after the loop and before the emptiness test.
    if request.max_step < f64::EPSILON {
        return failure(format!(
            "max_step must be > 0, got {} (moveit_msgs/GetCartesianPath: \
             \"This must always be specified and > 0\")",
            request.max_step
        ));
    }

    // `:155`.
    if waypoints.is_empty() {
        return Computed::NothingRequested;
    }

    // `:180-184`, the position upstream reads `jump_threshold` at. See this
    // module's "Request fields upstream accepts and does not implement".
    for (field, value) in [
        ("jump_threshold", request.jump_threshold),
        ("prismatic_jump_threshold", request.prismatic_jump_threshold),
        ("revolute_jump_threshold", request.revolute_jump_threshold),
        ("max_cartesian_speed", request.max_cartesian_speed),
    ] {
        if value > 0.0 {
            return failure(format!(
                "{field} = {value} asks for a filter this service does not apply; \
                 upstream stopped applying it in moveit2 dae612696 and answers as if \
                 it were 0.0, which would make the fraction reported here silently \
                 unfiltered"
            ));
        }
    }

    // `:105-107`, deferred to here so that `INVALID_GROUP_NAME` still wins
    // for a request that gets both wrong. Upstream overlays the message onto
    // the monitored scene's current state and discards the return value;
    // `StartState` is this port's model of that overlay (see its own doc).
    let mut start_state = snapshot.current_state().clone();
    let start = match StartState::try_from(StartStateMsg(request.start_state.clone())) {
        Ok(start) => start,
        Err(e) => return failure(format!("start_state is not representable: {e}")),
    };
    if let Err(e) = start.apply_to(&mut start_state) {
        return failure(format!("start_state does not apply to this model: {e}"));
    }

    // `:160-173`. The two halves are independently optional: `kset` is built
    // whenever the callback is, then nulled if it came out empty (`:168`), so
    // `avoid_collisions` alone gives a collision-only callback and a
    // non-empty `path_constraints` alone gives a constraints-only one.
    let path_constraints_empty = constraints_are_empty(&request.path_constraints);
    let constraints = if request.avoid_collisions || !path_constraints_empty {
        match KinematicConstraintSet::try_from(ConstraintsMsg {
            model,
            // Cloned rather than moved out: `finish` below still reads
            // `request`, and a `request` with one field emptied would be a
            // value whose meaning depends on how far execution got.
            msg: request.path_constraints.clone(),
        }) {
            Ok(set) if set.is_empty() => None,
            Ok(set) => Some(set),
            Err(e) => return failure(format!("path_constraints is not representable: {e}")),
        }
    } else {
        None
    };

    // Upstream's `group->getSolverInstance()`, which comes from the group's
    // configured kinematics plugin. This port resolves it by name from
    // `KINEMATICS_SOLVERS` instead (PORTING-PLAN.md §177: the slice's order
    // is not a contract, so the name is the selection rule).
    //
    // A group with no constructible solver is upstream's null-instance case:
    // `setFromIK` returns false on the first waypoint, the path stops at the
    // start state, and the answer is `SUCCESS` with `fraction == 0.0` -- not
    // an error. Reproduced here rather than refused, because upstream's
    // answer is a fraction and this port can produce the same one.
    let solver = resolve_solver(
        model,
        &request.group_name,
        DEFAULT_SOLVER_NAME,
        &SolverParams::default(),
    );
    let mut solver = match solver {
        Ok(solver) => solver,
        Err(e) => {
            eprintln!(
                "{SERVICE_NAME}: group '{}' has no constructible '{DEFAULT_SOLVER_NAME}' \
                 solver ({e}); answering fraction 0.0, as upstream does for a group with \
                 no kinematics plugin",
                request.group_name
            );
            return finish(model, &request, start_state, Vec::new(), 0.0);
        }
    };

    // Upstream's `constraint_fn` (`:165-172`), whose body is the
    // anonymous-namespace `isStateValid` at `:59-67`. Its first two lines --
    // `state->setJointGroupPositions(group, ik_solution); state->update();` --
    // have no counterpart here: this port's `set_from_ik` applies the
    // candidate to the state *before* calling the hook and passes the values
    // back out of the state, so the hook is handed a state that already holds
    // them (`set_from_ik.rs`'s deviation 2).
    //
    // The collision half needs a scene whose *current state* is the
    // candidate, because `PlanningScene::is_state_colliding` reads the
    // scene's own state where upstream's takes one by argument. That is a
    // second `diff` child, disjoint from `start_state`, so writing the
    // candidate into it cannot perturb the path's own start.
    let mut check_scene = snapshot.diff();
    let env = ParryCollisionEnv::new(check_scene.world().clone(), Default::default());
    let collision_request = CollisionRequest {
        group_name: Some(request.group_name.clone()),
        ..Default::default()
    };
    let avoid_collisions = request.avoid_collisions;
    let mut validity = |state: &mut RobotState<'m>, _: &JointModelGroup, _: &[f64]| -> bool {
        check_scene
            .current_state_mut()
            .set_variable_positions(state.positions());
        if avoid_collisions && check_scene.is_state_colliding(&env, &collision_request) {
            return false;
        }
        match &constraints {
            Some(set) => {
                set.decide(&check_scene.current_state_mut().update())
                    .satisfied
            }
            None => true,
        }
    };

    let interpolator = CartesianInterpolator {
        group_name: &request.group_name,
        link_name: &link_name,
        link_offset: Isometry3::identity(),
        // `moveit::core::MaxEEFStep(req->max_step)` (`:188`), the single-argument
        // constructor, whose `rotation` is `3.5 * step_size`.
        max_step: MaxEefStep::from_step_size(request.max_step),
        // `moveit::core::CartesianPrecision{}` (`:188`), all defaults.
        precision: Default::default(),
        global_reference_frame: frame.global_reference_frame(),
    };
    let mut ik = IkContext {
        attached: &NoAttachedFrames,
        consistency_limits: None,
        validity: Some(&mut validity),
    };
    let (traj, fraction) =
        match interpolator.through_waypoints(&start_state, solver.as_mut(), &waypoints, &mut ik) {
            Ok(out) => out,
            // Upstream cannot reach this: `getLinkModel(link_name)` returning
            // null makes `getGlobalLinkTransform` *throw* out of the service
            // callback instead (`robot_state.hpp:1252-1257`).
            Err(e) => {
                return failure(format!(
                    "computing the Cartesian path for link '{link_name}' failed: {e}"
                ));
            }
        };

    finish(model, &request, start_state, traj, fraction.value())
}

/// `:189-200`: encode the start state, wrap the waypoints in a
/// `RobotTrajectory`, time-parameterise it, and encode that.
///
/// Split out because the no-solver arm above reaches it with an empty
/// trajectory and has to produce the same response *shape* as a path that
/// solved nothing — that shape being the whole reason [`Computed::Path`] and
/// [`Computed::NothingRequested`] are separate variants.
fn finish<'m>(
    model: &'m moveit_model::RobotModel,
    request: &GetCartesianPath::Request,
    start_state: RobotState<'m>,
    traj: Vec<RobotState<'m>>,
    fraction: f64,
) -> Computed {
    // `:189`. `start_state` is the *request's* start state, not where the
    // path stopped: the overload at `:186` takes `const RobotState*`, so
    // nothing upstream writes back into it either.
    let start_state = match RobotStateMsgOut::try_from(start_state) {
        Ok(out) => out.0,
        Err(e) => return failure(format!("start_state does not serialise: {e}")),
    };

    // `:191-193`.
    let mut rt = RobotTrajectory::new(model);
    if let Err(e) = rt.set_group_name(&request.group_name) {
        return failure(format!("group '{}': {e}", request.group_name));
    }
    for state in traj {
        if let Err(e) = rt.add_suffix_way_point(state, 0.0) {
            return failure(format!("building the solution trajectory: {e}"));
        }
    }

    // `:197-198`. Upstream discards `computeTimeStamps`' `bool` and sends the
    // trajectory either way, so a failure here is reported to the operator
    // and the untimed trajectory still goes out: refusing would answer an
    // error where upstream answers a fraction.
    //
    // Built by assignment rather than by struct-update syntax because
    // `TotgOptions::resample_dt` is `pub(crate)` in `moveit-trajectory` (it
    // has a validating setter there), which makes `..Default::default()`
    // E0451 from outside that crate.
    let mut options = TotgOptions::default();
    options.max_velocity_scaling_factor = request.max_velocity_scaling_factor;
    options.max_acceleration_scaling_factor = request.max_acceleration_scaling_factor;
    if let Err(e) = compute_time_stamps(&mut rt, &options) {
        eprintln!(
            "{SERVICE_NAME}: time parameterization failed, sending the path untimed \
             (upstream discards the same failure): {e}"
        );
    }

    // `:200`.
    match RobotTrajectoryMsgOut::try_from(rt) {
        Ok(out) => Computed::Path(Box::new(SolvedPath {
            fraction,
            start_state,
            solution: out.0,
        })),
        Err(e) => failure(format!("the solution trajectory does not serialise: {e}")),
    }
}

/// `tf2::fromMsg(req->waypoints[i], waypoints[i])` (`:125`, `:134`).
///
/// `UnitQuaternion::new_normalize` rather than `new_unchecked`: `tf2`'s own
/// `fromMsg` builds an `Eigen::Quaterniond` from the four wire components and
/// Eigen normalises lazily on use, so a client that sent an unnormalised
/// quaternion gets the normalised rotation upstream would have used, not a
/// non-isometry that `ASSERT_ISOMETRY` would reject in a debug build.
fn pose_to_isometry(pose: &r2r::geometry_msgs::msg::Pose) -> Isometry3 {
    Isometry3::from_parts(
        nalgebra::Translation3::new(pose.position.x, pose.position.y, pose.position.z),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )),
    )
}

/// Upstream `moveit::core::isEmpty(const moveit_msgs::msg::Constraints&)`
/// (`utils/src/message_checks.cpp:70-75`), the `:160` half of the test that
/// decides whether the validity callback is built at all.
fn constraints_are_empty(constraints: &r2r::moveit_msgs::msg::Constraints) -> bool {
    constraints.position_constraints.is_empty()
        && constraints.orientation_constraints.is_empty()
        && constraints.visibility_constraints.is_empty()
        && constraints.joint_constraints.is_empty()
}

#[cfg(test)]
mod tests {
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use r2r::moveit_msgs::msg::{Constraints, JointConstraint, RobotState as RobotStateMsgWire};

    use super::*;

    /// `ros/fixtures/one_joint.{urdf,srdf}`, inline — the robot every live leg
    /// of `ros/verify-ros-interop.sh` loads, so a fraction computed here is a
    /// fraction for the robot the node is actually serving.
    ///
    /// Its shape is what makes a Cartesian path testable at all with one
    /// degree of freedom: `j1` has no `<origin>`, so `tip`'s frame is
    /// `base_link`'s rotated about z by `j1` and its translation is
    /// identically zero. A straight-line Cartesian interpolation between two
    /// such poses is a pure slerp about z, and *every* intermediate pose on it
    /// is exactly reachable — a target that is reachable therefore yields
    /// fraction `1.0` rather than the `1/steps` a general 1-DOF arm would give
    /// for an off-axis chord.
    const ONE_JOINT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base_link"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
"#;

    const ONE_JOINT_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <group name="arm">
    <chain base_link="base_link" tip_link="tip"/>
  </group>
</robot>
"#;

    /// `arm`'s `link_names()` here is `["tip", "tool"]`, not `["tool"]`: a
    /// group's link list is the *child* links of its joints, so a chain needs
    /// two joints before its front and its back are different elements. The
    /// one-joint fixture's `arm` is `["tip"]` alone, which cannot tell
    /// `link_names().last()` from `.first()`.
    ///
    /// `j1` slides instead of turning, and `j2`'s offset is along the same
    /// axis, so a straight-line Cartesian path is exactly reachable for
    /// *both* links and each answers `1.0` — the two readings of `:110-112`
    /// then differ in the joint value they arrive at, `0.2` against `0.5`,
    /// rather than in whether they solved at all.
    const TOOL_OFFSET_URDF: &str = r#"<?xml version="1.0"?>
<robot name="tool_offset">
  <link name="base_link"/>
  <link name="tip"/>
  <link name="tool"/>
  <joint name="j1" type="prismatic">
    <parent link="base_link"/>
    <child link="tip"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="tip"/>
    <child link="tool"/>
    <origin xyz="0.3 0 0"/>
  </joint>
</robot>
"#;

    const TOOL_OFFSET_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="tool_offset">
  <group name="arm">
    <chain base_link="base_link" tip_link="tool"/>
  </group>
</robot>
"#;

    fn leak(urdf_str: &'static str, srdf_str: &str) -> (&'static RobotModel, &'static SrdfModel) {
        let urdf = urdf_rs::read_from_string(urdf_str).expect("inline URDF must parse");
        let srdf = SrdfModel::parse_str(srdf_str).expect("inline SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_str, &srdf, &MeshSearchPaths::none())
                .expect("valid inline urdf");
        (Box::leak(Box::new(model)), Box::leak(Box::new(srdf)))
    }

    fn scene() -> Arc<PlanningScene<'static>> {
        let (model, srdf) = leak(ONE_JOINT_URDF, ONE_JOINT_SRDF);
        Arc::new(PlanningScene::new(model, srdf))
    }

    fn tool_offset_scene() -> Arc<PlanningScene<'static>> {
        let (model, srdf) = leak(TOOL_OFFSET_URDF, TOOL_OFFSET_SRDF);
        Arc::new(PlanningScene::new(model, srdf))
    }

    /// A `Pose` that is `tip`'s pose at `j1 == angle`: a rotation of `angle`
    /// about z with zero translation.
    fn tip_at(angle: f64) -> r2r::geometry_msgs::msg::Pose {
        let (sin, cos) = (angle / 2.0).sin_cos();
        r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: r2r::geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: sin,
                w: cos,
            },
        }
    }

    /// `MoveGroupInterfaceImpl::computeCartesianPath`'s request, field for
    /// field (`move_group_interface.cpp:878-889`), for the fixture's group:
    /// the ten fields that client assigns, with `header.frame_id` at its
    /// `getPoseReferenceFrame()` default (the model frame) and the five it
    /// does not assign left at zero.
    fn client_request(waypoints: Vec<r2r::geometry_msgs::msg::Pose>) -> GetCartesianPath::Request {
        GetCartesianPath::Request {
            header: r2r::std_msgs::msg::Header {
                frame_id: "base_link".to_string(),
                ..Default::default()
            },
            group_name: "arm".to_string(),
            waypoints,
            max_step: 0.1,
            avoid_collisions: true,
            ..Default::default()
        }
    }

    #[track_caller]
    fn assert_val(response: &GetCartesianPath::Response, expected: i32, why: &str) {
        assert_eq!(
            response.error_code.val, expected,
            "{why} (message: {:?})",
            response.error_code.message
        );
    }

    /// The success criterion for this endpoint: an unmodified client's own
    /// request shape comes back `SUCCESS` with a fraction of `1.0` and a
    /// trajectory that ends where it asked.
    ///
    /// `fraction` is asserted exactly, not as "> 0": it is the return value of
    /// `MoveGroupInterface::computeCartesianPath`, so a path that solved 60%
    /// of the way and a path that solved all of it are two different answers
    /// to the client and only one of them is this one.
    #[test]
    fn a_reachable_waypoint_answers_success_with_fraction_one() {
        let scene = scene();
        let response = handle(&scene, client_request(vec![tip_at(0.5)]));

        assert_val(
            &response,
            MoveItErrorCodes::SUCCESS as i32,
            "reachable path",
        );
        assert_eq!(response.fraction, 1.0);
        assert_eq!(response.error_code.source, SERVICE_NAME);

        let points = &response.solution.joint_trajectory.points;
        assert!(
            points.len() >= 2,
            "a path from j1 = 0 to j1 = 0.5 has a start and an end, got {} point(s)",
            points.len()
        );
        let last = points.last().expect("checked non-empty above");
        assert!(
            (last.positions[0] - 0.5).abs() < 1e-6,
            "the last waypoint must be at the requested pose, got j1 = {}",
            last.positions[0]
        );
    }

    /// The other side of the fraction: a target past `j1`'s `[-1, 1]` limit
    /// is followed as far as it goes and reported as a proper fraction, not
    /// as an error and not as `1.0`.
    ///
    /// Bounds are the discriminator, not IK convergence: a solver that
    /// ignored `<limit>` would answer `1.0` here.
    #[test]
    fn a_waypoint_past_the_joint_limit_answers_success_with_a_partial_fraction() {
        let scene = scene();
        let response = handle(&scene, client_request(vec![tip_at(1.5)]));

        assert_val(&response, MoveItErrorCodes::SUCCESS as i32, "partial path");
        assert!(
            response.fraction > 0.0 && response.fraction < 1.0,
            "a target beyond j1's upper limit of 1.0 rad is partly followable, got fraction {}",
            response.fraction
        );
        let last = response
            .solution
            .joint_trajectory
            .points
            .last()
            .expect("a partly solved path has waypoints");
        assert!(
            last.positions[0] <= 1.0 + 1e-9,
            "no waypoint may leave j1's limit, got {}",
            last.positions[0]
        );
    }

    /// `:108`/`:219`. First in upstream's order, so it wins over every other
    /// defect in the same request — asserted with a request that is *also*
    /// unusable three other ways.
    #[test]
    fn an_unknown_group_name_answers_invalid_group_name_before_anything_else() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            group_name: "not_a_group".to_string(),
            max_step: 0.0,
            jump_threshold: 2.0,
            header: r2r::std_msgs::msg::Header {
                frame_id: "some_other_frame".to_string(),
                ..Default::default()
            },
            waypoints: vec![tip_at(0.5)],
            ..Default::default()
        };
        assert_val(
            &handle(&scene, request),
            MoveItErrorCodes::INVALID_GROUP_NAME as i32,
            "an unknown group must be reported as such, not as the other three faults",
        );
    }

    /// `:110-112`: an empty `link_name` is the group's *last* link, not its
    /// first.
    ///
    /// This runs on [`TOOL_OFFSET_URDF`] because the one-joint fixture cannot
    /// state the boundary: its `arm` has a single link, so `.first()` and
    /// `.last()` name the same one and either reading passes. Here both
    /// readings also *solve* — reaching `x = 0.5` with `tool` puts `j1` at
    /// `0.2` and with `tip` at `0.5` — so what separates them is the joint
    /// value arrived at, which a fraction of `1.0` alone would hide.
    #[test]
    fn an_empty_link_name_defaults_to_the_groups_last_link() {
        let scene = tool_offset_scene();
        let at_x = |x: f64| r2r::geometry_msgs::msg::Pose {
            position: r2r::geometry_msgs::msg::Point { x, y: 0.0, z: 0.0 },
            orientation: r2r::geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let answer = |link: &str| -> (i32, f64, f64) {
            let response = handle(
                &scene,
                GetCartesianPath::Request {
                    link_name: link.to_string(),
                    ..client_request(vec![at_x(0.5)])
                },
            );
            let end = response
                .solution
                .joint_trajectory
                .points
                .last()
                .map_or(f64::NAN, |point| point.positions[0]);
            (response.error_code.val, response.fraction, end)
        };

        let defaulted = answer("");
        assert_eq!(
            defaulted,
            answer("tool"),
            "an empty link_name must answer exactly what an explicit `tool` answers"
        );
        assert_eq!(defaulted.0, MoveItErrorCodes::SUCCESS as i32);
        assert_eq!(defaulted.1, 1.0);
        assert!(
            (defaulted.2 - 0.2).abs() < 1e-6,
            "the default must move `tool` to x = 0.5, which is j1 = 0.2, got j1 = {}",
            defaulted.2
        );

        let front = answer("tip");
        assert!(
            (front.2 - 0.5).abs() < 1e-6,
            "the front of the list must reach a different joint value, or this test \
             cannot tell the two readings apart; got j1 = {}",
            front.2
        );
    }

    /// The fourth refusal in this module's "Where this refuses rather than
    /// guesses": `base_link` is a link of `arm` that no solver tip is rigidly
    /// connected to, where upstream's `setFromIK` answers `false` and this
    /// port's `set_from_ik` answers `Err`.
    ///
    /// Upstream would turn that into `SUCCESS` with a fraction of `0.0`. The
    /// discriminator is which of the two a client sees, so the error code and
    /// the named link are both asserted -- a bare "not 1.0" would pass under
    /// either.
    #[test]
    fn a_group_link_no_solver_tip_reaches_is_refused_rather_than_scored_zero() {
        let scene = scene();
        let response = handle(
            &scene,
            GetCartesianPath::Request {
                link_name: "base_link".to_string(),
                ..client_request(vec![tip_at(0.5)])
            },
        );

        assert_val(
            &response,
            MoveItErrorCodes::FAILURE as i32,
            "base_link path",
        );
        assert!(
            response.error_code.message.contains("base_link"),
            "the refusal must name the link it could not solve for, got {:?}",
            response.error_code.message
        );
    }

    /// The three [`WaypointFrame`] variants, at the boundaries of upstream's
    /// two boolean expressions (`:117-119`, `:174`).
    ///
    /// The last case is the one the variant order exists for: a `frame_id`
    /// equal to *both* the model frame and `link_name` is `Link` upstream,
    /// because `global_frame` is `!sameFrame(link_name, frame_id)` and never
    /// consults the model frame.
    #[test]
    fn waypoint_frame_resolves_upstreams_two_booleans_together() {
        assert_eq!(
            WaypointFrame::resolve("", "base_link", "tip"),
            WaypointFrame::Model
        );
        assert_eq!(
            WaypointFrame::resolve("base_link", "base_link", "tip"),
            WaypointFrame::Model
        );
        assert_eq!(
            WaypointFrame::resolve("tip", "base_link", "tip"),
            WaypointFrame::Link
        );
        assert_eq!(
            WaypointFrame::resolve("world", "base_link", "tip"),
            WaypointFrame::Foreign("world".to_string())
        );
        assert_eq!(
            WaypointFrame::resolve("base_link", "base_link", "base_link"),
            WaypointFrame::Link,
            "a frame_id equal to both the model frame and link_name is link-relative upstream"
        );

        assert!(WaypointFrame::Model.global_reference_frame());
        assert!(!WaypointFrame::Link.global_reference_frame());
        assert!(WaypointFrame::Foreign("world".to_string()).global_reference_frame());
    }

    /// [`WaypointFrame::Link`] is not a relabelling of
    /// [`WaypointFrame::Model`]: the same waypoint means a different target
    /// under each, and only a start state away from the identity can tell
    /// them apart.
    ///
    /// Starting at `j1 == 0.3`, a waypoint of `rotZ(0.5)` is `j1 == 0.5`
    /// read as a model-frame pose and `j1 == 0.8` read as a pose relative to
    /// `tip`'s own current frame.
    #[test]
    fn a_link_frame_waypoint_is_relative_to_the_links_current_pose() {
        let scene = scene();
        let start = RobotStateMsgWire {
            joint_state: r2r::sensor_msgs::msg::JointState {
                name: vec!["j1".to_string()],
                position: vec![0.3],
                ..Default::default()
            },
            is_diff: true,
            ..Default::default()
        };

        let model_frame = GetCartesianPath::Request {
            start_state: start.clone(),
            ..client_request(vec![tip_at(0.5)])
        };
        let model_frame = handle(&scene, model_frame);
        assert_eq!(model_frame.fraction, 1.0);
        let reached = model_frame
            .solution
            .joint_trajectory
            .points
            .last()
            .expect("a solved path has waypoints")
            .positions[0];
        assert!(
            (reached - 0.5).abs() < 1e-6,
            "a model-frame waypoint is an absolute pose, expected j1 = 0.5, got {reached}"
        );

        let link_frame = GetCartesianPath::Request {
            start_state: start,
            header: r2r::std_msgs::msg::Header {
                frame_id: "tip".to_string(),
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        let link_frame = handle(&scene, link_frame);
        assert_eq!(link_frame.fraction, 1.0);
        let reached = link_frame
            .solution
            .joint_trajectory
            .points
            .last()
            .expect("a solved path has waypoints")
            .positions[0];
        assert!(
            (reached - 0.8).abs() < 1e-6,
            "a link-frame waypoint is relative to tip's current pose, expected j1 = 0.8, \
             got {reached}"
        );
    }

    /// `:132`/`:216`, through `performTransform`'s first statement
    /// (`move_group_capability.cpp:194-198`): with no TF client a foreign
    /// frame is `FRAME_TRANSFORM_FAILURE`.
    #[test]
    fn a_foreign_frame_with_waypoints_answers_frame_transform_failure() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            header: r2r::std_msgs::msg::Header {
                frame_id: "world".to_string(),
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        assert_val(
            &handle(&scene, request),
            MoveItErrorCodes::FRAME_TRANSFORM_FAILURE as i32,
            "a frame this node cannot resolve must not be silently treated as the model frame",
        );
    }

    /// The same foreign frame with *no* waypoints answers `SUCCESS`, because
    /// upstream's transform lives inside the per-waypoint loop (`:121-143`)
    /// and a zero-iteration loop leaves `ok == true`.
    ///
    /// The pair with the test above is the whole point: they differ only in
    /// `waypoints`, so a port that hoisted the frame check out of the loop
    /// passes the other one and fails this one.
    #[test]
    fn a_foreign_frame_with_no_waypoints_answers_success() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            header: r2r::std_msgs::msg::Header {
                frame_id: "world".to_string(),
                ..Default::default()
            },
            ..client_request(Vec::new())
        };
        assert_val(
            &handle(&scene, request),
            MoveItErrorCodes::SUCCESS as i32,
            "the transform upstream would have failed on never runs for zero waypoints",
        );
    }

    /// `:147-152`, and its position: `max_step` is tested *before*
    /// `!waypoints.empty()` (`:155`), so an empty request with an unusable
    /// `max_step` is `FAILURE` and not the `SUCCESS` the emptiness arm gives.
    #[test]
    fn a_zero_max_step_is_rejected_even_with_no_waypoints() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            max_step: 0.0,
            ..client_request(Vec::new())
        };
        assert_val(
            &handle(&scene, request),
            MoveItErrorCodes::FAILURE as i32,
            "max_step is checked ahead of the emptiness test",
        );
    }

    /// [`Computed::NothingRequested`]: no waypoints and a usable `max_step`
    /// is `SUCCESS` with every other response field default-constructed,
    /// because upstream never enters `:155-211`.
    ///
    /// The empty `start_state.name` is the assertion that matters — it is
    /// what separates this from the zero-fraction path below, which a
    /// `fraction`-only check cannot do.
    #[test]
    fn no_waypoints_answers_success_with_an_untouched_response() {
        let scene = scene();
        let response = handle(&scene, client_request(Vec::new()));

        assert_val(&response, MoveItErrorCodes::SUCCESS as i32, "empty request");
        assert_eq!(response.fraction, 0.0);
        assert!(
            response.start_state.joint_state.name.is_empty(),
            "upstream never reaches `:189` for an empty waypoint list"
        );
        assert!(response.solution.joint_trajectory.points.is_empty());
    }

    /// The other reading of `fraction == 0.0`: the interpolator ran and
    /// solved nothing. `:189` and `:200` are unconditional inside the block,
    /// so `start_state` and `solution` are filled here where the test above
    /// leaves them empty.
    ///
    /// The unsolvable target is a *translation*: `j1` has no `<origin>`, so
    /// `tip`'s translation is identically zero and no interpolated pose on
    /// the way to `(1, 0, 0)` is reachable -- the first waypoint fails and
    /// the walk stops there.
    #[test]
    fn a_path_that_solves_nothing_still_reports_its_start_state() {
        let scene = scene();
        let mut away = tip_at(0.0);
        away.position.x = 1.0;
        let response = handle(&scene, client_request(vec![away]));

        assert_val(&response, MoveItErrorCodes::SUCCESS as i32, "0% path");
        assert_eq!(response.fraction, 0.0);
        assert_eq!(
            response.start_state.joint_state.name,
            vec!["j1".to_string()],
            "a path that ran must report the start state it ran from"
        );
        assert_eq!(
            response.solution.joint_trajectory.points.len(),
            1,
            "upstream's trajectory keeps the start state even when nothing solved"
        );
    }

    /// `:105-107`: the request's `start_state` is overlaid on the scene's
    /// current state and is where the path starts from. A port that ignored
    /// it would answer the same trajectory whatever the client sent.
    #[test]
    fn the_requests_start_state_is_where_the_path_starts() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            start_state: RobotStateMsgWire {
                joint_state: r2r::sensor_msgs::msg::JointState {
                    name: vec!["j1".to_string()],
                    position: vec![0.4],
                    ..Default::default()
                },
                is_diff: true,
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        let response = handle(&scene, request);

        assert_val(
            &response,
            MoveItErrorCodes::SUCCESS as i32,
            "overlaid start",
        );
        assert_eq!(response.start_state.joint_state.position, vec![0.4]);
        assert!(
            (response.solution.joint_trajectory.points[0].positions[0] - 0.4).abs() < 1e-9,
            "the trajectory's first point is the start state the request named, got {}",
            response.solution.joint_trajectory.points[0].positions[0]
        );
    }

    /// A `start_state` this port cannot represent is refused rather than
    /// partly applied. Upstream discards `robotStateMsgToRobotState`'s return
    /// value (`:107`), so it would compute a fraction from a start state that
    /// is not the one the client sent.
    #[test]
    fn an_unrepresentable_start_state_is_refused() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            start_state: RobotStateMsgWire {
                multi_dof_joint_state: r2r::sensor_msgs::msg::MultiDOFJointState {
                    joint_names: vec!["virtual_joint".to_string()],
                    ..Default::default()
                },
                is_diff: true,
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        let response = handle(&scene, request);
        assert_val(
            &response,
            MoveItErrorCodes::FAILURE as i32,
            "bad start_state",
        );
        assert!(
            response.error_code.message.contains("start_state"),
            "the refusal must name the field it is about, got {:?}",
            response.error_code.message
        );
    }

    /// The `path_constraints` half of the same rule (`:164`, whose `kset->add`
    /// return value upstream also discards).
    #[test]
    fn an_unrepresentable_path_constraint_is_refused() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            path_constraints: Constraints {
                joint_constraints: vec![JointConstraint {
                    joint_name: "not_a_joint".to_string(),
                    position: 0.0,
                    tolerance_above: 0.1,
                    tolerance_below: 0.1,
                    weight: 1.0,
                }],
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        let response = handle(&scene, request);
        assert_val(
            &response,
            MoveItErrorCodes::FAILURE as i32,
            "bad constraint",
        );
        assert!(
            response.error_code.message.contains("path_constraints"),
            "the refusal must name the field it is about, got {:?}",
            response.error_code.message
        );
    }

    /// `:187` reaches `getLinkModel(link_name)` with no null check and
    /// `getGlobalLinkTransform` throws on the result
    /// (`robot_state.hpp:1252-1257`), taking the node's executor with it.
    /// This answers instead.
    #[test]
    fn an_unknown_link_name_is_refused_rather_than_crashing() {
        let scene = scene();
        let request = GetCartesianPath::Request {
            link_name: "no_such_link".to_string(),
            ..client_request(vec![tip_at(0.5)])
        };
        let response = handle(&scene, request);
        assert_val(&response, MoveItErrorCodes::FAILURE as i32, "unknown link");
        assert!(
            response.error_code.message.contains("no_such_link"),
            "the refusal must name the link it could not find, got {:?}",
            response.error_code.message
        );
    }

    /// The four request fields upstream accepts and never applies. Each is
    /// refused above its own documented no-op value and accepted at it, so a
    /// client asking for a filter is told it did not get one.
    ///
    /// One test over the four because they are one guard with four entries; a
    /// dropped entry fails here with the field named in the message.
    #[test]
    fn a_filter_this_service_does_not_apply_is_refused_rather_than_ignored() {
        let scene = scene();
        /// Writes one of the four fields, so the table below names them
        /// rather than repeating the whole test four times.
        type SetField = fn(&mut GetCartesianPath::Request, f64);

        let fields: [(&str, SetField); 4] = [
            ("jump_threshold", |r, v| r.jump_threshold = v),
            ("prismatic_jump_threshold", |r, v| {
                r.prismatic_jump_threshold = v
            }),
            ("revolute_jump_threshold", |r, v| {
                r.revolute_jump_threshold = v
            }),
            ("max_cartesian_speed", |r, v| r.max_cartesian_speed = v),
        ];
        for (name, set) in fields {
            let mut request = client_request(vec![tip_at(0.5)]);
            set(&mut request, 2.0);
            let response = handle(&scene, request);
            assert_val(
                &response,
                MoveItErrorCodes::FAILURE as i32,
                &format!("{name} > 0 must be refused"),
            );
            assert!(
                response.error_code.message.contains(name),
                "the refusal must name {name}, got {:?}",
                response.error_code.message
            );

            let mut request = client_request(vec![tip_at(0.5)]);
            set(&mut request, 0.0);
            assert_val(
                &handle(&scene, request),
                MoveItErrorCodes::SUCCESS as i32,
                &format!("{name} at its documented no-op value must be accepted"),
            );
        }
    }

    /// `:160-172`: the validity callback's two halves are independently
    /// optional, and this is the half a naive `if req->avoid_collisions` gate
    /// drops — a `path_constraints` with `avoid_collisions == false` still
    /// gates every IK solution.
    ///
    /// The constraint window stops the path at `j1 <= 0.25`, so the fraction
    /// falls below the unconstrained `1.0` for the same request.
    #[test]
    fn a_path_constraint_gates_ik_even_with_avoid_collisions_off() {
        let scene = scene();
        let constrained = GetCartesianPath::Request {
            avoid_collisions: false,
            path_constraints: Constraints {
                joint_constraints: vec![JointConstraint {
                    joint_name: "j1".to_string(),
                    position: 0.0,
                    tolerance_above: 0.25,
                    tolerance_below: 0.25,
                    weight: 1.0,
                }],
                ..Default::default()
            },
            ..client_request(vec![tip_at(0.5)])
        };
        let constrained = handle(&scene, constrained);
        assert_val(
            &constrained,
            MoveItErrorCodes::SUCCESS as i32,
            "constrained",
        );
        assert!(
            constrained.fraction < 1.0,
            "a joint constraint of j1 in [-0.25, 0.25] cannot admit a path to j1 = 0.5, \
             got fraction {}",
            constrained.fraction
        );

        let unconstrained = GetCartesianPath::Request {
            avoid_collisions: false,
            ..client_request(vec![tip_at(0.5)])
        };
        assert_eq!(
            handle(&scene, unconstrained).fraction,
            1.0,
            "the same request without the constraint must solve completely, or the test above \
             is measuring something else"
        );
    }
}
