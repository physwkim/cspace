// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `moveit_msgs/MotionPlanRequest`/`MotionPlanResponse` <->
//! [`moveit_planning::PlanningRequest`]/[`moveit_planning::PlanningResponse`]
//! (round 2, PORTING-PLAN.md Phase 9, lowest-priority item in this round's
//! brief).
//!
//! `PlanningRequest`'s own doc comment (`crates/moveit-planning/src/request.rs`)
//! already states its scope: it carries only the six fields this crate's own
//! request adapters read (`group_name`, `goal_constraints`,
//! `path_constraints`, `workspace_parameters` -> `workspace_bounds`,
//! `max_velocity_scaling_factor`, `max_acceleration_scaling_factor`) and
//! deliberately excludes planner-selection/tuning concerns. Of
//! `MotionPlanRequest`'s remaining wire fields:
//!
//! - `pipeline_id`/`num_planning_attempts`/`allowed_planning_time`/
//!   `cartesian_speed_limited_link`/`max_cartesian_speed`/`smoothness_level`
//!   are planner-orchestration metadata with no `PlanningRequest` field to
//!   land in, by the same documented design choice as the tuning fields
//!   above -- dropped, not rejected (there is no invariant a dropped tuning
//!   knob could violate). For `num_planning_attempts`/`allowed_planning_time`
//!   the *normalization* upstream applies to them
//!   (`planning_interface.cpp:92-103`'s `setMotionPlanRequest`) is decided
//!   separately and declined in `PORTING-PLAN.md` §236; the two
//!   `*_boundaries_are_not_observable_on_the_core_request` tests in this
//!   file's test module are that decision's expiry tripwires, and they are
//!   what makes "dropped" a checked claim here rather than a stated one.
//! - `start_state`/`reference_trajectories` are genuine *content* (an
//!   assumed robot state, seed trajectories) that `PlanningRequest` has
//!   nowhere to put -- msg->core **rejects** a message that sets either of
//!   these non-default, per D6 (same rule as `RobotState`'s
//!   `is_diff`/`attached_collision_objects`/`multi_dof_joint_state` in
//!   `state.rs`).
//! - `planner_id` and `trajectory_constraints` **are** representable
//!   (`PlanningRequest::{planner_id, trajectory_constraints}`, added to
//!   `moveit-planning` after this crate's round 2) -- mapped directly, not
//!   dropped or rejected. `trajectory_constraints` is `Vec<Constraints>` on
//!   the wire and `Vec<KinematicConstraintSet>` on the core side, i.e. the
//!   exact same per-element conversion as `goal_constraints` above, just a
//!   different field. [`PlanningResponse::planner_id`] (also added after
//!   round 2) has **no** counterpart on `MotionPlanResponse` at all --
//!   see the `TryFrom<PlanningResponseMsg>` impl below for the confirmed
//!   `.msg` text this corrects.
//!
//! `WorkspaceParameters.header` (frame_id/stamp) has no [`WorkspaceBounds`]
//! field either -- dropped as metadata, not content, matching this round's
//! `RobotTrajectory`/`RobotState`'s own treatment of `header`.
//!
//! # Not ported here: `MotionPlanDetailedResponse::getMessage`
//!
//! `moveit_core/planning_interface/src/planning_response.cpp` holds two
//! conversions. `MotionPlanResponse::getMessage` (`:40-50`) is the
//! `TryFrom<PlanningResponse<'m>> for PlanningResponseMsgOut` impl below.
//! `MotionPlanDetailedResponse::getMessage` (`:52-79`, the file's larger
//! half) **is deliberately not ported, here or anywhere**, and this crate is
//! where it would have to live: D6 puts every `moveit_msgs` conversion in
//! `moveit-ros`.
//!
//! D6 also names what is missing. A `TryFrom` needs a core-side source type,
//! and this workspace's only counterpart to
//! `planning_interface::MotionPlanDetailedResponse` is
//! `moveit_planners_chomp::ChompSolution`, which holds one `RobotTrajectory`
//! and one `description` string rather than the parallel vectors upstream
//! iterates. Every rule the upstream function has -- the first-non-empty
//! trajectory supplying `trajectory_start`/`group_name`, the `continue` over
//! empty ones, the source-indexed `description`/`processing_time` guards --
//! needs those vectors to exist, so a conversion written against
//! `ChompSolution` would carry none of them. `PORTING-PLAN.md` §234 records
//! the measurements (upstream calls this function from nowhere; no
//! `.srv`/`.action`/`.msg` embeds the `MotionPlanDetailedResponse` wire type)
//! and the three conditions that re-open the decision.

use moveit_constraints::KinematicConstraintSet;
use moveit_error::Error;
use moveit_geometry::Vector3 as CoreVector3;
use moveit_model::RobotModel;
use moveit_planning::{PlanningRequest, PlanningResponse, WorkspaceBounds};
use moveit_trajectory::RobotTrajectory;
use r2r::moveit_msgs::msg as moveit_msgs;

use crate::constraints::set::{ConstraintsMsg, ConstraintsMsgOut};
use crate::geometry::Vector3;
use crate::state::{RobotStateMsg, RobotStateMsgOut};
use crate::trajectory::{JointTrajectoryMsg, JointTrajectoryMsgOut};

fn constraints_msg_is_empty(c: &moveit_msgs::Constraints) -> bool {
    c.joint_constraints.is_empty()
        && c.position_constraints.is_empty()
        && c.orientation_constraints.is_empty()
        && c.visibility_constraints.is_empty()
}

fn robot_state_msg_is_default(s: &moveit_msgs::RobotState) -> bool {
    !s.is_diff
        && s.attached_collision_objects.is_empty()
        && s.joint_state.name.is_empty()
        && s.multi_dof_joint_state.joint_names.is_empty()
}

/// `moveit_msgs/RobotTrajectory` <-> [`RobotTrajectory`]. Only the
/// `joint_trajectory` field is representable, same gap as
/// `RobotState.multi_dof_joint_state` (`state.rs`).
pub struct RobotTrajectoryMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::RobotTrajectory,
}

/// Plain local wrapper, for the core->msg direction.
pub struct RobotTrajectoryMsgOut(pub moveit_msgs::RobotTrajectory);

impl<'m> TryFrom<RobotTrajectoryMsg<'m>> for RobotTrajectory<'m> {
    type Error = Error;

    fn try_from(wrapped: RobotTrajectoryMsg<'m>) -> Result<Self, Self::Error> {
        let RobotTrajectoryMsg { model, msg } = wrapped;
        let mdjt = &msg.multi_dof_joint_trajectory;
        if !mdjt.joint_names.is_empty() || !mdjt.points.is_empty() {
            return Err(Error::other(
                "RobotTrajectory.multi_dof_joint_trajectory has no core \
                 representation this round (same gap as \
                 RobotState.multi_dof_joint_state, see state.rs)",
            ));
        }
        RobotTrajectory::try_from(JointTrajectoryMsg {
            model,
            msg: msg.joint_trajectory,
        })
    }
}

impl<'m> TryFrom<RobotTrajectory<'m>> for RobotTrajectoryMsgOut {
    type Error = Error;

    fn try_from(traj: RobotTrajectory<'m>) -> Result<Self, Self::Error> {
        let joint_trajectory = JointTrajectoryMsgOut::try_from(traj)?.0;
        Ok(RobotTrajectoryMsgOut(moveit_msgs::RobotTrajectory {
            joint_trajectory,
            multi_dof_joint_trajectory: Default::default(),
        }))
    }
}

/// Wraps the wire message with the `&RobotModel` needed by every
/// constraint-set element conversion.
pub struct PlanningRequestMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::MotionPlanRequest,
}

/// Plain local wrapper, for the core->msg direction.
pub struct PlanningRequestMsgOut(pub moveit_msgs::MotionPlanRequest);

impl<'m> TryFrom<PlanningRequestMsg<'m>> for PlanningRequest {
    type Error = Error;

    fn try_from(wrapped: PlanningRequestMsg<'m>) -> Result<Self, Self::Error> {
        let PlanningRequestMsg { model, msg } = wrapped;

        // Expiry (PORTING-PLAN.md §153.1): both rejections below clear only
        // if `moveit_planning::PlanningRequest` itself gains the matching
        // field (`start_state`/seed trajectories) -- neither is blocked on
        // anything outside `moveit-planning`, unlike `RobotState`'s
        // `attached_collision_objects`/`is_diff` gap in `state.rs`, which
        // needs a new *conversion entry point* here, not a new core field.
        if !robot_state_msg_is_default(&msg.start_state) {
            return Err(Error::other(
                "MotionPlanRequest.start_state is not representable: \
                 PlanningRequest has no start-state field, and silently \
                 assuming a different start state than the one requested \
                 would change what the plan actually solves for",
            ));
        }
        if !msg.reference_trajectories.is_empty() {
            return Err(Error::other(
                "MotionPlanRequest.reference_trajectories has no \
                 PlanningRequest field this round",
            ));
        }

        let mut goal_constraints = Vec::with_capacity(msg.goal_constraints.len());
        for constraints_msg in msg.goal_constraints {
            goal_constraints.push(KinematicConstraintSet::try_from(ConstraintsMsg {
                model,
                msg: constraints_msg,
            })?);
        }
        let path_constraints = if constraints_msg_is_empty(&msg.path_constraints) {
            None
        } else {
            Some(KinematicConstraintSet::try_from(ConstraintsMsg {
                model,
                msg: msg.path_constraints,
            })?)
        };
        let workspace_bounds = WorkspaceBounds {
            min_corner: CoreVector3::try_from(Vector3(msg.workspace_parameters.min_corner))?,
            max_corner: CoreVector3::try_from(Vector3(msg.workspace_parameters.max_corner))?,
        };

        let mut trajectory_constraints =
            Vec::with_capacity(msg.trajectory_constraints.constraints.len());
        for constraints_msg in msg.trajectory_constraints.constraints {
            trajectory_constraints.push(KinematicConstraintSet::try_from(ConstraintsMsg {
                model,
                msg: constraints_msg,
            })?);
        }

        Ok(PlanningRequest {
            group_name: msg.group_name,
            goal_constraints,
            path_constraints,
            workspace_bounds,
            max_velocity_scaling_factor: msg.max_velocity_scaling_factor,
            max_acceleration_scaling_factor: msg.max_acceleration_scaling_factor,
            trajectory_constraints,
            planner_id: msg.planner_id,
        })
    }
}

impl TryFrom<PlanningRequest> for PlanningRequestMsgOut {
    type Error = Error;

    fn try_from(req: PlanningRequest) -> Result<Self, Self::Error> {
        let mut goal_constraints = Vec::with_capacity(req.goal_constraints.len());
        for set in req.goal_constraints {
            goal_constraints.push(ConstraintsMsgOut::try_from(set)?.0);
        }
        let path_constraints = match req.path_constraints {
            Some(set) => ConstraintsMsgOut::try_from(set)?.0,
            None => Default::default(),
        };
        let mut trajectory_constraints_msg = Vec::with_capacity(req.trajectory_constraints.len());
        for set in req.trajectory_constraints {
            trajectory_constraints_msg.push(ConstraintsMsgOut::try_from(set)?.0);
        }
        Ok(PlanningRequestMsgOut(moveit_msgs::MotionPlanRequest {
            workspace_parameters: moveit_msgs::WorkspaceParameters {
                min_corner: Vector3::try_from(req.workspace_bounds.min_corner)?.0,
                max_corner: Vector3::try_from(req.workspace_bounds.max_corner)?.0,
                ..Default::default()
            },
            goal_constraints,
            path_constraints,
            group_name: req.group_name,
            max_velocity_scaling_factor: req.max_velocity_scaling_factor,
            max_acceleration_scaling_factor: req.max_acceleration_scaling_factor,
            trajectory_constraints: moveit_msgs::TrajectoryConstraints {
                constraints: trajectory_constraints_msg,
            },
            planner_id: req.planner_id,
            ..Default::default()
        }))
    }
}

/// Wraps the wire message with the `&RobotModel` [`RobotTrajectoryMsg`]
/// needs.
pub struct PlanningResponseMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: moveit_msgs::MotionPlanResponse,
}

/// Plain local wrapper, for the core->msg direction.
pub struct PlanningResponseMsgOut(pub moveit_msgs::MotionPlanResponse);

impl<'m> TryFrom<PlanningResponseMsg<'m>> for PlanningResponse<'m> {
    type Error = Error;

    /// `trajectory_start` maps to [`PlanningResponse::start_state`], which
    /// `moveit-planning`'s `pipeline::generate_plan` fills before any planner
    /// runs. It carried no core field until that one landed, and was listed
    /// here as dropped; decoding it is what keeps a response that crosses the
    /// wire twice equal to the one that never left.
    ///
    /// `group_name`/`planning_time`/`error_code` still have no
    /// [`PlanningResponse`] field (see that type's own doc comment:
    /// `error_code` is this crate's `Result` instead) -- dropped, not
    /// rejected, since none of them are trajectory content the conversion
    /// could silently corrupt. `group_name` is asymmetric for that reason and
    /// only in this direction: msg->core has nowhere to put it, while core->msg
    /// derives it from the trajectory the way `planning_response.cpp:48` does
    /// (see the opposite impl below), so a `group_name` set on the wire is lost
    /// on the way in and re-derived on the way out rather than preserved. `planning_time` stays unported by
    /// p1-fixtures' own conclusion (`crates/moveit-planning/src/response.rs:39-68`):
    /// every upstream fill site sits inside a `PlanningContext`-equivalent's
    /// `solve()`, never the pipeline this crate ports, and no crate in this
    /// workspace implements [`moveit_planning::pipeline::Planner`] yet --
    /// there is no reachable site to fill it from. Expires the moment any
    /// crate implements `Planner` for a concrete planner; `moveit-planning`'s
    /// call, not this crate's.
    ///
    /// `planner_id` has **no** wire counterpart on this message:
    /// `moveit-planning`'s own doc comment on
    /// [`PlanningResponse::planner_id`] claims it matches "an unset
    /// `moveit_msgs::msg::MotionPlanResponse::planner_id`", but
    /// `third_party/moveit_msgs/msg/MotionPlanResponse.msg` (confirmed
    /// against both the `.msg` text and the r2r-generated struct, whose
    /// only fields are `trajectory_start`/`group_name`/`trajectory`/
    /// `planning_time`/`error_code`) has no `planner_id` field at all --
    /// only `MotionPlanRequest` does. This is a genuine, previously
    /// undocumented gap the other direction from `MoveItErrorCodes.val`
    /// (§2): a core-only field with nowhere on *this* wire message to go.
    /// msg->core has no source, so `planner_id` is always `""` (unset,
    /// matching `PlanningRequest`/`PlanningResponse`'s shared "empty string
    /// means unset" convention); core->msg has nowhere to put a non-empty
    /// value, so it is dropped, not rejected -- rejecting would make this
    /// conversion fail on every response a real planner produces, since
    /// `moveit-planning`'s own `pipeline::generate_plan` always fills
    /// `planner_id` in (backfilled from the request if the planner left it
    /// blank -- never itself empty in practice).
    fn try_from(wrapped: PlanningResponseMsg<'m>) -> Result<Self, Self::Error> {
        let PlanningResponseMsg { model, msg } = wrapped;
        let trajectory = RobotTrajectory::try_from(RobotTrajectoryMsg {
            model,
            msg: msg.trajectory,
        })?;
        let start_state = moveit_state::RobotState::try_from(RobotStateMsg {
            model,
            msg: msg.trajectory_start,
        })?;
        Ok(PlanningResponse {
            trajectory,
            planner_id: String::new(),
            start_state,
        })
    }
}

impl<'m> TryFrom<PlanningResponse<'m>> for PlanningResponseMsgOut {
    type Error = Error;

    fn try_from(res: PlanningResponse<'m>) -> Result<Self, Self::Error> {
        // `moveit_core/planning_interface/src/planning_response.cpp:48`:
        // `msg.group_name = trajectory->getGroupName()`. This side can derive
        // it -- [`moveit_trajectory::RobotTrajectory::group_name`] is the same
        // accessor with the same empty-group answer as upstream's
        // `getGroupName` (`robot_trajectory.cpp:88-94`: the group's name, or
        // `""` when `group_` is null) -- so leaving the wire field empty was
        // dropping a field with a source, unlike `planning_time`/`error_code`,
        // which have no [`PlanningResponse`] field to read at all.
        //
        // Read before the move below, and guarded on emptiness because
        // upstream's is: its three `trajectory_start`/`trajectory`/`group_name`
        // writes all sit inside `if (trajectory && !trajectory->empty())`, so a
        // group name on a message carrying no waypoints is a combination
        // upstream never emits.
        let group_name = if res.trajectory.is_empty() {
            String::new()
        } else {
            res.trajectory.group_name().to_string()
        };
        let trajectory = RobotTrajectoryMsgOut::try_from(res.trajectory)?.0;
        let trajectory_start = RobotStateMsgOut::try_from(res.start_state)?.0;
        Ok(PlanningResponseMsgOut(moveit_msgs::MotionPlanResponse {
            trajectory,
            trajectory_start,
            group_name,
            error_code: moveit_msgs::MoveItErrorCodes {
                // `r2r`-generated constant, not a literal (PORTING-PLAN.md
                // §191.2) -- see doc/message-mapping.md §2's note on
                // MoveItErrorCodes.message/source being a separate, still-open gap
                val: moveit_msgs::MoveItErrorCodes::SUCCESS as i32,
                ..Default::default()
            },
            ..Default::default()
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    /// Asserts the call was rejected *for the reason named*, not merely
    /// that it was rejected. `TryFrom<PlanningRequestMsg>::try_from` has two
    /// independent `Error::Other` sites (`start_state`,
    /// `reference_trajectories`) -- `matches!(err, Error::Other(_))` alone
    /// cannot tell a test that a routing bug swapped which branch fired
    /// (same shape as `moveit-constraints`' `e3b40c6`).
    #[track_caller]
    fn assert_err_mentions<T: std::fmt::Debug>(result: Result<T, Error>, needle: &str) {
        let rendered = result
            .expect_err("expected this call to be rejected")
            .to_string();
        assert!(
            rendered.contains(needle),
            "expected the rejection to come from the branch that reports {needle:?}, got: {rendered}"
        );
    }

    fn identity_workspace(model: &RobotModel) -> moveit_msgs::WorkspaceParameters {
        moveit_msgs::WorkspaceParameters {
            header: r2r::std_msgs::msg::Header {
                frame_id: model.model_frame().to_string(),
                ..Default::default()
            },
            min_corner: r2r::geometry_msgs::msg::Vector3 {
                x: -1.0,
                y: -1.0,
                z: -1.0,
            },
            max_corner: r2r::geometry_msgs::msg::Vector3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        }
    }

    fn joint_goal(name: &str, position: f64) -> moveit_msgs::Constraints {
        moveit_msgs::Constraints {
            name: String::new(),
            joint_constraints: vec![moveit_msgs::JointConstraint {
                joint_name: name.to_string(),
                position,
                tolerance_above: 0.01,
                tolerance_below: 0.01,
                weight: 1.0,
            }],
            position_constraints: vec![],
            orientation_constraints: vec![],
            visibility_constraints: vec![],
        }
    }

    fn valid_request(model: &RobotModel) -> moveit_msgs::MotionPlanRequest {
        moveit_msgs::MotionPlanRequest {
            workspace_parameters: identity_workspace(model),
            group_name: "arm".to_string(),
            goal_constraints: vec![joint_goal("j1", 0.5)],
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            ..Default::default()
        }
    }

    #[test]
    fn converts_minimal_request() {
        let model = one_joint_model();
        let req = PlanningRequest::try_from(PlanningRequestMsg {
            model: &model,
            msg: valid_request(&model),
        })
        .unwrap();
        assert_eq!(req.group_name, "arm");
        assert_eq!(req.goal_constraints.len(), 1);
        assert!(req.path_constraints.is_none());
    }

    #[test]
    fn nonempty_path_constraints_is_mapped_not_treated_as_absent() {
        // Sibling of `converts_minimal_request`'s `path_constraints.is_none()`
        // check: that fixture's `path_constraints` is empty by construction,
        // so a bare `is_none()` there cannot tell "correctly detected an
        // empty message" apart from a `constraints_msg_is_empty` that
        // returns `true` unconditionally, ignoring its argument.
        let model = one_joint_model();
        let mut msg = valid_request(&model);
        msg.path_constraints = joint_goal("j1", 0.1);
        let req = PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }).unwrap();
        assert_eq!(req.path_constraints.as_ref().unwrap().len(), 1);

        let back = PlanningRequestMsgOut::try_from(req).unwrap().0;
        assert_eq!(back.path_constraints.joint_constraints.len(), 1);
        assert_eq!(back.path_constraints.joint_constraints[0].joint_name, "j1");
    }

    #[test]
    fn nondefault_start_state_is_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let mut msg = valid_request(&model);
        msg.start_state.joint_state = r2r::sensor_msgs::msg::JointState {
            name: vec!["j1".to_string()],
            position: vec![0.1],
            ..Default::default()
        };
        assert_err_mentions(
            PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }),
            "start_state is not representable",
        );
    }

    #[test]
    fn nonempty_reference_trajectories_is_rejected_not_silently_dropped() {
        // Sibling of `nondefault_start_state_is_rejected_not_silently_dropped`
        // in the same function -- previously untested entirely.
        let model = one_joint_model();
        let mut msg = valid_request(&model);
        msg.reference_trajectories.push(Default::default());
        assert_err_mentions(
            PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }),
            "reference_trajectories has no",
        );
    }

    #[test]
    fn trajectory_constraints_and_planner_id_are_mapped_not_dropped() {
        let model = one_joint_model();
        let mut msg = valid_request(&model);
        msg.trajectory_constraints
            .constraints
            .push(joint_goal("j1", 0.0));
        msg.planner_id = "RRTConnectkConfigDefault".to_string();
        let req = PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }).unwrap();
        assert_eq!(req.trajectory_constraints.len(), 1);
        assert_eq!(req.planner_id, "RRTConnectkConfigDefault");

        let back = PlanningRequestMsgOut::try_from(req).unwrap().0;
        assert_eq!(back.trajectory_constraints.constraints.len(), 1);
        assert_eq!(back.planner_id, "RRTConnectkConfigDefault");
    }

    /// Given one `(label, rendering)` per boundary value, returns the first
    /// row's label and the labels of every later row that differs from it.
    ///
    /// Each row is compared against a row built from a *different* input value
    /// rather than against a baseline built the same way, because the msg
    /// default is itself one of the boundaries in both callers below (`0.0`,
    /// `0`) and a baseline comparison would leave that row checking a
    /// conversion against itself. All differing rows are returned, not the
    /// first found, so one observable boundary cannot hide the rest.
    fn labels_differing_from_the_first(
        rows: &[(&'static str, String)],
    ) -> (&'static str, Vec<&'static str>) {
        let (first_label, first) = &rows[0];
        (
            first_label,
            rows[1..]
                .iter()
                .filter(|(_, rendering)| rendering != first)
                .map(|(label, _)| *label)
                .collect(),
        )
    }

    /// Expiry tripwire for `PORTING-PLAN.md` §236, the decision not to port
    /// `PlanningContext::setMotionPlanRequest`'s normalization
    /// (`moveit_core/planning_interface/src/planning_interface.cpp:92-96`:
    /// `allowed_planning_time <= 0.0` becomes `1.0`).
    ///
    /// That decision rests on the field having no reader on this side, which
    /// is a claim about absence: there is no clamp here to call, so the clamp
    /// cannot be what is tested. The premise can be. Every value the upstream
    /// rule distinguishes must be indistinguishable *here* — including the two
    /// the upstream guard itself lets through, `f64::NAN` (which fails
    /// `<= 0.0`) and a positive budget below the 1 µs its consumer can
    /// represent; both are `doc/upstream-bugs.md`'s
    /// `set-motion-plan-request-time-guard-polarity`.
    ///
    /// [`PlanningRequest`] derives `Debug`/`Clone`/`Default` and not
    /// `PartialEq`, so rows are compared on the derived `Debug` rendering,
    /// which prints every field.
    #[test]
    fn allowed_planning_time_boundaries_are_not_observable_on_the_core_request() {
        let model = one_joint_model();
        let boundaries: [(&'static str, f64); 5] = [
            ("-1.0, which upstream logs about and clamps to 1.0", -1.0),
            ("0.0, the msg default, clamped to 1.0 without a log", 0.0),
            ("f64::EPSILON, positive so upstream keeps it", f64::EPSILON),
            ("5.0, a normal budget", 5.0),
            (
                "f64::NAN, which fails `<= 0.0` so upstream keeps it",
                f64::NAN,
            ),
        ];

        let rows: Vec<(&'static str, String)> = boundaries
            .iter()
            .map(|(label, value)| {
                let mut msg = valid_request(&model);
                msg.allowed_planning_time = *value;
                let req =
                    PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }).unwrap();
                (*label, format!("{req:?}"))
            })
            .collect();

        let (first, observable) = labels_differing_from_the_first(&rows);
        assert!(
            observable.is_empty(),
            "MotionPlanRequest.allowed_planning_time reached PlanningRequest at {observable:?}, \
             differing from the row for {first:?}: the field now has a reader here, so §236's \
             decision not to port planning_interface.cpp:92-96 has expired and the clamp (or a \
             replacement that also rejects NaN and sub-microsecond budgets) must be re-decided"
        );
    }

    /// Sibling of `allowed_planning_time_boundaries_are_not_observable_on_the_core_request`
    /// for the other half of the same upstream function
    /// (`planning_interface.cpp:98-103`: `RCLCPP_ERROR` for `< 0`, then
    /// `num_planning_attempts = std::max(1, num_planning_attempts)` for every
    /// value). `-1` and `0` are separate rows because upstream treats them
    /// differently — only the negative one is reported — even though both end
    /// as `1`.
    #[test]
    fn num_planning_attempts_boundaries_are_not_observable_on_the_core_request() {
        let model = one_joint_model();
        let boundaries: [(&'static str, i32); 4] = [
            ("-1, the only value upstream logs an error for", -1),
            ("0, the msg default, raised to 1 silently", 0),
            ("1, already the value the clamp produces", 1),
            ("2, the first value the clamp leaves alone", 2),
        ];

        let rows: Vec<(&'static str, String)> = boundaries
            .iter()
            .map(|(label, value)| {
                let mut msg = valid_request(&model);
                msg.num_planning_attempts = *value;
                let req =
                    PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }).unwrap();
                (*label, format!("{req:?}"))
            })
            .collect();

        let (first, observable) = labels_differing_from_the_first(&rows);
        assert!(
            observable.is_empty(),
            "MotionPlanRequest.num_planning_attempts reached PlanningRequest at {observable:?}, \
             differing from the row for {first:?}: the field now has a reader here, so §236's \
             decision not to port planning_interface.cpp:98-103 has expired and must be re-decided"
        );
    }

    /// `planning_response.cpp:44-49` writes `group_name` only inside its
    /// `if (trajectory && !trajectory->empty())` guard, so both sides of that
    /// guard are checked here: without the empty case a conversion that always
    /// emitted the group name would pass, and that is a message upstream never
    /// produces.
    #[test]
    fn response_group_name_comes_from_the_trajectory_and_only_when_it_has_waypoints() {
        let model = crate::state::tests::one_joint_model_with_arm_group();

        let mut traj = RobotTrajectory::for_group_name(&model, "arm").unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_variable_position("j1", 0.4).unwrap();
        traj.add_suffix_way_point(state, 0.0).unwrap();
        let filled = PlanningResponseMsgOut::try_from(PlanningResponse {
            trajectory: traj,
            planner_id: String::new(),
            start_state: moveit_state::RobotState::new(&model),
        })
        .unwrap()
        .0;
        assert_eq!(filled.group_name, "arm");

        let empty = PlanningResponseMsgOut::try_from(PlanningResponse {
            trajectory: RobotTrajectory::for_group_name(&model, "arm").unwrap(),
            planner_id: String::new(),
            start_state: moveit_state::RobotState::new(&model),
        })
        .unwrap()
        .0;
        assert_eq!(
            empty.group_name, "",
            "upstream's guard leaves group_name unset for an empty trajectory, \
             even one carrying a group"
        );
    }

    #[test]
    fn round_trip_request_through_msg() {
        let model = one_joint_model();
        let req = PlanningRequest::try_from(PlanningRequestMsg {
            model: &model,
            msg: valid_request(&model),
        })
        .unwrap();
        let back = PlanningRequestMsgOut::try_from(req).unwrap().0;
        assert_eq!(back.group_name, "arm");
        assert_eq!(back.goal_constraints.len(), 1);
        assert_eq!(
            back.goal_constraints[0].joint_constraints[0].joint_name,
            "j1"
        );
    }

    #[test]
    fn converts_response_trajectory() {
        let model = one_joint_model();
        let mut traj = RobotTrajectory::new(&model);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_variable_position("j1", 0.2).unwrap();
        traj.add_suffix_way_point(state, 0.0).unwrap();
        let traj_msg = crate::trajectory::JointTrajectoryMsgOut::try_from(traj)
            .unwrap()
            .0;

        let msg = moveit_msgs::MotionPlanResponse {
            trajectory: moveit_msgs::RobotTrajectory {
                joint_trajectory: traj_msg,
                multi_dof_joint_trajectory: Default::default(),
            },
            ..Default::default()
        };
        let res = PlanningResponse::try_from(PlanningResponseMsg { model: &model, msg }).unwrap();
        assert_eq!(res.trajectory.way_point_count(), 1);
    }

    #[test]
    fn multi_dof_joint_trajectory_is_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let mut msg = moveit_msgs::RobotTrajectory::default();
        msg.multi_dof_joint_trajectory.joint_names = vec!["virtual_joint".to_string()];
        let err = RobotTrajectory::try_from(RobotTrajectoryMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    // Assertion-discrimination sweep (round 8, folded-operand audit): the
    // guard is `!mdjt.joint_names.is_empty() || !mdjt.points.is_empty()`,
    // but only `joint_names` had a test above -- `points` was a blind
    // operand a dropped `||` clause would not have been caught by anything.
    #[test]
    fn multi_dof_joint_trajectory_points_is_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let mut msg = moveit_msgs::RobotTrajectory::default();
        msg.multi_dof_joint_trajectory.points = vec![Default::default()];
        let err = RobotTrajectory::try_from(RobotTrajectoryMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
    }

    #[test]
    fn round_trip_response_through_msg() {
        let model = one_joint_model();
        let mut traj = RobotTrajectory::new(&model);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_variable_position("j1", 0.3).unwrap();
        traj.add_suffix_way_point(state, 0.0).unwrap();
        // Deliberately not the trajectory's own first waypoint: a conversion
        // that reconstructed `start_state` from the trajectory instead of from
        // `trajectory_start` would pass with the two equal.
        let mut start = moveit_state::RobotState::new(&model);
        start.set_variable_position("j1", -0.7).unwrap();
        let res = PlanningResponse {
            trajectory: traj,
            planner_id: "STOMP".to_string(),
            start_state: start,
        };
        let msg = PlanningResponseMsgOut::try_from(res).unwrap().0;
        assert_eq!(msg.error_code.val, 1);
        let back = PlanningResponse::try_from(PlanningResponseMsg { model: &model, msg }).unwrap();
        assert_eq!(back.start_state.variable_position("j1").unwrap(), -0.7);
        // `planner_id` has no wire counterpart on `MotionPlanResponse` (see
        // the `TryFrom<PlanningResponseMsg>` impl's doc) -- "STOMP" is
        // dropped on the way out, not preserved.
        assert_eq!(back.planner_id, "");
        assert_eq!(
            back.trajectory
                .way_point(0)
                .unwrap()
                .variable_position("j1")
                .unwrap(),
            0.3
        );
    }
}
