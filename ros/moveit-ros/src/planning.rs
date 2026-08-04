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
//!   knob could violate).
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
    /// could silently corrupt. `planning_time` stays unported by
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
        let trajectory = RobotTrajectoryMsgOut::try_from(res.trajectory)?.0;
        let trajectory_start = RobotStateMsgOut::try_from(res.start_state)?.0;
        Ok(PlanningResponseMsgOut(moveit_msgs::MotionPlanResponse {
            trajectory,
            trajectory_start,
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
    fn nondefault_start_state_is_rejected_not_silently_dropped() {
        let model = one_joint_model();
        let mut msg = valid_request(&model);
        msg.start_state.joint_state = r2r::sensor_msgs::msg::JointState {
            name: vec!["j1".to_string()],
            position: vec![0.1],
            ..Default::default()
        };
        let err = PlanningRequest::try_from(PlanningRequestMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
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
