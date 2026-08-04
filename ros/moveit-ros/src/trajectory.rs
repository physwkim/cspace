// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `trajectory_msgs/JointTrajectory` <-> [`moveit_trajectory::RobotTrajectory`]
//! (round 2, PORTING-PLAN.md Phase 9). See `doc/message-mapping.md` §10 for
//! the full survey this module codes against (`moveit_msgs/RobotTrajectory`'s
//! `multi_dof_joint_trajectory` field is a separate, not-yet-coded gap, same
//! shape as `RobotState`'s -- this module only handles the single
//! `joint_trajectory` field, matching this round's brief).

use moveit_error::Error;
use moveit_model::RobotModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use r2r::trajectory_msgs::msg as trajectory_msgs;

/// Wraps `trajectory_msgs::msg::JointTrajectory` with the `&RobotModel`
/// needed to build each waypoint's [`RobotState`] (same context-carrying
/// wrapper shape as [`crate::state::RobotStateMsg`]).
pub struct JointTrajectoryMsg<'m> {
    pub model: &'m RobotModel,
    pub msg: trajectory_msgs::JointTrajectory,
}

/// Wraps `trajectory_msgs::msg::JointTrajectory` as a plain local newtype,
/// for the core->msg direction.
pub struct JointTrajectoryMsgOut(pub trajectory_msgs::JointTrajectory);

fn duration_seconds(d: &r2r::builtin_interfaces::msg::Duration) -> f64 {
    d.sec as f64 + d.nanosec as f64 * 1e-9
}

fn seconds_to_duration(t: f64) -> r2r::builtin_interfaces::msg::Duration {
    let sec = t.floor();
    r2r::builtin_interfaces::msg::Duration {
        sec: sec as i32,
        nanosec: ((t - sec) * 1e9).round() as u32,
    }
}

fn set_point_array(
    state: &mut RobotState,
    joint_names: &[String],
    values: &[f64],
    field: &'static str,
    set_by_name: impl Fn(&mut RobotState, &str, f64) -> moveit_error::Result<()>,
) -> moveit_error::Result<()> {
    if !values.is_empty() && values.len() != joint_names.len() {
        return Err(Error::construct(format!(
            "JointTrajectoryPoint.{field} has length {} but joint_names has \
             length {}",
            values.len(),
            joint_names.len()
        )));
    }
    for (name, &value) in joint_names.iter().zip(values.iter()) {
        set_by_name(state, name, value)?;
    }
    Ok(())
}

impl<'m> TryFrom<JointTrajectoryMsg<'m>> for RobotTrajectory<'m> {
    type Error = Error;

    fn try_from(wrapped: JointTrajectoryMsg<'m>) -> Result<Self, Self::Error> {
        let JointTrajectoryMsg { model, msg } = wrapped;
        let mut traj = RobotTrajectory::new(model);
        let mut prev_t = 0.0f64;

        for (i, point) in msg.points.iter().enumerate() {
            if point.positions.len() != msg.joint_names.len() {
                return Err(Error::construct(format!(
                    "JointTrajectoryPoint[{i}].positions has length {} but \
                     joint_names has length {}",
                    point.positions.len(),
                    msg.joint_names.len()
                )));
            }
            let mut state = RobotState::new(model);
            for (name, &pos) in msg.joint_names.iter().zip(point.positions.iter()) {
                state.set_variable_position(name, pos)?;
            }
            set_point_array(
                &mut state,
                &msg.joint_names,
                &point.velocities,
                "velocities",
                |s, n, v| s.set_variable_velocity(n, v),
            )?;
            set_point_array(
                &mut state,
                &msg.joint_names,
                &point.accelerations,
                "accelerations",
                |s, n, v| s.set_variable_acceleration(n, v),
            )?;
            set_point_array(
                &mut state,
                &msg.joint_names,
                &point.effort,
                "effort",
                |s, n, v| s.set_variable_effort(n, v),
            )?;

            let t = duration_seconds(&point.time_from_start);
            let dt = if i == 0 { t } else { t - prev_t };
            // `add_suffix_way_point`'s own invariant is `duration_from_previous[0]
            // == 0.0`; a nonzero first `time_from_start` has no core
            // representation and must be rejected, not silently zeroed (D6) --
            // see doc/message-mapping.md §10.
            if i == 0 && t != 0.0 {
                return Err(Error::construct(format!(
                    "JointTrajectoryPoint[0].time_from_start is {t}s, not 0s; \
                     RobotTrajectory's duration_from_previous[0] is \
                     structurally 0.0 and cannot represent a nonzero start \
                     offset"
                )));
            }
            if dt < 0.0 {
                return Err(Error::construct(format!(
                    "JointTrajectoryPoint[{i}].time_from_start ({t}s) is \
                     less than point[{}]'s ({prev_t}s); time_from_start must \
                     be non-decreasing",
                    i - 1
                )));
            }
            traj.add_suffix_way_point(state, dt)?;
            prev_t = t;
        }
        Ok(traj)
    }
}

impl<'m> TryFrom<RobotTrajectory<'m>> for JointTrajectoryMsgOut {
    type Error = Error;

    /// Total: every waypoint is a full [`RobotState`] over the same model,
    /// so `joint_names`/positions always line up.
    fn try_from(traj: RobotTrajectory<'m>) -> Result<Self, Self::Error> {
        let joint_names = traj.robot_model().variable_names().to_vec();
        let mut points = Vec::with_capacity(traj.way_point_count());
        let mut t = 0.0f64;
        for (i, (state, dt)) in traj.iter().enumerate() {
            if i > 0 {
                t += dt;
            }
            points.push(trajectory_msgs::JointTrajectoryPoint {
                positions: state.positions().to_vec(),
                velocities: if state.has_velocities() {
                    state.velocities().to_vec()
                } else {
                    Vec::new()
                },
                accelerations: if state.has_accelerations() {
                    state.accelerations().to_vec()
                } else {
                    Vec::new()
                },
                effort: if state.has_effort() {
                    state.effort().to_vec()
                } else {
                    Vec::new()
                },
                time_from_start: seconds_to_duration(t),
            });
        }
        Ok(JointTrajectoryMsgOut(trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names,
            points,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::tests::one_joint_model;

    fn point(position: f64, sec: i32, nanosec: u32) -> trajectory_msgs::JointTrajectoryPoint {
        trajectory_msgs::JointTrajectoryPoint {
            positions: vec![position],
            velocities: vec![],
            accelerations: vec![],
            effort: vec![],
            time_from_start: r2r::builtin_interfaces::msg::Duration { sec, nanosec },
        }
    }

    #[test]
    fn converts_and_computes_deltas_from_cumulative_time() {
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![
                point(0.0, 0, 0),
                point(0.5, 1, 0),
                point(1.0, 1, 500_000_000),
            ],
        };
        let traj = RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }).unwrap();
        assert_eq!(traj.way_point_count(), 3);
        assert_eq!(*traj.way_point_durations(), [0.0, 1.0, 0.5]);
        assert_eq!(
            traj.way_point(1).unwrap().variable_position("j1").unwrap(),
            0.5
        );
    }

    #[test]
    fn nonzero_start_time_is_rejected() {
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![point(0.0, 1, 0)],
        };
        let err = RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn decreasing_time_from_start_is_rejected() {
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![
                point(0.0, 0, 0),
                point(0.1, 1, 0),
                point(0.2, 0, 500_000_000),
            ],
        };
        let err = RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn round_trip_through_msg() {
        let model = one_joint_model();
        let mut traj = RobotTrajectory::new(&model);
        let mut s0 = RobotState::new(&model);
        s0.set_variable_position("j1", 0.0).unwrap();
        traj.add_suffix_way_point(s0, 0.0).unwrap();
        let mut s1 = RobotState::new(&model);
        s1.set_variable_position("j1", 1.0).unwrap();
        traj.add_suffix_way_point(s1, 2.0).unwrap();

        let msg = JointTrajectoryMsgOut::try_from(traj).unwrap().0;
        assert_eq!(msg.points[1].time_from_start.sec, 2);
        let back = RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }).unwrap();
        assert_eq!(*back.way_point_durations(), [0.0, 2.0]);
        assert_eq!(
            back.way_point(1).unwrap().variable_position("j1").unwrap(),
            1.0
        );
    }
}
