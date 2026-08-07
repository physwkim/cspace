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
    /// Resolves `msg.joint_names` to the variable indices each waypoint sets.
    pub model: &'m RobotModel,
    /// The wire message, unmodified.
    pub msg: trajectory_msgs::JointTrajectory,
}

/// Wraps `trajectory_msgs::msg::JointTrajectory` as a plain local newtype,
/// for the core->msg direction.
pub struct JointTrajectoryMsgOut(pub trajectory_msgs::JointTrajectory);

fn duration_seconds(d: &r2r::builtin_interfaces::msg::Duration) -> f64 {
    d.sec as f64 + d.nanosec as f64 * 1e-9
}

/// `t` -> `builtin_interfaces/Duration {sec: i32, nanosec: u32}`.
///
/// PORTING-PLAN.md §172: narrowing a `double` to `int`/`unsigned` is UB in
/// C++ for out-of-range magnitude, a negative value, and `NaN` alike, while
/// Rust's `as` instead saturates or zeroes -- neither is "the same answer"
/// upstream gives, and there is no upstream answer to compare against for
/// an input in that range. §153.1: this crate has no legitimate negative,
/// non-finite, or beyond-`i32::MAX`-second `time_from_start` (a
/// `RobotTrajectory` spanning more than ~68 years), so those inputs are
/// rejected here rather than silently saturated or zeroed. Expires only if
/// a caller needs to represent a trajectory that long.
fn seconds_to_duration(t: f64) -> moveit_error::Result<r2r::builtin_interfaces::msg::Duration> {
    if !t.is_finite() || t < 0.0 || t > i32::MAX as f64 {
        return Err(Error::construct(format!(
            "time_from_start {t}s is negative, non-finite, or exceeds \
             i32::MAX seconds; cannot represent as builtin_interfaces/Duration \
             (PORTING-PLAN.md §172)"
        )));
    }
    let sec = t.floor();
    let mut nanosec = ((t - sec) * 1e9).round();
    let mut sec = sec as i32;
    // `t - sec` (pre-cast `sec`) is in [0, 1) for any finite `t` (`sec` is
    // its floor), so `nanosec` before rounding is in [0, 1e9) -- but
    // rounding a value just under 1e9 can itself land exactly on 1e9,
    // which is not a valid Duration.nanosec. Carry that one nanosecond
    // into `sec` instead of emitting it: a nonzero carry means `t` had a
    // nonzero fractional part, so `sec < t <= i32::MAX` (the guard above),
    // meaning `sec <= i32::MAX - 1` and `sec + 1` cannot overflow `i32`.
    if nanosec >= 1e9 {
        nanosec -= 1e9;
        sec += 1;
    }
    Ok(r2r::builtin_interfaces::msg::Duration {
        sec,
        nanosec: nanosec as u32,
    })
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
        set_by_name(state, name, value)
            .map_err(|e| Error::construct(format!("JointTrajectoryPoint.{field}: {e}")))?;
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
                state.set_variable_position(name, pos).map_err(|e| {
                    Error::construct(format!("JointTrajectoryPoint[{i}].positions: {e}"))
                })?;
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
            //
            // Expiry (PORTING-PLAN.md §153.1, round 13 correction): not a
            // missing field, so a new `moveit_trajectory::RobotTrajectory`
            // field cannot clear this -- it expires only if that crate's own
            // `add_suffix_way_point` invariant relaxes to allow a nonzero
            // start-time offset. Unlike D14's `weight` case, this rejection
            // is only *half* mechanically visible: `add_suffix_way_point`
            // already enforces the identical condition
            // (`waypoints.is_empty() && dt != 0.0`) internally, so
            // `tests::add_suffix_way_point_rejects_a_nonzero_first_dt` below
            // is a real tripwire on that invariant and goes red the moment
            // it relaxes -- but this check right here does NOT go red with
            // it, because it fires first and short-circuits before ever
            // calling `add_suffix_way_point` (it exists only to give a
            // wire-specific message; see the two error strings). A relaxed
            // core invariant would go unnoticed at the wire boundary unless
            // a person also removes or updates this `if`, purely from
            // reading the tripwire failure -- still §153.1's memory case for
            // *this* line, even though the underlying fact is now
            // tripwired.
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

    /// Every waypoint is a full [`RobotState`] over the same model, so
    /// `joint_names`/positions always line up -- but not total: `dt`
    /// (`RobotTrajectory::add_suffix_way_point`'s parameter, `moveit-trajectory`)
    /// carries no validation of its own, so a negative, non-finite, or
    /// beyond-`i32::MAX`-second cumulative time can reach `seconds_to_duration`,
    /// which now rejects it (PORTING-PLAN.md §172) rather than silently
    /// saturating or zeroing it into the output message.
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
                time_from_start: seconds_to_duration(t)?,
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

    /// Asserts the call was rejected *for the reason named*, not merely
    /// that it was rejected. `TryFrom<JointTrajectoryMsg>::try_from`'s loop
    /// body has three independent `Error::Construct` sites (positions-length
    /// mismatch, nonzero start time, decreasing time) and `set_point_array`
    /// has a fourth (velocities/accelerations/effort length mismatch) --
    /// all the same variant, so `matches!(err, Error::Construct(_))` alone
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
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "not 0s",
        );
    }

    #[test]
    fn positions_length_mismatch_is_rejected() {
        // The sibling of `nonzero_start_time_is_rejected` and
        // `decreasing_time_from_start_is_rejected` in the same loop body --
        // previously untested (not merely undiscriminated).
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string(), "j2".to_string()],
            points: vec![point(0.0, 0, 0)],
        };
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "positions has length",
        );
    }

    #[test]
    fn velocities_length_mismatch_is_rejected() {
        // `set_point_array`'s own Error::Construct site, called with a
        // different `field` for velocities/accelerations/effort --
        // previously untested for any of the three.
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![trajectory_msgs::JointTrajectoryPoint {
                positions: vec![0.0],
                velocities: vec![1.0, 2.0],
                accelerations: vec![],
                effort: vec![],
                time_from_start: r2r::builtin_interfaces::msg::Duration { sec: 0, nanosec: 0 },
            }],
        };
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "velocities has length",
        );
    }

    #[test]
    fn unknown_joint_name_in_positions_is_rejected() {
        // Previously untested entirely (not merely undiscriminated): the
        // positions loop's per-name `Error::UnknownName` had no test at all.
        // Not a sibling of `set_point_array`'s velocities/accelerations/effort
        // calls: those share this same loop's `msg.joint_names`, and
        // positions' unconditional (non-`is_empty`-exempt) length check runs
        // first against every name in that list, via the same
        // `RobotModel::variable_index` lookup positions itself uses -- so an
        // unresolvable name always fails here first, before
        // velocities/accelerations/effort's own per-name loops ever run.
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["no_such_joint".to_string()],
            points: vec![point(0.0, 0, 0)],
        };
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "JointTrajectoryPoint[0].positions: no variable named",
        );
    }

    /// Tripwire (PORTING-PLAN.md §153.1/§205) on `moveit_trajectory::
    /// RobotTrajectory::add_suffix_way_point`'s own invariant, bypassing
    /// this crate's `TryFrom` entirely -- see the expiry comment on the
    /// `i == 0 && t != 0.0` check above for why the wire-level
    /// `nonzero_start_time_is_rejected` test above cannot serve as this
    /// tripwire (it short-circuits before ever reaching
    /// `add_suffix_way_point`). If this goes red, `moveit_trajectory`'s
    /// invariant relaxed to allow a nonzero first `dt` -- go update or
    /// remove `trajectory.rs`'s own `i == 0 && t != 0.0` check to match,
    /// since that check no longer describes a core limitation once this
    /// fails.
    #[test]
    fn add_suffix_way_point_rejects_a_nonzero_first_dt() {
        let model = one_joint_model();
        let mut traj = RobotTrajectory::new(&model);
        let state = RobotState::new(&model);
        let err = traj.add_suffix_way_point(state, 1.0).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "got: {err:?}");
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
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "is less than point",
        );
    }

    // Assertion-discrimination sweep (round 8, folded-operand audit): the
    // wire-boundary guard is `i == 0 && t != 0.0` -- a different check from
    // `add_suffix_way_point`'s own internal invariant of the same shape
    // (`add_suffix_way_point_rejects_a_nonzero_first_dt` above never goes
    // through `TryFrom<JointTrajectoryMsg>` at all, so it is not evidence
    // for this guard). Before this round neither operand of *this* guard
    // had any test through the wire path. These isolate each: bite-checked
    // by dropping one clause from the `&&` and confirming only the test
    // for the *other* operand's claim changes outcome.
    #[test]
    fn first_point_nonzero_time_from_start_is_rejected_at_the_wire_boundary() {
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![point(0.0, 1, 0)],
        };
        assert_err_mentions(
            RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg }),
            "time_from_start is 1s, not 0s",
        );
    }

    #[test]
    fn later_point_with_nonzero_absolute_time_from_start_is_accepted() {
        // Isolates the `i == 0` clause: without it, this point's nonzero
        // *absolute* time_from_start (1s) would be wrongly rejected by the
        // guard above even though its `dt` from point[0] is a valid 1.0s.
        let model = one_joint_model();
        let msg = trajectory_msgs::JointTrajectory {
            header: Default::default(),
            joint_names: vec!["j1".to_string()],
            points: vec![point(0.0, 0, 0), point(0.1, 1, 0)],
        };
        RobotTrajectory::try_from(JointTrajectoryMsg { model: &model, msg })
            .expect("point[1]'s nonzero absolute time_from_start is a valid positive dt");
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

    // PORTING-PLAN.md §172: boundaries of `seconds_to_duration`'s f64 -> i32/u32
    // narrowing, not narrative scenarios -- the truncation/saturation edge and
    // the value just short of it, negative, NaN, infinity, and zero.

    #[test]
    fn seconds_to_duration_accepts_zero() {
        let d = seconds_to_duration(0.0).unwrap();
        assert_eq!((d.sec, d.nanosec), (0, 0));
    }

    #[test]
    fn seconds_to_duration_accepts_i32_max_seconds() {
        let d = seconds_to_duration(i32::MAX as f64).unwrap();
        assert_eq!((d.sec, d.nanosec), (i32::MAX, 0));
    }

    #[test]
    fn seconds_to_duration_rejects_just_above_i32_max_seconds() {
        let err = seconds_to_duration(i32::MAX as f64 + 1.0).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn seconds_to_duration_rejects_negative() {
        let err = seconds_to_duration(-0.001).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn seconds_to_duration_rejects_nan() {
        let err = seconds_to_duration(f64::NAN).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn seconds_to_duration_rejects_infinity() {
        let err = seconds_to_duration(f64::INFINITY).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }

    #[test]
    fn seconds_to_duration_carries_a_rounding_tie_into_seconds() {
        // 5.0 + 0.9999999999s: the fractional part rounds to exactly 1e9
        // nanoseconds, which is not a valid `Duration.nanosec` -- must carry
        // into `sec` rather than emitting `nanosec == 1_000_000_000`.
        let d = seconds_to_duration(5.9999999999).unwrap();
        assert_eq!((d.sec, d.nanosec), (6, 0));
    }

    #[test]
    fn negative_cumulative_duration_from_an_unvalidated_trajectory_is_rejected() {
        // `RobotTrajectory::add_suffix_way_point`'s `dt` has no validation of
        // its own (`moveit-trajectory`) -- a trajectory built directly, not
        // through `JointTrajectoryMsg`'s own `dt < 0.0` check, can carry a
        // negative duration into this crate's msg export boundary.
        let model = one_joint_model();
        let mut traj = RobotTrajectory::new(&model);
        let mut s0 = RobotState::new(&model);
        s0.set_variable_position("j1", 0.0).unwrap();
        traj.add_suffix_way_point(s0, 0.0).unwrap();
        let mut s1 = RobotState::new(&model);
        s1.set_variable_position("j1", 1.0).unwrap();
        traj.add_suffix_way_point(s1, -1.0).unwrap();

        match JointTrajectoryMsgOut::try_from(traj) {
            Err(err) => assert!(matches!(err, Error::Construct(_)), "got: {err:?}"),
            Ok(_) => panic!("expected a negative cumulative duration to be rejected"),
        }
    }
}
