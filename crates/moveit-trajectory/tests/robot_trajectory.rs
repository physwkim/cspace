// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_trajectory/test/test_robot_trajectory.cpp
//
// Not ported (see UNFIXED in the task report): `RobotTrajectoryShallowCopy`
// (tests shared_ptr waypoint aliasing this port deliberately does not have —
// see `robot_trajectory.rs`'s "Deviations from upstream"), `ChainEdits` and
// `DoubleReverse` (adapted below to drop the `setRobotTrajectoryMsg`/
// `getRobotTrajectoryMsg` steps, D1), `MultiDofTrajectoryToJointStates`
// (`toJointTrajectory`, D1), `SetMultiDofTrajectory` (`setRobotTrajectoryMsg`
// plus velocity/acceleration, both out of scope).

//! Ported `test_robot_trajectory.cpp` cases, plus boundary tests for the new
//! `duration_from_previous[0] == 0.0` invariant and for typed-error index
//! access.

use std::fs;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use moveit_trajectory::robot_trajectory::{path_length, smoothness, waypoint_density};

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        file_name
    )
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let urdf_path = fixture_path(urdf_file);
    let srdf_path = fixture_path(srdf_file);
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn panda() -> RobotModel {
    build_model("panda.urdf", "panda.srdf")
}

fn pr2() -> RobotModel {
    build_model("pr2.urdf", "pr2.srdf")
}

const ARM_GROUP: &str = "panda_arm";
const DT: f64 = 0.1;
const WAYPOINT_COUNT: usize = 5;

/// `RobotTrajectoryTestFixture::initTestTrajectory`.
fn init_test_trajectory(model: &RobotModel) -> RobotTrajectory<'_> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();

    let mut trajectory = RobotTrajectory::for_group_name(model, ARM_GROUP).unwrap();
    assert_eq!(trajectory.group_name(), ARM_GROUP);
    assert!(trajectory.is_empty());

    for i in 0..WAYPOINT_COUNT {
        let dt = if i == 0 { 0.0 } else { DT };
        trajectory.add_suffix_way_point(state.clone(), dt).unwrap();
    }

    // Upstream's fixture pushes `duration_from_previous` (0.1) unconditionally,
    // including at waypoint 0; this port's structural invariant forces
    // waypoint 0 to 0.0 instead (see robot_trajectory.rs's "Deviations from
    // upstream"), so the expected total is one `DT` short of upstream's
    // `duration_from_previous * waypoint_count`.
    assert_eq!(trajectory.duration(), DT * (WAYPOINT_COUNT - 1) as f64);
    assert_eq!(trajectory.way_point_durations().len(), WAYPOINT_COUNT);
    assert_eq!(trajectory.way_point_count(), WAYPOINT_COUNT);

    trajectory
}

// ---- ModifyFirstWaypointByPtr / ModifyFirstWaypointByValue -----------------

#[test]
fn modify_first_waypoint_by_ptr_updates_the_trajectory() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);

    let before = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    let waypoint = trajectory.way_point_mut(0).unwrap();
    waypoint
        .set_variable_position("panda_joint1", before + 0.01)
        .unwrap();

    let after = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    assert_eq!(after, before + 0.01);

    let before_dt = trajectory.way_point_duration_from_previous(0);
    trajectory
        .set_way_point_duration_from_previous(0, before_dt)
        .unwrap();
    assert_eq!(trajectory.way_point_duration_from_previous(0), before_dt);
}

#[test]
fn modify_first_waypoint_by_value_does_not_update_the_trajectory() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);

    let mut copy = trajectory.way_point(0).unwrap().clone();
    let before = copy.variable_position("panda_joint1").unwrap();
    copy.set_variable_position("panda_joint1", before + 0.01)
        .unwrap();

    let still_original = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    assert_ne!(still_original, before + 0.01);
    assert_eq!(still_original, before);
}

// ---- DoubleReverse (adapted: compare trajectories directly, not via msg) --

#[test]
fn double_reverse_restores_the_original_trajectory() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);
    let initial = trajectory.clone();

    let mut edited = trajectory;
    edited.reverse().reverse();

    assert_eq!(initial, edited);
}

// ---- ChainEdits (adapted: no setRobotTrajectoryMsg round trip) ------------

#[test]
fn chained_edits_compose_left_to_right() {
    let model = panda();
    let initial = init_test_trajectory(&model);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.set_group_name(ARM_GROUP).unwrap();
    trajectory
        .clear()
        .append(&initial, 0.0, 0, initial.way_point_count())
        .unwrap()
        .reverse()
        .add_suffix_way_point(state.clone(), DT)
        .unwrap()
        .add_prefix_way_point(state.clone())
        .insert_way_point(1, state.clone(), DT)
        .unwrap()
        .append(&initial, DT, 0, initial.way_point_count())
        .unwrap();

    assert_eq!(trajectory.group_name(), ARM_GROUP);
    assert_eq!(
        trajectory.way_point_count(),
        initial.way_point_count() * 2 + 3
    );
}

// ---- Append -----------------------------------------------------------

#[test]
fn append_inserts_source_waypoints_and_overwrites_the_join_duration() {
    let model = panda();
    let mut initial = init_test_trajectory(&model);
    let traj2 = init_test_trajectory(&model);
    assert_eq!(initial.way_point_count(), 5);
    assert_eq!(traj2.way_point_count(), 5);

    let expected_duration = 0.1;
    initial.append(&traj2, expected_duration, 0, 5).unwrap();
    assert_eq!(initial.way_point_count(), 10);

    assert_eq!(
        initial.way_point_duration_from_previous(4),
        expected_duration
    );
    assert_eq!(
        initial.way_point_duration_from_previous(5),
        expected_duration
    );
    assert_eq!(
        initial.way_point_duration_from_previous(6),
        expected_duration
    );
}

// ---- RobotTrajectoryDeepCopy (Clone is always a deep copy) ----------------

#[test]
fn clone_is_always_independent_of_the_original() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    let copy = trajectory.clone();
    assert_eq!(copy.duration(), trajectory.duration());
    assert_eq!(
        copy.way_point_durations().len(),
        trajectory.way_point_durations().len()
    );

    let before = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    trajectory
        .way_point_mut(0)
        .unwrap()
        .set_variable_position("panda_joint1", before + 0.01)
        .unwrap();
    trajectory
        .set_way_point_duration_from_previous(1, 0.5)
        .unwrap();

    let original_after = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    let copy_after = copy
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    assert_ne!(original_after, copy_after);
    assert_ne!(
        trajectory.way_point_duration_from_previous(1),
        copy.way_point_duration_from_previous(1)
    );
}

// ---- RobotTrajectoryIterator (adapted to .iter()) --------------------------

#[test]
fn iter_yields_every_waypoint_and_its_duration_in_order() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    assert_eq!(trajectory.way_point_count(), 5);

    let start_pos = trajectory
        .way_point(0)
        .unwrap()
        .variable_position("panda_joint1")
        .unwrap();
    for i in 0..trajectory.way_point_count() {
        let waypoint = trajectory.way_point_mut(i).unwrap();
        let value = waypoint.variable_position("panda_joint1").unwrap();
        waypoint
            .set_variable_position("panda_joint1", value + 0.01 * i as f64)
            .unwrap();
    }

    let mut count = 0;
    for (waypoint, _dt) in trajectory.iter() {
        let position = waypoint.variable_position("panda_joint1").unwrap();
        assert_eq!(position, start_pos + count as f64 * 0.01);
        count += 1;
    }
    assert_eq!(count, trajectory.way_point_count());
}

// ---- RobotTrajectoryLength / Smoothness / Density --------------------------

fn perturb_first_joint(trajectory: &mut RobotTrajectory<'_>) {
    for i in 0..trajectory.way_point_count() {
        let waypoint = trajectory.way_point_mut(i).unwrap();
        let value = waypoint.variable_position("panda_joint1").unwrap();
        waypoint
            .set_variable_position("panda_joint1", value + 0.01 * i as f64)
            .unwrap();
    }
}

#[test]
fn path_length_is_zero_for_identical_waypoints_and_positive_after_perturbation() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    assert_eq!(path_length(&trajectory), 0.0);

    perturb_first_joint(&mut trajectory);
    assert!(path_length(&trajectory) > 0.0);
}

#[test]
fn smoothness_is_some_positive_value_and_none_when_empty() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    perturb_first_joint(&mut trajectory);

    let value = smoothness(&trajectory);
    assert!(value.is_some());
    assert!(value.unwrap() > 0.0);

    trajectory.clear();
    assert_eq!(smoothness(&trajectory), None);
}

#[test]
fn waypoint_density_is_none_at_zero_length_and_some_after_perturbation() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    assert_eq!(waypoint_density(&trajectory), None);

    perturb_first_joint(&mut trajectory);
    let density = waypoint_density(&trajectory);
    assert!(density.is_some());
    assert!(density.unwrap() > 0.0);

    trajectory.clear();
    assert_eq!(waypoint_density(&trajectory), None);
}

// ---- findWayPointIndicesForDurationAfterStart edge cases -------------------

#[test]
fn find_way_point_indices_between_waypoints() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);
    assert_eq!(trajectory.way_point_count(), 5);
    assert_eq!(trajectory.duration(), 0.4);

    let (before, after, blend) = trajectory.find_way_point_indices_for_duration_after_start(0.15);
    assert_eq!(before, 1);
    assert_eq!(after, 2);
    assert!((blend - 0.5).abs() < 1e-6);
}

#[test]
fn find_way_point_indices_at_last_of_many_waypoints() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);
    let total = trajectory.duration();
    let (before, after, blend) = trajectory.find_way_point_indices_for_duration_after_start(total);
    assert_eq!(before, 3);
    assert_eq!(after, 4);
    // `blend`'s subtraction (`running_duration - duration_from_previous[index]`)
    // is not exact here: upstream's own `EXPECT_DOUBLE_EQ` tolerates the same
    // few-ULP rounding this assertion allows for.
    assert!((blend - 1.0).abs() < 1e-9, "blend = {blend}");
}

#[test]
fn find_way_point_indices_after_last_waypoint() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);
    let total = trajectory.duration();
    let (before, after, blend) =
        trajectory.find_way_point_indices_for_duration_after_start(total + 100.0);
    assert_eq!(before, 4);
    assert_eq!(after, 4);
    assert_eq!(blend, 1.0);
}

#[test]
fn find_way_point_indices_empty_trajectory() {
    let model = panda();
    let trajectory = RobotTrajectory::for_group_name(&model, ARM_GROUP).unwrap();
    assert_eq!(trajectory.duration(), 0.0);
    let (before, after, blend) = trajectory.find_way_point_indices_for_duration_after_start(1.0);
    assert_eq!(before, 0);
    assert_eq!(after, 0);
    assert_eq!(blend, 0.0);
}

#[test]
fn find_way_point_indices_before_first_waypoint() {
    let model = panda();
    let trajectory = init_test_trajectory(&model);
    let (before, after, blend) = trajectory.find_way_point_indices_for_duration_after_start(-0.1);
    assert_eq!(before, 0);
    assert_eq!(after, 0);
    assert_eq!(blend, 0.0);
}

#[test]
fn find_way_point_indices_at_last_of_single_waypoint() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::for_group_name(&model, ARM_GROUP).unwrap();
    trajectory.add_suffix_way_point(state, 0.0).unwrap();

    let total = trajectory.duration();
    let (before, after, blend) = trajectory.find_way_point_indices_for_duration_after_start(total);
    assert_eq!(before, 0);
    assert_eq!(after, 0);
    assert_eq!(blend, 1.0);
}

// ---- Unwind / UnwindFromState (pr2, whole-robot: `fl_caster_rotation_joint`) --

#[test]
fn unwind_wraps_a_large_initial_continuous_joint_angle() {
    const EPSILON: f64 = 1e-4;
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let mut trajectory = RobotTrajectory::new(&model);
    for i in 0..WAYPOINT_COUNT {
        let dt = if i == 0 { 0.0 } else { DT };
        trajectory.add_suffix_way_point(state.clone(), dt).unwrap();
    }

    let random_large_angle = 20.2; // rad, should unwind to 1.350444 rad
    trajectory
        .first_way_point_mut()
        .unwrap()
        .set_variable_position("fl_caster_rotation_joint", random_large_angle)
        .unwrap();
    trajectory.unwind();

    let unwound = trajectory
        .first_way_point()
        .unwrap()
        .variable_position("fl_caster_rotation_joint")
        .unwrap();
    assert!((unwound - 1.350444).abs() < EPSILON, "unwound = {unwound}");
}

#[test]
fn unwind_from_state_unwinds_relative_to_the_reference_state() {
    const EPSILON: f64 = 1e-4;
    let model = pr2();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let mut trajectory = RobotTrajectory::new(&model);
    for i in 0..WAYPOINT_COUNT {
        let dt = if i == 0 { 0.0 } else { DT };
        trajectory.add_suffix_way_point(state.clone(), dt).unwrap();
    }

    let mut reference = trajectory.first_way_point().unwrap().clone();
    let base = reference
        .variable_position("fl_caster_rotation_joint")
        .unwrap();
    let wrapped_angle = base + 12.566_371; // +4*pi, as if the live robot wound up
    reference
        .set_variable_position("fl_caster_rotation_joint", wrapped_angle)
        .unwrap();

    trajectory.unwind_from(&reference);

    let unwound = trajectory
        .first_way_point()
        .unwrap()
        .variable_position("fl_caster_rotation_joint")
        .unwrap();
    assert!(
        (unwound - wrapped_angle).abs() < EPSILON,
        "unwound = {unwound}"
    );
}

// ---- Boundary tests: the new duration_from_previous[0] == 0.0 invariant ---

#[test]
fn add_suffix_way_point_on_an_empty_trajectory_rejects_a_nonzero_dt() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::new(&model);

    assert!(trajectory.add_suffix_way_point(state.clone(), 0.1).is_err());
    assert!(trajectory.add_suffix_way_point(state, 0.0).is_ok());
}

#[test]
fn add_prefix_way_point_always_lands_at_a_zero_duration() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();
    trajectory.add_suffix_way_point(state.clone(), 0.5).unwrap();

    trajectory.add_prefix_way_point(state);
    assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
    // the waypoint that used to be index 0 (dt 0.0) is now index 1, unchanged
    assert_eq!(trajectory.way_point_duration_from_previous(1), 0.0);
}

#[test]
fn insert_way_point_at_zero_rejects_a_nonzero_dt() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();

    assert!(trajectory.insert_way_point(0, state.clone(), 0.2).is_err());
    assert!(trajectory.insert_way_point(0, state, 0.0).is_ok());
}

#[test]
fn set_way_point_duration_from_previous_at_zero_rejects_a_nonzero_value() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.add_suffix_way_point(state, 0.0).unwrap();

    assert!(
        trajectory
            .set_way_point_duration_from_previous(0, 0.3)
            .is_err()
    );
    assert!(
        trajectory
            .set_way_point_duration_from_previous(0, 0.0)
            .is_ok()
    );
}

#[test]
fn append_onto_an_empty_trajectory_rejects_a_nonzero_dt() {
    let model = panda();
    let source = init_test_trajectory(&model);
    let mut empty = RobotTrajectory::new(&model);

    assert!(
        empty
            .append(&source, 0.2, 0, source.way_point_count())
            .is_err()
    );
    assert!(
        empty
            .append(&source, 0.0, 0, source.way_point_count())
            .is_ok()
    );
}

// ---- Boundary tests: empty / single waypoint --------------------------

#[test]
fn empty_trajectory_accessors_return_typed_errors_not_panics() {
    let model = panda();
    let trajectory = RobotTrajectory::new(&model);
    assert!(trajectory.way_point(0).is_err());
    assert!(trajectory.first_way_point().is_err());
    assert!(trajectory.last_way_point().is_err());
    assert_eq!(trajectory.duration(), 0.0);
    assert_eq!(trajectory.average_segment_duration(), 0.0);
}

#[test]
fn single_waypoint_trajectory_average_segment_duration_is_zero() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.add_suffix_way_point(state, 0.0).unwrap();
    assert_eq!(trajectory.average_segment_duration(), 0.0);
}

#[test]
fn reverse_on_an_empty_trajectory_is_a_no_op() {
    let model = panda();
    let mut trajectory = RobotTrajectory::new(&model);
    trajectory.reverse();
    assert!(trajectory.is_empty());
}

// ---- Boundary tests: out-of-range index access -----------------------

#[test]
fn out_of_range_index_access_is_a_typed_error() {
    let model = panda();
    let mut trajectory = init_test_trajectory(&model);
    let len = trajectory.way_point_count();

    assert!(trajectory.way_point(len).is_err());
    assert!(trajectory.way_point_mut(len).is_err());
    assert!(trajectory.remove_way_point(len).is_err());

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    assert!(trajectory.insert_way_point(len + 1, state, 0.1).is_err());
    // exactly one past the end is a valid insertion point (matches Vec::insert)
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    assert!(trajectory.insert_way_point(len, state, 0.1).is_ok());
}

// ---- Boundary tests: unknown group name --------------------------------

#[test]
fn unknown_group_name_is_a_typed_error_not_a_silent_whole_robot_fallback() {
    let model = panda();
    assert!(RobotTrajectory::for_group_name(&model, "no_such_group").is_err());
    assert!(RobotTrajectory::for_group_name(&model, "").is_ok());
}
