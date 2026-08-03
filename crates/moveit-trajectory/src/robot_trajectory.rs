// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_trajectory/include/moveit/robot_trajectory/robot_trajectory.hpp
//   moveit_core/robot_trajectory/src/robot_trajectory.cpp

//! [`RobotTrajectory`]: a sequence of [`RobotState`] waypoints plus the
//! duration from each waypoint to the previous one.
//!
//! # Out of scope
//!
//! - `getRobotTrajectoryMsg` and all three `setRobotTrajectoryMsg` overloads,
//!   and the free function `toJointTrajectory` — `moveit_msgs`/
//!   `trajectory_msgs` conversions, out of scope per `PORTING-PLAN.md` D1;
//!   they belong in the optional `moveit-ros` crate.
//! - `RobotTrajectory::print` and `operator<<` — upstream's per-waypoint
//!   dump includes velocity and acceleration columns, and this port's
//!   [`RobotState`] carries neither (`moveit-state`'s own scope defers
//!   velocity/acceleration/effort tracking); `#[derive(Debug)]` gives a
//!   structural dump instead.
//! - The hand-rolled `RobotTrajectory::Iterator` class — [`RobotTrajectory::iter`]
//!   is the idiomatic Rust replacement for the `begin()`/`end()` pair it
//!   existed to support.
//!
//! # Deviations from upstream
//!
//! - **No shallow-copy waypoint aliasing.** Upstream's `RobotStatePtr` is a
//!   `shared_ptr`, so `operator=` and the `deepcopy = false` copy
//!   constructor alias waypoints across trajectories. This port's
//!   [`RobotState`] is already a plain, `Clone`-able value type (that choice
//!   was made when `moveit-state` was ported, not here), so `#[derive(Clone)]`
//!   on [`RobotTrajectory`] always deep-copies — there is no cheaper aliasing
//!   mode to preserve, and no `deepcopy: bool` parameter.
//! - **`reverse()` does not invert velocity.** Upstream's `reverse()` calls
//!   `RobotState::invertVelocity()` on every waypoint. This port's
//!   `RobotState` carries no velocity at all, so there is nothing to invert;
//!   this is not a missing feature, it is a consequence of a choice already
//!   made upstream of this crate.
//! - **Unknown group names are a typed error, not a silent whole-robot
//!   fallback.** Upstream's `RobotTrajectory(robot_model, group: string)`
//!   constructor calls `robot_model->getJointModelGroup(group)`, which logs
//!   and returns `nullptr` for an unknown name — the trajectory silently
//!   becomes a whole-robot trajectory. [`RobotTrajectory::for_group_name`]
//!   returns `Err` instead, matching how every other name lookup in this
//!   workspace already behaves (`moveit_error::Error::UnknownName`).
//! - **New invariant: `duration_from_previous[0]` is always `0.0`.** Upstream
//!   stores an arbitrary caller-supplied duration at waypoint 0 (there is no
//!   previous waypoint for it to measure a gap from), and
//!   `getAverageSegmentDuration`'s own "if the initial segment has a duration
//!   of 0, exclude it" comment is a tacit admission that it is expected to be
//!   zero without ever being enforced. This port makes it structurally
//!   impossible to violate: every mutating operation that would place a
//!   value at index 0 either has no `dt` parameter to violate the invariant
//!   with ([`RobotTrajectory::add_prefix_way_point`]) or returns
//!   [`moveit_error::Error`] when the caller supplies a nonzero one
//!   ([`RobotTrajectory::add_suffix_way_point`] on an empty trajectory,
//!   [`RobotTrajectory::insert_way_point`] at index 0,
//!   [`RobotTrajectory::set_way_point_duration_from_previous`] at index 0,
//!   [`RobotTrajectory::append`] onto an empty trajectory).
//! - **Panicking index access becomes `Result`.** Upstream indexes
//!   `waypoints_`/`duration_from_previous_` directly in several places
//!   (`getWayPoint`, `getWayPointPtr`, `getFirstWayPoint`/`getLastWayPoint`,
//!   `removeWayPoint`, `insertWayPoint`) — out-of-range or empty-trajectory
//!   access is undefined behaviour in C++. This port returns
//!   `Result<_, moveit_error::Error>` from all of these instead.
//! - **`getStateAtDurationFromStart` returns a fresh `RobotState` instead of
//!   writing through an out-parameter.** Upstream's `bool
//!   getStateAtDurationFromStart(duration, RobotStatePtr&)` mutates a
//!   caller-supplied state and returns `false` only when the trajectory is
//!   empty. [`RobotTrajectory::state_at_duration_from_start`] returns
//!   `Option<RobotState<'m>>` instead, since a caller here does not need to
//!   pre-construct a scratch state to receive it.

use std::collections::VecDeque;
use std::f64::consts::PI;

use moveit_error::{Error, Result};
use moveit_model::joint::JointModel;
use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::RobotState;

/// A sequence of waypoints (full [`RobotState`]s) and the time duration from
/// each waypoint to the one before it.
///
/// Ported from upstream `robot_trajectory::RobotTrajectory`. If a group is
/// set, only that group's joints are considered "the trajectory's joints" by
/// [`RobotTrajectory::unwind`]; every waypoint still stores a full
/// `RobotState` regardless, matching upstream.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotTrajectory<'m> {
    robot_model: &'m RobotModel,
    group: Option<&'m JointModelGroup>,
    waypoints: VecDeque<RobotState<'m>>,
    duration_from_previous: VecDeque<f64>,
}

impl<'m> RobotTrajectory<'m> {
    /// `RobotTrajectory(robot_model)`: a trajectory for the whole robot.
    pub fn new(robot_model: &'m RobotModel) -> Self {
        Self {
            robot_model,
            group: None,
            waypoints: VecDeque::new(),
            duration_from_previous: VecDeque::new(),
        }
    }

    /// `RobotTrajectory(robot_model, group: const JointModelGroup*)`. `None`
    /// is equivalent to [`RobotTrajectory::new`].
    pub fn for_group(robot_model: &'m RobotModel, group: Option<&'m JointModelGroup>) -> Self {
        Self {
            robot_model,
            group,
            waypoints: VecDeque::new(),
            duration_from_previous: VecDeque::new(),
        }
    }

    /// `RobotTrajectory(robot_model, group: const std::string&)`. An empty
    /// `group` is equivalent to [`RobotTrajectory::new`], matching upstream.
    ///
    /// See the module-level "Deviations from upstream" note: an unknown
    /// non-empty `group` is `Err`, not a silent whole-robot fallback.
    pub fn for_group_name(robot_model: &'m RobotModel, group: &str) -> Result<Self> {
        if group.is_empty() {
            return Ok(Self::new(robot_model));
        }
        let group = robot_model.joint_model_group(group)?;
        Ok(Self::for_group(robot_model, Some(group)))
    }

    /// `getRobotModel`.
    pub fn robot_model(&self) -> &'m RobotModel {
        self.robot_model
    }

    /// `getGroup`.
    pub fn group(&self) -> Option<&'m JointModelGroup> {
        self.group
    }

    /// `getGroupName`: `""` if no group is set.
    pub fn group_name(&self) -> &str {
        self.group.map_or("", JointModelGroup::name)
    }

    /// `setGroupName`. See the module-level "Deviations from upstream" note:
    /// an unknown `group_name` is `Err`, not a silent whole-robot fallback.
    pub fn set_group_name(&mut self, group_name: &str) -> Result<()> {
        self.group = if group_name.is_empty() {
            None
        } else {
            Some(self.robot_model.joint_model_group(group_name)?)
        };
        Ok(())
    }

    /// `getWayPointCount` / `size`.
    pub fn way_point_count(&self) -> usize {
        self.waypoints.len()
    }

    /// `empty`.
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// `getWayPoint`.
    pub fn way_point(&self, index: usize) -> Result<&RobotState<'m>> {
        self.waypoints
            .get(index)
            .ok_or_else(|| Self::index_error(index, self.waypoints.len()))
    }

    /// `getWayPointPtr`.
    pub fn way_point_mut(&mut self, index: usize) -> Result<&mut RobotState<'m>> {
        let len = self.waypoints.len();
        self.waypoints
            .get_mut(index)
            .ok_or_else(|| Self::index_error(index, len))
    }

    /// `getFirstWayPoint`.
    pub fn first_way_point(&self) -> Result<&RobotState<'m>> {
        self.waypoints.front().ok_or_else(Self::empty_error)
    }

    /// `getFirstWayPointPtr`.
    pub fn first_way_point_mut(&mut self) -> Result<&mut RobotState<'m>> {
        self.waypoints.front_mut().ok_or_else(Self::empty_error)
    }

    /// `getLastWayPoint`.
    pub fn last_way_point(&self) -> Result<&RobotState<'m>> {
        self.waypoints.back().ok_or_else(Self::empty_error)
    }

    /// `getLastWayPointPtr`.
    pub fn last_way_point_mut(&mut self) -> Result<&mut RobotState<'m>> {
        self.waypoints.back_mut().ok_or_else(Self::empty_error)
    }

    /// `getWayPointDurations`.
    pub fn way_point_durations(&self) -> &VecDeque<f64> {
        &self.duration_from_previous
    }

    /// `getWayPointDurationFromStart`: clamps `index` to the last waypoint,
    /// matching upstream, rather than erroring.
    pub fn way_point_duration_from_start(&self, index: usize) -> f64 {
        if self.duration_from_previous.is_empty() {
            return 0.0;
        }
        let index = index.min(self.duration_from_previous.len() - 1);
        self.duration_from_previous.iter().take(index + 1).sum()
    }

    /// `getWayPointDurationFromPrevious`: `0.0` if `index` is out of range,
    /// matching upstream.
    pub fn way_point_duration_from_previous(&self, index: usize) -> f64 {
        self.duration_from_previous
            .get(index)
            .copied()
            .unwrap_or(0.0)
    }

    /// `setWayPointDurationFromPrevious`.
    ///
    /// `Err` if `index == 0` and `value != 0.0` — see the module-level
    /// "Deviations from upstream" note on the `duration_from_previous[0]`
    /// invariant.
    pub fn set_way_point_duration_from_previous(&mut self, index: usize, value: f64) -> Result<()> {
        if index == 0 && value != 0.0 {
            return Err(Self::first_duration_error());
        }
        if self.duration_from_previous.len() <= index {
            self.duration_from_previous.resize(index + 1, 0.0);
        }
        self.duration_from_previous[index] = value;
        Ok(())
    }

    /// `addSuffixWayPoint`.
    ///
    /// `Err` if the trajectory is currently empty and `dt != 0.0` — the new
    /// waypoint would become waypoint 0. See the module-level "Deviations
    /// from upstream" note.
    pub fn add_suffix_way_point(&mut self, state: RobotState<'m>, dt: f64) -> Result<&mut Self> {
        if self.waypoints.is_empty() && dt != 0.0 {
            return Err(Self::first_duration_error());
        }
        self.waypoints.push_back(state);
        self.duration_from_previous.push_back(dt);
        Ok(self)
    }

    /// `addPrefixWayPoint`.
    ///
    /// Upstream's `dt` parameter is dropped: the new front waypoint always
    /// becomes waypoint 0, whose duration-from-previous is structurally
    /// `0.0` in this port (see the module-level "Deviations from upstream"
    /// note) — there is no value for a `dt` parameter here to hold.
    pub fn add_prefix_way_point(&mut self, state: RobotState<'m>) -> &mut Self {
        self.waypoints.push_front(state);
        self.duration_from_previous.push_front(0.0);
        self
    }

    /// `insertWayPoint`.
    ///
    /// `Err` if `index > way_point_count()`, or if `index == 0` and
    /// `dt != 0.0` (see the module-level "Deviations from upstream" note).
    pub fn insert_way_point(
        &mut self,
        index: usize,
        state: RobotState<'m>,
        dt: f64,
    ) -> Result<&mut Self> {
        if index > self.waypoints.len() {
            return Err(Self::index_error(index, self.waypoints.len()));
        }
        if index == 0 && dt != 0.0 {
            return Err(Self::first_duration_error());
        }
        self.waypoints.insert(index, state);
        self.duration_from_previous.insert(index, dt);
        Ok(self)
    }

    /// `removeWayPoint`.
    pub fn remove_way_point(&mut self, index: usize) -> Result<&mut Self> {
        if index >= self.waypoints.len() {
            return Err(Self::index_error(index, self.waypoints.len()));
        }
        self.waypoints.remove(index);
        self.duration_from_previous.remove(index);
        Ok(self)
    }

    /// `clear`.
    pub fn clear(&mut self) -> &mut Self {
        self.waypoints.clear();
        self.duration_from_previous.clear();
        self
    }

    /// `append`. `start_index`/`end_index` select the half-open range
    /// `[start_index, end_index)` of `source`'s waypoints to append, matching
    /// upstream; `end_index` is clamped to `source`'s length and an empty
    /// range is a silent no-op, both matching upstream.
    ///
    /// `Err` if `self` is currently empty and `dt != 0.0` — the first
    /// appended waypoint would become waypoint 0 (see the module-level
    /// "Deviations from upstream" note).
    pub fn append(
        &mut self,
        source: &Self,
        dt: f64,
        start_index: usize,
        end_index: usize,
    ) -> Result<&mut Self> {
        let end_index = end_index.min(source.waypoints.len());
        if start_index >= end_index {
            return Ok(self);
        }
        if self.waypoints.is_empty() && dt != 0.0 {
            return Err(Self::first_duration_error());
        }
        for i in start_index..end_index {
            self.waypoints.push_back(source.waypoints[i].clone());
        }
        let insert_pos = self.duration_from_previous.len();
        for i in start_index..end_index {
            self.duration_from_previous
                .push_back(source.duration_from_previous[i]);
        }
        if self.duration_from_previous.len() > insert_pos {
            self.duration_from_previous[insert_pos] = dt;
        }
        Ok(self)
    }

    /// `getDuration`.
    pub fn duration(&self) -> f64 {
        self.duration_from_previous.iter().sum()
    }

    /// `getAverageSegmentDuration`.
    ///
    /// Upstream branches on whether `duration_from_previous[0] == 0`; in
    /// this port that is always true (see the module-level "Deviations from
    /// upstream" note), so only that branch is reachable. The shape is kept
    /// so the two stay easy to compare.
    pub fn average_segment_duration(&self) -> f64 {
        if self.duration_from_previous.is_empty() {
            return 0.0;
        }
        debug_assert_eq!(self.duration_from_previous[0], 0.0);
        if self.duration_from_previous.len() <= 1 {
            return 0.0;
        }
        self.duration() / (self.duration_from_previous.len() - 1) as f64
    }

    /// `reverse`.
    ///
    /// See the module-level "Deviations from upstream" note: this does not
    /// invert velocity, since this port's `RobotState` carries none.
    pub fn reverse(&mut self) -> &mut Self {
        let reversed: VecDeque<_> = self.waypoints.drain(..).rev().collect();
        self.waypoints = reversed;

        if let Some(&first) = self.duration_from_previous.front() {
            self.duration_from_previous.push_back(first);
            let reversed: VecDeque<_> = self.duration_from_previous.drain(..).rev().collect();
            self.duration_from_previous = reversed;
            self.duration_from_previous.pop_back();
        }

        self
    }

    /// `unwind()`: unwrap every continuous joint's positions across
    /// waypoints so consecutive waypoints never jump by more than `PI`
    /// (before the running unwrap offset is added back in), instead of
    /// wrapping at the joint's `[-PI, PI]` bounds.
    pub fn unwind(&mut self) -> &mut Self {
        self.unwind_impl(None);
        self
    }

    /// `unwind(const RobotState& state)`: like [`RobotTrajectory::unwind`],
    /// but the first waypoint is unwound relative to `state` instead of to
    /// its own bounds-enforced value.
    pub fn unwind_from(&mut self, state: &RobotState<'m>) -> &mut Self {
        self.unwind_impl(Some(state));
        self
    }

    fn unwind_impl(&mut self, reference: Option<&RobotState<'m>>) {
        if self.waypoints.is_empty() {
            return;
        }

        for joint in self.continuous_joint_models() {
            let name = joint.name();

            let (mut running_offset, mut last_value) = match reference {
                None => {
                    let mut value = self.waypoints[0]
                        .joint_position(name)
                        .expect("continuous joint of this trajectory's own robot model")[0];
                    joint.enforce_position_bounds(std::slice::from_mut(&mut value));
                    self.waypoints[0]
                        .set_joint_positions(name, &[value])
                        .expect("continuous joint of this trajectory's own robot model");
                    (0.0, value)
                }
                Some(state) => {
                    let reference_value0 = state
                        .joint_position(name)
                        .expect("continuous joint of this trajectory's own robot model")[0];
                    let mut reference_value = reference_value0;
                    joint.enforce_position_bounds(std::slice::from_mut(&mut reference_value));
                    let mut offset = reference_value0 - reference_value;

                    let mut value = self.waypoints[0]
                        .joint_position(name)
                        .expect("continuous joint of this trajectory's own robot model")[0];
                    joint.enforce_position_bounds(std::slice::from_mut(&mut value));
                    if value > reference_value + PI {
                        offset -= 2.0 * PI;
                    } else if value < reference_value - PI {
                        offset += 2.0 * PI;
                    }
                    let start_value = value + offset;
                    self.waypoints[0]
                        .set_joint_positions(name, &[start_value])
                        .expect("continuous joint of this trajectory's own robot model");
                    (offset, value)
                }
            };

            for j in 1..self.waypoints.len() {
                let mut current_value = self.waypoints[j]
                    .joint_position(name)
                    .expect("continuous joint of this trajectory's own robot model")[0];
                joint.enforce_position_bounds(std::slice::from_mut(&mut current_value));
                if last_value > current_value + PI {
                    running_offset += 2.0 * PI;
                } else if current_value > last_value + PI {
                    running_offset -= 2.0 * PI;
                }
                last_value = current_value;
                let unwound = current_value + running_offset;
                self.waypoints[j]
                    .set_joint_positions(name, &[unwound])
                    .expect("continuous joint of this trajectory's own robot model");
            }
        }

        for waypoint in &mut self.waypoints {
            waypoint.update();
        }
    }

    /// `group_->getContinuousJointModels()` / `robot_model_->getContinuousJointModels()`.
    fn continuous_joint_models(&self) -> Vec<&'m JointModel> {
        match self.group {
            Some(group) => group
                .joint_indices()
                .iter()
                .map(|&i| self.robot_model.joint_model_at(i))
                .filter(|joint| joint.as_revolute().is_some_and(|r| r.is_continuous()))
                .collect(),
            None => self
                .robot_model
                .joint_models()
                .filter(|joint| joint.as_revolute().is_some_and(|r| r.is_continuous()))
                .collect(),
        }
    }

    /// `findWayPointIndicesForDurationAfterStart`. Returns
    /// `(before, after, blend)`; see upstream's doc comment for the edge
    /// cases (empty trajectory, negative duration, duration past the total,
    /// single-waypoint trajectory), all reproduced by this implementation.
    pub fn find_way_point_indices_for_duration_after_start(
        &self,
        duration: f64,
    ) -> (usize, usize, f64) {
        if duration < 0.0 || self.waypoints.is_empty() {
            return (0, 0, 0.0);
        }

        let num_points = self.waypoints.len();
        let mut index = 0;
        let mut running_duration = 0.0;
        while index < num_points {
            running_duration += self.duration_from_previous[index];
            if running_duration >= duration {
                break;
            }
            index += 1;
        }
        let before = index.saturating_sub(1);
        let after = index.min(num_points - 1);

        let blend = if after == before {
            1.0
        } else {
            let before_time = running_duration - self.duration_from_previous[index];
            (duration - before_time) / self.duration_from_previous[index]
        };

        (before, after, blend)
    }

    /// `getStateAtDurationFromStart`. `None` if the trajectory is empty
    /// (upstream returns `false`); see the module-level "Deviations from
    /// upstream" note on the return-value shape.
    pub fn state_at_duration_from_start(&self, request_duration: f64) -> Option<RobotState<'m>> {
        if self.waypoints.is_empty() {
            return None;
        }
        let (before, after, blend) =
            self.find_way_point_indices_for_duration_after_start(request_duration);
        let mut state = self.waypoints[before].clone();
        Self::interpolate_into(
            &self.waypoints[before],
            &self.waypoints[after],
            blend,
            &mut state,
        );
        Some(state)
    }

    /// `RobotState::interpolate(to, t, state)` (the group-less overload
    /// `getStateAtDurationFromStart` uses): linearly interpolate every
    /// active (non-mimic, non-fixed) joint of the whole robot model between
    /// `from` and `to`, writing the result into `out`.
    fn interpolate_into(
        from: &RobotState<'m>,
        to: &RobotState<'m>,
        t: f64,
        out: &mut RobotState<'m>,
    ) {
        let model = from.model();
        for &index in model.active_joint_indices() {
            let joint = model.joint_model_at(index);
            if joint.variable_count() == 0 {
                continue;
            }
            let from_values = from
                .joint_position(joint.name())
                .expect("active joint of this trajectory's own robot model");
            let to_values = to
                .joint_position(joint.name())
                .expect("active joint of this trajectory's own robot model");
            let mut buffer = vec![0.0; joint.variable_count()];
            joint.interpolate(from_values, to_values, t, &mut buffer);
            out.set_joint_positions(joint.name(), &buffer)
                .expect("active joint of this trajectory's own robot model");
        }
    }

    /// `RobotState::distance(other)` (the group-less overload `pathLength`
    /// and `smoothness` use): the L1 sum of active-joint distances between
    /// two states of the whole robot model.
    fn distance(a: &RobotState<'m>, b: &RobotState<'m>) -> f64 {
        let model = a.model();
        model
            .active_joint_indices()
            .iter()
            .map(|&index| {
                let joint = model.joint_model_at(index);
                let a_values = a
                    .joint_position(joint.name())
                    .expect("active joint of this trajectory's own robot model");
                let b_values = b
                    .joint_position(joint.name())
                    .expect("active joint of this trajectory's own robot model");
                joint.distance_factor() * joint.distance(a_values, b_values)
            })
            .sum()
    }

    /// `begin()`/`end()`, replacing the hand-rolled `RobotTrajectory::Iterator`
    /// (see the module-level "Out of scope" note) with the standard iterator
    /// adaptor a Rust caller expects.
    pub fn iter(&self) -> impl Iterator<Item = (&RobotState<'m>, f64)> {
        self.waypoints
            .iter()
            .zip(self.duration_from_previous.iter().copied())
    }

    fn index_error(index: usize, len: usize) -> Error {
        Error::other(format!(
            "waypoint index {index} out of bounds (trajectory has {len} waypoints)"
        ))
    }

    fn empty_error() -> Error {
        Error::other("trajectory has no waypoints")
    }

    fn first_duration_error() -> Error {
        Error::other(
            "duration_from_previous[0] must be 0.0: the first waypoint has no previous waypoint \
             to measure a duration from",
        )
    }
}

/// `pathLength`: the sum of consecutive-waypoint distances (an L1 norm over
/// active joints).
#[must_use]
pub fn path_length(trajectory: &RobotTrajectory<'_>) -> f64 {
    let mut length = 0.0;
    for i in 1..trajectory.waypoints.len() {
        length += RobotTrajectory::distance(&trajectory.waypoints[i - 1], &trajectory.waypoints[i]);
    }
    length
}

/// `smoothness`: `None` if the trajectory has two or fewer waypoints
/// (matching upstream's "path is too short" case).
#[must_use]
pub fn smoothness(trajectory: &RobotTrajectory<'_>) -> Option<f64> {
    if trajectory.waypoints.len() <= 2 {
        return None;
    }

    let mut total = 0.0;
    let mut a = RobotTrajectory::distance(&trajectory.waypoints[0], &trajectory.waypoints[1]);
    for k in 2..trajectory.waypoints.len() {
        let b = RobotTrajectory::distance(&trajectory.waypoints[k - 1], &trajectory.waypoints[k]);
        let c = RobotTrajectory::distance(&trajectory.waypoints[k - 2], &trajectory.waypoints[k]);
        let acos_value = (a * a + b * b - c * c) / (2.0 * a * b);
        if acos_value > -1.0 && acos_value < 1.0 {
            let angle = PI - acos_value.acos();
            let u = 2.0 * angle;
            total += u * u;
        }
        a = b;
    }
    Some(total / trajectory.waypoints.len() as f64)
}

/// `waypointDensity`: `None` if the trajectory has fewer than two waypoints
/// or a zero path length, matching upstream.
#[must_use]
pub fn waypoint_density(trajectory: &RobotTrajectory<'_>) -> Option<f64> {
    if trajectory.waypoints.len() <= 1 {
        return None;
    }
    let length = path_length(trajectory);
    if length > 0.0 {
        Some(trajectory.waypoints.len() as f64 / length)
    } else {
        None
    }
}
