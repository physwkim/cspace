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
//!   they belong in the optional `cspace-ros` crate.
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
//!   was made when `cspace_core::state` was ported, not here), so `#[derive(Clone)]`
//!   on [`RobotTrajectory`] always deep-copies — there is no cheaper aliasing
//!   mode to preserve, and no `deepcopy: bool` parameter.
//! - **Unknown group names are a typed error, not a silent whole-robot
//!   fallback.** Upstream's `RobotTrajectory(robot_model, group: string)`
//!   constructor calls `robot_model->getJointModelGroup(group)`, which logs
//!   and returns `nullptr` for an unknown name — the trajectory silently
//!   becomes a whole-robot trajectory. [`RobotTrajectory::for_group_name`]
//!   returns `Err` instead, matching how every other name lookup in this
//!   workspace already behaves (`crate::error::Error::UnknownName`).
//! - **New invariant: `duration_from_previous[0]` is always `0.0`.** Upstream
//!   stores an arbitrary caller-supplied duration at waypoint 0 (there is no
//!   previous waypoint for it to measure a gap from), and
//!   `getAverageSegmentDuration`'s own "if the initial segment has a duration
//!   of 0, exclude it" comment is a tacit admission that it is expected to be
//!   zero without ever being enforced. This port makes it structurally
//!   impossible to violate: every mutating operation that would place a
//!   value at index 0 either has no `dt` parameter to violate the invariant
//!   with ([`RobotTrajectory::add_prefix_way_point`]) or returns
//!   [`crate::error::Error`] when the caller supplies a nonzero one
//!   ([`RobotTrajectory::add_suffix_way_point`] on an empty trajectory,
//!   [`RobotTrajectory::insert_way_point`] at index 0,
//!   [`RobotTrajectory::set_way_point_duration_from_previous`] at index 0,
//!   [`RobotTrajectory::append`] onto an empty trajectory), or resets the new
//!   index 0 to `0.0` itself ([`RobotTrajectory::remove_way_point`] on
//!   waypoint 0).
//! - **Panicking index access becomes `Result`.** Upstream indexes
//!   `waypoints_`/`duration_from_previous_` directly in several places
//!   (`getWayPoint`, `getWayPointPtr`, `getFirstWayPoint`/`getLastWayPoint`,
//!   `removeWayPoint`, `insertWayPoint`) — out-of-range or empty-trajectory
//!   access is undefined behaviour in C++. This port returns
//!   `Result<_, crate::error::Error>` from all of these instead.
//! - **`getStateAtDurationFromStart` returns a fresh `RobotState` instead of
//!   writing through an out-parameter.** Upstream's `bool
//!   getStateAtDurationFromStart(duration, RobotStatePtr&)` mutates a
//!   caller-supplied state and returns `false` only when the trajectory is
//!   empty. [`RobotTrajectory::state_at_duration_from_start`] returns
//!   `Option<RobotState<'m>>` instead, since a caller here does not need to
//!   pre-construct a scratch state to receive it.
//! - **`Display` has no `variable_indexes` override parameter.** Upstream's
//!   `print(std::ostream&, std::vector<int> variable_indexes = {})` lets a
//!   caller pass an explicit column subset; `std::fmt::Display::fmt`'s
//!   signature is fixed by the trait, so there is nowhere to thread that
//!   argument through. The `Display` impl always takes upstream's *default*
//!   branch (`variable_indexes.empty()`): the group's own variables if a
//!   group is set, else every model variable (see the private
//!   `print_variable_names` helper below). Upstream's actual per-line column
//!   order is waypoint index, time, position, velocity (if present),
//!   acceleration (if present), effort (if present); this matches it.

use std::collections::VecDeque;
use std::f64::consts::PI;

use crate::error::{Error, Result};
use crate::model::joint::JointModel;
use crate::model::{JointModelGroup, RobotModel};
use crate::state::RobotState;

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
    ///
    /// Calls [`RobotState::update`] on `state` before storing it, matching
    /// upstream's `state->update()`: [`RobotState`] derives `PartialEq` over
    /// its cached transforms and dirty-subtree bookkeeping, so a waypoint
    /// stored without settling those first would not compare equal to the
    /// same logical state stored after an explicit `update()` elsewhere.
    pub fn add_suffix_way_point(
        &mut self,
        mut state: RobotState<'m>,
        dt: f64,
    ) -> Result<&mut Self> {
        if self.waypoints.is_empty() && dt != 0.0 {
            return Err(Self::first_duration_error());
        }
        state.update();
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
    ///
    /// Calls [`RobotState::update`] on `state` before storing it, matching
    /// upstream's `state->update()`; see [`RobotTrajectory::add_suffix_way_point`]'s
    /// doc comment for why.
    pub fn add_prefix_way_point(&mut self, mut state: RobotState<'m>) -> &mut Self {
        state.update();
        self.waypoints.push_front(state);
        self.duration_from_previous.push_front(0.0);
        self
    }

    /// `insertWayPoint`.
    ///
    /// `Err` if `index > way_point_count()`, or if `index == 0` and
    /// `dt != 0.0` (see the module-level "Deviations from upstream" note).
    ///
    /// Calls [`RobotState::update`] on `state` before storing it, matching
    /// upstream's `state->update()`; see [`RobotTrajectory::add_suffix_way_point`]'s
    /// doc comment for why.
    pub fn insert_way_point(
        &mut self,
        index: usize,
        mut state: RobotState<'m>,
        dt: f64,
    ) -> Result<&mut Self> {
        if index > self.waypoints.len() {
            return Err(Self::index_error(index, self.waypoints.len()));
        }
        if index == 0 && dt != 0.0 {
            return Err(Self::first_duration_error());
        }
        state.update();
        self.waypoints.insert(index, state);
        self.duration_from_previous.insert(index, dt);
        Ok(self)
    }

    /// `removeWayPoint`.
    ///
    /// Removing waypoint 0 makes the former waypoint 1 the new waypoint 0,
    /// which by the module-level "Deviations from upstream" note has no
    /// previous waypoint of its own -- so its `duration_from_previous` is
    /// reset to `0.0`, the same value every other route to becoming waypoint
    /// 0 produces, rather than keeping the real (but now meaningless) gap
    /// that used to separate it from the just-removed waypoint.
    pub fn remove_way_point(&mut self, index: usize) -> Result<&mut Self> {
        if index >= self.waypoints.len() {
            return Err(Self::index_error(index, self.waypoints.len()));
        }
        self.waypoints.remove(index);
        self.duration_from_previous.remove(index);
        if index == 0 {
            if let Some(new_first) = self.duration_from_previous.front_mut() {
                *new_first = 0.0;
            }
        }
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
    /// Inverts every waypoint's velocity (via `RobotState::invert_velocity`,
    /// a no-op where no velocity was ever set) as it swaps waypoint order,
    /// matching upstream's `waypoint->invertVelocity()` call. Acceleration
    /// is deliberately left untouched: upstream's `invertVelocity` only
    /// negates velocity, not acceleration, despite the method living inside
    /// `reverse()` — see `RobotState::invert_velocity`'s doc comment.
    pub fn reverse(&mut self) -> &mut Self {
        let reversed: VecDeque<_> = self.waypoints.drain(..).rev().collect();
        self.waypoints = reversed;
        for waypoint in &mut self.waypoints {
            waypoint.invert_velocity();
        }

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

    /// The variable name list `print`'s `Display` impl walks per waypoint:
    /// upstream's `variable_indexes.empty()` default-resolution branch of
    /// `getVariableIndexList()` (the group's own variables if a group is
    /// set, else every model variable, `std::iota`'d over
    /// `getVariableCount()`), expressed as names rather than indices since
    /// [`RobotState`]'s own accessors are name-keyed here.
    fn print_variable_names(&self) -> &'m [String] {
        match self.group {
            Some(group) => group.variable_names(),
            None => self.robot_model.variable_names(),
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

/// `print` / `operator<<`. Always takes upstream's default
/// `variable_indexes.empty()` branch — see the module-level "Deviations from
/// upstream" note on why `Display::fmt`'s fixed signature has no room for an
/// explicit column-subset override.
///
/// Column order per waypoint line matches upstream exactly: index, time,
/// position, then velocity/acceleration/effort, each only if the waypoint's
/// [`RobotState`] carries it (`has_velocities`/`has_accelerations`/
/// `has_effort`). Byte-identical output is not required (no fixture compares
/// against it), so this uses Rust's own `{:width.precision}` formatting
/// rather than reproducing `std::ios`'s persistent `std::fixed <<
/// std::setprecision(3)` stream-flag state and its end-of-function
/// flag/precision restore — `Display::fmt` has no ambient stream state to
/// leak into, so there is nothing to restore.
impl std::fmt::Display for RobotTrajectory<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.waypoints.is_empty() {
            return write!(f, "Empty trajectory.");
        }

        writeln!(
            f,
            "Trajectory has {} points over {:.3} seconds",
            self.waypoints.len(),
            self.duration()
        )?;

        let names = self.print_variable_names();

        for (index, waypoint) in self.waypoints.iter().enumerate() {
            write!(
                f,
                "  waypoint {:>3} time {:>5.3} pos ",
                index,
                self.way_point_duration_from_start(index)
            )?;
            for name in names {
                write!(
                    f,
                    "{:>6.3} ",
                    waypoint
                        .variable_position(name)
                        .expect("name from this trajectory's own robot model/group")
                )?;
            }
            if waypoint.has_velocities() {
                write!(f, "vel ")?;
                for name in names {
                    write!(
                        f,
                        "{:>6.3} ",
                        waypoint
                            .variable_velocity(name)
                            .expect("name from this trajectory's own robot model/group")
                    )?;
                }
            }
            if waypoint.has_accelerations() {
                write!(f, "acc ")?;
                for name in names {
                    write!(
                        f,
                        "{:>6.3} ",
                        waypoint
                            .variable_acceleration(name)
                            .expect("name from this trajectory's own robot model/group")
                    )?;
                }
            }
            if waypoint.has_effort() {
                write!(f, "eff ")?;
                for name in names {
                    write!(
                        f,
                        "{:>6.3} ",
                        waypoint
                            .variable_effort(name)
                            .expect("name from this trajectory's own robot model/group")
                    )?;
                }
            }
            writeln!(f)?;
        }

        Ok(())
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

#[cfg(test)]
mod display_tests {
    use std::fs;

    use crate::model::MeshSearchPaths;
    use crate::srdf::SrdfModel;
    use crate::state::RobotState;

    use super::*;

    fn panda() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("{urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    #[test]
    fn empty_trajectory_prints_the_upstream_placeholder() {
        let model = panda();
        let trajectory = RobotTrajectory::new(&model);
        assert_eq!(trajectory.to_string(), "Empty trajectory.");
    }

    #[test]
    fn position_only_waypoints_omit_the_conditional_columns_and_use_the_group_variables() {
        let model = panda();
        let mut trajectory =
            RobotTrajectory::for_group_name(&model, "panda_arm").expect("panda_arm group exists");
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        trajectory
            .add_suffix_way_point(state, 0.0)
            .expect("add waypoint");

        let printed = trajectory.to_string();
        assert!(printed.starts_with("Trajectory has 1 points over 0.000 seconds\n"));
        assert!(printed.contains("  waypoint   0 time 0.000 pos "));
        // No velocity/acceleration/effort was ever set on the waypoint, so
        // none of the three conditional columns should appear.
        assert!(!printed.contains("vel "));
        assert!(!printed.contains("acc "));
        assert!(!printed.contains("eff "));

        // Group is set, so the printed columns are `panda_arm`'s own
        // variables, not the whole model's.
        for name in trajectory.group().unwrap().variable_names() {
            assert!(
                printed.contains(&format!("{:>6.3} ", state_value_for(&trajectory, 0, name))),
                "printed output missing column for {name}: {printed}"
            );
        }
    }

    fn state_value_for(trajectory: &RobotTrajectory<'_>, index: usize, name: &str) -> f64 {
        trajectory
            .way_point(index)
            .expect("waypoint exists")
            .variable_position(name)
            .expect("variable exists")
    }

    #[test]
    fn velocity_and_acceleration_columns_appear_when_the_waypoint_carries_them() {
        let model = panda();
        let mut trajectory = RobotTrajectory::new(&model);
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let variable_count = model.variable_count();
        state.set_variable_velocities(&vec![0.1; variable_count]);
        state.set_variable_accelerations(&vec![0.2; variable_count]);
        trajectory
            .add_suffix_way_point(state, 0.0)
            .expect("add waypoint");

        let printed = trajectory.to_string();
        assert!(printed.contains(" pos "));
        assert!(printed.contains(" vel "));
        assert!(printed.contains(" acc "));
        assert!(
            !printed.contains(" eff "),
            "a state carrying accelerations reports hasEffort() == false"
        );

        // No group set: falls back to every model variable, matching
        // upstream's `variable_indexes.resize(getVariableCount())` branch.
        // Isolate the velocity segment (between "vel " and "acc ") before
        // counting `0.100`s, since the position segment's own default
        // values could otherwise coincidentally contain the same text.
        let vel_segment = printed
            .split("vel ")
            .nth(1)
            .expect("vel column present")
            .split("acc ")
            .next()
            .expect("acc column present");
        assert_eq!(
            vel_segment.matches("0.100").count(),
            model.variable_names().len()
        );
    }

    /// The effort half of the same column logic, and the reason it needs
    /// its own waypoint rather than one state carrying both: upstream's
    /// `hasAccelerations()`/`hasEffort()` are mutually exclusive
    /// (`robot_state.hpp:320`, `:418`), so `operator<<`'s three
    /// independent `if`s (`robot_trajectory.cpp:671-694`) can never print
    /// `acc` and `eff` for one waypoint. An earlier version of this test
    /// set accelerations *and* efforts on one state and asserted all three
    /// columns — a state upstream cannot construct.
    #[test]
    fn the_effort_column_replaces_the_acceleration_column_it_excludes() {
        let model = panda();
        let mut trajectory = RobotTrajectory::new(&model);
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let variable_count = model.variable_count();
        state.set_variable_velocities(&vec![0.1; variable_count]);
        state.set_variable_accelerations(&vec![0.2; variable_count]);
        state.set_variable_efforts(&vec![0.3; variable_count]);
        trajectory
            .add_suffix_way_point(state, 0.0)
            .expect("add waypoint");

        let printed = trajectory.to_string();
        assert!(printed.contains(" vel "));
        assert!(printed.contains(" eff "));
        assert!(
            !printed.contains(" acc "),
            "setting effort clears has_accelerations, so no acc column"
        );
    }
}
