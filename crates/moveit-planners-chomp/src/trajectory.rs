// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_trajectory.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_trajectory.cpp

//! [`ChompTrajectory`]: a discretized joint-space trajectory matrix for
//! CHOMP, `num_points` rows by `num_joints` columns.
//!
//! # Deviations from upstream
//!
//! - **`operator()` is [`std::ops::Index`]/[`std::ops::IndexMut`] on
//!   `(usize, usize)`**, not a named method — `traj[(traj_point, joint)]`
//!   mirrors upstream's `traj(traj_point, joint)` directly.
//! - **`getTrajectoryPoint`/`getJointTrajectory` return owned copies**
//!   ([`ChompTrajectory::trajectory_point`] returns `Vec<f64>`,
//!   [`ChompTrajectory::joint_trajectory`] likewise), not a live
//!   `Eigen::MatrixXd::RowXpr`/`ColXpr` view. Every call site in this
//!   round's ported code either reads a row/column once or overwrites one
//!   wholesale ([`ChompTrajectory::set_trajectory_point`]); nothing needs a
//!   view that stays live across further matrix mutation.
//! - **`getFreeTrajectoryBlock`/`getFreeJointTrajectoryBlock` are not
//!   ported this round.** Both return a live, writable `Eigen::Block` view
//!   with no call site anywhere in `chomp_trajectory.cpp` itself — every
//!   known caller is in `chomp_optimizer.cpp`, out of this round's scope.
//!   Unlike the row/column accessors above, an `Eigen::Block`'s usage
//!   pattern (read-modify-write in place, aliased with other views) is not
//!   safely guessable from the declaration alone; porting it now risks
//!   picking a shape `chomp_optimizer.rs` cannot actually use, which is the
//!   exact "redo it" risk this round's brief calls out for the data
//!   structures a harder file depends on. Deferred to the round that ports
//!   `chomp_optimizer` and can see the real call sites.
//! - **Reachable invariant violations are typed errors, not `assert()`/UB.**
//!   Upstream's `assert()`s (compiled out in release builds) and unchecked
//!   `size_t` arithmetic on caller-supplied indices are both replaced by
//!   `Result::Err` here, matching the convention this port already
//!   established in `moveit-trajectory`'s
//!   `time_optimal_trajectory_generation.rs` (see that module's
//!   `mimic_joint_group_is_a_typed_error_not_a_panic` test): a mismatched
//!   active-joint count or a multi-DOF active joint in
//!   [`ChompTrajectory::assign_chomp_trajectory_point_from_robot_state`], a
//!   too-small `num_points` in the constructors, and a missing group in
//!   [`ChompTrajectory::fill_in_from_trajectory`] are all `Err`, not a
//!   panic or a silently wrong write.
//! - **[`ChompTrajectory::num_free_points`] does not rely on `size_t`
//!   double-wraparound.** Upstream computes `getNumFreePoints()` as
//!   `(end_index_ - start_index_) + 1` in `size_t` arithmetic; for a
//!   2-point trajectory (`start_index_ == 1`, `end_index_ == 0`) this
//!   underflows to `SIZE_MAX` and then overflows back to `0` — the
//!   "correct" answer only by virtue of two wraparounds cancelling out.
//!   This port computes the same `0`-when-inverted result directly via
//!   `(end_index + 1).saturating_sub(start_index)`, not by relying on that
//!   coincidence — see `num_free_points_zero_for_inverted_range` below.
use moveit_error::{Error, Result};
use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use nalgebra::DMatrix;

/// A discretized joint-space trajectory for CHOMP.
///
/// Ported from `chomp::ChompTrajectory`.
#[derive(Debug, Clone, PartialEq)]
pub struct ChompTrajectory {
    planning_group_name: String,
    num_points: usize,
    num_joints: usize,
    discretization: f64,
    duration: f64,
    trajectory: DMatrix<f64>,
    start_index: usize,
    end_index: usize,
    full_trajectory_index: Vec<usize>,
}

impl ChompTrajectory {
    /// Constructs a trajectory for a given robot model, trajectory duration
    /// and discretization.
    ///
    /// Ported from the `(robot_model, double duration, double
    /// discretization, group_name)` constructor, which delegates to the
    /// num-points constructor via `static_cast<size_t>(duration /
    /// discretization) + 1`. Rust's `as usize` cast saturates instead of
    /// invoking C++'s undefined behaviour for a negative or non-finite
    /// `duration / discretization`; both agree on every case where upstream
    /// itself is well-defined.
    pub fn from_duration(
        robot_model: &RobotModel,
        duration: f64,
        discretization: f64,
        group_name: &str,
    ) -> Result<Self> {
        let num_points = (duration / discretization) as usize + 1;
        Self::from_num_points(robot_model, num_points, discretization, group_name)
    }

    /// Constructs a trajectory for a given robot model, number of trajectory
    /// points and discretization.
    ///
    /// Ported from the `(robot_model, size_t num_points, double
    /// discretization, group_name)` constructor. `num_points < 2` is `Err`
    /// (see the module doc's "Reachable invariant violations" note):
    /// upstream computes `end_index_ = num_points_ - 2` in `size_t`
    /// arithmetic, which underflows for `num_points_ < 2`.
    pub fn from_num_points(
        robot_model: &RobotModel,
        num_points: usize,
        discretization: f64,
        group_name: &str,
    ) -> Result<Self> {
        if num_points < 2 {
            return Err(Error::other(format!(
                "ChompTrajectory needs at least 2 points, got {num_points}"
            )));
        }
        let group = robot_model.joint_model_group(group_name)?;
        let num_joints = group.active_joint_indices().len();
        Ok(Self {
            planning_group_name: group_name.to_string(),
            num_points,
            num_joints,
            discretization,
            duration: (num_points - 1) as f64 * discretization,
            trajectory: DMatrix::zeros(num_points, num_joints),
            start_index: 1,
            end_index: num_points - 2,
            full_trajectory_index: Vec::new(),
        })
    }

    /// Creates a new trajectory containing only the joints of `group_name`,
    /// padded at the start and end (if needed) to have enough trajectory
    /// points for `diff_rule_length`-wide differentiation rules.
    ///
    /// Ported from the `(source_traj, group_name, int diff_rule_length)`
    /// constructor. `start_extra`/`end_extra` are computed in `i64` (upstream:
    /// `int`) since they can be negative when `source`'s own margin already
    /// exceeds `diff_rule_length - 1`; a resulting non-positive point count
    /// or negative end index is `Err` rather than the `size_t` wraparound
    /// upstream would produce.
    pub fn from_source_trajectory(
        source: &ChompTrajectory,
        group_name: &str,
        diff_rule_length: usize,
    ) -> Result<Self> {
        let num_joints = source.num_joints;
        let discretization = source.discretization;

        let diff_rule_length_i = diff_rule_length as i64;
        let start_extra = diff_rule_length_i - 1 - source.start_index as i64;
        let end_extra =
            diff_rule_length_i - 1 - (source.num_points as i64 - 1 - source.end_index as i64);

        let num_points_i = source.num_points as i64 + start_extra + end_extra;
        if num_points_i < 1 {
            return Err(Error::other(format!(
                "ChompTrajectory padding produced a non-positive point count ({num_points_i})"
            )));
        }
        let num_points = num_points_i as usize;

        let end_index_i = num_points_i - 1 - (diff_rule_length_i - 1);
        if end_index_i < 0 {
            return Err(Error::other(format!(
                "ChompTrajectory padding produced a negative end index ({end_index_i})"
            )));
        }
        let start_index = diff_rule_length - 1;
        let end_index = end_index_i as usize;
        let duration = (num_points - 1) as f64 * discretization;

        let mut trajectory = Self {
            planning_group_name: group_name.to_string(),
            num_points,
            num_joints,
            discretization,
            duration,
            trajectory: DMatrix::zeros(num_points, num_joints),
            start_index,
            end_index,
            full_trajectory_index: Vec::with_capacity(num_points),
        };

        for i in 0..num_points {
            let mut source_traj_point = i as i64 - start_extra;
            if source_traj_point < 0 {
                source_traj_point = 0;
            }
            if source_traj_point as usize >= source.num_points {
                source_traj_point = source.num_points as i64 - 1;
            }
            let source_traj_point = source_traj_point as usize;
            trajectory.full_trajectory_index.push(source_traj_point);
            for j in 0..num_joints {
                trajectory.trajectory[(i, j)] = source.trajectory[(source_traj_point, j)];
            }
        }

        Ok(trajectory)
    }

    /// Gets the number of points in the trajectory.
    ///
    /// Ported from `getNumPoints`.
    pub fn num_points(&self) -> usize {
        self.num_points
    }

    /// Gets the number of points (that are free to be optimized) in the
    /// trajectory: `0` if `end_index < start_index` (an inverted range),
    /// else `end_index - start_index + 1`.
    ///
    /// Ported from `getNumFreePoints` — see the module doc's
    /// `num_free_points` deviation note.
    pub fn num_free_points(&self) -> usize {
        (self.end_index + 1).saturating_sub(self.start_index)
    }

    /// Gets the number of joints in each trajectory point.
    ///
    /// Ported from `getNumJoints`.
    pub fn num_joints(&self) -> usize {
        self.num_joints
    }

    /// Gets the discretization time interval of the trajectory.
    ///
    /// Ported from `getDiscretization`.
    pub fn discretization(&self) -> f64 {
        self.discretization
    }

    /// Gets the duration of the trajectory.
    ///
    /// Ported from `getDuration`.
    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Gets the planning group name this trajectory corresponds to.
    ///
    /// Ported from the private `planning_group_name_` field (upstream has
    /// no getter for it; added here since every constructor takes it and
    /// nothing else exposes it back).
    pub fn planning_group_name(&self) -> &str {
        &self.planning_group_name
    }

    /// Sets the start and end index for the modifiable part of the
    /// trajectory. Everything before `start_index` and after `end_index` is
    /// considered fixed.
    ///
    /// Ported from `setStartEndIndex`.
    pub fn set_start_end_index(&mut self, start_index: usize, end_index: usize) {
        self.start_index = start_index;
        self.end_index = end_index;
    }

    /// Gets the start index.
    ///
    /// Ported from `getStartIndex`.
    pub fn start_index(&self) -> usize {
        self.start_index
    }

    /// Gets the end index.
    ///
    /// Ported from `getEndIndex`.
    pub fn end_index(&self) -> usize {
        self.end_index
    }

    /// Gets the entire trajectory matrix.
    ///
    /// Ported from `getTrajectory`.
    pub fn trajectory_matrix(&self) -> &DMatrix<f64> {
        &self.trajectory
    }

    /// Gets the index in the source trajectory that trajectory point `i` was
    /// copied from by [`ChompTrajectory::from_source_trajectory`].
    ///
    /// Ported from `getFullTrajectoryIndex`.
    pub fn full_trajectory_index(&self, i: usize) -> usize {
        self.full_trajectory_index[i]
    }

    /// Gets trajectory point `traj_point` as an owned copy — see the module
    /// doc's "owned copies" deviation note.
    ///
    /// Ported from `getTrajectoryPoint`.
    pub fn trajectory_point(&self, traj_point: usize) -> Vec<f64> {
        self.trajectory.row(traj_point).iter().copied().collect()
    }

    /// Overwrites trajectory point `traj_point` with `values`.
    ///
    /// Ported from assigning through `getTrajectoryPoint`'s mutable
    /// `RowXpr` (e.g. `getTrajectoryPoint(i) = other.getTrajectoryPoint(j)`
    /// in the padding constructor) — see the module doc's "owned copies"
    /// deviation note.
    pub fn set_trajectory_point(&mut self, traj_point: usize, values: &[f64]) {
        for (j, &v) in values.iter().enumerate() {
            self.trajectory[(traj_point, j)] = v;
        }
    }

    /// Gets joint `joint`'s trajectory as an owned copy — see the module
    /// doc's "owned copies" deviation note.
    ///
    /// Ported from `getJointTrajectory`.
    pub fn joint_trajectory(&self, joint: usize) -> Vec<f64> {
        self.trajectory.column(joint).iter().copied().collect()
    }

    /// Generates a minimum-jerk trajectory from `start_index - 1` to
    /// `end_index + 1`. Only modifies points in `[start_index, end_index]`.
    ///
    /// Ported from `fillInMinJerk`. Panics if `start_index == 0` (matching
    /// upstream's own unchecked `start_index_ - 1` in `size_t`/`double`
    /// arithmetic — every constructor sets `start_index >= 1`;
    /// [`ChompTrajectory::set_start_end_index`] is the only way to violate
    /// that).
    pub fn fill_in_min_jerk(&mut self) {
        let start_index = self.start_index - 1;
        let end_index = self.end_index + 1;
        let mut td = [0.0f64; 6];
        td[0] = 1.0;
        td[1] = (end_index - start_index) as f64 * self.discretization;
        for k in 2..=5 {
            td[k] = td[k - 1] * td[1];
        }

        let mut coeff = vec![[0.0f64; 6]; self.num_joints];
        for joint in 0..self.num_joints {
            let x0 = self[(start_index, joint)];
            let x1 = self[(end_index, joint)];
            coeff[joint][0] = x0;
            coeff[joint][1] = 0.0;
            coeff[joint][2] = 0.0;
            coeff[joint][3] = (-20.0 * x0 + 20.0 * x1) / (2.0 * td[3]);
            coeff[joint][4] = (30.0 * x0 - 30.0 * x1) / (2.0 * td[4]);
            coeff[joint][5] = (-12.0 * x0 + 12.0 * x1) / (2.0 * td[5]);
        }

        for traj_point in (start_index + 1)..end_index {
            let mut ti = [0.0f64; 6];
            ti[0] = 1.0;
            ti[1] = (traj_point - start_index) as f64 * self.discretization;
            for k in 2..=5 {
                ti[k] = ti[k - 1] * ti[1];
            }
            for joint in 0..self.num_joints {
                let mut value = 0.0;
                for k in 0..=5 {
                    value += ti[k] * coeff[joint][k];
                }
                self[(traj_point, joint)] = value;
            }
        }
    }

    /// Generates a linearly interpolated trajectory from `start_index - 1`
    /// to `end_index + 1`. Only modifies points in `[start_index,
    /// end_index]`.
    ///
    /// Ported from `fillInLinearInterpolation`. Same unchecked-`start_index`
    /// precondition as [`ChompTrajectory::fill_in_min_jerk`].
    ///
    /// Transcribed exactly as upstream, including a numeric quirk worth
    /// flagging rather than "fixing": the per-step slope `theta` divides by
    /// `end_index - 1` (`end_index` being `self.end_index + 1`), not by the
    /// true span `end_index - start_index`. The two coincide only when
    /// `start_index == 1` (the constructors' default), i.e. when the local
    /// `start_index` above is `0` — the common case this method is actually
    /// exercised with. For any other `start_index` (only reachable via
    /// [`ChompTrajectory::set_start_end_index`]), upstream's own formula
    /// still uses this same `end_index - 1` denominator, so this is not a
    /// deviation from upstream, just a documented surprise in what upstream
    /// computes.
    pub fn fill_in_linear_interpolation(&mut self) {
        let start_index = self.start_index - 1;
        let end_index = self.end_index + 1;
        let end_index_f = end_index as f64;
        for i in 0..self.num_joints {
            let theta = (self[(end_index, i)] - self[(start_index, i)]) / (end_index_f - 1.0);
            for j in (start_index + 1)..end_index {
                self[(j, i)] = self[(start_index, i)] + j as f64 * theta;
            }
        }
    }

    /// Generates a cubic interpolation of the trajectory from `start_index -
    /// 1` to `end_index + 1`. Only modifies points in `[start_index,
    /// end_index]`.
    ///
    /// Ported from `fillInCubicInterpolation`. Same unchecked-`start_index`
    /// precondition as [`ChompTrajectory::fill_in_min_jerk`]. Upstream's
    /// `coeffs[1]` (the linear term) is initialized to `0` and never
    /// written or read from again — it contributes nothing to the
    /// evaluated polynomial, so it is not represented here at all; this
    /// changes no computed value. `pow(x, 2)`/`pow(x, 3)` (C++'s `pow(double,
    /// int)` overload) are ported as [`f64::powi`], the closest Rust
    /// equivalent to a repeated-multiplication integer-exponent `pow`.
    pub fn fill_in_cubic_interpolation(&mut self) {
        let start_index = self.start_index - 1;
        let end_index = self.end_index + 1;
        let dt = 0.001;
        let total_time = (end_index as f64 - 1.0) * dt;
        for i in 0..self.num_joints {
            let x0 = self[(start_index, i)];
            let x1 = self[(end_index, i)];
            let c0 = x0;
            let c2 = (3.0 / total_time.powi(2)) * (x1 - x0);
            let c3 = (-2.0 / total_time.powi(3)) * (x1 - x0);
            for j in (start_index + 1)..end_index {
                let t = j as f64 * dt;
                self[(j, i)] = c0 + c2 * t.powi(2) + c3 * t.powi(3);
            }
        }
    }

    /// Updates `self` (the full trajectory) from `group_trajectory`'s free
    /// block, at `self`'s own `start_index`.
    ///
    /// Ported from `updateFromGroupTrajectory`.
    pub fn update_from_group_trajectory(&mut self, group_trajectory: &ChompTrajectory) {
        let num_vars_free = self.end_index - self.start_index + 1;
        for r in 0..num_vars_free {
            for c in 0..self.num_joints {
                self.trajectory[(self.start_index + r, c)] =
                    group_trajectory.trajectory[(group_trajectory.start_index + r, c)];
            }
        }
    }

    /// Receives a path (e.g. from OMPL) and puts it into the trajectory
    /// format CHOMP requires, resampling `trajectory` to this trajectory's
    /// `num_points` by piecewise-linear interpolation over waypoint index.
    /// Returns `false` (matching upstream) if `trajectory` has fewer than 2
    /// waypoints.
    ///
    /// Ported from `fillInFromTrajectory`. `trajectory.group()` being
    /// `None` is `Err` — see the module doc's "Reachable invariant
    /// violations" note; upstream instead passes a possibly-null group
    /// pointer straight into `RobotState::interpolate`.
    pub fn fill_in_from_trajectory(&mut self, trajectory: &RobotTrajectory) -> Result<bool> {
        if trajectory.way_point_count() < 2 {
            return Ok(false);
        }

        let max_output_index = self.num_points - 1;
        let max_input_index = trajectory.way_point_count() - 1;
        let group = trajectory.group().ok_or_else(|| {
            Error::other("fill_in_from_trajectory requires trajectory.group() to be Some")
        })?;
        let mut interpolated = RobotState::new(trajectory.robot_model());

        for i in 0..=max_output_index {
            let fraction_full = (i * max_input_index) as f64 / max_output_index as f64;
            let prev_idx = fraction_full.trunc() as usize;
            let fraction = fraction_full - prev_idx as f64;
            let next_idx = if prev_idx == max_input_index {
                prev_idx
            } else {
                prev_idx + 1
            };
            let from = trajectory.way_point(prev_idx)?;
            let to = trajectory.way_point(next_idx)?;
            interpolate_group_into(from, to, fraction, group, &mut interpolated);
            self.assign_chomp_trajectory_point_from_robot_state(&interpolated, i, group)?;
        }
        Ok(true)
    }

    /// Assigns `source`'s active-joint positions for `group` into
    /// trajectory point `chomp_trajectory_point`.
    ///
    /// Ported from `assignCHOMPTrajectoryPointFromRobotState`. Upstream's
    /// two `assert()`s (`group`'s active-joint count matches this
    /// trajectory's joint-column count; every active joint has exactly 1
    /// variable) are both `Err` here instead — see the module doc's
    /// "Reachable invariant violations" note.
    pub fn assign_chomp_trajectory_point_from_robot_state(
        &mut self,
        source: &RobotState,
        chomp_trajectory_point: usize,
        group: &JointModelGroup,
    ) -> Result<()> {
        if group.active_joint_indices().len() != self.num_joints {
            return Err(Error::other(format!(
                "group {:?} has {} active joints, but this ChompTrajectory has {} joint columns",
                group.name(),
                group.active_joint_indices().len(),
                self.num_joints
            )));
        }
        for (joint_index, &model_index) in group.active_joint_indices().iter().enumerate() {
            let joint = source.model().joint_model_at(model_index);
            if joint.variable_count() != 1 {
                return Err(Error::other(format!(
                    "joint {:?} has {} variables; ChompTrajectory requires every active joint in the group to have exactly 1",
                    joint.name(),
                    joint.variable_count()
                )));
            }
            let value = source.joint_position(joint.name())?[0];
            self.trajectory[(chomp_trajectory_point, joint_index)] = value;
        }
        Ok(())
    }

    /// Gets the joint velocities at trajectory point `traj_point`, one entry
    /// per joint column, via the [`crate::utils::DIFF_RULES`] velocity
    /// stencil.
    ///
    /// Ported from `getJointVelocities`. Panics (matching upstream's own
    /// unchecked windowed read, now via an in-bounds Rust index panic
    /// instead of undefined behaviour) if `traj_point` is closer than
    /// `DIFF_RULE_LENGTH / 2` rows to either end of the trajectory.
    pub fn joint_velocities(&self, traj_point: usize) -> Vec<f64> {
        let inv_time = 1.0 / self.discretization;
        let half = crate::utils::DIFF_RULE_LENGTH as i64 / 2;
        let mut velocities = vec![0.0; self.num_joints];
        for k in -half..=half {
            let row = (traj_point as i64 + k) as usize;
            let coeff = inv_time * crate::utils::DIFF_RULES[0][(k + half) as usize];
            for (j, velocity) in velocities.iter_mut().enumerate() {
                *velocity += coeff * self.trajectory[(row, j)];
            }
        }
        velocities
    }
}

impl std::ops::Index<(usize, usize)> for ChompTrajectory {
    type Output = f64;

    /// Ported from `operator()(size_t, size_t) const`.
    fn index(&self, (traj_point, joint): (usize, usize)) -> &f64 {
        &self.trajectory[(traj_point, joint)]
    }
}

impl std::ops::IndexMut<(usize, usize)> for ChompTrajectory {
    /// Ported from `operator()(size_t, size_t)`.
    fn index_mut(&mut self, (traj_point, joint): (usize, usize)) -> &mut f64 {
        &mut self.trajectory[(traj_point, joint)]
    }
}

/// `RobotState::interpolate(to, t, state, group)`: linearly interpolate
/// every active joint of `group` between `from` and `to`, writing the
/// result into `out`. There is no group-scoped `RobotState::interpolate` in
/// this port yet (only the whole-model interpolation
/// `moveit-trajectory::robot_trajectory` uses privately) — this mirrors
/// that helper's shape, restricted to `group.active_joint_indices()`.
fn interpolate_group_into(
    from: &RobotState,
    to: &RobotState,
    t: f64,
    group: &JointModelGroup,
    out: &mut RobotState,
) {
    let model = from.model();
    for &index in group.active_joint_indices() {
        let joint = model.joint_model_at(index);
        if joint.variable_count() == 0 {
            continue;
        }
        let from_values = from
            .joint_position(joint.name())
            .expect("active joint of this group's own robot model");
        let to_values = to
            .joint_position(joint.name())
            .expect("active joint of this group's own robot model");
        let mut buffer = vec![0.0; joint.variable_count()];
        joint.interpolate(from_values, to_values, t, &mut buffer);
        out.set_joint_positions(joint.name(), &buffer)
            .expect("active joint of this group's own robot model");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;
    use std::sync::OnceLock;

    const EPS: f64 = 1e-12;
    const GROUP: &str = "panda_arm";
    const N: usize = 7;

    fn panda_model() -> &'static RobotModel {
        static MODEL: OnceLock<RobotModel> = OnceLock::new();
        MODEL.get_or_init(|| {
            let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
            let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
            let urdf_xml = std::fs::read_to_string(urdf_path)
                .unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
            let urdf = urdf_rs::read_file(urdf_path).expect("panda.urdf parses");
            let srdf = SrdfModel::parse_file(srdf_path).expect("panda.srdf parses");
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("panda model builds")
        })
    }

    /// A length-`N` row with `value` in `joint`'s column and `0.0` elsewhere.
    fn row(joint: usize, value: f64) -> Vec<f64> {
        let mut v = vec![0.0; N];
        v[joint] = value;
        v
    }

    #[test]
    fn from_num_points_rejects_fewer_than_two_points() {
        let model = panda_model();
        let err = ChompTrajectory::from_num_points(model, 1, 0.1, GROUP).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
        let err = ChompTrajectory::from_num_points(model, 0, 0.1, GROUP).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn from_num_points_sets_upstream_default_start_end_index() {
        let model = panda_model();
        let traj = ChompTrajectory::from_num_points(model, 10, 0.1, GROUP).unwrap();
        assert_eq!(traj.num_joints(), N);
        assert_eq!(traj.start_index(), 1);
        assert_eq!(traj.end_index(), 8);
        assert_eq!(traj.num_free_points(), 8);
        assert_relative_eq!(traj.duration(), 0.9, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn num_free_points_zero_for_inverted_range() {
        // A 2-point trajectory: start_index=1, end_index=0 (num_points-2).
        // Upstream's own `(end_index_ - start_index_) + 1` in size_t
        // arithmetic reaches 0 here only via double-wraparound; this port's
        // direct computation must reach the same 0 without relying on that.
        let model = panda_model();
        let traj = ChompTrajectory::from_num_points(model, 2, 0.1, GROUP).unwrap();
        assert_eq!(traj.start_index(), 1);
        assert_eq!(traj.end_index(), 0);
        assert_eq!(traj.num_free_points(), 0);
    }

    #[test]
    fn from_source_trajectory_pads_short_source_and_pins_full_trajectory_index() {
        let model = panda_model();
        // start_index=1, end_index=num_points-2=3 for a 5-point source.
        let mut source = ChompTrajectory::from_num_points(model, 5, 0.1, GROUP).unwrap();
        for i in 0..5 {
            source.set_trajectory_point(i, &row(0, i as f64));
        }

        // diff_rule_length=7 needs 6 points of margin on each side;
        // source's own margin is start_index=1 and
        // (num_points-1)-end_index=(5-1)-3=1, so start_extra = end_extra =
        // 6-1 = 5, num_points = 5+5+5 = 15.
        let padded = ChompTrajectory::from_source_trajectory(&source, GROUP, 7).unwrap();
        assert_eq!(padded.num_points(), 15);
        assert_eq!(padded.start_index(), 6);
        assert_eq!(padded.end_index(), 8);

        // source_traj_point(i) = clamp(i - start_extra, 0, source.num_points-1)
        // = clamp(i - 5, 0, 4): rows 0..=5 clamp to source row 0, rows 6..9
        // map 1:1 to source rows 1..4, rows 10..14 clamp to source row 4.
        let expected_source_rows = [0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 4, 4, 4, 4, 4];
        for (i, &expected) in expected_source_rows.iter().enumerate() {
            assert_eq!(padded.full_trajectory_index(i), expected, "row {i}");
            assert_eq!(
                padded.trajectory_point(i),
                source.trajectory_point(expected),
                "row {i} content"
            );
        }
    }

    #[test]
    fn from_source_trajectory_shrinks_when_source_already_has_enough_margin() {
        let model = panda_model();
        // start_index=diff_rule_length-1=2 directly, end_index=num_points-1-2.
        let source = ChompTrajectory::from_source_trajectory(
            &ChompTrajectory::from_num_points(model, 20, 0.1, GROUP).unwrap(),
            GROUP,
            3,
        )
        .unwrap();
        // Re-padding with the same diff_rule_length=3 the source already
        // satisfies exactly (start_extra = end_extra = 0) must not change
        // the point count.
        let repadded = ChompTrajectory::from_source_trajectory(&source, GROUP, 3).unwrap();
        assert_eq!(repadded.num_points(), source.num_points());
    }

    #[test]
    fn index_and_set_trajectory_point_round_trip() {
        let model = panda_model();
        let mut traj = ChompTrajectory::from_num_points(model, 4, 0.1, GROUP).unwrap();
        let values = row(0, 1.5);
        traj.set_trajectory_point(2, &values);
        assert_relative_eq!(traj[(2, 0)], 1.5, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(traj[(2, 1)], 0.0, epsilon = EPS, max_relative = EPS);
        assert_eq!(traj.trajectory_point(2), values);
        traj[(2, 0)] = 9.0;
        assert_relative_eq!(traj[(2, 0)], 9.0, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn fill_in_linear_interpolation_matches_upstream_end_index_minus_one_denominator() {
        // num_points=6 -> start_index=1, end_index=4; local start=0, local
        // end=5. theta = (5.0-0.0)/(5-1) = 1.25 (see fill_in_linear_
        // interpolation's doc comment on why this is *not* 5.0/(5-0)).
        // Rows 0 and 5 are the fixed anchors set below and left untouched;
        // rows 1..=4 (the free region [start_index, end_index]) are filled.
        let model = panda_model();
        let mut traj = ChompTrajectory::from_num_points(model, 6, 0.1, GROUP).unwrap();
        traj.set_trajectory_point(0, &row(0, 0.0));
        traj.set_trajectory_point(5, &row(0, 5.0));
        traj.fill_in_linear_interpolation();
        let expected = [0.0, 1.25, 2.5, 3.75, 5.0, 5.0];
        for (i, &want) in expected.iter().enumerate() {
            assert_relative_eq!(traj[(i, 0)], want, epsilon = EPS, max_relative = EPS);
        }
    }

    #[test]
    fn fill_in_min_jerk_starts_and_ends_at_boundary_values_with_zero_endpoint_velocity() {
        let model = panda_model();
        let mut traj = ChompTrajectory::from_num_points(model, 6, 0.1, GROUP).unwrap();
        traj.set_trajectory_point(0, &row(0, 1.0));
        traj.set_trajectory_point(5, &row(0, 4.0));
        traj.fill_in_min_jerk();
        assert_relative_eq!(traj[(0, 0)], 1.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(traj[(5, 0)], 4.0, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn fill_in_cubic_interpolation_starts_and_ends_at_boundary_values() {
        let model = panda_model();
        let mut traj = ChompTrajectory::from_num_points(model, 6, 0.1, GROUP).unwrap();
        traj.set_trajectory_point(0, &row(0, 1.0));
        traj.set_trajectory_point(5, &row(0, 4.0));
        traj.fill_in_cubic_interpolation();
        assert_relative_eq!(traj[(0, 0)], 1.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(traj[(5, 0)], 4.0, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn assign_chomp_trajectory_point_rejects_group_column_mismatch() {
        let model = panda_model();
        let mut traj = ChompTrajectory::from_num_points(model, 4, 0.1, GROUP).unwrap();
        let other_group = model.joint_model_group("hand").unwrap();
        assert_ne!(other_group.active_joint_indices().len(), N);
        let state = RobotState::new(model);
        let err = traj
            .assign_chomp_trajectory_point_from_robot_state(&state, 1, other_group)
            .unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn fill_in_from_trajectory_rejects_fewer_than_two_waypoints() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = ChompTrajectory::from_num_points(model, 4, 0.1, GROUP).unwrap();
        let mut source = RobotTrajectory::for_group(model, Some(group));
        source
            .add_suffix_way_point(RobotState::new(model), 0.0)
            .unwrap();
        assert!(!traj.fill_in_from_trajectory(&source).unwrap());
    }

    #[test]
    fn fill_in_from_trajectory_resamples_across_full_output_range() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut source = RobotTrajectory::for_group(model, Some(group));
        let mut start = RobotState::new(model);
        start.set_joint_positions("panda_joint1", &[0.0]).unwrap();
        let mut end = RobotState::new(model);
        end.set_joint_positions("panda_joint1", &[1.0]).unwrap();
        source.add_suffix_way_point(start, 0.0).unwrap();
        source.add_suffix_way_point(end, 1.0).unwrap();

        let mut traj = ChompTrajectory::from_num_points(model, 5, 0.1, GROUP).unwrap();
        assert!(traj.fill_in_from_trajectory(&source).unwrap());
        assert_relative_eq!(traj[(0, 0)], 0.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(traj[(4, 0)], 1.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(traj[(2, 0)], 0.5, epsilon = EPS, max_relative = EPS);
    }
}
