// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (class Trajectory)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (Trajectory:: methods)

//! `Trajectory`: the time-optimal path-velocity profile built from a
//! [`crate::Path`].
//!
//! `TimeOptimalTrajectoryGeneration` (the `robot_trajectory::RobotTrajectory`
//! adapter, header line 193 on) is **ported**, in
//! [`crate::time_optimal_trajectory_generation`] — this note previously said
//! it was out of scope, which stopped being true once that module landed;
//! see that module's own doc comment for what it covers and what it
//! deliberately does not. This module is `pub` only so this note and
//! [`Trajectory`]'s own doc comment are reachable from `crate::trajectory`
//! links elsewhere in the crate; every other item in it stays private or
//! `pub(crate)`.

use nalgebra::DVector;

use moveit_error::{Error, Result};

use crate::numeric::cxx_min;
use crate::path::Path;

/// Upstream anonymous-namespace `EPS`: the epsilon most switching-point and
/// intersection comparisons in this module are measured against.
const EPS: f64 = 1e-6;

/// Upstream anonymous-namespace `DEFAULT_TIMESTEP`: the fixed scan step
/// [`Trajectory::next_velocity_switching_point`] advances by while searching
/// for a velocity-limit switching point.
///
/// # Deviation from upstream (a non-deviation, spelled out)
///
/// This is **not** the same quantity as [`Trajectory::create`]'s
/// `time_step` parameter (the forward/backward Euler integration step) even
/// though both default to `0.001` — upstream keeps them as two unrelated
/// constants (`DEFAULT_TIMESTEP` here, `time_step = 0.001` in the header's
/// default argument) and this port keeps that separation rather than
/// collapsing them into one shared value.
const VELOCITY_SWITCHING_SCAN_STEP: f64 = 1e-3;

/// One sample of the time-optimal path-velocity profile: how far along the
/// path (`path_pos`), how fast (`path_vel`), and at what point in time.
///
/// Upstream `Trajectory::TrajectoryStep`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrajectoryStep {
    path_pos: f64,
    path_vel: f64,
    time: f64,
}

impl TrajectoryStep {
    fn new(path_pos: f64, path_vel: f64) -> Self {
        Self {
            path_pos,
            path_vel,
            time: 0.0,
        }
    }
}

/// A time-optimal parameterization of a [`Path`]: the path-velocity profile
/// that gets from one end to the other as fast as `max_velocity`/
/// `max_acceleration` allow. Build one with [`Trajectory::create`], then
/// sample it with [`Trajectory::position`]/[`Trajectory::velocity`]/
/// [`Trajectory::acceleration`] at any time in `[0, `[`Trajectory::duration`]`]`.
///
/// Upstream `trajectory_processing::Trajectory`. Three private fields this
/// port drops, each documented at the point it would have been used:
///
/// - `end_trajectory_` — write-only in upstream (assigned on both
///   `integrateBackward` failure paths, `include/.../time_optimal_trajectory_generation.hpp:184`)
///   with no getter and no other reader anywhere in `moveit2`
///   (confirmed by search); a pure debugger-inspectable breadcrumb, not
///   part of the ported behaviour.
/// - `cached_time_`/`cached_trajectory_segment_` — a `mutable` forward-scan
///   cache in `getTrajectorySegment` that speeds up repeated
///   monotonically-increasing time queries. Every value it can produce is
///   also produced by a fresh binary search (`trajectory` is sorted by
///   `time`, see `Trajectory::segment_index`), so dropping it changes
///   nothing observable; keeping it would mean smuggling interior
///   mutability into a query method for a benefit this crate does not
///   (yet) need to preserve.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    path: Path,
    max_velocity: DVector<f64>,
    max_acceleration: DVector<f64>,
    joint_num: usize,
    valid: bool,
    trajectory: Vec<TrajectoryStep>,
    time_step: f64,
}

impl Trajectory {
    /// Generate a time-optimal trajectory along `path`.
    ///
    /// Upstream `Trajectory::create`, which returns `std::optional<Trajectory>`;
    /// see [`Path::create`]'s doc comment for why this port returns `Result`
    /// instead.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `time_step <= 0.0`, or when the forward/
    /// backward integration cannot produce a valid trajectory (for example,
    /// a `max_acceleration` component that is `0.0` along a path direction
    /// that needs it — see this crate's `testRelevantZeroMaxAccelerationsInvalidateTrajectory`
    /// test).
    pub fn create(
        path: Path,
        max_velocity: &DVector<f64>,
        max_acceleration: &DVector<f64>,
        time_step: f64,
    ) -> Result<Self> {
        if time_step <= 0.0 {
            return Err(Error::construct(
                "the trajectory is invalid because the time step is <= 0.0",
            ));
        }

        let joint_num = max_velocity.len();
        let mut traj = Self {
            path,
            max_velocity: max_velocity.clone(),
            max_acceleration: max_acceleration.clone(),
            joint_num,
            valid: true,
            trajectory: vec![TrajectoryStep::new(0.0, 0.0)],
            time_step,
        };

        let mut after_acceleration = traj.min_max_path_acceleration(0.0, 0.0, true);
        // Upstream's loop condition re-checks `valid_` a third time, after
        // `integrateForward` returns, even though every path that sets
        // `valid_ = false` also returns `true` (so the recheck can never
        // actually differ) — ported anyway, faithfully, since it costs
        // nothing to keep.
        while traj.valid && !traj.integrate_forward(after_acceleration) && traj.valid {
            let last_pos = traj
                .trajectory
                .last()
                .expect("initial step always present")
                .path_pos;
            let Some((switching_point, before_acceleration, next_after_acceleration)) =
                traj.next_switching_point(last_pos)
            else {
                break;
            };
            after_acceleration = next_after_acceleration;
            traj.integrate_backward(
                switching_point.path_pos,
                switching_point.path_vel,
                before_acceleration,
            );
        }

        if !traj.valid {
            return Err(Error::construct(
                "trajectory not valid after integrateForward and integrateBackward",
            ));
        }

        let before_acceleration = traj.min_max_path_acceleration(traj.path.length(), 0.0, false);
        traj.integrate_backward(traj.path.length(), 0.0, before_acceleration);

        if !traj.valid {
            return Err(Error::construct(
                "trajectory not valid after the second integrateBackward pass",
            ));
        }

        // Calculate timing.
        for i in 1..traj.trajectory.len() {
            let previous = traj.trajectory[i - 1];
            let current = &mut traj.trajectory[i];
            current.time = previous.time
                + (current.path_pos - previous.path_pos)
                    / ((current.path_vel + previous.path_vel) / 2.0);
        }

        Ok(traj)
    }

    /// `getDuration`.
    pub fn duration(&self) -> f64 {
        self.trajectory
            .last()
            .expect("Trajectory::create always leaves at least one step")
            .time
    }

    /// `getPosition`.
    pub fn position(&self, time: f64) -> DVector<f64> {
        let idx = self.segment_index(time);
        let path_pos = Self::position_at(self.trajectory[idx - 1], self.trajectory[idx], time);
        self.path.config(path_pos)
    }

    /// `getVelocity`.
    ///
    /// # Deviation from upstream (a non-deviation, spelled out)
    ///
    /// Upstream's `getVelocity` computes the `path_pos`/`path_vel` it feeds
    /// to `path_.getTangent` using the *full* enclosing segment's time step
    /// (`it->time_ - previous->time_`), never re-deriving it against the
    /// query `time` the way `getPosition` does (`time - previous->time_`).
    /// Concretely: for any `time` inside a segment `(previous, current]`,
    /// `getVelocity` returns the value at `current.time_`, not at `time` —
    /// it is a step function of `time`, constant within each segment,
    /// unlike [`Trajectory::position`]. This looks like it could be an
    /// upstream oversight, but it is upstream's actual, observable
    /// behaviour (confirmed disagreeing with an earlier, incorrect version
    /// of this port against the `totg` oracle op — see
    /// `tests/totg_parity.rs`), so it is transcribed here rather than
    /// "fixed" into a continuously interpolated curve. [`Trajectory::
    /// acceleration`] has the same non-re-derivation property, for the same
    /// reason (see its doc comment); `Trajectory::segment_endpoint_state`
    /// is the computation both share.
    pub fn velocity(&self, time: f64) -> DVector<f64> {
        let idx = self.segment_index(time);
        let (path_pos, path_vel) =
            Self::segment_endpoint_state(self.trajectory[idx - 1], self.trajectory[idx]);
        self.path.tangent(path_pos) * path_vel
    }

    /// `getAcceleration`.
    ///
    /// # Deviation from upstream (a non-deviation, spelled out)
    ///
    /// Like [`Trajectory::velocity`] (see its doc comment), this evaluates
    /// the segment's constant acceleration using the *full* segment span
    /// `current.time - previous.time` throughout rather than re-deriving
    /// against `time` — exactly as upstream does. That is not an oversight
    /// — `trajectory` is built by fixed-acceleration Euler steps (see
    /// `Trajectory::integrate_forward`/`Trajectory::integrate_backward`),
    /// so acceleration is constant *within* a segment; querying `path_pos`/
    /// `path_vel` at the segment's own end point and differencing the
    /// tangent there is upstream's way of reading off that constant.
    pub fn acceleration(&self, time: f64) -> DVector<f64> {
        let idx = self.segment_index(time);
        let current = self.trajectory[idx];
        let previous = self.trajectory[idx - 1];
        let time_step = current.time - previous.time;

        let (path_pos, path_vel) = Self::segment_endpoint_state(previous, current);

        let mut path_acc = self.path.tangent(path_pos) * path_vel
            - self.path.tangent(previous.path_pos) * previous.path_vel;
        if time_step > 0.0 {
            path_acc /= time_step;
        }
        path_acc
    }

    /// The path position at `time`, found by fitting the constant-
    /// acceleration quadratic between the trajectory steps `previous` and
    /// `current` bracketing `time` and evaluating it at `time` itself — as
    /// opposed to [`Trajectory::segment_endpoint_state`], which deliberately
    /// evaluates at `current.time` instead (see [`Trajectory::velocity`]'s
    /// doc comment).
    fn position_at(previous: TrajectoryStep, current: TrajectoryStep, time: f64) -> f64 {
        let segment_time_step = current.time - previous.time;
        let acceleration = 2.0
            * (current.path_pos - previous.path_pos - segment_time_step * previous.path_vel)
            / (segment_time_step * segment_time_step);

        let time_step = time - previous.time;
        previous.path_pos
            + time_step * previous.path_vel
            + 0.5 * time_step * time_step * acceleration
    }

    /// Shared by [`Trajectory::velocity`]/[`Trajectory::acceleration`]: the
    /// path position and path velocity evaluated at the *end* of the
    /// segment `(previous, current)`, found by fitting the segment's
    /// constant-acceleration quadratic and evaluating it at `current.time`
    /// — see [`Trajectory::velocity`]'s doc comment for why that, not the
    /// query time, is upstream's actual behaviour.
    fn segment_endpoint_state(previous: TrajectoryStep, current: TrajectoryStep) -> (f64, f64) {
        let time_step = current.time - previous.time;
        let acceleration = 2.0
            * (current.path_pos - previous.path_pos - time_step * previous.path_vel)
            / (time_step * time_step);

        let path_pos = previous.path_pos
            + time_step * previous.path_vel
            + 0.5 * time_step * time_step * acceleration;
        let path_vel = previous.path_vel + time_step * acceleration;
        (path_pos, path_vel)
    }

    /// The index of the trajectory step such that `trajectory[index - 1]`
    /// and `trajectory[index]` bracket `time` (or `trajectory.len() - 1` if
    /// `time` is at or past the end).
    ///
    /// Upstream `getTrajectorySegment`, minus the forward-scan cache — see
    /// [`Trajectory`]'s doc comment for why dropping it is behaviour
    /// preserving. `self.trajectory` is sorted by `time` (built
    /// monotonically by [`Trajectory::create`]'s timing pass), so
    /// `partition_point` finds the same index the cache's `while (time >=
    /// it->time_) ++it;` scan would have.
    fn segment_index(&self, time: f64) -> usize {
        let last = self.trajectory.len() - 1;
        if time >= self.trajectory[last].time {
            return last;
        }
        self.trajectory.partition_point(|step| step.time <= time)
    }

    // ---- Switching-point search --------------------------------------

    /// The next point (acceleration- or velocity-limited) at or after
    /// `path_pos` where the maximum-velocity curve either has a
    /// discontinuity the trajectory must integrate around, or a local
    /// extremum. `None` means no more switching points before the end of
    /// the path.
    ///
    /// Upstream `Trajectory::getNextSwitchingPoint`.
    fn next_switching_point(&self, path_pos: f64) -> Option<(TrajectoryStep, f64, f64)> {
        let mut accel_switch = TrajectoryStep::new(path_pos, 0.0);
        let mut accel_before = 0.0;
        let mut accel_after = 0.0;
        let accel_reached_end = loop {
            match self.next_acceleration_switching_point(accel_switch.path_pos) {
                None => break true,
                Some((step, before, after)) => {
                    accel_switch = step;
                    accel_before = before;
                    accel_after = after;
                    if accel_switch.path_vel
                        <= self.velocity_max_path_velocity(accel_switch.path_pos)
                    {
                        break false;
                    }
                }
            }
        };

        let mut vel_switch = TrajectoryStep::new(path_pos, 0.0);
        let mut vel_before = 0.0;
        let mut vel_after = 0.0;
        let vel_reached_end = loop {
            match self.next_velocity_switching_point(vel_switch.path_pos) {
                None => break true,
                Some((step, before, after)) => {
                    vel_switch = step;
                    vel_before = before;
                    vel_after = after;
                    let keep_scanning = vel_switch.path_pos <= accel_switch.path_pos
                        && (vel_switch.path_vel
                            > self.acceleration_max_path_velocity(vel_switch.path_pos - EPS)
                            || vel_switch.path_vel
                                > self.acceleration_max_path_velocity(vel_switch.path_pos + EPS));
                    if !keep_scanning {
                        break false;
                    }
                }
            }
        };

        if accel_reached_end && vel_reached_end {
            None
        } else if !accel_reached_end
            && (vel_reached_end || accel_switch.path_pos <= vel_switch.path_pos)
        {
            Some((accel_switch, accel_before, accel_after))
        } else {
            Some((vel_switch, vel_before, vel_after))
        }
    }

    /// Upstream `getNextAccelerationSwitchingPoint`. `None` means the
    /// search ran off the end of the path.
    fn next_acceleration_switching_point(
        &self,
        path_pos: f64,
    ) -> Option<(TrajectoryStep, f64, f64)> {
        let mut switching_path_pos = path_pos;
        loop {
            let (pos, discontinuity) = self.path.next_switching_point(switching_path_pos);
            switching_path_pos = pos;

            if switching_path_pos > self.path.length() - EPS {
                return None;
            }

            if discontinuity {
                let before_path_vel = self.acceleration_max_path_velocity(switching_path_pos - EPS);
                let after_path_vel = self.acceleration_max_path_velocity(switching_path_pos + EPS);
                let switching_path_vel = cxx_min(before_path_vel, after_path_vel);
                let before_acceleration = self.min_max_path_acceleration(
                    switching_path_pos - EPS,
                    switching_path_vel,
                    false,
                );
                let after_acceleration = self.min_max_path_acceleration(
                    switching_path_pos + EPS,
                    switching_path_vel,
                    true,
                );

                if (before_path_vel > after_path_vel
                    || self.min_max_phase_slope(
                        switching_path_pos - EPS,
                        switching_path_vel,
                        false,
                    ) > self
                        .acceleration_max_path_velocity_deriv(switching_path_pos - 2.0 * EPS))
                    && (before_path_vel < after_path_vel
                        || self.min_max_phase_slope(
                            switching_path_pos + EPS,
                            switching_path_vel,
                            true,
                        ) < self
                            .acceleration_max_path_velocity_deriv(switching_path_pos + 2.0 * EPS))
                {
                    return Some((
                        TrajectoryStep::new(switching_path_pos, switching_path_vel),
                        before_acceleration,
                        after_acceleration,
                    ));
                }
            } else {
                let switching_path_vel = self.acceleration_max_path_velocity(switching_path_pos);

                if self.acceleration_max_path_velocity_deriv(switching_path_pos - EPS) < 0.0
                    && self.acceleration_max_path_velocity_deriv(switching_path_pos + EPS) > 0.0
                {
                    return Some((
                        TrajectoryStep::new(switching_path_pos, switching_path_vel),
                        0.0,
                        0.0,
                    ));
                }
            }
        }
    }

    /// Upstream `getNextVelocitySwitchingPoint`. `None` means the search
    /// ran off the end of the path.
    fn next_velocity_switching_point(&self, path_pos: f64) -> Option<(TrajectoryStep, f64, f64)> {
        let mut start = false;
        let mut path_pos = path_pos - VELOCITY_SWITCHING_SCAN_STEP;
        loop {
            path_pos += VELOCITY_SWITCHING_SCAN_STEP;

            if self.min_max_phase_slope(path_pos, self.velocity_max_path_velocity(path_pos), false)
                >= self.velocity_max_path_velocity_deriv(path_pos)
            {
                start = true;
            }

            let keep_scanning = (!start
                || self.min_max_phase_slope(
                    path_pos,
                    self.velocity_max_path_velocity(path_pos),
                    false,
                ) > self.velocity_max_path_velocity_deriv(path_pos))
                && path_pos < self.path.length();
            if !keep_scanning {
                break;
            }
        }

        if path_pos >= self.path.length() {
            return None;
        }

        let mut before_path_pos = path_pos - VELOCITY_SWITCHING_SCAN_STEP;
        let mut after_path_pos = path_pos;
        while after_path_pos - before_path_pos > EPS {
            let mid = (before_path_pos + after_path_pos) / 2.0;
            if self.min_max_phase_slope(mid, self.velocity_max_path_velocity(mid), false)
                > self.velocity_max_path_velocity_deriv(mid)
            {
                before_path_pos = mid;
            } else {
                after_path_pos = mid;
            }
        }

        let before_acceleration = self.min_max_path_acceleration(
            before_path_pos,
            self.velocity_max_path_velocity(before_path_pos),
            false,
        );
        let after_acceleration = self.min_max_path_acceleration(
            after_path_pos,
            self.velocity_max_path_velocity(after_path_pos),
            true,
        );
        Some((
            TrajectoryStep::new(
                after_path_pos,
                self.velocity_max_path_velocity(after_path_pos),
            ),
            before_acceleration,
            after_acceleration,
        ))
    }

    // ---- Forward/backward integration ---------------------------------

    /// Integrate forward from the last step of `self.trajectory` at
    /// `acceleration`, pushing new steps, until either the end of the path
    /// is reached (`true`) or a switching point forces a switch back to
    /// backward integration (`false`). Sets `self.valid = false` on an
    /// unrecoverable failure (also returning `true`, matching upstream).
    ///
    /// Upstream `Trajectory::integrateForward`. Always called with
    /// `self.trajectory` as both the list read from and the list pushed to
    /// (as upstream's own two call sites do — both pass `output.trajectory_`),
    /// so this port takes no separate list parameter, unlike upstream's
    /// signature; upstream's own body reflects the same fact once, at
    /// `getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, ...)`
    /// (`.cpp:680`), which reads the *parameter* `trajectory` and the
    /// *member* `trajectory_` in the same expression — meaningless unless
    /// they are the same object, which at every real call site they are.
    fn integrate_forward(&mut self, mut acceleration: f64) -> bool {
        let last = *self.trajectory.last().expect("initial step always present");
        let mut path_pos = last.path_pos;
        let mut path_vel = last.path_vel;

        let switching_points = self.path.switching_points().to_vec();
        let mut next_discontinuity = 0usize;

        loop {
            while next_discontinuity < switching_points.len()
                && (switching_points[next_discontinuity].0 <= path_pos
                    || !switching_points[next_discontinuity].1)
            {
                next_discontinuity += 1;
            }

            let old_path_pos = path_pos;
            let old_path_vel = path_vel;

            path_vel += self.time_step * acceleration;
            path_pos += self.time_step * 0.5 * (old_path_vel + path_vel);

            if next_discontinuity < switching_points.len()
                && path_pos > switching_points[next_discontinuity].0
            {
                // Avoid a TrajectoryStep with path_pos near a switching
                // point, which would cause an almost-identical step to get
                // added on the next iteration (moveit/moveit#1665).
                if path_pos - switching_points[next_discontinuity].0 < EPS {
                    continue;
                }
                path_vel = old_path_vel
                    + (switching_points[next_discontinuity].0 - old_path_pos)
                        * (path_vel - old_path_vel)
                        / (path_pos - old_path_pos);
                path_pos = switching_points[next_discontinuity].0;
            }

            if path_pos > self.path.length() {
                self.trajectory
                    .push(TrajectoryStep::new(path_pos, path_vel));
                return true;
            } else if path_vel < 0.0 {
                self.valid = false;
                return true;
            }

            if path_vel > self.velocity_max_path_velocity(path_pos)
                && self.min_max_phase_slope(
                    old_path_pos,
                    self.velocity_max_path_velocity(old_path_pos),
                    false,
                ) <= self.velocity_max_path_velocity_deriv(old_path_pos)
            {
                path_vel = self.velocity_max_path_velocity(path_pos);
            }

            self.trajectory
                .push(TrajectoryStep::new(path_pos, path_vel));
            acceleration = self.min_max_path_acceleration(path_pos, path_vel, true);

            if path_vel == 0.0 && acceleration == 0.0 {
                // The position will never change if velocity and
                // acceleration are both zero; nothing would ever satisfy
                // an exit condition below.
                self.valid = false;
                return true;
            }

            if path_vel > self.acceleration_max_path_velocity(path_pos)
                || path_vel > self.velocity_max_path_velocity(path_pos)
            {
                // Find a more accurate intersection with the max-velocity
                // curve by bisection.
                let overshoot = self.trajectory.pop().expect("just pushed");
                let before_step = *self.trajectory.last().expect("initial step always present");
                let mut before = before_step.path_pos;
                let mut before_path_vel = before_step.path_vel;
                let mut after = overshoot.path_pos;
                let mut after_path_vel = overshoot.path_vel;
                while after - before > EPS {
                    let midpoint = 0.5 * (before + after);
                    let mut midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

                    if midpoint_path_vel > self.velocity_max_path_velocity(midpoint)
                        && self.min_max_phase_slope(
                            before,
                            self.velocity_max_path_velocity(before),
                            false,
                        ) <= self.velocity_max_path_velocity_deriv(before)
                    {
                        midpoint_path_vel = self.velocity_max_path_velocity(midpoint);
                    }

                    if midpoint_path_vel > self.acceleration_max_path_velocity(midpoint)
                        || midpoint_path_vel > self.velocity_max_path_velocity(midpoint)
                    {
                        after = midpoint;
                        after_path_vel = midpoint_path_vel;
                    } else {
                        before = midpoint;
                        before_path_vel = midpoint_path_vel;
                    }
                }
                self.trajectory
                    .push(TrajectoryStep::new(before, before_path_vel));

                if self.acceleration_max_path_velocity(after)
                    < self.velocity_max_path_velocity(after)
                {
                    // Upstream dereferences `next_discontinuity` here
                    // unconditionally, even when it is `switching_points.end()`
                    // (technically UB, though harmless in practice with a
                    // `std::list` end sentinel). This port instead treats
                    // "no next discontinuity" as "`after` cannot be past
                    // one", which is the only sound reading of the
                    // comparison when there isn't one.
                    if next_discontinuity < switching_points.len()
                        && after > switching_points[next_discontinuity].0
                    {
                        return false;
                    }
                    let last = *self.trajectory.last().expect("just pushed");
                    if self.min_max_phase_slope(last.path_pos, last.path_vel, true)
                        > self.acceleration_max_path_velocity_deriv(last.path_pos)
                    {
                        return false;
                    }
                } else {
                    let last = *self.trajectory.last().expect("just pushed");
                    if self.min_max_phase_slope(last.path_pos, last.path_vel, false)
                        > self.velocity_max_path_velocity_deriv(last.path_pos)
                    {
                        return false;
                    }
                }
            }
        }
    }

    /// Integrate backward from `(path_pos, path_vel)` at `acceleration`
    /// until the backward trajectory intersects `self.trajectory`, then
    /// truncate `self.trajectory` at the intersection and append the
    /// backward-integrated steps. Sets `self.valid = false` if the two
    /// trajectories never intersect.
    ///
    /// Upstream `Trajectory::integrateBackward`. Same aliasing note as
    /// [`Trajectory::integrate_forward`] applies (`start_trajectory` is
    /// always `output.trajectory_` at both call sites), so this port
    /// operates on `self.trajectory` directly rather than taking a
    /// parameter.
    ///
    /// # `std::list::push_front` → `Vec::push` + reverse
    ///
    /// Upstream builds its local backward trajectory with `push_front`, so
    /// `trajectory.front()` always names the step most recently added —
    /// the smallest `path_pos` reached *so far* in the backward walk. This
    /// port instead `push`es (appends) to a plain `Vec`, so the same
    /// "most recently added" step is `trajectory.last()` throughout the
    /// walk; only once the walk ends (at the `return` on finding an
    /// intersection) is the accumulated `Vec` reversed before extending
    /// `self.trajectory`, restoring ascending-`path_pos` order. This
    /// changes nothing observable — every value the loop body reads or
    /// writes is expressed relative to "most recently added", not to a
    /// fixed index — and avoids `O(n)` per-step front-insertion.
    fn integrate_backward(&mut self, mut path_pos: f64, mut path_vel: f64, mut acceleration: f64) {
        let mut start2 = self.trajectory.len() - 1;
        let mut start1 = start2 - 1;
        let mut backward: Vec<TrajectoryStep> = Vec::new();
        let mut slope = 0.0;
        debug_assert!(self.trajectory[start1].path_pos <= path_pos);

        loop {
            if !(start1 != 0 || path_pos >= 0.0) {
                break;
            }

            if self.trajectory[start1].path_pos <= path_pos {
                backward.push(TrajectoryStep::new(path_pos, path_vel));
                let most_recent = *backward.last().expect("just pushed");
                path_vel -= self.time_step * acceleration;
                path_pos -= self.time_step * 0.5 * (path_vel + most_recent.path_vel);
                acceleration = self.min_max_path_acceleration(path_pos, path_vel, false);
                slope = (most_recent.path_vel - path_vel) / (most_recent.path_pos - path_pos);

                if path_vel < 0.0 {
                    self.valid = false;
                    return;
                }
            } else {
                start1 -= 1;
                start2 -= 1;
            }

            let s1 = self.trajectory[start1];
            let s2 = self.trajectory[start2];
            let start_slope = (s2.path_vel - s1.path_vel) / (s2.path_pos - s1.path_pos);
            let intersection_path_pos = (s1.path_vel - path_vel + slope * path_pos
                - start_slope * s1.path_pos)
                / (slope - start_slope);
            let most_recent = *backward
                .last()
                .expect("first iteration always takes the if-branch above");
            if s1.path_pos.max(path_pos) - EPS <= intersection_path_pos
                && intersection_path_pos <= EPS + s2.path_pos.min(most_recent.path_pos)
            {
                let intersection_path_vel =
                    s1.path_vel + start_slope * (intersection_path_pos - s1.path_pos);
                self.trajectory.truncate(start2);
                self.trajectory.push(TrajectoryStep::new(
                    intersection_path_pos,
                    intersection_path_vel,
                ));
                backward.reverse();
                self.trajectory.extend(backward);
                return;
            }
        }

        self.valid = false;
    }

    // ---- Per-position velocity/acceleration limits ---------------------

    /// Upstream `getMinMaxPathAcceleration`.
    fn min_max_path_acceleration(&self, path_pos: f64, path_vel: f64, max: bool) -> f64 {
        let config_deriv = self.path.tangent(path_pos);
        let config_deriv2 = self.path.curvature(path_pos);
        let factor = if max { 1.0 } else { -1.0 };
        let mut max_path_acceleration = f64::MAX;
        for i in 0..self.joint_num {
            if config_deriv[i] != 0.0 {
                max_path_acceleration = cxx_min(
                    max_path_acceleration,
                    self.max_acceleration[i] / config_deriv[i].abs()
                        - factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i],
                );
            }
        }
        factor * max_path_acceleration
    }

    /// Upstream `getMinMaxPhaseSlope`.
    fn min_max_phase_slope(&self, path_pos: f64, path_vel: f64, max: bool) -> f64 {
        self.min_max_path_acceleration(path_pos, path_vel, max) / path_vel
    }

    /// Upstream `getAccelerationMaxPathVelocity`.
    fn acceleration_max_path_velocity(&self, path_pos: f64) -> f64 {
        let mut max_path_velocity = f64::INFINITY;
        let config_deriv = self.path.tangent(path_pos);
        let config_deriv2 = self.path.curvature(path_pos);
        for i in 0..self.joint_num {
            if config_deriv[i] != 0.0 {
                for j in (i + 1)..self.joint_num {
                    if config_deriv[j] != 0.0 {
                        let a_ij =
                            config_deriv2[i] / config_deriv[i] - config_deriv2[j] / config_deriv[j];
                        if a_ij != 0.0 {
                            max_path_velocity = cxx_min(
                                max_path_velocity,
                                ((self.max_acceleration[i] / config_deriv[i].abs()
                                    + self.max_acceleration[j] / config_deriv[j].abs())
                                    / a_ij.abs())
                                .sqrt(),
                            );
                        }
                    }
                }
            } else if config_deriv2[i] != 0.0 {
                max_path_velocity = cxx_min(
                    max_path_velocity,
                    (self.max_acceleration[i] / config_deriv2[i].abs()).sqrt(),
                );
            }
        }
        max_path_velocity
    }

    /// Upstream `getVelocityMaxPathVelocity`.
    fn velocity_max_path_velocity(&self, path_pos: f64) -> f64 {
        let tangent = self.path.tangent(path_pos);
        let mut max_path_velocity = f64::MAX;
        for i in 0..self.joint_num {
            max_path_velocity = cxx_min(max_path_velocity, self.max_velocity[i] / tangent[i].abs());
        }
        max_path_velocity
    }

    /// Upstream `getAccelerationMaxPathVelocityDeriv`: a central-difference
    /// numerical derivative of [`Trajectory::acceleration_max_path_velocity`].
    fn acceleration_max_path_velocity_deriv(&self, path_pos: f64) -> f64 {
        (self.acceleration_max_path_velocity(path_pos + EPS)
            - self.acceleration_max_path_velocity(path_pos - EPS))
            / (2.0 * EPS)
    }

    /// Upstream `getVelocityMaxPathVelocityDeriv`: the analytic derivative
    /// of [`Trajectory::velocity_max_path_velocity`] with respect to
    /// `path_pos`, taken along whichever joint is the active (binding)
    /// velocity constraint at `path_pos`.
    fn velocity_max_path_velocity_deriv(&self, path_pos: f64) -> f64 {
        let tangent = self.path.tangent(path_pos);
        let mut max_path_velocity = f64::MAX;
        let mut active_constraint = 0usize;
        for i in 0..self.joint_num {
            let this_max_path_velocity = self.max_velocity[i] / tangent[i].abs();
            if this_max_path_velocity < max_path_velocity {
                max_path_velocity = this_max_path_velocity;
                active_constraint = i;
            }
        }
        let curvature = self.path.curvature(path_pos);
        -(self.max_velocity[active_constraint] * curvature[active_constraint])
            / (tangent[active_constraint] * tangent[active_constraint].abs())
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::path::DEFAULT_PATH_TOLERANCE;

    fn v(values: &[f64]) -> DVector<f64> {
        DVector::from_vec(values.to_vec())
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, test1)`.
    ///
    /// # Tolerance
    ///
    /// Upstream checks `getDuration()` and every `getPosition()` component
    /// with `EXPECT_DOUBLE_EQ` (near bit-exact, ~4 ULP) -- not
    /// `EXPECT_NEAR` with a loose epsilon. An earlier round of this port
    /// substituted a norm-of-difference `assert_relative_eq!` with
    /// `epsilon = 1e-6`/`1e-9` for the duration/position checks, which is
    /// both looser than upstream's actual tolerance and a different (weaker)
    /// invariant: a norm bound only constrains the aggregate error, not each
    /// component (PORTING-PLAN.md §78.1/§79's "transcribe the numerics
    /// rather than rewriting them into something cleaner" -- this had been
    /// rewritten). A `to_bits()`-based ULP measurement found this port's
    /// `duration()`/`position()` are bit-identical to upstream's own
    /// `EXPECT_DOUBLE_EQ` literals for this case (0 ULP), so both are now
    /// `assert_eq!`, per-component for position, matching upstream's actual
    /// test shape.
    #[test]
    fn upstream_test1() {
        let waypoints = [
            v(&[1424.0, 984.999_694_824_219, 2126.0, 0.0]),
            v(&[1423.0, 985.000_244_140_625, 2126.0, 0.0]),
        ];
        let max_velocity = v(&[1.3, 0.67, 0.67, 0.5]);
        let max_acceleration = v(&[0.00249, 0.00249, 0.00249, 0.00249]);

        let path = Path::create(&waypoints, 100.0).unwrap();
        let trajectory = Trajectory::create(path, &max_velocity, &max_acceleration, 10.0).unwrap();

        assert_eq!(trajectory.duration(), 40.080_256_821_829_85);

        let start = trajectory.position(0.0);
        for i in 0..4 {
            assert_eq!(start[i], waypoints[0][i]);
        }

        let end = trajectory.position(trajectory.duration());
        for i in 0..4 {
            assert_eq!(end[i], waypoints[1][i]);
        }

        assert_relative_eq!(trajectory.velocity(0.0)[0], 0.0, epsilon = 0.1);
        assert_relative_eq!(
            trajectory.velocity(trajectory.duration())[0],
            0.0,
            epsilon = 0.1
        );
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, test2)`.
    ///
    /// # Tolerance
    ///
    /// Same `EXPECT_DOUBLE_EQ` fidelity issue as [`upstream_test1`] (see its
    /// doc comment): position start/end are bit-identical to upstream (0
    /// ULP) and are now per-component `assert_eq!`. `duration()` is the one
    /// exception in this whole file -- unlike `test1`/`test3`, it is
    /// genuinely *not* bit-exact against upstream's literal: measured diff
    /// `8.893e-9` (`1922.14184275348748` actual vs `1922.1418427445944`
    /// expected), the same ~4-order-of-magnitude floor
    /// `totg_robot_trajectory_parity.rs`'s duration group and
    /// `totg_parity.rs`'s case-4 duration measured independently against
    /// the oracle, so this is this port's genuine numeric behaviour on this
    /// waypoint set, not noise from this specific test. `epsilon = 1e-6`
    /// (~2 orders of headroom over the measured floor) predates this round
    /// and was already tight enough to have caught a masked
    /// `max_relative`-only pass. `max_relative` is pinned to `f64::EPSILON`
    /// explicitly -- i.e. the same value it would silently default to --
    /// rather than to `epsilon` the way the sibling parity files in this
    /// sweep pin it: those files' compared magnitudes stay near `1`, so
    /// `max_relative = epsilon` barely changes the effective bound, but
    /// `duration` here is `~1922`, and pinning `max_relative = 1e-6` would
    /// widen the *effective* tolerance to `1e-6 * 1922 ≈ 1.9e-3` -- roughly
    /// 1900x looser than `epsilon` alone, discovered live by perturbing
    /// this test with a `+1e-5` offset and watching it keep passing instead
    /// of failing. At `f64::EPSILON`, the relative branch contributes at
    /// most `f64::EPSILON * 1922 ≈ 4.3e-13` -- six orders below `epsilon`
    /// -- so it is provably inert here, matching the pre-existing
    /// (undocumented) behaviour without the magnitude trap.
    #[test]
    fn upstream_test2() {
        let waypoints = [
            v(&[1427.0, 368.0, 690.0, 90.0]),
            v(&[1427.0, 368.0, 790.0, 90.0]),
            v(&[952.499_938_964_844, 433.0, 1051.0, 90.0]),
            v(&[452.5, 533.0, 1051.0, 90.0]),
            v(&[452.5, 533.0, 951.0, 90.0]),
        ];
        let max_velocity = v(&[1.3, 0.67, 0.67, 0.5]);
        let max_acceleration = v(&[0.002, 0.002, 0.002, 0.002]);

        let path = Path::create(&waypoints, 100.0).unwrap();
        let trajectory = Trajectory::create(path, &max_velocity, &max_acceleration, 10.0).unwrap();

        assert_relative_eq!(
            trajectory.duration(),
            1_922.141_842_744_594_4,
            epsilon = 1e-6,
            max_relative = f64::EPSILON
        );

        let start = trajectory.position(0.0);
        for i in 0..4 {
            assert_eq!(start[i], waypoints[0][i]);
        }

        let end = trajectory.position(trajectory.duration());
        let last_waypoint = waypoints.last().unwrap();
        for i in 0..4 {
            assert_eq!(end[i], last_waypoint[i]);
        }

        assert_relative_eq!(trajectory.velocity(0.0)[0], 0.0, epsilon = 0.1);
        assert_relative_eq!(
            trajectory.velocity(trajectory.duration())[0],
            0.0,
            epsilon = 0.1
        );
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, test3)`: identical
    /// to `test1`/`test2` except it exercises upstream's default
    /// `time_step = 0.001` (this port has no default-argument equivalent, so
    /// the literal is spelled out instead).
    ///
    /// # Tolerance
    ///
    /// Same `EXPECT_DOUBLE_EQ` fidelity issue as [`upstream_test1`] (see its
    /// doc comment); unlike [`upstream_test2`], `duration()` here is also
    /// bit-identical to upstream (0 ULP against `1919.5597888812974`) --
    /// the smaller `time_step` apparently avoids whatever accumulates
    /// `test2`'s `8.893e-9` divergence. Both duration and position are now
    /// `assert_eq!`.
    #[test]
    fn upstream_test3() {
        let waypoints = [
            v(&[1427.0, 368.0, 690.0, 90.0]),
            v(&[1427.0, 368.0, 790.0, 90.0]),
            v(&[952.499_938_964_844, 433.0, 1051.0, 90.0]),
            v(&[452.5, 533.0, 1051.0, 90.0]),
            v(&[452.5, 533.0, 951.0, 90.0]),
        ];
        let max_velocity = v(&[1.3, 0.67, 0.67, 0.5]);
        let max_acceleration = v(&[0.002, 0.002, 0.002, 0.002]);

        let path = Path::create(&waypoints, 100.0).unwrap();
        let trajectory = Trajectory::create(path, &max_velocity, &max_acceleration, 0.001).unwrap();

        assert_eq!(trajectory.duration(), 1_919.559_788_881_297_4);

        let start = trajectory.position(0.0);
        for i in 0..4 {
            assert_eq!(start[i], waypoints[0][i]);
        }

        let end = trajectory.position(trajectory.duration());
        let last_waypoint = waypoints.last().unwrap();
        for i in 0..4 {
            assert_eq!(end[i], last_waypoint[i]);
        }

        assert_relative_eq!(trajectory.velocity(0.0)[0], 0.0, epsilon = 0.1);
        assert_relative_eq!(
            trajectory.velocity(trajectory.duration())[0],
            0.0,
            epsilon = 0.1
        );
    }

    // Upstream `TEST(time_optimal_trajectory_generation, testLargeAccel)` is
    // ported as `tests/large_accel.rs`, not here: its fixture data is
    // upstream's own test literals at full `f64` precision, and one of them
    // is (coincidentally — see the fixture's `source` field and that test
    // file's doc comment) 4550 ULPs from `FRAC_PI_4`, close enough that
    // `clippy::approx_constant` fires on the literal. Loading the data from
    // a committed JSON fixture removes the literal clippy was matching
    // entirely, rather than disguising it from the lint.

    /// Upstream `TEST(time_optimal_trajectory_generation, AccelerationLimitIsRespected)`.
    #[test]
    fn upstream_acceleration_limit_is_respected() {
        let path_tolerance = 0.001;
        let resample_dt = 0.01;
        let waypoints = [
            v(&[0.0, 0.0, 0.0]),
            v(&[1.0, 0.0, 0.0]),
            v(&[1.0, 1.0, 0.0]),
        ];
        let max_velocity = v(&[0.1, 0.1, 0.1]);
        let max_acceleration = v(&[0.5, 0.5, 0.5]);

        let path = Path::create(&waypoints, path_tolerance).unwrap();
        let trajectory = Trajectory::create(path, &max_velocity, &max_acceleration, 0.001).unwrap();

        let sample_count = (trajectory.duration() / resample_dt).ceil() as u64;
        let mut previous_velocity = v(&[0.0, 0.0, 0.0]);
        for sample in 0..=sample_count {
            let t = cxx_min(trajectory.duration(), sample as f64 * resample_dt);
            let velocity = trajectory.velocity(t);
            let acceleration = (&velocity - &previous_velocity).norm() / resample_dt;
            assert!(
                acceleration < max_acceleration.norm() + 1e-3,
                "sample {sample}: {acceleration}"
            );
            previous_velocity = velocity;
        }
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, testSingleDofDiscontinuity)`.
    ///
    /// # Tolerance
    ///
    /// `getPosition(0.0)[0]` is upstream's one `EXPECT_DOUBLE_EQ` in this
    /// test (everything else here is `EXPECT_NEAR` with an upstream-chosen
    /// literal epsilon, already faithfully transcribed with matching
    /// values). Measured bit-identical to upstream (0 ULP against
    /// `start_position`), so it is now `assert_eq!` rather than the
    /// `assert_relative_eq!(..., epsilon = 1e-9)` an earlier round
    /// substituted -- see [`upstream_test1`]'s doc comment for the same
    /// fidelity issue found across this file.
    #[test]
    fn upstream_test_single_dof_discontinuity() {
        let start_position = 1.881_943;
        let waypoints = [v(&[start_position]), v(&[2.600_542])];
        let max_velocity = v(&[4.54]);
        let max_acceleration = v(&[28.0]);

        let path = Path::create(&waypoints, 0.1).unwrap();
        let trajectory = Trajectory::create(path, &max_velocity, &max_acceleration, 0.001).unwrap();

        assert!(trajectory.duration() > 0.0);
        let traj_duration = trajectory.duration();
        assert_relative_eq!(traj_duration, 0.320_681, epsilon = 1e-3);

        assert_eq!(trajectory.position(0.0)[0], start_position);
        assert_relative_eq!(trajectory.velocity(0.0)[0], 0.0, epsilon = 0.1);
        assert_relative_eq!(trajectory.velocity(traj_duration)[0], 0.0, epsilon = 0.1);

        let t_switch = 0.160_340_7;
        let mut time = 0.0;
        while time < traj_duration {
            if time < t_switch {
                assert_relative_eq!(
                    trajectory.acceleration(time)[0],
                    max_acceleration[0],
                    epsilon = 1e-3
                );
            } else if time > t_switch {
                assert_relative_eq!(
                    trajectory.acceleration(time)[0],
                    -max_acceleration[0],
                    epsilon = 1e-3
                );
            }
            time += 0.01;
        }
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, testRelevantZeroMaxAccelerationsInvalidateTrajectory)`.
    #[test]
    fn upstream_test_relevant_zero_max_accelerations_invalidate_trajectory() {
        let max_velocity = v(&[1.0, 1.0]);
        let waypoints = [v(&[0.0, 0.0]), v(&[1.0, 1.0])];
        let path = Path::create(&waypoints, DEFAULT_PATH_TOLERANCE).unwrap();

        assert!(Trajectory::create(path.clone(), &max_velocity, &v(&[0.0, 1.0]), 0.001).is_err());
        assert!(Trajectory::create(path.clone(), &max_velocity, &v(&[1.0, 0.0]), 0.001).is_err());
        assert!(Trajectory::create(path, &max_velocity, &v(&[0.0, 0.0]), 0.001).is_err());
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, testIrrelevantZeroMaxAccelerationsDontInvalidateTrajectory)`.
    #[test]
    fn upstream_test_irrelevant_zero_max_accelerations_dont_invalidate_trajectory() {
        let max_velocity = v(&[1.0, 1.0]);

        let path = Path::create(&[v(&[0.0, 0.0]), v(&[0.0, 1.0])], DEFAULT_PATH_TOLERANCE).unwrap();
        assert!(Trajectory::create(path, &max_velocity, &v(&[0.0, 1.0]), 0.001).is_ok());

        let path = Path::create(&[v(&[0.0, 0.0]), v(&[1.0, 0.0])], DEFAULT_PATH_TOLERANCE).unwrap();
        assert!(Trajectory::create(path, &max_velocity, &v(&[1.0, 0.0]), 0.001).is_ok());
    }

    /// Upstream `TEST(time_optimal_trajectory_generation, testTimeStepZeroMakesTrajectoryInvalid)`.
    #[test]
    fn upstream_test_time_step_zero_makes_trajectory_invalid() {
        let path = Path::create(&[v(&[0.0, 0.0]), v(&[1.0, 1.0])], DEFAULT_PATH_TOLERANCE).unwrap();
        assert!(Trajectory::create(path, &v(&[1.0, 1.0]), &v(&[1.0, 1.0]), 0.0).is_err());
    }

    // ---- Boundary-condition tests (not from the upstream suite) --------

    #[test]
    fn two_waypoints_only_produces_a_valid_trajectory() {
        let path = Path::create(&[v(&[0.0]), v(&[1.0])], DEFAULT_PATH_TOLERANCE).unwrap();
        let trajectory = Trajectory::create(path, &v(&[1.0]), &v(&[1.0]), 0.001).unwrap();
        assert!(trajectory.duration() > 0.0);
    }

    /// A boundary this test exists specifically to record: unlike a `0.0`
    /// **acceleration** limit on a dimension the path needs (see
    /// `upstream_test_relevant_zero_max_accelerations_invalidate_trajectory`,
    /// which does invalidate), a `0.0` **velocity** limit on such a
    /// dimension does *not* invalidate the trajectory.
    ///
    /// [`Trajectory::velocity_max_path_velocity`] pins the path-velocity
    /// ceiling to `0.0` everywhere the zero-limited joint has a nonzero
    /// tangent. [`Trajectory::integrate_forward`] still computes a nonzero
    /// [`Trajectory::min_max_path_acceleration`] there (acceleration is
    /// unconstrained; only velocity is `0`), so the `path_vel == 0.0 &&
    /// acceleration == 0.0` deadlock guard never fires. Instead, every
    /// integration step overshoots the `0.0` ceiling and gets pulled back
    /// by the overshoot-correction bisection to within `EPS` (`1e-6`) of
    /// the ceiling — advancing `path_pos` by one `EPS`-scale increment per
    /// step, forever, since the ceiling is `0.0` at every subsequent
    /// `path_pos` too. The trajectory is technically still constructed
    /// (`valid` never becomes `false`, so this is `Ok`, not the
    /// `testRelevantZeroMaxAccelerationsInvalidateTrajectory`-style
    /// failure), but each step's `path_vel` stays exactly `0.0`, so the
    /// timing pass divides a position delta by `(0.0 + 0.0) / 2.0 = 0.0` —
    /// landing on `+inf` when the delta is a nonzero float, or on NaN when
    /// floating-point cancellation at the path's particular scale rounds
    /// the terminal delta itself to exactly `0.0` (both were observed —
    /// see the test body).
    ///
    /// This crawl needs on the order of `path.length() / EPS` steps, so
    /// this test uses a `1e-5`-scale path rather than the `1.0`-scale one
    /// used elsewhere in this module — the same crawl over a `1.0`-scale
    /// path is real, verified behaviour (traced by hand and confirmed by
    /// running it), but takes on the order of two million steps and tens
    /// of seconds, which does not belong in a unit test suite.
    #[test]
    fn a_max_velocity_component_of_zero_crawls_rather_than_invalidating() {
        let path =
            Path::create(&[v(&[0.0, 0.0]), v(&[1e-5, 1e-5])], DEFAULT_PATH_TOLERANCE).unwrap();
        let trajectory = Trajectory::create(path, &v(&[0.0, 1.0]), &v(&[1.0, 1.0]), 0.001).unwrap();
        // Not a sane finite duration: whether the terminal 0.0/0.0-scale
        // cancellation lands on NaN or on +inf is sensitive to the path's
        // absolute scale (a 1.0-scale path lands on +inf; this 1e-5-scale
        // one lands on NaN) — both were observed, and neither is "more
        // correct" than the other, so this only asserts the shared,
        // scale-independent property: the crawl never produces a normal
        // finite answer.
        assert!(!trajectory.duration().is_finite());
    }

    /// A path of two identical waypoints has `length() == 0.0`, and its
    /// single [`crate::path_segment::PathSegment`] has a NaN tangent —
    /// `(end - start) / length` computes `0.0 / 0.0` — for the same reason
    /// Eigen's element-wise division would: neither upstream nor this port
    /// special-cases a zero-length segment's tangent.
    ///
    /// That NaN does not panic and is not silently discarded: it feeds
    /// [`Trajectory::min_max_path_acceleration`]'s `config_deriv[i] !=
    /// 0.0` check, which is `true` for NaN (NaN compares unequal to
    /// everything), so the NaN candidate reaches `cxx_min`. `cxx_min`'s
    /// asymmetric NaN handling then *discards* it (NaN is always the
    /// second argument at that call site), so the accumulator is left at
    /// its untouched seed, `f64::MAX`. That happens on the way both
    /// forward (`factor = 1.0`) and backward (`factor = -1.0`), so
    /// `integrate_forward` takes one step at (effectively) `+f64::MAX`
    /// acceleration and immediately overshoots the zero-length path, and
    /// the subsequent `integrate_backward` pass takes one step at
    /// (effectively) `-f64::MAX` and finds an intersection almost exactly
    /// back at the start — landing on a second [`TrajectoryStep`] whose
    /// `path_vel` is (as good as) `0.0` again. The final timing pass then
    /// divides by `(0.0 + 0.0) / 2.0`, producing a NaN `time` and hence a
    /// NaN [`Trajectory::duration`].
    ///
    /// This is not a defect introduced by this port: the same 0/0 divide,
    /// the same NaN-discarding `std::min`/`std::max`, and the same
    /// zero-over-zero in the timing loop (`(current.path_pos_ -
    /// previous.path_pos_) / ((current.path_vel_ + previous.path_vel_) /
    /// 2)`) exist verbatim in upstream's C++. `Path::create`/
    /// `Trajectory::create` have no length-zero guard in upstream either,
    /// so this is upstream's actual, documented-by-tracing-through-it
    /// behaviour for this degenerate input, transcribed rather than
    /// papered over.
    #[test]
    fn a_zero_length_path_produces_a_nan_duration_trajectory() {
        let p = v(&[5.0, 5.0]);
        let path = Path::create(&[p.clone(), p.clone()], DEFAULT_PATH_TOLERANCE).unwrap();
        assert_eq!(path.length(), 0.0);
        assert!(path.tangent(0.0).iter().all(|x| x.is_nan()));

        let trajectory = Trajectory::create(path, &v(&[1.0, 1.0]), &v(&[1.0, 1.0]), 0.001).unwrap();
        assert!(trajectory.duration().is_nan());
    }
}
