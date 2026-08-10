// Copyright (c) 2021, PickNik Robotics
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/ruckig_traj_smoothing.hpp
//   moveit_core/trajectory_processing/src/ruckig_traj_smoothing.cpp

//! Time-parameterize a [`RobotTrajectory`] so it also satisfies jerk limits,
//! by re-running it through the `ruckig` online trajectory generator
//! (offline / one-shot mode) one segment at a time.
//!
//! Upstream `trajectory_processing::RuckigSmoothing`, a static-method-only
//! class; this port is free functions instead, since Rust has no use for an
//! empty marker type to hang associated functions off of.
//!
//! # Out of scope
//!
//! - The `applySmoothing(..., const std::vector<moveit_msgs::msg::JointLimits>&, ...)`
//!   overload — a `moveit_msgs` conversion, out of scope per `PORTING-PLAN.md`
//!   D1. It is a thin wrapper around the two overloads ported here
//!   ([`apply_smoothing`] and [`apply_smoothing_with_limits`]) that unpacks a
//!   `JointLimits` message into the same `velocity_limits`/
//!   `acceleration_limits`/`jerk_limits` maps [`apply_smoothing_with_limits`]
//!   already takes, so nothing behavioural is lost by skipping it.
//! - `ruckig_filter.cpp`'s `RuckigFilterPlugin` (a different upstream file,
//!   `moveit_core/online_signal_smoothing`) — a `pluginlib`/`rclcpp::Node`
//!   online smoothing plugin coupled to ROS and a generated `Params` struct
//!   throughout, operating on raw `Eigen::VectorXd`, not a `RobotTrajectory`.
//!   Unrelated to this file beyond sharing the `ruckig` dependency.
//!
//! # Deviations from upstream
//!
//! - **No default parameters.** Upstream defaults
//!   `max_velocity_scaling_factor`/`max_acceleration_scaling_factor` to
//!   `1.0`, `mitigate_overshoot` to `false`, and `overshoot_threshold` to
//!   `0.01`; Rust has no default parameters, so these four are grouped into
//!   [`SmoothingOptions`] instead, whose [`Default`] impl matches the
//!   upstream defaults (`SmoothingOptions { max_velocity_scaling_factor: 0.5,
//!   ..Default::default() }` reproduces a call that only overrode one
//!   parameter in C++). This also keeps both `apply_smoothing*` functions
//!   under clippy's argument-count lint without an `#[allow(...)]`.
//! - **`getRobotModelBounds` always succeeds.** Upstream's `[[nodiscard]]
//!   bool getRobotModelBounds(...)` has no path that returns `false` — every
//!   branch, bounded or not, assigns a value and falls through. This port
//!   drops the dead return value and makes `set_robot_model_bounds`
//!   infallible.
//! - **The `RCLCPP_WARN_STREAM_ONCE`/`RCLCPP_ERROR*` logging calls are not
//!   ported.** They are diagnostics with no effect on the computed
//!   trajectory; `cspace_core::trajectory` has no logging dependency to route them
//!   through, and the "using the default N" warnings upstream logs at most
//!   once per process are not observable behaviour this crate's tests could
//!   assert on. `apply_smoothing`/`apply_smoothing_with_limits` still return
//!   [`Error`] in the two cases upstream would have logged
//!   `RCLCPP_ERROR`+`return false` (missing group; Ruckig failing even after
//!   the retry loop's maximum duration extension).
//! - **`extendTrajectoryDuration`'s header doc comment does not match its
//!   `.cpp` definition, and this port follows the `.cpp` definition.** The
//!   header declares `size_t num_waypoints` as the second parameter and
//!   documents the function as extending "the duration of every trajectory
//!   segment"; the `.cpp` definition names that same parameter
//!   `waypoint_idx` and the function extends only the single segment ending
//!   at `waypoint_idx + 1`. `extend_trajectory_duration` here is named and
//!   shaped after the `.cpp` definition (the code that actually runs), and
//!   its one call site in `run_ruckig` confirms the single-segment
//!   behaviour: `waypoint_idx` there is the specific segment that just
//!   failed or overshot, not a count.
//! - **`getVariableIndexList()` (a global per-DOF integer index into the
//!   whole `RobotState`) is replaced by iterating
//!   `JointModelGroup::variable_names()` and addressing each `RobotState`
//!   variable by name.** [`crate::model::JointModelGroup`] has no
//!   index-list-returning method in this workspace (out of scope for this
//!   crate to add), and this crate's [`RobotState`] accessors are
//!   name-based. The two are equivalent: [`JointModelGroup::variable_names`]
//!   and upstream's `getVariableIndexList()` both walk the same
//!   `joint_indices()`-then-per-joint-variables order (checked against
//!   `JointModelGroup`'s own constructor), so DOF `j` names the same
//!   variable both ways.
//! - **Ruckig's `calculate()` failure vs. `Result<RuckigResult>` value.**
//!   `rsruckig::Ruckig::calculate` returns
//!   `Result<RuckigResult, RuckigError>`: `Err` for malformed input (a bug
//!   in this port, since every input here is built from a valid
//!   `RobotState`/`JointModelGroup`) and `Ok(RuckigResult::ErrorXxx)` for
//!   every *infeasible-trajectory* condition upstream's retry loop is built
//!   to react to. `run_ruckig` uses `IgnoreErrorHandler` (matching
//!   upstream's undecorated `ruckig::Ruckig<ruckig::DynamicDOFs>`, which
//!   never throws) and, like upstream, drives the retry loop off the
//!   `Ok(RuckigResult)` value; an `Err` is treated as
//!   [`Error::other`] rather than a panic, since it should not occur but a
//!   third-party library returning it is not this crate's bug to hide.
//! - **`checkOvershoot` only requests the resampled position.** Upstream
//!   also asks `ruckig::Trajectory::at_time` for velocity and acceleration
//!   but never reads either back; `check_overshoot` passes `None` for
//!   both, which changes no computed value.

use std::collections::HashMap;

use rsruckig::error::IgnoreErrorHandler;
use rsruckig::input_parameter::InputParameter;
use rsruckig::result::RuckigResult;
use rsruckig::ruckig::Ruckig;
use rsruckig::trajectory::Trajectory as RuckigTrajectory;
use rsruckig::util::DataArrayOrVec;

use crate::error::{Error, Result};
use crate::model::{JointModelGroup, RobotModel};
use crate::state::RobotState;

use crate::trajectory::numeric::cxx_clamp;
use crate::trajectory::robot_trajectory::RobotTrajectory;

/// `DEFAULT_MAX_VELOCITY`, rad/s: used for a DOF whose `VariableBounds`
/// leaves velocity unbounded.
const DEFAULT_MAX_VELOCITY: f64 = 5.0;
/// `DEFAULT_MAX_ACCELERATION`, rad/s^2.
const DEFAULT_MAX_ACCELERATION: f64 = 10.0;
/// `DEFAULT_MAX_JERK`, rad/s^3.
const DEFAULT_MAX_JERK: f64 = 1000.0;
/// `MAX_DURATION_EXTENSION_FACTOR`: [`run_ruckig`]'s retry loop gives up once
/// the accumulated duration extension exceeds this.
const MAX_DURATION_EXTENSION_FACTOR: f64 = 50.0;
/// `DURATION_EXTENSION_FRACTION`: the extension factor is multiplied by this
/// on every retry.
const DURATION_EXTENSION_FRACTION: f64 = 1.1;
/// `OVERSHOOT_CHECK_PERIOD`, sec: [`check_overshoot`]'s resampling interval.
const OVERSHOOT_CHECK_PERIOD: f64 = 0.01;

/// The four trailing, independently-defaulted parameters every upstream
/// `applySmoothing` overload takes. See the module-level "Deviations from
/// upstream" note on "No default parameters".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SmoothingOptions {
    /// A factor in the range `[0, 1]` which can slow down the trajectory.
    pub max_velocity_scaling_factor: f64,
    /// A factor in the range `[0, 1]` which can slow down the trajectory.
    pub max_acceleration_scaling_factor: f64,
    /// If `true`, overshoot is mitigated by extending trajectory duration.
    pub mitigate_overshoot: bool,
    /// If an overshoot is greater than this, duration is extended (radians,
    /// for a single joint).
    pub overshoot_threshold: f64,
}

impl Default for SmoothingOptions {
    /// Matches every upstream overload's default parameter values.
    fn default() -> Self {
        Self {
            max_velocity_scaling_factor: 1.0,
            max_acceleration_scaling_factor: 1.0,
            mitigate_overshoot: false,
            overshoot_threshold: 0.01,
        }
    }
}

/// `RuckigSmoothing::applySmoothing` (the scaling-factors-only overload).
///
/// Re-parameterizes `trajectory` in place so every segment also satisfies
/// jerk limits, using velocity/acceleration/jerk bounds read from
/// `trajectory`'s own `RobotModel` (via its `JointModelGroup`), scaled by
/// `options.max_velocity_scaling_factor`/`options.max_acceleration_scaling_factor`.
///
/// A trajectory with fewer than 2 waypoints is returned unmodified — there
/// is nothing to smooth.
///
/// # Errors
///
/// [`Error`] if `trajectory` has no group set, or if Ruckig cannot find a
/// feasible re-parameterization even after extending the duration up to
/// `MAX_DURATION_EXTENSION_FACTOR`.
pub fn apply_smoothing(
    trajectory: &mut RobotTrajectory<'_>,
    options: &SmoothingOptions,
) -> Result<()> {
    let group = validate_group(trajectory)?;
    if trajectory.way_point_count() < 2 {
        return Ok(());
    }

    let model = trajectory.robot_model();
    let num_dof = group.variable_names().len();
    let mut input = InputParameter::<0>::new(Some(num_dof));
    set_robot_model_bounds(
        &mut input,
        model,
        group,
        options.max_velocity_scaling_factor,
        options.max_acceleration_scaling_factor,
    );

    run_ruckig(trajectory, group, input, options)
}

/// `RuckigSmoothing::applySmoothing` (the explicit per-joint limits overload).
///
/// Like [`apply_smoothing`], but every joint variable named in
/// `velocity_limits`/`acceleration_limits`/`jerk_limits` overrides the
/// corresponding `RobotModel` bound (still scaled by
/// `options.max_velocity_scaling_factor`/`options.max_acceleration_scaling_factor`
/// for velocity and acceleration, but not for jerk — matching upstream,
/// which never scales jerk). A variable not named in a given map keeps its
/// `RobotModel` bound.
///
/// # Errors
///
/// Same as [`apply_smoothing`].
pub fn apply_smoothing_with_limits(
    trajectory: &mut RobotTrajectory<'_>,
    velocity_limits: &HashMap<String, f64>,
    acceleration_limits: &HashMap<String, f64>,
    jerk_limits: &HashMap<String, f64>,
    options: &SmoothingOptions,
) -> Result<()> {
    let group = validate_group(trajectory)?;
    if trajectory.way_point_count() < 2 {
        return Ok(());
    }

    let model = trajectory.robot_model();
    let num_dof = group.variable_names().len();
    let mut input = InputParameter::<0>::new(Some(num_dof));
    set_robot_model_bounds(
        &mut input,
        model,
        group,
        options.max_velocity_scaling_factor,
        options.max_acceleration_scaling_factor,
    );

    for (i, name) in group.variable_names().iter().enumerate() {
        if let Some(&velocity) = velocity_limits.get(name) {
            input.max_velocity[i] = velocity * options.max_velocity_scaling_factor;
        }
        if let Some(&acceleration) = acceleration_limits.get(name) {
            input.max_acceleration[i] = acceleration * options.max_acceleration_scaling_factor;
        }
        if let Some(&jerk) = jerk_limits.get(name) {
            input.max_jerk[i] = jerk;
        }
    }

    run_ruckig(trajectory, group, input, options)
}

/// `validateGroup`.
fn validate_group<'g>(trajectory: &RobotTrajectory<'g>) -> Result<&'g JointModelGroup> {
    trajectory
        .group()
        .ok_or_else(|| Error::other("the planner did not set the group the plan was computed for"))
}

/// `getRobotModelBounds`. See the module-level "Deviations from upstream"
/// note on why this is infallible here.
fn set_robot_model_bounds(
    input: &mut InputParameter<0>,
    model: &RobotModel,
    group: &JointModelGroup,
    max_velocity_scaling_factor: f64,
    max_acceleration_scaling_factor: f64,
) {
    let mut i = 0;
    for &joint_index in group.joint_indices() {
        let joint = model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            continue;
        }
        for bounds in joint.variable_bounds() {
            input.max_velocity[i] = if bounds.velocity_bounded {
                max_velocity_scaling_factor * bounds.max_velocity
            } else {
                max_velocity_scaling_factor * DEFAULT_MAX_VELOCITY
            };
            input.max_acceleration[i] = if bounds.acceleration_bounded {
                max_acceleration_scaling_factor * bounds.max_acceleration
            } else {
                max_acceleration_scaling_factor * DEFAULT_MAX_ACCELERATION
            };
            input.max_jerk[i] = if bounds.jerk_bounded {
                bounds.max_jerk
            } else {
                DEFAULT_MAX_JERK
            };
            i += 1;
        }
    }
}

/// `runRuckig`.
fn run_ruckig(
    trajectory: &mut RobotTrajectory<'_>,
    group: &JointModelGroup,
    mut input: InputParameter<0>,
    options: &SmoothingOptions,
) -> Result<()> {
    let num_waypoints = trajectory.way_point_count();
    let num_dof = group.variable_names().len();

    // This lib does not work properly when angles wrap, so we need to unwind the path first.
    trajectory.unwind();

    let mut ruckig =
        Ruckig::<0, IgnoreErrorHandler>::new(Some(num_dof), trajectory.average_segment_duration());
    initialize_ruckig_state(trajectory.first_way_point()?, group, &mut input);

    // Cache the trajectory in case we need to reset it.
    let original_trajectory = trajectory.clone();

    let mut ruckig_output = RuckigTrajectory::<0>::new(Some(num_dof));
    let mut duration_extension_factor = 1.0_f64;
    let mut smoothing_complete = false;
    let mut waypoint_idx = 0_usize;
    let mut last_result: Option<RuckigResult> = None;

    while duration_extension_factor <= MAX_DURATION_EXTENSION_FACTOR && !smoothing_complete {
        while waypoint_idx < num_waypoints - 1 {
            get_next_ruckig_input(trajectory, waypoint_idx, group, &mut input)?;

            let result = ruckig
                .calculate(&input, &mut ruckig_output)
                .map_err(|error| Error::other(format!("ruckig calculate failed: {error}")))?;

            // Step through the trajectory at the given OVERSHOOT_CHECK_PERIOD and check for
            // overshoot. We will extend the duration to mitigate it.
            let overshoots = options.mitigate_overshoot
                && check_overshoot(&ruckig_output, num_dof, &input, options.overshoot_threshold);

            // The difference between Result::Working and Result::Finished is that Finished can be
            // reached in one Ruckig timestep (constructor parameter). Both are acceptable for
            // trajectories. (The difference is only relevant for streaming mode.)
            let succeeded = matches!(result, RuckigResult::Working | RuckigResult::Finished);

            // If successful and at the last trajectory segment.
            if !overshoots && waypoint_idx == num_waypoints - 2 && succeeded {
                trajectory.set_way_point_duration_from_previous(
                    waypoint_idx + 1,
                    ruckig_output.get_duration(),
                )?;
                smoothing_complete = true;
                last_result = Some(result);
                break;
            }

            // Extend the trajectory duration if Ruckig could not reach the waypoint successfully.
            if overshoots || !succeeded {
                duration_extension_factor *= DURATION_EXTENSION_FRACTION;
                // Reset the trajectory.
                *trajectory = original_trajectory.clone();

                extend_trajectory_duration(
                    duration_extension_factor,
                    waypoint_idx,
                    group,
                    &original_trajectory,
                    trajectory,
                )?;

                initialize_ruckig_state(trajectory.first_way_point()?, group, &mut input);
                last_result = Some(result);
                // Begin the inner loop again.
                break;
            }

            last_result = Some(result);
            waypoint_idx += 1;
        }
    }

    if !matches!(
        last_result,
        Some(RuckigResult::Working) | Some(RuckigResult::Finished)
    ) {
        return Err(Error::other(format!(
            "ruckig trajectory smoothing failed: {last_result:?}"
        )));
    }

    Ok(())
}

/// `extendTrajectoryDuration`. Named and shaped after the upstream `.cpp`
/// definition — see the module-level "Deviations from upstream" note on why
/// this extends only the single segment ending at `waypoint_idx + 1`, not
/// every segment.
fn extend_trajectory_duration(
    duration_extension_factor: f64,
    waypoint_idx: usize,
    group: &JointModelGroup,
    original_trajectory: &RobotTrajectory<'_>,
    trajectory: &mut RobotTrajectory<'_>,
) -> Result<()> {
    let extended_duration = duration_extension_factor
        * original_trajectory.way_point_duration_from_previous(waypoint_idx + 1);
    trajectory.set_way_point_duration_from_previous(waypoint_idx + 1, extended_duration)?;

    let timestep = trajectory.way_point_duration_from_previous(waypoint_idx + 1);

    for name in group.variable_names() {
        let prev_velocity = trajectory
            .way_point(waypoint_idx)?
            .variable_velocity(name)
            .expect("group variable of this trajectory's own robot model");

        let target = trajectory.way_point_mut(waypoint_idx + 1)?;
        let old_velocity = target
            .variable_velocity(name)
            .expect("group variable of this trajectory's own robot model");
        let new_velocity = old_velocity / duration_extension_factor;
        target
            .set_variable_velocity(name, new_velocity)
            .expect("group variable of this trajectory's own robot model");
        target
            .set_variable_acceleration(name, (new_velocity - prev_velocity) / timestep)
            .expect("group variable of this trajectory's own robot model");
    }

    Ok(())
}

/// `initializeRuckigState`.
fn initialize_ruckig_state(
    first_waypoint: &RobotState<'_>,
    group: &JointModelGroup,
    input: &mut InputParameter<0>,
) {
    for (i, name) in group.variable_names().iter().enumerate() {
        let position = first_waypoint
            .variable_position(name)
            .expect("group variable of this trajectory's own robot model");
        let velocity = first_waypoint
            .variable_velocity(name)
            .expect("group variable of this trajectory's own robot model");
        let acceleration = first_waypoint
            .variable_acceleration(name)
            .expect("group variable of this trajectory's own robot model");

        // Clamp velocities/accelerations in case they exceed the limit due to small numerical
        // errors. `std::clamp` (`ruckig_traj_smoothing.cpp`'s
        // `initializeRuckigState`) — `cxx_clamp`, not `.clamp()`: the bound
        // is model-sourced, not a constant, and `f64::clamp` panics on a
        // NaN bound where `std::clamp` does not.
        input.current_position[i] = position;
        input.current_velocity[i] =
            cxx_clamp(velocity, -input.max_velocity[i], input.max_velocity[i]);
        input.current_acceleration[i] = cxx_clamp(
            acceleration,
            -input.max_acceleration[i],
            input.max_acceleration[i],
        );
    }
}

/// `getNextRuckigInput`.
fn get_next_ruckig_input(
    trajectory: &RobotTrajectory<'_>,
    waypoint_idx: usize,
    group: &JointModelGroup,
    input: &mut InputParameter<0>,
) -> Result<()> {
    let current = trajectory.way_point(waypoint_idx)?;
    let next = trajectory.way_point(waypoint_idx + 1)?;

    for (i, name) in group.variable_names().iter().enumerate() {
        let current_position = current
            .variable_position(name)
            .expect("group variable of this trajectory's own robot model");
        let current_velocity = current
            .variable_velocity(name)
            .expect("group variable of this trajectory's own robot model");
        let current_acceleration = current
            .variable_acceleration(name)
            .expect("group variable of this trajectory's own robot model");

        let target_position = next
            .variable_position(name)
            .expect("group variable of this trajectory's own robot model");
        let target_velocity = next
            .variable_velocity(name)
            .expect("group variable of this trajectory's own robot model");
        let target_acceleration = next
            .variable_acceleration(name)
            .expect("group variable of this trajectory's own robot model");

        input.current_position[i] = current_position;
        // Clamp velocities/accelerations in case they exceed the limit due to small numerical
        // errors. `std::clamp` (`ruckig_traj_smoothing.cpp`'s
        // `getNextRuckigInput`) — `cxx_clamp`, same reason as
        // `initialize_ruckig_state` above.
        input.current_velocity[i] = cxx_clamp(
            current_velocity,
            -input.max_velocity[i],
            input.max_velocity[i],
        );
        input.current_acceleration[i] = cxx_clamp(
            current_acceleration,
            -input.max_acceleration[i],
            input.max_acceleration[i],
        );

        input.target_position[i] = target_position;
        input.target_velocity[i] = cxx_clamp(
            target_velocity,
            -input.max_velocity[i],
            input.max_velocity[i],
        );
        input.target_acceleration[i] = cxx_clamp(
            target_acceleration,
            -input.max_acceleration[i],
            input.max_acceleration[i],
        );
    }

    Ok(())
}

/// `checkOvershoot`. See the module-level "Deviations from upstream" note:
/// only position is resampled, since upstream never reads back the velocity
/// or acceleration it also asks for.
fn check_overshoot(
    ruckig_trajectory: &RuckigTrajectory<0>,
    num_dof: usize,
    ruckig_input: &InputParameter<0>,
    overshoot_threshold: f64,
) -> bool {
    let mut time_from_start = OVERSHOOT_CHECK_PERIOD;
    while time_from_start < ruckig_trajectory.get_duration() {
        let mut new_position = DataArrayOrVec::<f64, 0>::new(Some(num_dof), 0.0);
        let mut position_out = Some(&mut new_position);
        let mut velocity_out: Option<&mut DataArrayOrVec<f64, 0>> = None;
        let mut acceleration_out: Option<&mut DataArrayOrVec<f64, 0>> = None;
        let mut jerk_out: Option<&mut DataArrayOrVec<f64, 0>> = None;
        let mut section_out: Option<usize> = None;
        ruckig_trajectory.at_time(
            time_from_start,
            &mut position_out,
            &mut velocity_out,
            &mut acceleration_out,
            &mut jerk_out,
            &mut section_out,
        );

        for joint in 0..num_dof {
            // If the sign of the error changed and the threshold difference was exceeded.
            let error = new_position[joint] - ruckig_input.target_position[joint];
            let denominator =
                ruckig_input.current_position[joint] - ruckig_input.target_position[joint];
            if (error / denominator) < 0.0 && error.abs() > overshoot_threshold {
                return true;
            }
        }

        time_from_start += OVERSHOOT_CHECK_PERIOD;
    }
    false
}

#[cfg(test)]
mod tests {
    use std::fs;

    use crate::model::{MeshSearchPaths, RobotModel};
    use crate::srdf::SrdfModel;

    use super::*;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            file_name
        )
    }

    fn panda() -> RobotModel {
        let urdf_path = fixture_path("panda.urdf");
        let srdf_path = fixture_path("panda.srdf");
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    /// `panda_joint1` has a URDF `velocity` limit (2.3925 rad/s) but no
    /// acceleration or jerk limit, so this DOF exercises both branches of
    /// every `if bounds.*_bounded` in `set_robot_model_bounds` at once: the
    /// velocity branch reads the bound, the acceleration/jerk branches fall
    /// back to `DEFAULT_MAX_ACCELERATION`/`DEFAULT_MAX_JERK`.
    #[test]
    fn set_robot_model_bounds_reads_bounded_velocity_and_defaults_the_rest() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let joint1 = group
            .variable_names()
            .iter()
            .position(|name| name == "panda_joint1")
            .expect("panda_arm must contain panda_joint1");

        let mut input = InputParameter::<0>::new(Some(group.variable_names().len()));
        set_robot_model_bounds(&mut input, &model, group, 1.0, 1.0);

        assert!((input.max_velocity[joint1] - 2.3925).abs() < 1e-9);
        assert_eq!(input.max_acceleration[joint1], DEFAULT_MAX_ACCELERATION);
        assert_eq!(input.max_jerk[joint1], DEFAULT_MAX_JERK);
    }

    /// Upstream scales the bounded velocity/acceleration by the caller's
    /// scaling factors (including the default-fallback branches), but never
    /// scales jerk.
    #[test]
    fn set_robot_model_bounds_scales_velocity_and_acceleration_but_not_jerk() {
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let joint1 = group
            .variable_names()
            .iter()
            .position(|name| name == "panda_joint1")
            .expect("panda_arm must contain panda_joint1");

        let mut input = InputParameter::<0>::new(Some(group.variable_names().len()));
        set_robot_model_bounds(&mut input, &model, group, 0.5, 0.25);

        assert!((input.max_velocity[joint1] - 2.3925 * 0.5).abs() < 1e-9);
        assert_eq!(
            input.max_acceleration[joint1],
            DEFAULT_MAX_ACCELERATION * 0.25
        );
        assert_eq!(input.max_jerk[joint1], DEFAULT_MAX_JERK);
    }

    #[test]
    fn smoothing_options_default_matches_every_upstream_overloads_defaults() {
        let options = SmoothingOptions::default();
        assert_eq!(options.max_velocity_scaling_factor, 1.0);
        assert_eq!(options.max_acceleration_scaling_factor, 1.0);
        assert!(!options.mitigate_overshoot);
        assert_eq!(options.overshoot_threshold, 0.01);
    }
}
