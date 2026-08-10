// Copyright (c) 2024, Andrew Zelenak
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/ruckig_filter.hpp
//   moveit_core/online_signal_smoothing/src/ruckig_filter.cpp
//
// `RuckigFilterPlugin` (the `SmoothingBaseClass`/pluginlib wrapper around
// this filter) is not ported — see this crate's `lib.rs`.
//
// # Deviations from upstream
//
// - **No `rclcpp::Node`/`ParamListener`.** Upstream's `initialize()` uses its
//   `node` argument for exactly one thing: constructing a
//   `generate_parameter_library` `ParamListener` and reading
//   `params_.planning_group_name`/`params_.update_period` from ROS parameter
//   YAML. That is the same ROS-parameter-tooling coupling
//   `ButterworthFilterPlugin` has (see `lib.rs`'s "Out of scope" entry for
//   it) and nothing else in `RuckigFilterPlugin` touches `node` at all —
//   `doSmoothing`, `reset`, and `getVelAccelJerkBounds` only ever use
//   `robot_model_`/`params_`-as-already-loaded and plain `Eigen`/`std`
//   types. [`RuckigFilter::new`] takes the already-resolved vel/accel/jerk
//   bound vectors and `update_period` as plain arguments instead (see
//   [`joint_vel_accel_jerk_bounds`] for the `RobotModel`-reading half of
//   `initialize`, factored out the same way), matching `ButterworthFilter`'s
//   existing precedent of taking its coefficient as a plain constructor
//   argument.
// - **`RuckigFilter::new` fails only on a non-finite or non-positive
//   `update_period`.** Upstream's `initialize()` can also fail via
//   `getVelAccelJerkBounds` returning `false`; that fallible half is
//   [`joint_vel_accel_jerk_bounds`] here, called separately by the caller
//   before constructing a [`RuckigFilter`]. `rsruckig::InputParameter::new`/
//   `Ruckig::new` are not fallible either, matching upstream's own
//   `ruckig::InputParameter`/`ruckig::Ruckig` constructors -- but
//   `Ruckig`'s internal `delta_time <= 0.0` check
//   (`rsruckig::ruckig::Ruckig::validate_input`) is silently discarded by
//   this filter's `IgnoreErrorHandler`, and upstream's own enforcement is
//   `ruckig_filter_parameters.yaml`'s `update_period: { validation: { gt<>:
//   0.0 } }`, checked by ROS parameter-loading tooling before
//   `RuckigFilterPlugin::initialize` is ever called -- neither the C++
//   constructor body nor `rsruckig` itself rejects it. Without this port's
//   own check, a zero or negative `update_period` reaches
//   `Ruckig::update`'s `output.time += self.delta_time` (never advancing)
//   and freezes [`Self::do_smoothing`]'s output at the `reset` state
//   forever, silently returning `Ok` every tick instead of an error. It is
//   a caller bug (not validated here, matching the "trust internal callers"
//   convention this crate's `ButterworthFilter::new` does not extend to
//   constructor arguments beyond its own four documented checks) to pass
//   `velocity_bounds`, `acceleration_bounds`, and `jerk_bounds` of different
//   lengths; the natural caller is [`joint_vel_accel_jerk_bounds`]'s output,
//   which always returns three equal-length vectors.
// - **`do_smoothing`/`reset` validate slice lengths against the filter's
//   joint count and return [`Error`] rather than silently mismatching.**
//   Upstream's `doSmoothing`/`reset` do not validate `positions`/
//   `velocities`/`accelerations` at all (unlike `AccelerationLimitedPlugin`,
//   which does): `Eigen::Map`/`std::vector` construction from the caller's
//   data just adopts whatever size it is given, so a mismatch surfaces
//   later, if at all, deep inside Ruckig. Rust's fixed-length slice
//   indexing has no equivalent implicit resize, so an unchecked mismatch
//   would panic on out-of-bounds indexing instead. This port turns that
//   panic into an [`Error`] at the public API boundary.
// - **`printRuckigState`/`RCLCPP_ERROR_STREAM`/`RCLCPP_WARN_STREAM` logging
//   calls are not ported.** Same reasoning as `ruckig_smoothing.rs`'s
//   equivalent note: they are diagnostics with no effect on the computed
//   output, and this crate has no logging dependency to route them through.
// - **`getVelAccelJerkBounds`'s `joint->getVariableBounds(joint->getName())`
//   lookup is transcribed as-is, including its implicit single-DOF-joint
//   assumption — now an intentional, checked contract, not an accidental
//   byproduct.** For a joint with more than one variable, no variable is
//   named identically to the joint itself
//   ([`crate::model::joint::JointModel::new_multi_variable`] names every
//   variable `"{joint_name}/{local_name}"`), so the equivalent upstream
//   lookup (`getVariableBounds`, no multi-DOF fallback either) and this
//   port's own [`crate::model::joint::JointModel::variable_bounds_for`]
//   would both fail on it regardless. Rather than leave that failure as a
//   generic "unknown name" surfaced by coincidence, [`joint_vel_accel_jerk_bounds`]
//   checks `joint.variable_names().len() != 1` explicitly and rejects a
//   multi-variable active joint with a dedicated [`Error`] naming the joint
//   and its variable count — the port's contract is **single-DOF active
//   joints only**, stated and tested, not merely true by construction.
//   Extending this to a real per-variable multi-DOF bound is not attempted:
//   there is no fixture robot in this workspace with a multi-DOF active
//   joint, and (see `acceleration_filter.rs`'s analogous note) upstream's
//   own multi-DOF handling elsewhere in `online_signal_smoothing` is a bug,
//   not a real algorithm to port.
//   `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`
//   (this module's tests, below) exercises the contract against a synthetic
//   planar joint.
// - **Oracle-verified via `tools/moveit-oracle/src/oracle.cpp`'s
//   `ruckig_filter` op.** [`RuckigFilter::do_smoothing`] is cross-checked
//   against real `online_signal_smoothing::RuckigFilterPlugin::doSmoothing`
//   (`crates/cspace-core/tests/ruckig_filter_parity.rs`) — a separate
//   fixture and op from `ruckig_parity.rs`'s, which exercises
//   `rsruckig::Ruckig::calculate` (the offline/one-shot path
//   `ruckig_smoothing.rs` uses), a different code path from the streaming
//   `update`/`pass_to_input` loop this filter drives. The oracle op loads
//   `RuckigFilterPlugin` the same way `acceleration_filter.rs`'s op loads
//   `AccelerationLimitedPlugin` — `pluginlib::ClassLoader` plus a
//   never-spun `rclcpp::Node`, since `moveit_ruckig_filter` sits under the
//   same non-exported `moveit_core_pluginTargets` CMake export set (see
//   that op's own comment in `oracle.cpp` for the full rationale). Like
//   `ruckig_parity.rs`, `rsruckig` is an independent Rust reimplementation
//   of the same published algorithm, not a binding to upstream's C++
//   `ruckig` — its root-finding does not walk identical floating-point
//   operations in identical order, so the parity test's tolerance is set
//   from what the fixture actually produces, not assumed to be exact.
//   **The fixture's discriminating power is checked, not assumed**: every
//   non-trivial computation in `do_smoothing`/`reset` has been confirmed, by
//   temporarily deleting or perturbing it and re-running the parity test, to
//   actually change the fixture's outcome when broken — see
//   `ruckig_filter_parity.rs`'s own module doc for which case kills which
//   computation, including the `target_velocity` extrapolation (needed a
//   fixed-target-to-settling and a moving-target case; the original 3 cases
//   never left the opening jerk ramp) and the `RuckigResult` early-return
//   branch (needed a zero-jerk-bound case; none of the original 3 ever
//   produced a `RuckigResult` outside the branch's allowed set).

use crate::error::{Error, Result};
use crate::model::{JointModelGroup, RobotModel};
use rsruckig::error::IgnoreErrorHandler;
use rsruckig::input_parameter::{InputParameter, Synchronization};
use rsruckig::output_parameter::OutputParameter;
use rsruckig::result::RuckigResult;
use rsruckig::ruckig::Ruckig;

/// `RuckigFilterPlugin::getVelAccelJerkBounds`.
///
/// Reads the per-joint velocity/acceleration/jerk bounds for every active
/// joint in `group`, in `group.active_joint_names()` order. See the module
/// doc's "Deviations from upstream" note on the single-DOF-joint assumption
/// this transcribes from `getVariableBounds(joint->getName())`.
///
/// # Errors
///
/// [`Error`] the moment a joint is missing its own-named variable, or that
/// variable lacks a velocity, acceleration, or jerk bound — matching
/// upstream's `return false` in the same three cases (checked in the same
/// order).
pub fn joint_vel_accel_jerk_bounds(
    model: &RobotModel,
    group: &JointModelGroup,
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    let mut velocity_bounds = Vec::with_capacity(group.active_joint_names().len());
    let mut acceleration_bounds = Vec::with_capacity(group.active_joint_names().len());
    let mut jerk_bounds = Vec::with_capacity(group.active_joint_names().len());

    for name in group.active_joint_names() {
        let joint = model.joint_model(name)?;
        if joint.variable_names().len() != 1 {
            return Err(Error::other(format!(
                "RuckigFilterPlugin only supports single-DOF active joints: {name} has {} \
                 variables",
                joint.variable_names().len()
            )));
        }
        let bound = joint.variable_bounds_for(joint.name())?;

        if !bound.velocity_bounded {
            return Err(Error::other(format!(
                "no joint velocity limit defined for {name}"
            )));
        }
        velocity_bounds.push(bound.max_velocity);

        if !bound.acceleration_bounded {
            return Err(Error::other(format!(
                "no joint acceleration limit defined for {name}"
            )));
        }
        acceleration_bounds.push(bound.max_acceleration);

        if !bound.jerk_bounded {
            return Err(Error::other(format!(
                "no joint jerk limit defined for {name}: the output from Ruckig would not be jerk-limited"
            )));
        }
        jerk_bounds.push(bound.max_jerk);
    }

    Ok((velocity_bounds, acceleration_bounds, jerk_bounds))
}

/// A `Synchronization::Phase`-mode Ruckig instance, run one control-cycle
/// tick at a time (upstream `RuckigFilterPlugin`, minus the `SmoothingBaseClass`/
/// pluginlib/ROS-parameter layer — see the module doc).
#[derive(Debug)]
pub struct RuckigFilter {
    ruckig: Ruckig<0, IgnoreErrorHandler>,
    input: InputParameter<0>,
    output: OutputParameter<0>,
    have_initial_output: bool,
}

impl RuckigFilter {
    /// `RuckigFilterPlugin::initialize`, minus the `rclcpp::Node` argument —
    /// see the module doc's "Deviations from upstream" note.
    /// `velocity_bounds`/`acceleration_bounds`/`jerk_bounds` are one entry
    /// per joint, e.g. from [`joint_vel_accel_jerk_bounds`]; `update_period`
    /// is the control-cycle duration in seconds (upstream
    /// `params_.update_period`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Construct`] if `update_period` is not finite and
    /// positive -- see the module doc's "Deviations from upstream" note on
    /// why neither upstream's C++ constructor nor `rsruckig` itself rejects
    /// this.
    pub fn new(
        velocity_bounds: &[f64],
        acceleration_bounds: &[f64],
        jerk_bounds: &[f64],
        update_period: f64,
    ) -> Result<Self> {
        if !update_period.is_finite() || update_period <= 0.0 {
            return Err(Error::construct(format!(
                "online_signal_smoothing::RuckigFilter: update_period must be finite and \
                 positive, got {update_period}"
            )));
        }
        let num_joints = velocity_bounds.len();
        let mut input = InputParameter::<0>::new(Some(num_joints));
        for i in 0..num_joints {
            input.max_velocity[i] = velocity_bounds[i];
            input.max_acceleration[i] = acceleration_bounds[i];
            input.max_jerk[i] = jerk_bounds[i];
            input.current_position[i] = 0.0;
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
        }
        input.synchronization = Synchronization::Phase;

        Ok(Self {
            ruckig: Ruckig::<0, IgnoreErrorHandler>::new(Some(num_joints), update_period),
            input,
            output: OutputParameter::<0>::new(Some(num_joints)),
            have_initial_output: false,
        })
    }

    /// The number of joints this filter was constructed for.
    pub fn num_joints(&self) -> usize {
        self.input.degrees_of_freedom
    }

    /// `RuckigFilterPlugin::doSmoothing`.
    ///
    /// Advances the filter by one control cycle: `positions` is the desired
    /// (unfiltered) target position, and on success is overwritten with the
    /// jerk-limited position/velocity/acceleration Ruckig computed for this
    /// tick (`velocities`/`accelerations` are output-only, matching
    /// upstream). The target velocity for this tick is not taken from the
    /// caller — matching upstream, it is extrapolated from the filter's own
    /// current velocity/acceleration (`target_acceleration` stays zero).
    ///
    /// This filter tracks a continuously-updating reference (e.g. a live
    /// teleoperation stream), not a fixed setpoint: because the target
    /// velocity is always the *current* velocity extrapolated forward, a
    /// caller that repeats the same static `positions` forever gives Ruckig
    /// no reason to decelerate to rest there — it has no restoring force
    /// back to zero velocity, matching upstream, which has no
    /// target-velocity input either. A caller that wants the robot to stop
    /// needs to arrange that upstream of this filter (e.g. by no longer
    /// calling [`Self::do_smoothing`], or by driving `positions` itself
    /// toward a decelerating profile), the same as it would with upstream's
    /// `RuckigFilterPlugin`.
    ///
    /// On a [`RuckigResult`] other than `Working`/`Finished`/
    /// `ErrorSynchronizationCalculation`, matches upstream: returns `Ok(())`
    /// without modifying `positions`/`velocities`/`accelerations`, and the
    /// next call starts a fresh calculation rather than continuing from the
    /// last output.
    ///
    /// # Errors
    ///
    /// [`Error`] if `positions`, `velocities`, or `accelerations` does not
    /// have length [`Self::num_joints`] (see the module doc's "Deviations
    /// from upstream" note), or if the underlying `rsruckig` call itself
    /// errors (malformed input — should not occur given this filter's own
    /// bound setup; same `rsruckig::Result` vs. upstream `bool` mismatch
    /// `ruckig_smoothing.rs` documents).
    pub fn do_smoothing(
        &mut self,
        positions: &mut [f64],
        velocities: &mut [f64],
        accelerations: &mut [f64],
    ) -> Result<()> {
        let num_joints = self.num_joints();
        if positions.len() != num_joints
            || velocities.len() != num_joints
            || accelerations.len() != num_joints
        {
            return Err(Error::other(format!(
                "positions/velocities/accelerations must each have length {num_joints}, got {}/{}/{}",
                positions.len(),
                velocities.len(),
                accelerations.len()
            )));
        }

        if self.have_initial_output {
            self.output.pass_to_input(&mut self.input);
        }

        for (i, &position) in positions.iter().enumerate() {
            self.input.target_position[i] = position;
            self.input.target_velocity[i] = self.input.current_velocity[i]
                + self.input.current_acceleration[i] * self.ruckig.delta_time;
        }
        // target_acceleration remains the zero vector `InputParameter::new` set.

        let result = self
            .ruckig
            .update(&self.input, &mut self.output)
            .map_err(|error| Error::other(format!("ruckig update failed: {error}")))?;

        if !matches!(
            result,
            RuckigResult::Finished
                | RuckigResult::Working
                | RuckigResult::ErrorSynchronizationCalculation
        ) {
            // Return without modifying the position/vel/accel.
            self.have_initial_output = false;
            return Ok(());
        }

        for i in 0..num_joints {
            positions[i] = self.output.new_position[i];
            velocities[i] = self.output.new_velocity[i];
            accelerations[i] = self.output.new_acceleration[i];
        }
        self.have_initial_output = true;

        Ok(())
    }

    /// `RuckigFilterPlugin::reset`.
    ///
    /// # Errors
    ///
    /// [`Error`] if `positions`, `velocities`, or `accelerations` does not
    /// have length [`Self::num_joints`] — see the module doc's "Deviations
    /// from upstream" note.
    pub fn reset(
        &mut self,
        positions: &[f64],
        velocities: &[f64],
        accelerations: &[f64],
    ) -> Result<()> {
        let num_joints = self.num_joints();
        if positions.len() != num_joints
            || velocities.len() != num_joints
            || accelerations.len() != num_joints
        {
            return Err(Error::other(format!(
                "positions/velocities/accelerations must each have length {num_joints}, got {}/{}/{}",
                positions.len(),
                velocities.len(),
                accelerations.len()
            )));
        }

        for i in 0..num_joints {
            self.input.current_position[i] = positions[i];
            self.input.current_velocity[i] = velocities[i];
            self.input.current_acceleration[i] = accelerations[i];
        }
        self.have_initial_output = false;

        Ok(())
    }
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

    /// `panda_arm`'s URDF has velocity limits but no acceleration/jerk
    /// limits, so this exercises `joint_vel_accel_jerk_bounds`'s first
    /// failure branch (velocity bounded, acceleration not) — matching the
    /// order upstream's `getVelAccelJerkBounds` checks in.
    #[test]
    fn joint_vel_accel_jerk_bounds_fails_without_acceleration_limits() {
        // `matches!` alone cannot tell this apart from the sibling
        // single-DOF/velocity/jerk guards, all Error::Other in this
        // function; message-swap bite-checked against them.
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let err = joint_vel_accel_jerk_bounds(&model, group).unwrap_err();
        assert!(
            err.to_string().contains("acceleration limit defined"),
            "{err}"
        );
    }

    fn clear_velocity_bounds(model: &mut RobotModel) {
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_velocity_limits = false;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
    }

    /// Clearing `panda_arm`'s velocity limits (present by default, see
    /// [`joint_vel_accel_jerk_bounds_fails_without_acceleration_limits`])
    /// exercises the guard ahead of the acceleration one — `matches!` alone
    /// cannot tell this apart from its `Error::Other` siblings either.
    #[test]
    fn joint_vel_accel_jerk_bounds_fails_without_velocity_limits() {
        let mut model = panda();
        clear_velocity_bounds(&mut model);
        let group = model.joint_model_group("panda_arm").unwrap();
        let err = joint_vel_accel_jerk_bounds(&model, group).unwrap_err();
        assert!(err.to_string().contains("velocity limit defined"), "{err}");
    }

    fn set_uniform_acceleration_bounds(model: &mut RobotModel, max_acceleration: f64) {
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = max_acceleration;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
    }

    /// `panda_arm` has velocity limits by default and this sets acceleration
    /// ones without touching jerk, so this exercises the last of the three
    /// bound guards on its own — `matches!` alone cannot tell it apart from
    /// its `Error::Other` siblings either.
    #[test]
    fn joint_vel_accel_jerk_bounds_fails_without_jerk_limits() {
        let mut model = panda();
        set_uniform_acceleration_bounds(&mut model, 3.0);
        let group = model.joint_model_group("panda_arm").unwrap();
        let err = joint_vel_accel_jerk_bounds(&model, group).unwrap_err();
        assert!(err.to_string().contains("jerk limit defined"), "{err}");
    }

    fn set_uniform_accel_jerk_bounds(model: &mut RobotModel, max_acceleration: f64, max_jerk: f64) {
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = max_acceleration;
                limit.has_jerk_limits = true;
                limit.max_jerk = max_jerk;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
    }

    #[test]
    fn joint_vel_accel_jerk_bounds_succeeds_once_all_three_bounds_are_set() {
        let mut model = panda();
        set_uniform_accel_jerk_bounds(&mut model, 3.0, 1000.0);
        let group = model.joint_model_group("panda_arm").unwrap();
        let (velocity, acceleration, jerk) = joint_vel_accel_jerk_bounds(&model, group).unwrap();
        assert_eq!(velocity.len(), 7);
        assert!(acceleration.iter().all(|&a| a == 3.0));
        assert!(jerk.iter().all(|&j| j == 1000.0));
    }

    /// The single-DOF-active-joint contract (module doc's "Deviations from
    /// upstream" note): a group whose active joint has more than one
    /// variable is a dedicated [`Error`], not a lookup that happens to fail
    /// with "unknown name", and not upstream's silent last-variable-wins.
    ///
    /// Reuses `cspace_core::trajectory`'s own `totg_synthetic.{urdf,srdf}` (also
    /// this worker's crate) rather than adding a new fixture file here: a
    /// new synthetic fixture needs registering in
    /// `tools/ci/verify-fixture-provenance.sh`'s `SYNTHETIC` allowlist, which
    /// lives outside this crate's ownership, and `totg_synthetic` already
    /// defines exactly what this test needs (`planar_group`, a single
    /// `planar_joint`, 3 variables) for the identical reason
    /// (`totg_robot_trajectory_parity.rs`'s own mimic/multi-DOF coverage
    /// gap).
    #[test]
    fn multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins() {
        let path = |file_name: &str| {
            format!(
                concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/trajectory/{}"),
                file_name
            )
        };
        let urdf_path = path("totg_synthetic.urdf");
        let srdf_path = path("totg_synthetic.srdf");
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        let group = model.joint_model_group("planar_group").unwrap();

        let err = joint_vel_accel_jerk_bounds(&model, group).unwrap_err();
        let message = err.to_string();
        assert!(
            message.contains("planar_joint") && message.contains('3'),
            "expected the error to name the joint and its variable count: {message}"
        );
    }

    /// Upstream never checks its own vel/accel/jerk bounds against Ruckig's
    /// output (it trusts Ruckig to enforce the `InputParameter`s it was
    /// given), so this is not a transcribed upstream test — it is this
    /// port's own check that [`RuckigFilter::new`]'s bounds actually reach
    /// Ruckig and are honored over a run long enough to pass through
    /// acceleration ramp-up, cruise, and the repeated-target steady state
    /// documented on [`RuckigFilter::do_smoothing`] (see its doc comment:
    /// this filter does not decelerate to rest at a held-static target, so
    /// this test asserts bound compliance throughout, not convergence).
    #[test]
    fn do_smoothing_respects_velocity_and_acceleration_bounds_throughout() {
        let mut filter = RuckigFilter::new(&[1.0], &[2.0], &[20.0], 0.01).unwrap();
        filter.reset(&[0.0], &[0.0], &[0.0]).unwrap();

        let mut positions = [0.05];
        let mut velocities = [0.0];
        let mut accelerations = [0.0];
        for tick in 0..200 {
            filter
                .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
                .unwrap();
            assert!(
                velocities[0].abs() <= 1.0 + 1e-9,
                "tick {tick}: {velocities:?}"
            );
            assert!(
                accelerations[0].abs() <= 2.0 + 1e-9,
                "tick {tick}: {accelerations:?}"
            );
            positions[0] = 0.05;
        }
    }

    /// The very first tick from rest, with the target already close, must
    /// move toward it (not away, not stand still) and stay within bounds —
    /// the simplest possible closed-loop sanity check, mirroring
    /// `ButterworthFilter`'s `FilterConverge` case without relying on the
    /// long-run steady-state behaviour the test above documents.
    #[test]
    fn first_tick_from_rest_moves_toward_a_nearby_target() {
        let mut filter = RuckigFilter::new(&[1.0], &[2.0], &[20.0], 0.01).unwrap();
        filter.reset(&[0.0], &[0.0], &[0.0]).unwrap();

        let mut positions = [0.02];
        let mut velocities = [0.0];
        let mut accelerations = [0.0];
        filter
            .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
            .unwrap();

        assert!(positions[0] > 0.0, "{positions:?}");
        assert!(positions[0] < 0.02, "{positions:?}");
        assert!(velocities[0] > 0.0, "{velocities:?}");
    }

    #[test]
    fn do_smoothing_rejects_a_mismatched_length() {
        // `matches!` alone cannot tell this apart from do_smoothing's
        // sibling ruckig-update-failure site (`:289`), also an
        // Error::Other -- but that site is unreachable, not merely
        // untested: `RuckigFilter` fixes `Ruckig<0, IgnoreErrorHandler>`,
        // and every `Err(RuckigError)` construction in the `rsruckig`
        // crate is gated behind the error-handler type parameter's
        // `handle_calculator_error`/`handle_validation_error`, both
        // no-ops under `IgnoreErrorHandler` (return `Ok(())`
        // unconditionally). `self.ruckig.update(...)` can never return
        // `Err` in this configuration, so no message-swap bite against
        // it was ever possible, despite what an earlier version of this
        // comment claimed.
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        filter.reset(&[0.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        let mut positions = [0.5];
        let mut velocities = [0.0];
        let mut accelerations = [0.0];
        let err = filter
            .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
            .unwrap_err();
        assert!(err.to_string().contains("must each have length"), "{err}");
    }

    #[test]
    fn do_smoothing_rejects_a_positions_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        filter.reset(&[0.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        let mut positions = [0.5];
        let mut velocities = [0.0, 0.0];
        let mut accelerations = [0.0, 0.0];
        let err = filter
            .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
            .unwrap_err();
        assert!(err.to_string().contains("must each have length"), "{err}");
    }

    #[test]
    fn do_smoothing_rejects_a_velocities_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        filter.reset(&[0.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        let mut positions = [0.5, 0.5];
        let mut velocities = [0.0];
        let mut accelerations = [0.0, 0.0];
        let err = filter
            .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
            .unwrap_err();
        assert!(err.to_string().contains("must each have length"), "{err}");
    }

    #[test]
    fn do_smoothing_rejects_an_accelerations_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        filter.reset(&[0.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        let mut positions = [0.5, 0.5];
        let mut velocities = [0.0, 0.0];
        let mut accelerations = [0.0];
        let err = filter
            .do_smoothing(&mut positions, &mut velocities, &mut accelerations)
            .unwrap_err();
        assert!(err.to_string().contains("must each have length"), "{err}");
    }

    #[test]
    fn reset_rejects_a_mismatched_length() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        let err = filter.reset(&[0.0], &[0.0], &[0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    #[test]
    fn reset_rejects_a_positions_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        let err = filter.reset(&[0.0], &[0.0, 0.0], &[0.0, 0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    #[test]
    fn reset_rejects_a_velocities_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        let err = filter.reset(&[0.0, 0.0], &[0.0], &[0.0, 0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    #[test]
    fn reset_rejects_an_accelerations_only_mismatch() {
        let mut filter = RuckigFilter::new(&[1.0, 1.0], &[2.0, 2.0], &[20.0, 20.0], 0.01).unwrap();
        let err = filter.reset(&[0.0, 0.0], &[0.0, 0.0], &[0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    // Before this check existed, `update_period = 0.0` reached
    // `Ruckig::new` unchecked: `Ruckig::validate_input`'s own `delta_time <=
    // 0.0` check (rsruckig `ruckig.rs`) fired but was silently swallowed by
    // `IgnoreErrorHandler`, and `Ruckig::update`'s `output.time +=
    // self.delta_time` never advanced -- every tick resampled the
    // just-recalculated trajectory at its own t=0, i.e. exactly the current
    // state, freezing the filter's output at the `reset` position forever
    // while still returning `Ok` every tick. See [`RuckigFilter::new`]'s
    // `# Errors`.
    #[test]
    fn new_rejects_a_non_positive_update_period() {
        let err = RuckigFilter::new(&[1.0], &[2.0], &[20.0], 0.0).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "{err:?}");
        let err = RuckigFilter::new(&[1.0], &[2.0], &[20.0], -0.01).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "{err:?}");
    }

    #[test]
    fn new_rejects_a_non_finite_update_period() {
        let err = RuckigFilter::new(&[1.0], &[2.0], &[20.0], f64::NAN).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "{err:?}");
        let err = RuckigFilter::new(&[1.0], &[2.0], &[20.0], f64::INFINITY).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "{err:?}");
    }
}
