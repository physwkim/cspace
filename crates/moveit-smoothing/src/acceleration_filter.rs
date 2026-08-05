// Copyright (c) 2024, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/acceleration_filter.hpp
//   moveit_core/online_signal_smoothing/src/acceleration_filter.cpp
//
// `AccelerationLimitedPlugin` (the `SmoothingBaseClass`/pluginlib wrapper) is
// not ported -- see this crate's `lib.rs`. This ports `initialize`/
// `doSmoothing`/`reset`: the 1-D, single-variable box-constrained QP
// `lib.rs`'s former "not yet ported" doc block derived in closed form. That
// derivation is now verified against real ground truth --
// `tools/moveit-oracle/src/oracle.cpp`'s `acceleration_filter` op, captured
// into `crates/moveit-smoothing/tests/fixtures/acceleration_filter_{request,response}.json`
// -- rather than resting on the derivation alone (see `PORTING-PLAN.md` §62.3
// on why "we derived it, so it is right" is not accepted as its own proof in
// this port).
//
// # Deviations from upstream
//
// - **No `rclcpp::Node`/`ParamListener`.** Same coupling, same fix, as
//   `ruckig_filter.rs`'s equivalent note: upstream's `initialize()` uses
//   `node` only to construct a `generate_parameter_library` `ParamListener`
//   and read `params_.planning_group_name`/`params_.update_period`.
//   [`joint_acceleration_bounds`] takes the already-resolved
//   `RobotModel`/`JointModelGroup` instead (the `RobotModel`-reading half of
//   `initialize`, split out the same way
//   `ruckig_filter::joint_vel_accel_jerk_bounds` is), and
//   [`AccelerationLimitedFilter::new`] takes the resolved bound vectors and
//   `update_period` as plain arguments.
// - **`accelerations` is dropped from `do_smoothing`/`reset` entirely,
//   unlike `RuckigFilter`'s.** Upstream's own signatures keep a third
//   `Eigen::VectorXd& accelerations` parameter on both methods, but neither
//   reads nor writes it (see the unnamed/`/* unused */` parameter names in
//   the upstream source) -- unlike `RuckigFilter::do_smoothing`, where
//   `accelerations` is a real computed output. Threading a parameter through
//   this port's public API that upstream itself never touches would just be
//   signature cosplay, so it is omitted; `tools/moveit-oracle/src/oracle.cpp`'s
//   `acceleration_filter` op does the same on the wire (see that op's own
//   comment).
// - **`positions_offset_`/`velocities_offset_` are locals, not fields.**
//   Upstream stores them as `AccelerationLimitedPlugin` members, but nothing
//   outside `doSmoothing` ever reads them (not `reset`, not the destructor);
//   this port computes them as ordinary locals inside
//   [`AccelerationLimitedFilter::do_smoothing`] instead of carrying two
//   vectors nothing else uses across calls.
// - **Bounds extraction sidesteps upstream's per-*joint* (not per-*variable*)
//   index-advance bug by construction, and shares
//   `ruckig_filter::joint_vel_accel_jerk_bounds`'s single-DOF-joint
//   contract.** Upstream's own `initialize()` walks
//   `getActiveJointModelsBounds()` (one entry per active *joint*) and, for
//   each, an inner loop over that joint's own *variables* -- but advances its
//   flat write index once per outer (joint) iteration, not once per inner
//   (variable) one, so a joint with more than one variable silently keeps
//   only its last variable's bound and never advances into the next joint's
//   slot. [`joint_acceleration_bounds`] reads one bound per
//   [`moveit_model::JointModelGroup::active_joint_names`] entry from that
//   joint's own-named variable -- the same lookup-by-joint-name pattern
//   `joint_vel_accel_jerk_bounds` already uses for velocity/acceleration/jerk
//   -- which has no way to represent a multi-variable joint at all, so there
//   is no index to misadvance. This is not a fix applied to upstream's loop;
//   it is a different, narrower data representation that the bug's
//   precondition cannot occur in.
//
//   This port's contract is **single-DOF active joints only**, and that is
//   now an intentional, checked precondition rather than an accidental
//   byproduct of the name lookup: [`joint_acceleration_bounds`] rejects a
//   multi-variable active joint with a dedicated [`Error`] naming the joint
//   and its variable count, before attempting `variable_bounds_for`, rather
//   than relying on that lookup to fail with a generic "unknown name" once
//   it discovers no variable is named identically to the joint (which it
//   always does for a multi-variable joint --
//   [`moveit_model::joint::JointModel::new_multi_variable`] names every
//   variable `"{joint_name}/{local_name}"`, never the bare joint name).
//   Extending this to a real per-variable multi-DOF bound is not attempted:
//   upstream's own multi-DOF behaviour here is the index-advance bug above,
//   not a real algorithm this port could transcribe, and there is no
//   fixture robot in this workspace with a multi-DOF active joint whose
//   correct per-variable bound behaviour could be derived independently.
//   `multi_dof_active_joint_is_a_typed_error_not_a_silent_last_variable_wins`
//   (this module's tests, below) exercises the contract against a
//   synthetic planar joint.
// - **`do_smoothing`'s length check reads `velocities.len()`, not
//   `positions.len()`, transcribed as upstream's own quirk.** Upstream names
//   its check variable `num_positions` but assigns it from
//   `velocities.size()` (`const size_t num_positions = velocities.size();`);
//   a `positions`/`velocities` pair that disagree in length passes or fails
//   this check based on `velocities`' length alone, with `positions`' actual
//   length only checked indirectly, against `last_positions_.size()`, by the
//   very next `else if`. Transcribed here exactly as upstream computes it,
//   including the confusing name and the checked-second `positions` failure
//   message, rather than "fixed" into a check against `positions.len()`
//   directly.
// - **Exact closed-form solve vs. upstream's iterative `osqp` solve: verified
//   equal to float precision away from a degenerate constraint, and
//   documented, not silently normalized, at one.** The QP upstream hands to
//   `osqp` has exactly one variable and every constraint linear in it, so
//   [`solve_alpha`] computes the intersection of every constraint's
//   `alpha`-interval directly (see its own doc comment) instead of running an
//   iterative solver. Every non-degenerate case this port's fixture and unit
//   tests cover agrees with `osqp`'s own answer to float precision (`osqp`
//   really does converge to the exact optimum away from a boundary). At a
//   *degenerate* constraint -- a joint given a zero-width acceleration
//   interval (`min_acceleration == max_acceleration == 0.0`) while its
//   position target differs from its last position, which forces the exact
//   optimum to `alpha == 1.0` precisely -- `osqp`'s own default `eps_abs`/
//   `eps_rel` (upstream sets neither; `initialize()` only overrides
//   `warm_starting`/`verbose`) stop its ADMM iteration once the residual is
//   merely *small*, not zero, so its returned `alpha` in that case is only
//   *close to* the exact optimum. Measured directly from
//   `acceleration_filter_response.json`'s third case (`panda_joint1`, `M =
//   0.0`, target `3.0`): `osqp` returns a position `0.0008294991991130152`
//   away from the exact-`alpha == 1.0` answer of `0.0`, an `alpha`-space
//   error of about `2.76e-4` -- consistent with (well under) `osqp`'s
//   documented default `eps_abs = eps_rel = 1e-3`. This port always computes
//   the exact optimum, so its parity test for that one case
//   (`single_point_intersection_forces_alpha_to_one` in this module's tests,
//   below) uses an explicitly widened, measured-not-guessed tolerance
//   instead of the tight one every other case uses --
//   [`crate::EPSILON`]-scale tolerances would fail on that case forever, not
//   from a bug in either side.

use moveit_error::{Error, Result};
use moveit_model::{JointModelGroup, RobotModel};

/// The threshold below which any position or velocity difference is
/// considered zero (rad and rad/s). `COMMAND_DIFFERENCE_THRESHOLD`.
const COMMAND_DIFFERENCE_THRESHOLD: f64 = 1e-4;

/// `AccelerationLimitedPlugin::initialize`'s bounds-extraction loop, reading
/// the per-joint `(min, max)` acceleration bound for every active joint in
/// `group`, in [`JointModelGroup::active_joint_names`] order, from that
/// joint's own-named variable. See the module doc's "Deviations from
/// upstream" note on the per-joint (not per-variable) data representation
/// this reads into, and `ruckig_filter::joint_vel_accel_jerk_bounds`'s doc
/// comment for the shared single-DOF-joint assumption.
///
/// # Errors
///
/// [`Error`] the moment a joint is missing its own-named variable, or that
/// variable has no acceleration bound -- matching upstream's
/// `RCLCPP_ERROR` + `return false` in the same case.
pub fn joint_acceleration_bounds(
    model: &RobotModel,
    group: &JointModelGroup,
) -> Result<(Vec<f64>, Vec<f64>)> {
    let mut min_acceleration_limits = Vec::with_capacity(group.active_joint_names().len());
    let mut max_acceleration_limits = Vec::with_capacity(group.active_joint_names().len());

    for name in group.active_joint_names() {
        let joint = model.joint_model(name)?;
        if joint.variable_names().len() != 1 {
            return Err(Error::other(format!(
                "AccelerationLimitedPlugin only supports single-DOF active joints: {name} has \
                 {} variables",
                joint.variable_names().len()
            )));
        }
        let bound = joint.variable_bounds_for(joint.name())?;

        if !bound.acceleration_bounded {
            return Err(Error::other(format!(
                "the robot must have acceleration joint limits specified for all joints to use \
                 AccelerationLimitedPlugin: {name} has none"
            )));
        }
        min_acceleration_limits.push(bound.min_acceleration);
        max_acceleration_limits.push(bound.max_acceleration);
    }

    Ok((min_acceleration_limits, max_acceleration_limits))
}

/// `jointLimitAccelerationScalingFactor` (free function in
/// `acceleration_filter.cpp`). The uniform scale-down applied to a candidate
/// acceleration vector so every joint stays within its own `[min, max]`
/// acceleration bound: the minimum, over every joint with a nonzero target
/// acceleration, of the ratio between that joint's bound-clamped and raw
/// acceleration -- `1.0` (no scaling) if every target acceleration is zero.
fn joint_limit_acceleration_scaling_factor(
    accelerations: &[f64],
    min_acceleration_limits: &[f64],
    max_acceleration_limits: &[f64],
) -> f64 {
    let mut min_scaling_factor = 1.0_f64;
    for i in 0..accelerations.len() {
        let target_accel = accelerations[i];
        if target_accel != 0.0 {
            let bounded_accel =
                target_accel.clamp(min_acceleration_limits[i], max_acceleration_limits[i]);
            let joint_scaling_factor = bounded_accel / target_accel;
            min_scaling_factor = min_scaling_factor.min(joint_scaling_factor);
        }
    }
    min_scaling_factor
}

/// Exact closed-form solution to `AccelerationLimitedPlugin::doSmoothing`'s
/// QP -- see the module doc's "Deviations from upstream" note on how this
/// compares to upstream's iterative `osqp` solve.
///
/// `offset[i]` is `last_positions[i] - positions[i]` (the constraint
/// coefficient upstream's `constraints_sparse_` carries); `lower_bound[i]`/
/// `upper_bound[i]` are upstream's own `lower_bound`/`upper_bound` vectors.
/// Returns the `alpha` in `[0, 1]` minimizing `alpha^2` subject to every
/// `lower_bound[i] <= offset[i] * alpha <= upper_bound[i]`, or `None` if no
/// such `alpha` exists.
///
/// For `offset[i] == 0.0` the constraint does not depend on `alpha` at all
/// (`0 <= alpha * 0 <= 0` reduces to checking `lower_bound[i] <= 0 <=
/// upper_bound[i]` directly); for `offset[i] != 0.0`, dividing through gives
/// an `alpha`-interval (flipped if `offset[i]` is negative), intersected
/// with every other joint's interval and with `[0, 1]`. Because that
/// intersection's lower edge can never go below the box's own `0.0` floor,
/// the minimizer of `alpha^2` over a nonempty intersection is always its
/// lower edge.
fn solve_alpha(offset: &[f64], lower_bound: &[f64], upper_bound: &[f64]) -> Option<f64> {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    for ((&o, &l), &u) in offset.iter().zip(lower_bound).zip(upper_bound) {
        if o == 0.0 {
            if l > 0.0 || u < 0.0 {
                return None;
            }
            continue;
        }
        let (row_lo, row_hi) = if o > 0.0 {
            (l / o, u / o)
        } else {
            (u / o, l / o)
        };
        lo = lo.max(row_lo);
        hi = hi.min(row_hi);
    }
    if lo > hi { None } else { Some(lo) }
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// `AccelerationLimitedPlugin`, minus the `SmoothingBaseClass`/pluginlib/
/// ROS-parameter layer -- see the module doc. Limits acceleration between
/// consecutive commands by finding the point on the line from the last
/// commanded position to the newly desired one that is closest to the new
/// desired position while respecting every joint's acceleration limit.
pub struct AccelerationLimitedFilter {
    min_acceleration_limits: Vec<f64>,
    max_acceleration_limits: Vec<f64>,
    update_period: f64,
    /// Empty until [`Self::reset`] is called -- see
    /// [`Self::do_smoothing`]'s doc comment on why that makes calling it
    /// before `reset` an [`Error`], matching upstream.
    last_positions: Vec<f64>,
    last_velocities: Vec<f64>,
}

impl AccelerationLimitedFilter {
    /// `AccelerationLimitedPlugin::initialize`, minus the `rclcpp::Node`
    /// argument -- see the module doc. `min_acceleration_limits`/
    /// `max_acceleration_limits` are one entry per joint, e.g. from
    /// [`joint_acceleration_bounds`]; `update_period` is the control-cycle
    /// duration in seconds (upstream `params_.update_period`).
    ///
    /// Infallible and untrusting of nothing beyond equal-length inputs,
    /// matching `RuckigFilter::new`'s identical convention: the natural
    /// caller ([`joint_acceleration_bounds`]) always returns a matching
    /// pair.
    pub fn new(
        min_acceleration_limits: &[f64],
        max_acceleration_limits: &[f64],
        update_period: f64,
    ) -> Self {
        Self {
            min_acceleration_limits: min_acceleration_limits.to_vec(),
            max_acceleration_limits: max_acceleration_limits.to_vec(),
            update_period,
            last_positions: Vec::new(),
            last_velocities: Vec::new(),
        }
    }

    /// The number of joints this filter was constructed for.
    pub fn num_joints(&self) -> usize {
        self.max_acceleration_limits.len()
    }

    /// `AccelerationLimitedPlugin::reset`. Must be called before
    /// [`Self::do_smoothing`].
    ///
    /// # Errors
    ///
    /// [`Error`] if `positions`/`velocities` does not have length
    /// [`Self::num_joints`]. Upstream's own `reset` has no such check (unlike
    /// its `doSmoothing`) -- Rust's fixed-length slice indexing has no
    /// equivalent implicit resize, so an unchecked mismatch here would panic
    /// the next [`Self::do_smoothing`] call instead, the same reasoning
    /// `ruckig_filter.rs`'s identical check documents.
    pub fn reset(&mut self, positions: &[f64], velocities: &[f64]) -> Result<()> {
        let num_joints = self.num_joints();
        if positions.len() != num_joints || velocities.len() != num_joints {
            return Err(Error::other(format!(
                "positions/velocities must each have length {num_joints}, got {}/{}",
                positions.len(),
                velocities.len(),
            )));
        }
        self.last_positions = positions.to_vec();
        self.last_velocities = velocities.to_vec();
        Ok(())
    }

    /// `AccelerationLimitedPlugin::doSmoothing`. On success, `positions`/
    /// `velocities` are overwritten with the acceleration-limited next
    /// commanded point; upstream's `accelerations` parameter is dropped
    /// entirely (see the module doc).
    ///
    /// # Errors
    ///
    /// [`Error`] if `velocities.len()` is not [`Self::num_joints`], or if
    /// [`Self::reset`] was never called (`positions.len()` then disagrees
    /// with the empty `last_positions`) -- see the module doc's "Deviations
    /// from upstream" note on why the first check reads `velocities`, not
    /// `positions`.
    pub fn do_smoothing(&mut self, positions: &mut [f64], velocities: &mut [f64]) -> Result<()> {
        let num_joints = self.num_joints();
        let num_positions = velocities.len();
        if num_positions != num_joints {
            return Err(Error::other(format!(
                "the length of the joint positions parameter is not equal to the number of \
                 joints, expected {num_joints} got {num_positions}"
            )));
        }
        if self.last_positions.len() != positions.len() {
            return Err(Error::other(format!(
                "the length of the last joint positions not equal to the current, expected {} \
                 got {}. Make sure the reset was called",
                self.last_positions.len(),
                positions.len()
            )));
        }

        let dt = self.update_period;
        let mut positions_offset = vec![0.0; num_joints];
        let mut velocities_offset = vec![0.0; num_joints];
        let mut lower_bound = vec![0.0; num_joints];
        let mut upper_bound = vec![0.0; num_joints];
        for i in 0..num_joints {
            positions_offset[i] = self.last_positions[i] - positions[i];
            velocities_offset[i] = self.last_velocities[i] - velocities[i];
            let vel_point = self.last_positions[i] + self.last_velocities[i] * dt;
            upper_bound[i] = vel_point - positions[i] + self.max_acceleration_limits[i] * dt * dt;
            lower_bound[i] = vel_point - positions[i] + self.min_acceleration_limits[i] * dt * dt;
        }

        if norm(&positions_offset) < COMMAND_DIFFERENCE_THRESHOLD
            && norm(&velocities_offset) < COMMAND_DIFFERENCE_THRESHOLD
        {
            positions.copy_from_slice(&self.last_positions);
            velocities.copy_from_slice(&self.last_velocities);
        } else if let Some(alpha) = solve_alpha(&positions_offset, &lower_bound, &upper_bound) {
            for (p, &last_p) in positions.iter_mut().zip(self.last_positions.iter()) {
                *p = alpha * last_p + (1.0 - alpha) * *p;
            }
            for ((v, &p), &last_p) in velocities
                .iter_mut()
                .zip(positions.iter())
                .zip(self.last_positions.iter())
            {
                *v = (p - last_p) / dt;
            }
        } else {
            let mut cur_acceleration: Vec<f64> =
                self.last_velocities.iter().map(|&v| -v / dt).collect();
            let scale = joint_limit_acceleration_scaling_factor(
                &cur_acceleration,
                &self.min_acceleration_limits,
                &self.max_acceleration_limits,
            );
            for a in &mut cur_acceleration {
                *a *= scale;
            }
            for i in 0..num_joints {
                velocities[i] = self.last_velocities[i] + cur_acceleration[i] * dt;
                positions[i] = self.last_positions[i] + velocities[i] * dt;
            }
        }

        self.last_velocities.copy_from_slice(velocities);
        self.last_positions.copy_from_slice(positions);

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;

    use super::*;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
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

    const PANDA_ARM_JOINTS: [&str; 7] = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ];

    fn set_uniform_acceleration_bound(model: &mut RobotModel, max_acceleration: f64) {
        for name in PANDA_ARM_JOINTS {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = max_acceleration;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
    }

    /// `panda_arm`'s URDF has no acceleration limits, so this exercises
    /// `joint_acceleration_bounds`'s failure branch. Upstream does exercise
    /// this scenario -- `test_acceleration_filter.cpp`'s `FilterInitialize`
    /// (`TEST_F(AccelerationFilterTest, FilterInitialize)`, pinned SHA lines
    /// 80-104) calls `setLimits({})` then asserts at lines 97-98 that
    /// `AccelerationLimitedPlugin::initialize` fails. This is not a
    /// line-for-line transcription of that assertion, because
    /// `joint_acceleration_bounds` is this port's own extraction of the
    /// bound lookup `initialize` performs inline -- there is no
    /// upstream test scoped to that narrower function. This test checks the
    /// same "must have every bound" contract `AccelerationLimitedPlugin::initialize`
    /// enforces, at the granularity this port's own function split created.
    #[test]
    fn joint_acceleration_bounds_fails_without_acceleration_limits() {
        // `matches!` alone cannot tell this apart from the sibling
        // single-DOF-active-joint guard, also an Error::Other in this
        // function; message-swap bite-checked against it.
        let model = panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let err = joint_acceleration_bounds(&model, group).unwrap_err();
        assert!(
            err.to_string()
                .contains("must have acceleration joint limits"),
            "{err}"
        );
    }

    #[test]
    fn joint_acceleration_bounds_succeeds_once_every_bound_is_set() {
        let mut model = panda();
        set_uniform_acceleration_bound(&mut model, 2.0);
        let group = model.joint_model_group("panda_arm").unwrap();
        let (min, max) = joint_acceleration_bounds(&model, group).unwrap();
        assert_eq!(min.len(), 7);
        assert!(min.iter().all(|&m| m == -2.0));
        assert!(max.iter().all(|&m| m == 2.0));
    }

    /// The single-DOF-active-joint contract (module doc's "Deviations from
    /// upstream" note): a group whose active joint has more than one
    /// variable is a dedicated [`Error`], not a lookup that happens to fail
    /// with "unknown name", and not upstream's silent last-variable-wins.
    ///
    /// Reuses `moveit-trajectory`'s own `totg_synthetic.{urdf,srdf}` (also
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
        let urdf_path = format!(
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../moveit-trajectory/tests/fixtures/{}"
            ),
            "totg_synthetic.urdf"
        );
        let srdf_path = format!(
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../moveit-trajectory/tests/fixtures/{}"
            ),
            "totg_synthetic.srdf"
        );
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        let group = model.joint_model_group("planar_group").unwrap();

        let err = joint_acceleration_bounds(&model, group).unwrap_err();
        let message = err.to_string();
        assert!(
            message.contains("planar_joint") && message.contains('3'),
            "expected the error to name the joint and its variable count: {message}"
        );
    }

    #[test]
    fn do_smoothing_before_reset_is_an_error() {
        // `matches!` alone cannot tell this apart from do_smoothing's
        // sibling velocities-length guard (`:329`), also an Error::Other.
        // That guard has no test of its own in this file or anywhere in
        // the workspace (its message has exactly one hit: its own
        // `format!` call) -- it was never message-swap bite-checked
        // against this one, despite what an earlier version of this
        // comment claimed. See `do_smoothing_rejects_a_length_mismatch`
        // below for the guard's own coverage and bite.
        let mut filter = AccelerationLimitedFilter::new(&[-2.0], &[2.0], 1.0);
        let mut positions = [0.5];
        let mut velocities = [0.5];
        let err = filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap_err();
        assert!(
            err.to_string().contains("Make sure the reset was called"),
            "{err}"
        );
    }

    #[test]
    fn reset_rejects_a_mismatched_length() {
        let mut filter = AccelerationLimitedFilter::new(&[-2.0, -2.0], &[2.0, 2.0], 1.0);
        let err = filter.reset(&[0.0], &[0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    #[test]
    fn reset_rejects_a_positions_only_mismatch() {
        let mut filter = AccelerationLimitedFilter::new(&[-2.0, -2.0], &[2.0, 2.0], 1.0);
        let err = filter.reset(&[0.0], &[0.0, 0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    #[test]
    fn reset_rejects_a_velocities_only_mismatch() {
        let mut filter = AccelerationLimitedFilter::new(&[-2.0, -2.0], &[2.0, 2.0], 1.0);
        let err = filter.reset(&[0.0, 0.0], &[0.0]).unwrap_err();
        assert!(matches!(err, Error::Other(_)), "{err:?}");
    }

    // The six cases below are `acceleration_filter_request.json`/
    // `_response.json` (`crates/moveit-smoothing/tests/fixtures/`), captured
    // from `tools/moveit-oracle/src/oracle.cpp`'s `acceleration_filter` op
    // against real `AccelerationLimitedPlugin::doSmoothing`, transcribed
    // here as a parity test per case -- named for the boundary each one
    // isolates, matching `PORTING-PLAN.md`'s "test by boundary, not by
    // narrative" instruction for this port. All seven `panda_arm` joints are
    // present on the wire; every case below drives only the one joint its
    // name calls out, holding the rest at their reset value (an `offset` of
    // `0.0`, always non-binding regardless of that joint's own bound).

    const READY_POSE: [f64; 7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];

    fn reset_filter(
        max_acceleration_limits: &[f64],
        last_velocities: &[f64],
    ) -> AccelerationLimitedFilter {
        let min_acceleration_limits: Vec<f64> =
            max_acceleration_limits.iter().map(|&m| -m).collect();
        let mut filter =
            AccelerationLimitedFilter::new(&min_acceleration_limits, max_acceleration_limits, 1.0);
        filter.reset(&READY_POSE, last_velocities).unwrap();
        filter
    }

    /// No bound binds: `panda_joint1`'s target is well within what a
    /// generous acceleration limit allows in one `update_period`, so the
    /// exact target is reached in a single step (`alpha == 0.0`). Fixture
    /// case 1, first step.
    #[test]
    fn unconstrained_reaches_the_target_in_one_step() {
        let mut filter = reset_filter(&[2.0; 7], &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[0] = 0.5;
        let mut velocities = [0.0; 7];
        velocities[0] = 0.5;
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        assert_eq!(positions[0], 0.5);
        assert_eq!(velocities[0], 0.5);
        for i in 1..7 {
            assert_eq!(positions[i], READY_POSE[i]);
        }
    }

    /// The same filter, given a second, further command: `last_positions`/
    /// `last_velocities` from the first call carry forward correctly.
    /// Fixture case 1, second step.
    #[test]
    fn unconstrained_threads_state_across_two_steps() {
        let mut filter = reset_filter(&[2.0; 7], &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[0] = 0.5;
        let mut velocities = [0.0; 7];
        velocities[0] = 0.5;
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();

        positions[0] = 1.0;
        velocities[0] = 1.0;
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        assert_eq!(positions[0], 1.0);
        assert_eq!(velocities[0], 0.5);
    }

    /// One bound binds alone, on the group's first joint: a target `5.0`
    /// away with only `max_acceleration = 1.0` throttles the step to
    /// `alpha == 0.8` exactly (`1.0 - 1.0/5.0`). Fixture case 2.
    #[test]
    fn single_bound_binds_alone_on_the_first_joint() {
        let mut max_acceleration_limits = [2.0; 7];
        max_acceleration_limits[0] = 1.0;
        let mut filter = reset_filter(&max_acceleration_limits, &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[0] = 5.0;
        let mut velocities = [0.0; 7];
        velocities[0] = 5.0;
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        assert!((positions[0] - 1.0).abs() < 1e-9, "{positions:?}");
        assert!((velocities[0] - 1.0).abs() < 1e-9, "{velocities:?}");
    }

    /// The same binding-alone case, but on the group's fourth joint rather
    /// than its first -- guards the per-joint bound lookup against an
    /// off-by-index mistake that a first-joint-only case could not catch.
    /// Fixture case 4.
    #[test]
    fn single_bound_binds_alone_on_the_fourth_joint() {
        let mut max_acceleration_limits = [2.0; 7];
        max_acceleration_limits[3] = 1.0;
        let mut filter = reset_filter(&max_acceleration_limits, &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[3] = READY_POSE[3] + 5.0;
        let mut velocities = [0.0; 7];
        velocities[3] = positions[3];
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        let expected = 0.8 * READY_POSE[3] + 0.2 * (READY_POSE[3] + 5.0);
        assert!((positions[3] - expected).abs() < 1e-9, "{positions:?}");
        assert!((velocities[3] - 1.0).abs() < 1e-9, "{velocities:?}");
    }

    /// The interval intersection is empty: `panda_joint1` keeps its target
    /// position (`offset == 0.0`) but carries a huge last velocity (`100.0
    /// rad/s`) that a tiny acceleration limit (`1.0`) cannot arrest within
    /// one `update_period` -- infeasible for *every* `alpha`, so
    /// `solve_alpha` returns `None` and the decelerate-toward-rest fallback
    /// runs. Fixture case 5; the fallback's own numbers are hand-derivable
    /// exactly (`jointLimitAccelerationScalingFactor` clamps `-100.0` to
    /// `-1.0`, a `0.01` scale, giving `velocity = 100.0 + (-1.0) = 99.0`).
    #[test]
    fn empty_intersection_falls_back_to_decelerate_toward_rest() {
        let mut max_acceleration_limits = [2.0; 7];
        max_acceleration_limits[0] = 1.0;
        let mut last_velocities = [0.0; 7];
        last_velocities[0] = 100.0;
        let mut filter = reset_filter(&max_acceleration_limits, &last_velocities);
        let mut positions = READY_POSE;
        let mut velocities = [0.0; 7];
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        assert_eq!(positions[0], 99.0);
        assert_eq!(velocities[0], 99.0);
        for i in 1..7 {
            assert_eq!(positions[i], READY_POSE[i]);
            assert_eq!(velocities[i], 0.0);
        }
    }

    /// Both offsets are below `COMMAND_DIFFERENCE_THRESHOLD`: the command
    /// holds at `last_positions`/`last_velocities` rather than reaching for
    /// osqp at all. Fixture case 6.
    #[test]
    fn tiny_offset_holds_at_the_last_command() {
        let mut filter = reset_filter(&[2.0; 7], &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[0] = 1e-5;
        let mut velocities = [0.0; 7];
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        assert_eq!(positions[0], 0.0);
        assert_eq!(velocities[0], 0.0);
    }

    /// The interval intersection is a single point: `panda_joint1` is given
    /// a zero-width acceleration bound (`min == max == 0.0`) while its
    /// target differs from its last position, which forces the exact
    /// optimum to `alpha == 1.0` (no motion at all) precisely -- see the
    /// module doc's "Deviations from upstream" note on why upstream's own
    /// `osqp` answer lands close to, not at, that point, and why this case's
    /// tolerance is measured rather than reused from another case. Fixture
    /// case 3.
    #[test]
    fn single_point_intersection_forces_alpha_to_one() {
        let mut max_acceleration_limits = [2.0; 7];
        max_acceleration_limits[0] = 0.0;
        let mut filter = reset_filter(&max_acceleration_limits, &[0.0; 7]);
        let mut positions = READY_POSE;
        positions[0] = 3.0;
        let mut velocities = [0.0; 7];
        velocities[0] = 3.0;
        filter
            .do_smoothing(&mut positions, &mut velocities)
            .unwrap();
        // Exact closed-form answer: alpha == 1.0, so position/velocity == 0.0.
        assert_eq!(positions[0], 0.0);
        assert_eq!(velocities[0], 0.0);
        // Upstream's own osqp answer for this exact case, measured directly
        // from `acceleration_filter_response.json`'s third case: within
        // 2e-3 of this port's exact 0.0, well inside osqp's default
        // `eps_abs = eps_rel = 1e-3` and well outside plain float noise --
        // see the module doc for the full derivation.
        let upstream_osqp_position = 0.0008294991991130152;
        assert!((positions[0] - upstream_osqp_position).abs() < 2e-3);
    }
}
