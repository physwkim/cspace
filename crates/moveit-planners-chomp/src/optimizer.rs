// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_optimizer.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_optimizer.cpp

//! The model/collision-independent numeric core of `chomp::ChompOptimizer`.
//!
//! # Scope: free functions, not a `ChompOptimizer` struct
//!
//! Upstream's `ChompOptimizer` is one class whose constructor requires a
//! live `planning_scene::PlanningSceneConstPtr` and whose central method,
//! `optimize()`, is a single loop that inseparably interleaves two costs:
//! smoothness (a pure function of the trajectory, already portable via
//! [`crate::cost::ChompCost`]) and collision (a function of
//! `collision_detection::CollisionEnvHybrid`/`GroupStateRepresentation` —
//! types with no counterpart in `moveit-collision`/`moveit-scene`, which
//! this crate does not and must not depend on; those crates are other
//! workers' scope this round). This is not a missing-dependency situation
//! `moveit-collision` could plug in later without a semantic change: the
//! *struct itself* stores per-collision-point bookkeeping
//! (`collision_point_pos_eigen_`, `_vel_eigen_`, `_acc_eigen_`,
//! `_potential_`, `_potential_gradient_`, `joint_axes_`, `joint_positions_`,
//! `state_is_in_collision_`, `point_is_in_collision_`) that has no meaning
//! without a live collision environment, so a faithful `ChompOptimizer`
//! port cannot be constructed at all without one.
//!
//! Rather than invent a stub collision environment (not upstream's design)
//! or silently drop the whole file, every one of `ChompOptimizer`'s private
//! methods was read and classified individually. The ones that touch only
//! already-ported types ([`crate::trajectory::ChompTrajectory`],
//! [`crate::cost::ChompCost`], [`crate::parameters::ChompParameters`],
//! `moveit_model`'s joint tree) are ported here as free functions, each
//! taking exactly the state it needs rather than a `&ChompOptimizer` that
//! cannot exist in this crate. The rest are named below, not silently
//! absent.
//!
//! ## Precondition: `group_trajectory` must already be `DIFF_RULE_LENGTH`-padded
//!
//! Every function below that takes a `group_trajectory: &ChompTrajectory`
//! assumes its free block (`start_index()..=end_index()`, i.e.
//! [`ChompTrajectory::num_free_points`]) already coincides with the
//! `joint_costs`' own internal free-variable count
//! ([`ChompCost::quadratic_cost_inverse`]'s dimension). This is **not**
//! true of a trajectory built directly via
//! [`ChompTrajectory::from_num_points`] (default `start_index = 1`,
//! `end_index = num_points - 2`, so `num_free_points() = num_points - 2`);
//! it is only true once the trajectory has been padded via
//! [`ChompTrajectory::from_source_trajectory`] with
//! `diff_rule_length = `[`crate::utils::DIFF_RULE_LENGTH`] (`start_index =
//! DIFF_RULE_LENGTH - 1`, `end_index = num_points - 1 - (DIFF_RULE_LENGTH -
//! 1)`, so `num_free_points() = num_points - 2*(DIFF_RULE_LENGTH - 1)` —
//! exactly [`ChompCost::new`]'s own `num_vars_free` formula). Confirmed
//! against `chomp_optimizer.cpp:90-95`'s `initialize()`
//! (`num_vars_free_ = group_trajectory_.getNumFreePoints(); free_vars_start_
//! = group_trajectory_.getStartIndex(); free_vars_end_ =
//! group_trajectory_.getEndIndex();`) alongside `ChompCost`'s own
//! constructor: upstream's real call path (`chomp_planner.cpp`, not ported
//! this round) always constructs `group_trajectory_` via the padding
//! constructor before handing it to `ChompOptimizer`, which is the only
//! reason these two independently-computed free-variable counts agree at
//! all — a plain `from_num_points` trajectory has no such guarantee, and
//! every function below returns a typed error rather than a silently wrong
//! answer if `joint_costs`' dimension does not match `group_trajectory`'s
//! free block. Every test in this module builds its fixture through the
//! padding constructor for exactly this reason.
//!
//! ## Ported (this module)
//!
//! - `getPotential` (inline in the header) → [`crate::optimizer::get_potential`]. Pure
//!   3-branch scalar formula; no `ChompOptimizer` state at all.
//! - `calculateSmoothnessIncrements` → [`crate::optimizer::calculate_smoothness_increments`].
//!   Built entirely from [`crate::cost::ChompCost::derivative`] (already
//!   ported) and [`crate::trajectory::ChompTrajectory`] accessors.
//! - `calculateTotalIncrements` → [`crate::optimizer::calculate_total_increments`]. **This is
//!   the round's weighted-combination callout.** Upstream:
//!   `final_increments_.col(i) = learning_rate_ * (quad_cost_inv *
//!   (smoothness_cost_weight_ * smoothness_increments_.col(i) +
//!   obstacle_cost_weight_ * collision_increments_.col(i)))`. The three
//!   coefficients are, by name, exactly
//!   [`crate::parameters::ChompParameters::learning_rate`],
//!   [`crate::parameters::ChompParameters::smoothness_cost_weight`], and
//!   [`crate::parameters::ChompParameters::obstacle_cost_weight`] — all
//!   already-ported fields, confirmed against `chomp_optimizer.cpp`'s
//!   `calculateTotalIncrements` body directly (not inferred from the field
//!   names alone). This function takes `collision_increments` as a plain
//!   `&DMatrix<f64>` input rather than computing it internally, which is
//!   exactly what makes it portable: the formula does not care whether the
//!   caller's collision increments came from a real collision environment
//!   or a test fixture.
//! - `addIncrementsToTrajectory` → [`crate::optimizer::add_increments_to_trajectory`]. Pure
//!   per-joint scale-and-clamp against
//!   [`crate::parameters::ChompParameters::joint_update_limit`], writing
//!   through [`crate::trajectory::ChompTrajectory::free_trajectory_block_mut`].
//! - `getSmoothnessCost` → [`crate::optimizer::get_smoothness_cost`]. Sum of
//!   [`crate::cost::ChompCost::cost`] per joint, weighted by
//!   [`crate::parameters::ChompParameters::smoothness_cost_weight`] — the
//!   other half of the round's weight-mapping callout (the collision half,
//!   `getCollisionCost`'s `obstacle_cost_weight_`, is named in
//!   [`crate::optimizer::calculate_total_increments`]'s doc above since `getCollisionCost`
//!   itself is not portable — see below).
//! - `handleJointLimits` → [`crate::optimizer::handle_joint_limits`]. Needs only
//!   [`crate::trajectory::ChompTrajectory`], [`crate::cost::ChompCost`], and
//!   `moveit_model`'s joint bounds (`JointModel::variable_bounds`,
//!   `RevoluteJoint::is_continuous`) — all already dependencies of this
//!   crate, no collision environment involved.
//!
//! ## Not ported: collision-coupled (need `moveit-collision`/`moveit-scene`)
//!
//! Named individually, not silently absent — every one of these appears in
//! `chomp_optimizer.{hpp,cpp}` and was read in full:
//!
//! - `ChompOptimizer` (the class itself), its constructor, `optimize()`,
//!   `destroy()`, `isInitialized()`, `isCollisionFree()` — `optimize()`'s
//!   termination condition (this round's other callout) is documented
//!   below in full, as a specification, since it cannot be executed without
//!   the collision half it depends on.
//! - `performForwardKinematics` — populates every `collision_point_*`
//!   field via `hy_env_->getCollisionGradients(...)`; the only caller of
//!   [`crate::optimizer::get_potential`] upstream.
//! - `getCollisionCost`, `getTrajectoryCost` — read `collision_point_*`
//!   populated by `performForwardKinematics`.
//! - `calculateCollisionIncrements`, `calculatePseudoInverse`, `getJacobian`
//!   — collision-point Jacobian computation.
//! - `computeJointProperties`, `setRobotStateFromPoint` — forward kinematics
//!   against a live `moveit::core::RobotState`, feeding `getJacobian`.
//! - `registerParents`, the private `isParent` (inline) and
//!   `joint_parent_map_` — built once by the constructor solely to answer
//!   `getJacobian`'s "is this joint an ancestor of this collision point's
//!   link" query; no other consumer.
//! - `isCurrentTrajectoryMeshToMeshCollisionFree` — calls
//!   `planning_scene_->isPathValid(...)`.
//! - `debugCost` — confirmed dead: not called anywhere in
//!   `chomp_optimizer.cpp` (a `std::cout` debug helper, same formula as
//!   [`crate::optimizer::get_smoothness_cost`] but unweighted and unused).
//! - `perturbTrajectory`, `getRandomMomentum`, `updateMomentum`,
//!   `updatePositionFromMomentum` (the HMC path) — confirmed dead in
//!   upstream itself, not merely out of this crate's scope: every call site
//!   in `optimize()` is commented out (`/// TODO: HMC BASED COMMENTED
//!   CODE...`), and `getRandomMomentum`/`updateMomentum`/
//!   `updatePositionFromMomentum` have **no implementation anywhere in
//!   `chomp_optimizer.cpp`** — declared in the header (lines 212-214) and
//!   never defined, which only compiles because nothing calls them. This is
//!   why this crate does not depend on `moveit-sampling` this round despite
//!   the round-16 dispatch's `MultivariateGaussian` note: the one call site
//!   (`multivariate_gaussian_` in `initialize()`, feeding exactly this dead
//!   HMC path) has no live consumer to port it against.
//!
//! ## `optimize()`'s termination condition (specification only)
//!
//! Transcribed from `chomp_optimizer.cpp:290-518` for the record, since
//! this round's brief calls it out specifically, even though it cannot be
//! executed here. The loop runs `iteration_` from `0` to
//! [`crate::parameters::ChompParameters::max_iterations`] (exclusive),
//! computing `cost = collision_cost + smoothness_cost` each iteration and
//! tracking the minimum-cost trajectory seen (`best_group_trajectory_`,
//! restored at the very end regardless of how the loop exits — the
//! returned trajectory is never simply "the last iteration's"). Within an
//! iteration, three independent conditions can end the loop early, checked
//! in this order:
//!
//! 1. Every 10th iteration (`iteration_ % 10 == 0`), a full mesh-to-mesh
//!    collision check
//!    ([`isCurrentTrajectoryMeshToMeshCollisionFree`](#not-ported-collision-coupled-need-moveit-collision-moveit-scene))
//!    against the *current* trajectory (not `best_group_trajectory_`); a
//!    pass sets `num_collision_free_iterations_ = 0` (break on the very
//!    next check below).
//! 2. Unless [`crate::parameters::ChompParameters::filter_mode`] is set, the
//!    scalar comparison `collision_cost <
//!    `[`crate::parameters::ChompParameters::collision_threshold`]` — an
//!    **absolute**, not relative, comparison against one scalar (the raw
//!    collision cost, *not* `obstacle_cost_weight_ *` collision cost —
//!    confirmed from `getCollisionCost`'s call site in `optimize()`, which
//!    reads `c_cost` before the weight is applied inside `getCollisionCost`
//!    itself, i.e. `collision_threshold_` is compared in the *same*
//!    already-weighted units `getCollisionCost()` returns). On first
//!    satisfying this, `num_collision_free_iterations_` is set to
//!    [`crate::parameters::ChompParameters::max_iterations_after_collision_free`]
//!    — the optimizer keeps refining for up to that many further
//!    iterations rather than stopping immediately (immediately only when
//!    that parameter is `0`).
//! 3. A wall-clock check, `elapsed >`
//!    [`crate::parameters::ChompParameters::planning_time_limit`] — a plain
//!    `break`, no `should_break_out`/counter bookkeeping, checked
//!    unconditionally every iteration regardless of collision state.
//!
//! Once either early-exit condition (1) or (2) fires,
//! `collision_free_iteration_` increments every iteration thereafter, and
//! the loop breaks once it exceeds `num_collision_free_iterations_` (or
//! immediately if that count is `0`) — this is the "run a few more
//! iterations after first going collision-free, then stop" behavior.
//! `optimize()`'s boolean return is simply the final `is_collision_free_`
//! flag: `true` only if a collision-free trajectory was found before
//! `max_iterations_` ran out (or the wall clock did), regardless of the
//! final smoothness cost.

use crate::cost::ChompCost;
use crate::parameters::ChompParameters;
use crate::trajectory::ChompTrajectory;
use moveit_error::{Error, Result};
use moveit_model::joint::JointType;
use moveit_model::{JointModelGroup, RobotModel};
use nalgebra::DMatrix;

/// The CHOMP potential function: a 3-branch scalar penalty for a collision
/// point at `field_distance` from the nearest obstacle, with a sphere of
/// `radius` and a desired clearance of `clearance`.
///
/// Ported from the private inline `getPotential`
/// (`chomp_optimizer.hpp:84-102`).
pub fn get_potential(field_distance: f64, radius: f64, clearance: f64) -> f64 {
    let d = field_distance - radius;
    if d >= clearance {
        0.0
    } else if d >= 0.0 {
        let diff = d - clearance;
        let gradient_magnitude = diff / clearance;
        0.5 * gradient_magnitude * diff
    } else {
        -d + 0.5 * clearance
    }
}

/// Computes the smoothness-cost increment for every joint and every free
/// trajectory point: `-derivative(joint_trajectory)` restricted to the free
/// (non-boundary-padding) segment, one column per joint.
///
/// `joint_costs[i]` must be the [`ChompCost`] for `group_trajectory`'s
/// joint column `i`; a length mismatch is a typed error rather than an
/// out-of-bounds panic.
///
/// Ported from `calculateSmoothnessIncrements`.
pub fn calculate_smoothness_increments(
    joint_costs: &[ChompCost],
    group_trajectory: &ChompTrajectory,
) -> Result<DMatrix<f64>> {
    let num_joints = group_trajectory.num_joints();
    if joint_costs.len() != num_joints {
        return Err(Error::other(format!(
            "joint_costs has {} entries, expected {num_joints}",
            joint_costs.len()
        )));
    }
    let num_vars_free = group_trajectory.num_free_points();
    let start_index = group_trajectory.start_index();
    let mut increments = DMatrix::<f64>::zeros(num_vars_free, num_joints);
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        let derivative = joint_cost.derivative(&group_trajectory.joint_trajectory(i))?;
        for r in 0..num_vars_free {
            increments[(r, i)] = -derivative[start_index + r];
        }
    }
    Ok(increments)
}

/// Combines smoothness and collision increments into the final per-joint
/// trajectory update, weighted by
/// [`ChompParameters::smoothness_cost_weight`]/
/// [`ChompParameters::obstacle_cost_weight`] and scaled by
/// [`ChompParameters::learning_rate`] — see this module's doc comment for
/// the full field-name mapping.
///
/// `smoothness_increments` and `collision_increments` must have the same
/// shape (`num_vars_free` rows, `joint_costs.len()` columns); a mismatch
/// (either between the two matrices, or against `joint_costs`) is a typed
/// error.
///
/// Ported from `calculateTotalIncrements`.
pub fn calculate_total_increments(
    joint_costs: &[ChompCost],
    smoothness_increments: &DMatrix<f64>,
    collision_increments: &DMatrix<f64>,
    parameters: &ChompParameters,
) -> Result<DMatrix<f64>> {
    let num_joints = joint_costs.len();
    if smoothness_increments.ncols() != num_joints || collision_increments.ncols() != num_joints {
        return Err(Error::other(format!(
            "joint_costs has {num_joints} entries, but smoothness_increments has {} columns and collision_increments has {} columns",
            smoothness_increments.ncols(),
            collision_increments.ncols()
        )));
    }
    if smoothness_increments.nrows() != collision_increments.nrows() {
        return Err(Error::other(format!(
            "smoothness_increments has {} rows but collision_increments has {} rows",
            smoothness_increments.nrows(),
            collision_increments.nrows()
        )));
    }
    let num_vars_free = smoothness_increments.nrows();

    let mut final_increments = DMatrix::<f64>::zeros(num_vars_free, num_joints);
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        let quad_cost_inv = joint_cost.quadratic_cost_inverse();
        if quad_cost_inv.nrows() != num_vars_free || quad_cost_inv.ncols() != num_vars_free {
            return Err(Error::other(format!(
                "joint {i}'s quadratic_cost_inverse is {}x{}, expected {num_vars_free}x{num_vars_free}",
                quad_cost_inv.nrows(),
                quad_cost_inv.ncols()
            )));
        }
        let combined = parameters.smoothness_cost_weight * smoothness_increments.column(i)
            + parameters.obstacle_cost_weight * collision_increments.column(i);
        final_increments.set_column(i, &(parameters.learning_rate * (quad_cost_inv * combined)));
    }
    Ok(final_increments)
}

/// Adds `final_increments` into `group_trajectory`'s free block, scaling
/// each joint's column so its largest-magnitude entry never exceeds
/// [`ChompParameters::joint_update_limit`].
///
/// `final_increments` must be `group_trajectory.num_free_points()` rows by
/// `group_trajectory.num_joints()` columns; a mismatch is a typed error.
/// Matches upstream's own IEEE-754 behavior on an all-zero column exactly:
/// `joint_update_limit / 0.0.abs()` is `+inf`, which never scales `scale`
/// down (`inf < 1.0` is `false`), the same outcome
/// `parameters_->joint_update_limit_ / fabs(0.0)` reaches upstream — not a
/// special case added here.
///
/// Ported from `addIncrementsToTrajectory`.
pub fn add_increments_to_trajectory(
    group_trajectory: &mut ChompTrajectory,
    final_increments: &DMatrix<f64>,
    joint_update_limit: f64,
) -> Result<()> {
    let num_joints = group_trajectory.num_joints();
    let num_vars_free = group_trajectory.num_free_points();
    if final_increments.nrows() != num_vars_free || final_increments.ncols() != num_joints {
        return Err(Error::other(format!(
            "final_increments is {}x{}, expected {num_vars_free}x{num_joints}",
            final_increments.nrows(),
            final_increments.ncols()
        )));
    }

    let mut block = group_trajectory.free_trajectory_block_mut();
    for i in 0..num_joints {
        let max = final_increments.column(i).max();
        let min = final_increments.column(i).min();
        let mut scale = 1.0f64;
        let max_scale = joint_update_limit / max.abs();
        let min_scale = joint_update_limit / min.abs();
        if max_scale < scale {
            scale = max_scale;
        }
        if min_scale < scale {
            scale = min_scale;
        }
        let mut col = block.column_mut(i);
        col += scale * final_increments.column(i);
    }
    Ok(())
}

/// The weighted sum of every joint's smoothness cost:
/// [`ChompParameters::smoothness_cost_weight`] times the sum of
/// [`ChompCost::cost`] over `joint_costs`/`group_trajectory`'s matching
/// joint columns.
///
/// Ported from `getSmoothnessCost`.
pub fn get_smoothness_cost(
    joint_costs: &[ChompCost],
    group_trajectory: &ChompTrajectory,
    smoothness_cost_weight: f64,
) -> Result<f64> {
    let mut smoothness_cost = 0.0;
    for (i, joint_cost) in joint_costs.iter().enumerate() {
        smoothness_cost += joint_cost.cost(&group_trajectory.joint_trajectory(i))?;
    }
    Ok(smoothness_cost_weight * smoothness_cost)
}

/// Repairs joint-limit violations in `group_trajectory`'s free block, one
/// joint at a time: up to 10 passes each pushing the single worst-violating
/// free trajectory point back toward its bound by
/// `violation / quad_cost_inv[(free_var_index, free_var_index)]` along
/// `quad_cost_inv`'s corresponding column (a smoothness-respecting
/// correction, not a hard clamp). Continuous revolute joints
/// ([`moveit_model::joint::RevoluteJoint::is_continuous`]) are skipped
/// entirely, matching upstream (a continuous joint has no bound to
/// violate).
///
/// `group`'s active joint count must equal `joint_costs.len()`; a mismatch
/// is a typed error rather than an out-of-bounds panic on either.
///
/// Ported from `handleJointLimits`.
pub fn handle_joint_limits(
    robot_model: &RobotModel,
    group: &JointModelGroup,
    group_trajectory: &mut ChompTrajectory,
    joint_costs: &[ChompCost],
) -> Result<()> {
    let active_joints = group.active_joint_indices();
    if active_joints.len() != joint_costs.len() {
        return Err(Error::other(format!(
            "group {:?} has {} active joints, but joint_costs has {} entries",
            group.name(),
            active_joints.len(),
            joint_costs.len()
        )));
    }

    let free_vars_start = group_trajectory.start_index();
    let free_vars_end = group_trajectory.end_index();

    for (joint_i, &model_index) in active_joints.iter().enumerate() {
        let joint_model = robot_model.joint_model_at(model_index);
        if joint_model.joint_type() == JointType::Revolute {
            if let Some(revolute) = joint_model.as_revolute() {
                if revolute.is_continuous() {
                    continue;
                }
            }
        }

        let mut joint_max = f64::MIN;
        let mut joint_min = f64::MAX;
        for bound in joint_model.variable_bounds() {
            if bound.min_position < joint_min {
                joint_min = bound.min_position;
            }
            if bound.max_position > joint_max {
                joint_max = bound.max_position;
            }
        }

        let mut count = 0;
        loop {
            let mut max_abs_violation = 1e-6;
            let mut max_violation = 0.0;
            let mut max_violation_index = free_vars_start;
            let mut violation = false;

            for i in free_vars_start..=free_vars_end {
                let value = group_trajectory[(i, joint_i)];
                let (amount, absolute_amount) = if value > joint_max {
                    let amount = joint_max - value;
                    (amount, amount.abs())
                } else if value < joint_min {
                    let amount = joint_min - value;
                    (amount, amount.abs())
                } else {
                    (0.0, 0.0)
                };
                if absolute_amount > max_abs_violation {
                    max_abs_violation = absolute_amount;
                    max_violation = amount;
                    max_violation_index = i;
                    violation = true;
                }
            }

            if violation {
                let free_var_index = max_violation_index - free_vars_start;
                let quad_cost_inv = joint_costs[joint_i].quadratic_cost_inverse();
                let multiplier = max_violation / quad_cost_inv[(free_var_index, free_var_index)];
                let update = quad_cost_inv.column(free_var_index) * multiplier;
                let mut block = group_trajectory.free_joint_trajectory_block_mut(joint_i);
                block += update;
            }

            count += 1;
            if count > 10 || !violation {
                break;
            }
        }
    }
    Ok(())
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

    /// Builds a `DIFF_RULE_LENGTH`-padded trajectory (see this module's
    /// "Precondition" doc section) with `source_num_points` points before
    /// padding.
    fn trajectory(source_num_points: usize) -> ChompTrajectory {
        let source = ChompTrajectory::from_num_points(panda_model(), source_num_points, 0.1, GROUP)
            .expect("valid num_points");
        ChompTrajectory::from_source_trajectory(&source, GROUP, crate::utils::DIFF_RULE_LENGTH)
            .expect("valid padding")
    }

    fn joint_costs(traj: &ChompTrajectory, ridge_factor: f64) -> Vec<ChompCost> {
        (0..traj.num_joints())
            .map(|_| ChompCost::new(traj, &[0.0, 1.0, 0.0], ridge_factor).unwrap())
            .collect()
    }

    // get_potential: one case per branch plus both boundaries.
    #[test]
    fn get_potential_is_zero_at_and_beyond_clearance() {
        assert_relative_eq!(get_potential(1.0, 0.0, 0.2), 0.0, epsilon = EPS);
        assert_relative_eq!(get_potential(2.0, 0.0, 0.2), 0.0, epsilon = EPS);
    }

    #[test]
    fn get_potential_transition_branch_matches_closed_form() {
        // d = field_distance - radius = 0.1, clearance = 0.2: transition
        // branch. diff = 0.1-0.2 = -0.1, gradient = -0.5, 0.5*-0.5*-0.1 =
        // 0.025.
        let got = get_potential(0.3, 0.2, 0.2);
        assert_relative_eq!(got, 0.025, epsilon = EPS);
    }

    #[test]
    fn get_potential_at_d_equals_zero_matches_both_adjacent_branches() {
        // d == 0.0 exactly: takes the transition branch (d >= 0.0), and by
        // continuity must equal the collision branch's limit as d -> 0^-.
        let clearance = 0.4;
        let transition = get_potential(0.5, 0.5, clearance);
        assert_relative_eq!(transition, 0.5 * clearance, epsilon = EPS);
    }

    #[test]
    fn get_potential_collision_branch_matches_closed_form() {
        // d = -0.05, clearance = 0.2: collision branch, -d + 0.5*clearance.
        let got = get_potential(0.15, 0.2, 0.2);
        assert_relative_eq!(got, 0.05 + 0.1, epsilon = EPS);
    }

    #[test]
    fn calculate_smoothness_increments_rejects_joint_costs_length_mismatch() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let err = calculate_smoothness_increments(&costs[..costs.len() - 1], &traj).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn calculate_smoothness_increments_is_negative_gradient_restricted_to_free_rows() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let increments = calculate_smoothness_increments(&costs, &traj).unwrap();
        assert_eq!(increments.nrows(), traj.num_free_points());
        assert_eq!(increments.ncols(), traj.num_joints());

        let derivative = costs[0].derivative(&traj.joint_trajectory(0)).unwrap();
        for r in 0..traj.num_free_points() {
            assert_relative_eq!(
                increments[(r, 0)],
                -derivative[traj.start_index() + r],
                epsilon = EPS,
                max_relative = EPS
            );
        }
    }

    #[test]
    fn calculate_total_increments_matches_hand_rolled_weighted_combination() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let num_free = traj.num_free_points();
        let num_joints = traj.num_joints();

        let smoothness = DMatrix::<f64>::from_fn(num_free, num_joints, |r, c| (r + c) as f64);
        let collision = DMatrix::<f64>::from_fn(num_free, num_joints, |r, c| (r * c) as f64 * 0.1);

        let parameters = ChompParameters {
            smoothness_cost_weight: 0.3,
            obstacle_cost_weight: 0.7,
            learning_rate: 0.05,
            ..ChompParameters::default()
        };

        let got = calculate_total_increments(&costs, &smoothness, &collision, &parameters).unwrap();

        let expected_col0 = parameters.learning_rate
            * (costs[0].quadratic_cost_inverse()
                * (parameters.smoothness_cost_weight * smoothness.column(0)
                    + parameters.obstacle_cost_weight * collision.column(0)));
        for r in 0..num_free {
            assert_relative_eq!(
                got[(r, 0)],
                expected_col0[r],
                epsilon = EPS,
                max_relative = EPS
            );
        }
    }

    #[test]
    fn calculate_total_increments_rejects_column_count_mismatch() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let num_free = traj.num_free_points();
        let smoothness = DMatrix::<f64>::zeros(num_free, costs.len());
        let collision = DMatrix::<f64>::zeros(num_free, costs.len() - 1);
        let parameters = ChompParameters::default();
        let err =
            calculate_total_increments(&costs, &smoothness, &collision, &parameters).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn add_increments_to_trajectory_clamps_to_joint_update_limit() {
        let mut traj = trajectory(20);
        let num_free = traj.num_free_points();
        let num_joints = traj.num_joints();
        // Column 0 has max entry 10.0 -> scale = joint_update_limit / 10.0.
        let mut final_increments = DMatrix::<f64>::zeros(num_free, num_joints);
        final_increments[(0, 0)] = 10.0;
        final_increments[(1, 0)] = -2.0;

        let joint_update_limit = 0.1;
        add_increments_to_trajectory(&mut traj, &final_increments, joint_update_limit).unwrap();

        let scale = joint_update_limit / 10.0;
        assert_relative_eq!(
            traj[(traj.start_index(), 0)],
            scale * 10.0,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            traj[(traj.start_index() + 1, 0)],
            scale * -2.0,
            epsilon = EPS,
            max_relative = EPS
        );
    }

    #[test]
    fn add_increments_to_trajectory_rejects_shape_mismatch() {
        let mut traj = trajectory(20);
        let wrong = DMatrix::<f64>::zeros(traj.num_free_points() - 1, traj.num_joints());
        let err = add_increments_to_trajectory(&mut traj, &wrong, 0.1).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn get_smoothness_cost_matches_weighted_sum_of_per_joint_cost() {
        let traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-6);
        let weight = 0.4;
        let got = get_smoothness_cost(&costs, &traj, weight).unwrap();

        let mut expected = 0.0;
        for (i, joint_cost) in costs.iter().enumerate() {
            expected += joint_cost.cost(&traj.joint_trajectory(i)).unwrap();
        }
        expected *= weight;
        assert_relative_eq!(got, expected, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn handle_joint_limits_pulls_an_out_of_bounds_point_back_toward_the_bound() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);

        let joint_model = model.joint_model_at(group.active_joint_indices()[0]);
        let joint_max = joint_model
            .variable_bounds()
            .iter()
            .map(|b| b.max_position)
            .fold(f64::MIN, f64::max);

        let mid = (traj.start_index() + traj.end_index()) / 2;
        let violating = joint_max + 10.0;
        let mut row = traj.trajectory_point(mid);
        row[0] = violating;
        traj.set_trajectory_point(mid, &row);

        handle_joint_limits(model, group, &mut traj, &costs).unwrap();

        assert!(
            traj[(mid, 0)] < violating,
            "expected handle_joint_limits to reduce the violating value, got {}",
            traj[(mid, 0)]
        );
    }

    #[test]
    fn handle_joint_limits_rejects_joint_costs_length_mismatch() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);
        let err =
            handle_joint_limits(model, group, &mut traj, &costs[..costs.len() - 1]).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn handle_joint_limits_is_a_noop_when_no_bound_is_violated() {
        let model = panda_model();
        let group = model.joint_model_group(GROUP).unwrap();
        let mut traj = trajectory(20);
        let costs = joint_costs(&traj, 1e-3);
        let before = traj.clone();
        handle_joint_limits(model, group, &mut traj, &costs).unwrap();
        assert_eq!(traj, before);
    }
}
