// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/cost_functions.hpp

//! `stomp_moveit::costs`: the generic (no `PlanningScene`) half of
//! `cost_functions.hpp`.
//!
//! # Ported: `getCostFunctionFromStateValidator`, `costs::sum`
//!
//! Both operate on an arbitrary [`StateValidatorFn`] the caller supplies --
//! no `moveit_core::planning_scene::PlanningScene` dependency, so both are
//! in reach of this crate.
//!
//! # Round 24: `getCollisionCostFunction`/`getConstraintsCostFunction`, the `PlanningScene`-backed half
//!
//! An earlier round deferred both as needing a dependency this crate did
//! not have; that was checked and found false (`cargo tree -p
//! cspace_planning::scene -e normal`/`cargo tree -p cspace-collision -e normal`
//! neither lists `stomp` -- no cycle -- and the sibling planner crate
//! `cspace_planners::sbp` already depends on both). Both are ported this
//! round: [`get_collision_cost_function`] (`costs::getCollisionCostFunction`,
//! `cost_functions.hpp:199-216`) and [`get_constraints_cost_function`]
//! (`costs::getConstraintsCostFunction`, `cost_functions.hpp:230-250`).
//!
//! # Deviation: `&KinematicConstraintSet`, not `moveit_msgs::msg::Constraints`
//!
//! Upstream's `getConstraintsCostFunction` takes a ROS
//! `moveit_msgs::msg::Constraints` and builds a `KinematicConstraintSet`
//! internally (`constraints.add(constraints_msg,
//! planning_scene->getTransforms())`, `cost_functions.hpp:236-237`). D1
//! excludes `moveit_msgs` types; this port takes an already-built
//! `&KinematicConstraintSet` directly instead, the same signature
//! `cspace_planners::sbp::planning_scene_validity::PlanningSceneValidityChecker`
//! already established for the same reason.
//!
//! # Deviation: interior mutability, not upstream's private per-closure state clone
//!
//! Upstream's `PlanningScene` is `const`/shared
//! (`std::shared_ptr<const planning_scene::PlanningScene>`); each factory
//! builds its own private `static moveit::core::RobotState state(...)`
//! clone inside the closure, so `getCollisionCostFunction` and
//! `getConstraintsCostFunction` can be combined via `costs::sum` against
//! one shared `planning_scene` with no aliasing concern. This port's
//! `PlanningScene` collapsed upstream's const/non-const method-overload
//! pairs into `&mut self`-only methods that always act on the scene's own
//! current state (see `PlanningScene::is_state_colliding`'s own doc) --
//! there is no explicit-state, non-mutating overload to call instead. Both
//! functions below take `&'a RefCell<&'a mut PlanningScene<'m>>`, the same
//! bridge `cspace_planners::sbp::planning_scene_validity::PlanningSceneValidityChecker`
//! already uses to combine a collision check and a constraints check
//! against one scene, so a caller wanting both (matching upstream's own
//! typical `costs::sum({getCollisionCostFunction(...),
//! getConstraintsCostFunction(...)})` usage) shares one `RefCell` between
//! both factory calls rather than needing two scenes.
//! [`get_constraints_cost_function`] does not call any collision-specific
//! `PlanningScene` method -- it only reaches `current_state_mut()`, the
//! same as upstream's own body, which never calls a
//! `planning_scene`-specific method either past `getCurrentState()` -- but
//! keeps the `PlanningScene`-typed parameter anyway so it can share the
//! collision function's `RefCell` when both are composed.
//!
//! # `long` truncation in the Gaussian-smoothing kernel bounds
//!
//! `const long kernel_start = mu - static_cast<long>(sigma) * 4;` truncates
//! `sigma` to a `long` *before* multiplying by 4, then truncates the
//! `double` result of `mu - (double)that` to `long` again on assignment.
//! `sigma * 4.0` (no truncation) is a different number whenever `sigma`
//! is not already an integer (`sigma = max(1.0, 0.5 * window_size)` is
//! fractional for any even `window_size`). This port reproduces both
//! truncations with explicit `as i64` casts, split out into
//! `kernel_bounds` rather than computing in `f64` throughout.
//!
//! # No reachable input overflows the `i64` narrowing, measured not assumed
//!
//! `sigma`/`mu` derive only from `start`/`end` (`0 <= start <= end <
//! num_timesteps = values.ncols()`), and `sigma_offset = (sigma as i64) *
//! 4` only overflows `i64` once `num_timesteps` exceeds ~4.611e18. A
//! `DMatrix<f64>` cannot reach that: its backing `Vec<f64>` refuses to
//! allocate past `isize::MAX` bytes, capping any real (single-row, most
//! generous) trajectory's `num_timesteps` at `isize::MAX /
//! size_of::<f64>() ~= 1.153e18` -- about 4x below the overflow
//! threshold. `kernel_bounds`'s tests call the production arithmetic
//! directly at that ceiling (no `DMatrix` allocation needed) and confirm
//! it does not overflow under nextest's default `overflow-checks = true`
//! dev profile, rather than asserting the ceiling comparison in prose
//! alone.
//!
//! # Expiry (§153.1): this is a property of `DMatrix`, not of the algorithm
//!
//! The 4x margin above comes entirely from `values: &DMatrix<f64>` being
//! backed by a single contiguous `Vec<f64>`, whose own allocator caps
//! `num_timesteps` at `isize::MAX` bytes. `kernel_bounds` itself has no
//! such cap -- it takes a plain `usize`. If `cost_function_from_state_validator`
//! or its caller ever stops deriving `num_timesteps` from a
//! `Vec`-backed `DMatrix` (a memory-mapped or chunked trajectory
//! representation, a lazy/streamed column count, or widening the index
//! type past 64 bits), the margin this doc claims no longer holds and
//! the overflow becomes reachable -- re-run this module's `kernel_bounds`
//! tests against the new ceiling before relying on this claim again.

use std::cell::RefCell;

use nalgebra::{DMatrix, DVector};

use cspace_collision::{CollisionEnv, CollisionRequest};
use cspace_core::error::Result;
use cspace_core::model::JointModelGroup;
use cspace_core::state::Posed;
use cspace_planning::constraints::KinematicConstraintSet;
use cspace_planning::scene::PlanningScene;

use crate::stomp::composable_task::CostFn;
use crate::stomp::conversion_functions::set_positions;
use crate::stomp::require_single_variable;

/// `StateValidatorFn`: upstream's `std::function<double(const
/// Eigen::VectorXd&)>` -- a single state's positions in, a penalty out (`0.0`
/// valid, `> 0.0` invalid, magnitude is the cost).
pub type StateValidatorFn<'a> = Box<dyn Fn(&DVector<f64>) -> f64 + 'a>;

/// `COL_CHECK_DISTANCE` (`cost_functions.hpp:59`): the interpolation step
/// size [`get_collision_cost_function`] passes to
/// [`cost_function_from_state_validator`].
pub const COL_CHECK_DISTANCE: f64 = 0.05;

/// `CONSTRAINT_CHECK_DISTANCE` (`cost_functions.hpp:60`): the interpolation
/// step size [`get_constraints_cost_function`] passes to
/// [`cost_function_from_state_validator`].
pub const CONSTRAINT_CHECK_DISTANCE: f64 = 0.05;

/// `getCostFunctionFromStateValidator(state_validator_fn,
/// interpolation_step_size)`.
///
/// Per timestep: scores the waypoint itself via `state_validator_fn`; if
/// valid and not the last timestep and `interpolation_step_size > 0`, walks
/// interpolated samples toward the next waypoint at an L2-norm step size
/// (capped at a fraction of `0.5`) looking for the first invalid one, and if
/// found, splits that sample's penalty between the two bracketing
/// timesteps weighted by interpolation fraction. Contiguous invalid
/// timesteps form a "window"; each window's total cost is then spread as a
/// Gaussian centered at the window's midpoint (`sigma = max(1.0, 0.5 *
/// window_size)`, `+/- 4*sigma` truncated per this module's doc), rescaled
/// so the kernel's own total still equals the window's original total cost.
///
/// # Preserved: later windows can overwrite an earlier window's smoothed
/// costs
///
/// The kernel-write loop (`costs(j) = <gaussian density>`) overwrites
/// whatever `costs(j)` already holds, including a previous window's
/// already-smoothed contribution if two windows' `+/- 4*sigma` kernels
/// overlap. Upstream does this unconditionally; this port does too, rather
/// than silently accumulating instead of overwriting.
///
/// Always returns `Some` -- upstream's lambda always returns `true`; only
/// `validity` (not this function's own success) ever reports "invalid".
pub fn cost_function_from_state_validator<'a>(
    state_validator_fn: StateValidatorFn<'a>,
    interpolation_step_size: f64,
) -> CostFn<'a> {
    Box::new(move |values: &DMatrix<f64>| {
        let num_timesteps = values.ncols();
        let mut costs = DVector::zeros(num_timesteps);
        let mut validity = true;

        let mut invalid_windows: Vec<(usize, usize)> = Vec::new();
        let mut in_invalid_window = false;

        for timestep in 0..num_timesteps {
            let current = values.column(timestep).clone_owned();
            costs[timestep] += state_validator_fn(&current);
            let mut found_invalid_state = costs[timestep] > 0.0;

            let continue_interpolation = !found_invalid_state
                && timestep + 1 < num_timesteps
                && interpolation_step_size > 0.0;
            if continue_interpolation {
                let next = values.column(timestep + 1).clone_owned();
                let interpolation_step =
                    (0.5_f64).min(interpolation_step_size / (&next - &current).norm());
                let mut interpolation_fraction = interpolation_step;
                while interpolation_fraction < 1.0 {
                    let sample_vec =
                        (1.0 - interpolation_fraction) * &current + interpolation_fraction * &next;
                    let penalty = state_validator_fn(&sample_vec);
                    found_invalid_state = penalty > 0.0;
                    if found_invalid_state {
                        costs[timestep] += (1.0 - interpolation_fraction) * penalty;
                        costs[timestep + 1] += interpolation_fraction * penalty;
                        break;
                    }
                    interpolation_fraction += interpolation_step;
                }
            }

            if found_invalid_state {
                validity = false;
                if !in_invalid_window {
                    invalid_windows.push((timestep, timestep));
                    in_invalid_window = true;
                }
                invalid_windows.last_mut().unwrap().1 = timestep;
            } else {
                in_invalid_window = false;
            }
        }

        for &(start, end) in &invalid_windows {
            let window_cost: f64 = (start..=end).map(|i| costs[i]).sum();
            let window_size = (end - start) as f64 + 1.0;
            let sigma = (1.0_f64).max(0.5 * window_size);
            let mu = 0.5 * (start as f64 + end as f64);

            let (bounded_kernel_start, bounded_kernel_end) =
                kernel_bounds(mu, sigma, num_timesteps);

            for j in bounded_kernel_start..=bounded_kernel_end {
                costs[j] = (-((j as f64 - mu).powi(2)) / (2.0 * sigma.powi(2))).exp()
                    / (sigma * (2.0 * std::f64::consts::PI).sqrt());
            }

            let cost_sum: f64 = (bounded_kernel_start..=bounded_kernel_end)
                .map(|i| costs[i])
                .sum();
            let scale = window_cost / cost_sum;
            for j in bounded_kernel_start..=bounded_kernel_end {
                costs[j] *= scale;
            }
        }

        Some((costs, validity))
    })
}

/// The `+/- 4*sigma` kernel bounds for one invalid window, clamped to
/// `0..num_timesteps`. Split out from
/// [`cost_function_from_state_validator`]'s body so the truncation sequence
/// documented in this module's doc ("`long` truncation in the
/// Gaussian-smoothing kernel bounds") can be pinned directly against
/// `start`/`end`'s reachable extremes -- see this function's tests -- without
/// needing to allocate a `DMatrix` large enough to reach them.
///
/// See this module's doc, "`long` truncation in the Gaussian-smoothing
/// kernel bounds": truncate `sigma` to an integer before multiplying by 4,
/// then truncate the offset `mu` result to an integer again.
fn kernel_bounds(mu: f64, sigma: f64, num_timesteps: usize) -> (usize, usize) {
    let sigma_offset = (sigma as i64) * 4;
    let kernel_start = (mu - sigma_offset as f64) as i64;
    let kernel_end = (mu + sigma_offset as f64) as i64;
    let bounded_kernel_start = kernel_start.max(0) as usize;
    let bounded_kernel_end = kernel_end.min(num_timesteps as i64 - 1).max(0) as usize;
    (bounded_kernel_start, bounded_kernel_end)
}

/// `costs::getCollisionCostFunction(planning_scene, group, collision_penalty)`
/// (`cost_functions.hpp:199-216`). Builds a [`StateValidatorFn`] that writes
/// `positions` into `scene`'s current state
/// ([`crate::stomp::conversion_functions::set_positions`], upstream's
/// `setJointPositions`) and reports `collision_penalty` if
/// [`PlanningScene::is_state_colliding`] then finds a collision restricted to
/// `group`, `0.0` otherwise. See this module's doc for why `scene` is a
/// shared `RefCell` and why `group` (not `None`) is required rather than
/// optional here.
///
/// # Errors
///
/// [`cspace_core::error::Error`] if any of `group`'s active joints has more than
/// one variable -- see [`crate::stomp::conversion_functions::set_positions`]'s own
/// "Single-variable-joint precondition", checked once here rather than on
/// every call.
pub fn get_collision_cost_function<'a, 'm, E>(
    scene: &'a RefCell<&'a mut PlanningScene<'m>>,
    env: &'a E,
    group: &'a JointModelGroup,
    collision_penalty: f64,
) -> Result<CostFn<'a>>
where
    E: for<'s> CollisionEnv<Posed<'s, 'm>>,
{
    for name in group.active_joint_names() {
        let variable_count = scene
            .borrow()
            .current_state()
            .model()
            .joint_model(name)?
            .variable_count();
        require_single_variable(name, variable_count)?;
    }
    let group_name = group.name().to_string();
    let validator: StateValidatorFn<'a> = Box::new(move |positions: &DVector<f64>| {
        let mut scene = scene.borrow_mut();
        set_positions(positions, group, scene.current_state_mut())
            .expect("checked in get_collision_cost_function's own constructor");
        let request = CollisionRequest {
            group_name: Some(group_name.clone()),
            ..Default::default()
        };
        if scene.is_state_colliding(env, &request) {
            collision_penalty
        } else {
            0.0
        }
    });
    Ok(cost_function_from_state_validator(
        validator,
        COL_CHECK_DISTANCE,
    ))
}

/// `costs::getConstraintsCostFunction(planning_scene, group, constraints,
/// cost_scale)` (`cost_functions.hpp:230-250`). Builds a
/// [`StateValidatorFn`] that writes `positions` into `scene`'s current state,
/// updates its transforms, and scores it against `constraints`. See this
/// module's doc, "Deviation: `&KinematicConstraintSet`", for why
/// `constraints` is already built rather than a ROS message here.
///
/// # Deviation: gated on `satisfied`, not raw `distance * cost_scale`
///
/// Upstream returns `constraints.decide(state).distance * cost_scale`
/// unconditionally (`cost_functions.hpp:246`) -- it never reads
/// `.satisfied`. [`cspace_planning::constraints::ConstraintEvaluationResult::distance`] is "distance from
/// the exact nominal target" for every constraint kind (see e.g.
/// `JointConstraint::decide`, `kinematic_constraint.cpp:325`:
/// `constraint_weight_ * fabs(dif)`, nonzero for any `dif != 0` regardless
/// of tolerance), not "amount by which tolerance is exceeded" -- so a state
/// comfortably inside tolerance but not landing on the exact target still
/// gets `distance > 0.0`, and
/// [`cost_function_from_state_validator`]'s own `costs(timestep) > 0.0` test
/// then flags it invalid. For a joint constraint reached through STOMP's
/// noise-sampled optimization, which essentially never lands on a target
/// value bit-exactly, this makes every waypoint of every trajectory read as
/// invalid: measured on this tree (2026-08-08, 125 problems x 2, one
/// obstacle config, same seed), a joint constraint supplied as a path
/// constraint went from 0/125 solved to 119/125 solved when the identical
/// constraint was checked post-hoc instead of driving the optimizer.
///
/// This is not a porting-introduced regression -- `JointConstraint::decide`,
/// `getConstraintsCostFunction` and `getCostFunctionFromStateValidator` are
/// all ported faithfully (verified line-for-line against
/// `kinematic_constraint.cpp:289-326` and `cost_functions.hpp:76-130,
/// 230-250` this round) -- but it is also not a combination upstream has
/// ever exercised: `StompPlanningContext::createStompTask`
/// (`stomp_moveit_planning_context.cpp:155-167`) only routes
/// `request.path_constraints` into `getConstraintsCostFunction`;
/// `request.goal_constraints` -- where a `JointConstraint` normally lives
/// (`MoveGroupInterface::setJointValueTarget`) -- is consumed only once, by
/// `ConstraintSamplerManager::selectSampler` to sample a single goal state
/// (`stomp_moveit_planning_context.cpp:228-234`), and never re-enters cost
/// evaluation. No code in the upstream tree at
/// `e017c91ee12984393a28ba246075c65f69cde3bf` ever builds a
/// `path_constraints` containing a `JointConstraint`; `moveit_planners/
/// stomp/test/test_cost_functions.cpp` never calls
/// `getConstraintsCostFunction` at all, with any constraint type; the one
/// historical STOMP+constraints demo (`stomp_moveit_example.cpp`, added and
/// deleted the same day in commit `d566983e5`/`e00b89e28`) used an
/// end-effector orientation constraint, was an uncontested interactive RViz
/// loop, and no longer exists. So this is a latent upstream defect this
/// port is the first to actually measure, not a known, accepted tradeoff.
///
/// The fix: `if satisfied { 0.0 } else { distance * cost_scale }`. Uniform
/// across every [`cspace_planning::constraints::Constraint`] variant (all four `decide()` implementations
/// share the same "nonzero distance even when satisfied" shape --
/// `PositionConstraint::decide`'s `finish` and
/// `OrientationConstraint::decide` both compute a magnitude from the exact
/// target/center, gated separately by their own tolerance/region test), so
/// no per-variant special case is needed. This makes "satisfied implies
/// zero cost" hold by construction at this one call site rather than by an
/// implicit assumption the struct's own (now corrected) doc comment used to
/// state as a fact. The discontinuity this introduces at the tolerance
/// boundary is not new to this file: [`get_collision_cost_function`]
/// already scores every state as a flat `collision_penalty` or `0.0`, a
/// step function of the same shape: this cost function's post-boundary
/// slope still follows `distance`, unlike collision's flat penalty, so it
/// keeps a real gradient signal pointing back toward the target.
///
/// # Errors
///
/// Same precondition and reason as [`get_collision_cost_function`]'s own
/// "Errors" section.
pub fn get_constraints_cost_function<'a, 'm>(
    scene: &'a RefCell<&'a mut PlanningScene<'m>>,
    group: &'a JointModelGroup,
    constraints: &'a KinematicConstraintSet,
    cost_scale: f64,
) -> Result<CostFn<'a>> {
    for name in group.active_joint_names() {
        let variable_count = scene
            .borrow()
            .current_state()
            .model()
            .joint_model(name)?
            .variable_count();
        require_single_variable(name, variable_count)?;
    }
    let validator: StateValidatorFn<'a> = Box::new(move |positions: &DVector<f64>| {
        let mut scene = scene.borrow_mut();
        set_positions(positions, group, scene.current_state_mut())
            .expect("checked in get_constraints_cost_function's own constructor");
        let posed = scene.current_state_mut().update();
        let result = constraints.decide(&posed);
        if result.satisfied {
            0.0
        } else {
            result.distance * cost_scale
        }
    });
    Ok(cost_function_from_state_validator(
        validator,
        CONSTRAINT_CHECK_DISTANCE,
    ))
}

/// `costs::sum(cost_functions)`: evaluates every function in
/// `cost_functions` against the same `values` and adds their costs,
/// AND-ing their validity flags.
///
/// # Deviation: a failing constituent fails the sum
///
/// Upstream ignores each `cost_fn`'s own `bool` return entirely (`bool
/// valid = true; cost_fn(values, costs, valid); ...`) -- even a `false`
/// return still has its (unspecified) `costs`/`valid` out-parameters added
/// in. This port's [`CostFn`] returns `Option`, which carries no such
/// partial payload for a `None` case, so there is nothing to add in; `sum`
/// instead returns `None` itself if any constituent does. Unreachable in
/// practice through this crate's own [`cost_function_from_state_validator`]
/// (always returns `Some`, matching its own upstream lambda always
/// returning `true`) -- this only changes behavior for a hypothetical
/// caller-supplied [`CostFn`] that returns `None`, which upstream itself
/// has no real call site of either.
pub fn sum<'a>(mut cost_functions: Vec<CostFn<'a>>) -> CostFn<'a> {
    Box::new(move |values: &DMatrix<f64>| {
        let mut overall_costs = DVector::zeros(values.ncols());
        let mut overall_validity = true;
        for cost_fn in cost_functions.iter_mut() {
            let (costs, valid) = cost_fn(values)?;
            overall_costs += &costs;
            overall_validity = overall_validity && valid;
        }
        Some((overall_costs, overall_validity))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere_validator(center: f64, radius: f64, penalty: f64) -> StateValidatorFn<'static> {
        Box::new(move |state: &DVector<f64>| {
            if (state[0] - center).abs() < radius {
                penalty
            } else {
                0.0
            }
        })
    }

    // `kernel_bounds`: pinning the truncation sequence and its reachable
    // input extremes (PORTING-PLAN.md §172). `start`/`end` are `usize`
    // timestep indices with `0 <= start <= end < num_timesteps =
    // values.ncols()`, so `mu`/`sigma` are bounded by whatever `num_timesteps`
    // a real `DMatrix<f64>` can actually report.

    #[test]
    fn kernel_bounds_truncates_sigma_before_multiplying_by_four() {
        // window_size = 3 (odd) => sigma = max(1.0, 1.5) = 1.5, a
        // non-integer. Truncate-first (upstream, and this port):
        // `(1.5 as i64) * 4 == 4`. Multiply-first (what a naive all-f64
        // rewrite would compute instead): `(1.5 * 4.0) as i64 == 6`. These
        // must differ for this test to actually pin the cast order rather
        // than merely re-deriving whatever the code currently does.
        let sigma = 1.5_f64;
        let truncate_first = (sigma as i64) * 4;
        let multiply_first = (sigma * 4.0) as i64;
        assert_ne!(
            truncate_first, multiply_first,
            "test fixture no longer exercises a truncation-sensitive sigma"
        );

        let mu = 6.0; // (start + end) / 2 for start=5, end=7
        let num_timesteps = 100;
        let (bounded_start, bounded_end) = kernel_bounds(mu, sigma, num_timesteps);
        assert_eq!((bounded_start, bounded_end), (2, 10), "mu - 4, mu + 4");
    }

    #[test]
    fn kernel_bounds_clamps_a_kernel_that_spans_the_whole_trajectory() {
        // The window itself is the entire (small) trajectory: kernel_start
        // and kernel_end both fall outside [0, num_timesteps - 1] and must
        // be clamped, not merely computed.
        let (bounded_start, bounded_end) = kernel_bounds(2.0, 1.0, 5);
        assert_eq!((bounded_start, bounded_end), (0, 4));
    }

    #[test]
    fn kernel_bounds_at_the_dmatrix_allocation_ceiling_does_not_overflow() {
        // The reachable extreme of `num_timesteps` is not `usize::MAX`: it
        // is bounded by however many `f64` columns a `DMatrix` can actually
        // hold before nalgebra's backing `Vec<f64>` refuses to allocate,
        // which is `isize::MAX / size_of::<f64>()` elements (Rust caps a
        // single allocation's size at `isize::MAX` bytes). This is the most
        // generous possible reachable value (a real trajectory also carries
        // >= 1 joint per column, i.e. nrows > 1, which only lowers the
        // ceiling further) -- if `sigma_offset`'s `* 4` doesn't overflow
        // `i64` even here, it cannot overflow for any input this crate's
        // public API can actually construct.
        let max_reachable_num_timesteps = isize::MAX as usize / std::mem::size_of::<f64>();

        // Verify the ceiling is actually below the overflow threshold,
        // rather than asserting it in prose: this is the "measurement" the
        // audit claim is pinned on. If a future change moves the
        // multiplier (`* 4`) or narrows the target further, this bound
        // shifts and this assertion is what would catch a claim that no
        // longer holds.
        let sigma_at_ceiling = 0.5 * max_reachable_num_timesteps as f64;
        assert!(
            (sigma_at_ceiling as i64).checked_mul(4).is_some(),
            "the reachable DMatrix ceiling ({max_reachable_num_timesteps}) is large enough to \
             overflow i64 in sigma_offset -- the 'no reachable divergence' claim in \
             doc/claim-audit/cspace-planners-stomp.md is false, this is a defect"
        );

        // A window spanning the full (hypothetical, maximally reachable)
        // trajectory: start = 0, end = num_timesteps - 1. This calls the
        // exact production arithmetic (plain `*`, not `wrapping_mul`/
        // `checked_mul`) at that ceiling; under nextest's default dev
        // profile (`overflow-checks = true`), an actual overflow here
        // panics this test rather than silently wrapping.
        let start = 0usize;
        let end = max_reachable_num_timesteps - 1;
        let window_size = (end - start) as f64 + 1.0;
        let sigma = (1.0_f64).max(0.5 * window_size);
        let mu = 0.5 * (start as f64 + end as f64);

        let (bounded_start, bounded_end) = kernel_bounds(mu, sigma, max_reachable_num_timesteps);
        assert!(bounded_start <= bounded_end);
        assert_eq!(bounded_end, max_reachable_num_timesteps - 1);
    }

    #[test]
    fn a_fully_valid_trajectory_has_zero_cost_and_is_valid() {
        let mut cost_fn =
            cost_function_from_state_validator(sphere_validator(100.0, 0.1, 1.0), 0.0);
        let values = DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 2.0, 3.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(5));
    }

    #[test]
    fn an_invalid_waypoint_is_penalized_and_marks_the_trajectory_invalid() {
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.5, 3.0), 0.0);
        let values = DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 2.0, 3.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(!validity);
        // Cost is spread by the Gaussian smoothing step, not left as a bare
        // spike at timestep 2 -- but the total across the window must equal
        // the original penalty, and every entry outside the +/-4*sigma
        // kernel (all of it, here: window_size=1 => sigma=1.0 => kernel
        // radius 4) stays at its pre-smoothing value.
        assert!((costs.sum() - 3.0).abs() < 1e-9);
        assert!(costs[2] > 0.0);
    }

    #[test]
    fn interpolation_catches_an_invalid_state_between_two_valid_waypoints() {
        // Waypoints at x=0 and x=4 both valid; the obstacle sits at x=2,
        // reachable only by interpolating between them.
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.4, 5.0), 0.5);
        let values = DMatrix::from_row_slice(1, 2, &[0.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(!validity);
        assert!(costs.sum() > 0.0);
    }

    #[test]
    fn zero_interpolation_step_size_skips_interpolation() {
        // Same obstacle-between-waypoints setup as above, but
        // interpolation_step_size = 0.0 must disable the interpolation walk
        // entirely -- both waypoints score 0.0 at the obstacle's midpoint.
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.4, 5.0), 0.0);
        let values = DMatrix::from_row_slice(1, 2, &[0.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(2));
    }

    #[test]
    fn zero_timesteps_does_not_panic() {
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(0.0, 1.0, 1.0), 0.1);
        let values = DMatrix::<f64>::zeros(1, 0);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs.len(), 0);
    }

    #[test]
    fn sum_adds_costs_and_ands_validity() {
        let a: CostFn<'static> = Box::new(|values: &DMatrix<f64>| {
            Some((DVector::from_element(values.ncols(), 1.0), true))
        });
        let b: CostFn<'static> = Box::new(|values: &DMatrix<f64>| {
            Some((DVector::from_element(values.ncols(), 2.0), false))
        });
        let mut summed = sum(vec![a, b]);
        let values = DMatrix::zeros(1, 3);
        let (costs, validity) = summed(&values).unwrap();
        assert_eq!(costs, DVector::from_element(3, 3.0));
        assert!(!validity);
    }

    #[test]
    fn sum_of_zero_cost_functions_is_zero_and_valid() {
        let mut summed: CostFn<'static> = sum(Vec::new());
        let values = DMatrix::zeros(1, 4);
        let (costs, validity) = summed(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(4));
    }

    // Assertion-discrimination sweep (round 2): `sum`'s closure body
    // (lines 383-392) has exactly one place a `None` can originate, the
    // `cost_fn(values)?` propagation on line 387 -- rg for `None|\?` over
    // that body: one hit. `summed(&values).is_none()` is therefore
    // single-branch; there is no sibling None site to confuse it with.
    #[test]
    fn sum_propagates_a_failing_constituent_as_none() {
        let failing: CostFn<'static> = Box::new(|_values: &DMatrix<f64>| None);
        let mut summed = sum(vec![failing]);
        let values = DMatrix::zeros(1, 2);
        assert!(summed(&values).is_none());
    }

    /// Port of `test_cost_functions.cpp`'s `testGetCostFunctionAllValidStates`
    /// -- upstream's exact `TIMESTEPS=100`, `VARIABLES=6` literals, not this
    /// module's usual small synthetic fixtures.
    ///
    /// # Margin/reachability audit (round: margin audit): no bound, no shadowing
    ///
    /// Both assertions are exact equality checks (`validity` a bool,
    /// `costs.sum() == 0.0` deterministic for an always-valid validator) --
    /// no inequality-against-a-threshold bound to measure a margin for.
    /// Only two closely-coupled assertions, not a distinctive claim behind
    /// generic ones as in `upstream_test_get_cost_function_invalid_states`
    /// (see that test's own doc): no reordering/split needed.
    #[test]
    fn upstream_test_get_cost_function_all_valid_states() {
        const TIMESTEPS: usize = 100;
        const VARIABLES: usize = 6;
        let always_valid: StateValidatorFn<'static> = Box::new(|_state: &DVector<f64>| 0.0);
        let mut cost_fn = cost_function_from_state_validator(always_valid, 0.1);
        let values = DMatrix::from_fn(VARIABLES, TIMESTEPS, |_row, col| {
            col as f64 / (TIMESTEPS - 1) as f64
        });
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs.sum(), 0.0);
    }

    /// Port of `test_cost_functions.cpp`'s `testGetCostFunctionInvalidStates`
    /// -- upstream's exact `INVALID_TIMESTEPS` set (18 entries), `PENALTY`,
    /// and the `0.681 * PENALTY` two-sigma concentration claim. Interpolation
    /// is disabled (`0.0`), matching upstream, so `state_validator_fn` is
    /// called exactly once per timestep in timestep order -- the counter
    /// stands in for upstream's `timestep_counter`.
    ///
    /// # Split into four cases, not one: `assert!` short-circuits
    ///
    /// Upstream's test body makes five independent claims in sequence
    /// (validity, call count, `costs.maxCoeff() <= PENALTY`,
    /// `costs.minCoeff() >= 0.0`, the sum-preservation identity, and the
    /// `0.681` concentration bound). As one `#[test]` with five `assert!`s,
    /// `assert!` stops at the first failure -- a mutation that breaks an
    /// early claim (e.g. `validity`) never exercises whether a later claim
    /// (e.g. the `0.681` concentration bound) still holds, because the test
    /// never reaches that line. A round-24 bite-check confirmed this
    /// concretely: perturbing `sigma`/`mu` in `kernel_bounds` reddened this
    /// test on `costs.sum()` or `costs.max() <= PENALTY`, never once on the
    /// `0.681` line, even though that line is the test's most distinctive
    /// claim (see "Measured margins" below -- it is also the loosest bound,
    /// so it is the claim least likely to be caught incidentally by the
    /// others). Splitting each claim into its own `#[test]` (sharing
    /// [`run_upstream_invalid_states_scenario`] for setup) makes every claim
    /// independently reachable: a mutation that breaks only the
    /// concentration bound now reddens exactly one test, regardless of
    /// whether it also breaks an earlier claim.
    ///
    /// # Measured margins (round: margin audit)
    ///
    /// `costs.max() <= PENALTY` (upstream's own `EXPECT_LE`): measured
    /// `costs.max() = 0.9444393030890911` against `PENALTY = 1.0` --
    /// margin ~1.06x (~6% headroom), a genuinely tight bound.
    /// `costs.min() >= 0.0`: structural, not a probabilistic margin -- a
    /// penalty-only cost function cannot produce a negative value by
    /// construction, so this is an exact-equality-at-the-floor check, not
    /// something with a "margin" to measure.
    /// `invalid_costs_sum >= 0.681 * PENALTY` (upstream's own `EXPECT_GE`):
    /// measured `invalid_costs_sum = 12.37895087036898` against a bound of
    /// `0.681` -- margin ~18.2x. Not a documentation defect in the bound
    /// itself (it is upstream's own literal, reproduced exactly, and
    /// upstream's own assertion is equally loose since upstream's `costs`
    /// sum to `PENALTY * 18` too) -- the defect was this doc comment
    /// reading as though `0.681` pinned something tight, now corrected.
    fn run_upstream_invalid_states_scenario() -> (DVector<f64>, bool, usize) {
        const TIMESTEPS: usize = 100;
        const VARIABLES: usize = 6;
        let invalid_set: std::collections::HashSet<usize> =
            UPSTREAM_INVALID_TIMESTEPS.iter().copied().collect();

        let counter = std::rc::Rc::new(RefCell::new(0usize));
        let counter_captured = counter.clone();
        let validator: StateValidatorFn<'static> = Box::new(move |_state: &DVector<f64>| {
            let t = *counter_captured.borrow();
            *counter_captured.borrow_mut() += 1;
            if invalid_set.contains(&t) {
                UPSTREAM_INVALID_STATES_PENALTY
            } else {
                0.0
            }
        });
        let mut cost_fn = cost_function_from_state_validator(validator, 0.0);
        let values = DMatrix::from_fn(VARIABLES, TIMESTEPS, |_row, col| {
            col as f64 / (TIMESTEPS - 1) as f64
        });
        let (costs, validity) = cost_fn(&values).unwrap();
        (costs, validity, *counter.borrow())
    }

    const UPSTREAM_INVALID_STATES_TIMESTEPS: usize = 100;
    const UPSTREAM_INVALID_STATES_PENALTY: f64 = 1.0;
    const UPSTREAM_INVALID_TIMESTEPS: [usize; 18] = [
        0, 10, 11, 12, 25, 26, 27, 46, 63, 64, 65, 66, 67, 68, 69, 97, 98, 99,
    ];

    #[test]
    fn upstream_test_get_cost_function_invalid_states_marks_the_trajectory_invalid_and_visits_every_timestep()
     {
        let (_costs, validity, counter) = run_upstream_invalid_states_scenario();
        assert!(!validity);
        assert_eq!(counter, UPSTREAM_INVALID_STATES_TIMESTEPS);
    }

    #[test]
    fn upstream_test_get_cost_function_invalid_states_costs_stay_within_penalty_bounds() {
        let (costs, _validity, _counter) = run_upstream_invalid_states_scenario();
        // measured costs.max() = 0.9444393030890911, ~1.06x margin -- tight.
        assert!(costs.max() <= UPSTREAM_INVALID_STATES_PENALTY);
        // structural: a penalty-only cost function cannot go negative.
        assert!(costs.min() >= 0.0);
    }

    #[test]
    fn upstream_test_get_cost_function_invalid_states_total_cost_equals_penalty_times_invalid_count()
     {
        let (costs, _validity, _counter) = run_upstream_invalid_states_scenario();
        assert!(
            (costs.sum()
                - UPSTREAM_INVALID_STATES_PENALTY * UPSTREAM_INVALID_TIMESTEPS.len() as f64)
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn upstream_test_get_cost_function_invalid_states_concentrates_at_least_0681_of_penalty_at_invalid_timesteps()
     {
        let (costs, _validity, _counter) = run_upstream_invalid_states_scenario();
        let invalid_costs_sum: f64 = UPSTREAM_INVALID_TIMESTEPS.iter().map(|&i| costs[i]).sum();
        // measured invalid_costs_sum = 12.37895087036898, ~18.2x margin
        // against 0.681 -- upstream's own bound is this loose too (see the
        // doc comment on `run_upstream_invalid_states_scenario`).
        assert!(invalid_costs_sum >= 0.681 * UPSTREAM_INVALID_STATES_PENALTY);

        // The line above is the port of upstream's literal and stays exactly
        // as upstream wrote it. It has no power: shifting this cost
        // function's Gaussian centre by 20 timesteps -- mis-centring the
        // entire kernel -- leaves it passing, because 12.379 has to fall by
        // 18x before a bound of 0.681 notices.
        //
        // Read the same number as a *fraction* and the 18x margin turns into
        // 1.0099x: `costs.sum()` is `PENALTY * 18 == 18.0` here, so the
        // invalid timesteps carry `12.37895087036898 / 18.0 ==
        // 0.6877194927982767` of the total, one percent above `0.681`. A
        // ratio landing within 1% of the literal is not plausibly a
        // coincidence at this n; upstream almost certainly meant "at least
        // 68.1% of the cost concentrates on the invalid timesteps" and wrote
        // `0.681 * PENALTY` where the total, not the per-timestep penalty,
        // was the intended scale. This assertion is the reading that has
        // power -- it is this port's own, not a second port of upstream's
        // line, and its margin is measured rather than inherited. The
        // mis-centring mutation above drives it to 0.5745452302710986 and it
        // fails, which is the whole difference between the two readings.
        let concentration = invalid_costs_sum / costs.sum();
        assert!(
            concentration >= 0.681,
            "invalid timesteps carry {concentration} of the total cost, below the measured 0.6877194927982767"
        );
    }
}

#[cfg(test)]
mod planning_scene_tests {
    use std::fs;

    use cspace_collision::{LinkPaddingScale, ParryCollisionEnv, World};
    use cspace_core::model::{JointModelGroup, MeshSearchPaths, RobotModel};
    use cspace_core::srdf::SrdfModel;
    use cspace_planning::constraints::{Constraint, JointConstraint, KinematicConstraintSet};
    use cspace_planning::scene::PlanningScene;

    use super::*;

    fn fixture_mesh_search_paths() -> MeshSearchPaths {
        let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
        MeshSearchPaths::new([(
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        )])
    }

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// All-zero joint positions: panda's real, mesh-loaded collision
    /// geometry self-collides at this pose -- see
    /// `cspace_planners::sbp::planning_scene_validity`'s `ready_state` doc
    /// comment (oracle-verified `panda_collision.json`,
    /// `joint_values: {} => self_collision: true`).
    fn colliding_positions(group: &JointModelGroup) -> DVector<f64> {
        DVector::zeros(group.active_joint_names().len())
    }

    /// panda.srdf's own `"ready"` named `<group_state>` for `panda_arm` --
    /// moveit's own designed non-self-colliding demo pose, so it is
    /// collision-free without needing any world object at all.
    fn free_positions() -> DVector<f64> {
        DVector::from_vec(vec![0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    }

    /// [`free_positions`] with `panda_joint1` overridden -- the joint
    /// [`JointConstraint`] fixtures below constrain.
    fn positions_with_joint1(value: f64) -> DVector<f64> {
        let mut positions = free_positions();
        positions[0] = value;
        positions
    }

    fn single_waypoint(positions: &DVector<f64>) -> DMatrix<f64> {
        DMatrix::from_column_slice(positions.len(), 1, positions.as_slice())
    }

    fn empty_env() -> ParryCollisionEnv {
        ParryCollisionEnv::new(World::new(), LinkPaddingScale::default())
    }

    #[test]
    fn collision_free_trajectory_has_zero_cost_and_is_valid() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let env = empty_env();

        let mut cost_fn = get_collision_cost_function(&cell, &env, group, 10.0).unwrap();
        let free = free_positions();
        let values = DMatrix::from_columns(&[free.clone(), free.clone(), free]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity, "\"ready\" must not self-collide");
        assert_eq!(costs, DVector::zeros(3));
    }

    #[test]
    fn one_colliding_waypoint_among_free_ones_marks_the_trajectory_invalid() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let env = empty_env();

        let mut cost_fn = get_collision_cost_function(&cell, &env, group, 10.0).unwrap();
        let free = free_positions();
        let colliding = colliding_positions(group);
        let values = DMatrix::from_columns(&[free.clone(), colliding, free]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(
            !validity,
            "a self-colliding middle waypoint must invalidate the whole trajectory"
        );
        assert!(costs.sum() > 0.0);
    }

    #[test]
    fn every_waypoint_colliding_sums_to_num_timesteps_times_the_penalty() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let env = empty_env();

        let collision_penalty = 10.0;
        let mut cost_fn =
            get_collision_cost_function(&cell, &env, group, collision_penalty).unwrap();
        let colliding = colliding_positions(group);
        let num_timesteps = 4;
        let values = DMatrix::from_columns(&vec![colliding; num_timesteps]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(!validity);
        // The whole trajectory is one contiguous invalid window; Gaussian
        // smoothing redistributes but always rescales back to the window's
        // original total cost -- see `cost_function_from_state_validator`'s
        // own doc, "Preserved: later windows can overwrite...".
        assert!((costs.sum() - num_timesteps as f64 * collision_penalty).abs() < 1e-9);
    }

    #[test]
    fn collision_penalty_boundary_is_exact_with_no_interpolation_to_blend_it() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let collision_penalty = 7.5;

        let mut colliding_scene = PlanningScene::new(&model, &srdf);
        let colliding_cell = RefCell::new(&mut colliding_scene);
        let env = empty_env();
        let mut colliding_cost_fn =
            get_collision_cost_function(&colliding_cell, &env, group, collision_penalty).unwrap();
        let (colliding_costs, colliding_validity) =
            colliding_cost_fn(&single_waypoint(&colliding_positions(group))).unwrap();
        assert!(!colliding_validity);
        assert_eq!(
            colliding_costs[0], collision_penalty,
            "a single-waypoint trajectory has no next waypoint to interpolate toward, so the \
             raw penalty must pass through the Gaussian-smoothing step unchanged (a one-point \
             window's kernel is that one point, rescaled to itself)"
        );

        let mut free_scene = PlanningScene::new(&model, &srdf);
        let free_cell = RefCell::new(&mut free_scene);
        let mut free_cost_fn =
            get_collision_cost_function(&free_cell, &env, group, collision_penalty).unwrap();
        let (free_costs, free_validity) =
            free_cost_fn(&single_waypoint(&free_positions())).unwrap();
        assert!(free_validity);
        assert_eq!(free_costs[0], 0.0);
    }

    #[test]
    fn constraint_satisfied_exactly_at_the_target_value_has_zero_cost() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let mut cost_fn = get_constraints_cost_function(&cell, group, &set, 1.0).unwrap();
        let (costs, validity) = cost_fn(&single_waypoint(&positions_with_joint1(0.0))).unwrap();
        assert!(validity);
        assert_eq!(costs[0], 0.0);
    }

    #[test]
    fn constraint_violated_beyond_tolerance_has_the_exact_scaled_distance_as_cost() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let cost_scale = 2.0;
        let mut cost_fn = get_constraints_cost_function(&cell, group, &set, cost_scale).unwrap();
        let (costs, validity) = cost_fn(&single_waypoint(&positions_with_joint1(1.0))).unwrap();
        assert!(!validity);
        // weight=1.0, |dif|=|1.0-0.0|=1.0, distance=1.0, cost=distance*cost_scale.
        assert_eq!(costs[0], 2.0);
    }

    // Regression for the "satisfied-but-nonzero-distance" defect this
    // module's doc on `get_constraints_cost_function` records: before the
    // `if result.satisfied` gate this test failed both assertions (cost ==
    // weight * dif.abs() * cost_scale != 0.0, validity == false) for a
    // position squarely inside tolerance, not just at its exact edge --
    // this is the position class STOMP's noise-sampled optimizer actually
    // produces, so it is the case that decided 0/125 vs 119/125 on the
    // measured population, not the target-exact case
    // `constraint_satisfied_exactly_at_the_target_value_has_zero_cost`
    // above already covered.
    #[test]
    fn constraint_cost_is_exactly_zero_anywhere_inside_tolerance_not_only_at_the_target() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let tolerance = 0.1;
        let constraint =
            JointConstraint::new(&model, "panda_joint1", 0.0, tolerance, tolerance, 1.0).unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let cost_scale = 3.0;
        let mut cost_fn = get_constraints_cost_function(&cell, group, &set, cost_scale).unwrap();
        // Halfway into the tolerance window, not at the target and not at
        // the edge: `JointConstraint::decide` reports `distance =
        // weight * 0.05 = 0.05 != 0.0` here, but `satisfied == true`.
        let (costs, validity) =
            cost_fn(&single_waypoint(&positions_with_joint1(tolerance / 2.0))).unwrap();
        assert_eq!(
            costs[0], 0.0,
            "a state inside tolerance must cost exactly 0.0, not the raw distance to the exact \
             target"
        );
        assert!(
            validity,
            "a state inside tolerance must read as valid, matching JointConstraint::decide's \
             own satisfied flag"
        );
    }

    #[test]
    fn constraint_cost_at_the_exact_tolerance_edge_is_still_satisfied_and_zero() {
        let (model, srdf) = load_panda();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let tolerance = 0.1;
        let constraint =
            JointConstraint::new(&model, "panda_joint1", 0.0, tolerance, tolerance, 1.0).unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(constraint));

        let cost_scale = 3.0;
        let mut cost_fn = get_constraints_cost_function(&cell, group, &set, cost_scale).unwrap();
        // Exactly at the tolerance edge: `JointConstraint::decide`'s own
        // `dif <= tolerance_above + 2*EPS` reports this position as
        // satisfied (the boundary is inclusive), so the cost function must
        // now read it as free, not as `distance * cost_scale`.
        let (costs, validity) =
            cost_fn(&single_waypoint(&positions_with_joint1(tolerance))).unwrap();
        assert_eq!(
            costs[0], 0.0,
            "the tolerance edge is inclusive (satisfied == true there), so cost must be 0.0, \
             not weight * tolerance * cost_scale"
        );
        assert!(
            validity,
            "JointConstraint::decide reports this state as satisfied at the exact edge, and the \
             cost function must now agree"
        );
    }
}
