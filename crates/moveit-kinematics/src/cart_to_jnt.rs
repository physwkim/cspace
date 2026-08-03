// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2013, Sachin Chitta, Willow Garage
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//     (KDLKinematicsPlugin::CartToJnt, lines 417-497;
//      KDLKinematicsPlugin::clipToJointLimits, lines 499-522;
//      KDLKinematicsPlugin::searchPositionIK, lines 303-415)

use nalgebra::DVector;
use rand::{Rng, RngExt};

use moveit_geometry::Isometry3;
use moveit_state::RobotState;

use crate::chain::ChainInfo;
use crate::params::SolverParams;
use crate::velocity::solve_velocity;

/// Bundles the three per-solver-instance, call-invariant arguments
/// [`cart_to_jnt`] and [`search_position_ik`] both need, so a caller
/// passes one reference instead of three. Structural fix for
/// clippy's `too_many_arguments`, not a suppression of it: before this
/// bundling, `seed: &[f64]` and `joint_weights: &[f64]` were two
/// same-typed bare parameters sitting next to each other in the
/// argument list — exactly the "interchangeable arguments can be
/// silently swapped at a call site" hazard the lint exists to catch.
/// Moving `joint_weights` into this struct removes that hazard, not
/// just the argument count.
#[derive(Clone, Copy)]
pub(crate) struct SolveContext<'a> {
    pub(crate) chain: &'a ChainInfo,
    pub(crate) params: &'a SolverParams,
    pub(crate) joint_weights: &'a [f64],
}

/// `diff(f, p_in)`: the twist that would carry `current` to `target` in
/// unit time. Rows 0-2 linear, rows 3-5 angular (rotation vector taking
/// `current`'s orientation to `target`'s) — see
/// [`crate::chain::ChainInfo::full_jacobian`]'s doc comment for why this
/// crate uses that row convention throughout.
fn error_twist(current: &Isometry3, target: &Isometry3) -> DVector<f64> {
    let position_error = target.translation.vector - current.translation.vector;
    let orientation_error = (target.rotation * current.rotation.inverse()).scaled_axis();
    DVector::from_fn(6, |i, _| {
        if i < 3 {
            position_error[i]
        } else {
            orientation_error[i - 3]
        }
    })
}

/// `clipToJointLimits`: per full-space DOF, clamp `delta_q[i]` so
/// `q_full[i] + delta_q[i]` cannot leave `[chain.min[i], chain.max[i]]`, and
/// down-weight the clipped DOF's *master* column
/// (`extra_joint_weights[chain.map_index[i]] = 0.01`) — read by the next
/// iteration's [`solve_velocity`] call. `extra_joint_weights` is reset to
/// all-`1.0` on every call, matching upstream's own `weighting.setOnes()`
/// as this function's first statement: the down-weighting reflects only the
/// clip that *just happened*, not an accumulated history.
fn clip_to_joint_limits(
    chain: &ChainInfo,
    q_full: &[f64],
    delta_q: &mut [f64],
    extra_joint_weights: &mut [f64],
) {
    extra_joint_weights.fill(1.0);
    for i in 0..chain.dimension() {
        let delta_max = chain.max[i] - q_full[i];
        let delta_min = chain.min[i] - q_full[i];
        if delta_q[i] > delta_max {
            delta_q[i] = delta_max;
        } else if delta_q[i] < delta_min {
            delta_q[i] = delta_min;
        } else {
            continue;
        }
        extra_joint_weights[chain.map_index[i]] = 0.01;
    }
}

/// Write `q_full` into `state`'s whole-model positions buffer at each
/// full-space DOF's [`crate::chain::ChainInfo::variable_index`], leaving
/// every other model variable at whatever `state` already held, then commit
/// through [`RobotState::set_variable_positions`] (bulk,
/// no-mimic-propagation — see [`ChainInfo`]'s doc comment for why this loop
/// needs that, not the auto-propagating per-variable setter).
fn apply_full(chain: &ChainInfo, state: &mut RobotState, q_full: &[f64]) {
    let mut positions = state.positions().to_vec();
    for i in 0..chain.dimension() {
        positions[chain.variable_index[i]] = q_full[i];
    }
    state.set_variable_positions(&positions);
}

/// `KDLKinematicsPlugin::CartToJnt`: one Newton run from `seed` toward
/// `target` (both/either in the chain's own base-link frame — see
/// [`ChainInfo::root_pose_world`]), up to `params.max_solver_iterations`.
/// `joint_weights` is `getJointWeights`'s reduced-space output (this
/// solver's per-joint weight configuration, without the per-iteration
/// clip-driven down-weighting [`clip_to_joint_limits`] layers on top).
///
/// # Deviation from upstream: `state`/`q_full` split, not one raw buffer
///
/// See [`ChainInfo`]'s doc comment: `q_full` is this loop's own
/// full-space buffer (upstream's `KDL::JntArray q_out`, transiently
/// mimic-inconsistent after a clip exactly as upstream's is — the
/// per-DOF-independent clip in [`clip_to_joint_limits`] does not preserve
/// `q_full[mimic] == multiplier * q_full[master] + offset`, and this is
/// faithful, not a bug: upstream's raw buffer has the identical property).
/// `state` is re-synchronised to `q_full` at the top of every iteration
/// ([`apply_full`]) because both the FK read and the next
/// [`ChainInfo::full_jacobian`] call need a fresh [`moveit_state::Posed`]
/// built from those exact positions.
pub(crate) fn cart_to_jnt(
    ctx: &SolveContext,
    state: &mut RobotState,
    seed: &[f64],
    target: &Isometry3,
    pinv: &impl Fn(f64, f64) -> f64,
    rng: &mut impl Rng,
) -> Option<Vec<f64>> {
    let SolveContext {
        chain,
        params,
        joint_weights,
    } = *ctx;
    for (name, &value) in chain.active_joint_names.iter().zip(seed) {
        state
            .set_variable_position(name, value)
            .expect("chain's own active joint is a real model variable");
    }

    let mut q_full: Vec<f64> = chain
        .joint_names
        .iter()
        .map(|name| {
            state
                .variable_position(name)
                .expect("chain's own joint is a real model variable")
        })
        .collect();

    let position_only = params.orientation_weight() == 0.0;
    let cartesian_weights = DVector::from_fn(6, |i, _| {
        if i < 3 {
            1.0
        } else {
            params.orientation_weight()
        }
    });

    let mut extra_joint_weights = vec![1.0_f64; chain.reduced_dimension()];
    let mut step_size = 1.0_f64;
    let mut last_delta_twist_norm = f64::MAX;
    let mut q_backup = q_full.clone();
    let mut delta_q = vec![0.0_f64; chain.dimension()];

    for _ in 0..params.max_solver_iterations {
        apply_full(chain, state, &q_full);
        let posed = state.update();

        let root_pose_world = chain.root_pose_world(&posed);
        let current = root_pose_world * posed.global_link_transform_at(chain.tip_link_index);
        let twist = error_twist(&current, target);

        let position_error_norm = twist.rows(0, 3).norm();
        let orientation_error_norm = if position_only {
            0.0
        } else {
            twist.rows(3, 3).norm()
        };
        let delta_twist_norm = position_error_norm.max(orientation_error_norm);

        if delta_twist_norm <= params.epsilon {
            return Some(
                chain
                    .active_joint_names
                    .iter()
                    .map(|name| {
                        posed
                            .variable_position(name)
                            .expect("chain's own active joint is a real model variable")
                    })
                    .collect(),
            );
        }

        if delta_twist_norm >= last_delta_twist_norm {
            // Close to a singularity: back off rather than trusting the
            // last velocity solve's step.
            let old_step_size = step_size;
            step_size *= (0.2_f64).min(last_delta_twist_norm / delta_twist_norm);
            let scale = step_size / old_step_size;
            for d in &mut delta_q {
                *d *= scale;
            }
            q_full.copy_from_slice(&q_backup);
        } else {
            q_backup.copy_from_slice(&q_full);
            step_size = 1.0;
            last_delta_twist_norm = delta_twist_norm;

            let jacobian_full = chain.full_jacobian(&posed);
            let combined_weights: Vec<f64> = joint_weights
                .iter()
                .zip(&extra_joint_weights)
                .map(|(a, b)| a * b)
                .collect();
            let solved = solve_velocity(
                chain,
                &jacobian_full,
                &twist,
                &cartesian_weights,
                &combined_weights,
                pinv,
            );
            delta_q.copy_from_slice(solved.as_slice());
        }

        clip_to_joint_limits(chain, &q_full, &mut delta_q, &mut extra_joint_weights);

        let delta_q_norm: f64 = delta_q.iter().map(|v| v.abs()).sum();
        if delta_q_norm < params.epsilon {
            // Stuck in a singularity.
            if step_size < params.epsilon {
                break;
            }
            last_delta_twist_norm = f64::MAX;
            let wiggle_scale = (0.1_f64).min(delta_twist_norm);
            for d in &mut delta_q {
                *d = rng.random_range(-1.0..=1.0) * wiggle_scale;
            }
            clip_to_joint_limits(chain, &q_full, &mut delta_q, &mut extra_joint_weights);
            extra_joint_weights.fill(1.0);
        }

        for i in 0..chain.dimension() {
            q_full[i] += delta_q[i];
        }
    }
    None
}

/// `KDLKinematicsPlugin::searchPositionIK`'s single-pose, no-callback,
/// no-consistency-limits case: one [`cart_to_jnt`] attempt from `seed`,
/// then up to `params.max_restarts` further attempts from a uniformly
/// random reseed within each active joint's own bounds
/// (`getRandomConfiguration`).
///
/// # Deviation from upstream
///
/// No `consistency_limits`/`IKCallbackFn` — see this crate's "do not port
/// the ROS surface" doc comment; no wall-clock `timeout` — see
/// [`SolverParams::max_restarts`].
pub(crate) fn search_position_ik(
    ctx: &SolveContext,
    state: &mut RobotState,
    seed: &[f64],
    target: &Isometry3,
    pinv: &impl Fn(f64, f64) -> f64,
    rng: &mut impl Rng,
) -> Option<Vec<f64>> {
    if let Some(solution) = cart_to_jnt(ctx, state, seed, target, pinv, rng) {
        return Some(solution);
    }
    for _ in 0..ctx.params.max_restarts {
        let random_seed: Vec<f64> = ctx
            .chain
            .active_min
            .iter()
            .zip(&ctx.chain.active_max)
            .map(|(&min, &max)| rng.random_range(min..=max))
            .collect();
        if let Some(solution) = cart_to_jnt(ctx, state, &random_seed, target, pinv, rng) {
            return Some(solution);
        }
    }
    None
}
