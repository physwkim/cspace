// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//     (KDLKinematicsPlugin::CartToJnt, lines 417-497;
//      KDLKinematicsPlugin::clipToJointLimits, lines 499-522;
//      KDLKinematicsPlugin::searchPositionIK, lines 303-415)

use std::f64::consts::PI;

use nalgebra::DVector;
use rand::{Rng, RngExt};

use moveit_geometry::Isometry3;
use moveit_state::RobotState;

use crate::chain::ChainInfo;
use crate::params::SolverParams;
use crate::registry::SolveOptions;
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

/// `KDLKinematicsPlugin::getRandomConfiguration(jnt_array)`: each active
/// joint's value drawn uniformly across its own full `[min, max]` range,
/// independent of any seed.
fn random_configuration(chain: &ChainInfo, rng: &mut impl Rng) -> Vec<f64> {
    chain
        .active_min
        .iter()
        .zip(&chain.active_max)
        .map(|(&min, &max)| rng.random_range(min..=max))
        .collect()
}

/// `KDLKinematicsPlugin::getRandomConfiguration(seed_state, consistency_limits,
/// jnt_array)`, i.e. `JointModel::getVariableRandomPositionsNearBy`. A
/// non-continuous active joint's value is drawn uniformly from
/// `[max(min, near - limit), min(max, near + limit)]`; a continuous one
/// (`RevoluteJointModel::getVariableRandomPositionsNearBy`'s own
/// `continuous_` branch, `revolute_joint_model.cpp:122-136`) is drawn
/// unclamped from `[near - limit, near + limit]` and then wrapped into
/// `(-pi, pi]`, matching `RevoluteJoint::enforce_position_bounds`'s wrap
/// formula rather than clamping it to an edge. See
/// [`SolveOptions::consistency_limits`]'s doc comment for why
/// `consistency_limits` here is reduced-space rather than upstream's
/// full-space parameter.
fn near_by_configuration(
    chain: &ChainInfo,
    near: &[f64],
    consistency_limits: &[f64],
    rng: &mut impl Rng,
) -> Vec<f64> {
    chain
        .active_min
        .iter()
        .zip(&chain.active_max)
        .zip(&chain.active_continuous)
        .zip(near)
        .zip(consistency_limits)
        .map(|((((&min, &max), &continuous), &near), &limit)| {
            if continuous {
                let mut value = rng.random_range((near - limit)..=(near + limit));
                if value <= -PI || value > PI {
                    value %= 2.0 * PI;
                    if value <= -PI {
                        value += 2.0 * PI;
                    } else if value > PI {
                        value -= 2.0 * PI;
                    }
                }
                value
            } else {
                rng.random_range(min.max(near - limit)..=max.min(near + limit))
            }
        })
        .collect()
}

/// `KDLKinematicsPlugin::checkConsistency`, reduced-space — see
/// [`SolveOptions::consistency_limits`]'s doc comment.
fn satisfies_consistency(seed: &[f64], solution: &[f64], consistency_limits: &[f64]) -> bool {
    seed.iter()
        .zip(solution)
        .zip(consistency_limits)
        .all(|((&s, &sol), &limit)| (s - sol).abs() <= limit)
}

/// `KDLKinematicsPlugin::searchPositionIK`'s single-pose case in its
/// fullest form: one [`cart_to_jnt`] attempt from `seed`, then up to
/// `params.max_restarts` further attempts from a reseed — uniformly
/// near `seed` within `options.consistency_limits` if given, else
/// uniformly across each active joint's own full bounds
/// (`getRandomConfiguration`). A numerically-converged attempt that
/// `options` rejects (either gate) is treated exactly like a
/// non-converging one: the loop retries rather than returning `None`
/// outright.
///
/// # Deviation from upstream
///
/// No wall-clock `timeout` — see [`SolverParams::max_restarts`].
///
/// # Panics
///
/// If `seed` or `options.consistency_limits` does not have exactly one entry
/// per active joint. Both are validated here, at the one point every solver
/// funnels through, rather than in each `solve_with_options`: a per-solver
/// guard has to be remembered once per solver *and* once per new
/// [`SolveOptions`] field, which is how `consistency_limits` arrived with no
/// length check at all while `seed` had two identical ones.
pub(crate) fn search_position_ik(
    ctx: &SolveContext,
    state: &mut RobotState,
    seed: &[f64],
    target: &Isometry3,
    pinv: &impl Fn(f64, f64) -> f64,
    rng: &mut impl Rng,
    options: &mut SolveOptions,
) -> Option<Vec<f64>> {
    assert_eq!(
        seed.len(),
        ctx.chain.reduced_dimension(),
        "seed must have one entry per active joint"
    );
    // Upstream rejects a mis-sized `consistency_limits` outright
    // (`kdl_kinematics_plugin.cpp:329`, `NO_IK_SOLUTION`). Without this,
    // `satisfies_consistency`'s `zip` would silently stop at the shorter of
    // the two and accept a solution on the strength of a partial check.
    if let Some(limits) = options.consistency_limits {
        assert_eq!(
            limits.len(),
            ctx.chain.reduced_dimension(),
            "consistency_limits must have one entry per active joint"
        );
    }
    for attempt in 0..=ctx.params.max_restarts {
        let attempt_seed = if attempt == 0 {
            seed.to_vec()
        } else {
            match options.consistency_limits {
                Some(limits) => near_by_configuration(ctx.chain, seed, limits, rng),
                None => random_configuration(ctx.chain, rng),
            }
        };

        let Some(solution) = cart_to_jnt(ctx, state, &attempt_seed, target, pinv, rng) else {
            continue;
        };

        if let Some(limits) = options.consistency_limits {
            if !satisfies_consistency(seed, &solution, limits) {
                continue;
            }
        }
        if let Some(callback) = options.solution_callback.as_deref_mut() {
            if !callback(&solution) {
                continue;
            }
        }
        return Some(solution);
    }
    None
}

#[cfg(test)]
mod tests {
    use std::fs;

    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;

    use super::*;
    use crate::params::SolverParams;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
            file_name
        )
    }

    fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
        let urdf_path = fixture_path(urdf_file);
        let srdf_path = fixture_path(srdf_file);
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    fn truncated_pinv(threshold: f64) -> impl Fn(f64, f64) -> f64 {
        move |s, smax| if s > threshold * smax { 1.0 / s } else { 0.0 }
    }

    /// This test module's own FK: apply `config` (active-joint order) to a
    /// fresh state and read the tip pose in the chain's own base frame —
    /// the same computation [`cart_to_jnt`]'s iteration loop performs, so a
    /// target built this way is exactly reachable from that config.
    fn fk_of(chain: &ChainInfo, model: &RobotModel, config: &[f64]) -> Isometry3 {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, &value) in chain.active_joint_names.iter().zip(config) {
            state.set_variable_position(name, value).unwrap();
        }
        let posed = state.update();
        chain.root_pose_world(&posed) * posed.global_link_transform_at(chain.tip_link_index)
    }

    struct Fixture {
        model: RobotModel,
        chain: ChainInfo,
        params: SolverParams,
        joint_weights: Vec<f64>,
    }

    impl Fixture {
        fn panda_arm() -> Self {
            let model = build_model("panda.urdf", "panda.srdf");
            let chain = ChainInfo::build(&model, "panda_arm").expect("real panda_arm chain");
            let params = SolverParams::default();
            let joint_weights = chain.resolve_joint_weights(&params);
            Self {
                model,
                chain,
                params,
                joint_weights,
            }
        }

        /// `right_arm` has two `continuous` revolute joints
        /// (`r_forearm_roll_joint`, `r_wrist_roll_joint`) that `panda_arm`'s
        /// bounded-only chain cannot exercise -- see
        /// `near_by_configuration_wraps_a_continuous_joint_past_pi_instead_of_clamping`.
        fn pr2_right_arm() -> Self {
            let model = build_model("pr2.urdf", "pr2.srdf");
            let chain = ChainInfo::build(&model, "right_arm").expect("real right_arm chain");
            let params = SolverParams::default();
            let joint_weights = chain.resolve_joint_weights(&params);
            Self {
                model,
                chain,
                params,
                joint_weights,
            }
        }

        fn ctx(&self) -> SolveContext<'_> {
            SolveContext {
                chain: &self.chain,
                params: &self.params,
                joint_weights: &self.joint_weights,
            }
        }

        /// The chain's midpoint config, and that same config with joint `0`
        /// bumped by `fraction` of its own half-range — always in-bounds
        /// since the bump starts from the midpoint.
        fn midpoint_and_bumped(&self, fraction: f64) -> (Vec<f64>, Vec<f64>) {
            let mid: Vec<f64> = self
                .chain
                .active_min
                .iter()
                .zip(&self.chain.active_max)
                .map(|(&lo, &hi)| (lo + hi) / 2.0)
                .collect();
            let mut bumped = mid.clone();
            bumped[0] += fraction * (self.chain.active_max[0] - self.chain.active_min[0]) / 2.0;
            (mid, bumped)
        }
    }

    /// `satisfies_consistency`'s own defining boundary: exactly at the
    /// limit is accepted (upstream's `>` violation check, not `>=`), one ULP
    /// over it is rejected. `<=` on raw `f64`s has no hidden proportional
    /// tolerance the way `assert_relative_eq!` without `max_relative` does
    /// (see `PORTING-PLAN.md` §79) -- bisected at the literal bit level
    /// (`f64::from_bits`, not an arbitrary small delta) to confirm that
    /// directly: one ULP under `at_limit` is still accepted, one ULP over
    /// is rejected, with nothing in between.
    #[test]
    fn satisfies_consistency_accepts_at_the_limit_and_rejects_just_over_it() {
        let seed = [1.0, 2.0];
        let at_limit = [1.5, 2.5];
        let limits = [0.5, 0.5];
        assert!(satisfies_consistency(&seed, &at_limit, &limits));

        let one_ulp_under = [f64::from_bits(at_limit[0].to_bits() - 1), 2.5];
        assert!(satisfies_consistency(&seed, &one_ulp_under, &limits));

        let one_ulp_over = [f64::from_bits(at_limit[0].to_bits() + 1), 2.5];
        assert!(!satisfies_consistency(&seed, &one_ulp_over, &limits));
    }

    /// `search_position_ik` with a default (`None`, `None`) [`SolveOptions`]
    /// must behave exactly as it did before this crate had the concept of
    /// options at all — same seed, same rng draws, same accept criterion.
    #[test]
    fn default_options_accept_the_first_convergent_attempt() {
        let fixture = Fixture::panda_arm();
        let ctx = fixture.ctx();
        let (seed, _) = fixture.midpoint_and_bumped(0.3);
        let target = fk_of(&fixture.chain, &fixture.model, &seed);

        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let pinv = truncated_pinv(fixture.params.svd_threshold);
        let mut options = SolveOptions::default();

        let solution = search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
        assert!(
            solution.is_some(),
            "seed is already the exact target config, so attempt 0 must converge at the seed"
        );
    }

    /// The other boundary of `consistency_limits`: its *length*. One entry
    /// short is what `satisfies_consistency`'s `zip` would have swallowed,
    /// accepting a solution whose last active joint was never checked.
    #[test]
    #[should_panic(expected = "consistency_limits must have one entry per active joint")]
    fn consistency_limits_one_entry_short_panics() {
        let fixture = Fixture::panda_arm();
        let ctx = fixture.ctx();
        let (seed, _) = fixture.midpoint_and_bumped(0.3);
        let target = fk_of(&fixture.chain, &fixture.model, &seed);
        let pinv = truncated_pinv(fixture.params.svd_threshold);

        let short = vec![10.0; seed.len() - 1];
        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions {
            consistency_limits: Some(&short),
            solution_callback: None,
        };
        search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
    }

    /// The `seed`-length guard still fires now that it lives here rather
    /// than in each solver's `solve_with_options`.
    #[test]
    #[should_panic(expected = "seed must have one entry per active joint")]
    fn seed_one_entry_short_panics() {
        let fixture = Fixture::panda_arm();
        let ctx = fixture.ctx();
        let (seed, _) = fixture.midpoint_and_bumped(0.3);
        let target = fk_of(&fixture.chain, &fixture.model, &seed);
        let pinv = truncated_pinv(fixture.params.svd_threshold);

        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions::default();
        search_position_ik(
            &ctx,
            &mut state,
            &seed[..seed.len() - 1],
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
    }

    /// The consistency-limit gate at its own boundary: a converged solution
    /// that lands far from `seed` (by construction — the target is built
    /// from a config `0.3` of a joint's half-range away from `seed`) is
    /// accepted under a generous limit and rejected under a tight one, with
    /// `max_restarts = 0` so there is only ever the one attempt to judge.
    #[test]
    fn consistency_limit_gates_a_convergent_solution_by_distance_from_seed() {
        let mut fixture = Fixture::panda_arm();
        fixture.params.max_restarts = 0;
        let ctx = fixture.ctx();
        let (seed, bumped) = fixture.midpoint_and_bumped(0.3);
        let target = fk_of(&fixture.chain, &fixture.model, &bumped);
        let pinv = truncated_pinv(fixture.params.svd_threshold);

        let generous = vec![10.0; seed.len()];
        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions {
            consistency_limits: Some(&generous),
            solution_callback: None,
        };
        let solution = search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
        assert!(
            solution.is_some(),
            "a generous consistency limit must not reject a solution that had to move to converge"
        );

        let tight = vec![0.01; seed.len()];
        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions {
            consistency_limits: Some(&tight),
            solution_callback: None,
        };
        let solution = search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
        assert!(
            solution.is_none(),
            "a 0.01 rad limit must reject a solution that had to move by roughly 0.3 of a joint's half-range, with no restart left to retry"
        );
    }

    /// The solution-callback gate at its own boundary: an always-accepting
    /// callback must not change the outcome an option-free call already
    /// gets; an always-rejecting callback must turn that same convergent
    /// attempt into `None` (exhausting the sole `max_restarts = 0` attempt)
    /// rather than ever being skipped.
    #[test]
    fn solution_callback_gates_acceptance_independent_of_convergence() {
        let mut fixture = Fixture::panda_arm();
        fixture.params.max_restarts = 0;
        let ctx = fixture.ctx();
        let (seed, _) = fixture.midpoint_and_bumped(0.3);
        let target = fk_of(&fixture.chain, &fixture.model, &seed);
        let pinv = truncated_pinv(fixture.params.svd_threshold);

        let mut accept_calls = 0usize;
        let mut accept_all = |_: &[f64]| {
            accept_calls += 1;
            true
        };
        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions {
            consistency_limits: None,
            solution_callback: Some(&mut accept_all),
        };
        let solution = search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
        assert!(
            solution.is_some(),
            "an always-accepting callback must not reject a convergent solution"
        );
        assert_eq!(
            accept_calls, 1,
            "callback must be invoked exactly once for the one convergent attempt"
        );

        let mut reject_calls = 0usize;
        let mut reject_all = |_: &[f64]| {
            reject_calls += 1;
            false
        };
        let mut state = RobotState::new(&fixture.model);
        state.set_to_default_values();
        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut options = SolveOptions {
            consistency_limits: None,
            solution_callback: Some(&mut reject_all),
        };
        let solution = search_position_ik(
            &ctx,
            &mut state,
            &seed,
            &target,
            &pinv,
            &mut rng,
            &mut options,
        );
        assert!(
            solution.is_none(),
            "an always-rejecting callback must turn even a convergent attempt into no solution"
        );
        assert_eq!(
            reject_calls, 1,
            "callback must still be invoked once before the sole max_restarts=0 attempt is exhausted"
        );
    }

    /// The continuous-joint boundary `near_by_configuration`'s non-continuous
    /// branch cannot reach: sampling near `PI - 0.1` with a `0.5` limit spans
    /// past `PI`, and a continuous joint must wrap around to the negative
    /// side there (`RevoluteJointModel::getVariableRandomPositionsNearBy`'s
    /// `continuous_` branch, `revolute_joint_model.cpp:126-129`) rather than
    /// saturate at `PI` the way a bounded joint would.
    #[test]
    fn near_by_configuration_wraps_a_continuous_joint_past_pi_instead_of_clamping() {
        let fixture = Fixture::pr2_right_arm();
        let continuous_index = fixture
            .chain
            .active_joint_names
            .iter()
            .position(|name| name == "r_wrist_roll_joint")
            .expect("r_wrist_roll_joint is active in right_arm");
        assert!(
            fixture.chain.active_continuous[continuous_index],
            "r_wrist_roll_joint must be recorded as continuous"
        );

        let mut near: Vec<f64> = fixture
            .chain
            .active_min
            .iter()
            .zip(&fixture.chain.active_max)
            .map(|(&lo, &hi)| (lo + hi) / 2.0)
            .collect();
        near[continuous_index] = PI - 0.1;
        let limit = 0.5;
        let mut limits = vec![10.0; near.len()];
        limits[continuous_index] = limit;

        let mut rng = ChaCha8Rng::seed_from_u64(0);
        let mut wrapped_negative = false;
        for _ in 0..200 {
            let sample = near_by_configuration(&fixture.chain, &near, &limits, &mut rng);
            let value = sample[continuous_index];
            assert!(
                value > -PI && value <= PI,
                "wrapped value {value} must land in (-pi, pi]"
            );
            if value < 0.0 {
                wrapped_negative = true;
            }
        }
        assert!(
            wrapped_negative,
            "sampling near pi - 0.1 with a 0.5 limit must sometimes wrap past pi to the negative side, not saturate at pi"
        );
    }
}
