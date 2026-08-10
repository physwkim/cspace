// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/ompl/ompl_interface/include/moveit/ompl_interface/detail/constrained_goal_sampler.hpp
//   moveit_planners/ompl/ompl_interface/src/detail/constrained_goal_sampler.cpp
//   (ConstrainedGoalSampler::sampleUsingConstraintSampler)

//! Resolves one [`cspace_planning::PlanningRequest::goal_constraints`] set to one
//! concrete [`JointModelGroupSpace`] state, mirroring
//! `ConstrainedGoalSampler::sampleUsingConstraintSampler`
//! (`constrained_goal_sampler.cpp:96-170`).
//!
//! # Why one state, not a lazily-grown region
//!
//! Upstream's `ConstrainedGoalSampler` is an `ob::GoalLazySamples`: a
//! background thread keeps calling `sampleUsingConstraintSampler` while the
//! search runs, growing a *set* of up to `getMaximumGoalSamples()` (`10`,
//! `planning_context_manager.cpp:258`) accepted goal states that OMPL's
//! tree can pick among. [`crate::rrt_connect::rrt_connect`] instead takes
//! one fixed `goal: S::State` and roots a single goal tree on it (see that
//! function's own signature) — there is no multi-goal region for a second,
//! third, ... accepted sample to go into. [`sample_goal`] therefore returns
//! the *first* accepted sample and stops, rather than collecting up to ten;
//! `getMaximumGoalSamples()` has no port here. See `registry.rs`'s
//! `max_goal_samples_` disposition comment (next to
//! `DEFAULT_MAX_GOAL_SAMPLING_ATTEMPTS`) for the full citation, including
//! the consuming line (`detail/constrained_goal_sampler.cpp:106`) this
//! module doc summarizes. This is a deliberate scope limitation this
//! port's single-rooted goal tree imposes, not an oversight — see the
//! round 21 report for the "does a concrete goal state losslessly become a
//! `JointConstraint` set" determination this interacts with (if it does,
//! most callers never need a constraint region's multi-solution
//! generality in the first place).
//!
//! # What else is not ported
//!
//! - The 80%-invalid diagnostic warning (`invalid_sampled_constraints_`,
//!   `constrained_goal_sampler.cpp:147-154`) and `verbose_display_`
//!   throttling (`:117-124`) — logging-only, `RCLCPP_WARN`/`RCLCPP_DEBUG`
//!   side effects with no bearing on which state [`sample_goal`] accepts.
//! - `hasSolution()` early-exit (`:110`) — upstream's background sampling
//!   thread checks whether the tree search already found a solution and
//!   stops growing the goal region early. [`sample_goal`] runs once, to
//!   completion, before [`crate::rrt_connect::rrt_connect`] starts
//!   searching at all, so there is no concurrent search progress to poll.
//!
//! # Deviation: one combined validity order, not two per-branch orders
//!
//! Upstream's constrained branch checks `kinematic_constraint_set_->decide()`
//! before `checkStateValidity()` (`:140-144`); its uniform-fallback branch
//! checks `isValid()` before `decide()` (`:161-165`) — the opposite order.
//! Both orders exist only to gate the diagnostic counter above on the
//! constrained branch specifically; neither changes which candidate is
//! ultimately accepted (`A && B` does not depend on evaluation order for a
//! side-effect-free `A`/`B`). [`sample_goal`] checks `checker.is_valid`
//! before `goal_constraints.decide` in both branches, uniformly.

use cspace_constraints::{ConstraintSampler, KinematicConstraintSet};
use cspace_state::RobotState;
use rand::Rng;

use crate::compound::CompoundValue;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::space::StateSpace;
use crate::validity::StateValidityChecker;

/// Draws up to `max_goal_sampling_attempts` candidate states — from
/// `constrained_sampler` if `Some` (mirroring `:126-157`'s constrained
/// branch, one `ConstraintSampler::sample` call per attempt, no per-attempt
/// fallback to uniform sampling unlike
/// [`crate::rrt_connect::Sampler::sample_uniform`]'s path-constraint
/// sampling), or from [`crate::space::StateSpace::sample_uniform`] if `None`
/// (`:158-167`'s uniform branch) — and returns the first one that satisfies
/// both `checker` (collision plus [`cspace_planning::PlanningRequest::path_constraints`],
/// mirroring upstream's `si_->getStateValidityChecker()`) and
/// `goal_constraints` (mirroring `kinematic_constraint_set_`). `None` if no
/// attempt succeeds within the budget, mirroring `:102-103`'s
/// `attempts_so_far >= max_attempts` early return.
pub fn sample_goal<C>(
    space: &JointModelGroupSpace,
    checker: &C,
    goal_constraints: &KinematicConstraintSet,
    template: &RobotState<'_>,
    constrained_sampler: Option<&dyn ConstraintSampler>,
    rng: &mut dyn Rng,
    max_goal_sampling_attempts: u32,
) -> Option<Vec<CompoundValue>>
where
    C: StateValidityChecker<JointModelGroupSpace>,
{
    // `state` is cloned from `template` once, then reused (mutated in
    // place) across every attempt below -- matching upstream
    // `ConstrainedGoalSampler::work_state_`, a member initialised once and
    // never reset between `sampleUsingConstraintSampler` calls
    // (`constrained_goal_sampler.cpp:66,124-147`). An IK-backed
    // `constrained_sampler` seeds attempt 0 from `state`'s own current
    // group values (`IKConstraintSampler::sampleHelper`'s `use_as_seed`),
    // so resetting to `template` every attempt -- this function's own
    // design before this fix -- threw that warm start away on every draw;
    // see `crate::constrained_sampler::GroupConstraintSampler`'s doc
    // comment for the full measurement this shares its cause with.
    let mut state = template.clone();
    for _ in 0..max_goal_sampling_attempts {
        let sampled = match constrained_sampler {
            Some(sampler) => sampler.sample(&mut state, rng),
            None => {
                let candidate = space.sample_uniform(rng);
                space.write_robot_state(&candidate, &mut state);
                true
            }
        };
        if !sampled {
            continue;
        }

        let candidate = space.read_robot_state(&state);
        if !checker.is_valid(&candidate) {
            continue;
        }
        if goal_constraints.decide(&state.update()).satisfied {
            return Some(candidate);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::fs;

    use cspace_collision::ParryCollisionEnv;
    use cspace_constraints::{Constraint, JointConstraint, select_default_sampler};
    use cspace_model::{JointModelGroup, MeshSearchPaths, RobotModel};
    use cspace_scene::PlanningScene;
    use cspace_srdf::SrdfModel;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use super::*;
    use crate::planning_scene_validity::PlanningSceneValidityChecker;

    fn load_panda() -> (RobotModel, SrdfModel) {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("fixture model must build");
        (model, srdf)
    }

    /// A [`ConstraintSampler`] that records `panda_joint1`'s incoming value
    /// on every [`ConstraintSampler::sample`] call and writes back a
    /// deterministically different value -- the same recorder
    /// `constrained_sampler::tests::RecordingSampler` uses, duplicated here
    /// rather than shared across a private test boundary between the two
    /// modules.
    struct RecordingSampler {
        group: JointModelGroup,
        seen: RefCell<Vec<f64>>,
    }

    impl ConstraintSampler for RecordingSampler {
        fn joint_model_group(&self) -> &JointModelGroup {
            &self.group
        }

        fn frame_dependency(&self) -> &[String] {
            &[]
        }

        fn sample(&self, state: &mut RobotState<'_>, _rng: &mut dyn Rng) -> bool {
            let incoming = state.variable_position("panda_joint1").unwrap();
            self.seen.borrow_mut().push(incoming);
            state
                .set_variable_position("panda_joint1", incoming + 0.01)
                .unwrap();
            true
        }
    }

    /// Proves [`sample_goal`]'s `state` is upstream's `work_state_`, not
    /// `template` reset per attempt -- the goal-sampling half of the same
    /// fix `constrained_sampler::tests::try_sample_carries_the_previous_draws_result_forward_as_the_next_seed`
    /// proves for path sampling. A checker that always rejects forces
    /// [`sample_goal`] to spend its whole budget, so all three attempts'
    /// incoming `panda_joint1` values are recorded; they must be
    /// `[0.0, 0.01, 0.02]` -- each attempt's own previous output -- not
    /// `[0.0, 0.0, 0.0]`, which is what resetting to `template` every
    /// attempt (this function's behaviour before the fix this test guards)
    /// would have produced.
    #[test]
    fn sample_goal_carries_the_previous_draws_result_forward_as_the_next_seed() {
        let (model, _srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut template = RobotState::new(&model);
        template.set_variable_position("panda_joint1", 0.0).unwrap();

        let recorder = RecordingSampler {
            group: model.joint_model_group("panda_arm").unwrap().clone(),
            seen: RefCell::new(Vec::new()),
        };
        let goal_constraints = KinematicConstraintSet::new();
        let never_valid = |_: &Vec<CompoundValue>| false;

        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let result = sample_goal(
            &space,
            &never_valid,
            &goal_constraints,
            &template,
            Some(&recorder),
            &mut rng,
            3,
        );
        assert!(
            result.is_none(),
            "a checker that always rejects must exhaust sample_goal's whole budget"
        );

        assert_eq!(
            *recorder.seen.borrow(),
            vec![0.0, 0.01, 0.02],
            "each attempt's incoming panda_joint1 value must be the previous attempt's own \
             output (upstream's work_state_ semantics), not a fixed template value every time"
        );
    }

    /// Proves [`sample_goal`]'s constrained branch is load-bearing, not
    /// merely invoked: `panda_joint1` pinned to `+/-0.01` (against its own
    /// `+/-2.9671` bound, `crates/cspace-planners-sbp/tests/fixtures/panda.urdf:37`), empty world, no path
    /// constraints, budget 5 attempts.
    ///
    /// Window and budget were picked by a sweep, not derivation — same
    /// methodology as `registry.rs`'s
    /// `path_constraint_sampler_is_load_bearing_not_merely_invoked`. Swept
    /// (window, budget) pairs against 30 seeds each (`0..30`), counting how
    /// often the unwired control (`constrained_sampler: None`) failed to
    /// find any sample and how often the wired case (a real
    /// `select_default_sampler` output) succeeded:
    ///
    /// | window | budget | unwired fail | wired success |
    /// |--------|--------|---------------|----------------|
    /// | 0.05   | 50     | 14/30         | 30/30          |
    /// | 0.05   | 5      | 29/30         | 30/30          |
    /// | 0.01   | 10     | 30/30         | 30/30          |
    /// | 0.01   | 5      | 30/30         | 30/30          |
    /// | 0.005  | 20     | 30/30         | 30/30          |
    ///
    /// `(0.01, 5)` — the smallest budget in the sweep that still scored
    /// 30/30 unwired failures — is used below, and that row is what this
    /// test's own loop re-measures and asserts exactly.
    /// `window_budget_sweep_matches_the_documented_table` (below) commits
    /// the whole table as runnable code, not just the selected row —
    /// `PORTING-PLAN.md` §195: leaving the other four rows as prose citing
    /// a sweep nothing kept true is the same defect round 24's own
    /// four-scenario sweep left behind, just smaller.
    #[test]
    fn constrained_branch_is_load_bearing_not_merely_invoked() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut default_state = RobotState::new(&model);
        default_state.set_to_default_values();

        let window = 0.01;
        let budget = 5;

        for seed in 0..30u64 {
            let constraint =
                JointConstraint::new(&model, "panda_joint1", 0.0, window, window, 1.0).unwrap();
            let mut goal_constraints = KinematicConstraintSet::new();
            goal_constraints.push(Constraint::Joint(constraint));

            let mut scene = PlanningScene::new(&model, &srdf);
            let env = ParryCollisionEnv::default();
            let checker = PlanningSceneValidityChecker::new(
                &mut scene,
                &env,
                Default::default(),
                None,
                &space,
            );

            let mut unwired_rng = ChaCha8Rng::seed_from_u64(seed);
            let unwired = sample_goal(
                &space,
                &checker,
                &goal_constraints,
                &default_state,
                None,
                &mut unwired_rng,
                budget,
            );
            assert!(
                unwired.is_none(),
                "seed {seed}: unwired (plain uniform) goal sampling must NOT find a state \
                 inside the +/-{window} panda_joint1 window within {budget} attempts"
            );

            let sampler = select_default_sampler(
                &model,
                "panda_arm",
                goal_constraints.constraints(),
                None,
                vec![],
                4,
            )
            .expect("no subgroup_solvers, so select_default_sampler cannot error here")
            .expect("a single JointConstraint always yields a JointConstraintSampler");
            let mut wired_rng = ChaCha8Rng::seed_from_u64(seed);
            let wired = sample_goal(
                &space,
                &checker,
                &goal_constraints,
                &default_state,
                Some(sampler.as_ref()),
                &mut wired_rng,
                budget,
            );
            let wired = wired.unwrap_or_else(|| {
                panic!(
                    "seed {seed}: the wired (constrained) goal sampler must find a state \
                     within the same budget the unwired control above exhausted"
                )
            });

            let mut wired_state = default_state.clone();
            space.write_robot_state(&wired, &mut wired_state);
            let value = wired_state.variable_position("panda_joint1").unwrap();
            assert!(
                (-window..=window).contains(&value),
                "seed {seed}: wired sample's panda_joint1 = {value} escaped the +/-{window} \
                 window sample_goal was supposed to enforce"
            );
        }
    }

    /// Commits the full parameter-selection table from
    /// `constrained_branch_is_load_bearing_not_merely_invoked`'s own doc
    /// comment as runnable code: all five `(window, budget)` pairs, 30
    /// seeds each, tallying the same two counts that test's loop already
    /// asserts for its one selected row. `PORTING-PLAN.md` §195 -- a number
    /// cited in a doc comment is a claim, not an observation, the moment
    /// something (here, the choice of `(0.01, 5)`) depends on it, and
    /// nothing kept the other four rows true until this test existed.
    #[test]
    fn window_budget_sweep_matches_the_documented_table() {
        let (model, srdf) = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut default_state = RobotState::new(&model);
        default_state.set_to_default_values();

        // (window, budget, expected unwired-fail count, expected wired-success count)
        let cases = [
            (0.05, 50u32, 14u32, 30u32),
            (0.05, 5, 29, 30),
            (0.01, 10, 30, 30),
            (0.01, 5, 30, 30),
            (0.005, 20, 30, 30),
        ];

        for (window, budget, expected_unwired_fail, expected_wired_success) in cases {
            let mut unwired_fail = 0u32;
            let mut wired_success = 0u32;

            for seed in 0..30u64 {
                let constraint =
                    JointConstraint::new(&model, "panda_joint1", 0.0, window, window, 1.0).unwrap();
                let mut goal_constraints = KinematicConstraintSet::new();
                goal_constraints.push(Constraint::Joint(constraint));

                let mut scene = PlanningScene::new(&model, &srdf);
                let env = ParryCollisionEnv::default();
                let checker = PlanningSceneValidityChecker::new(
                    &mut scene,
                    &env,
                    Default::default(),
                    None,
                    &space,
                );

                let mut unwired_rng = ChaCha8Rng::seed_from_u64(seed);
                let unwired = sample_goal(
                    &space,
                    &checker,
                    &goal_constraints,
                    &default_state,
                    None,
                    &mut unwired_rng,
                    budget,
                );
                if unwired.is_none() {
                    unwired_fail += 1;
                }

                let sampler = select_default_sampler(
                    &model,
                    "panda_arm",
                    goal_constraints.constraints(),
                    None,
                    vec![],
                    4,
                )
                .expect("no subgroup_solvers, so select_default_sampler cannot error here")
                .expect("a single JointConstraint always yields a JointConstraintSampler");
                let mut wired_rng = ChaCha8Rng::seed_from_u64(seed);
                let wired = sample_goal(
                    &space,
                    &checker,
                    &goal_constraints,
                    &default_state,
                    Some(sampler.as_ref()),
                    &mut wired_rng,
                    budget,
                );
                if wired.is_some() {
                    wired_success += 1;
                }
            }

            assert_eq!(
                (unwired_fail, wired_success),
                (expected_unwired_fail, expected_wired_success),
                "window {window}, budget {budget}: moved off the documented table"
            );
        }
    }
}
