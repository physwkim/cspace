// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The bridge from a [`moveit_constraints::ConstraintSampler`] to
//! [`rrt_connect::ConstrainedStateSampler`]: [`GroupConstraintSampler`]
//! writes each attempt into a scratch [`RobotState`] and reads the result
//! back out through a [`JointModelGroupSpace`], mirroring
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`]'s own
//! sample-to-`RobotState`-and-back shape rather than inventing a second one.

use moveit_constraints::ConstraintSampler;
use moveit_state::RobotState;
use rand::Rng;

use crate::compound::CompoundValue;
use crate::joint_model_group_space::JointModelGroupSpace;
use crate::rrt_connect::ConstrainedStateSampler;

/// Adapts a [`moveit_constraints::ConstraintSampler`] (which samples into a
/// [`RobotState`]) to [`rrt_connect::ConstrainedStateSampler`] (which
/// samples a [`JointModelGroupSpace`] state).
///
/// # Why `template` is cloned per attempt, not reused
///
/// [`ConstraintSampler::sample`] writes into whatever [`RobotState`] it is
/// given, including variables outside the sampled group (it starts from
/// the state passed in). Reusing one scratch `RobotState` across attempts
/// would let a failed attempt's partial write leak into the next attempt's
/// input — [`try_sample`](Self::try_sample) instead clones `template` fresh
/// each attempt, so every attempt starts from the same known-good state and
/// a failure discards exactly what it wrote, nothing more.
///
/// # Known gap: `template` is a fixed seed, not re-anchored to tree locality
///
/// Every attempt starts IK from the *same* `template`, regardless of where
/// in the search tree this particular sample is being drawn for. For a
/// **goal** region this is fine — [`crate::goal_sampler::sample_goal`] only
/// needs one Cartesian-compliant joint-space point, reachable from anywhere.
/// For a **path**-constrained corridor sampled during tree growth
/// ([`crate::registry::RrtConnectContext::solve`]'s `path_constraints`
/// branch), it is not: an IK solution seeded from a fixed, possibly-distant
/// `template` can land anywhere in the corridor consistent with that seed,
/// independent of the tree node the new sample is meant to extend from —
/// [`crate::validity::DiscreteMotionValidator`]'s resolution-checked linear
/// interpolation between that tree node and the IK "teleport" then routinely
/// leaves the Cartesian region before reaching the destination, failing
/// validation.
///
/// Measured (round 24, `PORTING-PLAN.md` §163.3's follow-up): four scenarios
/// were tried against a wired vs. unwired `path_constraints` region on
/// `panda_arm` at matched `step_size`/iteration budget — a far-apart
/// self-motion position+orientation pair, an orientation-only region with a
/// free approach axis, a region built around an already IK-reachable nearby
/// goal, and a `step_size`/budget sweep looking for any crossover point. In
/// no scenario did wiring a solver reliably improve `solve()`'s success
/// rate for the region; in the tightest, most goal-region-analogous
/// scenario, wired performed *worse* than unwired (0/5 vs. 5/5 successful
/// solves at matched `step_size` and iteration budget) — the IK "teleports"
/// this gap describes are more disruptive to tree growth than plain uniform
/// sampling is. This is why
/// `crate::registry::tests::path_constraints_solver_wiring_matches_the_call_site`
/// tests `resolve_constraint_sampler` directly rather than `solve()`
/// end-to-end: an end-to-end test would measure this gap, not the wiring
/// change it was written to verify.
///
/// **Disposition:** re-anchoring `template` to tree locality (e.g. seeding
/// IK from the tree node being extended, not a fixed pre-search state) is
/// scheduled as its own round, not rejected or unknown — it is deferred
/// because it also touches goal sampling
/// ([`crate::goal_sampler::sample_goal`]) and joint-constraint sampling
/// (`moveit_constraints::JointConstraintSampler`), both of which currently
/// rely on the same "fixed template, group-local draw" shape this gap
/// describes, and reworking one without the others risks introducing a
/// second, differently-shaped inconsistency (see `PORTING-PLAN.md`'s
/// §153.1 convention). This note expires when that round lands and
/// `template` is re-anchored — at which point the measurement above should
/// be re-run, not assumed to still hold.
pub struct GroupConstraintSampler<'a, 'm> {
    space: &'a JointModelGroupSpace,
    sampler: &'a dyn ConstraintSampler,
    template: RobotState<'m>,
}

impl<'a, 'm> GroupConstraintSampler<'a, 'm> {
    /// `template` seeds every sampling attempt: [`ConstraintSampler::sample`]
    /// draws every variable of *its own* [`moveit_model::JointModelGroup`]
    /// afresh (constrained ones from their tolerance window, unconstrained
    /// ones from the joint's own bounds), so only a variable entirely
    /// outside that group comes from `template` unchanged in the state
    /// [`try_sample`](Self::try_sample) writes into — and even that only
    /// matters to a caller reading `state` directly, since
    /// [`try_sample`](Self::try_sample)'s own return value is scoped to the
    /// group either way (see [`JointModelGroupSpace::read_robot_state`]).
    pub fn new(
        space: &'a JointModelGroupSpace,
        sampler: &'a dyn ConstraintSampler,
        template: RobotState<'m>,
    ) -> Self {
        Self {
            space,
            sampler,
            template,
        }
    }
}

impl ConstrainedStateSampler<JointModelGroupSpace> for GroupConstraintSampler<'_, '_> {
    fn try_sample(&self, rng: &mut dyn Rng) -> Option<Vec<CompoundValue>> {
        let mut state = self.template.clone();
        self.sampler
            .sample(&mut state, rng)
            .then(|| self.space.read_robot_state(&state))
    }
}

#[cfg(test)]
mod tests {
    use moveit_constraints::{JointConstraint, JointConstraintSampler};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    use super::*;

    fn load_panda() -> RobotModel {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
        let urdf_xml = std::fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
        let no_meshes: MeshSearchPaths =
            MeshSearchPaths::new(std::iter::empty::<(String, String)>());
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &no_meshes)
            .expect("fixture model must build")
    }

    #[test]
    fn every_sample_satisfies_the_wrapped_constraint() {
        let model = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let template = RobotState::new(&model);

        // +/-0.05 around 0.0 is far narrower than panda_joint1's own
        // [-2.8973, 2.8973] bound, so a plain unconstrained sample_uniform
        // draw landing inside it by chance is vanishingly unlikely -- this
        // is the same "narrow window inside a wide bound" shape
        // `registry.rs`'s own wiring test uses to prove the sampler is
        // load-bearing rather than merely invoked.
        let constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.05, 0.05, 1.0)
            .expect("valid joint constraint");
        let owned = vec![constraint];
        let joint_sampler = JointConstraintSampler::new(&model, "panda_arm", &owned)
            .expect("panda_joint1 constraint must configure against panda_arm");

        let bridge = GroupConstraintSampler::new(&space, &joint_sampler, template);
        let mut rng = ChaCha8Rng::seed_from_u64(1);

        for _ in 0..50 {
            let state = bridge
                .try_sample(&mut rng)
                .expect("JointConstraintSampler::sample always succeeds");
            let mut robot_state = RobotState::new(&model);
            space.write_robot_state(&state, &mut robot_state);
            let value = robot_state.variable_position("panda_joint1").unwrap();
            assert!(
                (-0.05..=0.05).contains(&value),
                "panda_joint1 = {value} escaped the +/-0.05 constraint window"
            );
        }
    }
}
