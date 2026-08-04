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
