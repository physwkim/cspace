// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The bridge from a [`moveit_constraints::ConstraintSampler`] to
//! [`rrt_connect::ConstrainedStateSampler`]: [`GroupConstraintSampler`]
//! writes each attempt into a scratch [`RobotState`] and reads the result
//! back out through a [`JointModelGroupSpace`], mirroring
//! [`crate::planning_scene_validity::PlanningSceneValidityChecker`]'s own
//! sample-to-`RobotState`-and-back shape rather than inventing a second one.

use std::cell::RefCell;

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
/// # `working` persists across attempts: this is upstream's `work_state_`, not a bug
///
/// Through round 24, this type cloned a fixed `template` fresh on *every*
/// [`try_sample`](Self::try_sample) call. Read from upstream rather than
/// guessed: both `ompl_interface::ConstrainedSampler`
/// (`constrained_sampler.cpp`) and `ConstrainedGoalSampler`
/// (`constrained_goal_sampler.cpp`) hold their working `RobotState` —
/// `work_state_` — as a member initialised once at construction and *never
/// reset* between calls; each call's `IKConstraintSampler::sampleHelper`
/// seeds attempt 0's IK search from whatever `work_state_` already holds
/// for the group (`callIK`'s `use_as_seed` branch,
/// `default_constraint_samplers.cpp:670-673`), i.e. the *previous* accepted
/// (or even failed — see below) sample, not a fixed start state. A
/// reset-every-attempt design throws that warm start away every call. This
/// upstream reading is why `working` below is correct on its own terms; it
/// is not corroborated by round 24's own "0/5 wired vs 5/5 unwired" ad hoc
/// measurement (four scenarios, never committed as reusable code) the way
/// an earlier version of this comment implied — `PORTING-PLAN.md` §181
/// found that a committed re-measurement of this exact change does not
/// reproduce that flip; see "What `path_constraints_end_to_end_wired_vs_unwired`
/// actually shows," below, for what does.
///
/// `working` reproduces upstream's member exactly: cloned from `template`
/// once at construction, then mutated in place by every subsequent
/// [`try_sample`](Self::try_sample) call, so attempt *N*'s IK seed is
/// attempt *N-1*'s result. [`RefCell`] supplies the interior mutability
/// [`ConstrainedStateSampler::try_sample`]'s `&self` needs — matching
/// [`moveit_constraints::IkConstraintSamplerAdapter`]'s own `RefCell` use
/// for exactly the same "shared reference, mutable scratch" shape. A failed
/// attempt is *not* reverted before the next one: upstream's `callIK`
/// writes via `setJointGroupPositions` before `validate()` and never undoes
/// it either (`ik_sampler.rs`'s own `IkConstraintSampler::sample` doc
/// comment already documents this "wart" for attempts *within* one call;
/// this extends the same behaviour *across* calls, since `working` is now
/// the same persistence boundary upstream's `work_state_` is).
///
/// # What this does not change: `template` was, and remains, the source for out-of-group variables
///
/// [`ConstraintSampler::sample`] only ever writes its own group's variables
/// (`moveit_constraints`' `JointConstraintSampler`/`IkConstraintSampler`
/// both confirmed by reading their `sample` bodies) — `working`'s
/// out-of-group variables are set once, from `template`, at construction,
/// and no call ever touches them again. `template` itself is *not* kept as
/// a separate field: after seeding `working`, upstream's own reference role
/// for `template` (the value `IKConstraintSampler::sample`'s pose-sampling
/// step reads mobile reference frames against) is what upstream's own
/// second parameter, `reference_state`, is for — `moveit_constraints::ik_sampler`'s
/// module doc comment already records this port's `ConstraintSampler::sample`
/// deliberately collapses `state`/`reference_state` into one parameter, and
/// that a position or orientation constraint whose *reference* frame is a
/// mobile link inside the very group being sampled (not the case for any
/// constraint this workspace builds today, and not exercised by any test)
/// would now read that frame from `working`'s current, evolving pose rather
/// than a session-fixed one, which is a narrower deviation from upstream
/// than the seeding gap this section replaces. `PORTING-PLAN.md` §153.1:
/// this narrower gap expires if a caller ever builds a mobile-reference-frame
/// constraint whose reference link sits inside the sampled group.
///
/// # Deferred: re-anchoring the seed to tree locality
///
/// A different design was floated when round 24 first measured the
/// reset-per-attempt regression, and never built: instead of carrying
/// forward whatever the *previous draw* produced (upstream's own
/// `work_state_` behaviour, implemented above), seed each attempt from the
/// *specific tree node* `rrt_connect` is extending from, so an IK search
/// starts near where the tree currently is rather than near wherever the
/// sampler happened to leave off last. Upstream does not do this —
/// `work_state_` has no notion of "the current tree node" at all, so this
/// would be a deliberate improvement beyond upstream, not a parity
/// requirement — and `PORTING-PLAN.md` §181 found no measurement showing
/// the upstream-matching persistence design above underperforms once
/// `resolve_constraint_sampler`'s own wiring gap (`registry.rs`,
/// `PORTING-PLAN.md` §163.3) is also fixed. `PORTING-PLAN.md` §153.1: this
/// stays deferred until a measurement in the shape of
/// `crate::registry::tests::path_constraints_end_to_end_wired_vs_unwired`
/// — a wired path sampler, run at some budget or tolerance, where *this*
/// persistence design itself measurably underperforms unwired, not merely
/// fails to beat it by as much as hoped — shows the simpler persistence
/// design insufficient; only then does the added complexity of tracking
/// tree locality pay for itself.
///
/// # What `path_constraints_end_to_end_wired_vs_unwired` actually shows
///
/// `crate::registry::tests::path_constraints_end_to_end_wired_vs_unwired`
/// measures unwired 1/5, wired 5/5 for one `panda_arm` self-motion
/// scenario — but that 5/5 is *not* evidence this persistence fix works,
/// even though it was added in the same round. `PORTING-PLAN.md` §181
/// traced which of this round's two changes actually produces it: reverting
/// this persistence fix alone (`working` back to a per-attempt
/// `template.clone()`) leaves the numbers unchanged at 1/5 vs 5/5; reverting
/// instead the `resolve_constraint_sampler` wiring extension that lets a
/// solver reach the `path_constraints` call site at all
/// (`registry.rs`'s `PlanningRequest::solver` doc, `PORTING-PLAN.md`
/// §163.3) collapses wired to 1/5, identical to unwired. That test's goal
/// is a `Goal::State`, so without that wiring no `select_default_sampler`
/// call anywhere in the request ever sees a solver — wired and unwired
/// become behaviourally identical *by construction*, not merely by
/// coincidence of measurement. The wiring extension, not this persistence
/// fix, is what that test's 5/5 depends on. This fix's own effect is what
/// `try_sample_carries_the_previous_draws_result_forward_as_the_next_seed`
/// (below) measures directly, and reverting this fix alone reddens exactly
/// that one test.
pub struct GroupConstraintSampler<'a, 'm> {
    space: &'a JointModelGroupSpace,
    sampler: &'a dyn ConstraintSampler,
    working: RefCell<RobotState<'m>>,
}

impl<'a, 'm> GroupConstraintSampler<'a, 'm> {
    /// `template` seeds `working` once, at construction: [`ConstraintSampler::sample`]
    /// draws every variable of *its own* [`moveit_model::JointModelGroup`]
    /// (constrained ones from their tolerance window or IK, unconstrained
    /// ones from the joint's own bounds or a random restart), so only a
    /// variable entirely outside that group keeps `template`'s value for the
    /// lifetime of this sampler — and even that only matters to a caller
    /// reading `state` directly, since [`try_sample`](Self::try_sample)'s
    /// own return value is scoped to the group either way (see
    /// [`JointModelGroupSpace::read_robot_state`]).
    pub fn new(
        space: &'a JointModelGroupSpace,
        sampler: &'a dyn ConstraintSampler,
        template: RobotState<'m>,
    ) -> Self {
        Self {
            space,
            sampler,
            working: RefCell::new(template),
        }
    }
}

impl ConstrainedStateSampler<JointModelGroupSpace> for GroupConstraintSampler<'_, '_> {
    fn try_sample(&self, rng: &mut dyn Rng) -> Option<Vec<CompoundValue>> {
        let mut state = self.working.borrow_mut();
        self.sampler
            .sample(&mut state, rng)
            .then(|| self.space.read_robot_state(&state))
    }
}

#[cfg(test)]
mod tests {
    use moveit_constraints::{JointConstraint, JointConstraintSampler};
    use moveit_model::{JointModelGroup, MeshSearchPaths, RobotModel};
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

    /// A [`ConstraintSampler`] that records `panda_joint1`'s incoming value
    /// on every [`ConstraintSampler::sample`] call and writes back a
    /// deterministically different value, so successive calls' recorded
    /// history reveals whether the caller re-seeded from a fixed value each
    /// time or carried the previous call's output forward.
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

    /// Proves `working` is upstream's `work_state_`, not `template` reset
    /// per attempt: three [`GroupConstraintSampler::try_sample`] calls
    /// against a sampler that both records and mutates its own group's
    /// incoming value must see `[0.0, 0.01, 0.02]` -- each call's own
    /// previous output -- not `[0.0, 0.0, 0.0]`, which is what a
    /// reset-to-`template`-every-attempt design (this type's design before
    /// the fix this test guards) would have produced instead. See this
    /// type's own doc comment for why upstream's real `work_state_` behaves
    /// this way and what regressed without it.
    #[test]
    fn try_sample_carries_the_previous_draws_result_forward_as_the_next_seed() {
        let model = load_panda();
        let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
        let mut template = RobotState::new(&model);
        template.set_variable_position("panda_joint1", 0.0).unwrap();

        let recorder = RecordingSampler {
            group: model.joint_model_group("panda_arm").unwrap().clone(),
            seen: RefCell::new(Vec::new()),
        };
        let bridge = GroupConstraintSampler::new(&space, &recorder, template);
        let mut rng = ChaCha8Rng::seed_from_u64(1);

        for _ in 0..3 {
            bridge
                .try_sample(&mut rng)
                .expect("RecordingSampler::sample always succeeds");
        }

        assert_eq!(
            *recorder.seen.borrow(),
            vec![0.0, 0.01, 0.02],
            "each call's incoming panda_joint1 value must be the previous call's own output \
             (upstream's work_state_ semantics), not a fixed template value every time"
        );
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
