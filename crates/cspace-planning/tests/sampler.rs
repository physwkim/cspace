// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Unit tests for [`JointConstraintSampler`] and [`UnionConstraintSampler`],
//! one case per invariant boundary named in this round's brief: an empty
//! bounds intersection, a single-point (`tolerance == 0`) intersection, a
//! constraint on a joint the group does not contain, and a union whose
//! input order is the reverse of the order `order_samplers` requires. A
//! fifth test covers the one production code path the four boundary cases
//! don't reach: [`JointConstraintSampler::sample`]'s scratch-`RobotState`
//! draw for a group's *unconstrained* variables actually stays within each
//! joint's own bounds (see that method's doc comment for why it goes through
//! a scratch state rather than a duplicated per-joint-kind sampler). A sixth
//! raises the single-point boundary from one hand-built constraint to the
//! whole-group goal set a concrete-state request becomes
//! ([`a_zero_tolerance_goal_set_resolves_to_its_own_state`]).
//!
//! `panda.urdf`/`panda.srdf` (copied from `cspace-state`'s fixtures, already
//! oracle-verified — see `crates/cspace-core/tests/fixtures/constraints/model/panda_model_info.json`)
//! supply a real model. `panda_joint1`'s `[-2.8973, 2.8973]` bound below is
//! from that file's own `<safety_controller soft_lower_limit=""
//! soft_upper_limit="">` element on `panda_joint1` — `cspace-model`'s URDF
//! loader prefers safety-controller soft limits over `<limit>` hard limits
//! when both are present, matching upstream `RobotModel::computeVariableBoundsMsg`.

use std::fs;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::constraints::{
    ConstraintSampler, JointConstraint, JointConstraintSampler, UnionConstraintSampler,
    select_default_sampler,
};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/constraints/{}"),
        file_name
    )
}

fn panda_model() -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read panda.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse panda.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse panda.srdf");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("build panda model")
}

/// `panda_joint1..7`'s own bounds, transcribed from `panda.urdf`'s
/// `<safety_controller>` elements (see this file's doc comment) in
/// `panda_arm`'s own joint order.
const PANDA_ARM_BOUNDS: [(&str, f64, f64); 7] = [
    ("panda_joint1", -2.8973, 2.8973),
    ("panda_joint2", -1.7628, 1.7628),
    ("panda_joint3", -2.8973, 2.8973),
    ("panda_joint4", -3.0718, 0.0175),
    ("panda_joint5", -2.8973, 2.8973),
    ("panda_joint6", -0.0175, 3.7525),
    ("panda_joint7", -2.8973, 2.8973),
];

/// Two [`JointConstraint`]s on the same variable, `panda_joint1`, whose
/// tolerance windows do not overlap at all (`[0.95, 1.05]` and
/// `[1.95, 2.05]`): each is individually valid against the joint's own
/// `[-2.8973, 2.8973]` bounds (so [`JointConstraint::new`] accepts both
/// unmodified), but [`JointConstraintSampler::new`] must intersect the two
/// and discard the whole constraint set — upstream's "no possible values
/// for the joint", a configure-time failure, not a sample-time one.
#[test]
fn configure_fails_on_empty_intersection_between_two_constraints() {
    let model = panda_model();
    let a = JointConstraint::new(&model, "panda_joint1", 1.0, 0.05, 0.05, 1.0).unwrap();
    let b = JointConstraint::new(&model, "panda_joint1", 2.0, 0.05, 0.05, 1.0).unwrap();

    let err = JointConstraintSampler::new(&model, "panda_arm", &[a, b]).unwrap_err();
    assert!(
        err.to_string().contains("panda_joint1"),
        "error should name the offending joint variable: {err}"
    );
}

/// `tolerance_above == tolerance_below == 0.0` intersects to a single point,
/// not an empty range — `JointConstraintSampler::new` must accept it (a
/// zero-width window is still a valid, if degenerate, sampling range), and
/// every sample must land exactly on that point regardless of what the RNG
/// draws, since `rng.random_range(p..=p)` has exactly one possible outcome.
#[test]
fn single_point_intersection_samples_the_exact_position_every_time() {
    let model = panda_model();
    let c = JointConstraint::new(&model, "panda_joint1", 0.3, 0.0, 0.0, 1.0).unwrap();
    let sampler =
        JointConstraintSampler::new(&model, "panda_arm", std::slice::from_ref(&c)).unwrap();
    assert_eq!(sampler.constrained_variable_count(), 1);

    let mut state = RobotState::new(&model);
    let mut rng = ChaCha8Rng::seed_from_u64(7);
    for _ in 0..20 {
        assert!(sampler.sample(&mut state, &mut rng));
        let v = state.variable_position("panda_joint1").unwrap();
        assert_eq!(
            v, 0.3,
            "single-point constraint sampled {v}, expected exactly 0.3"
        );
    }
}

/// The tolerance a goal that names a concrete state must be built at, and
/// the subject of [`a_zero_tolerance_goal_set_resolves_to_its_own_state`]:
/// change this constant and that test fails, which is the whole point of it
/// being a constant rather than a literal inside the assertion.
const EXACT_STATE_GOAL_TOLERANCE: f64 = 0.0;

/// The whole-set form of the single-point case above, and the property a
/// concrete-state goal rests on: a goal set built from a state by
/// [`construct_goal_joint_constraints`] at
/// [`EXACT_STATE_GOAL_TOLERANCE`], resolved the way a planner resolves one
/// (`select_default_sampler` then [`ConstraintSampler::sample`]), gives that
/// state back bit for bit on every variable of the group.
///
/// The test above covers one hand-built constraint on one joint; this one
/// covers the path an actual goal takes — all seven variables at once, from
/// positions that are drawn rather than chosen. Without it a caller can
/// widen the tolerance it builds goals with and nothing in the workspace
/// notices, which is how `1e-9` reached `plan_benchmark_port` and put a
/// sampled region where the requested state should have been.
///
/// The `f64::EPSILON` half is what makes the first half mean something: that
/// width is upstream's own `constructGoalConstraints` default, it satisfies
/// every [`cspace_planning::constraints::JointConstraint::decide`] check here, and it
/// still misses the requested state. "The sample is accepted" and "the
/// sample is the state" are different properties, and only a zero-width
/// window buys the second.
#[test]
fn a_zero_tolerance_goal_set_resolves_to_its_own_state() {
    let model = panda_model();
    let mut rng = ChaCha8Rng::seed_from_u64(19);

    // Resolves `tolerance`-wide goal constraints built from `posed` and
    // returns each `panda_arm` variable's (requested, resolved) pair.
    let resolve =
        |posed: &cspace_core::state::Posed<'_, '_>, tolerance: f64, rng: &mut ChaCha8Rng| {
            let set =
                construct_goal_joint_constraints(&model, posed, "panda_arm", tolerance, tolerance)
                    .expect("panda_arm is real and every variable of it is set");
            let sampler =
                select_default_sampler(&model, "panda_arm", set.constraints(), None, vec![], 4)
                    .expect("no subgroup solvers are named, so the only Err arm is unreachable")
                    .expect("an all-joint-constraint set resolves to a JointConstraintSampler");

            let mut state = RobotState::new(&model);
            state.set_to_default_values();
            assert!(
                sampler.sample(&mut state, rng),
                "JointConstraintSampler::sample is infallible"
            );
            let resolved = state.update();
            assert!(
                set.decide(&resolved).satisfied,
                "tolerance {tolerance:e}: the resolved state must satisfy the set it was drawn from"
            );
            PANDA_ARM_BOUNDS
                .iter()
                .map(|(name, _, _)| {
                    (
                        *name,
                        posed.variable_position(name).unwrap(),
                        resolved.variable_position(name).unwrap(),
                    )
                })
                .collect::<Vec<_>>()
        };

    let mut epsilon_misses = 0usize;
    for _ in 0..64 {
        let mut goal = RobotState::new(&model);
        goal.set_to_random_positions_with(&mut rng);
        let posed = goal.update();

        for (name, want, got) in resolve(&posed, EXACT_STATE_GOAL_TOLERANCE, &mut rng) {
            assert_eq!(
                got,
                want,
                "an exact goal on {name} resolved to {got}, not the requested {want} \
                 (gap {})",
                got - want
            );
        }
        epsilon_misses += resolve(&posed, f64::EPSILON, &mut rng)
            .into_iter()
            .filter(|(_, want, got)| got != want)
            .count();
    }

    assert!(
        epsilon_misses > 0,
        "upstream's f64::EPSILON default reproduced the requested state exactly on all {} \
         variable draws; if that ever becomes true, the exact half above no longer \
         discriminates and this test's reason for existing must be re-derived",
        64 * PANDA_ARM_BOUNDS.len()
    );
}

/// A [`JointConstraint`] on `panda_finger_joint1`, which is not a member of
/// `panda_arm` (it belongs to the `hand` group) — `JointConstraintSampler::new`
/// must fail configuration entirely rather than silently building a sampler
/// with no constraints on the requested group (upstream: "No valid joint
/// constraints").
#[test]
fn configure_fails_when_the_only_constraint_is_on_a_joint_outside_the_group() {
    let model = panda_model();
    let c = JointConstraint::new(&model, "panda_finger_joint1", 0.02, 0.01, 0.01, 1.0).unwrap();

    let err = JointConstraintSampler::new(&model, "panda_arm", &[c]).unwrap_err();
    assert!(
        err.to_string().contains("panda_arm"),
        "error should name the group with no applicable constraint: {err}"
    );
}

/// `panda_arm_hand`'s updated links are a proper superset of `hand`'s
/// (the whole-arm-plus-hand group moves every link the hand-only group
/// does, plus the arm's own) -- `order_samplers` must place the
/// `panda_arm_hand` sampler first regardless of the order the caller
/// supplies them in. Built here in the reverse of that required order
/// (`hand` first, `panda_arm_hand` second) so the assertion actually
/// exercises the sort, not just an already-correct input order.
#[test]
fn union_sorts_by_link_containment_even_when_input_order_is_reversed() {
    let model = panda_model();

    let hand_constraint =
        JointConstraint::new(&model, "panda_finger_joint1", 0.02, 0.01, 0.01, 1.0).unwrap();
    let hand_sampler =
        JointConstraintSampler::new(&model, "hand", std::slice::from_ref(&hand_constraint))
            .unwrap();

    let arm_constraint = JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap();
    let arm_hand_sampler = JointConstraintSampler::new(
        &model,
        "panda_arm_hand",
        std::slice::from_ref(&arm_constraint),
    )
    .unwrap();

    // Input order is deliberately reversed relative to what `order_samplers`
    // requires: `hand` (the narrower group) before `panda_arm_hand` (the
    // wider one).
    let union = UnionConstraintSampler::new(
        &model,
        "panda_arm_hand",
        vec![
            Box::new(hand_sampler) as Box<dyn ConstraintSampler>,
            Box::new(arm_hand_sampler) as Box<dyn ConstraintSampler>,
        ],
    )
    .unwrap();

    let sorted_names: Vec<&str> = union.samplers().iter().map(|s| s.group_name()).collect();
    assert_eq!(
        sorted_names,
        vec!["panda_arm_hand", "hand"],
        "the wider group (panda_arm_hand) must sample before the narrower one it contains (hand)"
    );
}

/// The path the four boundary tests above don't reach: an *unconstrained*
/// group variable's random draw, which goes through a scratch
/// [`RobotState`] rather than a per-variable uniform draw (see
/// [`JointConstraintSampler::sample`]'s doc comment). Only `panda_joint1` is
/// constrained here; every repeated sample must still land within
/// `panda_joint2..7`'s own bounds (transcribed independently in
/// `PANDA_ARM_BOUNDS`, not read back from the model under test), and
/// `panda_joint1` must stay within its own tightened window.
#[test]
fn sample_keeps_unconstrained_variables_within_their_own_joint_bounds() {
    let model = panda_model();
    let c = JointConstraint::new(&model, "panda_joint1", 0.0, 0.2, 0.2, 1.0).unwrap();
    let sampler =
        JointConstraintSampler::new(&model, "panda_arm", std::slice::from_ref(&c)).unwrap();
    assert_eq!(sampler.unconstrained_variable_count(), 6);

    let mut state = RobotState::new(&model);
    let mut rng = ChaCha8Rng::seed_from_u64(11);
    for _ in 0..200 {
        assert!(sampler.sample(&mut state, &mut rng));
        for &(name, min, max) in &PANDA_ARM_BOUNDS {
            let v = state.variable_position(name).unwrap();
            assert!(
                (min..=max).contains(&v),
                "{name} sampled to {v}, outside its own bounds [{min}, {max}]"
            );
        }
        let joint1 = state.variable_position("panda_joint1").unwrap();
        assert!(
            (-0.2..=0.2).contains(&joint1),
            "panda_joint1 sampled to {joint1}, outside its constrained window [-0.2, 0.2]"
        );
    }
}
