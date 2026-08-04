// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! [`select_default_sampler`] tests, one per boundary of upstream's Step
//! A-E decision tree rather than one per narrative scenario: an unresolvable
//! group name (`Ok(None)`), an unresolvable subgroup name (`Err`), full vs.
//! partial joint coverage, an IK winner alone, a partial-joint-plus-IK
//! union, the per-link sampling-volume tie-break, and the one-level subgroup
//! recursion. Each test checks the *shape* of the decision actually taken
//! (which step fired), not just that some `Some(_)` came back — see each
//! test's own comment for how it distinguishes its step from its
//! neighbours.

use std::fs;

use moveit_constraints::{
    Constraint, JointConstraint, OrientationConstraint, OrientationTolerance, PositionConstraint,
    SubgroupSolver, select_default_sampler,
};
use moveit_geometry::{Isometry3, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
use moveit_kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolveOptions, SolverParams};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
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

const PANDA_ARM_JOINTS: [&str; 7] = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
];

/// A [`KinematicsSolver`] whose only job is to satisfy
/// [`moveit_constraints::IkConstraintSampler::new`]'s frame checks at
/// construction time — every test in this file that only needs
/// [`select_default_sampler`] to *build* a sampler (not to actually run
/// `.sample()` against a solver that can't really reach anything) uses this
/// rather than a real IK solver.
struct FakeTip {
    tip: String,
    base: String,
}

impl FakeTip {
    fn new(tip: &str) -> Self {
        Self {
            tip: tip.to_owned(),
            base: "world".to_owned(),
        }
    }
}

impl KinematicsSolver for FakeTip {
    fn group_name(&self) -> &str {
        "fake"
    }
    fn joint_names(&self) -> &[String] {
        &[]
    }
    fn base_frame(&self) -> &str {
        &self.base
    }
    fn tip_frame(&self) -> &str {
        &self.tip
    }
    fn solve_with_options(
        &mut self,
        _seed: &[f64],
        _target: &Isometry3,
        _options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        panic!("FakeTip is only used to satisfy construction-time frame checks, never solved")
    }
}

fn full_coverage_joint_constraints(model: &RobotModel) -> Vec<JointConstraint> {
    PANDA_ARM_JOINTS
        .iter()
        .map(|&name| JointConstraint::new(model, name, 0.0, 3.0, 3.0, 1.0).unwrap())
        .collect()
}

/// With every other argument empty (as `no_constraints_and_no_solver_returns_none`
/// below also is), Step D/E's fallthrough produces `Ok(None)` for *any*
/// group, real or not -- so a bare `result.is_none()` here would still pass
/// even if the unresolvable-group early return were deleted and `group_name`
/// fell through to a real group instead. `subgroup_solvers` names a subgroup
/// that only Step C would ever look at, and Step C errors immediately on an
/// unresolvable name (`unresolvable_subgroup_name_is_an_error` below) -- so
/// getting `Ok(None)` here rather than that error proves the early return
/// fired before Step C ran, not that Step C happened to also land on `None`.
#[test]
fn unknown_group_name_returns_none() {
    let model = panda_model();
    let solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_link8"));
    let result = select_default_sampler(
        &model,
        "no_such_group",
        &[],
        None,
        vec![SubgroupSolver {
            group_name: "no_such_subgroup".to_string(),
            solver,
            subgroup_solvers: vec![],
        }],
        1,
    );
    match result {
        Ok(None) => {}
        Ok(Some(_)) => panic!("expected None, got a sampler"),
        Err(e) => panic!(
            "expected the unresolvable top-level group to short-circuit before Step C ever \
             looked at subgroup_solvers, but got an error from there instead: {e}"
        ),
    }
}

#[test]
fn unresolvable_subgroup_name_is_an_error() {
    let model = panda_model();
    let solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_link8"));
    let result = select_default_sampler(
        &model,
        "panda_arm_hand",
        &[],
        None,
        vec![SubgroupSolver {
            group_name: "no_such_subgroup".to_string(),
            solver,
            subgroup_solvers: vec![],
        }],
        1,
    );
    match result {
        Err(moveit_error::Error::UnknownName { kind, name }) => {
            assert_eq!(kind, "group");
            assert_eq!(name, "no_such_subgroup");
        }
        other => panic!(
            "expected Err(UnknownName), got a different outcome: is_err={}",
            other.is_err()
        ),
    }
}

#[test]
fn no_constraints_and_no_solver_returns_none() {
    let model = panda_model();
    let result = select_default_sampler(&model, "panda_arm", &[], None, vec![], 1).unwrap();
    assert!(
        result.is_none(),
        "Step A found no joint constraints, Step B had no solver, Step C had no subgroup \
         solvers — samplers stays empty end to end"
    );
}

#[test]
fn full_joint_coverage_returns_the_joint_sampler_alone() {
    let model = panda_model();
    let constraints: Vec<Constraint> = full_coverage_joint_constraints(&model)
        .into_iter()
        .map(Constraint::Joint)
        .collect();

    let sampler = select_default_sampler(&model, "panda_arm", &constraints, None, vec![], 1)
        .unwrap()
        .expect("all 7 panda_arm variables are covered by a joint constraint");
    assert_eq!(sampler.joint_model_group().name(), "panda_arm");

    let mut state = RobotState::new(&model);
    let mut rng = ChaCha8Rng::seed_from_u64(10);
    assert!(
        sampler.sample(&mut state, &mut rng),
        "a JointConstraintSampler always succeeds by construction"
    );
    let posed = state.update();
    for name in PANDA_ARM_JOINTS {
        let joint = JointConstraint::new(&model, name, 0.0, 3.0, 3.0, 1.0).unwrap();
        assert!(
            joint.decide(&posed).satisfied,
            "{name}: sampled value must stay within its own tolerance window"
        );
    }
}

#[test]
fn partial_joint_coverage_without_ik_falls_back_to_the_partial_joint_sampler() {
    let model = panda_model();
    // Only one of panda_arm's 7 variables is constrained: not full coverage,
    // so this must fall through Step A's early return, find no solver in
    // Step B and no subgroup_solvers in Step C, and land on Step D/E's
    // `samplers.pop()` — the very sampler Step A pushed as a fallback.
    let joint = JointConstraint::new(&model, "panda_joint1", 0.4, 0.05, 0.05, 1.0).unwrap();
    let constraints = vec![Constraint::Joint(joint.clone())];

    let sampler = select_default_sampler(&model, "panda_arm", &constraints, None, vec![], 1)
        .unwrap()
        .expect("Step A's partial-coverage fallback must still produce a sampler");

    let mut state = RobotState::new(&model);
    let mut rng = ChaCha8Rng::seed_from_u64(11);
    assert!(sampler.sample(&mut state, &mut rng));
    let posed = state.update();
    assert!(
        joint.decide(&posed).satisfied,
        "the one constrained joint must land inside its tolerance window"
    );
}

#[test]
fn ik_alone_when_there_are_no_joint_constraints() {
    let model = panda_model();
    let params = SolverParams::default();
    let solver: Box<dyn KinematicsSolver> = Box::new(
        NewtonRaphsonSolver::new(&model, "panda_arm", &params).expect("panda_arm is a chain"),
    );

    let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
    let mut fk_state = RobotState::new(&model);
    fk_state.set_to_default_values();
    for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&true_values) {
        fk_state.set_variable_position(name, v).unwrap();
    }
    let target_pose = fk_state
        .update()
        .global_link_transform("panda_link8")
        .unwrap();

    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(Shape::Sphere(Sphere::new(0.02).unwrap()), target_pose)],
        1.0,
    )
    .unwrap();
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        target_pose.rotation,
        OrientationTolerance::RotationVector {
            x: 0.1,
            y: 0.1,
            z: 0.1,
        },
        1.0,
    )
    .unwrap();
    let constraints = vec![
        Constraint::Position(pc.clone()),
        Constraint::Orientation(oc.clone()),
    ];

    let sampler =
        select_default_sampler(&model, "panda_arm", &constraints, Some(solver), vec![], 30)
            .unwrap()
            .expect("a reachable target must produce an IK sampler");
    // No joint constraints, so Step B's `samplers.is_empty()` branch fires:
    // the winner is returned directly, not wrapped in a UnionConstraintSampler.
    // That directness is only observable by behaviour here (no downcast on
    // the trait object), so this is checked by the group name matching
    // panda_arm exactly rather than some union-of-panda_arm wrapper —
    // both report the same name, so the real distinguishing check is the
    // union test below actually satisfying two independent constraint
    // families at once.
    assert_eq!(sampler.joint_model_group().name(), "panda_arm");

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(12);
    assert!(
        sampler.sample(&mut state, &mut rng),
        "newton-raphson must find a solution near a reachable target"
    );
    let posed = state.update();
    assert!(pc.decide(&posed).satisfied);
    assert!(oc.decide(&posed).satisfied);
}

#[test]
fn partial_joint_plus_ik_returns_a_union_satisfying_both() {
    let model = panda_model();
    let params = SolverParams::default();
    let solver: Box<dyn KinematicsSolver> = Box::new(
        NewtonRaphsonSolver::new(&model, "panda_arm", &params).expect("panda_arm is a chain"),
    );

    // A wide-open joint constraint on panda_joint7 alone: partial coverage,
    // easy for any IK solution to also satisfy, so a failure to satisfy it
    // after sampling would mean the union dropped a member, not that the
    // window was too narrow to hit.
    let joint = JointConstraint::new(&model, "panda_joint7", 0.0, 3.0, 3.0, 1.0).unwrap();

    let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];
    let mut fk_state = RobotState::new(&model);
    fk_state.set_to_default_values();
    for (name, &v) in PANDA_ARM_JOINTS.iter().zip(&true_values) {
        fk_state.set_variable_position(name, v).unwrap();
    }
    let target_pose = fk_state
        .update()
        .global_link_transform("panda_link8")
        .unwrap();
    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(Shape::Sphere(Sphere::new(0.02).unwrap()), target_pose)],
        1.0,
    )
    .unwrap();
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        target_pose.rotation,
        OrientationTolerance::RotationVector {
            x: 0.1,
            y: 0.1,
            z: 0.1,
        },
        1.0,
    )
    .unwrap();
    let constraints = vec![
        Constraint::Joint(joint.clone()),
        Constraint::Position(pc.clone()),
        Constraint::Orientation(oc.clone()),
    ];

    let sampler =
        select_default_sampler(&model, "panda_arm", &constraints, Some(solver), vec![], 30)
            .unwrap()
            .expect("both a partial joint constraint and a reachable IK target are present");

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(13);
    assert!(sampler.sample(&mut state, &mut rng));
    let posed = state.update();
    assert!(
        joint.decide(&posed).satisfied,
        "a union must still run its JointConstraintSampler member"
    );
    assert!(
        pc.decide(&posed).satisfied && oc.decide(&posed).satisfied,
        "a union must still run its IK member"
    );
}

#[test]
fn per_link_tie_break_keeps_the_later_candidate() {
    // Two position constraints and two orientation constraints, all on
    // "panda_link8", with equal sampling volume (same sphere radius, same
    // rotation tolerances) but distinguishable mobile reference frames.
    // `collect_ik_candidates`'s pairing loop visits them in order (pc_a,
    // oc_a), (pc_a, oc_b), (pc_b, oc_a), (pc_b, oc_b); with every volume
    // tied, `insert_or_replace_on_tie`'s documented "tie -> later wins"
    // rule means the survivor must be built from (pc_b, oc_b) — observable
    // here via `frame_dependency()`, since each candidate's mobile frames
    // are baked in at construction and nothing else about them differs.
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let radius = 0.1;
    let region = |center: f64| {
        [(
            Shape::Sphere(Sphere::new(radius).unwrap()),
            Isometry3::from_parts(
                Vector3::new(center, 0.0, 0.0).into(),
                UnitQuaternion::identity(),
            ),
        )]
    };
    let pc_a = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "panda_link1",
        Vector3::zeros(),
        &region(0.1),
        1.0,
    )
    .unwrap();
    let pc_b = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "panda_link2",
        Vector3::zeros(),
        &region(0.2),
        1.0,
    )
    .unwrap();
    assert!(pc_a.mobile_reference_frame());
    assert!(pc_b.mobile_reference_frame());

    let tol = (0.2, 0.15, 0.1);
    let oc_a = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        UnitQuaternion::identity(),
        OrientationTolerance::RotationVector {
            x: tol.0,
            y: tol.1,
            z: tol.2,
        },
        1.0,
    )
    .unwrap();
    let oc_b = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "panda_link3",
        UnitQuaternion::identity(),
        OrientationTolerance::RotationVector {
            x: tol.0,
            y: tol.1,
            z: tol.2,
        },
        1.0,
    )
    .unwrap();
    assert!(!oc_a.mobile_reference_frame());
    assert!(oc_b.mobile_reference_frame());

    let solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_link8"));
    let constraints = vec![
        Constraint::Position(pc_a),
        Constraint::Position(pc_b),
        Constraint::Orientation(oc_a),
        Constraint::Orientation(oc_b),
    ];

    let sampler =
        select_default_sampler(&model, "panda_arm", &constraints, Some(solver), vec![], 1)
            .unwrap()
            .expect("all four pairings resolve to the same tip link, panda_link8");

    assert_eq!(
        sampler.frame_dependency(),
        &["panda_link2".to_string(), "panda_link3".to_string()],
        "expected the last-considered pairing (pc_b, oc_b) to win the tie; a \
         'tie -> earlier wins' bug would instead report [\"panda_link1\"] \
         (oc_a is fixed-frame and contributes no dependency)"
    );
}

#[test]
fn subgroup_recursion_wraps_the_winning_subgroup_candidate_in_a_union_named_for_the_parent_group() {
    // panda_arm_hand's own subgroups are panda_arm and hand (panda.srdf).
    // No direct solver for panda_arm_hand itself (Step B skipped), and the
    // position/orientation constraints target panda_hand, a link that
    // belongs to the "hand" subgroup, not "panda_arm" — so only the "hand"
    // subgroup_solvers entry can produce a candidate.
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_hand",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(0.1).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let constraints = vec![Constraint::Position(pc)];
    let hand_solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_hand"));

    let sampler = select_default_sampler(
        &model,
        "panda_arm_hand",
        &constraints,
        None,
        vec![SubgroupSolver {
            group_name: "hand".to_string(),
            solver: hand_solver,
            subgroup_solvers: vec![],
        }],
        1,
    )
    .unwrap()
    .expect("the hand subgroup solver can service the panda_hand position constraint");

    // The recursive call's own direct-return (Step B, no Step A in that
    // call) reports "hand" as its group; if Step C's wrap were missing or
    // dropped, that inner "hand"-named sampler would leak straight through
    // instead of the outer UnionConstraintSampler reporting the parent
    // group's own name.
    assert_eq!(
        sampler.joint_model_group().name(),
        "panda_arm_hand",
        "the subgroup candidate must come back wrapped in a union named for panda_arm_hand, \
         not leak through still named for its subgroup"
    );
}

#[test]
fn subgroup_recursion_two_levels_deep_resolves_through_both_levels() {
    // None of panda/fanuc/dual-arm panda nests a group two subgroup-levels
    // deep with a solver at the bottom (see this crate's
    // constraint_sampler_manager module doc comment), so this builds a
    // synthetic `top` -> subgroup `mid` -> subgroup `leaf` hierarchy on
    // panda's own links via `SrdfModel::parse_str`, no fixture file added.
    // `leaf` is a real chain (`panda_link6` -> `panda_link8`, covering
    // revolute joint7 and the fixed joint8) so it can carry a real IK
    // candidate; `mid` and `top` contribute no links of their own, only the
    // nested subgroup.
    let urdf_path = fixture_path("panda.urdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read panda.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse panda.urdf");
    // Mirrors panda.srdf's own virtual_joint: without it "world" isn't a
    // resolvable frame, and IkConstraintSampler::new's base_frame check
    // (ik_sampler.rs) rejects every candidate before the tip-frame check
    // this test actually means to exercise ever runs.
    let srdf_xml = r#"<?xml version="1.0" encoding="utf-8"?>
<robot name="panda">
  <virtual_joint child_link="panda_link0" name="virtual_joint" parent_frame="world" type="floating"/>
  <group name="leaf">
    <chain base_link="panda_link6" tip_link="panda_link8"/>
  </group>
  <group name="mid">
    <group name="leaf"/>
  </group>
  <group name="top">
    <group name="mid"/>
  </group>
</robot>"#;
    let srdf = SrdfModel::parse_str(srdf_xml).expect("parse synthetic top/mid/leaf SRDF");
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("build model with synthetic top/mid/leaf groups");

    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(0.1).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let constraints = vec![Constraint::Position(pc)];

    // `mid`'s own direct solver's tip frame deliberately does not match
    // `panda_link8`, so Step B at the `mid` level cannot satisfy the
    // constraint by itself — only recursing one level further, into `leaf`,
    // can. If Step C dropped `mid`'s own nested `subgroup_solvers` (the bug
    // the `SubgroupSolver` tree replaces, see this crate's
    // constraint_sampler_manager module doc comment), `mid` would have
    // nothing left to recurse into, `mid` would return `None`, and so would
    // `top`.
    let mid_solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_link0"));
    let leaf_solver: Box<dyn KinematicsSolver> = Box::new(FakeTip::new("panda_link8"));

    let sampler = select_default_sampler(
        &model,
        "top",
        &constraints,
        None,
        vec![SubgroupSolver {
            group_name: "mid".to_string(),
            solver: mid_solver,
            subgroup_solvers: vec![SubgroupSolver {
                group_name: "leaf".to_string(),
                solver: leaf_solver,
                subgroup_solvers: vec![],
            }],
        }],
        1,
    )
    .unwrap()
    .expect("depth-2 recursion (top -> mid -> leaf) must resolve the position constraint");

    assert_eq!(
        sampler.joint_model_group().name(),
        "top",
        "the depth-2 candidate must come back wrapped for the outermost group, top"
    );
}
