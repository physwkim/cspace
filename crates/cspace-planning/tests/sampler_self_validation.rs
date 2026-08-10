// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: `PORTING-PLAN.md` §5's Phase 5 second completion
// condition ("제약 샘플러가 생성한 상태 10,000개가 전부 자기 제약을 만족
// (자체 검증)"), as a command rather than a number in a report.

//! Every state this crate's constraint samplers produce is fed straight
//! back through the `decide()` of the constraints that sampler was
//! configured from, and must come back satisfied. No oracle: the sampler
//! and the decider are both this port, which is exactly what "자체 검증"
//! (self-validation) names.
//!
//! # The budget, and why it is split the way it is
//!
//! [`TOTAL_STATES`] is 10,000, split across [`SAMPLERS`]' seven
//! configurations. Each configuration draws a *fresh* constraint set per
//! state (a new random joint window, a new random reachable target pose),
//! so the sweep measures the sampler across its constraint space rather
//! than one constraint 10,000 times.
//!
//! # What each configuration can and cannot catch
//!
//! Three of the seven are genuinely unchecked paths, and four are
//! partially self-checking. That distinction is stated here rather than
//! left for a reader to discover, because a self-validation that only
//! re-asks a question the code under test already asked itself measures
//! nothing:
//!
//! * `joint_full_coverage`, `joint_partial_coverage` --
//!   [`JointConstraintSampler::sample`] has no internal validate step at
//!   all. It draws each constrained variable from the intersected window it
//!   computed at construction and writes it; nothing re-decides the
//!   constraint. A wrong intersection, a mis-scaled draw, or a variable
//!   written under the wrong name is caught here and nowhere else.
//! * `union_hand_joint_plus_arm_ik` -- [`UnionConstraintSampler::sample`]
//!   runs its members in `order_samplers` order and returns their `&&`. No
//!   member re-checks another member's work, so a union that drops a
//!   member, or an IK member that clobbers what the joint member wrote, is
//!   caught only from outside. The two members here are deliberately on
//!   *disjoint* variable sets (the `hand` group's finger joint versus
//!   `panda_arm`'s seven), so the joint constraint surviving the IK solve
//!   is a real requirement rather than a race the sampler was never asked
//!   to win.
//! * `ik_position_only`, `ik_orientation_only`, `ik_position_and_orientation`
//!   -- [`IkConstraintSampler::sample`] ends each accepted attempt with its
//!   own private `validate`, which decides the very same two constraints.
//!   Re-deciding them here therefore cannot catch a wrong *solution*; what
//!   it does catch is the state that comes back to the caller differing
//!   from the state `validate` approved, and a `sample` returning `true`
//!   with no accepted attempt behind it. Both are plumbing failures, and
//!   both are invisible from inside.
//! * `manager_partial_joint_plus_ik` -- `select_default_sampler` returns a
//!   union of a partial [`JointConstraintSampler`] and an
//!   [`IkConstraintSamplerAdapter`] over *overlapping* variables, which is
//!   upstream's own composition and upstream's own hazard: the IK member
//!   samples after the joint member and overwrites the arm joints it wrote.
//!   Upstream promises nothing about the joint constraint surviving that,
//!   so the joint window here is deliberately wide (it spans the joint's
//!   whole range) and the assertion that carries weight for this
//!   configuration is the IK pair's, not the joint constraint's. The
//!   narrow-window version of this configuration is
//!   `union_hand_joint_plus_arm_ik` above, where the members do not
//!   overlap and the joint window can be tight.
//!
//! # Cost
//!
//! Measured on this tree: see this file's `#[ignore]` note on
//! [`every_sampled_state_satisfies_its_own_constraints`].

use std::cell::RefCell;
use std::fs;
use std::rc::Rc;
use std::time::Instant;

use cspace_core::geometry::{Isometry3, Shape, Sphere, Transforms, Vector3};
use cspace_core::kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolverParams};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planning::constraints::{
    Constraint, ConstraintSampler, IkConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose,
    JointConstraint, JointConstraintSampler, OrientationConstraint, OrientationTolerance,
    PositionConstraint, UnionConstraintSampler, select_default_sampler,
};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// `PORTING-PLAN.md` §5's Phase 5 second clause, verbatim: 10,000 states
/// *generated* by the samplers -- so this counts states a sampler actually
/// handed back, not draws attempted. A draw where `sample` returns `false`
/// produced no state and cannot satisfy or violate anything; it is counted
/// separately (`attempted`) and reported, because how often a sampler has
/// to be asked twice is a real property of it, but it is not one of the
/// 10,000.
const TOTAL_STATES: usize = 10_000;

/// How many draws a configuration may spend to produce its quota, as a
/// multiple of that quota. Both bounds matter: without an upper bound a
/// sampler that never converges turns the sweep into a hang (which
/// `cargo nextest` reports as a slow test, not a failure), and without a
/// multiple greater than 1 a single non-converging draw would leave the
/// quota permanently short. 4 is far above what any configuration here
/// needs -- the worst measured is `ik_position_only` at 1.001 -- so
/// exhausting it means a sampler converges under 25% of the time, which is
/// a finding rather than a budget to widen.
const ATTEMPT_BUDGET: usize = 4;

/// The draw cap for a quota of `states`. See [`ATTEMPT_BUDGET`].
fn attempt_cap(states: usize) -> usize {
    states * ATTEMPT_BUDGET
}

/// Attempts each IK-backed sampler gets per state before it reports
/// failure. Bounds the only loop in this file that can run long: without a
/// cap, an unreachable target makes `IkConstraintSampler::sample` retry
/// forever and the sweep stops being a test and becomes a hang. 30 is what
/// `constraint_sampler_manager.rs`'s own tests already use.
const MAX_IK_ATTEMPTS: u32 = 30;

const PANDA_ARM_JOINTS: [&str; 7] = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
];

/// The finger joint the `hand` group's own sampler constrains in
/// `union_hand_joint_plus_arm_ik`. Not in `panda_arm`, so
/// `NewtonRaphsonSolver`'s solution never writes it -- which is the whole
/// point of that configuration.
const PANDA_FINGER_JOINT: &str = "panda_finger_joint1";

/// One configuration's name and how many of [`TOTAL_STATES`] it draws.
/// The counts sum to [`TOTAL_STATES`], checked by
/// [`the_budget_sums_to_the_completion_condition`].
const SAMPLERS: [(&str, usize); 7] = [
    ("joint_full_coverage", 2000),
    ("joint_partial_coverage", 1600),
    ("ik_position_only", 1600),
    ("ik_orientation_only", 1600),
    ("ik_position_and_orientation", 1600),
    ("union_hand_joint_plus_arm_ik", 800),
    ("manager_partial_joint_plus_ik", 800),
];

/// What one configuration's sweep produced. `produced` and `satisfied` are
/// separate counters on purpose: a sampler that converges on nothing has
/// `produced == 0` and a vacuously perfect `satisfied == produced`, which
/// is the exact shape a pass rate would hide, so
/// [`every_sampled_state_satisfies_its_own_constraints`] asserts on both.
#[derive(Debug)]
struct SamplerReport {
    name: &'static str,
    attempted: usize,
    produced: usize,
    satisfied: usize,
    /// First few dissatisfied states, for the failure message.
    violations: Vec<String>,
}

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

/// `(min, max)` for a single-variable joint, falling back to `[-pi, pi]`
/// when the URDF leaves a bound non-finite -- a continuous joint would
/// otherwise make every window below infinite and every draw trivially
/// inside it.
fn variable_range(model: &RobotModel, variable: &str) -> (f64, f64) {
    let bounds = model
        .joint_model(variable)
        .expect("caller passes a joint name")
        .variable_bounds_for(variable)
        .expect("a single-variable joint's variable is named after the joint");
    let min = if bounds.min_position.is_finite() {
        bounds.min_position
    } else {
        -std::f64::consts::PI
    };
    let max = if bounds.max_position.is_finite() {
        bounds.max_position
    } else {
        std::f64::consts::PI
    };
    (min, max)
}

/// A joint constraint on `variable` whose window is strictly inside the
/// joint's own range, so the sampler's construction-time intersection with
/// those bounds cannot silently widen it back to the full range and pass
/// for the wrong reason.
///
/// The tolerance is a *fraction of the joint's own span*, not an absolute
/// radian window: `panda_finger_joint1`'s whole range is 0.04m, so a fixed
/// 0.02-0.15 window is wider than the joint and leaves
/// `min + tolerance .. max - tolerance` empty. Scaling keeps the window
/// meaningfully narrow (at most 40% of the range) on both the 5.9-radian
/// arm joints and the 0.04m finger joint.
fn random_joint_constraint(
    model: &RobotModel,
    variable: &str,
    rng: &mut ChaCha8Rng,
) -> JointConstraint {
    let (min, max) = variable_range(model, variable);
    let span = max - min;
    let tolerance = rng.random_range(0.05 * span..0.20 * span);
    let position = rng.random_range(min + tolerance..max - tolerance);
    JointConstraint::new(model, variable, position, tolerance, tolerance, 1.0)
        .expect("the window sits strictly inside the joint's own bounds")
}

/// FK of a random in-bounds `panda_arm` configuration: a target pose that
/// is reachable by construction, so an IK sampler failing to converge on it
/// is a fact about the sampler and not about the target.
///
/// Only the seven arm joints are randomized; the model's other variables
/// (including `panda.srdf`'s floating virtual joint, which moves the whole
/// robot in the model frame) stay at their default values, matching the
/// state the sampler itself starts from. Randomizing the virtual joint here
/// but not there would move the target out from under the solver.
fn random_reachable_pose(model: &RobotModel, rng: &mut ChaCha8Rng) -> Isometry3 {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for name in PANDA_ARM_JOINTS {
        let (min, max) = variable_range(model, name);
        state
            .set_variable_position(name, rng.random_range(min..max))
            .expect("panda arm joint names are variables of this model");
    }
    state
        .update()
        .global_link_transform("panda_link8")
        .expect("panda_link8 is a link of this model")
}

fn position_constraint_at(model: &RobotModel, pose: Isometry3, radius: f64) -> PositionConstraint {
    let tf = Transforms::new("world").expect("world is a valid frame name");
    PositionConstraint::new(
        model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(radius).expect("radius is positive")),
            pose,
        )],
        1.0,
    )
    .expect("panda_link8 and world both resolve")
}

fn orientation_constraint_at(
    model: &RobotModel,
    pose: Isometry3,
    tolerance: f64,
) -> OrientationConstraint {
    let tf = Transforms::new("world").expect("world is a valid frame name");
    OrientationConstraint::new(
        model,
        &tf,
        "panda_link8",
        "world",
        pose.rotation,
        OrientationTolerance::RotationVector {
            x: tolerance,
            y: tolerance,
            z: tolerance,
        },
        1.0,
    )
    .expect("panda_link8 and world both resolve")
}

/// Records one draw: whether the sampler produced a state at all, and --
/// when it did -- whether every constraint it was configured from decides
/// `satisfied` on that state. The one place a state is judged, so all seven
/// configurations are judged the same way.
fn record(
    report: &mut SamplerReport,
    index: usize,
    state: &mut RobotState<'_>,
    produced: bool,
    constraints: &[Constraint],
) {
    report.attempted += 1;
    if !produced {
        return;
    }
    report.produced += 1;

    let posed = state.update();
    let mut unsatisfied: Vec<String> = Vec::new();
    for (i, constraint) in constraints.iter().enumerate() {
        let result = match constraint {
            Constraint::Joint(c) => c.decide(&posed),
            Constraint::Position(c) => c.decide(&posed),
            Constraint::Orientation(c) => c.decide(&posed),
            Constraint::Visibility(c) => c.decide(&posed),
        };
        if !result.satisfied {
            unsatisfied.push(format!("constraint {i} distance {}", result.distance));
        }
    }
    if unsatisfied.is_empty() {
        report.satisfied += 1;
    } else if report.violations.len() < 5 {
        report
            .violations
            .push(format!("state {index}: {}", unsatisfied.join(", ")));
    }
}

fn sweep_joint_sampler(
    model: &RobotModel,
    name: &'static str,
    states: usize,
    variables: &[&str],
    seed: u64,
) -> SamplerReport {
    let mut report = SamplerReport {
        name,
        attempted: 0,
        produced: 0,
        satisfied: 0,
        violations: Vec::new(),
    };
    let mut rng = ChaCha8Rng::seed_from_u64(seed);

    while report.produced < states && report.attempted < attempt_cap(states) {
        let index = report.attempted;
        let constraints: Vec<JointConstraint> = variables
            .iter()
            .map(|v| random_joint_constraint(model, v, &mut rng))
            .collect();
        let sampler = JointConstraintSampler::new(model, "panda_arm", &constraints)
            .expect("every constraint is on a panda_arm joint with a non-empty window");

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        let produced = sampler.sample(&mut state, &mut rng);
        let owned: Vec<Constraint> = constraints.into_iter().map(Constraint::Joint).collect();
        record(&mut report, index, &mut state, produced, &owned);
    }
    report
}

/// `with_position`/`with_orientation` pick which of the three IK
/// configurations this is; at least one must be true
/// ([`IkConstraintSampler::new`] rejects neither).
fn sweep_ik_sampler(
    model: &RobotModel,
    name: &'static str,
    states: usize,
    with_position: bool,
    with_orientation: bool,
    seed: u64,
) -> SamplerReport {
    let mut report = SamplerReport {
        name,
        attempted: 0,
        produced: 0,
        satisfied: 0,
        violations: Vec::new(),
    };
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let params = SolverParams::default();
    let mut solver =
        NewtonRaphsonSolver::new(model, "panda_arm", &params).expect("panda_arm is a chain");

    while report.produced < states && report.attempted < attempt_cap(states) {
        let index = report.attempted;
        let pose = random_reachable_pose(model, &mut rng);
        let pc = with_position.then(|| position_constraint_at(model, pose, 0.02));
        let oc = with_orientation.then(|| orientation_constraint_at(model, pose, 0.1));
        let sampler = IkConstraintSampler::new(
            model,
            &solver,
            IkSamplingPose {
                position_constraint: pc.clone(),
                orientation_constraint: oc.clone(),
            },
        )
        .expect("panda_arm's tip is panda_link8, the constrained link");

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        let produced = sampler.sample(&mut state, &mut solver, &mut rng, MAX_IK_ATTEMPTS, None);
        let mut owned: Vec<Constraint> = Vec::new();
        owned.extend(pc.map(Constraint::Position));
        owned.extend(oc.map(Constraint::Orientation));
        record(&mut report, index, &mut state, produced, &owned);
    }
    report
}

/// The disjoint-member union: a `hand`-group [`JointConstraintSampler`] on
/// `panda_finger_joint1` and a `panda_arm` [`IkConstraintSamplerAdapter`],
/// wrapped in a `panda_arm_hand` [`UnionConstraintSampler`]. See this
/// file's module doc for why the members must not overlap.
fn sweep_union_sampler(
    model: &RobotModel,
    name: &'static str,
    states: usize,
    seed: u64,
) -> SamplerReport {
    let mut report = SamplerReport {
        name,
        attempted: 0,
        produced: 0,
        satisfied: 0,
        violations: Vec::new(),
    };
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let params = SolverParams::default();
    let arm_group = model
        .joint_model_group("panda_arm")
        .expect("panda.srdf defines panda_arm")
        .clone();

    while report.produced < states && report.attempted < attempt_cap(states) {
        let index = report.attempted;
        let pose = random_reachable_pose(model, &mut rng);
        let pc = position_constraint_at(model, pose, 0.02);
        let oc = orientation_constraint_at(model, pose, 0.1);
        let finger = random_joint_constraint(model, PANDA_FINGER_JOINT, &mut rng);

        let joint_sampler =
            JointConstraintSampler::new(model, "hand", std::slice::from_ref(&finger))
                .expect("panda_finger_joint1 is a joint of the hand group");
        let solver: Rc<RefCell<Box<dyn KinematicsSolver>>> = Rc::new(RefCell::new(Box::new(
            NewtonRaphsonSolver::new(model, "panda_arm", &params).expect("panda_arm is a chain"),
        )));
        let ik_adapter = IkConstraintSamplerAdapter::new(
            model,
            &arm_group,
            solver,
            IkSamplingPose {
                position_constraint: Some(pc.clone()),
                orientation_constraint: Some(oc.clone()),
            },
            MAX_IK_ATTEMPTS,
        )
        .expect("panda_arm's tip is panda_link8, the constrained link");

        let union = UnionConstraintSampler::new(
            model,
            "panda_arm_hand",
            vec![Box::new(joint_sampler), Box::new(ik_adapter)],
        )
        .expect("panda.srdf defines panda_arm_hand");

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        let produced = union.sample(&mut state, &mut rng);
        let owned = vec![
            Constraint::Joint(finger),
            Constraint::Position(pc),
            Constraint::Orientation(oc),
        ];
        record(&mut report, index, &mut state, produced, &owned);
    }
    report
}

/// `select_default_sampler`'s own composition: a partial joint constraint
/// plus a reachable IK target on the same group. The joint window spans the
/// joint's whole range on purpose -- see this file's module doc.
fn sweep_manager_sampler(
    model: &RobotModel,
    name: &'static str,
    states: usize,
    seed: u64,
) -> SamplerReport {
    let mut report = SamplerReport {
        name,
        attempted: 0,
        produced: 0,
        satisfied: 0,
        violations: Vec::new(),
    };
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let params = SolverParams::default();

    while report.produced < states && report.attempted < attempt_cap(states) {
        let index = report.attempted;
        let pose = random_reachable_pose(model, &mut rng);
        let pc = position_constraint_at(model, pose, 0.02);
        let oc = orientation_constraint_at(model, pose, 0.1);
        let (min, max) = variable_range(model, "panda_joint7");
        let span = max - min;
        let wide = JointConstraint::new(model, "panda_joint7", 0.5 * (min + max), span, span, 1.0)
            .expect("a window spanning the whole range intersects the joint's bounds");

        let solver: Box<dyn KinematicsSolver> = Box::new(
            NewtonRaphsonSolver::new(model, "panda_arm", &params).expect("panda_arm is a chain"),
        );
        let constraints = vec![
            Constraint::Joint(wide.clone()),
            Constraint::Position(pc.clone()),
            Constraint::Orientation(oc.clone()),
        ];
        let sampler = select_default_sampler(
            model,
            "panda_arm",
            &constraints,
            Some(solver),
            vec![],
            MAX_IK_ATTEMPTS,
        )
        .expect("no subgroup names to resolve")
        .expect("a partial joint constraint plus a reachable IK target selects a union");

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        let produced = sampler.sample(&mut state, &mut rng);
        record(&mut report, index, &mut state, produced, &constraints);
    }
    report
}

/// The budget table is the completion condition's own number, split. A
/// split that no longer sums to it would leave the sweep quietly measuring
/// fewer states than it claims.
#[test]
fn the_budget_sums_to_the_completion_condition() {
    let total: usize = SAMPLERS.iter().map(|(_, n)| n).sum();
    assert_eq!(
        total, TOTAL_STATES,
        "SAMPLERS must draw exactly PORTING-PLAN.md §5's 10,000 states"
    );
}

/// Phase 5's second completion condition.
///
/// `#[ignore]`d on cost, not on a blocker: measured at 10.2-13.1s across
/// three runs on this tree, against 0.7s for this crate's other 103 tests
/// put together, so leaving it in the default suite would make one test the
/// whole crate's wall clock. Run it with
///
/// ```text
/// cargo nextest run -p cspace-planning --run-ignored all \
///   -E 'test(every_sampled_state_satisfies_its_own_constraints)'
/// ```
///
/// or let `tools/ci/verify-sampler-self-validation.sh` do it -- that script
/// is what `tools/ci/verify-all.sh` picks up per merge round, so this is
/// gated rather than merely written down. (`#[ignore]` alone would not be:
/// this repo's own rule is that a passing test left `#[ignore]`d never runs
/// again -- see `cspace-planning/tests/cost_sources_parity.rs`.)
#[test]
#[ignore = "10,000 sampled states, 10-13s; run via tools/ci/verify-sampler-self-validation.sh"]
fn every_sampled_state_satisfies_its_own_constraints() {
    let model = panda_model();
    let partial: Vec<&str> = PANDA_ARM_JOINTS[..3].to_vec();
    let full: Vec<&str> = PANDA_ARM_JOINTS.to_vec();

    let started = Instant::now();
    let reports = [
        sweep_joint_sampler(&model, SAMPLERS[0].0, SAMPLERS[0].1, &full, 101),
        sweep_joint_sampler(&model, SAMPLERS[1].0, SAMPLERS[1].1, &partial, 102),
        sweep_ik_sampler(&model, SAMPLERS[2].0, SAMPLERS[2].1, true, false, 103),
        sweep_ik_sampler(&model, SAMPLERS[3].0, SAMPLERS[3].1, false, true, 104),
        sweep_ik_sampler(&model, SAMPLERS[4].0, SAMPLERS[4].1, true, true, 105),
        sweep_union_sampler(&model, SAMPLERS[5].0, SAMPLERS[5].1, 106),
        sweep_manager_sampler(&model, SAMPLERS[6].0, SAMPLERS[6].1, 107),
    ];
    let elapsed = started.elapsed();

    let mut failures: Vec<String> = Vec::new();
    let mut attempted_total = 0usize;
    let mut produced_total = 0usize;
    let mut satisfied_total = 0usize;

    println!("per-sampler self-validation (attempted / produced / satisfied):");
    for (report, (_, quota)) in reports.iter().zip(SAMPLERS) {
        println!(
            "  {}: {} attempted, {} produced, {} satisfied",
            report.name, report.attempted, report.produced, report.satisfied
        );
        attempted_total += report.attempted;
        produced_total += report.produced;
        satisfied_total += report.satisfied;

        // A sampler that converges on nothing satisfies its constraints
        // vacuously, and one that converges rarely never reaches its share
        // of the 10,000. Both are reported and failed as their own case
        // rather than folded into the rate below.
        if report.produced < quota {
            failures.push(format!(
                "{}: produced {} of its {} states in {} attempts (cap {}){}",
                report.name,
                report.produced,
                quota,
                report.attempted,
                attempt_cap(quota),
                if report.produced == 0 {
                    " -- a vacuous 100%"
                } else {
                    ""
                }
            ));
        }
        if report.satisfied != report.produced {
            failures.push(format!(
                "{}: {} of {} produced states do not satisfy their own constraints; first few: {}",
                report.name,
                report.produced - report.satisfied,
                report.produced,
                report.violations.join(" | ")
            ));
        }
    }
    println!(
        "totals: {attempted_total} attempted, {produced_total} produced, \
         {satisfied_total} satisfied, {:.1}s",
        elapsed.as_secs_f64()
    );

    assert!(
        failures.is_empty(),
        "sampler self-validation failed:\n  {}",
        failures.join("\n  ")
    );
    // After the per-configuration checks above, not before: if a
    // configuration came up short this restates it as one number, but the
    // line naming *which* sampler is the one worth reading.
    assert_eq!(
        produced_total, TOTAL_STATES,
        "the sweep must produce exactly PORTING-PLAN.md §5's 10,000 states \
         ({attempted_total} draws attempted)"
    );
    assert_eq!(
        satisfied_total, produced_total,
        "every produced state must satisfy its own constraints"
    );
}
