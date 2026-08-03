// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! [`IkConstraintSampler`] tests, one case per invariant boundary named in
//! round 9's brief rather than one per narrative scenario: the three
//! `getSamplingVolume` shapes (sphere region, box region, rotation-vector
//! tolerance product) each on their own; a position-only constraint; an
//! orientation-only constraint; the restart path when IK never converges;
//! and seed/solution vector ordering by name. A final test runs the whole
//! pipeline — real geometry, a real [`NewtonRaphsonSolver`], and a
//! genuinely non-identity `transform_ik` (panda_arm's solver base frame,
//! `panda_link0`, differs from the model frame, `world`, per
//! `panda.srdf`'s floating virtual joint) — since the boundary cases below
//! all use fakes or direct `sample_pose` calls that never exercise that
//! composition together with a real solve.
//!
//! `panda.urdf`/`panda.srdf` (copied from `moveit-state`'s fixtures,
//! already oracle-verified — see
//! `crates/moveit-model/tests/fixtures/panda_model_info.json`) supply a
//! real model.

use std::cell::Cell;
use std::f64::consts::PI;
use std::fs;

use moveit_constraints::{
    IkConstraintSampler, IkSamplingPose, OrientationConstraint, OrientationTolerance,
    PositionConstraint,
};
use moveit_geometry::{Cuboid, Isometry3, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
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

// ---- getSamplingVolume: sphere / box / rotation-vector, each on its own ----

#[test]
fn sampling_volume_sums_sphere_region_bodies() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let radius = 0.12;
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(radius).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();

    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let expected = 4.0 / 3.0 * PI * radius.powi(3);
    assert!(
        (ik.sampling_volume() - expected).abs() < 1e-12,
        "got {}, expected {expected}",
        ik.sampling_volume()
    );
}

#[test]
fn sampling_volume_sums_box_region_bodies() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let (x, y, z) = (0.2, 0.3, 0.4);
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(x, y, z).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();

    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let expected = x * y * z;
    assert!(
        (ik.sampling_volume() - expected).abs() < 1e-12,
        "got {}, expected {expected}",
        ik.sampling_volume()
    );
}

#[test]
fn sampling_volume_multiplies_rotation_vector_tolerances() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        UnitQuaternion::identity(),
        OrientationTolerance::RotationVector {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        },
        1.0,
    )
    .unwrap();

    let sampler = IkSamplingPose {
        position_constraint: None,
        orientation_constraint: Some(oc),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let expected = 0.1 * 0.2 * 0.3;
    assert!(
        (ik.sampling_volume() - expected).abs() < 1e-12,
        "got {}, expected {expected}",
        ik.sampling_volume()
    );
}

// ---- samplePose: a position-only constraint, an orientation-only constraint ----

#[test]
fn sample_pose_position_only_lands_inside_the_region_every_time() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    // Off-origin and small enough that "always returns the origin" (a
    // plausible default-value bug) would fail this immediately.
    let center = Vector3::new(0.4, 0.2, 0.1);
    let radius = 0.05;
    let pose = Isometry3::from_parts(center.into(), UnitQuaternion::identity());
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(Shape::Sphere(Sphere::new(radius).unwrap()), pose)],
        1.0,
    )
    .unwrap();

    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng = ChaCha8Rng::seed_from_u64(1);

    for _ in 0..50 {
        let (pos, quat) = ik
            .sample_pose(&reference, &mut rng, 20)
            .expect("a generously-sized region must always be sampleable");
        assert!(
            (pos - center).norm() <= radius + 1e-9,
            "sampled position {pos:?} is outside the constraint sphere (center {center:?}, radius {radius})"
        );
        assert!(
            (quat.norm() - 1.0).abs() < 1e-9,
            "unconstrained orientation must still be a unit quaternion, got norm {}",
            quat.norm()
        );
    }
}

#[test]
fn sample_pose_orientation_only_stays_within_the_triangle_inequality_angle_bound() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let (x_tol, y_tol, z_tol) = (0.05, 0.04, 0.03);
    let desired = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        desired,
        OrientationTolerance::RotationVector {
            x: x_tol,
            y: y_tol,
            z: z_tol,
        },
        1.0,
    )
    .unwrap();

    let sampler = IkSamplingPose {
        position_constraint: None,
        orientation_constraint: Some(oc.clone()),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng = ChaCha8Rng::seed_from_u64(2);

    // The angular distance on SO(3) is a bi-invariant metric, so composing
    // three axis-bounded rotations (or, equivalently, a single rotation by
    // the tolerance-bounded rotation vector) can never exceed the sum of
    // the per-axis tolerances — a property of SO(3) itself, not of this
    // port's own `diff` construction, so this bound holds independently of
    // how `sample_pose` actually built `quat`.
    let bound = x_tol + y_tol + z_tol + 1e-9;

    let mut positions = Vec::new();
    for _ in 0..50 {
        let (pos, quat) = ik
            .sample_pose(&reference, &mut rng, 20)
            .expect("no position constraint to fail sampling");
        assert!(
            (quat.norm() - 1.0).abs() < 1e-9,
            "quat must stay a unit quaternion, got norm {}",
            quat.norm()
        );
        let angle = (oc.desired_rotation_matrix().inverse() * quat.to_rotation_matrix()).angle();
        assert!(
            angle <= bound,
            "sampled orientation is {angle} rad from the target, exceeding the {bound} rad bound"
        );
        positions.push(pos);
    }
    assert!(
        positions.windows(2).any(|w| (w[0] - w[1]).norm() > 1e-9),
        "with no position constraint, the sampled link position should vary across calls \
         (forward kinematics of a freshly randomized state), not stay fixed"
    );
}

// ---- sample: the restart path when IK never converges, and seed/solution ordering ----

struct FakeTip {
    tip: String,
}

impl FakeTip {
    fn new(tip: &str) -> Self {
        Self {
            tip: tip.to_owned(),
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
        "world"
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
        panic!("FakeTip is only used to satisfy IkConstraintSampler::new's frame checks")
    }
}

/// A solver whose `solve_with_options` never converges, counting calls so
/// the test can confirm the outer retry loop actually runs the full
/// `max_attempts` budget rather than giving up early.
struct NoSolutionSolver {
    joint_names: Vec<String>,
    calls: Cell<u32>,
}

impl KinematicsSolver for NoSolutionSolver {
    fn group_name(&self) -> &str {
        "fake"
    }
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn base_frame(&self) -> &str {
        "world"
    }
    fn tip_frame(&self) -> &str {
        "panda_link8"
    }
    fn solve_with_options(
        &mut self,
        _seed: &[f64],
        _target: &Isometry3,
        _options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        self.calls.set(self.calls.get() + 1);
        None
    }
}

#[test]
fn sample_exhausts_max_attempts_when_ik_never_converges() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    // Huge relative to panda's ~0.85 m reach: sampling a point inside this
    // region must always succeed, so every attempt reaches `solve_with_options`.
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(10.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    let mut solver = NoSolutionSolver {
        joint_names: PANDA_ARM_JOINTS.iter().map(|s| s.to_string()).collect(),
        calls: Cell::new(0),
    };
    let ik = IkConstraintSampler::new(&model, &solver, sampler).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(3);

    let max_attempts = 7;
    let ok = ik.sample(&mut state, &mut solver, &mut rng, max_attempts);
    assert!(
        !ok,
        "IK that never converges must make sample() return false"
    );
    assert_eq!(
        solver.calls.get(),
        max_attempts,
        "the restart loop must run the full max_attempts budget, not stop early"
    );
}

/// A solver that hands the seed straight back as the "solution" — any
/// mismatch between how `sample` reads the seed and how it writes the
/// solution back would show up as a joint value silently landing on the
/// wrong name.
struct EchoSolver {
    joint_names: Vec<String>,
}

impl KinematicsSolver for EchoSolver {
    fn group_name(&self) -> &str {
        "fake"
    }
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn base_frame(&self) -> &str {
        "world"
    }
    fn tip_frame(&self) -> &str {
        "panda_link8"
    }
    fn solve_with_options(
        &mut self,
        seed: &[f64],
        _target: &Isometry3,
        _options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        Some(seed.to_vec())
    }
}

#[test]
fn sample_round_trips_seed_to_solution_by_name_even_with_a_reversed_joint_order() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    // Big enough that panda_link8's forward kinematics, for any joint
    // configuration this test sets, is certain to land inside — the point
    // here is the per-joint value round trip, not region containment.
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Sphere(Sphere::new(10.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    // Deliberately the reverse of PANDA_ARM_JOINTS / the model's own order.
    let mut solver = EchoSolver {
        joint_names: PANDA_ARM_JOINTS
            .iter()
            .rev()
            .map(|s| s.to_string())
            .collect(),
    };
    let ik = IkConstraintSampler::new(&model, &solver, sampler).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let original: Vec<f64> = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        .iter()
        .zip(PANDA_ARM_JOINTS)
        .map(|(&v, name)| {
            state.set_variable_position(name, v).unwrap();
            v
        })
        .collect();

    let mut rng = ChaCha8Rng::seed_from_u64(4);
    // `attempt == 0` reads `state`'s current values as the seed, so this
    // must succeed on the very first (and only) attempt.
    let ok = ik.sample(&mut state, &mut solver, &mut rng, 1);
    assert!(
        ok,
        "EchoSolver always converges and the region always contains panda_link8"
    );

    for (name, expected) in PANDA_ARM_JOINTS.iter().zip(&original) {
        let actual = state.variable_position(name).unwrap();
        assert!(
            (actual - expected).abs() < 1e-12,
            "{name}: expected the echoed-back value {expected}, got {actual} — \
             a name/order mismatch between seed and solution would show up here"
        );
    }
}

// ---- Full pipeline: real geometry, a real solver, a genuine transform_ik ----

#[test]
fn sample_with_a_real_solver_converges_on_a_position_and_orientation_target() {
    let model = panda_model();
    let params = SolverParams::default();
    let mut solver =
        NewtonRaphsonSolver::new(&model, "panda_arm", &params).expect("panda_arm is a chain");
    assert_ne!(
        solver.base_frame(),
        model.model_frame(),
        "this test is only meaningful if transform_ik actually fires \
         (panda_arm's chain base is panda_link0, the model frame is world)"
    );

    // A known-reachable target: FK of a fixed joint configuration.
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
    let sampling_pose = IkSamplingPose {
        position_constraint: Some(pc.clone()),
        orientation_constraint: Some(oc.clone()),
    };
    let ik = IkConstraintSampler::new(&model, &solver, sampling_pose).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(5);

    let ok = ik.sample(&mut state, &mut solver, &mut rng, 30);
    assert!(
        ok,
        "newton-raphson must find a solution near a reachable target within 30 attempts"
    );

    let posed = state.update();
    let position_result = pc.decide(&posed);
    let orientation_result = oc.decide(&posed);
    assert!(
        position_result.satisfied,
        "the accepted solution must independently satisfy the position constraint: {position_result:?}"
    );
    assert!(
        orientation_result.satisfied,
        "the accepted solution must independently satisfy the orientation constraint: {orientation_result:?}"
    );
}
