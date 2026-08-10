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
//! `panda.urdf`/`panda.srdf` (copied from `cspace-state`'s fixtures,
//! already oracle-verified — see
//! `crates/cspace-core/tests/fixtures/model/panda_model_info.json`) supply a
//! real model.

use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::fs;
use std::rc::Rc;

use cspace_constraints::{
    ConstraintSampler, IkConstraintSampler, IkConstraintSamplerAdapter, IkSamplingPose,
    OrientationConstraint, OrientationTolerance, PositionConstraint,
};
use cspace_core::geometry::{
    Cuboid, Isometry3, Rotation3, Shape, Sphere, Transforms, UnitQuaternion, Vector3,
};
use cspace_core::kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolveOptions, SolverParams};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use rand::{RngExt, SeedableRng};
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

// ---- samplePose: pinning the four lines a loose bound cannot distinguish ----
//
// The tests above check containment and a triangle-inequality bound — both
// satisfied by a sign or order inversion in `sample_pose`'s pose arithmetic.
// These four pin the exact returned value against an expected value derived
// independently of `sample_pose`: the same seeded RNG draws are replayed
// through the same already-tested primitives `sample_pose` itself calls
// (`ConstraintRegion`/`Body::sample_point_inside` for position,
// `Rng::random`/`random_range` for the per-axis angles — the u01-to-angle
// formula itself was already confirmed correct line-by-line against
// upstream and is not one of the four disputed lines), then combined by
// hand using the *correct* order/sign for whichever of the four lines this
// test targets. A test's expected value only agrees with `sample_pose`'s
// actual return if that one line is right.

/// Replays the position half of `sample_pose`'s RNG draws for a
/// single-region `Cuboid` constraint (`regions.len() == 1`, so the region
/// index draw `(i + k) % regions.len()` always selects region 0 regardless
/// of `k`'s value) and returns the sampled point before any mobile-frame or
/// link-offset adjustment. `Body::sample_point_inside` for a `Cuboid` always
/// succeeds on its first draw (`bodies.rs`: three `uniform()` calls, no
/// rejection loop), so this consumes exactly the same draws `sample_pose`
/// consumes for the position half, landing `rng` at the same point in the
/// stream `sample_pose` would reach before drawing the orientation angles.
fn replay_cuboid_position(pc: &PositionConstraint, rng: &mut ChaCha8Rng) -> Vector3 {
    let _k: usize = rng.random_range(0..1usize);
    let region = &pc.constraint_regions()[0];
    let body = region.body.clone_at(region.pose);
    body.sample_point_inside(1, &mut |lo, hi| rng.random_range(lo..hi))
        .expect("a Cuboid region never rejects its first sample")
}

/// Replays the three `2.0 * (u01 - 0.5) * (tol - eps)` draws `sample_pose`
/// makes for the orientation delta, in the same order (x, then y, then z).
/// This formula itself is not one of the four disputed lines (round 9
/// already confirmed it against upstream), so reproducing it here is
/// establishing known input to the four tests below, not begging the
/// question of what they each check.
fn replay_orientation_angles(rng: &mut ChaCha8Rng, x_tol: f64, y_tol: f64, z_tol: f64) -> Vector3 {
    let eps = f64::EPSILON;
    let u1: f64 = rng.random();
    let u2: f64 = rng.random();
    let u3: f64 = rng.random();
    Vector3::new(
        2.0 * (u1 - 0.5) * (x_tol - eps),
        2.0 * (u2 - 0.5) * (y_tol - eps),
        2.0 * (u3 - 0.5) * (z_tol - eps),
    )
}

/// The *correct* X, then Y, then Z axis-angle composition — independent of
/// `sample_pose`'s own `UnitQuaternion::from_axis_angle` chain in that it is
/// built from `Rotation3` matrices multiplied directly, not by calling into
/// `ik_sampler.rs` at all.
fn expected_xyz_euler_diff(angles: Vector3) -> UnitQuaternion {
    let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), angles.x);
    let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), angles.y);
    let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), angles.z);
    UnitQuaternion::from_rotation_matrix(&(rx * ry * rz))
}

#[test]
fn sample_pose_xyz_euler_order_kills_a_zyx_swap() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    // Three distinct, well-separated tolerances: X*Y*Z and Z*Y*X only agree
    // when the three per-axis rotations happen to commute, which generic
    // distinct nonzero angles about three different axes do not.
    let (x_tol, y_tol, z_tol) = (0.30, 0.15, 0.22);
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        UnitQuaternion::identity(),
        OrientationTolerance::XyzEuler {
            x: x_tol,
            y: y_tol,
            z: z_tol,
        },
        1.0,
    )
    .unwrap();
    let sampler = IkSamplingPose {
        position_constraint: Some(pc.clone()),
        orientation_constraint: Some(oc),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng_real = ChaCha8Rng::seed_from_u64(101);
    let mut rng_shadow = rng_real.clone();

    let (_actual_pos, actual_quat) = ik.sample_pose(&reference, &mut rng_real, 1).unwrap();

    replay_cuboid_position(&pc, &mut rng_shadow);
    let angles = replay_orientation_angles(&mut rng_shadow, x_tol, y_tol, z_tol);
    // desired == identity, so `desired_rotation_matrix() * diff == diff`.
    let expected_quat = expected_xyz_euler_diff(angles);

    assert!(
        actual_quat.angle_to(&expected_quat) < 1e-9,
        "X*Y*Z composition mismatch: got {actual_quat:?}, expected {expected_quat:?} \
         (angle between them: {} rad)",
        actual_quat.angle_to(&expected_quat)
    );
}

#[test]
fn sample_pose_rotation_vector_transpose_kills_dropping_the_transpose() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    // A desired rotation about a single coordinate axis is not symmetric
    // for any angle other than 0 or pi, so its transpose is a genuinely
    // different matrix from itself — the transpose is not a no-op here.
    let desired = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    let (x_tol, y_tol, z_tol) = (0.05, 0.04, 0.06);
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
    // Fixed frame, world == identity in `tf`, so `desired_rotation_matrix()`
    // and `desired_rotation_matrix_in_ref_frame()` both equal `desired`'s
    // own rotation matrix here — confirmed from `orientation.rs`: the Fixed
    // branch caches `rotation_matrix_inv = (tf * desired).inverse()`, and
    // `tf` here is the identity map.
    assert_eq!(oc.desired_rotation_matrix(), desired.to_rotation_matrix());
    assert_eq!(
        oc.desired_rotation_matrix_in_ref_frame(),
        desired.to_rotation_matrix()
    );

    let sampler = IkSamplingPose {
        position_constraint: Some(pc.clone()),
        orientation_constraint: Some(oc.clone()),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng_real = ChaCha8Rng::seed_from_u64(102);
    let mut rng_shadow = rng_real.clone();

    let (_actual_pos, actual_quat) = ik.sample_pose(&reference, &mut rng_real, 1).unwrap();

    replay_cuboid_position(&pc, &mut rng_shadow);
    let angles = replay_orientation_angles(&mut rng_shadow, x_tol, y_tol, z_tol);
    let rotation_vector = oc.desired_rotation_matrix_in_ref_frame().transpose() * angles;
    let expected_diff = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(rotation_vector),
        rotation_vector.norm(),
    );
    let expected_quat = desired * expected_diff;

    assert!(
        actual_quat.angle_to(&expected_quat) < 1e-9,
        "transpose mismatch: got {actual_quat:?}, expected {expected_quat:?} \
         (angle between them: {} rad)",
        actual_quat.angle_to(&expected_quat)
    );
}

/// Builds the rotation-vector sampler used by the two degenerate-axis cases
/// below. `tol` goes on all three axes.
fn rotation_vector_sampler(
    model: &RobotModel,
    tf: &Transforms,
    desired: UnitQuaternion,
    tol: f64,
) -> IkConstraintSampler {
    let pc = PositionConstraint::new(
        model,
        tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let oc = OrientationConstraint::new(
        model,
        tf,
        "panda_link8",
        "world",
        desired,
        OrientationTolerance::RotationVector {
            x: tol,
            y: tol,
            z: tol,
        },
        1.0,
    )
    .unwrap();
    let sampler = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: Some(oc),
    };
    IkConstraintSampler::new(model, &FakeTip::new("panda_link8"), sampler).unwrap()
}

/// `sample_pose` draws each angle as `2 * (u - 0.5) * (tol - f64::EPSILON)`
/// — upstream's own formula, `default_constraint_samplers.cpp:497-504` — so a
/// tolerance of exactly `f64::EPSILON` makes all three angles exactly zero and
/// the rotation vector exactly `[0, 0, 0]`, whatever the RNG draws.
///
/// Upstream then evaluates `AngleAxisd(rotation_vector.norm(),
/// rotation_vector.normalized())` (`:521`). Eigen's `normalized()` returns the
/// input unchanged at zero norm, so that is `AngleAxisd(0, [0,0,0])`, which is
/// the identity rotation — measured in this repo's oracle image at Eigen
/// 3.4.0, where `Isometry3d(AngleAxisd(0, Vector3d::Zero())).linear()` prints
/// exactly `[1 0 0; 0 1 0; 0 0 1]`. The sampled orientation is therefore the
/// desired one.
///
/// `nalgebra::Unit::new_normalize` has no such guard: it divides by the zero
/// norm anyway, and the resulting NaN axis survives `from_axis_angle`'s
/// `axis * sin(angle / 2)` even at angle zero, so the port returned an
/// all-NaN quaternion.
#[test]
fn a_degenerate_rotation_vector_samples_the_desired_orientation_not_nan() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let desired = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    let ik = rotation_vector_sampler(&model, &tf, desired, f64::EPSILON);

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng = ChaCha8Rng::seed_from_u64(102);

    let (_pos, quat) = ik.sample_pose(&reference, &mut rng, 1).unwrap();

    assert!(
        quat.coords.iter().all(|c| c.is_finite()),
        "degenerate rotation vector gave a non-finite orientation: {quat:?}"
    );
    assert!(
        quat.angle_to(&desired) < 1e-12,
        "expected upstream's identity diff, so the desired orientation \
         {desired:?}, got {quat:?} (angle between them: {} rad)",
        quat.angle_to(&desired)
    );
}

/// The demonstrated opposite of
/// [`a_degenerate_rotation_vector_samples_the_desired_orientation_not_nan`]:
/// with an ordinary tolerance the rotation vector is non-zero, both spellings
/// of the normalize agree, and the sample is a finite rotation genuinely away
/// from the desired one. Without this the test above would also pass on a port
/// that had collapsed every rotation-vector sample to the identity.
#[test]
fn an_ordinary_rotation_vector_tolerance_still_samples_away_from_desired() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let desired = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
    let ik = rotation_vector_sampler(&model, &tf, desired, 0.05);

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng = ChaCha8Rng::seed_from_u64(102);

    let (_pos, quat) = ik.sample_pose(&reference, &mut rng, 1).unwrap();

    assert!(
        quat.coords.iter().all(|c| c.is_finite()),
        "ordinary rotation vector gave a non-finite orientation: {quat:?}"
    );
    let angle = quat.angle_to(&desired);
    assert!(
        angle > 1e-9,
        "a 0.05 rad tolerance should move the sample off the desired \
         orientation, but the angle between them was {angle} rad"
    );
}

#[test]
fn sample_pose_mobile_frame_composition_order_kills_a_swap() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    let (x_tol, y_tol, z_tol) = (0.05, 0.04, 0.06);
    // "panda_link4" is not "world" and `tf` only maps "world", so this
    // resolves to `OrientationTarget::Mobile` — `mobile_reference_frame()`
    // is true and `sample_pose` must compose with a fresh frame lookup.
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "panda_link4",
        UnitQuaternion::identity(),
        OrientationTolerance::XyzEuler {
            x: x_tol,
            y: y_tol,
            z: z_tol,
        },
        1.0,
    )
    .unwrap();
    assert!(oc.mobile_reference_frame());

    let sampler = IkSamplingPose {
        position_constraint: Some(pc.clone()),
        orientation_constraint: Some(oc.clone()),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // Away from all-zero joint values, so panda_link4's orientation at the
    // reference state is genuinely non-identity (checked below) and the
    // composition order actually matters.
    for (name, &v) in PANDA_ARM_JOINTS
        .iter()
        .zip(&[0.2, -0.5, 0.3, -1.2, 0.4, 0.9, -0.3])
    {
        state.set_variable_position(name, v).unwrap();
    }
    let reference = state.update();
    let frame_rot = reference.frame_transform("panda_link4").unwrap().rotation;
    assert!(
        frame_rot.angle_to(&UnitQuaternion::identity()) > 1e-3,
        "this test only exercises the composition order if panda_link4's \
         orientation at the reference state is genuinely non-identity"
    );

    let mut rng_real = ChaCha8Rng::seed_from_u64(103);
    let mut rng_shadow = rng_real.clone();

    let (_actual_pos, actual_quat) = ik.sample_pose(&reference, &mut rng_real, 1).unwrap();

    replay_cuboid_position(&pc, &mut rng_shadow);
    let angles = replay_orientation_angles(&mut rng_shadow, x_tol, y_tol, z_tol);
    // desired == identity, so the pre-mobile-composition quat is `diff` alone.
    let pre_mobile_quat = expected_xyz_euler_diff(angles);
    let expected_quat = frame_rot * pre_mobile_quat;

    assert!(
        actual_quat.angle_to(&expected_quat) < 1e-9,
        "mobile-frame composition order mismatch: got {actual_quat:?}, \
         expected {expected_quat:?} (angle between them: {} rad)",
        actual_quat.angle_to(&expected_quat)
    );
}

#[test]
fn sample_pose_link_offset_kills_adding_instead_of_subtracting() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let offset = Vector3::new(0.05, -0.03, 0.02);
    let pc = PositionConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        offset,
        &[(
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
            Isometry3::identity(),
        )],
        1.0,
    )
    .unwrap();
    assert!(pc.has_link_offset());
    let (x_tol, y_tol, z_tol) = (0.30, 0.15, 0.22);
    let oc = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "world",
        UnitQuaternion::identity(),
        OrientationTolerance::XyzEuler {
            x: x_tol,
            y: y_tol,
            z: z_tol,
        },
        1.0,
    )
    .unwrap();
    let sampler = IkSamplingPose {
        position_constraint: Some(pc.clone()),
        orientation_constraint: Some(oc),
    };
    let ik = IkConstraintSampler::new(&model, &FakeTip::new("panda_link8"), sampler).unwrap();

    let mut state = RobotState::new(&model);
    let reference = state.update();
    let mut rng_real = ChaCha8Rng::seed_from_u64(104);
    let mut rng_shadow = rng_real.clone();

    let (actual_pos, actual_quat) = ik.sample_pose(&reference, &mut rng_real, 1).unwrap();

    let raw_pos = replay_cuboid_position(&pc, &mut rng_shadow);
    let angles = replay_orientation_angles(&mut rng_shadow, x_tol, y_tol, z_tol);
    let expected_quat = expected_xyz_euler_diff(angles);
    let expected_pos = raw_pos - expected_quat * offset;

    assert!(
        (actual_pos - expected_pos).norm() < 1e-9,
        "link-offset sign mismatch: got {actual_pos:?}, expected {expected_pos:?} \
         (raw sampled point {raw_pos:?}, offset {offset:?})"
    );
    // Sanity check independent of the sign under test: a `+=` bug would
    // also still leave `actual_quat` matching, so this alone would not
    // have caught it — the position assertion above is what does.
    assert!(actual_quat.angle_to(&expected_quat) < 1e-9);
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
    let ok = ik.sample(&mut state, &mut solver, &mut rng, max_attempts, None);
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
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        let solution = seed.to_vec();
        if let Some(callback) = options.solution_callback.as_deref_mut() {
            if !callback(&solution) {
                return None;
            }
        }
        Some(solution)
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
    let ok = ik.sample(&mut state, &mut solver, &mut rng, 1, None);
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

    let ok = ik.sample(&mut state, &mut solver, &mut rng, 30, None);
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

// ---- group_state_validity_callback: upstream's IK accept/reject hook ----

#[test]
fn sample_rejects_via_group_state_validity_callback_even_when_ik_converges() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
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
    let mut solver = EchoSolver {
        joint_names: PANDA_ARM_JOINTS.iter().map(|s| s.to_string()).collect(),
    };
    let ik = IkConstraintSampler::new(&model, &solver, sampler).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(6);

    let calls = Cell::new(0u32);
    let mut reject_always = |_solution: &[f64]| {
        calls.set(calls.get() + 1);
        false
    };
    let max_attempts = 3;
    let ok = ik.sample(
        &mut state,
        &mut solver,
        &mut rng,
        max_attempts,
        Some(&mut reject_always),
    );
    assert!(
        !ok,
        "EchoSolver always converges, but a callback that always rejects \
         must still make sample() fail"
    );
    assert_eq!(
        calls.get(),
        max_attempts,
        "each of the max_attempts converged candidates must be offered to the callback"
    );
}

#[test]
fn sample_retries_past_group_state_validity_callback_rejections_and_accepts_on_success() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
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
    let mut solver = EchoSolver {
        joint_names: PANDA_ARM_JOINTS.iter().map(|s| s.to_string()).collect(),
    };
    let ik = IkConstraintSampler::new(&model, &solver, sampler).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(7);

    let calls = Cell::new(0u32);
    let mut accept_on_third_call = |_solution: &[f64]| {
        calls.set(calls.get() + 1);
        calls.get() == 3
    };
    let ok = ik.sample(
        &mut state,
        &mut solver,
        &mut rng,
        5,
        Some(&mut accept_on_third_call),
    );
    assert!(
        ok,
        "sample() must retry a callback-rejected candidate rather than giving up early"
    );
    assert_eq!(
        calls.get(),
        3,
        "sample() must stop offering candidates to the callback as soon as one is accepted"
    );
}

#[test]
fn adapter_group_state_validity_callback_gates_the_trait_object_sample_path() {
    let model = panda_model();
    let group = model.joint_model_group("panda_arm").unwrap();
    let tf = Transforms::new("world").unwrap();
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
    let sampling_pose = IkSamplingPose {
        position_constraint: Some(pc),
        orientation_constraint: None,
    };
    let solver: Rc<RefCell<Box<dyn KinematicsSolver>>> =
        Rc::new(RefCell::new(Box::new(EchoSolver {
            joint_names: PANDA_ARM_JOINTS.iter().map(|s| s.to_string()).collect(),
        })));
    let mut adapter =
        IkConstraintSamplerAdapter::new(&model, group, solver, sampling_pose, 3).unwrap();

    let calls = Rc::new(Cell::new(0u32));
    let calls_in_callback = Rc::clone(&calls);
    adapter.set_group_state_validity_callback(Box::new(move |_solution: &[f64]| {
        calls_in_callback.set(calls_in_callback.get() + 1);
        false
    }));

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(8);

    let sampler: &dyn ConstraintSampler = &adapter;
    let ok = sampler.sample(&mut state, &mut rng);
    assert!(
        !ok,
        "EchoSolver always converges, but a callback installed via \
         set_group_state_validity_callback that always rejects must still \
         make the trait object's sample() fail"
    );
    assert_eq!(
        calls.get(),
        3,
        "each of the adapter's baked-in max_attempts candidates must reach the callback"
    );
}
