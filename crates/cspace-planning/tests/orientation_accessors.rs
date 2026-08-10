// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins for [`OrientationConstraint::desired_rotation_matrix_in_ref_frame`]
//! and [`OrientationConstraint::desired_rotation_matrix`] — the two
//! accessors round 6 left unported and round 8 confirmed are the last thing
//! `IKConstraintSampler::samplePose` needs from this type. Each expected
//! matrix below is hand-multiplied from the textbook `Rx`/`Rz` axis-angle
//! formulas, not re-derived by calling this port's own [`UnitQuaternion`]/
//! [`Rotation3`] conversion a second time — the same "pin against an
//! external value, not a round-trip through this port's own constructor"
//! rule the round-8 metrics sweep (`c2ff170`) used.

use std::f64::consts::PI;
use std::fs;

use cspace_core::geometry::{Isometry3, Rotation3, Transforms, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_planning::constraints::{OrientationConstraint, OrientationTolerance};
use nalgebra::Translation3;

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

fn assert_matrix_eq(actual: Rotation3, expected: [[f64; 3]; 3], msg: &str) {
    for (row, expected_row) in expected.iter().enumerate() {
        for (col, e) in expected_row.iter().enumerate() {
            let a = actual.matrix()[(row, col)];
            assert!(
                (a - e).abs() < 1e-12,
                "{msg}: [{row}][{col}] = {a}, expected {e} (full actual = {actual:?})"
            );
        }
    }
}

/// A 90-degree rotation about Z, `Rz(pi/2)`, written out from the textbook
/// `[[cos,-sin,0],[sin,cos,0],[0,0,1]]` formula rather than computed by this
/// port.
const RZ_90: [[f64; 3]; 3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];

/// `Rx(pi/2) * Rz(pi/2)`, hand-multiplied from the textbook `Rx`/`Rz`
/// formulas (row-by-column, `Rx` rows `[1,0,0]`, `[0,0,-1]`, `[0,1,0]`
/// against `Rz`'s columns `[0,1,0]`, `[-1,0,0]`, `[0,0,1]`).
const RX_90_TIMES_RZ_90: [[f64; 3]; 3] = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]];

/// [`OrientationTarget::Mobile`] never transforms `desired_R_in_frame_id_`
/// (upstream `kinematic_constraint.cpp:633`, the `else` branch simply
/// assigns `desired_rotation_matrix_ = Eigen::Matrix3d(q)` with no
/// transform applied) — both accessors must return the same, untransformed,
/// hand-known matrix.
#[test]
fn mobile_frame_leaves_both_accessors_at_the_untransformed_quaternion() {
    let model = panda_model();
    let tf = Transforms::new("world").unwrap();
    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);

    let c = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "panda_link1",
        q,
        OrientationTolerance::RotationVector {
            x: 0.1,
            y: 0.1,
            z: 0.1,
        },
        1.0,
    )
    .unwrap();
    assert!(
        c.mobile_reference_frame(),
        "panda_link1 is not registered in tf and is not the model frame, so this must be Mobile"
    );

    assert_matrix_eq(
        c.desired_rotation_matrix_in_ref_frame(),
        RZ_90,
        "desired_rotation_matrix_in_ref_frame",
    );
    assert_matrix_eq(
        c.desired_rotation_matrix(),
        RZ_90,
        "desired_rotation_matrix (Mobile branch)",
    );
}

/// [`OrientationTarget::Fixed`] composes the registered transform with the
/// header-frame quaternion (upstream `kinematic_constraint.cpp:624-627`,
/// `tf.transformQuaternion` then `Eigen::Matrix3d(q)`) — the ref-frame
/// accessor still returns the untransformed value, but
/// `desired_rotation_matrix` must return the transform-composed one.
#[test]
fn fixed_frame_composes_the_registered_transform_into_desired_rotation_matrix() {
    let model = panda_model();
    let mut tf = Transforms::new("world").unwrap();
    let sensor_rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 2.0);
    tf.set_transform(
        Isometry3::from_parts(Translation3::identity(), sensor_rotation),
        "sensor",
    )
    .unwrap();

    let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
    let c = OrientationConstraint::new(
        &model,
        &tf,
        "panda_link8",
        "sensor",
        q,
        OrientationTolerance::RotationVector {
            x: 0.1,
            y: 0.1,
            z: 0.1,
        },
        1.0,
    )
    .unwrap();
    assert!(
        !c.mobile_reference_frame(),
        "sensor is registered in tf, so this must be Fixed"
    );

    assert_matrix_eq(
        c.desired_rotation_matrix_in_ref_frame(),
        RZ_90,
        "desired_rotation_matrix_in_ref_frame is unaffected by the fixed transform",
    );
    assert_matrix_eq(
        c.desired_rotation_matrix(),
        RX_90_TIMES_RZ_90,
        "desired_rotation_matrix (Fixed branch)",
    );
}
