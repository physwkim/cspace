// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The moveit-rs side of the differential comparison: reshapes a built
//! `moveit_model::RobotModel` into the wire [`ModelInfo`] and drives
//! `moveit_state::RobotState` to answer [`fk`], mirroring the oracle's own
//! `modelInfo()`/`fk()` in `tools/moveit-oracle/src/oracle.cpp` field for
//! field so a disagreement here is a port defect, not a protocol mismatch.

use std::collections::BTreeMap;

use moveit_constraints::{
    Constraint, JointConstraint, KinematicConstraintSet, OrientationConstraint,
    OrientationTolerance, PositionConstraint, SensorSpec, TargetSpec, VisibilityConstraint,
    VisibilityCriteria,
};
use moveit_geometry::{
    Cuboid, Cylinder, Isometry3, Mesh, Rotation3, Shape, Sphere, Transforms, UnitQuaternion,
    Vector3,
};
use moveit_model::RobotModel;
use moveit_state::RobotState;
use nalgebra::{Matrix3, Quaternion, Translation3};

use crate::protocol::{
    ConstraintResult, ConstraintsResult, ConstraintsSpec, FkResult, JacobianResult, JointDetail,
    Mimic, ModelInfo, OrientationToleranceSpec, ShapeSpec,
};

/// Row-major 4x4, matching the oracle's `toRowMajor4x4`. `pub(crate)`: also
/// used by `main.rs`'s constraint-case generator to turn a computed pose into
/// the wire format `ConstraintsSpec` carries.
pub(crate) fn to_row_major_4x4(transform: &Isometry3) -> [f64; 16] {
    let m = transform.to_homogeneous();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = m[(r, c)];
        }
    }
    out
}

/// `RobotModel`'s own facts, reshaped into the wire [`ModelInfo`] the oracle
/// answers with. Infallible: everything a [`ModelInfo`] needs is already
/// public on a built `RobotModel`, so there is no failure path to report.
pub fn model_info(model: &RobotModel) -> ModelInfo {
    let joint_details = model
        .joint_models()
        .map(|joint| JointDetail {
            name: joint.name().to_owned(),
            type_name: joint.type_name().to_owned(),
            variable_names: joint.variable_names().to_vec(),
            bounds: joint
                .variable_bounds()
                .iter()
                .map(|b| {
                    (
                        b.min_position.is_finite().then_some(b.min_position),
                        b.max_position.is_finite().then_some(b.max_position),
                    )
                })
                .collect(),
            position_bounded: joint
                .variable_bounds()
                .iter()
                .map(|b| b.position_bounded)
                .collect(),
            mimic: joint.mimic().map(|m| Mimic {
                joint: m.joint_name.clone(),
                multiplier: m.factor,
                offset: m.offset,
            }),
        })
        .collect();

    let groups = model
        .joint_model_group_names()
        .map(|name| {
            let group = model
                .joint_model_group(name)
                .expect("name came from joint_model_group_names");
            (name.to_owned(), group.joint_names().to_vec())
        })
        .collect();

    ModelInfo {
        name: model.name().to_owned(),
        model_frame: model.model_frame().to_owned(),
        root_link: model.root_link_name().to_owned(),
        links: model.link_names().to_vec(),
        joints: model.joint_names().to_vec(),
        joint_details,
        groups,
    }
}

/// Forward kinematics for every link in the model, at `joint_values` layered
/// on top of the model's default positions. Resets to defaults first and
/// applies only the given variables, matching the oracle's own
/// `applyJointValues`: a variable the request omits must come out at its
/// default, never at whatever an earlier case in the same run left behind.
pub fn fk(model: &RobotModel, joint_values: &BTreeMap<String, f64>) -> Result<FkResult, String> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    let posed = state.update();

    let mut link_transforms = BTreeMap::new();
    for link_name in model.link_names() {
        let transform = posed
            .global_link_transform(link_name)
            .map_err(|e| format!("link {link_name}: {e}"))?;
        link_transforms.insert(link_name.clone(), to_row_major_4x4(&transform));
    }
    Ok(FkResult { link_transforms })
}

/// The geometric Jacobian of `group`'s last link at `joint_values` layered
/// on top of the model's default positions, reset-then-apply the same way
/// [`fk`] does. Matches the oracle's own `jacobian()`: a zero reference
/// point, `group->getLinkModels().back()` as the link.
pub fn jacobian(
    model: &RobotModel,
    group: &str,
    joint_values: &BTreeMap<String, f64>,
) -> Result<JacobianResult, String> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    let posed = state.update();

    let m = posed
        .jacobian(group, &Vector3::zeros())
        .map_err(|e| format!("group {group}: {e}"))?;
    let (rows, cols) = m.shape();
    let mut data = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        for c in 0..cols {
            data.push(m[(r, c)]);
        }
    }
    Ok(JacobianResult { rows, cols, data })
}

/// Row-major 4x4, matching the oracle's `fromRowMajor4x4`. Decomposes into a
/// rotation via `UnitQuaternion::from_matrix` rather than trusting the raw
/// 3x3 block directly, the same normalization `world_parity.rs`'s own
/// `isometry_from_row_major` applies for a request built from a wire value
/// rather than computed in-process.
fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::new(
        m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10],
    ));
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_rotation_matrix(&rotation))
}

fn shape_from_spec(spec: &ShapeSpec) -> Result<Shape, String> {
    Ok(match spec {
        ShapeSpec::Sphere { radius } => {
            Shape::Sphere(Sphere::new(*radius).map_err(|e| format!("sphere: {e}"))?)
        }
        ShapeSpec::Box { size } => {
            Shape::Cuboid(Cuboid::new(size[0], size[1], size[2]).map_err(|e| format!("box: {e}"))?)
        }
        ShapeSpec::Cylinder { radius, length } => {
            Shape::Cylinder(Cylinder::new(*radius, *length).map_err(|e| format!("cylinder: {e}"))?)
        }
        ShapeSpec::Mesh {
            vertices,
            triangles,
        } => Shape::Mesh(
            Mesh::new(
                vertices
                    .iter()
                    .map(|v| Vector3::new(v[0], v[1], v[2]))
                    .collect(),
                triangles.clone(),
            )
            .map_err(|e| format!("mesh: {e}"))?,
        ),
    })
}

fn sensor_view_direction_from_spec(
    name: &str,
) -> Result<moveit_constraints::SensorViewDirection, String> {
    match name {
        "sensor_x" => Ok(moveit_constraints::SensorViewDirection::SensorX),
        "sensor_y" => Ok(moveit_constraints::SensorViewDirection::SensorY),
        "sensor_z" => Ok(moveit_constraints::SensorViewDirection::SensorZ),
        other => Err(format!("unknown sensor_view_direction {other:?}")),
    }
}

/// Builds every constraint in `spec` against `model`/`tf`, in the same
/// joint/position/orientation/visibility order
/// `KinematicConstraintSet::add(msg, tf)` walks internally (see
/// `ConstraintsSpec`'s doc comment), then evaluates the resulting set at
/// `joint_values` layered on top of the model's default positions, reset-
/// then-apply the same way [`fk`]/[`jacobian`] do.
///
/// A `VisibilityConstraintSpec` with `target_radius` set runs the full
/// cone-vs-robot collision check (`moveit-constraints`' own
/// `VisibilityConstraint::decide`), not just the view/range-angle checks.
pub fn constraints(
    model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    spec: &ConstraintsSpec,
) -> Result<ConstraintsResult, String> {
    let tf = Transforms::new(model.model_frame()).map_err(|e| format!("Transforms::new: {e}"))?;

    let mut set = KinematicConstraintSet::new();

    for jc in &spec.joint_constraints {
        let c = JointConstraint::new(
            model,
            &jc.joint_name,
            jc.position,
            jc.tolerance_above,
            jc.tolerance_below,
            jc.weight,
        )
        .map_err(|e| format!("joint constraint {:?}: {e}", jc.joint_name))?;
        set.push(Constraint::Joint(c));
    }

    for pc in &spec.position_constraints {
        let regions: Vec<(Shape, Isometry3)> = pc
            .regions
            .iter()
            .map(|r| Ok((shape_from_spec(&r.shape)?, isometry_from_row_major(&r.pose))))
            .collect::<Result<_, String>>()?;
        let c = PositionConstraint::new(
            model,
            &tf,
            &pc.link_name,
            &pc.frame_id,
            Vector3::new(
                pc.target_point_offset[0],
                pc.target_point_offset[1],
                pc.target_point_offset[2],
            ),
            &regions,
            pc.weight,
        )
        .map_err(|e| format!("position constraint {:?}: {e}", pc.link_name))?;
        set.push(Constraint::Position(c));
    }

    for oc in &spec.orientation_constraints {
        let orientation = UnitQuaternion::from_quaternion(Quaternion::new(
            oc.orientation[3],
            oc.orientation[0],
            oc.orientation[1],
            oc.orientation[2],
        ));
        let tolerance = match oc.tolerance {
            OrientationToleranceSpec::XyzEuler { x, y, z } => {
                OrientationTolerance::XyzEuler { x, y, z }
            }
            OrientationToleranceSpec::RotationVector { x, y, z } => {
                OrientationTolerance::RotationVector { x, y, z }
            }
        };
        let c = OrientationConstraint::new(
            model,
            &tf,
            &oc.link_name,
            &oc.frame_id,
            orientation,
            tolerance,
            oc.weight,
        )
        .map_err(|e| format!("orientation constraint {:?}: {e}", oc.link_name))?;
        set.push(Constraint::Orientation(c));
    }

    for vc in &spec.visibility_constraints {
        let c = VisibilityConstraint::new(
            model,
            &tf,
            SensorSpec {
                frame_id: &vc.sensor_frame_id,
                pose: isometry_from_row_major(&vc.sensor_pose),
                view_direction: sensor_view_direction_from_spec(&vc.sensor_view_direction)?,
            },
            TargetSpec {
                frame_id: &vc.target_frame_id,
                pose: isometry_from_row_major(&vc.target_pose),
            },
            vc.cone_sides,
            VisibilityCriteria {
                target_radius: vc.target_radius,
                max_view_angle: vc.max_view_angle,
                max_range_angle: vc.max_range_angle,
            },
            vc.weight,
        )
        .map_err(|e| format!("visibility constraint: {e}"))?;
        set.push(Constraint::Visibility(c));
    }

    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    let posed = state.update();

    let results = set
        .decide_each(&posed)
        .into_iter()
        .map(|r| ConstraintResult {
            satisfied: r.satisfied,
            distance: r.distance,
        })
        .collect();

    Ok(ConstraintsResult { results })
}
