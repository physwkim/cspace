// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The moveit-rs side of the differential comparison: reshapes a built
//! `moveit_model::RobotModel` into the wire [`ModelInfo`] and drives
//! `moveit_state::RobotState` to answer [`fk`], mirroring the oracle's own
//! `modelInfo()`/`fk()` in `tools/moveit-oracle/src/oracle.cpp` field for
//! field so a disagreement here is a port defect, not a protocol mismatch.

use std::collections::BTreeMap;

use moveit_geometry::{Isometry3, Vector3};
use moveit_model::RobotModel;
use moveit_state::RobotState;

use crate::protocol::{FkResult, JacobianResult, JointDetail, Mimic, ModelInfo};

/// Row-major 4x4, matching the oracle's `toRowMajor4x4`.
fn to_row_major_4x4(transform: &Isometry3) -> [f64; 16] {
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
