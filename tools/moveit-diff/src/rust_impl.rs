// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The moveit-rs side of the differential comparison: reshapes a built
//! `moveit_model::RobotModel` into the wire [`ModelInfo`] and drives
//! `moveit_state::RobotState` to answer [`fk`], mirroring the oracle's own
//! `modelInfo()`/`fk()` in `tools/moveit-oracle/src/oracle.cpp` field for
//! field so a disagreement here is a port defect, not a protocol mismatch.

use std::collections::BTreeMap;

use moveit_geometry::{Isometry3, Vector3};
use moveit_kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolverParams};
use moveit_model::RobotModel;
use moveit_state::{Posed, RobotState};

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

/// `group`'s tip link pose expressed in `group`'s own base-link frame --
/// `root_pose_world.inverse() * tip_pose_world` -- the frame
/// [`moveit_kinematics::KinematicsSolver::solve`] takes its target in.
/// Rebuilt here from only public `RobotModel`/`Posed` API, exactly matching
/// `tests/ik_fk_roundtrip.rs`'s own `chain_relative_pose` helper in
/// `crates/moveit-kinematics`, since `moveit_kinematics::chain::ChainInfo`
/// is private to that crate.
fn chain_relative_pose(
    model: &RobotModel,
    group_name: &str,
    posed: &Posed,
) -> Result<Isometry3, String> {
    let group = model
        .joint_model_group(group_name)
        .map_err(|e| format!("group {group_name}: {e}"))?;
    let tip_name = group
        .link_names()
        .last()
        .ok_or_else(|| format!("group {group_name} has no links"))?;
    let tip_pose_world = posed
        .global_link_transform(tip_name)
        .map_err(|e| format!("link {tip_name}: {e}"))?;

    let root_joint = group.joint_indices()[0];
    let root_link = model
        .link_models()
        .iter()
        .find(|l| l.parent_joint_index() == root_joint)
        .and_then(|l| l.parent_link_index());

    Ok(match root_link {
        Some(root_link) => {
            let root_pose_world = posed.global_link_transform_at(root_link);
            root_pose_world.inverse() * tip_pose_world
        }
        None => tip_pose_world,
    })
}

/// Everything one [`crate::protocol::Op::Ik`] case needs on the moveit-rs
/// side: whether `NewtonRaphsonSolver` converged, the seed it started from
/// (so the caller can flag a degenerate "returned its seed" pass), and --
/// when it converged -- how far `FK(solution)` lands from the target pose
/// it was asked to reach.
pub struct IkOutcome {
    /// [`moveit_kinematics::KinematicsSolver::joint_names`] order.
    pub joint_names: Vec<String>,
    /// The deterministic, bounds-midpoint seed this side computed -- see
    /// [`crate::protocol::Op::Ik`]'s doc comment for why this never needs
    /// to cross the wire.
    pub seed: Vec<f64>,
    /// The solved joint values, [`IkOutcome::joint_names`] order. `None`
    /// when the solver did not converge.
    pub solution: Option<Vec<f64>>,
    /// `(FK(solution)`'s translation error, rotation error)` against the
    /// target pose, present only when [`IkOutcome::solution`] is.
    pub errors: Option<(f64, f64)>,
}

/// Drives `NewtonRaphsonSolver` -- the direct port of upstream's own (only)
/// solver, `ChainIkSolverVelMimicSVD` -- over a target pose built from
/// `joint_values` the same way [`fk`] builds one, restricted to `group`'s
/// own chain-relative frame. See [`crate::protocol::Op::Ik`]'s doc comment
/// for the full rationale.
pub fn ik(
    model: &RobotModel,
    group: &str,
    joint_values: &BTreeMap<String, f64>,
    position_only: bool,
) -> Result<IkOutcome, String> {
    let params = SolverParams {
        position_only,
        ..Default::default()
    };
    let mut solver = NewtonRaphsonSolver::new(model, group, &params)
        .map_err(|e| format!("constructing NewtonRaphsonSolver for {group}: {e}"))?;
    let joint_names = solver.joint_names().to_vec();

    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    let posed = state.update();
    let target = chain_relative_pose(model, group, &posed)?;

    let seed: Vec<f64> = joint_names
        .iter()
        .map(|name| {
            let bounds = &model
                .joint_model(name)
                .expect("solver's own joint name is a real model joint")
                .variable_bounds()[0];
            (bounds.min_position + bounds.max_position) / 2.0
        })
        .collect();

    let solution = solver.solve(&seed, &target);
    let errors = match &solution {
        Some(sol) => {
            let mut solved_state = RobotState::new(model);
            solved_state.set_to_default_values();
            for (name, &value) in joint_names.iter().zip(sol) {
                solved_state
                    .set_variable_position(name, value)
                    .map_err(|e| format!("setting solved {name}: {e}"))?;
            }
            let solved_posed = solved_state.update();
            let solved_pose = chain_relative_pose(model, group, &solved_posed)?;
            let translation_error =
                (solved_pose.translation.vector - target.translation.vector).norm();
            let rotation_error = (target.rotation.inverse() * solved_pose.rotation).angle();
            Some((translation_error, rotation_error))
        }
        None => None,
    };

    Ok(IkOutcome {
        joint_names,
        seed,
        solution,
        errors,
    })
}
