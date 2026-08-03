// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The moveit-rs side of the differential comparison: reshapes a built
//! `moveit_model::RobotModel` into the wire [`ModelInfo`] and drives
//! `moveit_state::RobotState` to answer [`fk`], mirroring the oracle's own
//! `modelInfo()`/`fk()` in `tools/moveit-oracle/src/oracle.cpp` field for
//! field so a disagreement here is a port defect, not a protocol mismatch.

use std::collections::BTreeMap;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, ParryCollisionEnv,
};
use moveit_constraints::{
    Constraint, JointConstraint, KinematicConstraintSet, OrientationConstraint,
    OrientationTolerance, PositionConstraint, SensorSpec, TargetSpec, VisibilityConstraint,
    VisibilityCriteria,
};
use moveit_geometry::{
    Cuboid, Cylinder, Isometry3, Mesh, Rotation3, Shape, Sphere, Transforms, UnitQuaternion,
    Vector3,
};
use moveit_kinematics::{KinematicsSolver, NewtonRaphsonSolver, SolverParams};
use moveit_model::RobotModel;
use moveit_state::{Posed, RobotState};
use nalgebra::{Matrix3, Quaternion, Translation3};

use crate::protocol::{
    CollisionCheckResult, ConstraintResult, ConstraintsResult, ConstraintsSpec, FkResult,
    JacobianResult, JointDetail, Mimic, ModelInfo, OrientationToleranceSpec, ShapeSpec,
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

/// Self- and robot-collision, plus signed self/robot distance, at
/// `joint_values` layered on top of the model's default positions,
/// reset-then-apply the same way [`fk`] does. Matches the oracle's own
/// `collision()`: default [`CollisionRequest`]s for the boolean checks, and
/// `enable_signed_distance = true` with `acm` set for both distance queries.
/// Contact/nearest-point coordinates are not read here -- PORTING-PLAN.md
/// §4.5 excludes them from Phase 3's completion condition.
pub fn collision(
    model: &RobotModel,
    env: &ParryCollisionEnv,
    acm: &AllowedCollisionMatrix,
    joint_values: &BTreeMap<String, f64>,
) -> Result<CollisionCheckResult, String> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    let posed = state.update();

    let self_result =
        env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(acm));
    let robot_result =
        env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(acm));

    let distance_request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(acm),
        ..DistanceRequest::default()
    };
    let self_distance = env.distance_self(&distance_request, &posed, &[]);
    let robot_distance = env.distance_robot(&distance_request, &posed, &[]);

    Ok(CollisionCheckResult {
        self_collision: self_result.collision,
        self_distance: self_distance.minimum_distance.distance,
        robot_collision: robot_result.collision,
        robot_distance: robot_distance.minimum_distance.distance,
    })
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
///
/// Built once for a whole run and reused across every case, because
/// `NewtonRaphsonSolver` owns the `ChaCha8Rng` its random restarts draw
/// from. Constructing one per case reseeds that stream to `DEFAULT_SEED`
/// every time, so all N cases would retry from the *same* `max_restarts`
/// configurations -- a strictly poorer search than upstream's, whose
/// `KDLKinematicsPlugin` is initialized once and whose `random_number_
/// generator_` therefore keeps advancing across the run. That asymmetry,
/// not the solve step, is what a restart-enabled success-rate comparison
/// would otherwise be measuring.
pub struct IkSolver<'m> {
    model: &'m RobotModel,
    group: String,
    solver: NewtonRaphsonSolver,
    joint_names: Vec<String>,
    /// The bounds-midpoint seed, identical for every case of a given group
    /// (bounds are group-constant), so it is computed once here.
    seed: Vec<f64>,
}

impl<'m> IkSolver<'m> {
    pub fn new(
        model: &'m RobotModel,
        group: &str,
        position_only: bool,
        max_restarts: u32,
    ) -> Result<Self, String> {
        let params = SolverParams {
            position_only,
            max_restarts: max_restarts as usize,
            ..Default::default()
        };
        let solver = NewtonRaphsonSolver::new(model, group, &params)
            .map_err(|e| format!("constructing NewtonRaphsonSolver for {group}: {e}"))?;
        let joint_names = solver.joint_names().to_vec();
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
        Ok(Self {
            model,
            group: group.to_owned(),
            solver,
            joint_names,
            seed,
        })
    }

    /// One case: build the target from `joint_values`, solve, and measure
    /// the solution's own FK error against that target.
    pub fn solve_case(
        &mut self,
        joint_values: &BTreeMap<String, f64>,
    ) -> Result<IkOutcome, String> {
        let model = self.model;
        let group = self.group.as_str();

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, &value) in joint_values {
            state
                .set_variable_position(name, value)
                .map_err(|e| format!("setting {name}: {e}"))?;
        }
        let posed = state.update();
        let target = chain_relative_pose(model, group, &posed)?;

        let solution = self.solver.solve(&self.seed, &target);
        let errors = match &solution {
            Some(sol) => {
                let mut solved_state = RobotState::new(model);
                solved_state.set_to_default_values();
                for (name, &value) in self.joint_names.iter().zip(sol) {
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
            joint_names: self.joint_names.clone(),
            seed: self.seed.clone(),
            solution,
            errors,
        })
    }
}
