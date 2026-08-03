// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/utils.hpp
//   moveit_core/kinematic_constraints/src/utils.cpp

//! The portable half of upstream's `kinematic_constraints/utils.{hpp,cpp}`:
//! 11 functions covering 13 of its 15 declarations, none of which touch a
//! ROS node, a parameter server, or `moveit_msgs`/`geometry_msgs`. Not
//! ported here:
//!
//! - `constructConstraints` and its six private `constructConstraint`/
//!   `constructPoseStamped`/`collectConstraints` helpers: these parse an
//!   `rclcpp::Node`'s YAML-sourced parameters, which this core crate has no
//!   access to at all (D1/D2) — a `moveit-ros` concern.
//! - `resolveConstraintFrames`: needs an attached body's frame to resolve to
//!   its parent link, which needs `moveit-state`'s `Posed::frame_transform`
//!   to fall back through `AttachedBody`/subframes, which does not exist yet
//!   (`crates/moveit-state/src/state.rs:872-878` already documents the gap
//!   this port is waiting on — see `PORTING-PLAN.md`'s `p1-fixtures` note on
//!   `AttachedBody`, the type expected to close it).
//!
//! # No `moveit_msgs::msg::Constraints` — "update" reconstructs, not mutates
//!
//! Upstream's `updateXConstraint` functions patch one field of an
//! already-built `moveit_msgs::msg::Constraints` message in place — a design
//! that only works because the message is a mutable, unresolved DTO that a
//! later, separate call turns into the real constraint objects `decide()`
//! evaluates. This crate's constraint types resolve fully at construction
//! (`PORTING-PLAN.md` D1/D2, and this crate's own introducing doc comment)
//! and keep no such mutable form. Each `update_*` function here instead finds
//! the matching entry and replaces it with a freshly reconstructed one,
//! carrying over every field upstream would have left untouched (tolerance,
//! weight, region shape) through a handful of narrow accessors/reconstructors
//! added to [`crate::JointConstraint`]/[`crate::PositionConstraint`]/
//! [`crate::OrientationConstraint`] for exactly this purpose — not a raw
//! field overwrite.
//!
//! # Collapsed overloads
//!
//! Rust has no default arguments, so upstream's thin delegating overloads
//! (e.g. `constructGoalConstraints(state, jmg, tolerance)` delegating to the
//! two-tolerance form with `tolerance_below == tolerance_above`) become one
//! function each here, with the call pattern that reproduces the delegating
//! overload documented instead of a second public function.

use moveit_error::{Error, Result};
use moveit_geometry::{Cuboid, Isometry3, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
use moveit_model::RobotModel;
use moveit_state::Posed;

use crate::orientation::OrientationTolerance;
use crate::{
    Constraint, JointConstraint, KinematicConstraintSet, OrientationConstraint, PositionConstraint,
};

/// `mergeConstraints`: appends `second`'s position/orientation/visibility
/// constraints after `first`'s (each kind segregated, first's before
/// second's, mirroring upstream's four parallel message vectors), and folds
/// joint constraints for the same joint together via
/// `JointConstraint::merged` (private: `crate::joint`'s per-joint merge
/// helper) instead of duplicating them — `first`'s bound
/// wins wherever the two disagree, matching upstream's `a`/`b` argument
/// order. A joint present in only one side is carried over unchanged; two
/// constraints on the same joint whose tolerance windows don't overlap at
/// all are dropped (upstream: logged as an error and discarded).
pub fn merge_constraints(
    first: &KinematicConstraintSet,
    second: &KinematicConstraintSet,
) -> KinematicConstraintSet {
    let mut merged = KinematicConstraintSet::new();

    for c in first.constraints() {
        let Constraint::Joint(a) = c else { continue };
        let matching_b = second.constraints().iter().find_map(|c2| match c2 {
            Constraint::Joint(b) if b.joint_variable_name() == a.joint_variable_name() => Some(b),
            _ => None,
        });
        match matching_b {
            Some(b) => {
                if let Some(m) = a.merged(b) {
                    merged.push(Constraint::Joint(m));
                }
            }
            None => merged.push(Constraint::Joint(a.clone())),
        }
    }
    for c in second.constraints() {
        let Constraint::Joint(b) = c else { continue };
        let in_first = first.constraints().iter().any(|c1| {
            matches!(c1, Constraint::Joint(a) if a.joint_variable_name() == b.joint_variable_name())
        });
        if !in_first {
            merged.push(Constraint::Joint(b.clone()));
        }
    }

    for pick in [first, second] {
        for c in pick.constraints() {
            if matches!(c, Constraint::Position(_)) {
                merged.push(c.clone());
            }
        }
    }
    for pick in [first, second] {
        for c in pick.constraints() {
            if matches!(c, Constraint::Orientation(_)) {
                merged.push(c.clone());
            }
        }
    }
    for pick in [first, second] {
        for c in pick.constraints() {
            if matches!(c, Constraint::Visibility(_)) {
                merged.push(c.clone());
            }
        }
    }

    merged
}

/// `countIndividualConstraints`: this crate's [`KinematicConstraintSet`] is
/// already one flat `Vec<Constraint>` (see this module's doc comment on why
/// there is no `moveit_msgs::msg::Constraints` to sum four segregated
/// vectors of), so this is [`KinematicConstraintSet::len`] under upstream's
/// name.
pub fn count_individual_constraints(constraints: &KinematicConstraintSet) -> usize {
    constraints.len()
}

/// `constructGoalConstraints(state, jmg, tolerance_below, tolerance_above)`,
/// also covering the single-tolerance overload: call with
/// `tolerance_below == tolerance_above` for that form.
///
/// One [`JointConstraint`] per variable of `group_name`, in
/// [`moveit_model::JointModelGroup::variable_names`] order, at `state`'s
/// current position for that variable — matching upstream's
/// `state.copyJointGroupPositions(jmg, vals)` plus `jmg->getVariableNames()`.
///
/// # Errors
///
/// [`Error::UnknownName`] if `group_name` does not name a group. Otherwise
/// whatever [`JointConstraint::new`] returns for one of `group_name`'s
/// variables (unreachable in practice: every name and position here comes
/// from `model`/`state` themselves).
pub fn construct_goal_joint_constraints(
    model: &RobotModel,
    state: &Posed,
    group_name: &str,
    tolerance_below: f64,
    tolerance_above: f64,
) -> Result<KinematicConstraintSet> {
    let jmg = model.joint_model_group(group_name)?;
    let mut goal = KinematicConstraintSet::new();
    for name in jmg.variable_names() {
        let position = state.variable_position(name)?;
        goal.push(Constraint::Joint(JointConstraint::new(
            model,
            name,
            position,
            tolerance_above,
            tolerance_below,
            1.0,
        )?));
    }
    Ok(goal)
}

/// `updateJointConstraints`: for every [`crate::JointConstraint`] already in
/// `constraints`, if its full `joint_variable_name` is active in
/// `group_name`, replace it with a freshly built one at `state`'s current
/// position (same tolerances/weight as before — see this module's doc
/// comment on why "update" reconstructs).
///
/// Reproduces upstream's membership check literally
/// (`utils.cpp:172-192`): it compares the constraint's full name (here,
/// [`JointConstraint::joint_variable_name`], which may carry a
/// `/local_variable` suffix for one variable of a multi-DOF joint) against
/// [`moveit_model::JointModelGroup::active_joint_names`] (plain joint
/// names, one per joint model). For a single-DOF joint the two forms
/// coincide and the check behaves as intended; for a per-variable
/// constraint on a multi-DOF joint the suffixed name can never string-match
/// a plain joint name, so the check fails even when the joint itself is
/// active — an upstream limitation this port reproduces rather than papers
/// over, since fixing it here would silently change which updates succeed
/// relative to upstream.
///
/// Returns `Ok(false)` the moment one constraint's joint is not active,
/// exactly matching upstream's early-return loop: constraints before the
/// mismatch are already updated in `constraints` by the time this returns.
///
/// # Errors
///
/// [`Error::UnknownName`] if `group_name` does not name a group.
pub fn update_joint_constraints(
    constraints: &mut KinematicConstraintSet,
    model: &RobotModel,
    state: &Posed,
    group_name: &str,
) -> Result<bool> {
    let jmg = model.joint_model_group(group_name)?;
    let active = jmg.active_joint_names();

    for c in constraints.constraints_mut() {
        let Constraint::Joint(jc) = c else { continue };
        if !active.iter().any(|name| name == jc.joint_variable_name()) {
            return Ok(false);
        }
        let position = state.variable_position(jc.joint_variable_name())?;
        *jc = JointConstraint::new(
            model,
            jc.joint_variable_name(),
            position,
            jc.joint_tolerance_above(),
            jc.joint_tolerance_below(),
            jc.weight(),
        )?;
    }
    Ok(true)
}

/// `constructGoalConstraints(link_name, pose, tolerance_pos, tolerance_angle)`
/// (sphere-region overload): one [`PositionConstraint`] (a sphere of radius
/// `tolerance_pos` centered on `pose`'s translation, zero target-point
/// offset) plus one [`OrientationConstraint`] targeting `pose`'s rotation
/// with [`OrientationTolerance::RotationVector`] (matching upstream's
/// hardcoded `ROTATION_VECTOR`, set explicitly only for this overload — see
/// [`construct_goal_orientation_constraints`]'s doc comment for the
/// quaternion-only overload, which defaults to Euler angles instead).
///
/// # Errors
///
/// Whatever [`PositionConstraint::new`]/[`OrientationConstraint::new`]
/// return for `link_name`/`frame_id`.
pub fn construct_goal_pose_constraints(
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    pose: Isometry3,
    tolerance_pos: f64,
    tolerance_angle: f64,
) -> Result<KinematicConstraintSet> {
    let mut goal = KinematicConstraintSet::new();
    goal.push(Constraint::Position(PositionConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        Vector3::zeros(),
        &[(Shape::Sphere(Sphere::new(tolerance_pos)?), pose)],
        1.0,
    )?));
    goal.push(Constraint::Orientation(OrientationConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        pose.rotation,
        OrientationTolerance::RotationVector {
            x: tolerance_angle,
            y: tolerance_angle,
            z: tolerance_angle,
        },
        1.0,
    )?));
    Ok(goal)
}

/// `constructGoalConstraints(link_name, pose, tolerance_pos: Vec<f64>,
/// tolerance_angle: Vec<f64>)` (box-region overload): same as
/// [`construct_goal_pose_constraints`], but the position region is a box of
/// dimensions `tolerance_pos` and each orientation axis gets its own
/// tolerance from `tolerance_angle`.
///
/// Upstream builds this by calling the sphere/uniform-tolerance overload
/// first and then overwriting the region shape and, independently, the
/// tolerances — each swap gated by its own `.size() == 3` runtime check, so
/// a caller could in principle keep the sphere default for one half and the
/// box override for the other. This port's `[f64; 3]` parameters are always
/// exactly 3, so both swaps always apply; nothing is left to build a
/// throwaway sphere or Euler-tolerance for, so this constructs the box
/// region and per-axis tolerances directly instead of delegating and then
/// overwriting.
///
/// # Errors
///
/// Whatever [`PositionConstraint::new`]/[`OrientationConstraint::new`]
/// return for `link_name`/`frame_id`.
pub fn construct_goal_pose_constraints_box(
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    pose: Isometry3,
    tolerance_pos: [f64; 3],
    tolerance_angle: [f64; 3],
) -> Result<KinematicConstraintSet> {
    let mut goal = KinematicConstraintSet::new();
    goal.push(Constraint::Position(PositionConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        Vector3::zeros(),
        &[(
            Shape::Cuboid(Cuboid::new(
                tolerance_pos[0],
                tolerance_pos[1],
                tolerance_pos[2],
            )?),
            pose,
        )],
        1.0,
    )?));
    goal.push(Constraint::Orientation(OrientationConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        pose.rotation,
        OrientationTolerance::RotationVector {
            x: tolerance_angle[0],
            y: tolerance_angle[1],
            z: tolerance_angle[2],
        },
        1.0,
    )?));
    Ok(goal)
}

/// `updatePoseConstraint`: delegates to
/// [`update_position_constraint`]/[`update_orientation_constraint`] for
/// `link_name`'s existing constraints. Written with `&&` rather than two
/// separate `let`s to reproduce upstream's short-circuit
/// (`utils.cpp:271-272`): if the position update fails, the orientation
/// update is never attempted at all, not merely reported as failed.
///
/// # Errors
///
/// Whatever either delegate returns.
pub fn update_pose_constraint(
    constraints: &mut KinematicConstraintSet,
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    pose: Isometry3,
) -> Result<bool> {
    Ok(update_position_constraint(
        constraints,
        model,
        tf,
        link_name,
        frame_id,
        pose.translation.vector,
    )? && update_orientation_constraint(
        constraints,
        model,
        tf,
        link_name,
        frame_id,
        pose.rotation,
    )?)
}

/// `constructGoalConstraints(link_name, quat, tolerance)` (quaternion-only
/// overload): one [`OrientationConstraint`] targeting `orientation`.
///
/// Unlike [`construct_goal_pose_constraints`], this overload never sets
/// `ocm.parameterization` (`utils.cpp:275-290`), so it keeps
/// `moveit_msgs::msg::OrientationConstraint`'s field default — `0`,
/// documented in the message itself as `XYZ_EULER_ANGLES` — rather than
/// `ROTATION_VECTOR`. This port makes that default explicit with
/// [`OrientationTolerance::XyzEuler`] instead of leaving it implicit in an
/// unset integer field.
///
/// # Errors
///
/// Whatever [`OrientationConstraint::new`] returns for `link_name`/`frame_id`.
pub fn construct_goal_orientation_constraints(
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    orientation: UnitQuaternion,
    tolerance: f64,
) -> Result<KinematicConstraintSet> {
    let mut goal = KinematicConstraintSet::new();
    goal.push(Constraint::Orientation(OrientationConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        orientation,
        OrientationTolerance::XyzEuler {
            x: tolerance,
            y: tolerance,
            z: tolerance,
        },
        1.0,
    )?));
    Ok(goal)
}

/// `updateOrientationConstraint`: replaces the first
/// [`crate::OrientationConstraint`] in `constraints` whose
/// [`OrientationConstraint::link_name`] is `link_name` with a freshly built
/// one targeting `orientation` (same `frame_id`, tolerance and weight as
/// before). `Ok(false)` if no orientation constraint names `link_name`.
///
/// Upstream additionally rejects an empty `frame_id` explicitly before
/// mutating, logging and returning `false`. This port has no such special
/// case: [`OrientationConstraint::new`] itself already rejects an
/// unresolvable frame (including empty) as [`Error::UnknownName`] — the
/// same "unresolvable frame is an error, not a warning" deviation already
/// documented on that type — so the same input surfaces as `Err` here
/// instead of a logged `Ok(false)`.
///
/// # Errors
///
/// Whatever [`OrientationConstraint::new`] returns for `frame_id`.
pub fn update_orientation_constraint(
    constraints: &mut KinematicConstraintSet,
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    orientation: UnitQuaternion,
) -> Result<bool> {
    for c in constraints.constraints_mut() {
        let Constraint::Orientation(oc) = c else {
            continue;
        };
        if oc.link_name() != link_name {
            continue;
        }
        *oc = OrientationConstraint::new(
            model,
            tf,
            link_name,
            frame_id,
            orientation,
            oc.tolerance(),
            oc.weight(),
        )?;
        return Ok(true);
    }
    Ok(false)
}

/// `constructGoalConstraints(link_name, reference_point, goal_point,
/// tolerance)`, also covering the delegating overload with no
/// `reference_point`: call with `reference_point = Vector3::zeros()` for
/// that form (upstream: `constructGoalConstraints(link_name, {0,0,0},
/// goal_point, tolerance)`).
///
/// One [`PositionConstraint`]: a sphere of radius `tolerance` centered on
/// `goal_point`, with `reference_point` as the link offset.
///
/// # Errors
///
/// Whatever [`PositionConstraint::new`] returns for `link_name`/`frame_id`.
pub fn construct_goal_position_constraints(
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    reference_point: Vector3,
    frame_id: &str,
    goal_point: Vector3,
    tolerance: f64,
) -> Result<KinematicConstraintSet> {
    let mut goal = KinematicConstraintSet::new();
    let mut pose = Isometry3::identity();
    pose.translation.vector = goal_point;
    goal.push(Constraint::Position(PositionConstraint::new(
        model,
        tf,
        link_name,
        frame_id,
        reference_point,
        &[(Shape::Sphere(Sphere::new(tolerance)?), pose)],
        1.0,
    )?));
    Ok(goal)
}

/// `updatePositionConstraint`: replaces the first
/// [`crate::PositionConstraint`] in `constraints` whose
/// [`PositionConstraint::link_name`] is `link_name` with
/// `PositionConstraint::with_updated_position` (private: `crate::position`'s
/// reconstruction helper) at `position` (re-resolved against `frame_id`, same
/// link offset/weight/region shape as before). `Ok(false)` if no position
/// constraint names `link_name`.
///
/// # Errors
///
/// [`Error::Other`] if the matching constraint has other than exactly one
/// region — see `PositionConstraint::with_updated_position`'s doc comment
/// for why this port narrows upstream's silently-partial `primitive_poses.
/// at(0)` update to that single supported case instead of reproducing it.
/// Otherwise whatever `PositionConstraint::with_updated_position` returns
/// for `frame_id`.
pub fn update_position_constraint(
    constraints: &mut KinematicConstraintSet,
    model: &RobotModel,
    tf: &Transforms,
    link_name: &str,
    frame_id: &str,
    position: Vector3,
) -> Result<bool> {
    for c in constraints.constraints_mut() {
        let Constraint::Position(pc) = c else {
            continue;
        };
        if pc.link_name() != link_name {
            continue;
        }
        return match pc.with_updated_position(model, tf, frame_id, position)? {
            Some(updated) => {
                *pc = updated;
                Ok(true)
            }
            None => Err(Error::other(format!(
                "position constraint for link '{link_name}' has {} regions; \
                 update_position_constraint only supports exactly one",
                pc.constraint_regions().len()
            ))),
        };
    }
    Ok(false)
}
