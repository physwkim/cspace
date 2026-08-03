// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp
//   (class JointConstraint)
//   moveit_core/kinematic_constraints/src/kinematic_constraint.cpp
//   (JointConstraint::configure, JointConstraint::decide, normalizeAngle)

use moveit_error::{Error, Result};
use moveit_model::RobotModel;
use moveit_model::joint::JointType;
use moveit_state::Posed;

use crate::ConstraintEvaluationResult;

const EPS: f64 = f64::EPSILON;

/// `normalizeAngle`, a `static` helper private to upstream's
/// `kinematic_constraint.cpp` — distinct from the `angles` package function
/// of the same name already ported at
/// [`moveit_model::joint::PlanarJoint`]'s call site. Ported verbatim from the
/// lines this port actually read (`kinematic_constraint.cpp:67-79`), not
/// reused from that other site, because upstream itself keeps two separate
/// implementations rather than sharing one.
fn normalize_angle(angle: f64) -> f64 {
    let mut v = angle % (2.0 * std::f64::consts::PI);
    if v < -std::f64::consts::PI {
        v += 2.0 * std::f64::consts::PI;
    } else if v > std::f64::consts::PI {
        v -= 2.0 * std::f64::consts::PI;
    }
    v
}

/// Single-DOF joint position constraint: satisfied when the joint variable
/// is within `[position - tolerance_below, position + tolerance_above]`,
/// wrapping around `±π` for a continuous joint.
///
/// Upstream `kinematic_constraints::JointConstraint`.
///
/// # Deviation from upstream: `Option<String>` for the local variable name
///
/// Upstream's `local_variable_name_` defaults to `""` to mean "this
/// constrains a whole single-DOF joint, not one variable of a multi-DOF
/// joint" — the same empty-string-as-absence pattern this port's
/// `moveit-model::JointModelGroup::EndEffectorParent` already replaces with
/// `Option`. [`JointConstraint::local_variable_name`] returns `Option<&str>`
/// here for the same reason.
///
/// # No design change needed otherwise
///
/// Unlike `Position`/`Orientation`/`VisibilityConstraint`, nothing else in
/// `moveit_msgs::msg::JointConstraint` has a `bool has_x` companion field —
/// `joint_name`, `position`, `tolerance_above`/`_below` and `weight` are all
/// unconditionally meaningful. [`JointConstraint::new`] takes them as plain
/// `f64`/`&str` arguments accordingly.
#[derive(Debug, Clone, PartialEq)]
pub struct JointConstraint {
    joint_variable_name: String,
    local_variable_name: Option<String>,
    variable_index: usize,
    is_continuous: bool,
    position: f64,
    tolerance_above: f64,
    tolerance_below: f64,
    weight: f64,
}

impl JointConstraint {
    /// Build and resolve a joint constraint against `model`.
    ///
    /// `joint_name` follows upstream's own convention: either a joint name
    /// (for a single-DOF joint) or `"joint_name/local_variable"` (for one
    /// variable of a multi-DOF joint, e.g. a planar joint's `theta`). This is
    /// not the `bool has_x` pattern under repair elsewhere in this crate —
    /// it is a single string naming one thing, matching how
    /// [`moveit_model::RobotModel::variable_names`] itself names multi-DOF
    /// variables.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `tolerance_above`/`tolerance_below` are
    /// negative, or `weight` is not strictly positive (upstream instead
    /// warns and substitutes `1.0` for a non-positive weight; substituting a
    /// value silently for invalid input is the failure mode `moveit-rs`
    /// prefers to surface as an error — see `Transforms`'s deviation 1 for
    /// the same call elsewhere in this port).
    /// [`Error::UnknownName`] if `joint_name` does not resolve to a joint (or
    /// a local variable of one) in `model`.
    /// [`Error::Other`] if the joint has zero variables (fixed joint) or,
    /// with no local variable given, more than one (a multi-DOF joint
    /// requires naming the variable).
    pub fn new(
        model: &RobotModel,
        joint_name: &str,
        position: f64,
        tolerance_above: f64,
        tolerance_below: f64,
        weight: f64,
    ) -> Result<Self> {
        if tolerance_above < 0.0 || tolerance_below < 0.0 {
            return Err(Error::construct(
                "JointConstraint tolerance values must be positive",
            ));
        }
        if weight <= EPS {
            return Err(Error::construct(
                "JointConstraint weight must be strictly positive",
            ));
        }

        // Upstream: `joint_variable_name_` is always the caller's full input
        // string, whether or not it turns out to name a local variable of a
        // multi-DOF joint — that string is already the exact form
        // `RobotModel::variable_index` expects (`"joint"` or
        // `"joint/local"`), so it is never rebuilt from the parts.
        let joint_variable_name = joint_name.to_string();

        let (joint_model, local_variable_name) = if model.has_joint_model(joint_name) {
            (model.joint_model(joint_name)?, None)
        } else if let Some((base, local)) = joint_name.rsplit_once('/') {
            // A trailing '/' (empty `local`) matches upstream's own
            // `pos + 1 < jc.joint_name.length()` guard: treated the same as
            // no local variable at all, not as an empty-named one.
            let local = if local.is_empty() { None } else { Some(local) };
            (model.joint_model(base)?, local)
        } else {
            (model.joint_model(joint_name)?, None)
        };

        match local_variable_name {
            Some(local) => {
                if !joint_model
                    .local_variable_names()
                    .iter()
                    .any(|n| n == local)
                {
                    return Err(Error::unknown_name("variable", local));
                }
            }
            None => {
                if joint_model.variable_count() == 0 {
                    return Err(Error::other(format!(
                        "joint '{joint_name}' has no parameters to constrain"
                    )));
                }
                if joint_model.variable_count() > 1 {
                    return Err(Error::other(format!(
                        "joint '{joint_name}' has more than one parameter to constrain; \
                         name the local variable (e.g. '{joint_name}/theta')"
                    )));
                }
            }
        }

        let local_variable_name = local_variable_name.map(str::to_string);
        let variable_index = model.variable_index(&joint_variable_name)?;

        let is_continuous = match joint_model.joint_type() {
            JointType::Revolute => joint_model.as_revolute().is_some_and(|r| r.is_continuous()),
            JointType::Planar => local_variable_name.as_deref() == Some("theta"),
            _ => false,
        };

        let (position, tolerance_above, tolerance_below) = if is_continuous {
            (normalize_angle(position), tolerance_above, tolerance_below)
        } else {
            let bounds = joint_model.variable_bounds_for(&joint_variable_name)?;
            if bounds.min_position > position + tolerance_above {
                (bounds.min_position, EPS, tolerance_below)
            } else if bounds.max_position < position - tolerance_below {
                (bounds.max_position, tolerance_above, EPS)
            } else {
                (position, tolerance_above, tolerance_below)
            }
        };

        Ok(Self {
            joint_variable_name,
            local_variable_name,
            variable_index,
            is_continuous,
            position,
            tolerance_above,
            tolerance_below,
            weight,
        })
    }

    /// `getJointVariableName`
    pub fn joint_variable_name(&self) -> &str {
        &self.joint_variable_name
    }

    /// `getLocalVariableName`, as `Option` — see this type's doc comment.
    pub fn local_variable_name(&self) -> Option<&str> {
        self.local_variable_name.as_deref()
    }

    /// `getJointVariableIndex`
    pub fn joint_variable_index(&self) -> usize {
        self.variable_index
    }

    /// `getDesiredJointPosition`
    pub fn desired_joint_position(&self) -> f64 {
        self.position
    }

    /// `getJointToleranceAbove`
    pub fn joint_tolerance_above(&self) -> f64 {
        self.tolerance_above
    }

    /// `getJointToleranceBelow`
    pub fn joint_tolerance_below(&self) -> f64 {
        self.tolerance_below
    }

    /// `JointConstraint::decide`.
    pub fn decide(&self, state: &Posed) -> ConstraintEvaluationResult {
        let current = state.variable_position_at(self.variable_index);

        let dif = if self.is_continuous {
            let mut dif = normalize_angle(current) - self.position;
            if dif > std::f64::consts::PI {
                dif = 2.0 * std::f64::consts::PI - dif;
            } else if dif < -std::f64::consts::PI {
                dif += 2.0 * std::f64::consts::PI;
            }
            dif
        } else {
            current - self.position
        };

        let satisfied =
            dif <= self.tolerance_above + 2.0 * EPS && dif >= -self.tolerance_below - 2.0 * EPS;
        ConstraintEvaluationResult::new(satisfied, self.weight * dif.abs())
    }
}
