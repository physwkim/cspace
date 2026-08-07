// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp
//   moveit_core/robot_model/src/joint_model.cpp

use std::collections::HashMap;
use std::f64::consts::PI;

use moveit_error::{Error, Result};
use moveit_geometry::Isometry3;

use super::bounds::{JointLimits, VariableBounds};
use super::fixed;
use super::floating::FloatingJoint;
use super::planar::PlanarJoint;
use super::prismatic::PrismaticJoint;
use super::revolute::RevoluteJoint;

/// The concrete kinds of joint this port supports.
///
/// Upstream `moveit::core::JointModel::JointType`. `UNKNOWN` is omitted: it
/// is a transient value upstream's base-class constructor assigns before a
/// concrete subclass constructor runs; no public constructor in this port
/// ever leaves a [`JointModel`] in that state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    /// Rotation about a fixed axis. 1 variable.
    Revolute,
    /// Translation along a fixed axis. 1 variable.
    Prismatic,
    /// Translation in a plane plus rotation about its normal. 3 variables.
    Planar,
    /// Unconstrained translation plus rotation. 7 variables (a redundant
    /// quaternion).
    Floating,
    /// No degrees of freedom. 0 variables.
    Fixed,
}

/// The variant-specific data for one [`JointModel`]. Unlike upstream's
/// class hierarchy (`JointModel` base, five `final`-in-practice
/// subclasses), this is a closed sum type: the five kinds are exhaustive,
/// so matching on it can never need a fallback arm for an unknown type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointKind {
    /// See [`RevoluteJoint`].
    Revolute(RevoluteJoint),
    /// See [`PrismaticJoint`].
    Prismatic(PrismaticJoint),
    /// See [`PlanarJoint`].
    Planar(PlanarJoint),
    /// See [`FloatingJoint`].
    Floating(FloatingJoint),
    /// No variant data: a fixed joint has no axis, no bounds and no state.
    Fixed,
}

/// A joint's mimic relationship, resolved to a name rather than a pointer.
///
/// Upstream `JointModel::mimic_`/`mimic_factor_`/`mimic_offset_`.
///
/// # Deviation from upstream
///
/// Upstream `mimic_` is a `const JointModel*` into the owning `RobotModel`'s
/// joint array, and `RobotModel::buildMimic` also collapses chains (a joint
/// mimicking a joint that itself mimics another) and rejects cycles. This
/// crate has no `RobotModel` — out of scope, see `PORTING-PLAN.md` Phase 1 —
/// so [`Mimic::joint_name`] is left unresolved, and chain collapsing/cycle
/// rejection is deferred to whatever builds the full model.
#[derive(Debug, Clone, PartialEq)]
pub struct Mimic {
    /// The name of the [`JointModel`] this joint mimics.
    pub joint_name: String,
    /// `mimic_factor_`: multiplier applied to the mimicked joint's value.
    pub factor: f64,
    /// `mimic_offset_`: offset added after the multiplier.
    pub offset: f64,
}

/// A joint from the robot: the transform it applies in the kinematic chain,
/// and the bounds and mimic relationship on the variables that describe it.
///
/// Upstream `moveit::core::JointModel`. Fields upstream keeps for tree
/// bookkeeping that only exists once a full model is built — `joint_index_`,
/// `first_variable_index_`, `parent_link_model_`/`child_link_model_`,
/// `descendant_*_models_`, `mimic_requests_`, `non_fixed_descendant_*` — are
/// not here: they are computed over the *whole* joint/link graph, which
/// `RobotModel` (a later phase, see `PORTING-PLAN.md`) owns, not any single
/// `JointModel`.
#[derive(Debug, Clone, PartialEq)]
pub struct JointModel {
    name: String,
    kind: JointKind,
    local_variable_names: Vec<String>,
    variable_names: Vec<String>,
    variable_bounds: Vec<VariableBounds>,
    variable_index: HashMap<String, usize>,
    mimic: Option<Mimic>,
    passive: bool,
    distance_factor: f64,
}

impl JointModel {
    fn new_single_variable(
        name: impl Into<String>,
        kind: JointKind,
        bounds: VariableBounds,
    ) -> Self {
        let name = name.into();
        let mut variable_index = HashMap::new();
        variable_index.insert(name.clone(), 0);
        Self {
            variable_names: vec![name.clone()],
            local_variable_names: Vec::new(),
            variable_bounds: vec![bounds],
            variable_index,
            name,
            kind,
            mimic: None,
            passive: false,
            distance_factor: 1.0,
        }
    }

    fn new_multi_variable(
        name: impl Into<String>,
        kind: JointKind,
        locals: &[&str],
        bounds: Vec<VariableBounds>,
    ) -> Self {
        let name = name.into();
        let local_variable_names: Vec<String> = locals.iter().map(|s| s.to_string()).collect();
        let variable_names: Vec<String> = local_variable_names
            .iter()
            .map(|l| format!("{name}/{l}"))
            .collect();
        let variable_index = variable_names
            .iter()
            .enumerate()
            .map(|(i, n)| (n.clone(), i))
            .collect();
        Self {
            name,
            kind,
            local_variable_names,
            variable_names,
            variable_bounds: bounds,
            variable_index,
            mimic: None,
            passive: false,
            distance_factor: 1.0,
        }
    }

    /// A revolute joint. Bounds default to `[-pi, pi]`, `position_bounded`;
    /// call [`JointModel::set_variable_bounds`] to override, and
    /// [`JointModel::set_continuous`] to make it wrap. Upstream
    /// `RevoluteJointModel`'s constructor.
    pub fn new_revolute(name: impl Into<String>) -> Self {
        let bounds = VariableBounds {
            min_position: -PI,
            max_position: PI,
            position_bounded: true,
            ..Default::default()
        };
        Self::new_single_variable(name, JointKind::Revolute(RevoluteJoint::default()), bounds)
    }

    /// A prismatic joint. Bounds default to `[-f64::MAX, f64::MAX]`,
    /// `position_bounded` — upstream uses
    /// `std::numeric_limits<double>::max()`, a large finite value, not
    /// infinity, unlike [`JointModel::new_planar`] and
    /// [`JointModel::new_floating`]'s translation bounds.
    pub fn new_prismatic(name: impl Into<String>) -> Self {
        let bounds = VariableBounds {
            min_position: -f64::MAX,
            max_position: f64::MAX,
            position_bounded: true,
            ..Default::default()
        };
        Self::new_single_variable(
            name,
            JointKind::Prismatic(PrismaticJoint::default()),
            bounds,
        )
    }

    /// A planar joint: `x`, `y`, `theta`. `x`/`y` default to
    /// `position_bounded` with an infinite range; `theta` defaults to
    /// *not* `position_bounded` despite having a finite `[-pi, pi]` range —
    /// see [`VariableBounds`]'s doc comment on why the flag and the range
    /// are independent.
    pub fn new_planar(name: impl Into<String>) -> Self {
        let inf = f64::INFINITY;
        let bounds = vec![
            VariableBounds {
                min_position: -inf,
                max_position: inf,
                position_bounded: true,
                ..Default::default()
            },
            VariableBounds {
                min_position: -inf,
                max_position: inf,
                position_bounded: true,
                ..Default::default()
            },
            VariableBounds {
                min_position: -PI,
                max_position: PI,
                position_bounded: false,
                ..Default::default()
            },
        ];
        Self::new_multi_variable(
            name,
            JointKind::Planar(PlanarJoint::default()),
            &["x", "y", "theta"],
            bounds,
        )
    }

    /// A floating joint: `trans_x, trans_y, trans_z, rot_x, rot_y, rot_z,
    /// rot_w`. Translation defaults to `position_bounded` with an infinite
    /// range (see [`FloatingJoint`]'s doc comment); the quaternion
    /// components default to `position_bounded` `[-1, 1]`.
    pub fn new_floating(name: impl Into<String>) -> Self {
        let inf = f64::INFINITY;
        let translation = || VariableBounds {
            min_position: -inf,
            max_position: inf,
            position_bounded: true,
            ..Default::default()
        };
        let quaternion_component = || VariableBounds {
            min_position: -1.0,
            max_position: 1.0,
            position_bounded: true,
            ..Default::default()
        };
        let bounds = vec![
            translation(),
            translation(),
            translation(),
            quaternion_component(),
            quaternion_component(),
            quaternion_component(),
            quaternion_component(),
        ];
        Self::new_multi_variable(
            name,
            JointKind::Floating(FloatingJoint::default()),
            &[
                "trans_x", "trans_y", "trans_z", "rot_x", "rot_y", "rot_z", "rot_w",
            ],
            bounds,
        )
    }

    /// A fixed joint: no variables, no bounds, no state.
    pub fn new_fixed(name: impl Into<String>) -> Self {
        let name = name.into();
        Self {
            variable_names: Vec::new(),
            local_variable_names: Vec::new(),
            variable_bounds: Vec::new(),
            variable_index: HashMap::new(),
            name,
            kind: JointKind::Fixed,
            mimic: None,
            passive: false,
            distance_factor: 1.0,
        }
    }

    // ---- Identity and type ----------------------------------------------

    /// `getName`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `getType`
    pub fn joint_type(&self) -> JointType {
        match &self.kind {
            JointKind::Revolute(_) => JointType::Revolute,
            JointKind::Prismatic(_) => JointType::Prismatic,
            JointKind::Planar(_) => JointType::Planar,
            JointKind::Floating(_) => JointType::Floating,
            JointKind::Fixed => JointType::Fixed,
        }
    }

    /// `getTypeName`
    pub fn type_name(&self) -> &'static str {
        match self.joint_type() {
            JointType::Revolute => "Revolute",
            JointType::Prismatic => "Prismatic",
            JointType::Planar => "Planar",
            JointType::Floating => "Floating",
            JointType::Fixed => "Fixed",
        }
    }

    /// `getStateSpaceDimension`
    pub fn state_space_dimension(&self) -> usize {
        match self.joint_type() {
            JointType::Revolute | JointType::Prismatic => 1,
            JointType::Planar => 3,
            JointType::Floating => 7,
            JointType::Fixed => 0,
        }
    }

    // ---- Kind-specific data access ---------------------------------------

    /// The kind-specific data (axis, continuous flag, motion model, ...).
    pub fn kind(&self) -> &JointKind {
        &self.kind
    }

    /// `&Self::Revolute` if this is a revolute joint.
    pub fn as_revolute(&self) -> Option<&RevoluteJoint> {
        match &self.kind {
            JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    /// `&mut Self::Revolute` if this is a revolute joint.
    pub fn as_revolute_mut(&mut self) -> Option<&mut RevoluteJoint> {
        match &mut self.kind {
            JointKind::Revolute(r) => Some(r),
            _ => None,
        }
    }

    /// `&Self::Prismatic` if this is a prismatic joint.
    pub fn as_prismatic(&self) -> Option<&PrismaticJoint> {
        match &self.kind {
            JointKind::Prismatic(p) => Some(p),
            _ => None,
        }
    }

    /// `&mut Self::Prismatic` if this is a prismatic joint.
    pub fn as_prismatic_mut(&mut self) -> Option<&mut PrismaticJoint> {
        match &mut self.kind {
            JointKind::Prismatic(p) => Some(p),
            _ => None,
        }
    }

    /// `&Self::Planar` if this is a planar joint.
    pub fn as_planar(&self) -> Option<&PlanarJoint> {
        match &self.kind {
            JointKind::Planar(p) => Some(p),
            _ => None,
        }
    }

    /// `&mut Self::Planar` if this is a planar joint.
    pub fn as_planar_mut(&mut self) -> Option<&mut PlanarJoint> {
        match &mut self.kind {
            JointKind::Planar(p) => Some(p),
            _ => None,
        }
    }

    /// `&Self::Floating` if this is a floating joint.
    pub fn as_floating(&self) -> Option<&FloatingJoint> {
        match &self.kind {
            JointKind::Floating(f) => Some(f),
            _ => None,
        }
    }

    /// `&mut Self::Floating` if this is a floating joint.
    pub fn as_floating_mut(&mut self) -> Option<&mut FloatingJoint> {
        match &mut self.kind {
            JointKind::Floating(f) => Some(f),
            _ => None,
        }
    }

    /// `RevoluteJointModel::setContinuous`. Only meaningful on a revolute
    /// joint: mutates that joint's sole [`VariableBounds`] as a side effect
    /// (`flag == true` forces `[-pi, pi]`, unbounded; `flag == false` forces
    /// `position_bounded`, leaving the range untouched), which is why this
    /// lives on [`JointModel`] rather than on [`RevoluteJoint`] itself,
    /// which does not own the bounds vector.
    ///
    /// # Deviation from upstream: the wrong-joint-type guard has no upstream
    /// counterpart
    ///
    /// `setContinuous` is declared only on `RevoluteJointModel`
    /// (`revolute_joint_model.hpp:70`), a subclass, not on the base
    /// `JointModel` at all — calling it on any other joint type is a compile
    /// error in C++, so upstream has no runtime check and cannot have one.
    /// This port collapses the whole subclass hierarchy into one
    /// [`JointModel`] with a closed [`JointKind`] enum instead (see
    /// [`JointKind`]'s own doc comment), which makes `set_continuous`
    /// callable on any variant, so the type-system guarantee upstream gets
    /// for free has to become this runtime guard.
    ///
    /// # Errors
    ///
    /// [`Error::other`] if this is not a revolute joint.
    pub fn set_continuous(&mut self, flag: bool) -> Result<()> {
        let name = self.name.clone();
        let JointKind::Revolute(revolute) = &mut self.kind else {
            return Err(Error::other(format!(
                "set_continuous called on non-revolute joint '{name}'"
            )));
        };
        revolute.set_continuous_flag(flag);
        let bounds = &mut self.variable_bounds[0];
        if flag {
            bounds.position_bounded = false;
            bounds.min_position = -PI;
            bounds.max_position = PI;
        } else {
            bounds.position_bounded = true;
        }
        Ok(())
    }

    // ---- Variables --------------------------------------------------------

    /// `getVariableNames`
    pub fn variable_names(&self) -> &[String] {
        &self.variable_names
    }

    /// `getLocalVariableNames`
    pub fn local_variable_names(&self) -> &[String] {
        &self.local_variable_names
    }

    /// `hasVariable`
    pub fn has_variable(&self, variable: &str) -> bool {
        self.variable_index.contains_key(variable)
    }

    /// `getVariableCount`
    pub fn variable_count(&self) -> usize {
        self.variable_names.len()
    }

    /// `getLocalVariableIndex`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `variable` is not one of this joint's
    /// variables.
    pub fn local_variable_index(&self, variable: &str) -> Result<usize> {
        self.variable_index
            .get(variable)
            .copied()
            .ok_or_else(|| Error::unknown_name("variable", variable))
    }

    // ---- Bounds -------------------------------------------------------

    /// `getVariableBounds() const` (the vector overload): every variable's
    /// bounds, in [`JointModel::variable_names`] order.
    pub fn variable_bounds(&self) -> &[VariableBounds] {
        &self.variable_bounds
    }

    /// `getVariableBounds(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `variable` is not one of this joint's
    /// variables.
    pub fn variable_bounds_for(&self, variable: &str) -> Result<&VariableBounds> {
        let index = self.local_variable_index(variable)?;
        Ok(&self.variable_bounds[index])
    }

    /// `setVariableBounds(const std::string&, const VariableBounds&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `variable` is not one of this joint's
    /// variables.
    pub fn set_variable_bounds(&mut self, variable: &str, bounds: VariableBounds) -> Result<()> {
        let index = self.local_variable_index(variable)?;
        self.variable_bounds[index] = bounds;
        Ok(())
    }

    /// `setVariableBounds(const std::vector<JointLimits>&)`: override bounds
    /// from a set of named limits. Unknown variables — either not one of
    /// this joint's names, or simply not mentioned — are left untouched.
    pub fn set_variable_bounds_from_limits(&mut self, limits: &[JointLimits]) {
        for (index, name) in self.variable_names.iter().enumerate() {
            let Some(limit) = limits.iter().find(|l| &l.joint_name == name) else {
                continue;
            };
            let bounds = &mut self.variable_bounds[index];
            bounds.position_bounded = limit.has_position_limits;
            if limit.has_position_limits {
                bounds.min_position = limit.min_position;
                bounds.max_position = limit.max_position;
            }
            bounds.velocity_bounded = limit.has_velocity_limits;
            if limit.has_velocity_limits {
                bounds.max_velocity = limit.max_velocity;
                bounds.min_velocity = -limit.max_velocity;
            }
            bounds.acceleration_bounded = limit.has_acceleration_limits;
            if limit.has_acceleration_limits {
                bounds.max_acceleration = limit.max_acceleration;
                bounds.min_acceleration = -limit.max_acceleration;
            }
            bounds.jerk_bounded = limit.has_jerk_limits;
            if limit.has_jerk_limits {
                bounds.max_jerk = limit.max_jerk;
                bounds.min_jerk = -limit.max_jerk;
            }
        }
    }

    /// `getVariableBoundsMsg`.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream caches this vector (`variable_bounds_msg_`) and recomputes it
    /// after every bounds mutation, so the getter can return a reference.
    /// A cache that must be kept in sync by every mutator is exactly the
    /// kind of dual-source-of-truth this port avoids elsewhere (see
    /// `PORTING-PLAN.md` §4.1); computing it fresh from
    /// [`JointModel::variable_bounds`] on every call has no staleness window
    /// to get wrong, at the cost of an allocation callers needing it in a
    /// hot loop can cache themselves.
    pub fn variable_bounds_msg(&self) -> Vec<JointLimits> {
        self.variable_names
            .iter()
            .zip(&self.variable_bounds)
            .map(|(name, b)| JointLimits {
                joint_name: name.clone(),
                has_position_limits: b.position_bounded,
                min_position: b.min_position,
                max_position: b.max_position,
                has_velocity_limits: b.velocity_bounded,
                max_velocity: b.min_velocity.abs().min(b.max_velocity.abs()),
                has_acceleration_limits: b.acceleration_bounded,
                max_acceleration: b.min_acceleration.abs().min(b.max_acceleration.abs()),
                has_jerk_limits: b.jerk_bounded,
                max_jerk: b.min_jerk.abs().min(b.max_jerk.abs()),
            })
            .collect()
    }

    // ---- Distance, mimic, passive -----------------------------------------

    /// `getDistanceFactor`
    pub fn distance_factor(&self) -> f64 {
        self.distance_factor
    }

    /// `setDistanceFactor`
    pub fn set_distance_factor(&mut self, factor: f64) {
        self.distance_factor = factor;
    }

    /// `getMimic`
    pub fn mimic(&self) -> Option<&Mimic> {
        self.mimic.as_ref()
    }

    /// `setMimic`
    pub fn set_mimic(&mut self, joint_name: impl Into<String>, factor: f64, offset: f64) {
        self.mimic = Some(Mimic {
            joint_name: joint_name.into(),
            factor,
            offset,
        });
    }

    /// Clear any mimic relationship. Upstream calls `setMimic(nullptr, 0.0,
    /// 0.0)` for this; that is not representable with an unresolved-by-name
    /// [`Mimic`] (there is no null joint name), so this port gives it its
    /// own method instead of overloading `set_mimic` with a sentinel.
    pub fn clear_mimic(&mut self) {
        self.mimic = None;
    }

    /// `isPassive`
    pub fn is_passive(&self) -> bool {
        self.passive
    }

    /// `setPassive`
    pub fn set_passive(&mut self, flag: bool) {
        self.passive = flag;
    }

    // ---- Position bounds and normalization --------------------------------

    /// `satisfiesPositionBounds`, with explicit `other_bounds`.
    pub fn satisfies_position_bounds_with(
        &self,
        values: &[f64],
        bounds: &[VariableBounds],
        margin: f64,
    ) -> bool {
        match &self.kind {
            JointKind::Revolute(r) => r.satisfies_position_bounds(values[0], &bounds[0], margin),
            JointKind::Prismatic(_) => {
                PrismaticJoint::satisfies_position_bounds(values[0], &bounds[0], margin)
            }
            JointKind::Planar(_) => PlanarJoint::satisfies_position_bounds(
                values[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
                bounds[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
                margin,
            ),
            JointKind::Floating(_) => FloatingJoint::satisfies_position_bounds(
                values[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
                bounds[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
                margin,
            ),
            JointKind::Fixed => true,
        }
    }

    /// `satisfiesPositionBounds`, using this joint's own bounds.
    pub fn satisfies_position_bounds(&self, values: &[f64], margin: f64) -> bool {
        self.satisfies_position_bounds_with(values, &self.variable_bounds, margin)
    }

    /// `enforcePositionBounds`, with explicit `other_bounds`. Returns
    /// `true` if `values` was changed.
    pub fn enforce_position_bounds_with(
        &self,
        values: &mut [f64],
        bounds: &[VariableBounds],
    ) -> bool {
        match &self.kind {
            JointKind::Revolute(r) => r.enforce_position_bounds(&mut values[0], &bounds[0]),
            JointKind::Prismatic(_) => {
                PrismaticJoint::enforce_position_bounds(&mut values[0], &bounds[0])
            }
            JointKind::Planar(_) => PlanarJoint::enforce_position_bounds(
                (&mut values[..3])
                    .try_into()
                    .expect("planar joint has 3 variables"),
                bounds[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
            ),
            JointKind::Floating(_) => FloatingJoint::enforce_position_bounds(
                (&mut values[..7])
                    .try_into()
                    .expect("floating joint has 7 variables"),
                bounds[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
            ),
            JointKind::Fixed => false,
        }
    }

    /// `enforcePositionBounds`, using this joint's own bounds.
    pub fn enforce_position_bounds(&self, values: &mut [f64]) -> bool {
        let bounds = self.variable_bounds.clone();
        self.enforce_position_bounds_with(values, &bounds)
    }

    /// `harmonizePosition`, with explicit `other_bounds`. The base class
    /// default (every kind except revolute) is a no-op that always returns
    /// `false`.
    pub fn harmonize_position_with(&self, values: &mut [f64], bounds: &[VariableBounds]) -> bool {
        match &self.kind {
            JointKind::Revolute(_) => RevoluteJoint::harmonize_position(&mut values[0], &bounds[0]),
            _ => false,
        }
    }

    /// `harmonizePosition`, using this joint's own bounds.
    pub fn harmonize_position(&self, values: &mut [f64]) -> bool {
        let bounds = self.variable_bounds.clone();
        self.harmonize_position_with(values, &bounds)
    }

    // ---- Velocity / acceleration / jerk bounds -----------------------------
    //
    // Base-class behaviour: identical across every joint kind, so these are
    // not dispatched through `kind` at all.

    /// `enforceVelocityBounds`, with explicit `other_bounds`. Returns `true`
    /// if `values` was changed.
    pub fn enforce_velocity_bounds_with(
        &self,
        values: &mut [f64],
        bounds: &[VariableBounds],
    ) -> bool {
        let mut changed = false;
        for (value, bound) in values.iter_mut().zip(bounds) {
            if bound.max_velocity < *value {
                *value = bound.max_velocity;
                changed = true;
            } else if bound.min_velocity > *value {
                *value = bound.min_velocity;
                changed = true;
            }
        }
        changed
    }

    /// `enforceVelocityBounds`, using this joint's own bounds.
    pub fn enforce_velocity_bounds(&self, values: &mut [f64]) -> bool {
        let bounds = self.variable_bounds.clone();
        self.enforce_velocity_bounds_with(values, &bounds)
    }

    /// `satisfiesVelocityBounds`, with explicit `other_bounds`.
    pub fn satisfies_velocity_bounds_with(
        &self,
        values: &[f64],
        bounds: &[VariableBounds],
        margin: f64,
    ) -> bool {
        values.iter().zip(bounds).all(|(value, bound)| {
            !bound.velocity_bounded
                || (*value <= bound.max_velocity + margin && *value >= bound.min_velocity - margin)
        })
    }

    /// `satisfiesVelocityBounds`, using this joint's own bounds.
    pub fn satisfies_velocity_bounds(&self, values: &[f64], margin: f64) -> bool {
        self.satisfies_velocity_bounds_with(values, &self.variable_bounds, margin)
    }

    /// `satisfiesAccelerationBounds`, with explicit `other_bounds`.
    pub fn satisfies_acceleration_bounds_with(
        &self,
        values: &[f64],
        bounds: &[VariableBounds],
        margin: f64,
    ) -> bool {
        values.iter().zip(bounds).all(|(value, bound)| {
            !bound.acceleration_bounded
                || (*value <= bound.max_acceleration + margin
                    && *value >= bound.min_acceleration - margin)
        })
    }

    /// `satisfiesAccelerationBounds`, using this joint's own bounds.
    pub fn satisfies_acceleration_bounds(&self, values: &[f64], margin: f64) -> bool {
        self.satisfies_acceleration_bounds_with(values, &self.variable_bounds, margin)
    }

    /// `satisfiesJerkBounds`, with explicit `other_bounds`.
    pub fn satisfies_jerk_bounds_with(
        &self,
        values: &[f64],
        bounds: &[VariableBounds],
        margin: f64,
    ) -> bool {
        values.iter().zip(bounds).all(|(value, bound)| {
            !bound.jerk_bounded
                || (*value <= bound.max_jerk + margin && *value >= bound.min_jerk - margin)
        })
    }

    /// `satisfiesJerkBounds`, using this joint's own bounds.
    pub fn satisfies_jerk_bounds(&self, values: &[f64], margin: f64) -> bool {
        self.satisfies_jerk_bounds_with(values, &self.variable_bounds, margin)
    }

    // ---- Default positions, distance, extent, interpolation ---------------

    /// `getVariableDefaultPositions`, with explicit `other_bounds`.
    pub fn variable_default_positions_with(&self, bounds: &[VariableBounds], out: &mut [f64]) {
        match &self.kind {
            JointKind::Revolute(_) => out[0] = RevoluteJoint::default_position(&bounds[0]),
            JointKind::Prismatic(_) => out[0] = PrismaticJoint::default_position(&bounds[0]),
            JointKind::Planar(_) => {
                let values = PlanarJoint::default_position(
                    bounds[..3]
                        .try_into()
                        .expect("planar joint has 3 variables"),
                );
                out[..3].copy_from_slice(&values);
            }
            JointKind::Floating(_) => {
                let values = FloatingJoint::default_position(
                    bounds[..7]
                        .try_into()
                        .expect("floating joint has 7 variables"),
                );
                out[..7].copy_from_slice(&values);
            }
            JointKind::Fixed => {}
        }
    }

    /// `getVariableDefaultPositions`, using this joint's own bounds.
    pub fn variable_default_positions(&self, out: &mut [f64]) {
        let bounds = self.variable_bounds.clone();
        self.variable_default_positions_with(&bounds, out);
    }

    /// `distance`
    pub fn distance(&self, values1: &[f64], values2: &[f64]) -> f64 {
        match &self.kind {
            JointKind::Revolute(r) => r.distance(values1[0], values2[0]),
            JointKind::Prismatic(_) => PrismaticJoint::distance(values1[0], values2[0]),
            JointKind::Planar(p) => p.distance(
                values1[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
                values2[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
            ),
            JointKind::Floating(f) => f.distance(
                values1[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
                values2[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
            ),
            JointKind::Fixed => 0.0,
        }
    }

    /// `getMaximumExtent`, with explicit `other_bounds`.
    pub fn maximum_extent_with(&self, bounds: &[VariableBounds]) -> f64 {
        match &self.kind {
            // Upstream's `other_bounds` parameter is unused here
            // (`RevoluteJointModel::getMaximumExtent(const Bounds&
            // /*other_bounds*/)`, revolute_joint_model.cpp:98-101) -- this
            // always reports the joint's own installed bounds, not the
            // caller-supplied `bounds`.
            JointKind::Revolute(_) => RevoluteJoint::maximum_extent(&self.variable_bounds[0]),
            JointKind::Prismatic(_) => {
                PrismaticJoint::maximum_extent(&self.variable_bounds[0], &bounds[0])
            }
            JointKind::Planar(p) => p.maximum_extent(
                bounds[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
            ),
            JointKind::Floating(f) => f.maximum_extent(
                bounds[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
            ),
            JointKind::Fixed => 0.0,
        }
    }

    /// `getMaximumExtent`, using this joint's own bounds.
    pub fn maximum_extent(&self) -> f64 {
        let bounds = self.variable_bounds.clone();
        self.maximum_extent_with(&bounds)
    }

    /// `interpolate`
    pub fn interpolate(&self, from: &[f64], to: &[f64], t: f64, state: &mut [f64]) {
        match &self.kind {
            JointKind::Revolute(r) => state[0] = r.interpolate(from[0], to[0], t),
            JointKind::Prismatic(_) => state[0] = PrismaticJoint::interpolate(from[0], to[0], t),
            JointKind::Planar(p) => {
                let values = p.interpolate(
                    from[..3].try_into().expect("planar joint has 3 variables"),
                    to[..3].try_into().expect("planar joint has 3 variables"),
                    t,
                );
                state[..3].copy_from_slice(&values);
            }
            JointKind::Floating(_) => {
                let values = FloatingJoint::interpolate(
                    from[..7]
                        .try_into()
                        .expect("floating joint has 7 variables"),
                    to[..7].try_into().expect("floating joint has 7 variables"),
                    t,
                );
                state[..7].copy_from_slice(&values);
            }
            JointKind::Fixed => {}
        }
    }

    // ---- Transforms ---------------------------------------------------

    /// `computeTransform`
    pub fn compute_transform(&self, values: &[f64]) -> Isometry3 {
        match &self.kind {
            JointKind::Revolute(r) => r.compute_transform(values[0]),
            JointKind::Prismatic(p) => p.compute_transform(values[0]),
            JointKind::Planar(_) => PlanarJoint::compute_transform(
                values[..3]
                    .try_into()
                    .expect("planar joint has 3 variables"),
            ),
            JointKind::Floating(_) => FloatingJoint::compute_transform(
                values[..7]
                    .try_into()
                    .expect("floating joint has 7 variables"),
            ),
            JointKind::Fixed => fixed::compute_transform(),
        }
    }

    /// `computeVariablePositions`
    pub fn compute_variable_positions(&self, transform: &Isometry3, out: &mut [f64]) {
        match &self.kind {
            JointKind::Revolute(r) => out[0] = r.compute_variable_position(transform),
            JointKind::Prismatic(p) => out[0] = p.compute_variable_position(transform),
            JointKind::Planar(_) => {
                let values = PlanarJoint::compute_variable_positions(transform);
                out[..3].copy_from_slice(&values);
            }
            JointKind::Floating(_) => {
                let values = FloatingJoint::compute_variable_positions(transform);
                out[..7].copy_from_slice(&values);
            }
            JointKind::Fixed => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_variable_joints_use_the_bare_joint_name() {
        let joint = JointModel::new_revolute("panda_joint1");
        assert_eq!(joint.variable_names(), ["panda_joint1"]);
        assert!(joint.local_variable_names().is_empty());
    }

    #[test]
    fn multi_variable_joints_prefix_local_names_with_the_joint_name() {
        let joint = JointModel::new_floating("virtual_joint");
        assert_eq!(
            joint.variable_names(),
            [
                "virtual_joint/trans_x",
                "virtual_joint/trans_y",
                "virtual_joint/trans_z",
                "virtual_joint/rot_x",
                "virtual_joint/rot_y",
                "virtual_joint/rot_z",
                "virtual_joint/rot_w",
            ]
        );
        assert_eq!(
            joint.local_variable_names(),
            [
                "trans_x", "trans_y", "trans_z", "rot_x", "rot_y", "rot_z", "rot_w"
            ]
        );
    }

    #[test]
    fn fixed_joint_has_no_variables_and_no_bounds() {
        let joint = JointModel::new_fixed("panda_joint8");
        assert!(joint.variable_names().is_empty());
        assert!(joint.variable_bounds().is_empty());
        assert_eq!(joint.state_space_dimension(), 0);
    }

    #[test]
    fn local_variable_index_errors_on_unknown_variable() {
        let joint = JointModel::new_revolute("panda_joint1");
        assert!(joint.local_variable_index("no_such_variable").is_err());
        assert_eq!(joint.local_variable_index("panda_joint1").unwrap(), 0);
    }

    #[test]
    fn set_continuous_errors_on_non_revolute_joint() {
        let mut joint = JointModel::new_prismatic("panda_finger_joint1");
        assert!(joint.set_continuous(true).is_err());
    }

    #[test]
    fn set_continuous_true_unbounds_position_and_forces_pi_range() {
        let mut joint = JointModel::new_revolute("j");
        joint
            .set_variable_bounds(
                "j",
                VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    ..Default::default()
                },
            )
            .unwrap();
        joint.set_continuous(true).unwrap();
        let bounds = &joint.variable_bounds()[0];
        assert!(!bounds.position_bounded);
        assert_eq!(bounds.min_position, -PI);
        assert_eq!(bounds.max_position, PI);
        assert!(joint.as_revolute().unwrap().is_continuous());
    }

    #[test]
    fn set_continuous_false_rebounds_position_without_touching_range() {
        let mut joint = JointModel::new_revolute("j");
        joint.set_continuous(true).unwrap();
        joint
            .set_variable_bounds(
                "j",
                VariableBounds {
                    min_position: -2.0,
                    max_position: 2.0,
                    position_bounded: false,
                    ..Default::default()
                },
            )
            .unwrap();
        joint.set_continuous(false).unwrap();
        let bounds = &joint.variable_bounds()[0];
        assert!(bounds.position_bounded);
        assert_eq!(bounds.min_position, -2.0);
        assert_eq!(bounds.max_position, 2.0);
    }

    #[test]
    fn mimic_set_get_clear_round_trip() {
        let mut joint = JointModel::new_prismatic("panda_finger_joint2");
        assert!(joint.mimic().is_none());
        joint.set_mimic("panda_finger_joint1", 1.0, 0.0);
        let mimic = joint.mimic().expect("just set");
        assert_eq!(mimic.joint_name, "panda_finger_joint1");
        assert_eq!(mimic.factor, 1.0);
        assert_eq!(mimic.offset, 0.0);
        joint.clear_mimic();
        assert!(joint.mimic().is_none());
    }

    #[test]
    fn variable_bounds_msg_reports_velocity_bounded_flag_independently() {
        let mut joint = JointModel::new_revolute("j");
        joint
            .set_variable_bounds(
                "j",
                VariableBounds {
                    min_position: -1.0,
                    max_position: 1.0,
                    position_bounded: true,
                    velocity_bounded: false,
                    ..Default::default()
                },
            )
            .unwrap();
        let msg = joint.variable_bounds_msg();
        assert_eq!(msg.len(), 1);
        assert!(msg[0].has_position_limits);
        assert!(!msg[0].has_velocity_limits);
    }

    #[test]
    fn set_variable_bounds_from_limits_ignores_unknown_names() {
        let mut joint = JointModel::new_revolute("panda_joint1");
        let original = joint.variable_bounds()[0];
        joint.set_variable_bounds_from_limits(&[JointLimits {
            joint_name: "not_this_joint".to_string(),
            has_position_limits: true,
            min_position: -9.0,
            max_position: 9.0,
            has_velocity_limits: false,
            max_velocity: 0.0,
            has_acceleration_limits: false,
            max_acceleration: 0.0,
            has_jerk_limits: false,
            max_jerk: 0.0,
        }]);
        assert_eq!(joint.variable_bounds()[0], original);
    }

    #[test]
    fn set_variable_bounds_from_limits_applies_matching_name() {
        let mut joint = JointModel::new_revolute("panda_joint1");
        joint.set_variable_bounds_from_limits(&[JointLimits {
            joint_name: "panda_joint1".to_string(),
            has_position_limits: true,
            min_position: -2.0,
            max_position: 2.0,
            has_velocity_limits: true,
            max_velocity: 3.0,
            has_acceleration_limits: false,
            max_acceleration: 0.0,
            has_jerk_limits: false,
            max_jerk: 0.0,
        }]);
        let bounds = joint.variable_bounds()[0];
        assert_eq!(bounds.min_position, -2.0);
        assert_eq!(bounds.max_position, 2.0);
        assert!(bounds.velocity_bounded);
        assert_eq!(bounds.max_velocity, 3.0);
        assert_eq!(bounds.min_velocity, -3.0);
    }

    #[test]
    fn revolute_dispatch_round_trips_through_joint_model() {
        let mut joint = JointModel::new_revolute("j");
        joint
            .as_revolute_mut()
            .unwrap()
            .set_axis(moveit_geometry::Vector3::new(0.0, 0.0, 1.0));
        let values = [0.7];
        let transform = joint.compute_transform(&values);
        let mut recovered = [0.0];
        joint.compute_variable_positions(&transform, &mut recovered);
        // Measured exact for this input; not asserted as a general property
        // of the round trip.
        assert_eq!(recovered[0], 0.7);
    }

    #[test]
    fn fixed_dispatch_is_a_transform_identity_noop() {
        let joint = JointModel::new_fixed("j");
        let transform = joint.compute_transform(&[]);
        // A fixed joint's transform is the identity; the zero vector's own
        // norm is exactly 0.0 under IEEE 754, not a value measured for this
        // input alone.
        assert_eq!(transform.translation.vector.norm(), 0.0);
        assert_eq!(joint.distance(&[], &[]), 0.0);
        assert_eq!(joint.maximum_extent(), 0.0);
    }

    #[test]
    fn revolute_maximum_extent_uses_its_own_bounds_not_the_callers() {
        // Upstream `RevoluteJointModel::getMaximumExtent` explicitly ignores
        // its `other_bounds` parameter (revolute_joint_model.cpp:98-101,
        // literally `getMaximumExtent(const Bounds& /*other_bounds*/)`) and
        // always reports its own installed bounds' extent -- unlike
        // Prismatic/Planar/Floating, whose siblings all read `other_bounds`
        // in some form. `other_bounds` here is deliberately far from the
        // joint's own default `[-PI, PI]` so a dispatch bug reading the
        // wrong side is unmistakable.
        let joint = JointModel::new_revolute("j");
        let other_bounds = [VariableBounds {
            min_position: -100.0,
            max_position: 100.0,
            position_bounded: true,
            ..Default::default()
        }];

        assert_eq!(joint.maximum_extent_with(&other_bounds), 2.0 * PI);
    }

    #[test]
    fn floating_dispatch_round_trips_through_joint_model() {
        let joint = JointModel::new_floating("virtual_joint");
        let values = [1.0, 2.0, 3.0, 0.5, 0.5, 0.5, 0.5];
        let transform = joint.compute_transform(&values);
        let mut recovered = [0.0; 7];
        joint.compute_variable_positions(&transform, &mut recovered);
        // Measured exact for these inputs; not asserted as a general
        // property of the round trip.
        for (a, b) in values.iter().zip(recovered.iter()) {
            assert_eq!(*a, *b);
        }
    }
}
