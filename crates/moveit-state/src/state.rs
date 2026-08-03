// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp

use std::collections::{BTreeSet, HashMap};
use std::f64::consts::PI;
use std::ops::Deref;

use nalgebra::DMatrix;

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Vector3};
use moveit_model::JointModelGroup;
use moveit_model::RobotModel;
use moveit_model::joint::{JointKind, JointModel, JointType};
use rand::{Rng, RngExt};

/// An index into [`RobotModel::joint_models`](moveit_model::RobotModel::joint_models).
///
/// Upstream identifies a joint by `const JointModel*`; this port uses its
/// position in the model's joint array instead, matching the index
/// convention [`RobotModel`] itself already uses.
pub type JointIndex = usize;

/// A robot's variable positions, plus the forward-kinematics cache derived
/// from them.
///
/// Upstream `moveit::core::RobotState`. Build one with [`RobotState::new`],
/// which matches upstream's raw constructor: variable positions start at
/// `0.0` (upstream's `position_.resize(n)` value-initializes to `0.0`
/// too — the *caller* is expected to call [`RobotState::set_to_default_values`]
/// or another setter before relying on the state).
///
/// # The `Posed` split
///
/// Upstream tracks three independent "am I stale" flags:
/// `dirty_link_transforms_`, `dirty_collision_body_transforms_` and
/// `dirty_joint_transforms_`. `PORTING-PLAN.md` §4.1/§8.2 rejects that:
///
/// - `dirty_collision_body_transforms_` exists to lazily separate "link
///   transforms are fresh" from "collision body transforms are fresh", but
///   upstream's own public `update()` always recomputes both together in
///   lockstep, and this port's [`moveit_model::LinkModel`] carries no
///   collision geometry yet (deferred to a later `moveit-collision` phase —
///   see its doc comment). There is nothing for a second flag to guard
///   today, so it is not modelled; see this crate's `UNFIXED` note for
///   what a real second axis would need.
/// - `dirty_joint_transforms_` (one `bool` per joint, upstream's own
///   per-joint transform cache) is not reproduced either: this port
///   recomputes a joint's local transform on demand from `positions`
///   whenever [`Posed::joint_transform`] is called, a pure ~7-multiply
///   computation not worth caching separately from the link-transform
///   sweep that already visits every dirty joint once.
///
/// What is left is exactly one axis: the private `dirty` field, `Option<JointIndex>`
/// holding the common root of every joint whose transform is stale — mirroring
/// upstream's actual `dirty_link_transforms_` field type (`const JointModel*`,
/// not `bool`; see `robot_state.hpp:1682` and `RobotModel::getCommonRoot`).
/// [`RobotState::update`] recomputes only that subtree and returns [`Posed`],
/// a view that can only be constructed by `update`. Reading a transform
/// therefore requires holding a `Posed`, and holding one keeps the `&mut
/// RobotState` borrow that produced it alive — the borrow checker rejects
/// `state.some_read()` on the original handle for the `Posed`'s lifetime
/// (measured as `E0502` against an isolated prototype, see `PORTING-PLAN.md`
/// §8.2.1). That is why [`Posed`] derefs to [`RobotState`]: every position
/// read a caller needs while holding a view must be reachable *through* the
/// view.
///
/// # Deviations from upstream
///
/// 1. **Acceleration and effort get independent storage, not upstream's
///    aliased buffer.** Upstream stores both in one
///    `effort_or_acceleration_` vector, switched by
///    `has_acceleration_`/`has_effort_` (`markAcceleration`/`markEffort`
///    each clear the other flag): a memory optimisation whose only
///    observable consequence is that setting one silently clobbers the
///    other. No caller in this workspace relies on that clobber, so this
///    port gives each its own `Vec<f64>` instead of reproducing the dual
///    meaning. [`RobotState::enforce_bounds`]/[`RobotState::satisfies_bounds`]
///    check velocity bounds too, when [`RobotState::has_velocities`] is
///    true, matching upstream's `enforceBounds(const JointModel*)`/
///    `satisfiesBounds(const JointModel*, double)` in full now — upstream
///    itself never extends either to acceleration or effort bounds, so
///    this port does not either.
/// 2. **No attached bodies.** `attachBody`/`getFrameTransform`'s
///    attached-body fallback/`knowsFrameTransform`'s attached-body fallback
///    are all deferred; see the crate's `UNFIXED` report.
/// 3. **`common_root` is an ancestor-chain walk, not a precomputed O(1)
///    table.** Upstream's `RobotModel::getCommonRoot` answers from a
///    `common_joint_roots_` matrix built once at model-construction time.
///    [`moveit_model::RobotModel`] deliberately does not build that table
///    (its own doc comment: "the FK algorithm itself does not need them to
///    be correct, only fast") — this port owns that table's job, and
///    chooses the simpler O(depth) walk over the O(n^2) precompute, since
///    nothing in this task's scope calls `update()` often enough to make
///    the walk a bottleneck.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState<'m> {
    model: &'m RobotModel,
    positions: Vec<f64>,
    /// `velocity_`.
    velocity: Vec<f64>,
    /// Upstream aliases this onto the same buffer as `effort`, switched by
    /// `has_acceleration_`/`has_effort_`; this port gives it independent
    /// storage instead (see this type's doc comment).
    acceleration: Vec<f64>,
    /// See `acceleration`'s doc comment.
    effort: Vec<f64>,
    has_velocity: bool,
    has_acceleration: bool,
    has_effort: bool,
    transforms: Vec<Isometry3>,
    dirty: Option<JointIndex>,
    /// `joint.first_variable_index()` upstream — this port's `RobotModel`
    /// only exposes that by name lookup (which fails for a fixed joint's
    /// empty variable range), so it is recomputed here once, positionally.
    first_variable_index: Vec<usize>,
    /// `getJointOfVariable`: global variable index -> its owning joint.
    joint_of_variable: Vec<JointIndex>,
    joint_index_by_name: HashMap<String, JointIndex>,
    /// `LinkModel::getLinkIndex` given a joint's own index: the link this
    /// joint is the parent of.
    link_of_joint: Vec<usize>,
    /// `JointModel::getMimic`, as an index: `Some(master)` if this joint
    /// mimics another.
    mimic_master_index: Vec<Option<JointIndex>>,
    /// `JointModel::getMimicRequests`: the joints that directly mimic a
    /// given joint. Flat (one level) because [`RobotModel`] already
    /// collapses mimic-of-a-mimic chains at construction.
    mimic_requests: Vec<Vec<JointIndex>>,
    root_joint_index: JointIndex,
}

/// A read-only view of a [`RobotState`] whose `transforms` are proven
/// current for its `positions`, by construction: [`RobotState::update`] is
/// the only way to build one.
///
/// Derefs to [`RobotState`] so every position/model read a caller needs is
/// still reachable while holding a view (see [`RobotState`]'s doc comment
/// for why that delegation is required, not just convenient). The transform
/// reads ([`Posed::global_link_transform`], [`Posed::joint_transform`],
/// [`Posed::frame_transform`]) live only here, never on [`RobotState`]
/// itself, because only a `Posed` can prove they are fresh.
///
/// `Send + Sync`: a `Posed` is just `&RobotState`, and every field of
/// `RobotState` is plain data with no interior mutability, so both are
/// automatic — a collision checker can fan a `Posed` out across threads.
#[derive(Debug, Clone, Copy)]
pub struct Posed<'s, 'm>(&'s RobotState<'m>);

impl<'s, 'm> Deref for Posed<'s, 'm> {
    type Target = RobotState<'m>;

    fn deref(&self) -> &RobotState<'m> {
        self.0
    }
}

impl<'m> RobotState<'m> {
    /// Upstream `RobotState::RobotState`/`RobotState::init`.
    ///
    /// Every variable starts at `0.0` and the whole tree starts dirty
    /// (matching upstream's constructor, which sets `dirty_link_transforms_
    /// = robot_model_->getRootJoint()`), so [`RobotState::update`] always
    /// does a full sweep before the first [`Posed`] is handed out.
    pub fn new(model: &'m RobotModel) -> Self {
        let n_joints = model.joint_names().len();
        let n_links = model.link_names().len();

        let mut first_variable_index = Vec::with_capacity(n_joints);
        let mut acc = 0usize;
        for joint in model.joint_models() {
            first_variable_index.push(acc);
            acc += joint.variable_count();
        }

        let mut joint_of_variable = vec![0usize; model.variable_count()];
        for (joint_index, joint) in model.joint_models().enumerate() {
            let first = first_variable_index[joint_index];
            for offset in 0..joint.variable_count() {
                joint_of_variable[first + offset] = joint_index;
            }
        }

        let mut joint_index_by_name = HashMap::with_capacity(n_joints);
        for (joint_index, name) in model.joint_names().iter().enumerate() {
            joint_index_by_name.insert(name.clone(), joint_index);
        }

        let mut link_of_joint = vec![0usize; n_joints];
        for link in model.link_models() {
            link_of_joint[link.parent_joint_index()] = link.link_index();
        }

        let mut mimic_master_index = vec![None; n_joints];
        let mut mimic_requests = vec![Vec::new(); n_joints];
        for (joint_index, joint) in model.joint_models().enumerate() {
            if let Some(mimic) = joint.mimic() {
                let master_index = *joint_index_by_name
                    .get(mimic.joint_name.as_str())
                    .expect("RobotModel already dropped mimics of unknown joints");
                mimic_master_index[joint_index] = Some(master_index);
                mimic_requests[master_index].push(joint_index);
            }
        }

        let root_joint_index = model
            .link_model_at(model.root_link_index())
            .parent_joint_index();

        Self {
            model,
            positions: vec![0.0; model.variable_count()],
            velocity: vec![0.0; model.variable_count()],
            acceleration: vec![0.0; model.variable_count()],
            effort: vec![0.0; model.variable_count()],
            has_velocity: false,
            has_acceleration: false,
            has_effort: false,
            transforms: vec![Isometry3::identity(); n_links],
            dirty: Some(root_joint_index),
            first_variable_index,
            joint_of_variable,
            joint_index_by_name,
            link_of_joint,
            mimic_master_index,
            mimic_requests,
            root_joint_index,
        }
    }

    /// `getRobotModel`
    pub fn model(&self) -> &'m RobotModel {
        self.model
    }

    /// `getVariablePositions`
    pub fn positions(&self) -> &[f64] {
        &self.positions
    }

    /// `getVariablePosition(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn variable_position(&self, name: &str) -> Result<f64> {
        let index = self.model.variable_index(name)?;
        Ok(self.positions[index])
    }

    /// `getVariablePosition(int)`
    pub fn variable_position_at(&self, index: usize) -> f64 {
        self.positions[index]
    }

    /// A joint's own variable slice of [`RobotState::positions`]. Empty for
    /// a fixed joint, matching upstream `getJointPositions` returning
    /// `nullptr` for one (there being no `[f64]` equivalent of a null
    /// pointer, an empty slice is the faithful translation).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_position(&self, name: &str) -> Result<&[f64]> {
        let index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(index);
        let first = self.first_variable_index[index];
        Ok(&self.positions[first..first + joint.variable_count()])
    }

    /// This joint's index into [`RobotModel::joint_models`].
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_index(&self, name: &str) -> Result<JointIndex> {
        self.joint_index_by_name
            .get(name)
            .copied()
            .ok_or_else(|| Error::unknown_name("joint", name))
    }

    // ---- Velocity, acceleration, effort --------------------------------
    //
    // Upstream aliases `acceleration_`/`effort_` onto one buffer, switched
    // by `has_acceleration_`/`has_effort_`; see this type's doc comment for
    // why this port gives them independent storage instead. None of these
    // setters propagate to mimic joints — upstream's own
    // `setVariableVelocity`/`setVariableAcceleration`/`setVariableEffort`
    // do not call `updateMimicJoint` either.

    /// `hasVelocities`
    pub fn has_velocities(&self) -> bool {
        self.has_velocity
    }

    /// `hasAccelerations`
    pub fn has_accelerations(&self) -> bool {
        self.has_acceleration
    }

    /// `hasEffort`
    pub fn has_effort(&self) -> bool {
        self.has_effort
    }

    /// `getVariableVelocities` (const overload)
    pub fn velocities(&self) -> &[f64] {
        &self.velocity
    }

    /// `getVariableAccelerations` (const overload)
    pub fn accelerations(&self) -> &[f64] {
        &self.acceleration
    }

    /// `getVariableEffort` (const overload)
    pub fn effort(&self) -> &[f64] {
        &self.effort
    }

    /// `setVariableVelocities(const double*)`: replace every velocity at
    /// once.
    ///
    /// # Panics
    ///
    /// If `values.len()` does not equal
    /// [`RobotModel::variable_count`](moveit_model::RobotModel::variable_count),
    /// matching upstream's own precondition (there enforced only by a
    /// debug-only `assert`; here by the slice-copy itself).
    pub fn set_variable_velocities(&mut self, values: &[f64]) {
        self.velocity.copy_from_slice(values);
        self.has_velocity = true;
    }

    /// `setVariableAccelerations(const double*)`
    ///
    /// # Panics
    ///
    /// See [`RobotState::set_variable_velocities`].
    pub fn set_variable_accelerations(&mut self, values: &[f64]) {
        self.acceleration.copy_from_slice(values);
        self.has_acceleration = true;
    }

    /// `setVariableEffort(const double*)`: replace every effort value at
    /// once. Named `_efforts` (upstream overloads on parameter type, which
    /// Rust cannot) to stay distinct from the per-variable
    /// [`RobotState::set_variable_effort`].
    ///
    /// # Panics
    ///
    /// See [`RobotState::set_variable_velocities`].
    pub fn set_variable_efforts(&mut self, values: &[f64]) {
        self.effort.copy_from_slice(values);
        self.has_effort = true;
    }

    /// `getVariableVelocity(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn variable_velocity(&self, name: &str) -> Result<f64> {
        let index = self.model.variable_index(name)?;
        Ok(self.velocity[index])
    }

    /// `getVariableVelocity(int)`
    pub fn variable_velocity_at(&self, index: usize) -> f64 {
        self.velocity[index]
    }

    /// `setVariableVelocity(const std::string&, double)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_velocity(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.velocity[index] = value;
        self.has_velocity = true;
        Ok(())
    }

    /// `setVariableVelocity(int, double)`
    pub fn set_variable_velocity_at(&mut self, index: usize, value: f64) {
        self.velocity[index] = value;
        self.has_velocity = true;
    }

    /// `invertVelocity`: negate every velocity in place, a no-op when no
    /// velocity has been set. Upstream's `invertVelocity` negates only
    /// velocity, not acceleration, despite what a "reversing a trajectory"
    /// intuition might suggest — this port transcribes that as-is rather
    /// than also negating acceleration.
    pub fn invert_velocity(&mut self) {
        if self.has_velocity {
            for value in &mut self.velocity {
                *value *= -1.0;
            }
        }
    }

    /// `getVariableAcceleration(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn variable_acceleration(&self, name: &str) -> Result<f64> {
        let index = self.model.variable_index(name)?;
        Ok(self.acceleration[index])
    }

    /// `getVariableAcceleration(int)`
    pub fn variable_acceleration_at(&self, index: usize) -> f64 {
        self.acceleration[index]
    }

    /// `setVariableAcceleration(const std::string&, double)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_acceleration(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.acceleration[index] = value;
        self.has_acceleration = true;
        Ok(())
    }

    /// `setVariableAcceleration(int, double)`
    pub fn set_variable_acceleration_at(&mut self, index: usize, value: f64) {
        self.acceleration[index] = value;
        self.has_acceleration = true;
    }

    /// `getVariableEffort(const std::string&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn variable_effort(&self, name: &str) -> Result<f64> {
        let index = self.model.variable_index(name)?;
        Ok(self.effort[index])
    }

    /// `getVariableEffort(int)`
    pub fn variable_effort_at(&self, index: usize) -> f64 {
        self.effort[index]
    }

    /// `setVariableEffort(const std::string&, double)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_effort(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.effort[index] = value;
        self.has_effort = true;
        Ok(())
    }

    /// `setVariableEffort(int, double)`
    pub fn set_variable_effort_at(&mut self, index: usize, value: f64) {
        self.effort[index] = value;
        self.has_effort = true;
    }

    /// `getJointVelocities`: a joint's own variable slice of
    /// [`RobotState::velocities`]. Empty for a fixed joint (see
    /// [`RobotState::joint_position`] for why an empty slice is the
    /// faithful translation of upstream's `nullptr`).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_velocity(&self, name: &str) -> Result<&[f64]> {
        let index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(index);
        let first = self.first_variable_index[index];
        Ok(&self.velocity[first..first + joint.variable_count()])
    }

    /// `getJointAccelerations`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_acceleration(&self, name: &str) -> Result<&[f64]> {
        let index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(index);
        let first = self.first_variable_index[index];
        Ok(&self.acceleration[first..first + joint.variable_count()])
    }

    /// `getJointEffort`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn joint_effort(&self, name: &str) -> Result<&[f64]> {
        let index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(index);
        let first = self.first_variable_index[index];
        Ok(&self.effort[first..first + joint.variable_count()])
    }

    // ---- Setting positions --------------------------------------------

    /// `setVariablePositions(const double*)`/`setVariablePositions(const
    /// std::vector<double>&)`: replace every variable at once. The caller's
    /// array is assumed already mimic-consistent (upstream: "the full state
    /// includes mimic joint values, so no need to update mimic here") — no
    /// mimic propagation runs.
    ///
    /// # Panics
    ///
    /// If `positions.len()` does not equal
    /// [`RobotModel::variable_count`](moveit_model::RobotModel::variable_count),
    /// matching upstream's own precondition (there enforced only by a
    /// debug-only `assert`; here by the slice-copy itself).
    pub fn set_variable_positions(&mut self, positions: &[f64]) {
        self.positions.copy_from_slice(positions);
        self.dirty = Some(self.root_joint_index);
    }

    /// `setVariablePosition`: one variable, by name. Propagates to any
    /// joint that mimics this variable's joint.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `name` is not a variable in this model.
    pub fn set_variable_position(&mut self, name: &str, value: f64) -> Result<()> {
        let index = self.model.variable_index(name)?;
        self.positions[index] = value;
        let joint_index = self.joint_of_variable[index];
        self.mark_dirty(joint_index);
        self.update_mimic_joint(joint_index);
        Ok(())
    }

    /// `setVariablePositions(const std::map<std::string, double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any key is not a variable in this model.
    /// Upstream throws partway through the map on the first unknown key,
    /// leaving earlier entries already applied; this port does the same
    /// (iteration order is the caller-provided slice order, not upstream's
    /// `std::map` sorted order, so which entries land before the error
    /// differs — nothing downstream depends on that order, since a caller
    /// that hits this error has a malformed request either way).
    pub fn set_variable_positions_by_name(&mut self, values: &HashMap<String, f64>) -> Result<()> {
        for (name, &value) in values {
            self.set_variable_position(name, value)?;
        }
        Ok(())
    }

    /// `setVariablePositions(const std::vector<std::string>&, const
    /// std::vector<double>&)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if any name is not a variable in this model.
    pub fn set_variable_positions_named(&mut self, names: &[&str], values: &[f64]) -> Result<()> {
        debug_assert_eq!(names.len(), values.len());
        for (&name, &value) in names.iter().zip(values) {
            self.set_variable_position(name, value)?;
        }
        Ok(())
    }

    /// `setJointPositions(const JointModel*, const double*)`: one joint's
    /// own variables. Propagates to any joint that mimics this one.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    ///
    /// # Panics
    ///
    /// If `values.len()` does not equal the joint's own variable count.
    pub fn set_joint_positions(&mut self, name: &str, values: &[f64]) -> Result<()> {
        let joint_index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return Ok(());
        }
        let first = self.first_variable_index[joint_index];
        self.positions[first..first + joint.variable_count()].copy_from_slice(values);
        self.mark_dirty(joint_index);
        self.update_mimic_joint(joint_index);
        Ok(())
    }

    /// `setJointPositions(const JointModel*, const Eigen::Isometry3d&)`:
    /// one joint's own variables, derived from a transform via
    /// [`JointModel::compute_variable_positions`]. Propagates to any joint
    /// that mimics this one.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `name`.
    pub fn set_joint_transform(&mut self, name: &str, transform: &Isometry3) -> Result<()> {
        let joint_index = self.joint_index(name)?;
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return Ok(());
        }
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        let mut buf = [0.0f64; 7];
        joint.compute_variable_positions(transform, &mut buf[..count]);
        self.positions[first..first + count].copy_from_slice(&buf[..count]);
        self.mark_dirty(joint_index);
        self.update_mimic_joint(joint_index);
        Ok(())
    }

    /// `setToDefaultValues()`: every active joint to
    /// [`JointModel::variable_default_positions`], then every mimic joint
    /// derived from its (possibly just-changed) master —
    /// `RobotModel::getVariableDefaultPositions` calls
    /// `RobotModel::updateMimicJoints` internally upstream, a whole-model
    /// pass this port's private `propagate_all_mimics` matches.
    /// Verified against a live oracle: a mimic joint's value tracks its
    /// master's default even when the state was previously randomized to a
    /// different value (not merely left over from construction's `0.0`).
    pub fn set_to_default_values(&mut self) {
        let model = self.model;
        for &joint_index in model.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let first = self.first_variable_index[joint_index];
            let count = joint.variable_count();
            joint.variable_default_positions(&mut self.positions[first..first + count]);
        }
        self.propagate_all_mimics();
        self.dirty = Some(self.root_joint_index);
    }

    /// `setToRandomPositions()`, sampling with a caller-supplied RNG.
    ///
    /// Upstream owns a lazily-seeded `random_numbers::RandomNumberGenerator`
    /// member and exposes both an implicit-RNG and an explicit-RNG overload
    /// for the group-scoped variants; this port only ever takes an explicit
    /// RNG (both for the whole-model and, when added, group-scoped
    /// variants), matching `PORTING-PLAN.md`'s stance that bit-exact RNG
    /// parity with the C++ oracle is not required for this task — tests
    /// check structural correctness (bounds, mimic, quaternion
    /// normalization), not specific sampled values, so nothing needs the
    /// hidden-implicit-RNG shape upstream has.
    pub fn set_to_random_positions_with(&mut self, rng: &mut impl Rng) {
        let model = self.model;
        for &joint_index in model.active_joint_indices() {
            let joint = model.joint_model_at(joint_index);
            let first = self.first_variable_index[joint_index];
            let count = joint.variable_count();
            sample_random_positions(joint, rng, &mut self.positions[first..first + count]);
        }
        self.propagate_all_mimics();
        self.dirty = Some(self.root_joint_index);
    }

    // ---- Bounds ---------------------------------------------------------

    /// `enforceBounds()`: every active joint's own position bounds, plus
    /// velocity bounds when [`RobotState::has_velocities`] is true —
    /// matching upstream's `enforceBounds(const JointModel*)`, which
    /// combines `enforcePositionBounds` with a conditional
    /// `enforceVelocityBounds`.
    pub fn enforce_bounds(&mut self) {
        let model = self.model;
        for &joint_index in model.active_joint_indices() {
            self.enforce_bounds_for(joint_index);
        }
    }

    /// `enforceBounds(const JointModelGroup*)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn enforce_bounds_group(&mut self, group_name: &str) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        for &joint_index in group.active_joint_indices() {
            self.enforce_bounds_for(joint_index);
        }
        Ok(())
    }

    /// `enforceBounds(const JointModel*)`
    fn enforce_bounds_for(&mut self, joint_index: JointIndex) {
        self.enforce_position_bounds_for(joint_index);
        if self.has_velocity {
            self.enforce_velocity_bounds_for(joint_index);
        }
    }

    fn enforce_position_bounds_for(&mut self, joint_index: JointIndex) {
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return;
        }
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        if joint.enforce_position_bounds(&mut self.positions[first..first + count]) {
            self.mark_dirty(joint_index);
            self.update_mimic_joint(joint_index);
        }
    }

    /// `enforceVelocityBounds`. Unlike position, an out-of-bounds velocity
    /// clamp never dirties a transform or propagates to a mimic — upstream
    /// does not call `markDirtyJointTransforms`/`updateMimicJoint` from
    /// `enforceVelocityBounds` either.
    fn enforce_velocity_bounds_for(&mut self, joint_index: JointIndex) {
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return;
        }
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        joint.enforce_velocity_bounds(&mut self.velocity[first..first + count]);
    }

    /// `satisfiesBounds(double)`: every active joint's own position bounds,
    /// plus velocity bounds when [`RobotState::has_velocities`] is true.
    pub fn satisfies_bounds(&self, margin: f64) -> bool {
        self.model
            .active_joint_indices()
            .iter()
            .all(|&joint_index| self.satisfies_bounds_for(joint_index, margin))
    }

    /// `satisfiesBounds(const JointModelGroup*, double)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn satisfies_bounds_group(&self, group_name: &str, margin: f64) -> Result<bool> {
        let group = self.model.joint_model_group(group_name)?;
        Ok(group
            .active_joint_indices()
            .iter()
            .all(|&joint_index| self.satisfies_bounds_for(joint_index, margin)))
    }

    /// `satisfiesBounds(const JointModel*, double)`
    fn satisfies_bounds_for(&self, joint_index: JointIndex, margin: f64) -> bool {
        self.satisfies_position_bounds_for(joint_index, margin)
            && (!self.has_velocity || self.satisfies_velocity_bounds_for(joint_index, margin))
    }

    fn satisfies_position_bounds_for(&self, joint_index: JointIndex, margin: f64) -> bool {
        let joint = self.model.joint_model_at(joint_index);
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        joint.satisfies_position_bounds(&self.positions[first..first + count], margin)
    }

    /// `satisfiesVelocityBounds`
    fn satisfies_velocity_bounds_for(&self, joint_index: JointIndex, margin: f64) -> bool {
        let joint = self.model.joint_model_at(joint_index);
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        joint.satisfies_velocity_bounds(&self.velocity[first..first + count], margin)
    }

    /// `harmonizePositions()`: every active joint's own
    /// [`JointModel::harmonize_position`]. Does not mark anything dirty —
    /// re-wrapping a continuous joint's stored angle by a multiple of 2π
    /// does not change the transform it produces — but does propagate to
    /// mimics, so a mimic's stored value tracks its master's rewrapped one.
    pub fn harmonize_positions(&mut self) {
        let model = self.model;
        for &joint_index in model.active_joint_indices() {
            self.harmonize_position_for(joint_index);
        }
    }

    /// `harmonizePositions(const JointModelGroup*)`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group_name`.
    pub fn harmonize_positions_group(&mut self, group_name: &str) -> Result<()> {
        let model = self.model;
        let group = model.joint_model_group(group_name)?;
        for &joint_index in group.active_joint_indices() {
            self.harmonize_position_for(joint_index);
        }
        Ok(())
    }

    fn harmonize_position_for(&mut self, joint_index: JointIndex) {
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return;
        }
        let first = self.first_variable_index[joint_index];
        let count = joint.variable_count();
        if joint.harmonize_position(&mut self.positions[first..first + count]) {
            self.update_mimic_joint(joint_index);
        }
    }

    // ---- Frames -----------------------------------------------------------

    /// `knowsFrameTransform`: whether [`Posed::frame_transform`] would
    /// resolve `frame_id`. Needs no fresh transforms (a pure name lookup),
    /// so it lives on `RobotState` rather than `Posed`.
    ///
    /// # Deviation from upstream: does not special-case the model frame
    ///
    /// Upstream's own `getFrameInfo` (which `getFrameTransform` calls)
    /// treats `frame_id == model_frame` as always resolvable, to the model
    /// root, with an identity transform. `knowsFrameTransform` does not
    /// carry that special case — only `hasLinkModel`. This is a genuine
    /// upstream asymmetry (`robot_state.cpp:1338` vs `:1386`), not
    /// something this port introduces: if the model frame is not itself a
    /// link name, `knows_frame_transform(model_frame)` is `false` while
    /// [`Posed::frame_transform`] on the same name still succeeds.
    pub fn knows_frame_transform(&self, frame_id: &str) -> bool {
        let frame_id = frame_id.strip_prefix('/').unwrap_or(frame_id);
        self.model.has_link_model(frame_id)
    }

    // ---- Mimic ------------------------------------------------------------

    /// `RobotModel::updateMimicJoints(double*)`: every mimic joint in the
    /// whole model, derived from its master's *current* value. Used by the
    /// whole-model setters ([`RobotState::set_to_default_values`],
    /// [`RobotState::set_to_random_positions_with`]) after they have
    /// already written every active joint's value, matching upstream
    /// calling this from `RobotModel::getVariableDefaultPositions`/
    /// `getVariableRandomPositions` rather than from `RobotState` itself.
    fn propagate_all_mimics(&mut self) {
        for joint_index in 0..self.mimic_master_index.len() {
            let Some(master_index) = self.mimic_master_index[joint_index] else {
                continue;
            };
            let mimic = self
                .model
                .joint_model_at(joint_index)
                .mimic()
                .expect("mimic_master_index[joint_index] is Some only when the joint mimics");
            let source = self.positions[self.first_variable_index[master_index]];
            self.positions[self.first_variable_index[joint_index]] =
                mimic.factor * source + mimic.offset;
        }
    }

    /// `RobotState::updateMimicJoint`: propagate this one joint's *current*
    /// value to the joints that directly mimic it (one level — mimic
    /// chains are already collapsed by [`RobotModel`] construction), and
    /// mark each follower's transform dirty. Used by every single-variable
    /// and single-joint setter, matching upstream calling this right after
    /// each of those writes (never after the whole-model setters, which use
    /// [`RobotState::propagate_all_mimics`] instead).
    fn update_mimic_joint(&mut self, joint_index: JointIndex) {
        let joint = self.model.joint_model_at(joint_index);
        if joint.variable_count() == 0 {
            return;
        }
        let source = self.positions[self.first_variable_index[joint_index]];
        for follower_index in self.mimic_requests[joint_index].clone() {
            let follower = self.model.joint_model_at(follower_index);
            let mimic = follower
                .mimic()
                .expect("mimic_requests only ever lists followers that still mimic");
            self.positions[self.first_variable_index[follower_index]] =
                mimic.factor * source + mimic.offset;
            self.mark_dirty(follower_index);
        }
    }

    // ---- Dirty tracking / forward kinematics -------------------------

    /// Merge `joint_index` into the dirty subtree: upstream
    /// `markDirtyJointTransforms`, which either takes the root (nothing was
    /// dirty yet) or the common ancestor of the current root and the new
    /// joint (`RobotModel::getCommonRoot`).
    fn mark_dirty(&mut self, joint_index: JointIndex) {
        self.dirty = Some(match self.dirty {
            None => joint_index,
            Some(current) => self.common_root(current, joint_index),
        });
    }

    /// The lowest common ancestor of two joints in the kinematic tree, by
    /// depth-equalizing ancestor walk. See this type's doc comment for why
    /// this is O(depth) rather than upstream's O(1) precomputed table.
    fn common_root(&self, a: JointIndex, b: JointIndex) -> JointIndex {
        let depth = |mut joint_index: JointIndex| -> usize {
            let mut depth = 0;
            while let Some(parent) = self.model.parent_joint_index(joint_index) {
                joint_index = parent;
                depth += 1;
            }
            depth
        };

        let (mut a, mut b) = (a, b);
        let (mut depth_a, mut depth_b) = (depth(a), depth(b));
        while depth_a > depth_b {
            a = self
                .model
                .parent_joint_index(a)
                .expect("depth_a > depth_b implies a is not yet the root");
            depth_a -= 1;
        }
        while depth_b > depth_a {
            b = self
                .model
                .parent_joint_index(b)
                .expect("depth_b > depth_a implies b is not yet the root");
            depth_b -= 1;
        }
        while a != b {
            a = self
                .model
                .parent_joint_index(a)
                .expect("equal-depth siblings share a root above them");
            b = self
                .model
                .parent_joint_index(b)
                .expect("equal-depth siblings share a root above them");
        }
        a
    }

    /// `update(bool force = false)`: recompute exactly the dirty subtree
    /// (or the whole tree, if `force`), then return the proof that
    /// [`Posed`]'s transforms now correspond to [`RobotState::positions`].
    ///
    /// Upstream's public `update()` unconditionally cascades
    /// `updateCollisionBodyTransforms()` -> `updateLinkTransforms()`; this
    /// port has no second transform cache to cascade into (see this type's
    /// doc comment), so `update` recomputes link transforms directly.
    pub fn update(&mut self) -> Posed<'_, 'm> {
        if let Some(root) = self.dirty.take() {
            self.recompute_link_transforms_from(root);
        }
        Posed(self)
    }

    /// `update(true)`
    pub fn update_forced(&mut self) -> Posed<'_, 'm> {
        self.dirty = Some(self.root_joint_index);
        self.update()
    }

    /// `updateLinkTransformsInternal`, generalized to one formula for every
    /// link/joint kind — see this type's doc comment.
    ///
    /// `global(link) = global(parent link, or Identity for the root) *
    /// link.joint_origin_transform() * joint.compute_transform(local
    /// positions)`. This holds without the branch upstream needs for a
    /// zero-variable root joint or an identity `joint_origin_transform`,
    /// because in this port: the root link's `joint_origin_transform` is
    /// always constructed as `Isometry3::identity()` (`RobotModel`'s SRDF
    /// virtual joint carries no origin offset — verified in
    /// `RobotModel::from_urdf_and_srdf`, which passes
    /// `Isometry3::identity()` as the root's `joint_origin`), and
    /// `JointKind::Fixed::compute_transform` always returns `Identity`
    /// regardless of its (here, empty) input slice.
    fn recompute_link_transforms_from(&mut self, start_joint_index: JointIndex) {
        let model = self.model;
        let start_link = self.link_of_joint[start_joint_index];
        let mut stack = vec![start_link];
        while let Some(link_index) = stack.pop() {
            let link = model.link_model_at(link_index);
            let parent_transform = match link.parent_link_index() {
                Some(parent_link_index) => self.transforms[parent_link_index],
                None => Isometry3::identity(),
            };
            let joint_index = link.parent_joint_index();
            let joint = model.joint_model_at(joint_index);
            let first = self.first_variable_index[joint_index];
            let count = joint.variable_count();
            let joint_transform = joint.compute_transform(&self.positions[first..first + count]);
            self.transforms[link_index] =
                parent_transform * (*link.joint_origin_transform()) * joint_transform;
            stack.extend(
                link.child_joint_indices()
                    .iter()
                    .map(|&child_joint_index| self.link_of_joint[child_joint_index]),
            );
        }
    }

    // ---- Jacobian support ---------------------------------------------
    //
    // moveit_model::JointModelGroup deliberately does not store
    // `is_chain_`/`joint_roots_`/`updated_link_model_set_` (see its own doc
    // comment: computed over the whole joint/link graph, which only
    // RobotState's per-call walk below actually needs). The four helpers
    // here recompute exactly what upstream's `JointModelGroup` constructor
    // and its `includesParent`/`jointPrecedes` free functions
    // (joint_model_group.cpp) precompute once, but on demand.

    /// The link a joint is attached from. Upstream
    /// `JointModel::getParentLinkModel()`; `None` only for the model's
    /// absolute root joint, matching upstream returning `nullptr` there.
    fn parent_link_of_joint(&self, joint_index: JointIndex) -> Option<usize> {
        self.model
            .link_model_at(self.link_of_joint[joint_index])
            .parent_link_index()
    }

    /// `includesParent`: true if some ancestor of `joint_index` — walking
    /// up through `RobotModel::parent_joint_index`, and recursing into a
    /// mimic ancestor's own master when that ancestor itself mimics
    /// another joint — is a non-mimic active joint of `group`. A `false`
    /// result means `joint_index` roots a distinct subtree within `group`.
    fn includes_parent(&self, joint_index: JointIndex, group: &JointModelGroup) -> bool {
        let mut current = joint_index;
        loop {
            let Some(next) = self.model.parent_joint_index(current) else {
                return false;
            };
            current = next;
            let joint = self.model.joint_model_at(current);
            if group.has_joint_model(joint.name())
                && joint.variable_count() > 0
                && joint.mimic().is_none()
            {
                return true;
            }
            if let Some(mimic) = joint.mimic() {
                let mimic_index = self.joint_index_by_name[mimic.joint_name.as_str()];
                let mimic_joint = self.model.joint_model_at(mimic_index);
                if group.has_joint_model(mimic_joint.name())
                    && mimic_joint.variable_count() > 0
                    && mimic_joint.mimic().is_none()
                {
                    return true;
                }
                if self.includes_parent(mimic_index, group) {
                    return true;
                }
            }
        }
    }

    /// `jointPrecedes`: true if `b` is `a`'s nearest ancestor once any
    /// fixed joints in between are skipped over.
    fn joint_precedes(&self, a: JointIndex, b: JointIndex) -> bool {
        let Some(mut p) = self.model.parent_joint_index(a) else {
            return false;
        };
        loop {
            if p == b {
                return true;
            }
            if self.model.joint_model_at(p).joint_type() != JointType::Fixed {
                return false;
            }
            let Some(next) = self.model.parent_joint_index(p) else {
                return false;
            };
            p = next;
        }
    }

    /// `JointModelGroup::isChain()`/`joint_roots_`: `Some(root)` iff
    /// `group` is a chain — exactly one of its active joints has no
    /// in-group ancestor, and every consecutive pair in the group's full
    /// (already depth-first-sorted, see `RobotModel::joint_indices`) joint
    /// list satisfies `joint_precedes`.
    fn chain_root(&self, group: &JointModelGroup) -> Option<JointIndex> {
        let roots: Vec<JointIndex> = group
            .active_joint_indices()
            .iter()
            .copied()
            .filter(|&joint_index| !self.includes_parent(joint_index, group))
            .collect();
        if roots.len() != 1 {
            return None;
        }

        let joints = group.joint_indices();
        for k in (1..joints.len()).rev() {
            if !self.joint_precedes(joints[k], joints[k - 1]) {
                return None;
            }
        }
        Some(roots[0])
    }

    /// `JointModel::getDescendantLinkModels()`, walked on demand for a
    /// single joint rather than precomputed for every joint in the model
    /// (this port only ever needs it for one chain root at a time — see
    /// `RobotModel`'s own doc comment on why it skips that precompute).
    /// Every link reachable from `joint_index`'s own child link by
    /// repeatedly following either a link's child joints or a joint's
    /// mimic followers (upstream's `computeDescendantsHelper` recurses
    /// into both `LinkModel::getChildJointModels` and
    /// `JointModel::getMimicRequests`), including `joint_index`'s own
    /// child link.
    fn descendant_links_of_joint(&self, joint_index: JointIndex) -> BTreeSet<usize> {
        let mut seen_joints = BTreeSet::new();
        let mut links = BTreeSet::new();
        let mut stack = vec![joint_index];
        while let Some(current) = stack.pop() {
            if !seen_joints.insert(current) {
                continue;
            }
            let child_link = self.link_of_joint[current];
            links.insert(child_link);
            let link = self.model.link_model_at(child_link);
            stack.extend(link.child_joint_indices().iter().copied());
            stack.extend(self.mimic_requests[current].iter().copied());
        }
        links
    }
}

impl<'s, 'm> Posed<'s, 'm> {
    /// `getGlobalLinkTransform`
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no link is named `link_name`.
    pub fn global_link_transform(&self, link_name: &str) -> Result<Isometry3> {
        let link = self.0.model.link_model(link_name)?;
        Ok(self.0.transforms[link.link_index()])
    }

    /// `getGlobalLinkTransform`, by index — avoids the name lookup when the
    /// caller already resolved the index (for example while walking every
    /// link in [`RobotModel::link_models`](moveit_model::RobotModel::link_models)
    /// order).
    pub fn global_link_transform_at(&self, link_index: usize) -> Isometry3 {
        self.0.transforms[link_index]
    }

    /// `getJointTransform`: this joint's own local transform, recomputed
    /// from its current positions (see [`RobotState`]'s doc comment for why
    /// this is not cached separately).
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no joint is named `joint_name`.
    pub fn joint_transform(&self, joint_name: &str) -> Result<Isometry3> {
        let joint_index = self.0.joint_index(joint_name)?;
        let joint = self.0.model.joint_model_at(joint_index);
        let first = self.0.first_variable_index[joint_index];
        let count = joint.variable_count();
        Ok(joint.compute_transform(&self.0.positions[first..first + count]))
    }

    /// `getFrameTransform`/`getFrameInfo`, restricted to what this port
    /// supports: a leading `/` is stripped, `frame_id == model_frame`
    /// resolves to the identity transform at the root link (upstream:
    /// `robot_state.cpp:1345`), and otherwise `frame_id` must name a link.
    /// Upstream's further fallback to attached bodies and their subframes
    /// is out of scope for this task (attached bodies are not ported); see
    /// this crate's `UNFIXED` report.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if `frame_id` names neither the model frame
    /// nor a link.
    pub fn frame_transform(&self, frame_id: &str) -> Result<Isometry3> {
        let frame_id = frame_id.strip_prefix('/').unwrap_or(frame_id);
        if frame_id == self.0.model.model_frame() {
            return Ok(Isometry3::identity());
        }
        self.global_link_transform(frame_id)
    }

    /// `getJacobian(const JointModelGroup*, const Eigen::Vector3d&)`: the
    /// 6xN geometric Jacobian of `group`'s last link
    /// (`group.link_models().last()`) at `reference_point` (a point fixed
    /// in that link's own frame), expressed in the model frame — rows
    /// ordered translation (0..3) then rotation (3..6), columns in
    /// `group`'s variable order.
    ///
    /// Upstream's 2-argument overload always calls its 4-argument sibling
    /// with `link = group->getLinkModels().back()` and
    /// `use_quaternion_representation = false`; this is that fixed
    /// combination.
    ///
    /// # Deviations from upstream
    ///
    /// 1. **No quaternion-representation branch.** The 4-argument
    ///    overload also supports a 7-row quaternion-derivative Jacobian
    ///    when `use_quaternion_representation` is true, but the
    ///    2-argument overload this method matches always passes `false`,
    ///    so that branch is dead code behind this entry point and is not
    ///    ported.
    /// 2. **Distinct typed errors, not a bool + log line collapsed into
    ///    one generic exception.** Upstream's 4-argument overload returns
    ///    `false` (and logs) for "not a chain", "link not in the chain"
    ///    and "unsupported joint type"; its 2-argument overload then
    ///    throws one generic `Exception` for all of them. This method
    ///    keeps them as distinct [`Error::Other`] messages instead.
    ///    Upstream's fourth rejection, "the group has no joint models", is
    ///    not reachable through this method: `group` being a chain
    ///    already implies at least one active joint (see the doc comment
    ///    on this crate's `chain_root` helper), so it is not ported as a
    ///    separate case.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownName`] if no group is named `group`.
    ///
    /// [`Error::Other`] if `group` is not a chain, if its last link falls
    /// outside that chain (only possible for a group whose own joints and
    /// links disagree — not reachable from any group [`RobotModel`]
    /// itself builds, but checked to stay faithful to upstream), or if an
    /// active joint on the path to the tip is neither revolute, prismatic
    /// nor planar (the only unported case in practice is a floating
    /// joint's group, since a fixed joint is never active).
    pub fn jacobian(&self, group: &str, reference_point: &Vector3) -> Result<DMatrix<f64>> {
        let model = self.0.model;
        let group_model = model.joint_model_group(group)?;

        let Some(chain_root) = self.0.chain_root(group_model) else {
            return Err(Error::other(format!(
                "the group '{group}' is not a chain; cannot compute Jacobian"
            )));
        };

        let tip_link = *group_model
            .link_indices()
            .last()
            .expect("a chain root's active joint gives the group at least one link");

        let descendant_links = self.0.descendant_links_of_joint(chain_root);
        if !descendant_links.contains(&tip_link) {
            return Err(Error::other(format!(
                "link '{}' does not belong to the chain rooted by group '{group}'; cannot compute Jacobian",
                model.link_model_at(tip_link).name()
            )));
        }

        let root_joint = group_model.joint_indices()[0];
        let root_pose_world = match self.0.parent_link_of_joint(root_joint) {
            Some(root_link) => self.global_link_transform_at(root_link).inverse(),
            None => Isometry3::identity(),
        };

        let columns = group_model.variable_names().len();
        let mut jacobian = DMatrix::<f64>::zeros(6, columns);

        let root_pose_tip = root_pose_world * self.global_link_transform_at(tip_link);
        // Eigen's `Isometry3d * Vector3d` is a full point transform
        // (rotation and translation); nalgebra's `Isometry3 * Vector3` is
        // rotation-only by design, so the translation is added by hand
        // here to match upstream `root_pose_tip * reference_point_position`.
        let tip_point = root_pose_tip.rotation * reference_point + root_pose_tip.translation.vector;

        let mut i = 0usize;
        for &joint_index in group_model.active_joint_indices() {
            if self.0.parent_link_of_joint(joint_index) == Some(tip_link) {
                break;
            }
            let child_link = self.0.link_of_joint[joint_index];
            let root_pose_link = root_pose_world * self.global_link_transform_at(child_link);
            let joint = model.joint_model_at(joint_index);

            match joint.kind() {
                JointKind::Revolute(revolute) => {
                    let axis_wrt_origin = root_pose_link.rotation * revolute.axis();
                    let linear =
                        axis_wrt_origin.cross(&(tip_point - root_pose_link.translation.vector));
                    jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&linear);
                    jacobian
                        .fixed_view_mut::<3, 1>(3, i)
                        .copy_from(&axis_wrt_origin);
                }
                JointKind::Prismatic(prismatic) => {
                    let axis_wrt_origin = root_pose_link.rotation * prismatic.axis();
                    jacobian
                        .fixed_view_mut::<3, 1>(0, i)
                        .copy_from(&axis_wrt_origin);
                }
                JointKind::Planar(_) => {
                    let x_axis = root_pose_link.rotation * Vector3::new(1.0, 0.0, 0.0);
                    let y_axis = root_pose_link.rotation * Vector3::new(0.0, 1.0, 0.0);
                    let z_axis = root_pose_link.rotation * Vector3::new(0.0, 0.0, 1.0);
                    let z_linear = z_axis.cross(&(tip_point - root_pose_link.translation.vector));
                    jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&x_axis);
                    jacobian.fixed_view_mut::<3, 1>(0, i + 1).copy_from(&y_axis);
                    jacobian
                        .fixed_view_mut::<3, 1>(0, i + 2)
                        .copy_from(&z_linear);
                    jacobian.fixed_view_mut::<3, 1>(3, i + 2).copy_from(&z_axis);
                }
                JointKind::Floating(_) | JointKind::Fixed => {
                    return Err(Error::other(format!(
                        "joint '{}' has unsupported type {} for Jacobian computation",
                        joint.name(),
                        joint.type_name()
                    )));
                }
            }

            i += joint.variable_count();
        }

        Ok(jacobian)
    }
}

/// `JointModel::getVariableRandomPositions`, reimplemented here since this
/// port's [`moveit_model::RobotModel`] deliberately excludes it (Phase 1
/// deviation #4: the C++ oracle owns randomness for differential testing).
/// Mirrors each joint kind's actual upstream sampling rule (verified by
/// reading `{revolute,prismatic,planar,floating}_joint_model.cpp`, not
/// assumed from the joint's general shape):
///
/// - Revolute/prismatic: uniform within bounds.
/// - Planar translation (x, y) and floating translation (x, y, z): uniform
///   within bounds, or `0.0` if that axis's bounds are non-finite (a
///   floating joint's translation is `position_bounded == true` with
///   infinite `min`/`max` — see [`moveit_model::joint::FloatingJoint`]'s
///   doc comment — so "bounded" cannot be used as the non-finite check
///   here; finiteness is checked directly).
/// - Planar rotation (theta): uniform within bounds directly, no
///   finiteness check — a planar joint's theta bounds are always finite
///   (`[-pi, pi]`) even though `position_bounded == false` marks it
///   unbounded (it wraps, per [`crate`]'s use of
///   [`moveit_model::joint::PlanarJoint`]).
/// - Floating rotation: a uniformly random unit quaternion (Shoemake's
///   algorithm), not sampled component-by-component (which would not stay
///   normalized).
fn sample_random_positions(joint: &JointModel, rng: &mut impl Rng, out: &mut [f64]) {
    let bounds = joint.variable_bounds();
    match joint.kind() {
        JointKind::Revolute(_) | JointKind::Prismatic(_) => {
            out[0] = sample_uniform(rng, bounds[0].min_position, bounds[0].max_position);
        }
        JointKind::Planar(_) => {
            out[0] = sample_uniform_or_zero(rng, bounds[0].min_position, bounds[0].max_position);
            out[1] = sample_uniform_or_zero(rng, bounds[1].min_position, bounds[1].max_position);
            out[2] = sample_uniform(rng, bounds[2].min_position, bounds[2].max_position);
        }
        JointKind::Floating(_) => {
            out[0] = sample_uniform_or_zero(rng, bounds[0].min_position, bounds[0].max_position);
            out[1] = sample_uniform_or_zero(rng, bounds[1].min_position, bounds[1].max_position);
            out[2] = sample_uniform_or_zero(rng, bounds[2].min_position, bounds[2].max_position);
            let (x, y, z, w) = sample_unit_quaternion(rng);
            out[3] = x;
            out[4] = y;
            out[5] = z;
            out[6] = w;
        }
        JointKind::Fixed => {}
    }
}

fn sample_uniform(rng: &mut impl Rng, min: f64, max: f64) -> f64 {
    if min == max {
        return min;
    }
    rng.random_range(min..=max)
}

fn sample_uniform_or_zero(rng: &mut impl Rng, min: f64, max: f64) -> f64 {
    if min.is_finite() && max.is_finite() {
        sample_uniform(rng, min, max)
    } else {
        0.0
    }
}

/// A uniformly random unit quaternion as `(x, y, z, w)`, via Shoemake's
/// algorithm (three independent uniform-`[0,1)` draws). This port makes no
/// attempt to match the C++ oracle's `random_numbers::RandomNumberGenerator`
/// bit-for-bit (`PORTING-PLAN.md`: not required for this task); this
/// produces a uniform distribution over the unit sphere in `R^4`, which is
/// what upstream's own `rng.quaternion(q)` guarantees too, and that
/// property (not specific sampled values) is what this crate's tests check.
fn sample_unit_quaternion(rng: &mut impl Rng) -> (f64, f64, f64, f64) {
    let u1: f64 = rng.random();
    let u2: f64 = rng.random();
    let u3: f64 = rng.random();
    let sqrt_1_u1 = (1.0 - u1).sqrt();
    let sqrt_u1 = u1.sqrt();
    let x = sqrt_1_u1 * (2.0 * PI * u2).sin();
    let y = sqrt_1_u1 * (2.0 * PI * u2).cos();
    let z = sqrt_u1 * (2.0 * PI * u3).sin();
    let w = sqrt_u1 * (2.0 * PI * u3).cos();
    (x, y, z, w)
}
